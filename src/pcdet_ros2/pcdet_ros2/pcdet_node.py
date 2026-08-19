"""PCDet ROS node with selectable MarkerArray or Detection3DArray output."""

# Imports
import rclpy 
from rclpy.node import Node
import ros2_numpy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

import numpy as np
import torch

from pyquaternion import Quaternion

from .config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
from .nn_3d import BYTETracker



class PCDetROS(Node):
    """! The PCDetROS class.
    Defines the ROS 2 Wrapper class for PCDet.
    """
    def __init__(self):
        """! The PCDetROS class initializer.
        @param config_file Path to the configuration file for OpenPCDet.
        @param package_folder_path Path to the configuration folder, generally inside the ROS 2 Package.
        @param model_file Path to model used for Detection.
        @param allow_memory_fractioning Boolean to activate fraction CUDA Memory.
        @param allow_score_thresholding Boolean to activate score thresholding.
        @param num_features Number of features in each pointcloud data. 4 for Kitti. 5 for NuScenes
        @param device_id CUDA Device ID.
        @param device_memory_fraction Use only the input fraction of the allowed CUDA Memory.
        @param threshold_array Per-class high/new-track score thresholds in model label order.
        """
        # ROS2 노드 이름을 'pcdet'으로 설정하고 피라미터와 필요한 객체들을 초기화
        super().__init__('pcdet')
        self.__initParams__()
        self.__initObjects__()

        class_thresholds = (
            self.__thr_arr__ if self.__allow_score_thresholding__ else []
        )
        self.bytetracker = BYTETracker(
            frame_rate=self.__tracker_frame_rate__,
            high_score_threshold=(
                0.5 if self.__allow_score_thresholding__
                else self.__tracker_low_score_threshold__
            ),
            class_score_thresholds=class_thresholds,
            low_score_threshold=self.__tracker_low_score_threshold__,
            match_threshold=self.__tracker_match_threshold__,
            second_match_threshold=self.__tracker_second_match_threshold__,
            unconfirmed_match_threshold=(
                self.__tracker_unconfirmed_match_threshold__
            ),
            max_time_lost=self.__tracker_max_lost_sec__,
            mahalanobis_gate=self.__tracker_mahalanobis_gate__,
            lost_velocity_decay=self.__tracker_lost_velocity_decay__,
        )
        self.get_logger().info(
            f'Tracker motion model: {self.bytetracker.motion_model}'
        )
    
    # Callback function
    def __cloudCB__(self, cloud_msg):
        # 1) 포인트 클라우드 -> 모델 추론
        cloud_array = ros2_numpy.point_cloud2.pointcloud2_to_array(cloud_msg)
        np_points = self.__convertCloudFormat__(cloud_array)
        scores, dt_box_lidar, types = self.__runTorch__(np_points)
        # dt_box_lidar.shape = (N, 7) -> 보통 [x, y, z, dx, dy, dz, heading] 순서

        timestamp = (
            float(cloud_msg.header.stamp.sec)
            + float(cloud_msg.header.stamp.nanosec) * 1e-9
        )

        # 2) 검출이 없는 프레임도 tracker에 전달해야 Lost timeout과
        #    Kalman prediction이 실제 센서 시간에 맞게 진행된다.
        if scores.size == 0:
            outputs = self.bytetracker.update(
                np.empty((0, 8), dtype=np.float32),
                np.empty(0, dtype=np.float32),
                np.empty(0, dtype=np.float32),
                timestamp=timestamp,
            )
            self.__publishResults__(cloud_msg.header, outputs)
            return

        # 3) ByteTracker에 전달할 boxes 생성
        #    [x, y, z, w, h, l, yaw, idx]
        #    주의: dt_box_lidar[i][3]~[5]가 실제로 w,h,l에 해당하는지 확인 필요
        boxes_for_tracking = []

        for i in range(scores.size):
            x = dt_box_lidar[i][0]
            y = dt_box_lidar[i][1]
            z = dt_box_lidar[i][2]
            
            # dx, dy, dz를 w, h, l로 매핑 (데이터셋별로 다름)
            w = dt_box_lidar[i][3]  # 예: width
            h = dt_box_lidar[i][4]  # 예: height
            l = dt_box_lidar[i][5]  # 예: length
            
            yaw = dt_box_lidar[i][6]  # heading (라디안)

            # idx는 단순히 i로 매긴다고 가정
            boxes_for_tracking.append([x, y, z, w, h, l, yaw, i])
            
        # numpy 배열로 변환
        boxes_for_tracking = np.array(boxes_for_tracking, dtype=np.float32)

        # 4) ByteTracker 업데이트 (이때 [x,y,z,w,h,l,yaw,idx]를 전달)
        outputs = self.bytetracker.update(
            boxes_for_tracking,
            np.asarray(scores),
            np.asarray(types),
            timestamp=timestamp,
        )
            
        # 5) outputs에는 [x1,y1,z1,x2,y2,z2, track_id, score, cls,
        #    idx, vx, vy, vz, yaw] 형태
        self.__publishResults__(cloud_msg.header, outputs)

    def __publishResults__(self, header, outputs):
        """Publish tracking results in the selected output message format."""
        if self.__output_format__ == 'marker_array':
            self.__publishMarkers__(header, outputs)
        else:
            self.__publishDetections__(header, outputs)

    def __publishDetections__(self, header, outputs):
        """Publish tracking results as a vision_msgs Detection3DArray."""
        class_map = {
            1.0: 'Car',
            2.0: 'Pedestrian',
            3.0: 'Cyclist'
        }
        message = Detection3DArray()
        message.header = header

        for track_res in outputs:
            x1, y1, z1, x2, y2, z2 = track_res[0:6]
            track_id = int(track_res[6])
            track_score = float(track_res[7])
            object_class = float(track_res[8])
            vx, vy, vz = track_res[10:13]
            yaw = float(track_res[13])

            detection = Detection3D()
            detection.header = header
            detection.tracking_id = str(track_id)
            detection.is_tracking = True
            detection.bbox.center.position.x = float((x1 + x2) / 2.0)
            detection.bbox.center.position.y = float((y1 + y2) / 2.0)
            detection.bbox.center.position.z = float((z1 + z2) / 2.0)
            detection.bbox.size.x = float(x2 - x1)
            detection.bbox.size.y = float(y2 - y1)
            detection.bbox.size.z = float(z2 - z1)

            quat = self.__yawToQuaternion__(yaw)
            detection.bbox.center.orientation.x = float(quat[1])
            detection.bbox.center.orientation.y = float(quat[2])
            detection.bbox.center.orientation.z = float(quat[3])
            detection.bbox.center.orientation.w = float(quat[0])

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = class_map.get(object_class, 'Unknown')
            hypothesis.score = track_score
            hypothesis.pose.pose.position.x = float(vx)
            hypothesis.pose.pose.position.y = float(vy)
            hypothesis.pose.pose.position.z = float(vz)
            detection.results.append(hypothesis)
            message.detections.append(detection)

        self.__pub_det__.publish(message)

    def __publishMarkers__(self, header, outputs):
        """Publish boxes, labels, and tracking velocity in one MarkerArray."""
        class_map = {
            1.0: "Car",
            2.0: "Pedestrian",
            3.0: "Cyclist"
        }

        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header = header
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for track_res in outputs:
            """
                track_res에 들어있는 정보 예시:
                track_res[0:6] -> [x1, y1, z1, x2, y2, z2] (추적된 3D 바운딩 박스 코너)
                track_res[6]   -> track_id
                track_res[7]   -> 추적된 score
                track_res[8]   -> cls
                track_res[9]   -> idx
                track_res[10:13] -> (EKF vx, vy, vz)
                track_res[13]    -> yaw  
            """
                
            x1, y1, z1, x2, y2, z2      = track_res[0:6]
            track_id                    = int(track_res[6])
            track_score                 = float(track_res[7])
            object_class                = track_res[8]
            vx, vy, vz                  = track_res[10:13]
            yaw                         = track_res[13]
                
            object_class_str = class_map.get(object_class, "Unknown")

            # 2) x1,y1,z1,x2,y2,z2 -> center/size로 변환
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            center_z = (z1 + z2) / 2.0
            size_x = (x2 - x1)
            size_y = (y2 - y1)
            size_z = (z2 - z1)

            # 3) Orientation(회전값) -> 쿼터니언 값으로 변환
            quat = self.__yawToQuaternion__(yaw)

            box_marker = Marker()
            box_marker.header = header
            box_marker.ns = 'pcdet_boxes'
            box_marker.id = track_id
            box_marker.type = Marker.CUBE
            box_marker.action = Marker.ADD
            box_marker.pose.position.x = float(center_x)
            box_marker.pose.position.y = float(center_y)
            box_marker.pose.position.z = float(center_z)
            box_marker.pose.orientation.x = float(quat[1])
            box_marker.pose.orientation.y = float(quat[2])
            box_marker.pose.orientation.z = float(quat[3])
            box_marker.pose.orientation.w = float(quat[0])
            box_marker.scale.x = max(1e-3, float(size_x))
            box_marker.scale.y = max(1e-3, float(size_y))
            box_marker.scale.z = max(1e-3, float(size_z))
            self.__setMarkerColor__(box_marker, object_class, 0.35)
            marker_array.markers.append(box_marker)

            speed = float(np.linalg.norm([vx, vy, vz]))
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = 'pcdet_labels'
            text_marker.id = track_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(center_x)
            text_marker.pose.position.y = float(center_y)
            text_marker.pose.position.z = float(center_z + size_z * 0.6)
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = max(0.2, float(size_z) * 0.25)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = (
                f'{object_class_str} ID:{track_id} '
                f'{track_score:.2f} v:{speed:.2f}m/s '
                f'({float(vx):.2f}, {float(vy):.2f}, {float(vz):.2f})'
            )
            marker_array.markers.append(text_marker)

            velocity_marker = Marker()
            velocity_marker.header = header
            velocity_marker.ns = 'pcdet_velocity'
            velocity_marker.id = track_id
            velocity_marker.type = Marker.ARROW
            velocity_marker.action = Marker.ADD
            velocity_marker.pose.orientation.w = 1.0
            velocity_marker.points = [
                Point(
                    x=float(center_x),
                    y=float(center_y),
                    z=float(center_z)
                ),
                Point(
                    x=float(center_x + vx),
                    y=float(center_y + vy),
                    z=float(center_z + vz)
                )
            ]
            velocity_marker.scale.x = 0.08
            velocity_marker.scale.y = 0.16
            velocity_marker.scale.z = 0.22
            self.__setMarkerColor__(velocity_marker, object_class, 1.0)
            marker_array.markers.append(velocity_marker)

        self.__pub_det__.publish(marker_array)

    @staticmethod
    def __setMarkerColor__(marker, object_class, alpha):
        """Assign a stable color to each detector class."""
        colors = {
            1.0: (0.1, 0.4, 1.0),
            2.0: (1.0, 0.2, 0.2),
            3.0: (0.2, 1.0, 0.3),
        }
        red, green, blue = colors.get(float(object_class), (1.0, 1.0, 0.0))
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha
                       
        
    def __convertCloudFormat__(self, cloud_array, remove_nans=True, dtype=np.float64):
        '''
        '''
        if remove_nans:
            mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
            cloud_array = cloud_array[mask]
        
        points = np.zeros(cloud_array.shape + (self.__num_features__,), dtype=dtype)
        points[...,0] = cloud_array['x']
        points[...,1] = cloud_array['y']
        points[...,2] = cloud_array['z']
        points[...,3] = cloud_array['intensity']
        return points

    def __runTorch__(self, points):
        if len(points) == 0:
            empty = np.empty(0, dtype=np.float32)
            return empty, np.empty((0, 7), dtype=np.float32), empty
        
        self.__points__ = points.reshape([-1, self.__num_features__])

        input_dict = {
            'points': self.__points__
        }
        with torch.no_grad():
            data_dict = self.__online_detection__.prepare_data(data_dict=input_dict)
            data_dict = self.__online_detection__.collate_batch([data_dict])
            load_data_to_gpu(data_dict)

            torch.cuda.synchronize()
            pred_dicts, _ = self.__net__.forward(data_dict)
            
            torch.cuda.synchronize()

            boxes_lidar = pred_dicts[0]["pred_boxes"].detach().cpu().numpy()
            scores = pred_dicts[0]["pred_scores"].detach().cpu().numpy()
            types = pred_dicts[0]["pred_labels"].detach().cpu().numpy()

        return scores, boxes_lidar, types
    
    def __yawToQuaternion__(self, yaw: float) -> Quaternion:
        return Quaternion(axis=[0, 0, 1], radians=yaw)
    
    def __readConfig__(self):
        cfg_from_yaml_file(self.__config_file__, cfg, self.__package_folder_path__)
        cfg.DATA_CONFIG._BASE_CONFIG_ = self.__package_folder_path__ + cfg.DATA_CONFIG._BASE_CONFIG_
        self.__online_detection__ = DatasetTemplate(dataset_cfg=cfg.DATA_CONFIG, 
                                                    class_names=cfg.CLASS_NAMES, 
                                                    training=False, 
                                                    root_path=self.__package_folder_path__,
                                                    logger=self.__logger__)
        
        torch.cuda.set_device(self.__device_id__)
        torch.backends.cudnn.benchmark = False
        self.__device__ = torch.device('cuda:'+ str(self.__device_id__) if torch.cuda.is_available() else "cpu")
        if(self.__allow_memory_fractioning__):
            torch.cuda.set_per_process_memory_fraction(self.__device_memory_fraction__, device=self.__device_id__)
        
        self.__net__ = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=self.__online_detection__)
        self.__net__.load_params_from_file(filename=self.__model_file__, logger=self.__logger__, to_cpu=True)
        self.__net__ = self.__net__.to(self.__device__).eval()
      
    def __initParams__(self):
        self.declare_parameter("config_file", rclpy.Parameter.Type.STRING)
        self.declare_parameter("package_folder_path", rclpy.Parameter.Type.STRING)
        self.declare_parameter("model_file", rclpy.Parameter.Type.STRING)
        self.declare_parameter("allow_memory_fractioning", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("allow_score_thresholding", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("num_features", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("device_id", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("device_memory_fraction", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("threshold_array", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("output_format", "marker_array")
        self.declare_parameter("tracker_frame_rate", 10.0)
        self.declare_parameter("tracker_low_score_threshold", 0.1)
        self.declare_parameter("tracker_match_threshold", 0.95)
        self.declare_parameter("tracker_second_match_threshold", 0.98)
        self.declare_parameter("tracker_unconfirmed_match_threshold", 0.90)
        self.declare_parameter("tracker_max_lost_sec", 1.0)
        self.declare_parameter("tracker_mahalanobis_gate", 16.27)
        self.declare_parameter("tracker_lost_velocity_decay", 0.95)

        self.__config_file__ = self.get_parameter("config_file").value
        self.__package_folder_path__ = self.get_parameter("package_folder_path").value
        self.__model_file__ = self.get_parameter("model_file").value
        self.__allow_memory_fractioning__ = self.get_parameter("allow_memory_fractioning").value
        self.__allow_score_thresholding__ = self.get_parameter("allow_score_thresholding").value
        self.__num_features__ = self.get_parameter("num_features").value
        self.__device_id__ = self.get_parameter("device_id").value
        self.__device_memory_fraction__ = self.get_parameter("device_memory_fraction").value
        self.__thr_arr__ = self.get_parameter("threshold_array").value
        self.__output_format__ = self.get_parameter("output_format").value
        self.__tracker_frame_rate__ = self.get_parameter("tracker_frame_rate").value
        self.__tracker_low_score_threshold__ = self.get_parameter(
            "tracker_low_score_threshold"
        ).value
        self.__tracker_match_threshold__ = self.get_parameter(
            "tracker_match_threshold"
        ).value
        self.__tracker_second_match_threshold__ = self.get_parameter(
            "tracker_second_match_threshold"
        ).value
        self.__tracker_unconfirmed_match_threshold__ = self.get_parameter(
            "tracker_unconfirmed_match_threshold"
        ).value
        self.__tracker_max_lost_sec__ = self.get_parameter(
            "tracker_max_lost_sec"
        ).value
        self.__tracker_mahalanobis_gate__ = self.get_parameter(
            "tracker_mahalanobis_gate"
        ).value
        self.__tracker_lost_velocity_decay__ = self.get_parameter(
            "tracker_lost_velocity_decay"
        ).value

        valid_output_formats = ('marker_array', 'detection3d_array')
        if self.__output_format__ not in valid_output_formats:
            raise ValueError(
                'output_format must be one of: '
                + ', '.join(valid_output_formats)
            )

        self.__config_file__ = self.__package_folder_path__ + "/" + self.__config_file__
        self.__model_file__ = self.__package_folder_path__ + "/" + self.__model_file__
        self.__points__ = None
        self.__logger__ = common_utils.create_logger()
        self.__readConfig__()
    
    def __initObjects__(self):
        self.sub_cloud = self.create_subscription(PointCloud2, 
                                                  "input", 
                                                  self.__cloudCB__, 
                                                  10)
        output_type = (
            MarkerArray
            if self.__output_format__ == 'marker_array'
            else Detection3DArray
        )
        self.__pub_det__ = self.create_publisher(output_type, "output", 10)
        self.get_logger().info(
            f'Output format: {self.__output_format__} on topic output'
        )
    

def main(args=None):
    rclpy.init(args=args)
    pcdet_node = PCDetROS()
    rclpy.spin(pcdet_node)
    pcdet_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
