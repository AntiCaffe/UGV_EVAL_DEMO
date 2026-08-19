"""! @brief Defines the PCDetROS Class.
The package subscribes to the pointcloud message and publishes instances of object detection.
"""

# Imports
import rclpy 
from rclpy.node import Node
import ros2_numpy
from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import Detection3D
from vision_msgs.msg import ObjectHypothesisWithPose
from sensor_msgs.msg import PointCloud2

import argparse
import numpy as np
from typing import List
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
        @param threshold_array Cutoff threshold array for detections. Even values for detection id, odd values for detection score. Sample: [0, 0.7, 1, 0.5, 2, 0.7].
        """
        # ROS2 노드 이름을 'pcdet'으로 설정하고 피라미터와 필요한 객체들을 초기화
        super().__init__('pcdet')
        self.__initParams__()
        self.__initObjects__()

        # BYTETracker 인스턴스 생성하여 객체 추적 수행
        self.bytetracker = BYTETracker(frame_rate=30)

    def tracker_coord(self, box_coord):
        
        # 3D 바운딩 박스 좌표 변환
        bc = list(map(float, box_coord))
        x1 = bc[0] - bc[3]/2  # x - width/2
        x2 = bc[0] + bc[3]/2  # x + width/2
        y1 = bc[1] - bc[4]/2  # y - height/2
        y2 = bc[1] + bc[4]/2  # y + height/2
        z1 = bc[2] - bc[5]/2  # z - depth/2
        z2 = bc[2] + bc[5]/2  # z + depth/2
        return [x1, y1, z1, x2, y2, z2]
    
    # Callback function
    def __cloudCB__(self, cloud_msg):
        
        out_msg = Detection3DArray()
        out_msg.header.frame_id = cloud_msg.header.frame_id
        out_msg.header.stamp = self.get_clock().now().to_msg()

        # 1) 포인트 클라우드 -> 모델 추론
        cloud_array = ros2_numpy.point_cloud2.pointcloud2_to_array(cloud_msg)
        np_points = self.__convertCloudFormat__(cloud_array)
        scores, dt_box_lidar, types = self.__runTorch__(np_points)
        # dt_box_lidar.shape = (N, 7) -> 보통 [x, y, z, dx, dy, dz, heading] 순서


        # 2) 만약 검출 결과가 없으면 바로 퍼블리시 후 return
        if scores.size == 0:
            out_msg.detections = []
            self.__pub_det__.publish(out_msg)
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
        outputs = self.bytetracker.update(boxes_for_tracking,
                                              np.array(scores),
                                              np.array(types))
            
        # 5) outputs에는 [x1,y1,z1,x2,y2,z2, track_id, score, cls, idx, vx, vy, vz, yaw] 형태
        #    해당 내용을 vision_msgs Detection3D 형식으로 변환
        #    이하 기존 코드와 동일
        class_map = {
            1.0: "Car",
            2.0: "Pedestrian",
            3.0: "Cyclist"
        }
            
        final_msg = Detection3DArray()
        final_msg.header.frame_id = cloud_msg.header.frame_id
        final_msg.header.stamp = self.get_clock().now().to_msg()

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
            detection_idx               = int(track_res[9])  # 필요하면 사용
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

            # 4) 메시지 생성
            det = Detection3D()
            det.header = final_msg.header

            # bbox 중심좌표 및 크기 설정
            det.bbox.center.position.x = float(center_x)
            det.bbox.center.position.y = float(center_y)
            det.bbox.center.position.z = float(center_z)
            det.bbox.size.x = float(size_x)
            det.bbox.size.y = float(size_y)
            det.bbox.size.z = float(size_z)

            # 회전값(orientation) 설정
            det.bbox.center.orientation.x = quat[1]
            det.bbox.center.orientation.y = quat[2]
            det.bbox.center.orientation.z = quat[3]
            det.bbox.center.orientation.w = quat[0]

            # 추적 ID 및 score 설정
            det.tracking_id = str(track_id)
            det.is_tracking = True  # 추적된 결과이므로 True
                
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = str(object_class_str)  # cls
            hypothesis.score = track_score

            # (필터링된 속도를 쓰고 싶다면 [13], [14], [15] 참조)
            hypothesis.pose.pose.position.x = float(vx)
            hypothesis.pose.pose.position.y = float(vy)
            hypothesis.pose.pose.position.z = float(vz)
                
            det.results.append(hypothesis)
            final_msg.detections.append(det)
            
        self.__pub_det__.publish(final_msg)
                       
        
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
            return 0, 0, 0
        
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
    
    def __getPubState__(self, id, score) -> bool:
        if(not self.__allow_score_thresholding__):
           return True
        
        for i in range(len(self.__thr_arr__)):
            if(i + 1 == id):
                if(self.__thr_arr__[i] > score):
                    return False
                else:
                    return True
        
        return True

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

        self.__config_file__ = self.get_parameter("config_file").value
        self.__package_folder_path__ = self.get_parameter("package_folder_path").value
        self.__model_file__ = self.get_parameter("model_file").value
        self.__allow_memory_fractioning__ = self.get_parameter("allow_memory_fractioning").value
        self.__allow_score_thresholding__ = self.get_parameter("allow_score_thresholding").value
        self.__num_features__ = self.get_parameter("num_features").value
        self.__device_id__ = self.get_parameter("device_id").value
        self.__device_memory_fraction__ = self.get_parameter("device_memory_fraction").value
        self.__thr_arr__ = self.get_parameter("threshold_array").value

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
        self.__pub_det__ = self.create_publisher(Detection3DArray,
                                                 "output",
                                                 10)
    

def main(args=None):
    rclpy.init(args=args)
    pcdet_node = PCDetROS()
    rclpy.spin(pcdet_node)
    pcdet_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()