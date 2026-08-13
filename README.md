# UGV_EVAL_DEMO

Livox LiDAR 기반 3D 객체 검출·추적과 결과 검수를 위한 ROS 2 워크스페이스입니다.

이 저장소에서 실행할 수 있는 주요 기능은 다음과 같습니다.

- OpenPCDet 기반 3D 객체 검출 및 ByteTrack 추적
- Livox 및 RealSense 데이터 수집
- OpenPCDet 학습, 평가 및 데모 실행

## 저장소 구성

| 경로 | 역할 |
| --- | --- |
| `src/pcdet_ros2` | OpenPCDet ROS 2 노드와 launch 파일 |
| `src/rtk_livox_dataset_tools` | 센서 데이터 기록 및 결과 시각화 도구 |
| `src/ros2_numpy` | ROS 메시지와 NumPy 배열 변환 라이브러리 |
| `src/vision_msgs_rviz_plugins` | `vision_msgs/Detection3DArray` RViz 플러그인 |
| `OpenPCDet` | 3D 검출 모델 학습·평가 코드 |
| `cfgs`, `config`, `checkpoints` | PCDet 모델 설정, ROS 파라미터 및 가중치 |
| `docs` | 현장 수집 절차와 파이프라인 설계 문서 |

## 실행 환경과 빌드

`entrypoint.bash`는 GPU, host network, X11을 연결하여 기존 Docker 이미지 `ugv_tracker:1.0.0`을 실행합니다. 스크립트의 `PROJECT_DIR`은 현재 저장소의 절대 경로와 일치해야 합니다.

```bash
./entrypoint.bash
```

호스트 또는 컨테이너 안에서 워크스페이스를 빌드합니다.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 -m pip install -e OpenPCDet
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

PCDet 실행에는 CUDA를 지원하는 PyTorch와 OpenPCDet 의존성이 필요합니다. 일부 launch는 저장소 밖의 드라이버를 호출하므로 기능별로 다음 패키지가 추가로 필요합니다.

- Livox: 사용하는 장비에 맞는 Livox ROS 2 driver
- RealSense: `realsense2_camera`
- PCDet launch: `nav2_common`

빌드 후 등록된 실행 파일은 다음 명령으로 확인할 수 있습니다.

```bash
ros2 pkg executables pcdet_ros2
ros2 pkg executables rtk_livox_dataset_tools
```

## 1. PCDet 3D 검출·추적

기본 launch는 `/livox/lidar`의 `sensor_msgs/PointCloud2`를 받아 CenterPoint 설정과 `checkpoint_epoch_100.pth`를 사용합니다. 결과는 기본적으로 `/lr_detections`에 `visualization_msgs/MarkerArray` 형식으로 발행합니다.

```bash
source install/setup.bash
ros2 launch pcdet_ros2 pcdet.launch.py
```

입출력 토픽이나 출력 메시지 형식을 바꿀 수 있습니다.

```bash
ros2 launch pcdet_ros2 pcdet.launch.py \
  input_topic:=/livox/lidar \
  output_topic:=/detections \
  output_format:=detection3d_array
```

`output_format`은 `marker_array` 또는 `detection3d_array`입니다. 다른 모델은 파라미터 파일을 지정하여 실행합니다.

```bash
ros2 launch pcdet_ros2 pcdet.launch.py \
  params_file:="$(ros2 pkg prefix pcdet_ros2)/share/pcdet_ros2/config/pcdet_second.param.yaml"
```

미리 준비된 모델별 launch도 있습니다.

```bash
ros2 launch pcdet_ros2 second_multihead_nds.launch.xml
ros2 launch pcdet_ros2 pp_multihead_nds.launch.xml
ros2 launch pcdet_ros2 parta2_free.launch.xml
```

모델과 가중치 조합은 `config/*.param.yaml`에서 선택합니다. 경로는 설치된 `pcdet_ros2` 패키지 디렉터리를 기준으로 해석됩니다.

## 2. Livox 데이터 수집

Livox driver와 검출·추적 노드를 먼저 실행한 뒤 필요한 토픽을 기록합니다.

```bash
ros2 launch rtk_livox_dataset_tools livox_collection.launch.py \
  bag_uri:=bags/run_01_livox \
  record_topics:="/livox/lidar /lr_detections"
```

포인트클라우드만 기록하려면 `record_topics:="/livox/lidar"`를 사용합니다.

## TEST

```bash
ros2 launch rtk_livox_dataset_tools rviz_gt_check.launch.py \
  calib:=calibration/run_01_lidar_rtk_alignment.yaml \
  time_offset_sec:=0.0 \
  livox_frame:=livox_frame \
  show_speed_text:=true
```

```bash
ros2 bag play bags/run_01_livox_rtk --loop
```

## 3. RealSense 2대 동기화 리허설

각 컴퓨터에서 서로 다른 namespace로 point cloud를 기록합니다.

```bash
# 컴퓨터 1
ros2 launch rtk_livox_dataset_tools realsense_collection.launch.py \
  camera_namespace:=rs1 \
  bag_uri:=bags/run_01_rs1 \
  record_topics:="/rs1/camera/depth/color/points"

# 컴퓨터 2
ros2 launch rtk_livox_dataset_tools realsense_collection.launch.py \
  camera_namespace:=rs2 \
  bag_uri:=bags/run_01_rs2 \
  record_topics:="/rs2/camera/depth/color/points"
```

두 point cloud를 나란히 표시하는 republisher와 RViz를 실행합니다.

```bash
ros2 launch rtk_livox_dataset_tools realsense_sync_check.launch.py \
  cloud1_topic:=/rs1/camera/depth/color/points \
  cloud2_topic:=/rs2/camera/depth/color/points \
  fixed_frame:=sync_check_world \
  cloud2_offset_y:=2.0
```

두 bag을 동일 시간축으로 재생합니다. offset은 `bag1 event time - bag2 event time`입니다.

```bash
ros2 run rtk_livox_dataset_tools realsense_sync_play \
  --bag1 bags/run_01_rs1 \
  --bag2 bags/run_01_rs2 \
  --time-offset-bag1-minus-bag2-sec 0.0
```

## 전체 실행 가능 항목

### ROS 2 launch 파일

| 패키지 | launch | 기능 |
| --- | --- | --- |
| `pcdet_ros2` | `pcdet.launch.py` | PCDet 검출·추적 노드 실행 |
| `pcdet_ros2` | `second_multihead_nds.launch.xml` | SECOND Multihead 설정 실행 |
| `pcdet_ros2` | `pp_multihead_nds.launch.xml` | PointPillar Multihead 설정 실행 |
| `pcdet_ros2` | `parta2_free.launch.xml` | Part-A2 Free 설정 실행 |
| `rtk_livox_dataset_tools` | `livox_collection.launch.py` | Livox/추적 토픽 rosbag 기록 |
| `rtk_livox_dataset_tools` | `realsense_collection.launch.py` | RealSense 실행 및 point cloud 기록 |
| `rtk_livox_dataset_tools` | `realsense_sync_check.launch.py` | 두 RealSense point cloud 비교 |

launch 인자는 다음 명령으로 확인할 수 있습니다.

```bash
ros2 launch <PACKAGE> <LAUNCH_FILE> --show-args
```

### 직접 실행 가능한 ROS 2 노드/도구

| 실행 파일 | 기능 |
| --- | --- |
| `pcdet_ros2 pcdet` | PCDet 검출 및 ByteTrack 추적 |
| `rtk_livox_dataset_tools realsense_sync_viewer` | 두 point cloud 좌표계/위치 조정 후 재발행 |
| `rtk_livox_dataset_tools realsense_sync_play` | 두 rosbag에 시간 offset을 적용하여 재생 |

각 도구의 상세 옵션은 `--help`로 확인합니다.

```bash
ros2 run rtk_livox_dataset_tools realsense_sync_play --help
```

### OpenPCDet 원본 도구

ROS 2를 거치지 않고 OpenPCDet의 원본 학습·평가·데모도 실행할 수 있습니다. 구체적인 모델 인자는 `OpenPCDet/README.md`를 참고하십시오.

```bash
python3 OpenPCDet/tools/train.py --cfg_file <MODEL_YAML>
python3 OpenPCDet/tools/test.py --cfg_file <MODEL_YAML> --ckpt <CHECKPOINT>
python3 OpenPCDet/tools/demo.py --cfg_file <MODEL_YAML> --ckpt <CHECKPOINT> --data_path <POINT_CLOUD>
```

## 테스트

```bash
colcon test --packages-select rtk_livox_dataset_tools pcdet_ros2
colcon test-result --verbose
```

## 관련 문서

- [PCDet ROS 2 패키지 설명](src/pcdet_ros2/README.md)
- [OpenPCDet 설명](OpenPCDet/README.md)

## 주의사항

- `bags/`, 모델 가중치(`*.pth`)와 colcon 산출물은 Git에서 제외됩니다.
- PCDet 모델 YAML과 checkpoint가 서로 호환되어야 합니다.
- 현장 운용 전 `/livox/lidar`의 실제 발행 여부와 주기를 확인하십시오.
