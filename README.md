# UGV_EVAL_DEMO

Livox LiDAR 기반 3D 객체 검출·추적과 RTK GNSS ground truth 검수를 위한 ROS 2 워크스페이스입니다.

이 저장소에서 실행할 수 있는 주요 기능은 다음과 같습니다.

- OpenPCDet 기반 3D 객체 검출 및 ByteTrack 추적
- Livox PointCloud2와 RTK GNSS 토픽의 개별 rosbag 기록
- RTK–LiDAR 좌표계 캘리브레이션 및 GT 시각 검수
- RealSense 2대의 수집·재생 시각 동기화 리허설
- OpenPCDet 학습, 평가 및 데모 실행

## 저장소 구성

| 경로 | 역할 |
| --- | --- |
| `src/pcdet_ros2` | OpenPCDet ROS 2 노드와 launch 파일 |
| `src/rtk_livox_dataset_tools` | Livox/RTK 토픽 기록, 좌표변환, 품질 확인 및 시각화 도구 |
| `src/ublox` | u-blox GNSS 드라이버와 ROS 메시지 |
| `src/ros2_numpy` | ROS 메시지와 NumPy 배열 변환 라이브러리 |
| `src/vision_msgs_rviz_plugins` | `vision_msgs/Detection3DArray` RViz 플러그인 |
| `OpenPCDet` | 3D 검출 모델 학습·평가 코드 |
| `cfgs`, `config`, `checkpoints` | PCDet 모델 설정, ROS 파라미터 및 가중치 |
| `docs` | 현장 수집 절차와 파이프라인 설계 문서 |

## 실행 환경과 빌드

`entrypoint.bash`는 GPU, host network와 X11을 연결하여 Docker 이미지 `anticaffe/ugv_project:1.0.0`을 실행합니다. 다음 순서로 환경을 구성합니다.

1. Docker 이미지를 다운로드합니다. 호스트 CUDA 환경은 CUDA 11.8 Toolkit 사용을 권장합니다.

   ```bash
   docker pull anticaffe/ugv_project:1.0.0
   ```

2. GitHub에서 저장소를 clone합니다.

   ```bash
   git clone https://github.com/AntiCaffe/UGV_EVAL_DEMO
   ```

3. `UGV_EVAL_DEMO/entrypoint.bash`를 열고 `PROJECT_DIR`을 현재 `UGV_EVAL_DEMO` 폴더의 절대 경로로 설정합니다.

   ```bash
   # 예시
   PROJECT_DIR="/home/ivl/UGV_EVAL_DEMO"
   ```

4. GUI 프로그램에서 호스트 X11을 사용할 수 있도록 다른 호스트 터미널에서 다음 명령을 실행합니다.

   ```bash
   xhost +local:docker
   ```

   매번 실행하지 않으려면 같은 명령을 호스트의 `~/.bashrc`에 등록한 뒤 새 터미널을 열거나 `source ~/.bashrc`를 실행합니다.

5. 호스트의 `UGV_EVAL_DEMO` 폴더에서 컨테이너를 실행합니다.

   ```bash
   cd /path/to/UGV_EVAL_DEMO
   ./entrypoint.bash
   ```

6. 컨테이너 안의 `/project/OpenPCDet`에서 OpenPCDet을 develop 모드로 설치합니다.

   ```bash
   cd /project/OpenPCDet
   python setup.py develop
   ```

7. 컨테이너 안의 `/project`로 이동하여 `cb` alias로 ROS 2 워크스페이스를 빌드합니다.

   ```bash
   cd /project
   cb
   ```

   `cb` alias가 등록되어 있지 않다면 다음 명령을 사용합니다.

   ```bash
   colcon build
   ```

8. 같은 `/project` 폴더에서 `si` alias로 빌드 결과를 현재 셸에 반영합니다.

   ```bash
   si
   ```

   `si` alias가 등록되어 있지 않다면 다음 명령을 사용합니다.

   ```bash
   source install/setup.bash
   ```

PCDet 실행에는 CUDA를 지원하는 PyTorch와 OpenPCDet 의존성이 필요합니다. 일부 launch는 저장소 밖의 드라이버를 호출하므로 기능별로 다음 패키지가 추가로 필요합니다.

- Livox: 사용하는 장비에 맞는 Livox ROS 2 driver
- RealSense: `realsense2_camera`
- NTRIP 사용 시: `ntrip_client`, `rtcm_msgs`
- PCDet launch: `nav2_common`

빌드 후 등록된 실행 파일은 다음 명령으로 확인할 수 있습니다.

```bash
ros2 pkg executables pcdet_ros2
ros2 pkg executables rtk_livox_dataset_tools
```

## ROS 2 실행

### Launch 파일 목록

| 패키지 | Launch | 기능 |
| --- | --- | --- |
| `pcdet_ros2` | `pcdet.launch.py` | PCDet 검출·추적 노드 실행 |
| `rtk_livox_dataset_tools` | `livox_collection.launch.py` | 지정된 기존 토픽을 rosbag으로 기록 |
| `rtk_livox_dataset_tools` | `rtk_collection.launch.py` | GNSS/NTRIP 실행, RTK 상태 확인 및 rosbag 기록 |
| `rtk_livox_dataset_tools` | `rtk_status_monitor.launch.py` | RTK 품질 및 RTCM 수신 상태 CSV 기록 |
| `rtk_livox_dataset_tools` | `rviz_gt_check.launch.py` | RTK 위치·속도를 LiDAR 좌표계 marker로 변환하고 RViz 실행 |
| `rtk_livox_dataset_tools` | `realsense_collection.launch.py` | RealSense 실행 및 point cloud 기록 |
| `rtk_livox_dataset_tools` | `realsense_sync_check.launch.py` | 두 RealSense point cloud를 나란히 재발행하고 RViz 실행 |

각 launch의 인자는 다음 명령으로 확인할 수 있습니다.

```bash
ros2 launch <PACKAGE> <LAUNCH_FILE> --show-args
```

### PCDet ROS 2 검출·추적

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

모델과 가중치 조합은 `config/*.param.yaml`에서 선택합니다. 경로는 설치된 `pcdet_ros2` 패키지 디렉터리를 기준으로 해석됩니다.

### `rtk_livox_dataset_tools`

이 패키지는 Livox 드라이버, PCDet 또는 추적 노드를 구현하거나 실행하지 않습니다. 외부 노드가 발행하는 센서 토픽을 기록하고, RTK 데이터를 LiDAR 좌표계로 변환·검수하는 도구입니다.

현재 Livox와 RTK는 서로 다른 launch가 별도 bag으로 기록합니다.

```text
외부 Livox driver ── /livox/lidar ── livox_collection.launch.py ── Livox bag

u-blox/C099 + NTRIP ── RTK 토픽 ── rtk_collection.launch.py ── RTK bag + 상태 CSV

Livox/RTK bag + calibration YAML ── 후처리·시각화 도구
```

#### Livox PointCloud2 기록

`livox_collection.launch.py`는 지정한 토픽으로 `ros2 bag record`를 실행하는 기록용 wrapper입니다. Livox driver는 별도로 먼저 실행되어 `/livox/lidar`를 발행하고 있어야 합니다.

```bash
ros2 launch rtk_livox_dataset_tools livox_collection.launch.py \
  bag_uri:=bags/run_01_livox \
  record_topics:="/livox/lidar"
```

위 명령은 사실상 다음 명령과 같습니다.

```bash
ros2 bag record -o bags/run_01_livox /livox/lidar
```

`record_topics`에 다른 토픽을 지정하면 함께 기록할 수 있지만, 이 launch가 해당 토픽의 생산 노드를 실행해 주지는 않습니다.

#### Launch 입출력 상세

| Launch | 읽는 입력 | 실행/처리 | 결과 |
| --- | --- | --- | --- |
| `livox_collection.launch.py` | `record_topics`에 지정된 기존 ROS 토픽 | `ros2 bag record` | `bag_uri`의 rosbag |
| `rtk_collection.launch.py` | u-blox serial 또는 C099 UDP, 외부 NTRIP `/rtcm` | GNSS/NTRIP 실행, RTK 토픽 기록, 상태 모니터 | RTK rosbag, 상태 CSV, 선택적 UDP raw log |
| `rtk_status_monitor.launch.py` | `/ublox_gps_node/navpvt`, `/fix`, `/fix_velocity`, `/rtcm` | RTK fix/RTCM 상태 판정 | `logs/rtk_status_*.csv` |
| `rviz_gt_check.launch.py` | calibration YAML, `/fix`, `/fix_velocity` | RTK 위치·속도를 LiDAR 좌표계로 변환하고 RViz 실행 | `/rtk_gt/livox/point`, `/rtk_gt/livox/velocity`, `/rtk_gt/livox/markers` |
| `realsense_collection.launch.py` | RealSense 장치 또는 기존 point cloud 토픽 | 외부 RealSense driver 실행 및 bag 기록 | RealSense rosbag |
| `realsense_sync_check.launch.py` | RealSense PointCloud2 토픽 2개 | 공통 frame으로 변경하고 두 번째 cloud를 이동하여 RViz 실행 | `/sync_check/rs1/points`, `/sync_check/rs2/points` |

`rtk_collection.launch.py`의 기본 기록 토픽은 다음과 같습니다.

```text
/ublox_gps_node/navpvt
/ublox_gps_node/fix
/ublox_gps_node/fix_velocity
/rtcm
```

C099 UDP bridge를 사용할 경우 `start_ublox:=false`, `start_c099_udp:=true`로 설정해야 동일한 RTK 토픽을 두 노드가 동시에 발행하는 것을 피할 수 있습니다.

#### 후처리 실행 도구

다음 도구는 launch에 자동으로 포함되지 않으며 필요할 때 `ros2 run`으로 직접 실행합니다.

| 실행 파일 | 입력 | 결과 |
| --- | --- | --- |
| `online_lidar_pose_calibrator` | 실시간 `/ublox_gps_node/navpvt`, 안테나 offset YAML | calibration YAML, `/rtk_livox_calibration/phase` |
| `lidar_pose_calibrator` | RTK bag, 전진/후진/정지 시간 구간, 안테나 offset YAML | calibration YAML |
| `gt_transformer` | RTK bag의 NavPVT, calibration YAML | LiDAR 좌표계 RTK GT CSV와 metadata YAML |
| `opencl_dataset_exporter` | Livox bag의 PointCloud2, RTK GT CSV | `points.bin`, `frames.bin`, `rtk_gt.bin`, `metadata.yaml` |
| `opencl_dataset_visualizer` | exporter가 만든 binary dataset | Point cloud와 보간된 RTK GT를 보여주는 GUI |

`config/dataset_collection.yaml`은 설치 대상에는 포함되지만 현재 어떤 launch나 Python 노드에서도 읽지 않습니다. 해당 파일의 토픽, RTK 품질 임계값과 캘리브레이션 항목은 현재 실행 동작에 영향을 주지 않습니다.

#### RTK–Livox GT 시각 검수

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

#### RealSense 2대 동기화 리허설

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

### 직접 실행 가능한 ROS 2 노드/도구

| 실행 파일 | 기능 |
| --- | --- |
| `pcdet_ros2 pcdet` | PCDet 검출 및 ByteTrack 추적 |
| `rtk_livox_dataset_tools c099_udp_bridge` | C099 UDP NMEA를 RTK ROS 토픽으로 변환하고 RTCM 전달 |
| `rtk_livox_dataset_tools rtk_status_monitor` | RTK 품질 및 RTCM 수신 상태 CSV 기록 |
| `rtk_livox_dataset_tools online_lidar_pose_calibrator` | 실시간 LiDAR–RTK pose 캘리브레이션 |
| `rtk_livox_dataset_tools lidar_pose_calibrator` | RTK bag 기반 LiDAR–RTK pose 캘리브레이션 |
| `rtk_livox_dataset_tools gt_transformer` | RTK NavPVT를 LiDAR 좌표계 GT CSV로 변환 |
| `rtk_livox_dataset_tools opencl_dataset_exporter` | Livox bag과 RTK GT CSV를 binary dataset으로 변환 |
| `rtk_livox_dataset_tools opencl_dataset_visualizer` | 변환된 dataset의 point cloud와 RTK GT 시각화 |
| `rtk_livox_dataset_tools rtk_livox_visualizer` | 실시간 RTK 위치·속도를 LiDAR frame 토픽으로 변환 |
| `rtk_livox_dataset_tools realsense_sync_viewer` | 두 point cloud 좌표계/위치 조정 후 재발행 |
| `rtk_livox_dataset_tools realsense_sync_play` | 두 rosbag에 시간 offset을 적용하여 재생 |

각 도구의 상세 옵션은 `--help`로 확인합니다.

```bash
ros2 run rtk_livox_dataset_tools realsense_sync_play --help
```

## OpenPCDet 원본 도구

ROS 2를 거치지 않고 OpenPCDet의 원본 학습·평가·데모도 실행할 수 있습니다. 구체적인 모델 인자는 `OpenPCDet/README.md`를 참고하십시오.

```bash
python3 OpenPCDet/tools/train.py --cfg_file <MODEL_YAML>
python3 OpenPCDet/tools/test.py --cfg_file <MODEL_YAML> --ckpt <CHECKPOINT>
python3 OpenPCDet/tools/demo.py --cfg_file <MODEL_YAML> --ckpt <CHECKPOINT> --data_path <POINT_CLOUD>
```

### `demo.py` 모델별 실행 예시

다음 명령은 `OpenPCDet/tools` 디렉터리를 기준으로 실행합니다.

```bash
cd OpenPCDet/tools
```

CenterPoint:

```bash
python demo.py --cfg_file cfgs/kitti_models/centerpoint_aug.yaml --ckpt checkpoints/checkpoint_epoch_80.pth --data_path ../../kitti_samples/velodyne/000000.bin
```

SECOND:

```bash
python demo.py --cfg_file cfgs/kitti_models/second_KN.yaml --ckpt checkpoints/second_KN.pth --data_path ../../kitti_samples/velodyne/000000.bin
```

### `tracking_demo.py` 검출·추적

`tracking_demo.py`는 파일명 순서대로 point cloud를 검출하고, `pcdet_ros2`의 3D ByteTracker로 추적합니다. 실행 모드에 따라 Open3D 시각화, 프레임별 TXT 저장 또는 두 기능을 함께 사용할 수 있습니다.

#### 옵션과 alias

| 짧은 alias | 긴 옵션 | 기본값 | 설명 |
| --- | --- | --- | --- |
| `-h` | `--help` | - | 도움말 출력 |
| - | `--cfg_file` | `cfgs/kitti_models/second.yaml` | OpenPCDet 모델 설정 YAML |
| - | `--data_path` | 필수 | 단일 `.bin`/`.npy` 또는 연속 프레임 디렉터리 |
| - | `--ckpt` | 필수 | 모델 checkpoint |
| `-m` | `--mode` | `demo` | `demo`, `infer`, `both` 중 실행 모드 선택 |
| `-o` | `--output_dir` | `tracking_results` | `infer`/`both` 모드의 프레임별 TXT 출력 폴더 |
| - | `--output_path` | `tracking_results` | `--output_dir`과 동일한 호환 alias이며 파일이 아닌 폴더를 지정 |
| - | `--ext` | `.bin` | 입력 확장자: `.bin` 또는 `.npy` |
| - | `--track_classes` | `2` | 추적할 1-based class ID 목록. KITTI는 `1=Car`, `2=Pedestrian`, `3=Cyclist` |
| - | `--frame_rate` | `30` | ByteTracker에 전달할 입력 frame rate |
| - | `--velocity_scale` | `1.0` | Open3D 속도 화살표 길이에 적용하는 시간 배율 |
| - | `--no_visualization` | 비활성 | `demo`/`both` 모드에서도 Open3D 창을 열지 않음 |

`--data_path`에 디렉터리를 지정하면 파일명을 정렬한 순서로 처리합니다. 시간축 추적을 확인하려면 단일 파일이 아닌 연속 point cloud 디렉터리를 사용해야 합니다.

#### 시각화 모드

박스는 track ID별 고정 색상으로 표시되며, 박스 위 라벨에는 ID, 위치, 속도 벡터와 속력이 표시됩니다. 속도가 계산된 트랙에는 이동 방향 화살표도 표시됩니다. 시각화 창을 닫으면 다음 프레임으로 넘어갑니다.

```bash
python tracking_demo.py \
  --cfg_file cfgs/kitti_models/centerpoint_aug.yaml \
  --ckpt checkpoints/checkpoint_epoch_80.pth \
  --data_path ../../kitti_samples/velodyne \
  --track_classes 2 \
  --velocity_scale 2.0 \
  -m demo
```

#### 프레임별 TXT 저장 모드

입력 파일마다 `<POINT_CLOUD_STEM>.txt`가 지정한 폴더에 생성됩니다. 예를 들어 `000000.bin`과 `000001.bin`은 각각 `000000.txt`와 `000001.txt`로 저장됩니다.

```bash
python tracking_demo.py \
  --cfg_file cfgs/kitti_models/second_KN.yaml \
  --ckpt checkpoints/second_KN.pth \
  --data_path ../../kitti_samples/velodyne \
  --track_classes 1 2 3 \
  -m infer \
  -o ../../tracking_results
```

TXT의 각 track 행은 탭으로 구분되며 다음 열을 가집니다.

```text
frame frame_file track_id class_id class_name score
x1 y1 z1 x2 y2 z2 yaw vx vy vz speed detection_index
```

#### 시각화와 TXT 저장 동시 실행

```bash
python tracking_demo.py \
  --cfg_file cfgs/kitti_models/second_KN.yaml \
  --ckpt checkpoints/second_KN.pth \
  --data_path ../../kitti_samples/velodyne \
  --track_classes 1 2 3 \
  -m both \
  -o ../../tracking_results
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
