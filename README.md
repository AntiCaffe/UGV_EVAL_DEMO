# UGV_EVAL_DEMO

Livox LiDAR 기반 3D 객체 검출·추적과 RTK GNSS ground truth 검수를 위한 ROS 2 워크스페이스입니다.

이 저장소에서 실행할 수 있는 주요 기능은 다음과 같습니다.

- OpenPCDet 기반 3D 객체 검출 및 ByteTrack 추적
- KITTI `.bin` point cloud 시퀀스의 ROS 2 `PointCloud2` 재생
- 기존 Livox/RTK rosbag의 누적 데이터셋 변환
- 기존 RTK–LiDAR calibration 기반 GT 변환 및 시각 검수
- OpenPCDet 학습, 평가 및 데모 실행

## 저장소 구성

| 경로 | 역할 |
| --- | --- |
| `src/pcdet_ros2` | OpenPCDet ROS 2 노드와 launch 파일 |
| `src/rtk_livox_dataset_tools` | Livox/RTK bag 후처리, 좌표변환 및 시각화 도구 |
| `src/ublox` | u-blox GNSS 드라이버와 ROS 메시지 |
| `src/ros2_numpy` | ROS 메시지와 NumPy 배열 변환 라이브러리 |
| `src/vision_msgs_rviz_plugins` | `vision_msgs/Detection3DArray` RViz 플러그인 |
| `OpenPCDet` | 3D 검출 모델 학습·평가 코드 |
| `cfgs`, `config`, `checkpoints` | PCDet 모델 설정, ROS 파라미터 및 가중치 |
| `docs` | 데이터셋 및 실행 관련 보조 문서 |

## 실행 환경과 빌드

`entrypoint.bash`는 GPU, host network와 X11을 연결하여 Docker 이미지 `anticaffe/ugv_project:1.0.0`을 `ugv_project`라는 고정된 컨테이너 이름으로 실행합니다. 컨테이너가 시작되면 OpenPCDet을 develop 모드로 설치하고, `/project`에서 `colcon build --symlink-install`을 실행한 다음 `install/setup.bash`를 자동으로 source하여 shell을 엽니다. Python 패키지는 source와 install이 서로 다른 복사본으로 남지 않도록 symlink 방식으로 설치됩니다. 다음 순서로 환경을 구성합니다.

1. Docker 이미지를 다운로드합니다. **호스트 CUDA 환경은 CUDA 11.8 Toolkit 사용을 권장합니다.**

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

6. `entrypoint.bash`를 실행할 때마다 다음 초기화 작업이 자동으로 수행됩니다.

   ```text
   cd /project/OpenPCDet && python setup.py develop
   cd /project && colcon build --symlink-install
   source /project/install/setup.bash
   ```

   초기화가 끝나면 컨테이너 shell이 열립니다. 컨테이너 이름이 고정되어 있으므로 이미 실행 중인 `ugv_project` 컨테이너가 있으면 새 컨테이너는 시작되지 않습니다.

### 실행 중인 컨테이너에 다시 접속

`ugv_project` 컨테이너가 이미 실행 중이라면 새 터미널에서 접속한 뒤 ROS 2
환경을 source합니다.

```bash
docker exec -it ugv_project bash -lc \
  'source /opt/ros/foxy/setup.bash && source /project/install/setup.bash && exec bash'
```

PCDet 실행에는 CUDA를 지원하는 PyTorch와 OpenPCDet 의존성이 필요합니다. 일부 launch는 저장소 밖의 드라이버를 호출하므로 기능별로 다음 패키지가 추가로 필요합니다.

- Livox: 사용하는 장비에 맞는 Livox ROS 2 driver
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
| `pcdet_ros2` | `kitti_bin_publisher.launch.py` | KITTI `.bin` 시퀀스를 `PointCloud2`로 재생 |
| `rtk_livox_dataset_tools` | `accumulated_bag_exporter.launch.py` | 결합 rosbag을 누적 OpenPCDet 데이터셋으로 추출 |
| `rtk_livox_dataset_tools` | `rviz_gt_check.launch.py` | RTK 위치·속도를 LiDAR 좌표계 marker로 변환하고 RViz 실행 |

각 launch의 인자는 다음 명령으로 확인할 수 있습니다.

```bash
ros2 launch <PACKAGE> <LAUNCH_FILE> --show-args
```

### PCDet ROS 2 검출·추적

기본 launch는 `/livox/lidar`의 `sensor_msgs/PointCloud2`를 받아 CenterPoint
설정과 `checkpoint_epoch_80.pth`를 사용합니다. 결과는 `/lr_detections`에
`visualization_msgs/MarkerArray` 형식으로 발행합니다.

```bash
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

#### KITTI `.bin` publisher

`kitti_bin_publisher.launch.py`는 KITTI Velodyne 형식의 `.bin` 파일을 파일명 순서대로 읽어 `sensor_msgs/PointCloud2`로 발행합니다. 각 point는 `float32` 형식의 `x`, `y`, `z`, `intensity` 네 필드로 구성되어야 합니다.

저장소의 기본 샘플(`kitti_samples/velodyne`)을 10 Hz로 반복 재생하려면 다음 명령을 실행합니다.

```bash
ros2 launch pcdet_ros2 kitti_bin_publisher.launch.py
```

기본 출력 토픽은 PCDet 입력과 같은 `/livox/lidar`이므로, 두 터미널에서 publisher와 PCDet launch를 각각 실행하면 KITTI 시퀀스로 검출·추적을 확인할 수 있습니다.

```bash
# 터미널 1: KITTI point cloud 재생
ros2 launch pcdet_ros2 kitti_bin_publisher.launch.py

# 터미널 2: PCDet 검출·추적
ros2 launch pcdet_ros2 pcdet.launch.py
```

다른 KITTI 데이터셋을 사용할 때는 KITTI root 또는 `.bin` 파일이 들어 있는 `velodyne` 디렉터리를 `dataset_path`에 지정합니다.

```bash
ros2 launch pcdet_ros2 kitti_bin_publisher.launch.py \
  dataset_path:=/path/to/kitti/training/velodyne \
  input_topic:=/livox/lidar \
  frame_id:=velodyne \
  publish_rate_hz:=10.0 \
  loop:=true
```

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `dataset_path` | 빈 문자열 | KITTI root 또는 `velodyne` 디렉터리. 비어 있으면 `pcdet_ros2/kitti_samples` symlink를 탐색 |
| `input_topic` | `/livox/lidar` | `PointCloud2` 출력 토픽 |
| `frame_id` | `velodyne` | 출력 message의 frame ID |
| `publish_rate_hz` | `10.0` | point cloud 발행 주기(Hz, 0보다 커야 함) |
| `loop` | `true` | 마지막 파일 발행 후 첫 파일부터 다시 재생할지 여부 |

한 번만 재생하려면 `loop:=false`를 사용합니다. 인자를 생략했는데 기본 샘플을 찾지 못하면 `dataset_path`를 절대 경로로 지정하십시오.

### Livox/RTK 데이터셋 후처리

이 패키지는 이미 기록된 Livox/RTK rosbag을 OpenPCDet 데이터셋으로
변환하고, RTK GT를 LiDAR 좌표계에서 시각적으로 검수합니다.

#### Sparse Livox 누적 데이터셋 추출

필요한 입력은 다음 두 가지입니다.

- `/livox/lidar`, RTK fix와 velocity가 함께 기록된 rosbag
- `origin_llh`, `yaw_enu_lidar_rad`, `lidar_position_enu`가 포함된 calibration YAML

출력 디렉터리가 이미 존재하면 덮어쓰지 않고 종료합니다. 실행할 때마다 새로운
`output_dir`을 지정하십시오.

```bash
ros2 launch rtk_livox_dataset_tools accumulated_bag_exporter.launch.py \
  bag:=/project/bags/run_01_livox_rtk \
  output_dir:=/project/datasets/run_02_accumulated \
  calib:=/project/calibration/run_01_lidar_rtk_alignment.yaml \
  accumulation_sec:=0.2 \
  output_rate_hz:=10.0 \
  max_rtk_age_sec:=0.5
```

##### 옵션과 alias

| Launch alias | 대응 CLI 옵션 | Launch 기본값 | 설명 |
| --- | --- | --- | --- |
| `bag` | `--bag` | `/project/bags/run_01_livox_rtk` | Livox와 RTK가 함께 기록된 rosbag |
| `output_dir` | `--output-dir` | `/project/datasets/run_01_accumulated` | 생성 데이터셋 경로. 기존 경로는 덮어쓰지 않음 |
| `calib` | `--calib` | `/project/calibration/run_01_lidar_rtk_alignment.yaml` | RTK를 LiDAR 좌표계로 변환할 YAML. 추적 평가에서는 필수 |
| `cloud_topic` | `--cloud-topic` | `/livox/lidar` | Livox `PointCloud2` 토픽 |
| `fix_topic` | `--fix-topic` | `/ublox_gps_node/fix` | RTK `NavSatFix` 토픽 |
| `velocity_topic` | `--velocity-topic` | `/ublox_gps_node/fix_velocity` | RTK velocity 토픽 |
| `storage_id` | `--storage-id` | `sqlite3` | rosbag 저장소 backend |
| `time_source` | `--time-source` | `aligned_header` | Livox header를 bag epoch에 정렬. `bag`은 record timestamp 사용 |
| `output_rate_hz` | `--output-rate-hz` | `10.0` | 출력 프레임 주기(Hz) |
| `accumulation_sec` | `--accumulation-sec` | `0.2` | 각 프레임에 합칠 과거 packet 시간창(초) |
| `max_rtk_age_sec` | `--max-rtk-age-sec` | `0.5` | 최신 RTK로 인정할 최대 age(초) |
| `drop_stale_rtk` | `--drop-stale-rtk` | `false` | `true`이면 최대 age를 넘은 RTK 프레임 제외 |
| `voxel_size` | `--voxel-size` | `0.0` | voxel 크기(m). `0.0`이면 downsampling 비활성화 |
| `max_points_per_frame` | `--max-points-per-frame` | `0` | 프레임당 point 제한. `0`이면 제한 없음 |

Exporter는 `(t - accumulation_sec, t]` 범위의 과거 Livox packet만 누적하고,
프레임 시각보다 미래가 아닌 최신 RTK를 결합합니다. `aligned_header`는 Livox의
상대 header clock을 rosbag epoch에 맞추며, 보간이나 미래 RTK 참조는 하지
않습니다. LiDAR가 움직이는 데이터에는 별도의 ego-motion compensation이
필요합니다.

출력 구조는 다음과 같습니다.

```text
datasets/run_01_accumulated/
├── velodyne/000000.bin  # float32 [x, y, z, intensity]
├── frames.csv           # 누적 구간과 point 수
├── rtk_latest.csv       # 최신 RTK, sample age, LiDAR 좌표계 GT
├── ImageSets/test.txt   # 프레임 순서
└── metadata.yaml        # 추출 설정과 통계
```

Calibration을 생략하면 `rtk_latest.csv`의 LiDAR 좌표계 열이 `nan`이므로 추적
평가용 GT로 사용할 수 없습니다. `fix_age_sec`, `velocity_age_sec`,
`rtk_is_fresh`로 오래된 RTK를 필터링할 수 있습니다.

생성된 point cloud는 OpenPCDet 추적 입력으로 사용합니다.

```bash
cd /project/OpenPCDet/tools
python tracking_demo.py \
  --cfg_file cfgs/kitti_models/centerpoint_aug.yaml \
  --ckpt checkpoints/checkpoint_epoch_80.pth \
  --data_path ../../datasets/run_01_accumulated/velodyne \
  --frame_rate 10 \
  --mode infer
```

#### RTK–Livox GT 시각 검수

첫 번째 터미널에서 visualizer와 RViz를 실행하고, 두 번째 터미널에서 rosbag을
재생합니다.

```bash
# 터미널 1
ros2 launch rtk_livox_dataset_tools rviz_gt_check.launch.py

# 터미널 2
ros2 bag play /project/bags/run_01_livox_rtk --loop
```

##### 옵션과 alias

| Launch alias | 대응 CLI 옵션 | Launch 기본값 | 설명 |
| --- | --- | --- | --- |
| `calib` | `--calib` | `calibration/run_01_lidar_rtk_alignment.yaml` | RTK를 LiDAR 좌표계로 변환할 calibration YAML |
| `time_offset_sec` | `--time-offset-sec` | `0.0` | RTK timestamp에 적용할 시간 보정값(초) |
| `fix_topic` | `--fix-topic` | `/ublox_gps_node/fix` | 입력 `NavSatFix` 토픽 |
| `fix_velocity_topic` | `--fix-velocity-topic` | `/ublox_gps_node/fix_velocity` | 입력 RTK velocity 토픽 |
| `livox_frame` | `--livox-frame` | `livox_frame` | 출력 메시지와 RViz fixed frame |
| `marker_lifetime_sec` | `--marker-lifetime-sec` | `2.0` | RViz marker 유지 시간(초) |
| `show_speed_text` | `--show-speed-text` | `true` | 속력 text marker 표시 여부 |
| `start_rviz` | - | `true` | visualizer와 함께 RViz를 실행할지 여부 |

출력 토픽은 `/rtk_gt/livox/point`, `/rtk_gt/livox/velocity`,
`/rtk_gt/livox/markers`입니다. RTK는 안테나의 점 궤적이므로 정식 tracking
metric에는 안테나–객체 중심 offset, 평가 class와 box 크기가 추가로 필요합니다.

## OpenPCDet 원본 도구

ROS 2를 거치지 않고 OpenPCDet의 원본 학습·평가·데모도 실행할 수 있습니다. 구체적인 모델 인자는 `OpenPCDet/README.md`를 참고하십시오.

```bash
python3 OpenPCDet/tools/train.py --cfg_file <MODEL_YAML>
python3 OpenPCDet/tools/test.py \
  --cfg_file <MODEL_YAML> --ckpt <CHECKPOINT>
python3 OpenPCDet/tools/demo.py \
  --cfg_file <MODEL_YAML> --ckpt <CHECKPOINT> --data_path <POINT_CLOUD>
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

시각화와 TXT 저장을 동시에 사용하려면 같은 명령에서 `-m both`를 지정합니다.

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
