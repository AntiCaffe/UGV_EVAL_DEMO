# UGV_EVAL_DEMO

Livox LiDAR 기반 3D 객체 검출·추적과 RTK GNSS ground truth 검수를 위한
ROS 2 Foxy 워크스페이스입니다.

## 한눈에 보기

| 목적 | 실행 파일 | 핵심 입력 | 결과 |
| --- | --- | --- | --- |
| 실시간/KITTI 검출·추적 | `pcdet.launch.py` | `/livox/lidar` | `/lr_detections` |
| KITTI point cloud 재생 | `kitti_bin_publisher.launch.py` | KITTI `.bin` 시퀀스 | `/livox/lidar` |
| RTK–Livox GT 검수 | `rviz_gt_check.launch.py` | Livox/RTK rosbag + calibration | rosbag 재생 + RViz GT marker |
| 누적 데이터셋 파싱 | `accumulated_bag_exporter.launch.py` | 검수한 rosbag + calibration | 누적 `.bin` + RTK CSV |
| 오프라인 검출·추적 | `tracking_demo.py` | 누적 `.bin` 시퀀스 | ByteTrack 결과 TXT/시각화 |
| Tracking 평가 | `evaluate.py` | Tracking TXT + `rtk_latest.csv` | 위치·속도 RMSE/MAE |

Livox/RTK 데이터는 아래 순서로 처리합니다.

```text
rosbag + calibration
  → 1. RViz에서 GT 정렬 검수
  → 2. accumulated 데이터셋 파싱
  → 3. OpenPCDet 검출·ByteTrack 추적
  → 4. RTK GT 기반 위치·속도 평가
```

## 1. 빠른 시작

### 준비 사항

- NVIDIA GPU와 Docker GPU runtime
- Docker 이미지 `anticaffe/ugv_project:1.0.0`
- GUI/RViz 사용 시 호스트 X11 접근 권한
- 권장 호스트 CUDA Toolkit: 11.8

### 최초 실행

```bash
docker pull anticaffe/ugv_project:1.0.0
git clone https://github.com/AntiCaffe/UGV_EVAL_DEMO
cd UGV_EVAL_DEMO
```

[entrypoint.bash](entrypoint.bash)의 `PROJECT_DIR`을 현재 저장소의 절대 경로로
설정합니다.

```bash
PROJECT_DIR="/home/ivl/UGV_EVAL_DEMO"
```

RViz/Open3D를 사용할 호스트 터미널에서 X11을 허용한 뒤 컨테이너를 실행합니다.

```bash
xhost +local:docker
./entrypoint.bash
```

`entrypoint.bash`는 `ugv_project` 컨테이너를 생성하고 다음 작업을 자동으로
수행한 뒤 `/project` shell을 엽니다.

```text
cd /project/OpenPCDet && python setup.py develop
cd /project && colcon build --symlink-install
source /project/install/setup.bash
```

### 실행 중인 컨테이너에 다시 접속

```bash
docker exec -it ugv_project bash -lc \
  'source /opt/ros/foxy/setup.bash && source /project/install/setup.bash && exec bash'
```

<details>
<summary>추가 runtime 의존성</summary>

- Livox 실시간 입력: 장비에 맞는 Livox ROS 2 driver
- NTRIP 사용: `ntrip_client`, `rtcm_msgs`
- PCDet launch: `nav2_common`
- PCDet 추론: CUDA 지원 PyTorch와 OpenPCDet 의존성

</details>

## 2. ROS 2 검출·추적

### PCDet 실행

기본 설정은 다음과 같습니다.

| 항목 | 기본값 |
| --- | --- |
| 입력 | `/livox/lidar` (`sensor_msgs/PointCloud2`) |
| 모델 | CenterPoint + `checkpoint_epoch_80.pth` |
| 출력 | `/lr_detections` (`visualization_msgs/MarkerArray`) |

```bash
ros2 launch pcdet_ros2 pcdet.launch.py
```

입출력 토픽이나 메시지 형식을 변경할 수 있습니다.

```bash
ros2 launch pcdet_ros2 pcdet.launch.py \
  input_topic:=/livox/lidar \
  output_topic:=/detections \
  output_format:=detection3d_array
```

`output_format`은 `marker_array` 또는 `detection3d_array`입니다. SECOND 모델을
사용하려면 파라미터 파일을 변경합니다.

```bash
ros2 launch pcdet_ros2 pcdet.launch.py \
  params_file:="$(ros2 pkg prefix pcdet_ros2)/share/pcdet_ros2/config/pcdet_second.param.yaml"
```

### KITTI `.bin` 시퀀스로 확인

각 point는 `float32 [x, y, z, intensity]` 형식이어야 합니다. 두 터미널에서
publisher와 PCDet을 각각 실행합니다.

```bash
# 터미널 1
ros2 launch pcdet_ros2 kitti_bin_publisher.launch.py

# 터미널 2
ros2 launch pcdet_ros2 pcdet.launch.py
```

다른 데이터셋을 사용할 때는 KITTI root 또는 `velodyne` 디렉터리를 지정합니다.

```bash
ros2 launch pcdet_ros2 kitti_bin_publisher.launch.py \
  dataset_path:=/path/to/kitti/training/velodyne \
  input_topic:=/livox/lidar \
  frame_id:=velodyne \
  publish_rate_hz:=10.0 \
  loop:=true
```

<details>
<summary>KITTI publisher 옵션</summary>

| Alias | 기본값 | 설명 |
| --- | --- | --- |
| `dataset_path` | 빈 문자열 | KITTI root 또는 `velodyne` 경로. 비어 있으면 기본 sample 탐색 |
| `input_topic` | `/livox/lidar` | `PointCloud2` 출력 토픽 |
| `frame_id` | `velodyne` | 출력 frame ID |
| `publish_rate_hz` | `10.0` | 발행 주기(Hz) |
| `loop` | `true` | 마지막 파일 이후 처음부터 다시 재생 |

</details>

## 3. Livox/RTK rosbag 처리

### 3.1 RTK–Livox GT 재생·검수

`rviz_gt_check.launch.py`는 **rosbag 재생기이자 GT 시각 검수 도구**입니다.
calibration을 계산하거나 데이터셋을 만들지는 않습니다.

| 구분 | 내용 |
| --- | --- |
| 입력 | Livox/RTK rosbag, calibration YAML |
| 처리 | rosbag 재생, RTK 위치·속도의 LiDAR 좌표계 변환, RViz 실행 |
| 출력 | `/rtk_gt/livox/point`, `/rtk_gt/livox/velocity`, `/rtk_gt/livox/markers` |
| 확인 | LiDAR point cloud와 RTK marker의 위치·방향·시간 정렬 |

Visualizer와 RViz가 준비되도록 기본 1초 후 rosbag 재생을 시작합니다.

```bash
ros2 launch rtk_livox_dataset_tools rviz_gt_check.launch.py \
  bag:=/project/bags/run_01_livox_rtk \
  calib:=/project/calibration/run_01_lidar_rtk_alignment.yaml \
  loop:=true
```

GT 정렬을 확인한 뒤 같은 rosbag과 calibration을 exporter에 전달합니다.

<details>
<summary>rviz_gt_check 옵션과 alias</summary>

| Launch alias | 대응 실행 옵션 | 기본값 | 설명 |
| --- | --- | --- | --- |
| `bag` | `ros2 bag play <PATH>` | `/project/bags/run_01_livox_rtk` | 재생할 rosbag |
| `loop` | `ros2 bag play --loop` | `true` | 반복 재생 여부 |
| `play_delay_sec` | Launch 전용 | `1.0` | rosbag 재생 전 대기 시간(초) |
| `calib` | `--calib` | `/project/calibration/run_01_lidar_rtk_alignment.yaml` | RTK–LiDAR calibration YAML |
| `time_offset_sec` | `--time-offset-sec` | `0.0` | 출력 RTK timestamp에 더할 값(초) |
| `fix_topic` | `--fix-topic` | `/ublox_gps_node/fix` | rosbag의 `NavSatFix` 토픽 |
| `fix_velocity_topic` | `--fix-velocity-topic` | `/ublox_gps_node/fix_velocity` | rosbag의 RTK velocity 토픽 |
| `livox_frame` | `--livox-frame` | `livox_frame` | 출력 및 RViz fixed frame |
| `marker_lifetime_sec` | `--marker-lifetime-sec` | `2.0` | marker 유지 시간(초) |
| `show_speed_text` | `--show-speed-text` | `true` | 속력 text 표시 여부 |
| `start_rviz` | Launch 전용 | `true` | RViz 자동 실행 여부 |

</details>

### 3.2 Accumulated 데이터셋 파싱

`accumulated_bag_exporter.launch.py`는 **rosbag과 calibration 기반의 오프라인
dataset parser**입니다. ROS 토픽으로 재생하지 않고 bag 파일을 직접 읽습니다.

| 구분 | 내용 |
| --- | --- |
| 입력 | 검수한 Livox/RTK rosbag, calibration YAML |
| LiDAR 처리 | sparse packet을 causal time window로 누적 |
| RTK 처리 | 각 프레임 시각 이전의 최신 fix/velocity 선택 및 LiDAR 좌표계 변환 |
| 출력 | OpenPCDet `.bin`, 프레임/RTK CSV, metadata YAML |

Calibration YAML에는 `origin_llh`, `yaw_enu_lidar_rad`,
`lidar_position_enu`가 있어야 합니다. 출력 디렉터리는 덮어쓰지 않으므로 매번
새로운 경로를 지정합니다.

```bash
ros2 launch rtk_livox_dataset_tools accumulated_bag_exporter.launch.py \
  bag:=/project/bags/run_01_livox_rtk \
  calib:=/project/calibration/run_01_lidar_rtk_alignment.yaml \
  output_dir:=/project/datasets/run_02_accumulated \
  accumulation_sec:=0.2 \
  output_rate_hz:=10.0 \
  max_rtk_age_sec:=0.5
```

#### 파싱 정책

- Livox는 `(t - accumulation_sec, t]` 범위의 과거 packet만 누적합니다.
- RTK는 미래 값이나 보간 값이 아닌 프레임 이전의 최신 값을 사용합니다.
- `aligned_header`는 Livox 상대 header clock을 rosbag epoch에 정렬합니다.
- 이동식 LiDAR에는 별도의 ego-motion compensation이 필요합니다.
- Calibration을 생략하면 LiDAR 좌표계 RTK 열이 `nan`이 되어 평가용 GT로
  사용할 수 없습니다.

#### 출력 구조

```text
datasets/run_02_accumulated/
├── velodyne/000000.bin  # 누적 float32 [x, y, z, intensity]
├── frames.csv           # 프레임 시간과 point 통계
├── rtk_latest.csv       # 최신 RTK, sample age, LiDAR 좌표계 GT
├── ImageSets/test.txt   # 프레임 순서
└── metadata.yaml        # 입력, 파싱 설정과 결과 통계
```

`fix_age_sec`, `velocity_age_sec`, `rtk_is_fresh`를 사용해 오래된 RTK를
필터링할 수 있습니다.

<details>
<summary>accumulated_bag_exporter 옵션과 alias</summary>

| Launch alias | 대응 CLI 옵션 | 기본값 | 설명 |
| --- | --- | --- | --- |
| `bag` | `--bag` | `/project/bags/run_01_livox_rtk` | 파싱할 rosbag |
| `calib` | `--calib` | `/project/calibration/run_01_lidar_rtk_alignment.yaml` | RTK–LiDAR calibration. 평가에서는 필수 |
| `output_dir` | `--output-dir` | `/project/datasets/run_01_accumulated` | 출력 경로. 기존 경로는 덮어쓰지 않음 |
| `cloud_topic` | `--cloud-topic` | `/livox/lidar` | Livox `PointCloud2` 토픽 |
| `fix_topic` | `--fix-topic` | `/ublox_gps_node/fix` | RTK `NavSatFix` 토픽 |
| `velocity_topic` | `--velocity-topic` | `/ublox_gps_node/fix_velocity` | RTK velocity 토픽 |
| `storage_id` | `--storage-id` | `sqlite3` | rosbag storage backend |
| `time_source` | `--time-source` | `aligned_header` | `aligned_header` 또는 `bag` timestamp 사용 |
| `output_rate_hz` | `--output-rate-hz` | `10.0` | 출력 frame rate(Hz) |
| `accumulation_sec` | `--accumulation-sec` | `0.2` | 누적 시간창(초) |
| `max_rtk_age_sec` | `--max-rtk-age-sec` | `0.5` | 최신 RTK로 인정할 최대 age(초) |
| `drop_stale_rtk` | `--drop-stale-rtk` | `false` | 오래된 RTK frame 제외 여부 |
| `voxel_size` | `--voxel-size` | `0.0` | voxel 크기(m). `0.0`이면 비활성화 |
| `max_points_per_frame` | `--max-points-per-frame` | `0` | frame당 point 제한. `0`이면 제한 없음 |

</details>

### 3.3 Tracking infer

Exporter가 만든 `velodyne` 디렉터리를 `tracking_demo.py`에 전달합니다.
입력 `.bin`/`.npy` 파일은 파일명 순서로 검출하고 ByteTrack으로 추적합니다.

```bash
cd /project/OpenPCDet/tools
python tracking_demo.py \
  --cfg_file cfgs/kitti_models/second_KN.yaml \
  --ckpt checkpoints/second_KN.pth \
  --data_path ../../datasets/run_02_accumulated/velodyne \
  --track_classes 1 2 3 \
  --frame_rate 10 \
  -m infer \
  -o ../../tracking_results/run_02
```

입력 파일마다 `<POINT_CLOUD_STEM>.txt`가 출력 디렉터리에 생성됩니다. 예를
들어 `000000.bin`과 `000001.bin`은 각각 `000000.txt`와 `000001.txt`로
저장됩니다.

각 track 행은 탭으로 구분되며 다음 열을 가집니다.

```text
frame frame_file track_id class_id class_name score
x1 y1 z1 x2 y2 z2 yaw vx vy vz speed detection_index
```

#### 실행 모드

| 모드 | Open3D 시각화 | 프레임별 TXT 저장 |
| --- | --- | --- |
| `demo` | 사용 | 사용 안 함 |
| `infer` | 사용 안 함 | 사용 |
| `both` | 사용 | 사용 |

<details>
<summary>tracking_demo.py 옵션과 alias</summary>

| 짧은 alias | 긴 옵션 | 기본값 | 설명 |
| --- | --- | --- | --- |
| `-h` | `--help` | - | 도움말 출력 |
| - | `--cfg_file` | `cfgs/kitti_models/second.yaml` | 모델 설정 YAML |
| - | `--data_path` | 필수 | point cloud 파일 또는 시퀀스 디렉터리 |
| - | `--ckpt` | 필수 | 모델 checkpoint |
| `-m` | `--mode` | `demo` | `demo`, `infer`, `both` |
| `-o` | `--output_dir` | `tracking_results` | TXT 출력 디렉터리 |
| - | `--output_path` | `tracking_results` | `--output_dir` 호환 alias |
| - | `--ext` | `.bin` | `.bin` 또는 `.npy` |
| - | `--track_classes` | `2` | KITTI class ID: `1=Car`, `2=Pedestrian`, `3=Cyclist` |
| - | `--frame_rate` | `30` | ByteTracker 입력 frame rate |
| - | `--velocity_scale` | `1.0` | 속도 화살표 시간 배율 |
| - | `--no_visualization` | 비활성 | Open3D 창 비활성화 |

</details>

RTK는 안테나의 점 궤적입니다. 정식 tracking metric에는 안테나–객체 중심
offset, 평가 class와 3D box 크기가 추가로 필요합니다.

### 3.4 Tracking 결과 평가

`evaluate.py`는 tracking TXT와 exporter의 `rtk_latest.csv`를 frame ID로
정렬해 단일 target track을 선택하고 결과를 저장합니다. RMSE/MAE 계산은
재사용 가능한 [track_metrics.py](OpenPCDet/tools/track_metrics.py)의 독립
RMSE/MAE 함수로 분리되어 있으며 ROS 2를 사용하지 않습니다.

```bash
cd /project/OpenPCDet/tools
python evaluate.py \
  --tracking-dir ../../tracking_results/run_02 \
  --rtk-csv ../../datasets/run_02_accumulated/rtk_latest.csv \
  --output-dir ../../evaluation_results/run_02 \
  --visualization true
```

| 평가값 | 정의 |
| --- | --- |
| 위치 RMSE/MAE | Tracking box 중심과 RTK `p_lidar_*` 사이의 2D/3D 거리 오차 |
| 속도 RMSE/MAE | Tracking `vx/vy/vz`와 RTK `v_lidar_*` 사이의 2D/3D vector 오차 |
| Speed RMSE/MAE | 두 velocity vector 크기의 scalar 차이 |
| Coverage | 유효한 RTK frame 중 선택된 track이 존재하는 frame 비율 |

`--track-id`를 생략하면 각 frame에서 RTK에 가장 가까운 track을 찾고, 가장
자주 선택된 하나의 일관된 track ID를 평가 대상으로 정합니다. 대상 ID를 알고
있다면 자동 선택보다 `--track-id <ID>`를 지정하는 것이 권장됩니다.

```bash
python evaluate.py \
  --tracking-dir ../../tracking_results/run_02 \
  --rtk-csv ../../datasets/run_02_accumulated/rtk_latest.csv \
  --output-dir ../../evaluation_results/run_02 \
  --track-id 7 \
  --class-id 2
```

기본적으로 `rtk_is_fresh=1`인 frame만 평가합니다. 오차는 track과 RTK가 모두
존재하는 frame에서 계산되므로 RMSE/MAE와 함께 coverage 및 누락 frame 수를
확인해야 합니다.

`-v true` 또는 `--visualization true`이면 평가 결과와 시각화 이미지가 같은
디렉터리에 저장됩니다. `false`, `no`, `0`, `off`를 지정하면 이미지 생성을
건너뜁니다. PNG 생성에는 Matplotlib을 사용하며 Docker나 SSH처럼 화면이 없는
환경에서도 바로 저장됩니다.

```text
evaluation_results/run_02/
├── summary.json          # 선택 track, coverage, RMSE/MAE 요약
├── frame_errors.csv      # frame별 예측·GT와 위치·속도 오차
├── trajectory_xy.png     # Tracking과 RTK GT의 XY 궤적
├── position_errors.png   # frame별 XYZ 및 2D/3D 위치 오차
├── velocity_errors.png   # frame별 XYZ 및 2D/3D 속도 오차
├── speed_comparison.png  # Tracking/RTK 속력과 signed error
└── metrics_summary.png   # RMSE/MAE와 coverage 요약
```

<details>
<summary>evaluate.py 옵션</summary>

| 옵션 | 기본값 | 설명 |
| --- | --- | --- |
| `--tracking-dir` | 필수 | frame별 tracking TXT 디렉터리 |
| `--rtk-csv` | 필수 | exporter가 생성한 `rtk_latest.csv` |
| `--output-dir` | `evaluation_results` | 평가 결과 디렉터리 |
| `-v`, `--visualization` | `true` | PNG 생성 여부: `true/false`, `yes/no`, `1/0`, `on/off` |
| `--plot-dpi` | `150` | 저장할 PNG 해상도(DPI) |
| `--track-id` | 자동 선택 | 평가할 고정 track ID |
| `--class-id` | 전체 class | association과 평가에 사용할 class ID |
| `--min-score` | `0.0` | 평가에 포함할 최소 tracking score |
| `--association-gate-m` | `5.0` | 자동 track 선택에 사용할 최대 거리(m) |
| `--association-dimension` | `2d` | 자동 association 거리: `2d` 또는 `3d` |
| `--include-stale-rtk` | 비활성 | `rtk_is_fresh!=1`인 RTK도 포함 |
| `--antenna-offset-lidar X Y Z` | `0 0 0` | RTK 위치에서 뺄 LiDAR-frame 안테나–target offset |

</details>

## 4. OpenPCDet 학습·평가 도구

아래 도구는 accumulated 파싱·tracking infer 흐름과 별개인 OpenPCDet 원본
학습, 평가 및 단일 프레임 데모입니다.

```bash
python3 OpenPCDet/tools/train.py --cfg_file <MODEL_YAML>
python3 OpenPCDet/tools/test.py \
  --cfg_file <MODEL_YAML> --ckpt <CHECKPOINT>
python3 OpenPCDet/tools/demo.py \
  --cfg_file <MODEL_YAML> --ckpt <CHECKPOINT> --data_path <POINT_CLOUD>
```

## 5. 저장소 구성

| 경로 | 역할 |
| --- | --- |
| `src/pcdet_ros2` | OpenPCDet ROS 2 노드와 launch |
| `src/rtk_livox_dataset_tools` | rosbag 재생·검수 및 accumulated parser |
| `src/ros2_numpy` | ROS 메시지–NumPy 변환 |
| `src/vision_msgs_rviz_plugins` | `Detection3DArray` RViz plugin |
| `src/ublox` | u-blox GNSS driver와 message |
| `OpenPCDet` | OpenPCDet 학습·평가·데모 코드 |
| `cfgs`, `config`, `checkpoints` | 모델 YAML, ROS parameter, checkpoint |

## 참고 및 주의사항

- [PCDet ROS 2 패키지 설명](src/pcdet_ros2/README.md)
- [OpenPCDet 설명](OpenPCDet/README.md)
- `bags/`, `*.pth`, `build/`, `install/`, `log/`는 Git 관리 대상이 아닙니다.
- 모델 YAML과 checkpoint는 서로 호환되어야 합니다.
- 현장 운용 전 `/livox/lidar`의 topic type과 발행 주기를 확인하십시오.
