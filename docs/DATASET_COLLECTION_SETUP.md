# DATASET_COLLECTION_SETUP

작성자: 김동희
상태: 초안
최종 편집 일시: 2026년 7월 2일 오후 4:13
생성 일시: 2026년 7월 2일 오후 4:12
상위 항목: 실증 데이터셋 수집 (https://app.notion.com/p/38e2d863200d80e99270c52b23c908f4?pvs=21)

# Livox Avia + RTK GNSS 데이터셋 수집 절차

이 문서는 Livox Avia 기반 3D detection/tracking 평가와 RTK GNSS 위치/속도 GT 수집을 위한 두 노트북 개발/운용 절차를 정리한다.

## 목표

- Livox Avia point cloud로 사람 3D detection/tracking 모델을 평가한다.
- 정해진 경로로 움직이는 사람 5명 중, 각 run에서 RTK GNSS를 든 1명의 위치와 속도를 GT로 사용한다.
- RTK GNSS가 1개뿐이므로 같은 시나리오를 5회 반복 촬영하고, 매 run마다 RTK 착용자를 바꾼다.

## 장비 구성

| 장비 | 역할 | 주요 기록 |
| --- | --- | --- |
| 노트북 1 | Livox Avia 연결, 차량로봇(Scout)에 고정된 LiDAR 정지 운용 | point cloud, detection/tracking 결과 또는 원본 point cloud |
| 노트북 2 | ZED-F9P RTK GNSS + NTRIP 연결 | `/ublox_gps_node/navpvt`, `/fix`, `/fix_velocity`, `/rtcm` |
| RTK GNSS | 각 run에서 GT 대상자 1명이 착용 | 위치, 속도, RTK 상태 |
| Livox Avia | 차량로봇에 장착 후 시나리오 중 정지 상태로 운동장 30 m 거리 측정 | 고정 LiDAR frame |

## 핵심 제약

- RTK GNSS가 1개뿐이므로 5명 전체의 동시 GT는 만들 수 없다.
- 각 run에서 RTK를 든 1명만 GT 대상이다.
- 최종 평가는 5개 run을 합쳐서 한다.
- 시나리오 촬영 중 Livox는 정지 상태이므로 LiDAR frame을 고정 world frame처럼 사용할 수 있다.
- 촬영 전 RTK를 차량로봇에 설치하고 직선 주행으로 LiDAR/차량 yaw를 추정한다.
- RTK는 ENU 기준 위치/속도를 제공하므로, LiDAR frame과 position/yaw 정렬이 필요하다.

## 노트북 1: Livox Avia 컴퓨터

### 설치/빌드

Livox ROS2 드라이버와 데이터 기록용 패키지를 준비한다. 실제 패키지 이름은 설치한 Livox 드라이버에 맞춘다.

```bash
cd ~/livox_ws
colcon build
source install/setup.bash
```

### 실행

LiDAR 드라이버를 실행한다.

```bash
source ~/livox_ws/install/setup.bash
ros2 launch <livox_driver_package> <livox_launch_file>
```

포인트클라우드 토픽을 확인한다.

```bash
ros2 topic list
ros2 topic hz /livox/lidar
```

토픽 이름은 실제 환경에 맞게 확인해서 사용한다. 예시는 `/livox/lidar`로 표기한다.

### 기록

원본 point cloud만 저장하는 경우:

```bash
ros2 bag record \
  /livox/lidar \
  -o bags/livox_run_01
```

tracking 결과까지 같이 저장하는 경우:

```bash
ros2 bag record \
  /livox/lidar \
  /tracking/objects \
  /tracking/tracks \
  -o bags/livox_run_01
```

### 원커맨드 수집 실행

현장에서는 Livox driver와 tracking pipeline을 먼저 띄운 뒤, 아래 launch로 필요한 토픽을 한 번에 기록한다. 실제 Livox topic 이름은 장비 환경에 맞게 바꾼다.

```bash
source ~/sensor_project_dataset_2026_ws/install/setup.bash
ros2 launch rtk_livox_dataset_tools livox_collection.launch.py \
  bag_uri:=bags/run_01_livox \
  record_topics:="/livox/lidar /tracking/objects /tracking/tracks"
```

point cloud만 기록할 때:

```bash
ros2 launch rtk_livox_dataset_tools livox_collection.launch.py \
  bag_uri:=bags/run_01_livox \
  record_topics:="/livox/lidar"
```

## 노트북 2: RTK GNSS 컴퓨터

### 빌드

```bash
cd ~/rtk_ws
colcon build
source install/setup.bash
```

### Ublox 실행

```bash
source ~/rtk_ws/install/setup.bash
ros2 launch ublox_gps ublox_gps_node-launch.py
```

### NTRIP 실행

NTRIP 접속 정보는 파일에 직접 저장하지 말고 launch 인자로 넘기는 것을 권장한다.

```bash
source ~/rtk_ws/install/setup.bash
ros2 launch ntrip_client ntrip_client_launch.py \
  host:=www.gnssdata.or.kr \
  port:=2101 \
  mountpoint:=<MOUNTPOINT> \
  authenticate:=True \
  username:=<NTRIP_USER> \
  password:=<NTRIP_PASSWORD> \
  rtcm_message_package:=rtcm_msgs
```

국토지리정보원 GNSS 자료 통합센터의 NTRIP 비밀번호는 사이트 로그인 비밀번호와 다를 수 있다. 서비스 안내에 나온 NTRIP 전용 비밀번호를 사용한다.

### RTK 상태 확인

RTCM이 들어오는지 확인한다.

```bash
ros2 topic hz /rtcm
ros2 topic echo /rtcm --once
```

RTK 상태를 확인한다.

```bash
ros2 topic echo /ublox_gps_node/navpvt --once
```

자주 볼 필드:

| 필드 | 좋은 상태 |
| --- | --- |
| `fix_type` | `3` |
| `flags` | `131`이면 RTK fixed, `67`이면 RTK float |
| `num_sv` | 높을수록 좋음. 10개 이상 권장, 30개 이상이면 매우 좋음 |
| `h_acc` | mm 단위. fixed에서 수 cm급 기대 |
| `v_acc` | mm 단위 |
| `s_acc` | mm/s 단위 |

해석:

```
flags 67  = 64 + 2 + 1   = RTK float + 보정 사용 + GNSS fix OK
flags 131 = 128 + 2 + 1  = RTK fixed + 보정 사용 + GNSS fix OK
```

### RTK rosbag 기록

LiDAR pose 캘리브레이션용 직선 주행/정지 구간:

```bash
ros2 bag record \
  /ublox_gps_node/navpvt \
  /ublox_gps_node/fix \
  /ublox_gps_node/fix_velocity \
  /rtcm \
  -o bags/run_01_lidar_pose_calib
```

시나리오 촬영 중 착용자 GT 구간:

```bash
ros2 bag record \
  /ublox_gps_node/navpvt \
  /ublox_gps_node/fix \
  /ublox_gps_node/fix_velocity \
  /rtcm \
  -o bags/rtk_run_01
```

### 원커맨드 수집 실행

RTK 노트북에서는 Ublox, NTRIP, RTK 상태 모니터, rosbag record를 한 번에 실행한다.

```bash
source ~/sensor_project_dataset_2026_ws/install/setup.bash
ros2 launch rtk_livox_dataset_tools rtk_collection.launch.py \
  bag_uri:=bags/run_01_rtk \
  ntrip_mountpoint:=<MOUNTPOINT> \
  ntrip_username:=<NTRIP_USER> \
  ntrip_password:=<NTRIP_PASSWORD>
```

이미 Ublox/NTRIP을 따로 실행해둔 상태에서 기록만 시작하려면:

```bash
ros2 launch rtk_livox_dataset_tools rtk_collection.launch.py \
  start_ublox:=false \
  start_ntrip:=false \
  bag_uri:=bags/run_01_rtk
```

### RTK 노트북을 닫고 가방에 넣는 운용

RTK+GNSS 노트북은 사람이 직접 Enter를 누를 수 없으므로, 노트북 1에서 SSH로 원격 실행하는 방식을 권장한다.

사전 준비:

```
1. RTK 노트북 전원 설정에서 lid close 시 suspend 안 되게 설정
2. 두 노트북이 같은 Wi-Fi 또는 같은 공유기/핫스팟에 연결
3. RTK 노트북에서 SSH server 활성화
4. 노트북 1에서 RTK 노트북으로 SSH key 로그인 확인
5. RTK 노트북은 tmux 설치 권장
```

RTK 노트북에서 한 번만 설정:

```bash
sudo systemctl enable ssh
sudo systemctl start ssh
sudo loginctl show-user $USER -p Linger
sudo loginctl enable-linger $USER
```

lid close suspend 방지는 Ubuntu 전원 설정 GUI에서 처리하거나, 현장 전용 노트북이면 `/etc/systemd/logind.conf`에서 아래 값을 설정한다.

```text
HandleLidSwitch=ignore
HandleLidSwitchExternalPower=ignore
```

설정 후:

```bash
sudo systemctl restart systemd-logind
```

노트북 1에서 RTK 수집 시작:

```bash
ssh <RTK_USER>@<RTK_LAPTOP_IP> \
  'tmux new-session -d -s rtk_run "cd ~/sensor_project_dataset_2026_ws && source install/setup.bash && ros2 launch rtk_livox_dataset_tools rtk_collection.launch.py bag_uri:=bags/run_01_rtk ntrip_mountpoint:=<MOUNTPOINT> ntrip_username:=<NTRIP_USER> ntrip_password:=<NTRIP_PASSWORD>"'
```

상태 확인:

```bash
ssh <RTK_USER>@<RTK_LAPTOP_IP> 'tmux capture-pane -pt rtk_run'
ssh <RTK_USER>@<RTK_LAPTOP_IP> 'ls -lh ~/sensor_project_dataset_2026_ws/bags/run_01_rtk'
```

수집 종료:

```bash
ssh <RTK_USER>@<RTK_LAPTOP_IP> 'tmux send-keys -t rtk_run C-c'
```

종료 후 bag이 정상 닫혔는지 확인:

```bash
ssh <RTK_USER>@<RTK_LAPTOP_IP> 'ros2 bag info ~/sensor_project_dataset_2026_ws/bags/run_01_rtk'
```

이 방식이면 노트북 2를 가방에 넣고 닫아둔 상태에서도 노트북 1에서 RTK 기록 시작/종료를 제어할 수 있다. 두 노트북의 시작 시각은 여전히 맞출 필요가 없고, 두 bag이 모두 recording 상태가 된 뒤 RTK 착용자가 동기화용 정지/출발 이벤트를 수행하면 된다.

## 두 노트북 시간 동기화

학교 Wi-Fi를 사용할 예정이다. 가장 좋은 방법은 두 노트북이 같은 Wi-Fi에 붙은 상태에서 `chrony`로 시간 동기화한 뒤, 각 run마다 LiDAR/RTK 공통 이벤트로 후처리 offset을 한 번 더 추정하는 것이다.

중요한 전제:

```
두 노트북의 rosbag record 시작 시각은 맞출 필요가 없다.
노트북 1에서 Enter를 누르는 시각과 노트북 2에서 Enter를 누르는 시각이 달라도 된다.
후처리는 bag 시작 시각이 아니라 각 message timestamp와 공통 motion event를 기준으로 정렬한다.
```

따라서 현장에서는 두 bag이 모두 recording 상태가 된 것을 확인한 뒤, RTK 착용자가 LiDAR 시야 안에서 동기화용 동작을 수행한다.

권장 구성:

```
노트북 1: Livox 컴퓨터, chrony server
노트북 2: RTK 컴퓨터, chrony client
```

먼저 두 노트북이 서로 통신 가능한지 확인한다. 학교 Wi-Fi는 보안 정책상 같은 AP에 있어도 client 간 통신이 막힐 수 있다.

```bash
hostname -I
ping <other_laptop_ip>
```

`ping`이 안 되면 chrony server/client 방식은 어렵다. 이 경우 두 노트북 모두 인터넷 NTP에 동기화하고, 후처리에서 시작/정지 이벤트로 시간 offset을 보정한다. 인터넷 NTP도 어렵다면 기록 자체는 가능하지만, 후처리 offset 추정 품질을 더 엄격하게 확인해야 한다.

### chrony server/client 방식

노트북 1에서 chrony server를 열고, 노트북 2가 노트북 1을 시간 기준으로 사용한다.

노트북 1에서 확인:

```bash
hostname -I
chronyc tracking
```

노트북 2에서 확인:

```bash
chronyc sources -v
chronyc tracking
```

노트북 2의 source가 노트북 1 IP로 잡히는지 확인한다.

권장 확인:

```bash
chronyc tracking
chronyc sources -v
```

목표:

```
두 노트북 시간 offset: 가능하면 10 ms 이하
```

학교 Wi-Fi에서 지연/roaming이 생길 수 있으므로, chrony가 동작하더라도 후처리 시간 보정용 이벤트는 반드시 만든다.

권장 이벤트 sequence:

```
0. 두 노트북 rosbag record가 모두 시작된 것을 확인
1. RTK 착용자가 LiDAR 시야 안의 잘 보이는 위치에서 3초 이상 정지
2. 명확하게 출발: 1~2초 안에 속도가 확 올라가도록 걷기/뛰기 시작
3. 시나리오 수행
4. 종료 후 LiDAR 시야 안에서 다시 3초 이상 정지
```

후처리에서는 LiDAR track 속도 변화와 RTK 속도 변화를 맞춰 시간 offset을 추정한다. `ros2 bag record` 시작 시각 차이는 offset 계산에 직접 사용하지 않는다.

권장 offset 정의:

```yaml
time_offset_livox_minus_rtk_sec: <t_livox_event - t_rtk_event>
```

이 값을 `dt`라고 하면 RTK timestamp를 Livox clock에 맞출 때는 아래처럼 적용한다.

```
t_rtk_on_livox_clock = t_rtk + dt
```

반대로 Livox timestamp `t_livox`에서 비교할 RTK 값을 interpolation할 때는 아래 시각의 RTK sample을 사용한다.

```
t_rtk_query = t_livox - dt
```

### 후처리 offset 추정 방법

1차 방법은 이벤트 기반 정렬이다.

```
1. LiDAR tracking 결과에서 RTK 착용자 track의 speed_2d(t)를 만든다.
2. RTK `/ublox_gps_node/fix_velocity` 또는 `navpvt` velocity에서 speed_2d(t)를 만든다.
3. 시작 정지 -> 출발 구간에서 speed가 threshold를 넘는 시각을 각각 찾는다.
4. 종료 이동 -> 정지 구간에서 speed가 threshold 아래로 내려가는 시각을 각각 찾는다.
5. 시작 event offset과 종료 event offset이 비슷한지 확인한다.
6. 두 값이 잘 맞으면 평균값을 run의 `time_offset_livox_minus_rtk_sec`로 저장한다.
```

권장 threshold:

```
정지 판정: speed_2d < 0.15 m/s
출발 판정: speed_2d > 0.5 m/s
event 탐색 window: 시작/종료 정지 주변 5~10초
허용 차이: 시작 offset과 종료 offset 차이 50 ms 이하 권장, 100 ms 이상이면 수동 확인
```

2차 방법은 speed cross-correlation이다.

```
1. 후보 offset 범위를 정한다. 예: -5.0 sec ~ +5.0 sec
2. 0.01 sec 간격으로 RTK speed를 LiDAR 시간축에 interpolation한다.
3. LiDAR speed와 RTK speed의 correlation 또는 RMSE를 계산한다.
4. score가 가장 좋은 offset을 선택한다.
5. 시작/종료 이벤트 기반 offset과 비교해 confidence를 기록한다.
```

권장 저장 파일:

```yaml
run_id: run_01
time_offset_livox_minus_rtk_sec: 0.0
method: event_start_end_and_speed_correlation
start_event_offset_sec: 0.0
end_event_offset_sec: 0.0
start_end_offset_diff_sec: 0.0
confidence: high
notes: "positive means RTK timestamps are behind Livox timestamps"
```

주의할 점:

- bag directory의 시작 시간이나 파일 생성 시간으로 정렬하지 않는다.
- ROS message `header.stamp`를 우선 사용하고, header가 없거나 부정확한 토픽만 bag record time을 fallback으로 사용한다.
- RTK의 GNSS time은 매우 안정적이지만, LiDAR point cloud와 비교하려면 Livox 컴퓨터의 ROS timestamp 기준과 연결되어야 한다.
- point cloud만 저장했다면 후처리 detection/tracking을 먼저 수행해 RTK 착용자 track의 위치/속도 시계열을 만든 뒤 offset을 추정한다.

## RealSense 2대 시간 동기화 리허설

Livox/RTK를 바로 쓰기 어렵다면, 두 노트북에 각각 RealSense D435를 연결해 pointcloud rosbag을 저장하고 같은 방식으로 시간 동기화를 검증한다.

목표:

```
노트북 1: RealSense D435 pointcloud 기록 -> bags/run_01_rs1
노트북 2: RealSense D435 pointcloud 기록 -> bags/run_01_rs2
후처리/검수: 두 bag을 같은 시간축으로 재생하고 RViz2에서 움직이는 물체가 같은 타이밍에 움직이는지 확인
```

중요:

```
두 노트북 모두 topic 이름을 다르게 기록한다.
노트북 1은 /rs1/...
노트북 2는 /rs2/...
같은 /camera/... 이름으로 기록하면 나중에 두 bag을 같이 play할 때 topic이 충돌한다.
```

### 노트북 1 RealSense 수집

```bash
source ~/sensor_project_dataset_2026_ws/install/setup.bash
ros2 launch rtk_livox_dataset_tools realsense_collection.launch.py \
  camera_namespace:=rs1 \
  camera_name:=camera \
  bag_uri:=bags/run_01_rs1 \
  record_topics:="/rs1/camera/depth/color/points"
```

토픽 이름이 다른 경우 먼저 확인한다.

```bash
ros2 topic list | grep points
```

예를 들어 pointcloud topic이 `/rs1/camera/camera/depth/color/points`로 뜨면 `record_topics`를 그 이름으로 바꾼다.

### 노트북 2 RealSense 수집

노트북 2에서 직접 실행하거나, 노트북 1에서 SSH로 원격 실행한다.

노트북 2에서 직접 실행:

```bash
source ~/sensor_project_dataset_2026_ws/install/setup.bash
ros2 launch rtk_livox_dataset_tools realsense_collection.launch.py \
  camera_namespace:=rs2 \
  camera_name:=camera \
  bag_uri:=bags/run_01_rs2 \
  record_topics:="/rs2/camera/depth/color/points"
```

노트북 1에서 SSH로 원격 실행:

```bash
ssh <RS2_USER>@<RS2_LAPTOP_IP> \
  'tmux new-session -d -s rs2_run "cd ~/sensor_project_dataset_2026_ws && source install/setup.bash && ros2 launch rtk_livox_dataset_tools realsense_collection.launch.py camera_namespace:=rs2 camera_name:=camera bag_uri:=bags/run_01_rs2 record_topics:=/rs2/camera/depth/color/points"'
```

종료:

```bash
ssh <RS2_USER>@<RS2_LAPTOP_IP> 'tmux send-keys -t rs2_run C-c'
```

### 리허설 동작

두 bag이 모두 recording 상태가 된 뒤, 두 카메라가 동시에 볼 수 있는 위치에서 사람이 손을 크게 흔들거나 물체를 좌우로 빠르게 움직인다.

권장 sequence:

```
1. 두 bag recording 확인
2. 물체 3초 정지
3. 손/물체를 좌우로 3~5회 빠르게 움직임
4. 다시 3초 정지
5. record 종료
```

이 움직임이 Livox/RTK에서 사용할 공통 motion event 역할을 한다.

### RViz2 비교

두 bag을 한 노트북으로 모은다.

```
bags/run_01_rs1
bags/run_01_rs2
```

먼저 RViz용 republisher를 실행한다. 이 노드는 두 pointcloud의 `frame_id`를 `sync_check_world`로 통일하고, 기본값으로 `rs2` cloud를 y 방향 2 m 옆으로 밀어 side-by-side로 보여준다.

```bash
source install/setup.bash
ros2 launch rtk_livox_dataset_tools realsense_sync_check.launch.py \
  cloud1_topic:=/rs1/camera/depth/color/points \
  cloud2_topic:=/rs2/camera/depth/color/points \
  fixed_frame:=sync_check_world \
  cloud2_offset_y:=2.0
```

RViz 설정:

```
Fixed Frame: sync_check_world
PointCloud2: /sync_check/rs1/points
PointCloud2: /sync_check/rs2/points
```

두 bag을 같은 실제 시간축으로 재생한다. chrony로 두 노트북 시간이 맞아 있었다면 offset은 0으로 둔다.

```bash
ros2 run rtk_livox_dataset_tools realsense_sync_play \
  --bag1 bags/run_01_rs1 \
  --bag2 bags/run_01_rs2 \
  --time-offset-bag1-minus-bag2-sec 0.0
```

나중에 visual event로 offset을 수동 추정했다면 아래처럼 넣는다.

```bash
ros2 run rtk_livox_dataset_tools realsense_sync_play \
  --bag1 bags/run_01_rs1 \
  --bag2 bags/run_01_rs2 \
  --time-offset-bag1-minus-bag2-sec <t_rs1_event_minus_t_rs2_event>
```

판정:

```
OK: 두 pointcloud에서 손/물체 움직임이 같은 순간에 시작/정지함
WARN: 1~2 frame 정도 차이. RealSense FPS와 Wi-Fi/clock 상태 확인
FAIL: 움직임 시작이 눈에 띄게 어긋남. chrony 상태 또는 event 기반 offset 재추정 필요
```

## LiDAR Pose 캘리브레이션

Livox가 시나리오 중 정지 상태이므로, 촬영 전에 LiDAR frame과 RTK ENU frame 사이의 고정 변환을 구하면 된다. RTK GNSS가 1개뿐이므로 먼저 차량로봇에 RTK를 설치해 LiDAR pose를 추정하고, 이후 RTK를 분리해 GT 대상자에게 장착한다.

### 필요한 사전 측정

차량로봇에 LiDAR와 RTK 안테나를 동시에 설치했을 때의 고정 TF를 미리 측정한다.

```
T_lidar_antenna 또는 T_base_antenna, T_base_lidar
```

최소한 LiDAR frame에서 본 안테나 offset은 알아야 한다.

```
p_antenna_in_lidar:
  x: LiDAR 전방 기준 안테나 위치 [m]
  y: LiDAR 좌측 기준 안테나 위치 [m]
  z: LiDAR 상방 기준 안테나 위치 [m]
```

이 offset을 모르면 RTK가 측정한 것은 LiDAR 위치가 아니라 안테나 위치이므로, LiDAR position GT에 수십 cm 이상의 bias가 생길 수 있다.

### 촬영 전 LiDAR Pose 추정 절차

1. 차량로봇의 정해진 위치에 Livox와 RTK 안테나를 함께 설치한다.
2. Livox와 RTK 안테나 사이의 고정 TF가 맞는지 확인한다.
3. 차량로봇을 조향 없이 앞/뒤로 직선 이동한다.
4. 이동 중 RTK 위치 궤적을 저장한다.
5. 직선 구간의 시작/끝 또는 전체 궤적 PCA로 차량/LiDAR yaw를 추정한다.
6. 차량로봇을 최종 촬영 위치에 세운다.
7. 10초 이상 정지한 RTK 위치 평균으로 최종 안테나 위치를 추정한다.
8. 안테나-LiDAR offset을 보정해서 최종 LiDAR position을 계산한다.
9. RTK를 차량로봇에서 분리하고 노트북 2/착용자 장비로 옮긴다.
10. RTK 착용자에게 장착한 뒤 `flags == 131`을 확인하고 시나리오 촬영을 시작한다.

### Yaw 추정

```
delta_enu = P_end_enu - P_start_enu
yaw_vehicle_from_enu = atan2(delta_east, delta_north)
```

직선 이동 거리가 짧으면 yaw 오차가 커진다. 가능하면 5-10 m 이상 직선 이동하고, 조향 없이 움직인다.

직선 구간이 여러 샘플이면 시작/끝 두 점만 쓰는 것보다, ENU 평면 궤적에 직선을 fitting하거나 PCA로 주방향을 구하는 편이 더 안정적이다.

주의: 앞/뒤 이동만으로는 방향이 180도 뒤집힐 수 있다. `전진 구간`인지 `후진 구간`인지 기록해서 LiDAR 전방 방향과 부호를 맞춘다.

### LiDAR Position 보정

RTK가 측정하는 위치는 안테나 phase center 위치다. 최종 정지 위치에서 안테나 평균 위치를 `P_antenna_enu`라 하면:

```
P_lidar_enu = P_antenna_enu - R_enu_lidar * p_antenna_in_lidar
```

여기서 `p_antenna_in_lidar`는 LiDAR frame에서 본 안테나 위치 offset이다.

### 사람 위치 변환

Ublox 위치는 위도/경도/고도(LLH)로 나오므로, 먼저 기준 원점 주변의 local ENU로 변환한다. 기준 원점은 보통 `P_lidar_enu` 또는 첫 캘리브레이션 위치 근처로 둔다.

```
LLH -> ENU 변환 후 사용
```

RTK 착용자의 위치를 LiDAR frame에서 비교하려면:

```
p_person_lidar = R_lidar_enu * (p_person_enu - P_lidar_enu)
```

사람 detection box 중심과 비교할 때는 RTK 안테나가 사람 몸 중심이 아니라는 점을 기록한다. 헬멧/배낭 상단에 장착하면 LiDAR detection center와 높이/평면 offset이 다를 수 있다. 위치 평가에서는 이 offset을 별도 bias로 보정하거나, 2D ground-plane 위치만 비교한다.

### 사람 속도 변환

Ublox `/fix_velocity`의 선속도는 ENU에 가깝게 해석한다.

```
v_enu = [v_east, v_north, v_up]
```

LiDAR frame에서 평가하려면 yaw 회전을 적용한다.

```
v_lidar = R_lidar_enu * v_enu
```

초기 평가는 좌표계 오차 영향을 줄이기 위해 2D speed부터 비교하는 것을 권장한다.

```
speed_gt_2d = sqrt(vx_gt^2 + vy_gt^2)
speed_pred_2d = sqrt(vx_pred^2 + vy_pred^2)
```

방향까지 평가할 때만 velocity vector를 비교한다.

## RTK 착용 방법

RTK 안테나는 손에 들지 않는 것이 좋다. 손 흔들림이 속도 GT에 섞인다.

권장:

- 헬멧, 모자, 배낭 상단 등 몸에 단단히 고정한다.
- 안테나가 사람 머리/몸에 가려지지 않게 위쪽에 둔다.
- 케이블이 흔들리거나 당겨지지 않게 고정한다.
- 가능하면 작은 금속 ground plane 위에 안테나를 고정한다.

## Run 구성

RTK가 1개뿐이므로 5회 반복 촬영한다.

| Run | RTK 착용자 | 평가 대상 |
| --- | --- | --- |
| 1 | 사람 A | 사람 A track |
| 2 | 사람 B | 사람 B track |
| 3 | 사람 C | 사람 C track |
| 4 | 사람 D | 사람 D track |
| 5 | 사람 E | 사람 E track |

각 run에서 나머지 4명은 detection/tracking 상황을 만드는 객체로 참여하지만, 위치/속도 GT 평가는 RTK 착용자 1명에 대해서만 수행한다.

## Run 시작 체크리스트

1. Livox 고정 설치 확인
2. 노트북 1 Livox topic hz 확인
3. 노트북 2 Ublox 실행
4. NTRIP 실행
5. `/rtcm` 수신 확인
6. `flags == 131` 또는 최소 `flags == 67` 확인
7. 차량로봇에 RTK 설치
8. 조향 없는 직선 주행으로 LiDAR yaw 캘리브레이션 bag 기록
9. 최종 촬영 위치에서 10초 이상 정지해 LiDAR position 평균 기록
10. RTK를 차량로봇에서 분리해 착용자에게 장착
11. `flags == 131` 또는 최소 `flags == 67` 재확인
12. 두 노트북 시간 동기화 확인
13. 두 노트북 rosbag record 시작. Enter 타이밍은 맞추지 않아도 됨
14. 두 bag이 모두 recording 상태인지 확인
15. RTK 착용자 LiDAR 시야 안에서 3초 이상 정지
16. 명확하게 출발 후 시나리오 수행
17. 종료 후 RTK 착용자 LiDAR 시야 안에서 3초 이상 정지
18. rosbag record 종료

## GT 품질 기준

권장 high-quality GT 조건:

```
fix_type == 3
flags == 131
h_acc < 30 mm
v_acc < 50 mm
s_acc < 100 mm/s
```

medium-quality GT:

```
fix_type == 3
flags == 67
h_acc < 200 mm
s_acc < 200 mm/s
```

`flags`가 `1` 또는 `3`인 구간은 RTK fixed/float가 아니므로 위치/속도 GT로 쓰더라도 별도 표시한다.

## 평가 지표

run별로 계산하고, 마지막에 전체 통합 결과를 함께 보고한다.

- 2D position MAE
- 2D position RMSE
- 3D position RMSE
- 2D speed MAE
- 2D speed RMSE
- velocity vector RMSE
- 정지 구간 false velocity
- RTK 상태별 결과: fixed 구간, float 구간, 전체 구간

권장 보고 방식:

```
Run별 결과 + 전체 frame-weighted 평균
```

단순히 5명 평균 하나만 보고하면 run별 환경 차이가 사라지므로, run별 표를 반드시 같이 둔다.

## 후처리 개요

1. Livox bag과 RTK bag을 로드한다.
2. LiDAR pose 캘리브레이션 bag에서 yaw와 position을 계산한다.
3. 시작 시각이 아니라 공통 motion event 기준으로 시간 offset을 보정한다.
4. RTK 위치/속도를 ENU에서 LiDAR frame으로 변환한다.
5. LiDAR tracking 결과 중 RTK 착용자 track을 매칭한다.
6. RTK 상태 필터를 적용한다.
7. 위치 오차와 속도 오차를 계산한다.
8. run별/전체 결과를 저장한다.

## RViz GT 시각 확인

시간 offset과 LiDAR/RTK alignment가 계산된 뒤에는 RTK `/fix`, `/fix_velocity`를 LiDAR frame으로 변환해 RViz marker로 확인한다.

실행:

```bash
source install/setup.bash
ros2 launch rtk_livox_dataset_tools rviz_gt_check.launch.py \
  calib:=calibration/run_01_lidar_rtk_alignment.yaml \
  time_offset_sec:=<time_offset_livox_minus_rtk_sec> \
  livox_frame:=livox_frame
```

별도 터미널에서 bag 재생:

```bash
ros2 bag play bags/run_01_livox bags/run_01_rtk --clock
```

RViz 설정:

```
Fixed Frame: livox_frame
PointCloud2: /livox/lidar 또는 실제 point cloud topic
MarkerArray: /rtk_gt/livox/markers
PointStamped: /rtk_gt/livox/point
```

표시 의미:

```
원기둥: RTK antenna 위치를 LiDAR frame으로 변환한 GT 위치
화살표: RTK 속도 방향과 크기. 속도가 클수록 화살표가 길고 두꺼워짐
```

이 시각화에서 확인할 것:

- 정지 구간에서 원기둥이 거의 움직이지 않는지
- 출발 순간의 화살표 방향이 실제 이동 방향과 맞는지
- point cloud 속 사람 위치와 원기둥 위치가 큰 bias 없이 따라가는지
- 속도 화살표가 종료 정지 구간에서 거의 사라지는지

## 파일 이름 규칙

추천:

```
bags/
  run_01_lidar_pose_calib/
  run_01_livox/
  run_01_rtk/
  run_02_lidar_pose_calib/
  run_02_livox/
  run_02_rtk/

calibration/
  run_01_lidar_rtk_alignment.yaml
  run_02_lidar_rtk_alignment.yaml

reports/
  run_01_metrics.csv
  summary_metrics.csv
```

캘리브레이션 YAML에는 최소한 아래 값을 저장한다.

```yaml
run_id: run_01
lidar_position_enu:
east:0.0
north:0.0
up:0.0
p_antenna_in_lidar:
x:0.0
y:0.0
z:0.0
antenna_stationary_mean_enu:
east:0.0
north:0.0
up:0.0
yaw_lidar_from_enu_rad:0.0
calibration_motion:
direction: forward
distance_m:0.0
time_offset_livox_minus_rtk_sec:0.0
time_offset_method: event_start_end_and_speed_correlation
```

## 현장 메모

- RTK fixed가 `131`에서 `67`로 가끔 떨어지는 것은 흔하다.
- `/rtcm`이 계속 들어오는데 `131`이 깨지면 대개 위성 환경, 멀티패스, 안테나 흔들림 문제다.
- `/rtcm` 자체가 끊기면 NTRIP 또는 네트워크 문제다.
- 정지 상태에서도 `/fix_velocity`는 0이 아니라 몇 cm/s 흔들릴 수 있다.
- `s_acc`는 mm/s 단위다. 예: `s_acc: 66`은 약 `0.066 m/s`이다.
