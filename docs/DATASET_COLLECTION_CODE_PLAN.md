# DATASET_COLLECTION_CODE_PLAN

작성자: 김동희
상태: 초안
최종 편집 일시: 2026년 7월 2일 오후 4:13
생성 일시: 2026년 7월 2일 오후 4:13
상위 항목: 실증 데이터셋 수집 (https://app.notion.com/p/38e2d863200d80e99270c52b23c908f4?pvs=21)

# 데이터셋 수집 코드/노드 설계 초안

이 문서는 `DATASET_COLLECTION_SETUP.md`의 현장 프로토콜을 실제 코드로 구현하기 위한 개발 명세다. 추후 Codex에 코드 작성을 요청할 때 이 문서를 기준으로 패키지/노드를 구현한다.
+ 
## 전체 파이프라인 순서

1. 시스템 사전 점검
2. 시간 동기화 상태 기록
3. RTK/NTRIP 상태 모니터링
4. LiDAR pose 캘리브레이션 수집
5. LiDAR pose 계산
6. 시나리오 rosbag 수집
7. RTK 착용자 GT 변환
8. LiDAR track과 RTK GT 시간 정렬
9. RTK 착용자 track 매칭
10. 위치/속도 평가
11. 리포트 생성

## 패키지 구성 제안

새 패키지 이름 예시:

```
rtk_livox_dataset_tools
```

권장 구조:

```
rtk_livox_dataset_tools/
  package.xml
  setup.py
  rtk_livox_dataset_tools/
    __init__.py
    rtk_status_monitor.py
    lidar_pose_calibrator.py
    gt_transformer.py
    time_offset_estimator.py
    rtk_livox_visualizer.py
    track_matcher.py
    metrics_evaluator.py
    report_writer.py
    geo.py
    transforms.py
  launch/
    rtk_status_monitor.launch.py
    lidar_pose_calibration.launch.py
    scenario_record.launch.py
    livox_collection.launch.py
    rtk_collection.launch.py
    rviz_gt_check.launch.py
    realsense_collection.launch.py
    realsense_sync_check.launch.py
  config/
    dataset_collection.yaml
```

## 공통 데이터 정의

### RTK 상태

입력 토픽:

```
/ublox_gps_node/navpvt
/ublox_gps_node/fix
/ublox_gps_node/fix_velocity
/rtcm
```

핵심 조건:

```
high_quality:
  fix_type == 3
  flags == 131
  h_acc < 30 mm
  v_acc < 50 mm
  s_acc < 100 mm/s

medium_quality:
  fix_type == 3
  flags == 67
  h_acc < 200 mm
  s_acc < 200 mm/s
```

### 좌표계

RTK 원본:

```
LLH: latitude, longitude, height
Velocity ENU: east, north, up
```

LiDAR 평가 좌표계:

```
LiDAR frame:
  x: LiDAR forward
  y: LiDAR left
  z: LiDAR up
```

변환:

```
LLH -> ENU
p_person_lidar = R_lidar_enu * (p_person_enu - P_lidar_enu)
v_person_lidar = R_lidar_enu * v_person_enu
```

## Protocol 0: 시스템 사전 점검

목적:

- 두 노트북에서 필수 토픽이 정상 publish되는지 확인한다.
- RTK fixed 여부와 `/rtcm` 수신 여부를 촬영 전에 확인한다.

실행 코드 후보:

```bash
ros2 run rtk_livox_dataset_tools rtk_status_monitor
```

입력:

```
/ublox_gps_node/navpvt
/ublox_gps_node/fix
/ublox_gps_node/fix_velocity
/rtcm
```

출력:

```
console summary
logs/rtk_status_YYYYMMDD_HHMMSS.csv
```

Pseudo code:

```python
class RtkStatusMonitor(Node):
    subscribe("/ublox_gps_node/navpvt", NavPVT, on_navpvt)
    subscribe("/ublox_gps_node/fix", NavSatFix, on_fix)
    subscribe("/ublox_gps_node/fix_velocity", TwistWithCovarianceStamped, on_velocity)
    subscribe("/rtcm", rtcm_msgs.Message, on_rtcm)

    every 1.0 sec:
        compute rtcm_hz
        decode navpvt.flags:
            is_fix_ok = flags & 1
            is_diff = flags & 2
            is_float = flags & 64
            is_fixed = flags & 128
        print:
            fix_type, flags, state, num_sv, h_acc_mm, v_acc_mm, s_acc_mm_s, rtcm_hz
        append csv row
```

판정:

```
OK:
  /rtcm hz > 0
  flags == 131
  h_acc < 30
  s_acc < 100

WARN:
  flags == 67

FAIL:
  fix_type != 3
  flags not in [67, 131]
  /rtcm missing
```

## Protocol 1: 시간 동기화 상태 기록

목적:

- 학교 Wi-Fi/chrony 상태를 촬영 메타데이터로 남긴다.
- 후처리에서 시간 offset 추정 여부를 판단한다.

실행 코드 후보:

```bash
ros2 run rtk_livox_dataset_tools time_sync_logger \
  --role livox_or_rtk \
  --peer-ip <OTHER_LAPTOP_IP> \
  --run-id run_01
```

출력:

```
metadata/run_01_time_sync.yaml
```

Pseudo code:

```python
def collect_time_sync(role, peer_ip):
    local_ips = run("hostname -I")
    ping_result = run("ping -c 5 peer_ip")
    chronyc_tracking = run("chronyc tracking")
    chronyc_sources = run("chronyc sources -v")

    write_yaml({
        "role": role,
        "peer_ip": peer_ip,
        "local_ips": local_ips,
        "ping": parse_ping(ping_result),
        "chronyc_tracking_raw": chronyc_tracking,
        "chronyc_sources_raw": chronyc_sources,
        "timestamp_system": now()
    })
```

후처리 메모:

```
chrony offset이 작아도 시작/정지 이벤트 기반 offset 보정은 수행한다.
```

## Protocol 2: LiDAR Pose 캘리브레이션 수집

목적:

- 차량로봇에 RTK를 설치하고 직선 주행/정지 구간을 기록한다.
- LiDAR yaw와 최종 LiDAR position 계산에 필요한 RTK trajectory를 저장한다.

실행 코드 후보:

```bash
ros2 launch rtk_livox_dataset_tools lidar_pose_calibration.launch.py run_id:=run_01
```

실제 내부 실행:

```bash
ros2 bag record \
  /ublox_gps_node/navpvt \
  /ublox_gps_node/fix \
  /ublox_gps_node/fix_velocity \
  /rtcm \
  -o bags/run_01_lidar_pose_calib
```

보조 노드:

```bash
ros2 run rtk_livox_dataset_tools calibration_event_marker --run-id run_01
```

이벤트:

```
START_FORWARD
END_FORWARD
START_STATIONARY
END_STATIONARY
```

Pseudo code:

```python
class CalibrationEventMarker(Node):
    publisher("/dataset_events", DatasetEvent)

    wait_keyboard_input()
    if key == "f":
        publish_event("START_FORWARD")
    if key == "g":
        publish_event("END_FORWARD")
    if key == "s":
        publish_event("START_STATIONARY")
    if key == "e":
        publish_event("END_STATIONARY")
```

대체:

```
이벤트 노드 구현 전에는 현장 노트에 각 이벤트 시각을 직접 기록한다.
```

## Protocol 3: LiDAR Pose 계산

목적:

- 캘리브레이션 bag에서 RTK 궤적을 읽어 LiDAR yaw와 position을 계산한다.
- 결과를 run별 calibration YAML로 저장한다.

실행 코드 후보:

```bash
ros2 run rtk_livox_dataset_tools lidar_pose_calibrator \
  --bag bags/run_01_lidar_pose_calib \
  --run-id run_01 \
  --antenna-offset config/antenna_in_lidar.yaml \
  --output calibration/run_01_lidar_rtk_alignment.yaml
```

입력:

```
bags/run_01_lidar_pose_calib
config/antenna_in_lidar.yaml
event timestamps or auto-selected time windows
```

출력:

```
calibration/run_01_lidar_rtk_alignment.yaml
```

Pseudo code:

```python
def calibrate_lidar_pose(bag, antenna_offset):
    navpvt_samples = read_navpvt(bag)
    valid_samples = filter(samples, flags == 131 and fix_type == 3)

    forward_samples = select_time_window(valid_samples, START_FORWARD, END_FORWARD)
    stationary_samples = select_time_window(valid_samples, START_STATIONARY, END_STATIONARY)

    enu_forward = llh_to_enu(forward_samples, origin_llh)
    enu_stationary = llh_to_enu(stationary_samples, origin_llh)

    # yaw from straight driving
    direction = fit_line_pca(enu_forward[:, east_north])
    if motion_direction == "backward":
        direction = -direction
    yaw_lidar_from_enu = direction_to_yaw(direction)

    R_enu_lidar = rotation_from_yaw(yaw_lidar_from_enu)

    P_antenna_enu = mean(enu_stationary)
    P_lidar_enu = P_antenna_enu - R_enu_lidar @ p_antenna_in_lidar

    write_yaml({
        "run_id": run_id,
        "origin_llh": origin_llh,
        "lidar_position_enu": P_lidar_enu,
        "antenna_stationary_mean_enu": P_antenna_enu,
        "p_antenna_in_lidar": p_antenna_in_lidar,
        "yaw_lidar_from_enu_rad": yaw_lidar_from_enu,
        "rtk_quality_summary": summarize(valid_samples)
    })
```

주의:

```
앞/뒤 직선 주행만으로는 yaw가 180도 뒤집힐 수 있으므로 motion_direction을 기록한다.
```

## Protocol 4: 시나리오 촬영 수집

목적:

- 노트북 1은 Livox point cloud와 tracking 결과를 기록한다.
- 노트북 2는 RTK 착용자 위치/속도 GT를 기록한다.
- 두 bag은 시작 시간이 달라도 후처리에서 offset 보정한다.
- rosbag 시작 시각은 동기화 기준으로 쓰지 않고, 두 bag이 모두 recording 중일 때 수행한 공통 motion event를 기준으로 맞춘다.

노트북 1 실행:

```bash
ros2 launch rtk_livox_dataset_tools livox_collection.launch.py \
  bag_uri:=bags/run_01_livox \
  record_topics:="/livox/lidar /tracking/objects /tracking/tracks"
```

노트북 2 실행:

```bash
ros2 launch rtk_livox_dataset_tools rtk_collection.launch.py \
  bag_uri:=bags/run_01_rtk \
  ntrip_mountpoint:=<MOUNTPOINT> \
  ntrip_username:=<NTRIP_USER> \
  ntrip_password:=<NTRIP_PASSWORD>
```

RTK 노트북은 가방에 넣고 닫아둘 수 있으므로, 실제 현장에서는 노트북 1에서 SSH로 위 명령을 원격 실행한다.

```bash
ssh <RTK_USER>@<RTK_LAPTOP_IP> \
  'tmux new-session -d -s rtk_run "cd ~/sensor_project_dataset_2026_ws && source install/setup.bash && ros2 launch rtk_livox_dataset_tools rtk_collection.launch.py bag_uri:=bags/run_01_rtk ntrip_mountpoint:=<MOUNTPOINT> ntrip_username:=<NTRIP_USER> ntrip_password:=<NTRIP_PASSWORD>"'
```

보조 이벤트:

```
START_STILL
START_SCENARIO
END_SCENARIO
END_STILL
```

Pseudo code for launch:

```python
def generate_launch_description():
    declare run_id
    start rtk_status_monitor
    start event_marker
    optionally start rosbag record via ExecuteProcess
```

수집 규칙:

```
1. 노트북 1/2 rosbag record 시작. Enter 타이밍은 달라도 됨
2. 두 bag이 모두 recording 상태인지 확인
3. RTK 착용자 LiDAR 시야 안에서 3초 이상 정지
4. 명확하게 출발
5. 시나리오 수행
6. 시나리오 종료
7. RTK 착용자 LiDAR 시야 안에서 3초 이상 정지
8. 두 bag record 종료
```

## Protocol 5: RTK GT 변환

목적:

- RTK 착용자 LLH 위치와 ENU 속도를 LiDAR frame으로 변환한다.
- RTK quality flag를 함께 저장한다.

실행 코드 후보:

```bash
ros2 run rtk_livox_dataset_tools gt_transformer \
  --rtk-bag bags/run_01_rtk \
  --calib calibration/run_01_lidar_rtk_alignment.yaml \
  --output gt/run_01_rtk_gt.csv
```

출력 CSV 예시:

```
stamp_sec,p_lidar_x,p_lidar_y,p_lidar_z,v_lidar_x,v_lidar_y,v_lidar_z,speed_2d,fix_type,flags,h_acc_mm,v_acc_mm,s_acc_mm_s,quality
```

Pseudo code:

```python
def transform_rtk_gt(rtk_bag, calib):
    navpvt = read_navpvt(rtk_bag)
    fix_velocity = read_fix_velocity(rtk_bag)
    sync navpvt and velocity by timestamp

    for sample in synced:
        p_enu = llh_to_enu(sample.lat, sample.lon, sample.height, calib.origin_llh)
        v_enu = [vel_e, vel_n, vel_u]

        p_lidar = R_lidar_enu @ (p_enu - P_lidar_enu)
        v_lidar = R_lidar_enu @ v_enu

        quality = classify_rtk_quality(sample.fix_type, sample.flags, sample.h_acc, sample.v_acc, sample.s_acc)
        write row
```

## Protocol 6: 시간 offset 추정

목적:

- Livox bag과 RTK bag의 time offset을 추정한다.
- chrony가 완벽하지 않거나 bag 시작 시간이 달라도 후처리에서 맞춘다.
- offset 부호 정의를 고정해 이후 GT interpolation과 평가에서 혼동을 막는다.

실행 코드 후보:

```bash
ros2 run rtk_livox_dataset_tools time_offset_estimator \
  --livox-tracks tracks/run_01_tracks.csv \
  --rtk-gt gt/run_01_rtk_gt.csv \
  --output calibration/run_01_time_offset.yaml
```

입력:

```
LiDAR tracking speed over time
RTK speed over time
known still/start/end event windows
```

출력:

```yaml
time_offset_livox_minus_rtk_sec:0.0
method: event_start_end_and_speed_correlation
start_event_offset_sec:0.0
end_event_offset_sec:0.0
start_end_offset_diff_sec:0.0
confidence:0.0
```

부호 정의:

```
dt = time_offset_livox_minus_rtk_sec = t_livox_event - t_rtk_event
t_rtk_on_livox_clock = t_rtk + dt
t_rtk_query_for_livox_sample = t_livox - dt
```

Pseudo code:

```python
def estimate_time_offset(livox_track, rtk_gt):
    livox_speed = norm_2d(livox_track.velocity)
    rtk_speed = norm_2d(rtk_gt.velocity)

    start_offset = detect_start_event(livox_speed) - detect_start_event(rtk_speed)
    end_offset = detect_end_event(livox_speed) - detect_end_event(rtk_speed)

    candidate_offsets = np.arange(-5.0, 5.0, 0.01)
    for offset in candidate_offsets:
        rtk_interp = interpolate(rtk_speed, livox_time - offset)
        score[offset] = correlation(livox_speed, rtk_interp)

    best_offset = argmax(score)
    confidence = compare_event_offsets_and_correlation(start_offset, end_offset, best_offset)
    return best_offset, confidence
```

대체 방법:

```
이벤트 marker가 있으면 START_SCENARIO 시각 차이를 직접 offset으로 사용한다.
LiDAR track이 아직 없으면 point cloud에서 detection/tracking을 먼저 수행한 뒤 offset을 추정한다.
```

## Protocol 7: RTK 착용자 track 매칭

목적:

- 5명 중 RTK 착용자에 해당하는 LiDAR track ID를 찾는다.
- 위치 GT와 가장 잘 맞는 track을 선택한다.

실행 코드 후보:

```bash
ros2 run rtk_livox_dataset_tools track_matcher \
  --tracks tracks/run_01_tracks.csv \
  --rtk-gt gt/run_01_rtk_gt.csv \
  --time-offset calibration/run_01_time_offset.yaml \
  --output matches/run_01_match.yaml
```

Pseudo code:

```python
def match_rtk_track(tracks, rtk_gt, time_offset):
    for track_id in unique(tracks.track_id):
        track = tracks[track_id]
        rtk_interp = interpolate_rtk_to_track_time(rtk_gt, track.time - time_offset)

        position_error = mean_norm_2d(track.position_xy - rtk_interp.position_xy)
        speed_error = mean_abs(track.speed_2d - rtk_interp.speed_2d)
        continuity_score = track_duration_overlap(track, rtk_gt)

        score = position_error + 0.5 * speed_error - 0.1 * continuity_score
        collect score

    best_track = min_score_track
    write match yaml
```

출력:

```yaml
run_id: run_01
rtk_subject: person_A
matched_track_id:12
mean_position_error_m:0.21
mean_speed_error_mps:0.08
```

수동 보정:

```
자동 매칭이 틀리면 사람이 track_id를 수동 지정할 수 있게 한다.
```

## Protocol 8: 위치/속도 평가

목적:

- matched track과 RTK GT를 비교해 위치/속도 오차를 계산한다.
- fixed/float 구간별 결과를 분리한다.

실행 코드 후보:

```bash
ros2 run rtk_livox_dataset_tools metrics_evaluator \
  --tracks tracks/run_01_tracks.csv \
  --rtk-gt gt/run_01_rtk_gt.csv \
  --match matches/run_01_match.yaml \
  --time-offset calibration/run_01_time_offset.yaml \
  --output reports/run_01_metrics.csv
```

Pseudo code:

```python
def evaluate_metrics(track, rtk_gt, offset):
    for track_sample in track:
        gt = interpolate(rtk_gt, track_sample.time - offset)
        if gt.quality not in ["high", "medium"]:
            continue

        pos_err_2d = norm(track_sample.xy - gt.xy)
        pos_err_3d = norm(track_sample.xyz - gt.xyz)
        speed_err = abs(track_sample.speed_2d - gt.speed_2d)
        vel_err = norm(track_sample.velocity_xyz - gt.velocity_xyz)

        append errors with gt.quality

    report:
        position_2d_mae
        position_2d_rmse
        position_3d_rmse
        speed_2d_mae
        speed_2d_rmse
        velocity_vector_rmse
        metrics_by_quality
```

권장 필터:

```
high-quality only:
  quality == high

analysis:
  high + medium

exclude:
  no fix, normal GNSS, missing RTK, s_acc too high
```

## Protocol 9: 전체 리포트 생성

목적:

- 5개 run 결과를 하나로 모아 요약한다.
- run별 결과와 전체 frame-weighted 평균을 함께 출력한다.

실행 코드 후보:

```bash
ros2 run rtk_livox_dataset_tools report_writer \
  --metrics reports/run_01_metrics.csv reports/run_02_metrics.csv reports/run_03_metrics.csv reports/run_04_metrics.csv reports/run_05_metrics.csv \
  --output reports/summary_metrics.md
```

Pseudo code:

```python
def write_summary(metrics_files):
    all_rows = load_all(metrics_files)
    by_run = groupby_run(all_rows)
    by_quality = groupby_quality(all_rows)

    write markdown tables:
        per-run metrics
        overall frame-weighted metrics
        high-quality-only metrics
        float-vs-fixed metrics
        dropped sample counts
```

출력:

```
reports/summary_metrics.md
reports/summary_metrics.csv
```

## Protocol 10: RViz GT 시각 검수

목적:

- 후처리로 얻은 LiDAR/RTK alignment와 time offset이 실제 point cloud 위에서 자연스럽게 맞는지 확인한다.
- `/ublox_gps_node/fix`, `/ublox_gps_node/fix_velocity`를 LiDAR frame으로 변환해 위치 원기둥과 속도 화살표 marker로 표시한다.

실행 코드 후보:

```bash
ros2 launch rtk_livox_dataset_tools rviz_gt_check.launch.py \
  calib:=calibration/run_01_lidar_rtk_alignment.yaml \
  time_offset_sec:=<time_offset_livox_minus_rtk_sec> \
  livox_frame:=livox_frame
```

별도 터미널:

```bash
ros2 bag play bags/run_01_livox bags/run_01_rtk --clock
```

입력:

```
/ublox_gps_node/fix
/ublox_gps_node/fix_velocity
calibration/run_01_lidar_rtk_alignment.yaml
time_offset_livox_minus_rtk_sec
```

출력:

```
/rtk_gt/livox/point      geometry_msgs/PointStamped
/rtk_gt/livox/velocity   geometry_msgs/TwistStamped
/rtk_gt/livox/markers    visualization_msgs/MarkerArray
```

Marker:

```
CYLINDER: RTK 위치
ARROW: RTK 속도 방향과 크기. speed_2d가 클수록 길고 두꺼움
```

## Protocol 11: RealSense 2대 시간 동기화 리허설

목적:

- Livox/RTK 투입 전에 두 노트북 rosbag timestamp 동기화 절차가 맞는지 RealSense D435 pointcloud 2대로 검증한다.
- 두 bag의 시작 시각이 달라도 timestamp와 visual event offset으로 같은 실제 시간축에 replay할 수 있는지 확인한다.

노트북 1:

```bash
ros2 launch rtk_livox_dataset_tools realsense_collection.launch.py \
  camera_namespace:=rs1 \
  bag_uri:=bags/run_01_rs1 \
  record_topics:="/rs1/camera/depth/color/points"
```

노트북 2:

```bash
ros2 launch rtk_livox_dataset_tools realsense_collection.launch.py \
  camera_namespace:=rs2 \
  bag_uri:=bags/run_01_rs2 \
  record_topics:="/rs2/camera/depth/color/points"
```

RViz 비교:

```bash
ros2 launch rtk_livox_dataset_tools realsense_sync_check.launch.py \
  cloud1_topic:=/rs1/camera/depth/color/points \
  cloud2_topic:=/rs2/camera/depth/color/points
```

동기 replay:

```bash
ros2 run rtk_livox_dataset_tools realsense_sync_play \
  --bag1 bags/run_01_rs1 \
  --bag2 bags/run_01_rs2 \
  --time-offset-bag1-minus-bag2-sec 0.0
```

구현:

```
realsense_collection.launch.py:
  realsense2_camera rs_launch.py 실행
  pointcloud.enable:=true
  camera_namespace로 rs1/rs2 topic 분리
  pointcloud topic rosbag record

realsense_sync_viewer.py:
  PointCloud2 2개 subscribe
  frame_id를 sync_check_world로 통일
  rs2 cloud를 side-by-side offset으로 이동
  /sync_check/rs1/points, /sync_check/rs2/points publish

realsense_sync_play.py:
  metadata.yaml에서 bag 시작 시각 읽기
  dt = t_bag1_event - t_bag2_event 적용
  두 ros2 bag play의 --start-offset 자동 계산
```

## 개발 우선순위

1. `rtk_status_monitor.py`
2. `lidar_pose_calibrator.py`
3. `livox_collection.launch.py`, `rtk_collection.launch.py`
4. `rtk_livox_visualizer.py`
5. `gt_transformer.py`
6. `time_offset_estimator.py`
7. `track_matcher.py`
8. `metrics_evaluator.py`
9. `report_writer.py`

처음 구현할 최소 기능:

```
RTK bag -> calibration YAML
RTK bag + calibration YAML -> LiDAR-frame GT CSV
tracks CSV + GT CSV -> speed RMSE
```

이후 확장:

```
event marker
automatic time offset estimation
automatic track matching
full report generation
```

## 구현 시 주의

- rosbag2 읽기 API를 사용한다.
- LLH -> ENU 변환은 검증된 라이브러리 또는 명확한 WGS84 변환을 사용한다.
- timestamp는 ROS message header stamp를 우선 사용한다.
- RTK status flag와 accuracy threshold를 모든 GT row에 함께 저장한다.
- raw bag은 절대 덮어쓰지 않는다.
- 추후 재현성을 위해 모든 parameter를 YAML로 저장한다.
