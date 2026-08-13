``````

# RTK+GNSS 

```
ros2 launch rtk_livox_dataset_tools rtk_collection.launch.py \
  start_bag:=false \
  start_ublox:=false \
  start_c099_udp:=true \
  c099_configure_rate_hz:=10 \
  start_ntrip:=true \
  ntrip_host:=www.gnssdata.or.kr \
  ntrip_port:=2101 \
  ntrip_mountpoint:=SUWN-RTCM31 \
  ntrip_authenticate:=True \
  ntrip_username:=inall0121@sju.ac.kr \
  ntrip_password:=gnss \
  rtcm_message_package:=rtcm_msgs
```

# Livox lidar TF calibration

```
source install/setup.bash
ros2 run rtk_livox_dataset_tools online_lidar_pose_calibrator \
  --forward-duration 5 \
  --backward-duration 5 \
  --stationary-duration 10 \
  --allow-low-quality \
  --output calibration/run_00_lidar_rtk_alignment.yaml
```

```
ros2 bag record -o bags/calib_run_00_livox_rtk \
  /rtk_livox_calibration/phase \
  /ublox_gps_node/navpvt \.
  /ublox_gps_node/fix_velocity \
  /rtcm \
  /tf \
  /tf_static \
  /diagnostics
```


# Livox lidar launch

```
Avia_rviz2
```

# rtk bag

```
ros2 topic echo /ublox_gps_node/navpvt
```

```
ros2 bag record -o bags/run_01_livox_rtk \
  /livox/lidar \
  /ublox_gps_node/navpvt \
  /ublox_gps_node/fix \
  /ublox_gps_node/fix_velocity \
  /rtcm \
  /tf \
  /tf_static \
  /diagnostics
```
# TEST

```
ros2 launch rtk_livox_dataset_tools rviz_gt_check.launch.py   calib:=calibration/run_01_lidar_rtk_alignment.yaml   time_offset_sec:=0.0   livox_frame:=livox_frame 
show_speed_text:=true
```

```
ros2 bag play bags/run_01_livox_rtk --loop
```

```
ros2 param set /rtk_livox_visualizer p_antenna_in_lidar_x 0.12
ros2 param set /rtk_livox_visualizer p_antenna_in_lidar_y 0.00
ros2 param set /rtk_livox_visualizer p_antenna_in_lidar_z 0.00
```