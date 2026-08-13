import math

import numpy as np

from rtk_livox_dataset_tools.geo import llh_to_ecef
from rtk_livox_dataset_tools.lidar_pose_calibrator import NavSample, compute_calibration
from rtk_livox_dataset_tools.rtk_quality import HIGH_QUALITY


def _enu_to_llh(east, north, up, origin_llh):
    lat0 = math.radians(origin_llh[0])
    lon0 = math.radians(origin_llh[1])
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)
    ox, oy, oz = llh_to_ecef(origin_llh[0], origin_llh[1], origin_llh[2])
    dx = -sin_lon * east - sin_lat * cos_lon * north + cos_lat * cos_lon * up
    dy = cos_lon * east - sin_lat * sin_lon * north + cos_lat * sin_lon * up
    dz = cos_lat * north + sin_lat * up
    x = ox + dx
    y = oy + dy
    z = oz + dz

    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    b = a * (1.0 - f)
    ep2 = (a * a - b * b) / (b * b)
    p = math.hypot(x, y)
    theta = math.atan2(z * a, p * b)
    lon = math.atan2(y, x)
    lat = math.atan2(
        z + ep2 * b * math.sin(theta) ** 3,
        p - e2 * a * math.cos(theta) ** 3,
    )
    n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), h


def _sample(t_rel, enu, vel_e, vel_n, origin_llh):
    lat, lon, height = _enu_to_llh(enu[0], enu[1], enu[2], origin_llh)
    return NavSample(
        t_rel=t_rel,
        lat_deg=lat,
        lon_deg=lon,
        height_m=height,
        vel_e_m_s=vel_e,
        vel_n_m_s=vel_n,
        vel_u_m_s=0.0,
        h_acc_mm=10,
        v_acc_mm=15,
        s_acc_mm_s=20,
        fix_type=3,
        flags=131,
        quality=HIGH_QUALITY,
    )


def test_compute_calibration_round_trip_velocity_yaw_and_position():
    origin_llh = [37.0, 127.0, 42.0]
    yaw_true = math.radians(70.0)
    p_true = np.array([10.0, -4.0, 1.2])
    direction = np.array([math.cos(yaw_true), math.sin(yaw_true), 0.0])

    samples = []
    for i in range(30):
        t = 4.0 + i * 0.1
        enu = p_true + direction * (0.7 * i)
        samples.append(_sample(t, enu, direction[0] * 1.2, direction[1] * 1.2, origin_llh))
    for i in range(30):
        t = 12.0 + i * 0.1
        enu = p_true + direction * (22.0 - 0.7 * i)
        samples.append(_sample(t, enu, -direction[0] * 1.1, -direction[1] * 1.1, origin_llh))
    for i in range(30):
        t = 20.0 + i * 0.1
        noise = np.array([0.005 * math.sin(i), 0.005 * math.cos(i), 0.002 * math.sin(0.5 * i)])
        samples.append(_sample(t, p_true + noise, 0.0, 0.0, origin_llh))

    result = compute_calibration(
        samples=samples,
        run_id="synthetic",
        navpvt_topic="/ublox_gps_node/navpvt",
        forward_window=[4.0, 8.0],
        backward_window=[12.0, 16.0],
        stationary_window=[20.0, 24.0],
        antenna_offset=[0.0, 0.0, 0.0],
        origin_llh=origin_llh,
        min_samples=20,
    )

    yaw_error_deg = abs(math.degrees(math.atan2(
        math.sin(result["yaw_enu_lidar_rad"] - yaw_true),
        math.cos(result["yaw_enu_lidar_rad"] - yaw_true),
    )))
    position_error = np.linalg.norm(np.array(result["lidar_position_enu"]) - p_true)

    assert yaw_error_deg < 0.5
    assert position_error < 0.03
    assert 175.0 < result["qc"]["fwd_bwd_angle_diff_deg"] < 185.0
    assert result["qc"]["fwd_vs_backward_plus_180_diff_deg"] < 5.0
