import argparse
import math
import os
import time
from collections import Counter
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ublox_msgs.msg import NavPVT

from rtk_livox_dataset_tools.geo import navpvt_to_measurement
from rtk_livox_dataset_tools.lidar_pose_calibrator import (
    NavSample,
    compute_calibration,
    default_output_path,
    load_antenna_offset,
    parse_origin_llh,
    write_yaml,
)
from rtk_livox_dataset_tools.rtk_quality import classify_rtk_quality


class OnlineLidarPoseCalibrator(Node):
    def __init__(self, args):
        super().__init__("online_lidar_pose_calibrator")
        self.args = args
        self.samples = []
        self.latest_msg = None
        self.collecting = False
        self.phase = "idle"
        self.phase_base_sec = 0.0
        self.phase_start_sec = None
        self.create_subscription(NavPVT, args.navpvt_topic, self._on_navpvt, 50)
        self.phase_pub = self.create_publisher(String, args.phase_topic, 10)
        self.phase_timer = self.create_timer(args.phase_publish_period_sec, self._publish_phase)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_navpvt(self, msg):
        self.latest_msg = msg
        if not self.collecting:
            return

        fields = navpvt_to_measurement(msg)
        quality = classify_rtk_quality(
            fields["fix_type"],
            fields["flags"],
            fields["h_acc_mm"],
            fields["v_acc_mm"],
            fields["s_acc_mm_s"],
        )
        t_rel = self.phase_base_sec + max(0.0, self._now_sec() - self.phase_start_sec)
        self.samples.append(NavSample(t_rel=t_rel, quality=quality, **fields))

    def _set_phase(self, phase):
        self.phase = phase
        self._publish_phase()

    def _publish_phase(self):
        msg = String()
        msg.data = self.phase
        self.phase_pub.publish(msg)

    def wait_for_navpvt(self):
        print("Waiting for %s ..." % self.args.navpvt_topic)
        while rclpy.ok() and self.latest_msg is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        msg = self.latest_msg
        print(
            "NavPVT ready: fix_type=%d flags=%d h_acc=%dmm s_acc=%dmm/s"
            % (msg.fix_type, msg.flags, msg.h_acc, msg.s_acc)
        )

    def collect_phase(self, name, instruction, duration_sec, phase_base_sec):
        self._set_phase("idle")
        if self.args.prompt:
            input("\n%s\nPress Enter when you are ready." % instruction)
        else:
            print("\n%s" % instruction)

        for remaining in range(int(math.ceil(self.args.countdown_sec)), 0, -1):
            print("Start in %d ..." % remaining)
            time.sleep(1.0)

        print("GO: %s for %.1f seconds" % (name, duration_sec))
        start_count = len(self.samples)
        self.phase_base_sec = phase_base_sec
        self.phase_start_sec = self._now_sec()
        self.collecting = True
        self._set_phase(name.lower())
        end_sec = self.phase_start_sec + duration_sec
        next_tick = self.phase_start_sec

        while rclpy.ok() and self._now_sec() < end_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            now_sec = self._now_sec()
            if now_sec >= next_tick:
                elapsed = min(duration_sec, now_sec - self.phase_start_sec)
                print("  %s %.1f / %.1f sec" % (name, elapsed, duration_sec))
                next_tick = now_sec + self.args.status_period_sec

        self.collecting = False
        self._set_phase("idle")
        rclpy.spin_once(self, timeout_sec=0.1)
        phase_samples = self.samples[start_count:]
        print("STOP: %s complete, collected %d samples." % (name, len(phase_samples)))
        self.print_phase_summary(name, phase_samples)
        return [phase_base_sec, phase_base_sec + duration_sec]

    def print_phase_summary(self, name, samples):
        if not samples:
            print("  %s summary: no samples" % name)
            return
        speeds = [math.hypot(sample.vel_e_m_s, sample.vel_n_m_s) for sample in samples]
        qualities = Counter(sample.quality for sample in samples)
        print(
            "  %s speed m/s: min=%.3f mean=%.3f max=%.3f"
            % (name, min(speeds), sum(speeds) / len(speeds), max(speeds))
        )
        print(
            "  %s quality: high=%d medium=%d low=%d"
            % (
                name,
                qualities.get("high", 0),
                qualities.get("medium", 0),
                qualities.get("low", 0),
            )
        )


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id")
    parser.add_argument("--navpvt-topic", default="/ublox_gps_node/navpvt")
    parser.add_argument("--antenna-offset", default="config/antenna_in_lidar.yaml")
    parser.add_argument("--output")
    parser.add_argument("--forward-duration", type=float, default=4.0)
    parser.add_argument("--backward-duration", type=float, default=4.0)
    parser.add_argument("--stationary-duration", type=float, default=10.0)
    parser.add_argument("--settle-duration", type=float, default=1.0)
    parser.add_argument("--countdown-sec", type=float, default=3.0)
    parser.add_argument("--status-period-sec", type=float, default=1.0)
    parser.add_argument("--phase-topic", default="/rtk_livox_calibration/phase")
    parser.add_argument("--phase-publish-period-sec", type=float, default=0.2)
    parser.add_argument("--forward-speed-min", type=float, default=0.05)
    parser.add_argument("--stationary-speed-max", type=float, default=0.05)
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--origin-llh", type=parse_origin_llh)
    parser.add_argument("--allow-low-quality", action="store_true")
    parser.add_argument("--no-prompt", dest="prompt", action="store_false")
    parser.set_defaults(prompt=True)
    return parser.parse_known_args(argv)


def _samples_in_window(samples, window):
    return [sample for sample in samples if window[0] <= sample.t_rel <= window[1]]


def _speed_summary(samples):
    if not samples:
        return "n=0"
    speeds = [math.hypot(sample.vel_e_m_s, sample.vel_n_m_s) for sample in samples]
    return "n=%d min=%.3f mean=%.3f max=%.3f m/s" % (
        len(samples),
        min(speeds),
        sum(speeds) / len(speeds),
        max(speeds),
    )


def main(argv=None):
    args, ros_args = parse_args(argv)
    if args.run_id is None:
        args.run_id = datetime.now().strftime("run_%Y%m%d_%H%M%S")
    if args.output is None:
        args.output = default_output_path(args.run_id)
    args.output = os.path.abspath(args.output)

    antenna_offset, antenna_warning = load_antenna_offset(args.antenna_offset)

    rclpy.init(args=ros_args)
    node = OnlineLidarPoseCalibrator(args)
    try:
        node.wait_for_navpvt()

        base = 0.0
        forward_window = node.collect_phase(
            "FORWARD",
            "Move straight FORWARD in the LiDAR +x direction.",
            args.forward_duration,
            base,
        )
        base = forward_window[1] + args.settle_duration
        backward_window = node.collect_phase(
            "BACKWARD",
            "Move straight BACKWARD along the same line.",
            args.backward_duration,
            base,
        )
        base = backward_window[1] + args.settle_duration
        stationary_window = node.collect_phase(
            "STATIONARY",
            "Keep the platform completely STATIONARY.",
            args.stationary_duration,
            base,
        )

        try:
            result = compute_calibration(
                samples=node.samples,
                run_id=args.run_id,
                navpvt_topic=args.navpvt_topic,
                forward_window=forward_window,
                backward_window=backward_window,
                stationary_window=stationary_window,
                antenna_offset=antenna_offset,
                origin_llh=args.origin_llh,
                forward_speed_min=args.forward_speed_min,
                stationary_speed_max=args.stationary_speed_max,
                min_samples=args.min_samples,
                allow_low_quality=args.allow_low_quality,
            )
        except RuntimeError as exc:
            print("\nCalibration failed: %s" % exc)
            print("Forward speed summary: %s" % _speed_summary(_samples_in_window(node.samples, forward_window)))
            print("Backward speed summary: %s" % _speed_summary(_samples_in_window(node.samples, backward_window)))
            print("Stationary speed summary: %s" % _speed_summary(_samples_in_window(node.samples, stationary_window)))
            print(
                "Try moving faster/longer, or rerun with --forward-speed-min 0.02 "
                "if NavPVT velocity is noisy but nonzero."
            )
            raise
        if antenna_warning:
            result["qc"]["warnings"].insert(0, antenna_warning)

        write_yaml(args.output, result)
        qc = result["qc"]
        print("\nWrote LiDAR pose calibration: %s" % args.output)
        print("yaw_enu_lidar_deg: %.3f" % result["yaw_enu_lidar_deg"])
        print("lidar_position_enu: [%.3f, %.3f, %.3f]" % tuple(result["lidar_position_enu"]))
        print(
            "qc: fwd_bwd_angle_diff=%s fwd_vs_bwd_plus_180_diff=%s yaw_std=%.3f quality_used=%s"
            % (
                "%.3f" % qc["fwd_bwd_angle_diff_deg"]
                if qc["fwd_bwd_angle_diff_deg"] is not None
                else "n/a",
                (
                    "%.3f" % qc["fwd_vs_backward_plus_180_diff_deg"]
                    if qc.get("fwd_vs_backward_plus_180_diff_deg") is not None
                    else "n/a"
                ),
                qc["yaw_std_deg"],
                qc["quality_used"],
            )
        )
        if qc["warnings"]:
            print("warnings:")
            for warning in qc["warnings"]:
                print("  - %s" % warning)
    finally:
        node.destroy_node()
        rclpy.shutdown()
