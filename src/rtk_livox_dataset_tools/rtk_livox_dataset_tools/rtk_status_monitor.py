import argparse
import csv
import os
from collections import deque
from datetime import datetime

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT

from rtk_livox_dataset_tools.rtk_quality import (
    classify_rtk_quality,
    decode_navpvt_flags,
)

try:
    from rtcm_msgs.msg import Message as RtcmMessage
except ImportError:
    RtcmMessage = None


class RtkStatusMonitor(Node):
    def __init__(self, args):
        super().__init__("rtk_status_monitor")
        self.args = args
        self.latest_navpvt = None
        self.latest_fix = None
        self.latest_velocity = None
        self.rtcm_times = deque(maxlen=200)

        self.create_subscription(NavPVT, args.navpvt_topic, self._on_navpvt, 10)
        self.create_subscription(NavSatFix, args.fix_topic, self._on_fix, 10)
        self.create_subscription(
            TwistWithCovarianceStamped,
            args.fix_velocity_topic,
            self._on_velocity,
            10,
        )
        if RtcmMessage is not None:
            self.create_subscription(RtcmMessage, args.rtcm_topic, self._on_rtcm, 10)
        else:
            self.get_logger().warning("rtcm_msgs is not importable; /rtcm will not be monitored")

        os.makedirs(args.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(args.output_dir, "rtk_status_%s.csv" % timestamp)
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "stamp_sec",
                "status",
                "quality",
                "fix_type",
                "flags",
                "num_sv",
                "h_acc_mm",
                "v_acc_mm",
                "s_acc_mm_s",
                "rtcm_hz",
                "gnss_fix_ok",
                "diff_soln",
                "carrier_float",
                "carrier_fixed",
            ]
        )

        self.get_logger().info("Writing RTK status CSV: %s" % self.csv_path)
        self.timer = self.create_timer(args.period_sec, self._on_timer)

    def destroy_node(self):
        if hasattr(self, "csv_file") and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()

    def _on_navpvt(self, msg):
        self.latest_navpvt = msg

    def _on_fix(self, msg):
        self.latest_fix = msg

    def _on_velocity(self, msg):
        self.latest_velocity = msg

    def _on_rtcm(self, _msg):
        self.rtcm_times.append(self.get_clock().now().nanoseconds * 1.0e-9)

    def _rtcm_hz(self, now_sec):
        while self.rtcm_times and now_sec - self.rtcm_times[0] > self.args.rtcm_window_sec:
            self.rtcm_times.popleft()
        if len(self.rtcm_times) < 2:
            return 0.0
        duration = self.rtcm_times[-1] - self.rtcm_times[0]
        if duration <= 0.0:
            return 0.0
        return (len(self.rtcm_times) - 1) / duration

    def _status(self, quality, rtcm_hz):
        if quality == "high" and rtcm_hz > 0.0:
            return "OK"
        if quality == "medium" and rtcm_hz > 0.0:
            return "WARN"
        return "FAIL"

    def _on_timer(self):
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        rtcm_hz = self._rtcm_hz(now_sec)

        if self.latest_navpvt is None:
            self.get_logger().warning("Waiting for %s" % self.args.navpvt_topic)
            return

        msg = self.latest_navpvt
        flags = decode_navpvt_flags(msg.flags)
        quality = classify_rtk_quality(
            msg.fix_type,
            msg.flags,
            msg.h_acc,
            msg.v_acc,
            msg.s_acc,
        )
        status = self._status(quality, rtcm_hz)

        self.csv_writer.writerow(
            [
                "%.3f" % now_sec,
                status,
                quality,
                msg.fix_type,
                msg.flags,
                msg.num_sv,
                msg.h_acc,
                msg.v_acc,
                msg.s_acc,
                "%.3f" % rtcm_hz,
                int(flags["gnss_fix_ok"]),
                int(flags["diff_soln"]),
                int(flags["carrier_float"]),
                int(flags["carrier_fixed"]),
            ]
        )
        self.csv_file.flush()

        self.get_logger().info(
            "%s quality=%s fix_type=%d flags=%d sv=%d h_acc=%dmm v_acc=%dmm s_acc=%dmm/s rtcm=%.2fHz"
            % (
                status,
                quality,
                msg.fix_type,
                msg.flags,
                msg.num_sv,
                msg.h_acc,
                msg.v_acc,
                msg.s_acc,
                rtcm_hz,
            )
        )


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--navpvt-topic", default="/ublox_gps_node/navpvt")
    parser.add_argument("--fix-topic", default="/ublox_gps_node/fix")
    parser.add_argument("--fix-velocity-topic", default="/ublox_gps_node/fix_velocity")
    parser.add_argument("--rtcm-topic", default="/rtcm")
    parser.add_argument("--output-dir", default="logs")
    parser.add_argument("--period-sec", type=float, default=1.0)
    parser.add_argument("--rtcm-window-sec", type=float, default=5.0)
    return parser.parse_known_args(argv)


def main(argv=None):
    args, ros_args = parse_args(argv)
    rclpy.init(args=ros_args)
    node = RtkStatusMonitor(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
