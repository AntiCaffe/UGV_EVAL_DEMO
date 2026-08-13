import argparse
import os
import signal
import subprocess
import sys
import time

import yaml


def _bag_metadata_path(bag_uri):
    path = os.path.join(bag_uri, "metadata.yaml")
    if not os.path.exists(path):
        raise RuntimeError("metadata.yaml not found: %s" % path)
    return path


def _duration_to_sec(value):
    if isinstance(value, dict):
        if "nanoseconds" in value:
            return float(value["nanoseconds"]) * 1.0e-9
        if "sec" in value or "nanosec" in value:
            return float(value.get("sec", 0)) + float(value.get("nanosec", 0)) * 1.0e-9
    if isinstance(value, (int, float)):
        return float(value) * 1.0e-9
    return 0.0


def _time_to_sec(value):
    if isinstance(value, dict):
        if "nanoseconds_since_epoch" in value:
            return float(value["nanoseconds_since_epoch"]) * 1.0e-9
        if "nanoseconds" in value:
            return float(value["nanoseconds"]) * 1.0e-9
        if "sec" in value or "nanosec" in value:
            return float(value.get("sec", 0)) + float(value.get("nanosec", 0)) * 1.0e-9
    if isinstance(value, (int, float)):
        return float(value) * 1.0e-9
    raise RuntimeError("Unsupported starting_time format: %r" % (value,))


def _read_bag_times(bag_uri):
    with open(_bag_metadata_path(bag_uri), "r") as f:
        data = yaml.safe_load(f) or {}
    info = data.get("rosbag2_bagfile_information", data)
    start = _time_to_sec(info.get("starting_time"))
    duration = _duration_to_sec(info.get("duration"))
    return start, duration


def _start_process(cmd):
    print("+ %s" % " ".join(cmd), flush=True)
    return subprocess.Popen(cmd)


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag1", required=True)
    parser.add_argument("--bag2", required=True)
    parser.add_argument(
        "--time-offset-bag1-minus-bag2-sec",
        type=float,
        default=0.0,
        help="dt = t_bag1_event - t_bag2_event. Use 0 when both laptop clocks are already synchronized.",
    )
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start-padding-sec", type=float, default=0.0)
    parser.add_argument("--clock-from", choices=["none", "bag1", "bag2"], default="none")
    parser.add_argument("--loop", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    bag1_start, bag1_duration = _read_bag_times(args.bag1)
    bag2_start, bag2_duration = _read_bag_times(args.bag2)
    dt = args.time_offset_bag1_minus_bag2_sec

    common_start_in_bag1_clock = max(bag1_start, bag2_start + dt) + args.start_padding_sec
    bag1_offset = max(0.0, common_start_in_bag1_clock - bag1_start)
    bag2_offset = max(0.0, common_start_in_bag1_clock - dt - bag2_start)

    print("bag1_start_sec: %.6f duration_sec: %.3f" % (bag1_start, bag1_duration))
    print("bag2_start_sec: %.6f duration_sec: %.3f" % (bag2_start, bag2_duration))
    print("time_offset_bag1_minus_bag2_sec: %.6f" % dt)
    print("bag1 --start-offset: %.6f" % bag1_offset)
    print("bag2 --start-offset: %.6f" % bag2_offset)

    cmd1 = ["ros2", "bag", "play", args.bag1, "--rate", str(args.rate), "--start-offset", str(bag1_offset)]
    cmd2 = ["ros2", "bag", "play", args.bag2, "--rate", str(args.rate), "--start-offset", str(bag2_offset)]
    if args.clock_from == "bag1":
        cmd1.append("--clock")
    elif args.clock_from == "bag2":
        cmd2.append("--clock")
    if args.loop:
        cmd1.append("--loop")
        cmd2.append("--loop")

    procs = [_start_process(cmd1), _start_process(cmd2)]
    try:
        while any(proc.poll() is None for proc in procs):
            time.sleep(0.2)
    except KeyboardInterrupt:
        for proc in procs:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        for proc in procs:
            proc.wait()
    return max(proc.returncode or 0 for proc in procs)


if __name__ == "__main__":
    sys.exit(main())
