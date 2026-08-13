import argparse
import struct
import math
import socket
from dataclasses import dataclass
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_msgs.msg import NavPVT

try:
    from rtcm_msgs.msg import Message as RtcmMessage
except ImportError:
    RtcmMessage = None


KNOT_TO_MPS = 0.5144444444444445
UBX_SYNC = b"\xb5\x62"


@dataclass
class GgaData:
    utc_time: str = ""
    latitude: float = math.nan
    longitude: float = math.nan
    quality: int = 0
    num_sv: int = 0
    hdop: float = math.nan
    altitude_msl_m: float = math.nan
    geoid_sep_m: float = math.nan


@dataclass
class RmcData:
    utc_time: str = ""
    valid: bool = False
    latitude: float = math.nan
    longitude: float = math.nan
    speed_mps: float = math.nan
    course_deg: float = math.nan
    date: str = ""


@dataclass
class VtgData:
    course_deg: float = math.nan
    speed_mps: float = math.nan


def nmea_checksum_ok(sentence):
    text = sentence.strip()
    if not text.startswith("$") or "*" not in text:
        return False
    body, checksum_text = text[1:].split("*", 1)
    checksum_text = checksum_text[:2]
    try:
        expected = int(checksum_text, 16)
    except ValueError:
        return False
    actual = 0
    for char in body:
        actual ^= ord(char)
    return actual == expected


def sentence_type(sentence):
    body = sentence.strip()[1:].split("*", 1)[0]
    talker_and_type = body.split(",", 1)[0]
    return talker_and_type[-3:]


def _fields(sentence):
    return sentence.strip()[1:].split("*", 1)[0].split(",")


def _float_or_nan(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return math.nan


def _int_or_zero(text):
    try:
        return int(text)
    except (TypeError, ValueError):
        return 0


def parse_lat_lon(value, hemisphere):
    if not value or not hemisphere:
        return math.nan
    dot = value.find(".")
    if dot < 0 or dot < 2:
        return math.nan
    deg_len = dot - 2
    try:
        degrees = float(value[:deg_len])
        minutes = float(value[deg_len:])
    except ValueError:
        return math.nan
    decimal = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        decimal *= -1.0
    return decimal


def parse_gga(sentence):
    parts = _fields(sentence)
    return GgaData(
        utc_time=parts[1] if len(parts) > 1 else "",
        latitude=parse_lat_lon(parts[2], parts[3]) if len(parts) > 3 else math.nan,
        longitude=parse_lat_lon(parts[4], parts[5]) if len(parts) > 5 else math.nan,
        quality=_int_or_zero(parts[6]) if len(parts) > 6 else 0,
        num_sv=_int_or_zero(parts[7]) if len(parts) > 7 else 0,
        hdop=_float_or_nan(parts[8]) if len(parts) > 8 else math.nan,
        altitude_msl_m=_float_or_nan(parts[9]) if len(parts) > 9 else math.nan,
        geoid_sep_m=_float_or_nan(parts[11]) if len(parts) > 11 else math.nan,
    )


def parse_rmc(sentence):
    parts = _fields(sentence)
    speed_knots = _float_or_nan(parts[7]) if len(parts) > 7 else math.nan
    return RmcData(
        utc_time=parts[1] if len(parts) > 1 else "",
        valid=len(parts) > 2 and parts[2] == "A",
        latitude=parse_lat_lon(parts[3], parts[4]) if len(parts) > 4 else math.nan,
        longitude=parse_lat_lon(parts[5], parts[6]) if len(parts) > 6 else math.nan,
        speed_mps=speed_knots * KNOT_TO_MPS if math.isfinite(speed_knots) else math.nan,
        course_deg=_float_or_nan(parts[8]) if len(parts) > 8 else math.nan,
        date=parts[9] if len(parts) > 9 else "",
    )


def parse_vtg(sentence):
    parts = _fields(sentence)
    return VtgData(
        course_deg=_float_or_nan(parts[1]) if len(parts) > 1 else math.nan,
        speed_mps=_float_or_nan(parts[7]) / 3.6 if len(parts) > 7 else math.nan,
    )


def navpvt_flags_from_quality(quality):
    if quality <= 0:
        return 0
    flags = NavPVT.FLAGS_GNSS_FIX_OK
    if quality in (2, 4, 5):
        flags |= NavPVT.FLAGS_DIFF_SOLN
    if quality == 4:
        flags |= NavPVT.CARRIER_PHASE_FIXED
    elif quality == 5:
        flags |= NavPVT.CARRIER_PHASE_FLOAT
    return flags


def navsat_status_from_quality(quality):
    if quality <= 0:
        return NavSatStatus.STATUS_NO_FIX
    if quality == 4:
        return NavSatStatus.STATUS_GBAS_FIX
    if quality in (2, 5):
        return NavSatStatus.STATUS_SBAS_FIX
    return NavSatStatus.STATUS_FIX


def horizontal_accuracy_mm(hdop, default_mm):
    if math.isfinite(hdop) and hdop > 0.0:
        return max(1, int(round(hdop * 1000.0)))
    return default_mm


def split_nmea_sentences(text):
    sentences = []
    for chunk in text.replace("\r", "\n").split("\n"):
        chunk = chunk.strip()
        if chunk.startswith("$") and "*" in chunk:
            sentences.append(chunk)
    return sentences


def extract_complete_nmea(buffer):
    sentences = []
    index = 0
    while True:
        start = buffer.find("$", index)
        if start < 0:
            return sentences, ""
        star = buffer.find("*", start)
        if star < 0 or len(buffer) < star + 3:
            return sentences, buffer[start:]
        sentence = buffer[start : star + 3]
        sentences.append(sentence.strip())
        index = star + 3


def ubx_checksum(payload):
    ck_a = 0
    ck_b = 0
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes((ck_a, ck_b))


def ubx_frame(class_id, message_id, payload):
    header_and_payload = bytes((class_id, message_id)) + struct.pack("<H", len(payload)) + payload
    return UBX_SYNC + header_and_payload + ubx_checksum(header_and_payload)


def ubx_cfg_rate(rate_hz, nav_rate):
    meas_rate_ms = int(round(1000.0 / rate_hz))
    payload = struct.pack("<HHH", meas_rate_ms, nav_rate, 1)
    return ubx_frame(0x06, 0x08, payload)


class C099UdpBridge(Node):
    def __init__(self, args):
        super().__init__("c099_udp_bridge")
        self.args = args
        self.gga = GgaData()
        self.rmc = RmcData()
        self.vtg = VtgData()
        self.partial_text = ""
        self.raw_log = open(args.raw_log, "ab") if args.raw_log else None

        self.navpvt_pub = self.create_publisher(NavPVT, args.navpvt_topic, 10)
        self.fix_pub = self.create_publisher(NavSatFix, args.fix_topic, 10)
        self.velocity_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            args.fix_velocity_topic,
            10,
        )
        self.rtcm_rx_count = 0
        self.rtcm_tx_bytes = 0
        if args.forward_rtcm:
            if RtcmMessage is None:
                self.get_logger().warning("rtcm_msgs is not importable; RTCM forwarding is disabled")
            else:
                self.create_subscription(RtcmMessage, args.rtcm_topic, self._on_rtcm, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.connect((args.host, args.port))
        self.sock.send(args.keepalive_payload)
        self._configure_receiver_if_requested()
        self.last_keepalive = self.get_clock().now()
        self.get_logger().info("Receiving C099-F9P UDP NMEA from %s:%d" % (args.host, args.port))

        self.timer = self.create_timer(args.poll_period_sec, self._poll_udp)
        self.keepalive_timer = self.create_timer(args.keepalive_period_sec, self._send_keepalive)

    def destroy_node(self):
        if self.raw_log is not None and not self.raw_log.closed:
            self.raw_log.close()
        self.sock.close()
        super().destroy_node()

    def _send_keepalive(self):
        try:
            self.sock.send(self.args.keepalive_payload)
        except OSError as exc:
            self.get_logger().warning("UDP keepalive failed: %s" % exc)

    def _on_rtcm(self, msg):
        try:
            payload = bytes(msg.message)
            self.sock.send(payload)
            self.rtcm_rx_count += 1
            self.rtcm_tx_bytes += len(payload)
        except OSError as exc:
            self.get_logger().warning("Failed to forward RTCM to C099 UDP: %s" % exc)

    def _configure_receiver_if_requested(self):
        if self.args.configure_rate_hz <= 0.0:
            return
        if self.args.configure_rate_hz > 20.0:
            self.get_logger().warning(
                "Refusing configure-rate-hz %.3f; use 20 Hz or lower for this bridge"
                % self.args.configure_rate_hz
            )
            return

        try:
            frame = ubx_cfg_rate(self.args.configure_rate_hz, self.args.configure_nav_rate)
            self.sock.send(frame)
            self.get_logger().info(
                "Sent UBX-CFG-RATE over UDP: rate=%.3f Hz nav_rate=%d meas_rate=%d ms"
                % (
                    self.args.configure_rate_hz,
                    self.args.configure_nav_rate,
                    int(round(1000.0 / self.args.configure_rate_hz)),
                )
            )
        except OSError as exc:
            self.get_logger().warning("Failed to send UBX-CFG-RATE over UDP: %s" % exc)

    def _poll_udp(self):
        while True:
            try:
                data = self.sock.recv(4096)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().warning("UDP receive failed: %s" % exc)
                return

            if self.raw_log is not None:
                self.raw_log.write(data)
                self.raw_log.flush()

            self.partial_text += data.decode("ascii", errors="ignore")
            sentences, self.partial_text = extract_complete_nmea(self.partial_text)
            for sentence in sentences:
                self._handle_sentence(sentence)

    def _handle_sentence(self, sentence):
        if self.args.validate_checksum and not nmea_checksum_ok(sentence):
            self.get_logger().debug("Ignoring NMEA sentence with bad checksum: %s" % sentence)
            return

        kind = sentence_type(sentence)
        if kind == "GGA":
            self.gga = parse_gga(sentence)
            self._publish_solution()
        elif kind == "RMC":
            self.rmc = parse_rmc(sentence)
        elif kind == "VTG":
            self.vtg = parse_vtg(sentence)

    def _publish_solution(self):
        stamp = self.get_clock().now().to_msg()
        navpvt = self._make_navpvt()
        fix = self._make_fix(stamp, navpvt)
        velocity = self._make_velocity(stamp, navpvt)

        self.navpvt_pub.publish(navpvt)
        self.fix_pub.publish(fix)
        self.velocity_pub.publish(velocity)

    def _make_navpvt(self):
        msg = NavPVT()
        dt = self._utc_datetime()
        if dt is not None:
            msg.year = dt.year
            msg.month = dt.month
            msg.day = dt.day
            msg.hour = dt.hour
            msg.min = dt.minute
            msg.sec = dt.second
            msg.valid = NavPVT.VALID_DATE | NavPVT.VALID_TIME | NavPVT.VALID_FULLY_RESOLVED
            msg.flags2 = (
                NavPVT.FLAGS2_CONFIRMED_AVAILABLE
                | NavPVT.FLAGS2_CONFIRMED_DATE
                | NavPVT.FLAGS2_CONFIRMED_TIME
            )
            msg.i_tow = self._gps_time_of_week_ms(dt)

        has_position = math.isfinite(self.gga.latitude) and math.isfinite(self.gga.longitude)
        msg.fix_type = NavPVT.FIX_TYPE_3D if self.gga.quality > 0 and has_position else NavPVT.FIX_TYPE_NO_FIX
        msg.flags = navpvt_flags_from_quality(self.gga.quality)
        msg.num_sv = self.gga.num_sv

        if has_position:
            msg.lat = int(round(self.gga.latitude * 1.0e7))
            msg.lon = int(round(self.gga.longitude * 1.0e7))
        if math.isfinite(self.gga.altitude_msl_m):
            msg.h_msl = int(round(self.gga.altitude_msl_m * 1000.0))
            height_m = self.gga.altitude_msl_m
            if math.isfinite(self.gga.geoid_sep_m):
                height_m += self.gga.geoid_sep_m
            msg.height = int(round(height_m * 1000.0))

        h_acc = horizontal_accuracy_mm(self.gga.hdop, self.args.default_h_acc_mm)
        v_acc = max(h_acc, self.args.default_v_acc_mm)
        s_acc = self.args.default_s_acc_mm_s
        if self.args.trust_nmea_rtk_quality:
            if self.gga.quality == 4:
                h_acc = min(h_acc, self.args.nmea_fixed_h_acc_mm)
                v_acc = min(v_acc, self.args.nmea_fixed_v_acc_mm)
                s_acc = min(s_acc, self.args.nmea_fixed_s_acc_mm_s)
            elif self.gga.quality == 5:
                h_acc = min(h_acc, self.args.nmea_float_h_acc_mm)
                v_acc = min(v_acc, self.args.nmea_float_v_acc_mm)
                s_acc = min(s_acc, self.args.nmea_float_s_acc_mm_s)
        msg.h_acc = h_acc
        msg.v_acc = v_acc
        msg.p_dop = int(round(self.gga.hdop * 100.0)) if math.isfinite(self.gga.hdop) else 9999

        speed_mps, course_deg = self._speed_and_course()
        if math.isfinite(speed_mps):
            msg.g_speed = int(round(speed_mps * 1000.0))
        msg.s_acc = s_acc
        if math.isfinite(course_deg):
            radians = math.radians(course_deg)
            if math.isfinite(speed_mps):
                msg.vel_n = int(round(math.cos(radians) * speed_mps * 1000.0))
                msg.vel_e = int(round(math.sin(radians) * speed_mps * 1000.0))
            msg.heading = int(round(course_deg * 1.0e5))
            msg.head_acc = self.args.default_head_acc_deg_e5
        return msg

    def _make_fix(self, stamp, navpvt):
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self.args.frame_id
        msg.latitude = navpvt.lat * 1.0e-7
        msg.longitude = navpvt.lon * 1.0e-7
        msg.altitude = navpvt.height * 1.0e-3
        msg.status.status = navsat_status_from_quality(self.gga.quality)
        msg.status.service = (
            NavSatStatus.SERVICE_GPS
            | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_GALILEO
            | NavSatStatus.SERVICE_COMPASS
        )
        var_h = (navpvt.h_acc * 1.0e-3) ** 2
        var_v = (navpvt.v_acc * 1.0e-3) ** 2
        msg.position_covariance[0] = var_h
        msg.position_covariance[4] = var_h
        msg.position_covariance[8] = var_v
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return msg

    def _make_velocity(self, stamp, navpvt):
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.args.frame_id
        msg.twist.twist.linear.x = navpvt.vel_e * 1.0e-3
        msg.twist.twist.linear.y = navpvt.vel_n * 1.0e-3
        msg.twist.twist.linear.z = -navpvt.vel_d * 1.0e-3
        speed_cov = (navpvt.s_acc * 1.0e-3) ** 2
        msg.twist.covariance[0] = speed_cov
        msg.twist.covariance[7] = speed_cov
        msg.twist.covariance[14] = speed_cov
        msg.twist.covariance[21] = -1.0
        return msg

    def _speed_and_course(self):
        speed = self.rmc.speed_mps
        course = self.rmc.course_deg
        if not math.isfinite(speed):
            speed = self.vtg.speed_mps
        if not math.isfinite(course):
            course = self.vtg.course_deg
        return speed, course

    def _utc_datetime(self):
        time_text = self.rmc.utc_time or self.gga.utc_time
        date_text = self.rmc.date
        if len(time_text) < 6 or len(date_text) != 6:
            return None
        try:
            hour = int(time_text[0:2])
            minute = int(time_text[2:4])
            second = int(float(time_text[4:]))
            day = int(date_text[0:2])
            month = int(date_text[2:4])
            year = 2000 + int(date_text[4:6])
            return datetime(year, month, day, hour, minute, second, tzinfo=timezone.utc)
        except ValueError:
            return None

    @staticmethod
    def _gps_time_of_week_ms(dt):
        gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
        elapsed = dt - gps_epoch
        return int((elapsed.total_seconds() % (7 * 24 * 3600)) * 1000.0)


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.0.1")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--frame-id", default="gps")
    parser.add_argument("--navpvt-topic", default="/ublox_gps_node/navpvt")
    parser.add_argument("--fix-topic", default="/ublox_gps_node/fix")
    parser.add_argument("--fix-velocity-topic", default="/ublox_gps_node/fix_velocity")
    parser.add_argument("--rtcm-topic", default="/rtcm")
    parser.add_argument("--forward-rtcm", action="store_true", help="Forward rtcm_msgs/Message payloads to the C099 UDP socket")
    parser.add_argument("--poll-period-sec", type=float, default=0.02)
    parser.add_argument("--keepalive-period-sec", type=float, default=1.0)
    parser.add_argument("--keepalive-payload", default="", help="String payload sent to register/keep UDP client")
    parser.add_argument(
        "--configure-rate-hz",
        type=float,
        default=0.0,
        help="Optionally send UBX-CFG-RATE over UDP at startup; 0 disables receiver configuration",
    )
    parser.add_argument("--configure-nav-rate", type=int, default=1)
    parser.add_argument("--raw-log", default="", help="Optional raw UDP append log path")
    parser.set_defaults(validate_checksum=True)
    parser.add_argument("--no-validate-checksum", dest="validate_checksum", action="store_false")
    parser.add_argument("--default-h-acc-mm", type=int, default=99999)
    parser.add_argument("--default-v-acc-mm", type=int, default=99999)
    parser.add_argument("--default-s-acc-mm-s", type=int, default=1000)
    parser.set_defaults(trust_nmea_rtk_quality=True)
    parser.add_argument(
        "--no-trust-nmea-rtk-quality",
        dest="trust_nmea_rtk_quality",
        action="store_false",
        help="Do not synthesize NavPVT accuracy from NMEA GGA RTK quality",
    )
    parser.add_argument("--nmea-fixed-h-acc-mm", type=int, default=20)
    parser.add_argument("--nmea-fixed-v-acc-mm", type=int, default=50)
    parser.add_argument("--nmea-fixed-s-acc-mm-s", type=int, default=50)
    parser.add_argument("--nmea-float-h-acc-mm", type=int, default=150)
    parser.add_argument("--nmea-float-v-acc-mm", type=int, default=300)
    parser.add_argument("--nmea-float-s-acc-mm-s", type=int, default=150)
    parser.add_argument("--default-head-acc-deg-e5", type=int, default=1000000)
    args, ros_args = parser.parse_known_args(argv)
    args.keepalive_payload = args.keepalive_payload.encode("ascii")
    return args, ros_args


def main(argv=None):
    args, ros_args = parse_args(argv)
    rclpy.init(args=ros_args)
    node = C099UdpBridge(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
