#!/usr/bin/python3
import math
import select
import socket
import struct
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


def clamp_u16(value: int) -> int:
    return max(0, min(0xFFFF, int(value)))


def to_u8(value: int) -> int:
    return int(value) & 0xFF


def read_be_u16(data: bytes, offset: int, signed: bool = False) -> int:
    raw = (data[offset] << 8) | data[offset + 1]
    if signed and raw >= 0x8000:
        raw -= 0x10000
    return raw


def read_u16(data: bytes, offset: int, signed: bool = False, byte_order: str = "big") -> int:
    if byte_order in ("little", "le", "low_high", "low-high"):
        raw = data[offset] | (data[offset + 1] << 8)
    else:
        raw = (data[offset] << 8) | data[offset + 1]
    if signed and raw >= 0x8000:
        raw -= 0x10000
    return raw


def hex16(value: int) -> str:
    return f"0x{int(value) & 0xFFFF:04X}"


class ChassisCanCommNode(Node):
    def float_param_with_alias(self, primary: str, alias: str, default: float) -> float:
        primary_value = float(self.get_parameter(primary).value)
        alias_value = float(self.get_parameter(alias).value)
        if primary_value == default and alias_value != default:
            return alias_value
        return primary_value

    def __init__(self) -> None:
        super().__init__("chassis_can_comm_node")

        self.declare_parameter("interface", "can0")
        self.declare_parameter("command_can_id", 0x01)
        self.declare_parameter("feedback_can_id", 0x02)
        self.declare_parameter("mode_function", 0x00)
        self.declare_parameter("mode_data", 1)
        self.declare_parameter("send_mode_on_enable", True)
        self.declare_parameter("forward_function", 0x02)
        self.declare_parameter("turn_function", 0x03)
        self.declare_parameter("right_motor_function", 0x02)
        self.declare_parameter("left_motor_function", 0x03)
        self.declare_parameter("fan_function", 0x06)
        self.declare_parameter("enable_function", 0x01)
        self.declare_parameter("enable_data", 1722)
        self.declare_parameter("enable_crc", -1)
        self.declare_parameter("motor_query_function", 0x08)
        self.declare_parameter("read_data", 0)
        self.declare_parameter("read_crc", -1)
        self.declare_parameter("command_crc_mode", "modbus")
        self.declare_parameter("listen_only", True)
        self.declare_parameter("enable_on_start", False)
        self.declare_parameter("query_on_start", False)
        self.declare_parameter("query_period_s", 1.0)
        # Backward-compatible aliases used by the earlier test commands.
        self.declare_parameter("query_data", 0)
        self.declare_parameter("query_crc", -1)
        self.declare_parameter("log_all_frames", True)
        self.declare_parameter("feedback_byte_order", "big")
        self.declare_parameter("feedback_signed", True)
        self.declare_parameter("feedback_angle_signed", False)
        self.declare_parameter("feedback_crc_enabled", False)
        self.declare_parameter("feedback_crc_mode", "modbus")
        self.declare_parameter("feedback_parse_mode", "small_board")
        self.declare_parameter("combined_left_speed_offset", 0)
        self.declare_parameter("combined_right_speed_offset", 2)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("wheel_odom_twist_topic", "/wheel_odom_twist")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_odom_tf", False)
        self.declare_parameter("odom_rate_hz", 50.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("wheel_base", 0.80)
        self.declare_parameter("wheel_radius", 0.22)
        self.declare_parameter("gear_ratio", 1.0)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("left_sign", 1.0)
        self.declare_parameter("right_sign", 1.0)
        # Backward-compatible aliases used by earlier versions of this package.
        self.declare_parameter("wheel_base_m", 0.80)
        self.declare_parameter("wheel_radius_m", 0.22)
        self.declare_parameter("max_motor_rpm", 1200.0)
        self.declare_parameter("left_motor_sign", 1.0)
        self.declare_parameter("right_motor_sign", 1.0)
        self.declare_parameter("left_feedback_sign", 1.0)
        self.declare_parameter("right_feedback_sign", 1.0)
        self.declare_parameter("feedback_speed_scale_to_rpm", 1.0)
        self.declare_parameter("feedback_angle_scale_to_rad", math.pi / 180.0)
        self.declare_parameter("speed_command_scale_from_rpm", 1.0)
        self.declare_parameter("send_zero_on_cmd_timeout", True)
        self.declare_parameter("command_control_mode", "forward_turn")
        self.declare_parameter("control_min_value", 282)
        self.declare_parameter("control_neutral_value", 1002)
        self.declare_parameter("control_max_value", 1722)
        self.declare_parameter("forward_value_per_mps", 720.0)
        self.declare_parameter("turn_value_per_radps", 720.0)
        self.declare_parameter("forward_cmd_sign", 1.0)
        self.declare_parameter("turn_cmd_sign", 1.0)
        self.declare_parameter("motor_command_crc_mode", "modbus")
        self.declare_parameter("motor_command_crc", 0)
        self.declare_parameter("known_right_800_crc", 0x7340)

        self.interface = self.get_parameter("interface").value
        self.command_can_id = int(self.get_parameter("command_can_id").value)
        self.feedback_can_id = int(self.get_parameter("feedback_can_id").value)
        self.motor_query_function = int(self.get_parameter("motor_query_function").value)
        self.listen_only = bool(self.get_parameter("listen_only").value)
        self.query_period_s = float(self.get_parameter("query_period_s").value)
        self.log_all_frames = bool(self.get_parameter("log_all_frames").value)
        self.feedback_byte_order = str(self.get_parameter("feedback_byte_order").value).lower()
        self.feedback_signed = bool(self.get_parameter("feedback_signed").value)
        self.feedback_angle_signed = bool(self.get_parameter("feedback_angle_signed").value)
        self.feedback_crc_enabled = bool(self.get_parameter("feedback_crc_enabled").value)
        self.feedback_crc_mode = str(self.get_parameter("feedback_crc_mode").value).lower()
        self.feedback_parse_mode = str(self.get_parameter("feedback_parse_mode").value).lower()
        self.wheel_base_m = self.float_param_with_alias("wheel_base", "wheel_base_m", 0.80)
        self.wheel_radius_m = self.float_param_with_alias("wheel_radius", "wheel_radius_m", 0.22)
        self.gear_ratio = max(float(self.get_parameter("gear_ratio").value), 1e-6)
        self.speed_scale = max(self.float_param_with_alias("speed_scale", "feedback_speed_scale_to_rpm", 1.0), 1e-9)
        self.max_motor_rpm = float(self.get_parameter("max_motor_rpm").value)
        self.left_motor_sign = self.float_param_with_alias("left_sign", "left_motor_sign", 1.0)
        self.right_motor_sign = self.float_param_with_alias("right_sign", "right_motor_sign", 1.0)
        self.left_feedback_sign = self.float_param_with_alias("left_sign", "left_feedback_sign", 1.0)
        self.right_feedback_sign = self.float_param_with_alias("right_sign", "right_feedback_sign", 1.0)
        self.feedback_speed_scale_to_rpm = float(self.get_parameter("feedback_speed_scale_to_rpm").value)
        self.feedback_angle_scale_to_rad = float(self.get_parameter("feedback_angle_scale_to_rad").value)
        self.speed_command_scale_from_rpm = float(self.get_parameter("speed_command_scale_from_rpm").value)
        self.cmd_vel_timeout_s = float(self.get_parameter("cmd_vel_timeout_s").value)
        self.send_zero_on_cmd_timeout = bool(self.get_parameter("send_zero_on_cmd_timeout").value)
        self.publish_odom_tf = bool(self.get_parameter("publish_odom_tf").value)
        self.command_control_mode = str(self.get_parameter("command_control_mode").value).lower()
        self.control_min_value = int(self.get_parameter("control_min_value").value)
        self.control_neutral_value = int(self.get_parameter("control_neutral_value").value)
        self.control_max_value = int(self.get_parameter("control_max_value").value)
        self.forward_value_per_mps = float(self.get_parameter("forward_value_per_mps").value)
        self.turn_value_per_radps = float(self.get_parameter("turn_value_per_radps").value)
        self.forward_cmd_sign = float(self.get_parameter("forward_cmd_sign").value)
        self.turn_cmd_sign = float(self.get_parameter("turn_cmd_sign").value)

        self.raw_rx_pub = self.create_publisher(String, "raw_rx", 20)
        self.raw_tx_pub = self.create_publisher(String, "raw_tx", 20)
        self.right_speed_pub = self.create_publisher(String, "right_motor/speed", 10)
        self.right_speed_hex_pub = self.create_publisher(String, "right_motor/speed_hex", 10)
        self.right_angle_pub = self.create_publisher(String, "right_motor/angle", 10)
        self.right_angle_hex_pub = self.create_publisher(String, "right_motor/angle_hex", 10)
        self.left_speed_pub = self.create_publisher(String, "left_motor/speed", 10)
        self.left_speed_hex_pub = self.create_publisher(String, "left_motor/speed_hex", 10)
        self.left_angle_pub = self.create_publisher(String, "left_motor/angle", 10)
        self.left_angle_hex_pub = self.create_publisher(String, "left_motor/angle_hex", 10)
        self.decoded_hex_pub = self.create_publisher(String, "chassis_can/decoded_hex", 10)
        self.left_track_velocity_pub = self.create_publisher(Float32, "left_track/velocity", 10)
        self.right_track_velocity_pub = self.create_publisher(Float32, "right_track/velocity", 10)
        self.vx_pub = self.create_publisher(Float32, "chassis/vx", 10)
        self.wz_pub = self.create_publisher(Float32, "chassis/wz", 10)
        self.fan_data1_pub = self.create_publisher(String, "fan/data1", 10)
        self.fan_data2_pub = self.create_publisher(String, "fan/data2", 10)
        self.odom_pub = self.create_publisher(Odometry, self.get_parameter("odom_topic").value, 20)
        self.twist_pub = self.create_publisher(Twist, self.get_parameter("wheel_odom_twist_topic").value, 20)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self.cmd_vel_cb,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_odom_tf else None

        self.enable_srv = self.create_service(Trigger, "~/enable", self.enable_cb)
        self.read_once_srv = self.create_service(Trigger, "~/read_once", self.read_once_cb)

        self.right_speed_raw: Optional[int] = None
        self.left_speed_raw: Optional[int] = None
        self.right_drive_rpm: Optional[float] = None
        self.left_drive_rpm: Optional[float] = None
        self.right_rpm: Optional[float] = None
        self.left_rpm: Optional[float] = None
        self.v_l: Optional[float] = None
        self.v_r: Optional[float] = None
        self.vx = 0.0
        self.wz = 0.0
        self.right_angle_rad: Optional[float] = None
        self.left_angle_rad: Optional[float] = None
        self.last_feedback_time = None
        self.last_cmd_time = None
        self.last_cmd_was_zero = True
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = self.get_clock().now()

        self.sock: Optional[socket.socket] = None
        self.open_can()

        self.poll_timer = self.create_timer(0.01, self.poll_can)
        odom_period = 1.0 / max(float(self.get_parameter("odom_rate_hz").value), 1.0)
        self.odom_timer = self.create_timer(odom_period, self.update_odom)

        if bool(self.get_parameter("enable_on_start").value) and not self.listen_only:
            self.send_enable()

        if bool(self.get_parameter("query_on_start").value) and not self.listen_only:
            self.send_motor_query()

        if self.query_period_s > 0.0 and not self.listen_only:
            self.query_timer = self.create_timer(self.query_period_s, self.send_motor_query)
        else:
            self.query_timer = None

    def open_can(self) -> None:
        try:
            self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((self.interface,))
            self.sock.setblocking(False)
        except OSError as exc:
            raise RuntimeError(
                f"Failed to open {self.interface}. Bring it up first, for example: "
                f"ros2 run chassis_can setup_can.sh {self.interface} 100000"
            ) from exc

        self.get_logger().info(
            f"Opened SocketCAN interface {self.interface}; "
            f"command_id=0x{self.command_can_id:03X}, feedback_id=0x{self.feedback_can_id:03X}"
        )

    def pack_frame(self, can_id: int, data: bytes) -> bytes:
        if len(data) > 8:
            raise ValueError("CAN data length cannot exceed 8 bytes")
        padded = data.ljust(8, b"\x00")
        return struct.pack(CAN_FRAME_FMT, can_id & CAN_SFF_MASK, len(data), padded)

    def send_frame(self, can_id: int, data: bytes) -> bool:
        if self.sock is None:
            return False
        try:
            self.sock.send(self.pack_frame(can_id, data))
        except OSError as exc:
            self.get_logger().error(f"CAN send failed: {exc}")
            return False
        line = f"TX id=0x{can_id:03X} data={data.hex(' ').upper()}"
        self.get_logger().info(line)
        self.raw_tx_pub.publish(String(data=line))
        return True

    def command_crc(self, function: int, data_value: int, explicit_crc: int) -> int:
        if explicit_crc >= 0:
            return clamp_u16(explicit_crc)
        mode = str(self.get_parameter("command_crc_mode").value).lower()
        crc_data = bytes([to_u8(function), (data_value >> 8) & 0xFF, data_value & 0xFF])
        if mode == "modbus":
            return self.crc16_modbus(crc_data)
        if mode == "sum16":
            return sum(crc_data) & 0xFFFF
        if mode in ("zero", "none", "off", "disable", "disabled"):
            return 0
        self.get_logger().warning(f"Unknown command_crc_mode={mode}; use zero CRC", throttle_duration_sec=5.0)
        return 0

    def protocol_payload(self, function: int, data_value: int, crc_value: int) -> bytes:
        data_value = clamp_u16(data_value)
        crc_value = self.command_crc(function, data_value, int(crc_value))
        return bytes(
            [
                to_u8(function),
                (data_value >> 8) & 0xFF,
                data_value & 0xFF,
                (crc_value >> 8) & 0xFF,
                crc_value & 0xFF,
            ]
        )

    def send_protocol_command(self, function: int, data_value: int, crc_value: int) -> bool:
        payload = self.protocol_payload(function, data_value, crc_value)
        return self.send_frame(self.command_can_id, payload)

    def send_enable(self) -> bool:
        ok = True
        if bool(self.get_parameter("send_mode_on_enable").value):
            ok = self.send_protocol_command(
                int(self.get_parameter("mode_function").value),
                int(self.get_parameter("mode_data").value),
                -1,
            )
        power_ok = self.send_protocol_command(
            int(self.get_parameter("enable_function").value),
            int(self.get_parameter("enable_data").value),
            int(self.get_parameter("enable_crc").value),
        )
        return ok and power_ok

    def send_motor_query(self) -> bool:
        function = self.motor_query_function
        read_data_param = self.get_parameter("read_data").value
        read_crc_param = self.get_parameter("read_crc").value
        query_data_param = self.get_parameter("query_data").value
        query_crc_param = self.get_parameter("query_crc").value
        if function == 0x08:
            data_value = int(query_data_param if query_data_param != 0 else read_data_param)
            crc_value = int(query_crc_param if query_crc_param >= 0 else read_crc_param)
        else:
            data_value = int(query_data_param)
            crc_value = int(query_crc_param)
        return self.send_protocol_command(function, data_value, crc_value)

    def crc16_modbus(self, data: bytes, init: int = 0xFFFF) -> int:
        crc = init
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
                crc &= 0xFFFF
        return crc

    def crc16_ccitt_false(self, data: bytes, init: int = 0xFFFF) -> int:
        crc = init
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def expected_feedback_crc(self, data_without_crc: bytes) -> Optional[int]:
        mode = self.feedback_crc_mode
        if mode in ("off", "none", "disable", "disabled"):
            return None
        if mode == "sum16":
            return sum(data_without_crc) & 0xFFFF
        if mode == "modbus":
            return self.crc16_modbus(data_without_crc)
        if mode == "modbus0":
            return self.crc16_modbus(data_without_crc, init=0x0000)
        if mode == "ccitt_false":
            return self.crc16_ccitt_false(data_without_crc)
        if mode == "ccitt0":
            return self.crc16_ccitt_false(data_without_crc, init=0x0000)
        if mode == "known_read_crc":
            if data_without_crc == b"\x00\x00\x00\x00\x00":
                return 0x0000
            if data_without_crc == b"\x08\x00\x00":
                return 0x81C0
            return None
        self.get_logger().warning(f"Unknown feedback_crc_mode={mode}; skip CRC check", throttle_duration_sec=5.0)
        return None

    def feedback_crc_ok(self, payload: bytes) -> bool:
        if not self.feedback_crc_enabled:
            return True
        if len(payload) < 7:
            self.get_logger().warning(f"feedback frame too short for CRC: dlc={len(payload)}")
            return False
        expected = self.expected_feedback_crc(payload[:5])
        actual = read_be_u16(payload, 5, False)
        if expected is None:
            self.get_logger().warning(
                f"CRC unchecked for data={payload[:5].hex(' ').upper()}, "
                f"actual=0x{actual:04X}; set feedback_crc_mode when algorithm is confirmed",
                throttle_duration_sec=5.0,
            )
            return True
        if expected != actual:
            self.get_logger().warning(
                f"drop feedback: CRC mismatch data={payload[:5].hex(' ').upper()} "
                f"actual=0x{actual:04X} expected=0x{expected:04X}",
                throttle_duration_sec=1.0,
            )
            return False
        return True

    def motor_command_crc(self, function: int, speed_command: int) -> int:
        mode = str(self.get_parameter("motor_command_crc_mode").value).lower()
        data_value = speed_command & 0xFFFF
        crc_data = bytes([to_u8(function), (data_value >> 8) & 0xFF, data_value & 0xFF])
        if mode == "modbus":
            return self.crc16_modbus(crc_data)
        if mode == "fixed":
            return clamp_u16(int(self.get_parameter("motor_command_crc").value))
        if mode == "known_right_800" and function == int(self.get_parameter("right_motor_function").value) and speed_command == 800:
            return clamp_u16(int(self.get_parameter("known_right_800_crc").value))
        if mode in ("zero", "none", "off", "disable", "disabled"):
            return 0
        self.get_logger().warning(f"Unknown motor_command_crc_mode={mode}; use zero CRC", throttle_duration_sec=5.0)
        return 0

    def send_motor_speed(self, function: int, rpm: float) -> bool:
        scaled = int(round(rpm))
        scaled = max(-32768, min(32767, scaled))
        data_value = scaled & 0xFFFF
        crc_value = self.motor_command_crc(function, scaled)
        return self.send_protocol_command(function, data_value, crc_value)

    def send_wheel_rpm(self, left_rpm: float, right_rpm: float) -> bool:
        if self.listen_only:
            self.get_logger().warning("listen_only is true; refuse to transmit wheel speed command")
            return False
        right_ok = self.send_motor_speed(int(self.get_parameter("right_motor_function").value), right_rpm)
        left_ok = self.send_motor_speed(int(self.get_parameter("left_motor_function").value), left_rpm)
        return right_ok and left_ok

    def clamp_control_value(self, value: float) -> int:
        low = min(self.control_min_value, self.control_max_value)
        high = max(self.control_min_value, self.control_max_value)
        return int(round(max(low, min(high, value))))

    def send_forward_turn(self, linear: float, angular: float) -> bool:
        if self.listen_only:
            self.get_logger().warning("listen_only is true; refuse to transmit forward/turn command")
            return False
        forward_value = self.clamp_control_value(
            self.control_neutral_value + linear * self.forward_value_per_mps * self.forward_cmd_sign
        )
        turn_value = self.clamp_control_value(
            self.control_neutral_value + angular * self.turn_value_per_radps * self.turn_cmd_sign
        )
        forward_ok = self.send_protocol_command(
            int(self.get_parameter("forward_function").value),
            forward_value,
            -1,
        )
        turn_ok = self.send_protocol_command(
            int(self.get_parameter("turn_function").value),
            turn_value,
            -1,
        )
        return forward_ok and turn_ok

    def send_motion_neutral(self) -> bool:
        return self.send_forward_turn(0.0, 0.0)

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.last_cmd_time = self.get_clock().now()
        if self.listen_only:
            self.get_logger().warning("listen_only is true; ignore /cmd_vel", throttle_duration_sec=2.0)
            return

        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        if self.command_control_mode in ("forward_turn", "new", "new_protocol"):
            self.last_cmd_was_zero = abs(linear) < 1e-6 and abs(angular) < 1e-6
            self.send_forward_turn(linear, angular)
            return

        left_mps = linear - angular * self.wheel_base_m * 0.5
        right_mps = linear + angular * self.wheel_base_m * 0.5
        meters_per_rev = 2.0 * math.pi * self.wheel_radius_m
        if meters_per_rev <= 0.0:
            self.get_logger().error("wheel_radius_m must be positive")
            return

        left_wheel_rpm = left_mps / meters_per_rev * 60.0
        right_wheel_rpm = right_mps / meters_per_rev * 60.0
        left_motor_rpm = max(-self.max_motor_rpm, min(self.max_motor_rpm, left_wheel_rpm * self.gear_ratio))
        right_motor_rpm = max(-self.max_motor_rpm, min(self.max_motor_rpm, right_wheel_rpm * self.gear_ratio))
        left_protocol_speed = left_motor_rpm / self.speed_scale * self.left_motor_sign
        right_protocol_speed = right_motor_rpm / self.speed_scale * self.right_motor_sign
        self.last_cmd_was_zero = abs(left_protocol_speed) < 1e-6 and abs(right_protocol_speed) < 1e-6
        self.send_wheel_rpm(left_protocol_speed, right_protocol_speed)

    def maybe_stop_on_cmd_timeout(self, now) -> None:
        if self.listen_only or not self.send_zero_on_cmd_timeout or self.last_cmd_time is None:
            return
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.cmd_vel_timeout_s and not self.last_cmd_was_zero:
            self.get_logger().warning("cmd_vel timeout; send neutral motion command")
            if self.command_control_mode in ("forward_turn", "new", "new_protocol"):
                self.send_motion_neutral()
            else:
                self.send_wheel_rpm(0.0, 0.0)
            self.last_cmd_was_zero = True

    def publish_odom_tf_msg(self, odom_msg: Odometry) -> None:
        if self.tf_broadcaster is None:
            return
        tf_msg = TransformStamped()
        tf_msg.header = odom_msg.header
        tf_msg.child_frame_id = odom_msg.child_frame_id
        tf_msg.transform.translation.x = odom_msg.pose.pose.position.x
        tf_msg.transform.translation.y = odom_msg.pose.pose.position.y
        tf_msg.transform.translation.z = odom_msg.pose.pose.position.z
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_float(self, publisher, value: float) -> None:
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def update_track_velocities(self) -> bool:
        if self.left_drive_rpm is None or self.right_drive_rpm is None:
            return False
        self.v_l = self.left_drive_rpm / 60.0 * 2.0 * math.pi * self.wheel_radius_m
        self.v_r = self.right_drive_rpm / 60.0 * 2.0 * math.pi * self.wheel_radius_m
        self.vx = 0.5 * (self.v_l + self.v_r)
        self.wz = (self.v_r - self.v_l) / self.wheel_base_m if self.wheel_base_m > 0.0 else 0.0
        self.publish_float(self.left_track_velocity_pub, self.v_l)
        self.publish_float(self.right_track_velocity_pub, self.v_r)
        self.publish_float(self.vx_pub, self.vx)
        self.publish_float(self.wz_pub, self.wz)
        return True

    def update_odom(self) -> None:
        now = self.get_clock().now()
        self.maybe_stop_on_cmd_timeout(now)
        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = now
        if dt <= 0.0 or not self.update_track_velocities():
            return

        mid_yaw = self.yaw + self.wz * dt * 0.5
        self.x += self.vx * math.cos(mid_yaw) * dt
        self.y += self.vx * math.sin(mid_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.wz * dt), math.cos(self.yaw + self.wz * dt))

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.get_parameter("odom_frame_id").value
        odom_msg.child_frame_id = self.get_parameter("base_frame_id").value
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom_msg)
        self.publish_odom_tf_msg(odom_msg)

        twist_msg = Twist()
        twist_msg.linear.x = self.vx
        twist_msg.angular.z = self.wz
        self.twist_pub.publish(twist_msg)

    def enable_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self.listen_only:
            response.success = False
            response.message = "listen_only is true; refuse to transmit enable command"
            return response
        response.success = self.send_enable()
        response.message = "enable command sent" if response.success else "enable command failed"
        return response

    def read_once_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self.listen_only:
            response.success = False
            response.message = "listen_only is true; refuse to transmit read command"
            return response
        response.success = self.send_motor_query()
        response.message = "read command sent" if response.success else "read command failed"
        return response

    def publish_string(self, publisher, value: str) -> None:
        msg = String()
        msg.data = value
        publisher.publish(msg)

    def apply_wheel_feedback(self, side: str, speed_raw: int, angle_raw: Optional[int]) -> None:
        sign = self.left_feedback_sign if side == "left" else self.right_feedback_sign
        motor_rpm = speed_raw * self.speed_scale * sign
        drive_rpm = motor_rpm / self.gear_ratio
        angle_rad = None
        if angle_raw is not None:
            angle_rad = angle_raw * self.feedback_angle_scale_to_rad * sign / self.gear_ratio

        if side == "left":
            self.left_speed_raw = speed_raw
            self.left_rpm = drive_rpm
            self.left_drive_rpm = drive_rpm
            self.left_angle_rad = angle_rad
            if angle_raw is not None:
                self.publish_string(self.left_angle_pub, hex16(angle_raw))
                self.publish_string(self.left_angle_hex_pub, hex16(angle_raw))
            self.publish_string(self.left_speed_pub, hex16(speed_raw))
            self.publish_string(self.left_speed_hex_pub, hex16(speed_raw))
        else:
            self.right_speed_raw = speed_raw
            self.right_rpm = drive_rpm
            self.right_drive_rpm = drive_rpm
            self.right_angle_rad = angle_rad
            if angle_raw is not None:
                self.publish_string(self.right_angle_pub, hex16(angle_raw))
                self.publish_string(self.right_angle_hex_pub, hex16(angle_raw))
            self.publish_string(self.right_speed_pub, hex16(speed_raw))
            self.publish_string(self.right_speed_hex_pub, hex16(speed_raw))

        self.last_feedback_time = self.get_clock().now()

    def handle_small_board_feedback(self, payload: bytes) -> None:
        if len(payload) < 8:
            self.get_logger().warning(f"small_board feedback too short: dlc={len(payload)}")
            return

        left_angle_raw = read_u16(payload, 0, self.feedback_angle_signed, self.feedback_byte_order)
        right_angle_raw = read_u16(payload, 2, self.feedback_angle_signed, self.feedback_byte_order)
        left_speed_raw = read_u16(payload, 4, self.feedback_signed, self.feedback_byte_order)
        right_speed_raw = read_u16(payload, 6, self.feedback_signed, self.feedback_byte_order)

        self.apply_wheel_feedback("left", left_speed_raw, left_angle_raw)
        self.apply_wheel_feedback("right", right_speed_raw, right_angle_raw)

        decoded_hex = (
            f"raw={payload.hex(' ').upper()} "
            f"left_angle={hex16(left_angle_raw)} "
            f"right_angle={hex16(right_angle_raw)} "
            f"left_speed={hex16(left_speed_raw)} "
            f"right_speed={hex16(right_speed_raw)}"
        )
        self.publish_string(self.decoded_hex_pub, decoded_hex)

        self.get_logger().info(
            "Feedback small_board: "
            f"byte_order={self.feedback_byte_order}, "
            f"left_angle_raw={hex16(left_angle_raw)}, "
            f"right_angle_raw={hex16(right_angle_raw)}, "
            f"left_speed_raw={hex16(left_speed_raw)}, "
            f"right_speed_raw={hex16(right_speed_raw)}"
        )

    def handle_big_board_feedback(self, payload: bytes) -> None:
        if len(payload) < 5:
            self.get_logger().warning(f"big_board feedback too short: dlc={len(payload)}")
            return

        function = payload[0]
        data_u16 = read_be_u16(payload, 1, False)
        data_i16 = read_be_u16(payload, 1, True)
        actual_crc = read_be_u16(payload, 3, False)
        expected_crc = self.crc16_modbus(payload[:3])
        crc_ok = actual_crc == expected_crc

        self.get_logger().info(
            "Feedback big_board: "
            f"function=0x{function:02X}, data={hex16(data_u16)}, "
            f"crc=0x{actual_crc:04X}, expected=0x{expected_crc:04X}, ok={crc_ok}"
        )

    def poll_can(self) -> None:
        if self.sock is None:
            return

        while True:
            readable, _, _ = select.select([self.sock], [], [], 0.0)
            if not readable:
                return

            try:
                raw = self.sock.recv(CAN_FRAME_SIZE)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().error(f"CAN receive failed: {exc}")
                return

            can_id, can_dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            is_extended = bool(can_id & CAN_EFF_FLAG)
            is_rtr = bool(can_id & CAN_RTR_FLAG)
            is_error = bool(can_id & CAN_ERR_FLAG)
            frame_id = can_id & (CAN_EFF_MASK if is_extended else CAN_SFF_MASK)
            payload = data[:can_dlc]
            self.handle_frame(frame_id, payload, is_extended, is_rtr, is_error)

    def handle_frame(
        self,
        frame_id: int,
        payload: bytes,
        is_extended: bool,
        is_rtr: bool,
        is_error: bool,
    ) -> None:
        if self.log_all_frames:
            flags = []
            if is_extended:
                flags.append("EFF")
            if is_rtr:
                flags.append("RTR")
            if is_error:
                flags.append("ERR")
            suffix = f" flags={','.join(flags)}" if flags else ""
            line = f"RX id=0x{frame_id:03X} dlc={len(payload)} data={payload.hex(' ').upper()}{suffix}"
            self.get_logger().info(line)
            self.raw_rx_pub.publish(String(data=line))

        if frame_id != self.feedback_can_id:
            return

        if self.feedback_parse_mode in ("small_board", "new_small_board", "motor_board"):
            if len(payload) >= 8:
                self.handle_small_board_feedback(payload)
            elif len(payload) >= 5:
                self.handle_big_board_feedback(payload)
            else:
                self.get_logger().warning(f"feedback frame too short: dlc={len(payload)}")
            return

        if len(payload) < 5 or not self.feedback_crc_ok(payload):
            return

        if self.feedback_parse_mode == "combined_speed":
            left_offset = int(self.get_parameter("combined_left_speed_offset").value)
            right_offset = int(self.get_parameter("combined_right_speed_offset").value)
            if len(payload) >= max(left_offset, right_offset) + 2:
                left_speed_raw = read_be_u16(payload, left_offset, self.feedback_signed)
                right_speed_raw = read_be_u16(payload, right_offset, self.feedback_signed)
                self.apply_wheel_feedback("left", left_speed_raw, None)
                self.apply_wheel_feedback("right", right_speed_raw, None)
                self.get_logger().info(
                    f"Feedback combined_speed: left_speed_raw={hex16(left_speed_raw)}, "
                    f"right_speed_raw={hex16(right_speed_raw)}"
                )
            else:
                self.get_logger().warning(
                    f"combined_speed frame too short: dlc={len(payload)}, "
                    f"left_offset={left_offset}, right_offset={right_offset}"
                )
            return

        # PDF/实测上报帧: byte0=功能位, byte1..2=DATA1(转速),
        # byte3..4=DATA2(角度)。若后续实测 DLC>=7 且最后两字节为 CRC，
        # 再将 feedback_crc_enabled 打开。
        function = payload[0]
        data1 = read_be_u16(payload, 1, self.feedback_signed)
        data2 = read_be_u16(payload, 3, self.feedback_signed)
        crc_text = ""
        if len(payload) >= 7:
            crc_value = read_be_u16(payload, 5, False)
            crc_text = f", crc=0x{crc_value:04X}"
        if len(payload) > 7:
            crc_text += f", extra={payload[7:].hex(' ').upper()}"

        right_function = int(self.get_parameter("right_motor_function").value)
        left_function = int(self.get_parameter("left_motor_function").value)
        fan_function = int(self.get_parameter("fan_function").value)

        if function == right_function:
            name = "right_motor"
            self.apply_wheel_feedback("right", data1, data2)
        elif function == left_function:
            name = "left_motor"
            self.apply_wheel_feedback("left", data1, data2)
        elif function == fan_function:
            name = "fan"
            self.publish_string(self.fan_data1_pub, hex16(data1))
            self.publish_string(self.fan_data2_pub, hex16(data2))
        else:
            name = f"function_0x{function:02X}"

        self.get_logger().info(
            f"Feedback {name}: speed_raw={hex16(data1)}, angle_raw={hex16(data2)}{crc_text}"
        )

    def destroy_node(self) -> bool:
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = ChassisCanCommNode()
    except RuntimeError as exc:
        temp_node = rclpy.create_node("chassis_can_comm_startup_error")
        temp_node.get_logger().error(str(exc))
        temp_node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
