#!/usr/bin/python3
"""Publish HWT601/WitMotion-like RS485 Modbus IMU data as sensor_msgs/Imu."""

from __future__ import annotations

import math
import struct
from typing import Optional

import rclpy
import serial
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Imu


G = 9.80665


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(data: bytes) -> bytes:
    crc = crc16_modbus(data)
    return data + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def verify_crc(packet: bytes) -> bool:
    if len(packet) < 4:
        return False
    expected = packet[-2] | (packet[-1] << 8)
    return crc16_modbus(packet[:-2]) == expected


def int16_be(high: int, low: int) -> int:
    return struct.unpack(">h", bytes((high, low)))[0]


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class Hwt601485ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("hwt601_485_imu_node")
        self.port_name = self.declare_parameter("port", "/dev/ttyUSB0").value
        self.baudrate = int(self.declare_parameter("baudrate", 115200).value)
        self.address = int(self.declare_parameter("address", 0x50).value)
        self.frame_id = self.declare_parameter("frame_id", "imu_link").value
        self.topic = self.declare_parameter("topic", "/imu/data").value
        self.rate_hz = float(self.declare_parameter("rate_hz", 150.0).value)
        self.timeout = float(self.declare_parameter("timeout", 0.05).value)
        self.publish_orientation = bool(self.declare_parameter("publish_orientation", True).value)
        self.invert_accel = bool(self.declare_parameter("invert_accel", False).value)
        self.accel_range_g = float(self.declare_parameter("accel_range_g", 4.0).value)
        self.yaw_zero = float(self.declare_parameter("yaw_zero_deg", 0.0).value)

        self.publisher = self.create_publisher(Imu, self.topic, 20)
        self.serial_port: Optional[serial.Serial] = None
        self.open_serial()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.poll_once)
        self.get_logger().info(
            f"Publishing {self.topic} from {self.port_name}, baud={self.baudrate}, "
            f"address=0x{self.address:02X}, rate={self.rate_hz:.1f} Hz"
        )

    def open_serial(self) -> None:
        try:
            self.serial_port = serial.Serial(
                self.port_name,
                baudrate=self.baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=self.timeout,
            )
        except serial.SerialException as exc:
            self.serial_port = None
            self.get_logger().error(f"Failed to open {self.port_name}: {exc}")

    def poll_once(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            self.open_serial()
            return

        response = self.read_registers(0x0034, 12)
        if response is None:
            self.get_logger().warn("No valid IMU Modbus response.", throttle_duration_sec=2.0)
            return

        msg = self.decode_to_imu(response)
        self.publisher.publish(msg)

    def read_registers(self, start_reg: int, count: int) -> Optional[bytes]:
        assert self.serial_port is not None
        request = append_crc(
            bytes(
                (
                    self.address & 0xFF,
                    0x03,
                    (start_reg >> 8) & 0xFF,
                    start_reg & 0xFF,
                    0x00,
                    count & 0xFF,
                )
            )
        )
        self.serial_port.reset_input_buffer()
        self.serial_port.write(request)
        self.serial_port.flush()
        expected_len = 5 + 2 * count
        response = self.serial_port.read(expected_len)
        if len(response) != expected_len:
            return None
        if response[0] != (self.address & 0xFF) or response[1] != 0x03 or response[2] != 2 * count:
            return None
        if not verify_crc(response):
            return None
        return response

    def decode_to_imu(self, response: bytes) -> Imu:
        data = response[3:-2]
        values = [int16_be(data[i], data[i + 1]) for i in range(0, len(data), 2)]
        ax, ay, az, gx, gy, gz = values[0:6]
        roll, pitch, yaw = values[9:12]

        accel_scale = self.accel_range_g * G / 32768.0
        gyro_scale = math.radians(2000.0) / 32768.0
        angle_scale = math.radians(180.0) / 32768.0

        sign = -1.0 if self.invert_accel else 1.0
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.linear_acceleration.x = sign * ax * accel_scale
        msg.linear_acceleration.y = sign * ay * accel_scale
        msg.linear_acceleration.z = sign * az * accel_scale
        msg.angular_velocity.x = gx * gyro_scale
        msg.angular_velocity.y = gy * gyro_scale
        msg.angular_velocity.z = gz * gyro_scale

        if self.publish_orientation:
            msg.orientation = quaternion_from_rpy(
                roll * angle_scale,
                pitch * angle_scale,
                yaw * angle_scale - math.radians(self.yaw_zero),
            )
            msg.orientation_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.2]
        else:
            msg.orientation_covariance[0] = -1.0

        msg.angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
        msg.linear_acceleration_covariance = [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2]
        return msg

    def destroy_node(self) -> bool:
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Hwt601485ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
