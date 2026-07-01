#!/usr/bin/python3
"""Probe a likely HWT601/WitMotion RS485 Modbus IMU."""

from __future__ import annotations

import argparse
import glob
import struct
import time
from dataclasses import dataclass
from typing import Iterable

import serial


DEFAULT_BAUDRATES = [9600, 115200, 57600, 38400, 19200]
DEFAULT_ADDRESSES = [0x50, 0x01, 0x02, 0x03, 0x04, 0x05]
G = 9.80665


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
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


def read_registers(port: serial.Serial, address: int, start_reg: int, count: int) -> bytes | None:
    request = append_crc(bytes((address, 0x03, (start_reg >> 8) & 0xFF, start_reg & 0xFF, 0x00, count)))
    port.reset_input_buffer()
    port.write(request)
    port.flush()
    expected_len = 5 + 2 * count
    response = port.read(expected_len)
    if len(response) != expected_len:
        return None
    if response[0] != address or response[1] != 0x03 or response[2] != 2 * count:
        return None
    if not verify_crc(response):
        return None
    return response


@dataclass
class ImuRegisters:
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    roll: float
    pitch: float
    yaw: float


def decode_wit_registers(response: bytes, accel_range_g: float) -> ImuRegisters:
    data = response[3:-2]
    values = [int16_be(data[i], data[i + 1]) for i in range(0, len(data), 2)]
    if len(values) < 12:
        raise ValueError("Expected at least 12 registers from 0x0034.")
    ax, ay, az, gx, gy, gz = values[0:6]
    roll, pitch, yaw = values[9:12]
    return ImuRegisters(
        ax=ax / 32768.0 * accel_range_g * G,
        ay=ay / 32768.0 * accel_range_g * G,
        az=az / 32768.0 * accel_range_g * G,
        gx=gx / 32768.0 * 2000.0,
        gy=gy / 32768.0 * 2000.0,
        gz=gz / 32768.0 * 2000.0,
        roll=roll / 32768.0 * 180.0,
        pitch=pitch / 32768.0 * 180.0,
        yaw=yaw / 32768.0 * 180.0,
    )


def candidate_ports() -> list[str]:
    ports = []
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"):
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))


def parse_int_list(text: str) -> list[int]:
    values = []
    for item in text.split(","):
        item = item.strip()
        if item:
            values.append(int(item, 0))
    return values


def parse_ports(text: str | None) -> list[str]:
    if text:
        return [item.strip() for item in text.split(",") if item.strip()]
    return candidate_ports()


def parse_baudrates(text: str | None) -> list[int]:
    if text:
        return parse_int_list(text)
    return DEFAULT_BAUDRATES


def parse_addresses(text: str | None) -> list[int]:
    if text:
        return parse_int_list(text)
    return DEFAULT_ADDRESSES


def probe(
    ports: Iterable[str],
    baudrates: Iterable[int],
    addresses: Iterable[int],
    timeout: float,
    accel_range_g: float,
) -> bool:
    found = False
    for device in ports:
        for baudrate in baudrates:
            try:
                with serial.Serial(device, baudrate=baudrate, bytesize=8, parity="N", stopbits=1, timeout=timeout) as ser:
                    print(f"scan port={device} baud={baudrate}")
                    time.sleep(0.05)
                    for address in addresses:
                        response = read_registers(ser, address, 0x0034, 12)
                        if response is None:
                            continue
                        decoded = decode_wit_registers(response, accel_range_g)
                        accel_norm = (decoded.ax**2 + decoded.ay**2 + decoded.az**2) ** 0.5
                        found = True
                        print(
                            "FOUND "
                            f"port={device} baud={baudrate} address=0x{address:02X} "
                            f"accel=[{decoded.ax:.3f}, {decoded.ay:.3f}, {decoded.az:.3f}] m/s^2 "
                            f"|accel|={accel_norm:.3f} m/s^2 "
                            f"gyro=[{decoded.gx:.3f}, {decoded.gy:.3f}, {decoded.gz:.3f}] deg/s "
                            f"rpy=[{decoded.roll:.3f}, {decoded.pitch:.3f}, {decoded.yaw:.3f}] deg"
                        )
            except serial.SerialException as exc:
                print(f"skip port={device} baud={baudrate}: {exc}")
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ports", help="Comma-separated serial ports. Default scans /dev/serial/by-id, /dev/ttyUSB*, /dev/ttyACM*.")
    parser.add_argument("--baudrates", help="Comma-separated baudrates. Default: 9600,115200,57600,38400,19200.")
    parser.add_argument("--addresses", help="Comma-separated Modbus addresses. Default: 0x50,0x01..0x05.")
    parser.add_argument(
        "--accel-range-g",
        type=float,
        default=4.0,
        help="Accelerometer full-scale range in g. HWT601-AGV-485 commonly behaves like 4g.",
    )
    parser.add_argument("--timeout", type=float, default=0.12, help="Serial read timeout in seconds.")
    args = parser.parse_args()

    ports = parse_ports(args.ports)
    if not ports:
        print("No serial ports found. Check USB-RS485 adapter, cable, and permissions.")
        return 2

    ok = probe(
        ports,
        parse_baudrates(args.baudrates),
        parse_addresses(args.addresses),
        args.timeout,
        args.accel_range_g,
    )
    if not ok:
        print("No valid HWT601/WitMotion Modbus response found.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
