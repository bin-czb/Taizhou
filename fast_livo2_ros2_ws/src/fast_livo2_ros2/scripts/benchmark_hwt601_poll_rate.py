#!/usr/bin/python3
"""Benchmark HWT601/WitMotion RS485 Modbus poll rates."""

from __future__ import annotations

import argparse
import struct
import time

import serial


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


def read_registers(port: serial.Serial, address: int, start_reg: int, count: int) -> bytes | None:
    request = append_crc(
        bytes(
            (
                address & 0xFF,
                0x03,
                (start_reg >> 8) & 0xFF,
                start_reg & 0xFF,
                0x00,
                count & 0xFF,
            )
        )
    )
    port.reset_input_buffer()
    port.write(request)
    port.flush()
    expected_len = 5 + 2 * count
    response = port.read(expected_len)
    if len(response) != expected_len:
        return None
    if response[0] != (address & 0xFF) or response[1] != 0x03 or response[2] != 2 * count:
        return None
    if not verify_crc(response):
        return None
    return response


def int16_be(high: int, low: int) -> int:
    return struct.unpack(">h", bytes((high, low)))[0]


def decode_short_summary(response: bytes) -> str:
    data = response[3:-2]
    values = [int16_be(data[i], data[i + 1]) for i in range(0, len(data), 2)]
    ax, ay, az, gx, gy, gz = values[0:6]
    roll, pitch, yaw = values[9:12]
    return (
        f"raw_acc=[{ax},{ay},{az}] raw_gyro=[{gx},{gy},{gz}] "
        f"raw_rpy=[{roll},{pitch},{yaw}]"
    )


def run_one_rate(args: argparse.Namespace, target_hz: float) -> None:
    period = 1.0 / target_hz
    valid = 0
    invalid = 0
    unique_payloads: set[bytes] = set()
    last_response = None
    started = time.monotonic()
    deadline = started + args.duration
    next_tick = started

    with serial.Serial(
        args.port,
        baudrate=args.baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=args.timeout,
    ) as ser:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
            next_tick += period

            response = read_registers(ser, args.address, args.start_register, args.register_count)
            if response is None:
                invalid += 1
                continue
            valid += 1
            payload = response[3:-2]
            unique_payloads.add(payload)
            last_response = response

    elapsed = time.monotonic() - started
    valid_hz = valid / elapsed if elapsed > 0.0 else 0.0
    attempted_hz = (valid + invalid) / elapsed if elapsed > 0.0 else 0.0
    unique_ratio = len(unique_payloads) / valid if valid else 0.0
    status = "OK" if valid_hz >= 0.9 * target_hz else "LIMITED"
    print(
        f"target={target_hz:6.1f} Hz  attempted={attempted_hz:6.1f} Hz  "
        f"valid={valid_hz:6.1f} Hz  valid_samples={valid:4d}  invalid={invalid:4d}  "
        f"unique_payloads={len(unique_payloads):4d}  unique_ratio={unique_ratio:5.2f}  {status}"
    )
    if last_response is not None:
        print(f"  last: {decode_short_summary(last_response)}")


def parse_rates(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x50)
    parser.add_argument("--rates", default="50,100,150,200")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--timeout", type=float, default=0.02)
    parser.add_argument("--start-register", type=lambda value: int(value, 0), default=0x0034)
    parser.add_argument("--register-count", type=int, default=12)
    args = parser.parse_args()

    print(
        f"Benchmarking {args.port}, baud={args.baudrate}, address=0x{args.address:02X}, "
        f"duration={args.duration:.1f}s per rate"
    )
    for target_hz in parse_rates(args.rates):
        run_one_rate(args, target_hz)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
