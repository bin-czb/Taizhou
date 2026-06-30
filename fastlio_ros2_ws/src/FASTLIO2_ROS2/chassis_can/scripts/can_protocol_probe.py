#!/usr/bin/python3
import argparse
import csv
import os
import select
import socket
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


@dataclass(frozen=True)
class Command:
    name: str
    function: int
    data: int


def parse_int(text: str) -> int:
    return int(text, 0)


def crc16_modbus(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def payload_for(function: int, data_value: int) -> bytes:
    data_value &= 0xFFFF
    body = bytes([function & 0xFF, (data_value >> 8) & 0xFF, data_value & 0xFF])
    crc = crc16_modbus(body)
    return body + bytes([(crc >> 8) & 0xFF, crc & 0xFF])


def pack_frame(can_id: int, payload: bytes) -> bytes:
    if len(payload) > 8:
        raise ValueError("CAN payload cannot exceed 8 bytes")
    return struct.pack(CAN_FRAME_FMT, can_id & CAN_SFF_MASK, len(payload), payload.ljust(8, b"\x00"))


def unpack_frame(raw: bytes) -> tuple[int, int, bytes, bool, bool, bool]:
    can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
    is_extended = bool(can_id & CAN_EFF_FLAG)
    is_rtr = bool(can_id & CAN_RTR_FLAG)
    is_error = bool(can_id & CAN_ERR_FLAG)
    frame_id = can_id & (CAN_EFF_MASK if is_extended else CAN_SFF_MASK)
    return frame_id, dlc, data[:dlc], is_extended, is_rtr, is_error


def u16_be(data: bytes, offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


def u16_le(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def i16_from_u16(value: int) -> int:
    return value - 0x10000 if value >= 0x8000 else value


def hex16(value: int) -> str:
    return f"0x{int(value) & 0xFFFF:04X}"


def frame_text(can_id: int, payload: bytes) -> str:
    return f"{can_id:03X}#{payload.hex().upper()}"


def decode_feedback(payload: bytes) -> dict[str, object]:
    decoded: dict[str, object] = {
        "feedback_kind": "",
        "fb_function": "",
        "big_board_data_hex": "",
        "big_board_data_u16": "",
        "big_board_data_i16": "",
        "left_angle_hex": "",
        "left_angle_u16": "",
        "left_angle_i16": "",
        "right_angle_hex": "",
        "right_angle_u16": "",
        "right_angle_i16": "",
        "left_speed_hex": "",
        "left_speed_u16": "",
        "left_speed_i16": "",
        "right_speed_hex": "",
        "right_speed_u16": "",
        "right_speed_i16": "",
        "data1_hex": "",
        "data1_u16": "",
        "data1_i16": "",
        "data2_hex": "",
        "data2_u16": "",
        "data2_i16": "",
        "feedback_crc": "",
        "feedback_crc_expected": "",
        "feedback_crc_ok": "",
    }
    if len(payload) >= 8:
        left_angle = u16_be(payload, 0)
        right_angle = u16_be(payload, 2)
        left_speed = u16_be(payload, 4)
        right_speed = u16_be(payload, 6)
        decoded.update(
            {
                "feedback_kind": "small_board",
                "left_angle_hex": hex16(left_angle),
                "left_angle_u16": left_angle,
                "left_angle_i16": i16_from_u16(left_angle),
                "right_angle_hex": hex16(right_angle),
                "right_angle_u16": right_angle,
                "right_angle_i16": i16_from_u16(right_angle),
                "left_speed_hex": hex16(left_speed),
                "left_speed_u16": left_speed,
                "left_speed_i16": i16_from_u16(left_speed),
                "right_speed_hex": hex16(right_speed),
                "right_speed_u16": right_speed,
                "right_speed_i16": i16_from_u16(right_speed),
            }
        )
        return decoded

    if len(payload) < 5:
        return decoded

    big_data = u16_be(payload, 1)
    actual = u16_be(payload, 3)
    expected = crc16_modbus(payload[:3])
    decoded.update(
        {
            "feedback_kind": "big_board",
            "fb_function": f"0x{payload[0]:02X}",
            "big_board_data_hex": hex16(big_data),
            "big_board_data_u16": big_data,
            "big_board_data_i16": i16_from_u16(big_data),
            "feedback_crc": f"0x{actual:04X}",
            "feedback_crc_expected": f"0x{expected:04X}",
            "feedback_crc_ok": actual == expected,
        }
    )
    return decoded


def feedback_summary(payload: bytes) -> str:
    decoded = decode_feedback(payload)
    if decoded["feedback_kind"] == "small_board":
        return (
            "small_board hex_be "
            f"L_angle={decoded['left_angle_hex']} "
            f"R_angle={decoded['right_angle_hex']} "
            f"L_speed={decoded['left_speed_hex']} "
            f"R_speed={decoded['right_speed_hex']}"
        )

    if decoded["feedback_kind"] == "big_board":
        return (
            f"big_board func={decoded['fb_function']} "
            f"DATA={decoded['big_board_data_hex']} "
            f"CRC={decoded['feedback_crc']} expected={decoded['feedback_crc_expected']} ok={decoded['feedback_crc_ok']}"
        )

    if not decoded["fb_function"]:
        return "short feedback"
    text = (
        f"func={decoded['fb_function']} "
        f"DATA1={decoded['data1_hex']} "
        f"DATA2={decoded['data2_hex']}"
    )
    if decoded["feedback_crc"]:
        text += (
            f" CRC={decoded['feedback_crc']} "
            f"expected={decoded['feedback_crc_expected']} ok={decoded['feedback_crc_ok']}"
        )
    else:
        text += " CRC=n/a"
    return text


def default_commands(args: argparse.Namespace) -> list[Command]:
    if args.profile == "safe":
        return [
            Command("mode_upper", 0x00, 1),
            Command("total_power_low", 0x01, args.low_value),
            Command("forward_neutral", 0x02, args.neutral_value),
            Command("turn_neutral", 0x03, args.neutral_value),
            Command("light_low", 0x04, args.low_value),
            Command("pump_low", 0x05, args.low_value),
            Command("fan_low", 0x06, args.low_value),
            Command("pump_fan_disable", 0x07, args.low_value),
            Command("motor_data_query", 0x08, 0),
        ]

    commands = [
        Command("mode_upper", 0x00, 1),
        Command("total_power", 0x01, args.power_data),
        Command("forward", 0x02, args.forward_data),
        Command("turn", 0x03, args.turn_data),
        Command("light", 0x04, args.light_data),
        Command("pump", 0x05, args.pump_data),
        Command("fan", 0x06, args.fan_data),
        Command("pump_fan_enable", 0x07, args.pump_fan_enable_data),
        Command("motor_data_query", 0x08, args.query_data),
    ]
    if args.stop_at_end:
        commands.extend(
            [
                Command("forward_neutral_end", 0x02, args.neutral_value),
                Command("turn_neutral_end", 0x03, args.neutral_value),
            ]
        )
    return commands


def extra_commands(values: Optional[Iterable[str]]) -> list[Command]:
    commands = []
    for item in values or []:
        parts = item.split(",")
        if len(parts) != 3:
            raise ValueError("--extra-command format: name,function,data")
        name, function_text, data_text = parts
        commands.append(Command(name.strip(), parse_int(function_text), parse_int(data_text)))
    return commands


def require_confirmation(args: argparse.Namespace, commands: list[Command]) -> None:
    def is_active(cmd: Command) -> bool:
        data = cmd.data & 0xFFFF
        if cmd.function in (0x02, 0x03):
            return data != args.neutral_value
        if cmd.function in (0x01, 0x04, 0x05, 0x06, 0x07):
            return data != args.low_value
        return False

    active = [cmd for cmd in commands if is_active(cmd)]
    if active and not args.yes:
        names = ", ".join(f"{cmd.name}(0x{cmd.function:02X}=0x{cmd.data & 0xFFFF:04X})" for cmd in active)
        raise SystemExit(
            "Refuse to send active power/motion/pump/fan commands without --yes. "
            f"Potentially active commands: {names}"
        )


class CanProbe:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
        except (AttributeError, OSError):
            pass
        self.sock.bind((args.interface,))
        self.sock.setblocking(False)

    def drain(self) -> None:
        while True:
            readable, _, _ = select.select([self.sock], [], [], 0.0)
            if not readable:
                return
            try:
                self.sock.recv(CAN_FRAME_SIZE)
            except BlockingIOError:
                return

    def receive_until(self, deadline: float) -> list[tuple[float, int, bytes]]:
        frames = []
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return frames
            readable, _, _ = select.select([self.sock], [], [], remaining)
            if not readable:
                return frames
            raw = self.sock.recv(CAN_FRAME_SIZE)
            frame_id, _dlc, payload, _eff, _rtr, _err = unpack_frame(raw)
            frames.append((time.time(), frame_id, payload))

    def send(self, command: Command) -> tuple[bytes, list[tuple[float, int, bytes]]]:
        payload = payload_for(command.function, command.data)
        self.drain()
        self.sock.send(pack_frame(self.args.command_id, payload))
        deadline = time.monotonic() + self.args.timeout
        frames = self.receive_until(deadline)
        time.sleep(self.args.delay)
        return payload, frames

    def close(self) -> None:
        self.sock.close()


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe the small chassis CAN protocol.")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--command-id", type=parse_int, default=0x001)
    parser.add_argument("--feedback-id", type=parse_int, default=0x002)
    parser.add_argument("--timeout", type=float, default=0.35, help="Seconds to wait after each TX.")
    parser.add_argument("--delay", type=float, default=0.08, help="Seconds to wait between commands.")
    parser.add_argument("--profile", choices=["safe", "active"], default="safe")
    parser.add_argument("--yes", action="store_true", help="Allow non-zero motor/pump/fan commands.")
    parser.add_argument("--low-value", type=parse_int, default=282)
    parser.add_argument("--neutral-value", type=parse_int, default=1002)
    parser.add_argument("--high-value", type=parse_int, default=1722)
    parser.add_argument("--power-data", type=parse_int, default=1722)
    parser.add_argument("--forward-data", type=parse_int, default=1002)
    parser.add_argument("--turn-data", type=parse_int, default=1002)
    parser.add_argument("--light-data", type=parse_int, default=282)
    parser.add_argument("--pump-data", type=parse_int, default=282)
    parser.add_argument("--fan-data", type=parse_int, default=282)
    parser.add_argument("--pump-fan-enable-data", type=parse_int, default=282)
    parser.add_argument("--query-data", type=parse_int, default=0)
    parser.add_argument("--no-stop-at-end", dest="stop_at_end", action="store_false")
    parser.set_defaults(stop_at_end=True)
    parser.add_argument(
        "--extra-command",
        action="append",
        help="Append a command as name,function,data, for example right_800,0x02,800.",
    )
    parser.add_argument(
        "--csv",
        default="",
        help="CSV output path. Default: /tmp/chassis_can_probe_<timestamp>.csv",
    )
    args = parser.parse_args()

    commands = default_commands(args) + extra_commands(args.extra_command)
    require_confirmation(args, commands)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = Path(args.csv) if args.csv else Path("/tmp") / f"chassis_can_probe_{timestamp}.csv"

    print(f"Interface: {args.interface}")
    print(f"Command ID: 0x{args.command_id:03X}, feedback ID: 0x{args.feedback_id:03X}")
    print(f"Profile: {args.profile}")
    print(f"CSV: {csv_path}")
    print("CRC: MODBUS over [function, data_hi, data_lo], output high byte first")
    print()

    rows: list[dict[str, object]] = []
    probe = CanProbe(args)
    try:
        for index, command in enumerate(commands, start=1):
            payload, frames = probe.send(command)
            tx_crc = u16_be(payload, 3)
            tx_time = time.time()
            print(
                f"[{index:02d}/{len(commands):02d}] TX {command.name}: "
                f"{frame_text(args.command_id, payload)} "
                f"func=0x{command.function:02X} data=0x{command.data & 0xFFFF:04X} crc=0x{tx_crc:04X}"
            )
            rows.append(
                {
                    "time": f"{tx_time:.6f}",
                    "command_name": command.name,
                    "direction": "TX",
                    "can_id": f"0x{args.command_id:03X}",
                    "dlc": len(payload),
                    "data_hex": payload.hex().upper(),
                    "function": f"0x{command.function:02X}",
                    "command_data": f"0x{command.data & 0xFFFF:04X}",
                    "fb_function": "",
                    "data1_hex": "",
                    "data2_hex": "",
                    "feedback_crc": "",
                    "feedback_crc_expected": "",
                    "feedback_crc_ok": "",
                }
            )

            feedback_count = 0
            for rx_time, frame_id, rx_payload in frames:
                direction = "RX_FEEDBACK" if frame_id == args.feedback_id else "RX_OTHER"
                decoded = decode_feedback(rx_payload) if frame_id == args.feedback_id else {}
                if frame_id == args.feedback_id:
                    feedback_count += 1
                summary = feedback_summary(rx_payload) if frame_id == args.feedback_id else ""
                print(
                    f"    {direction} {frame_text(frame_id, rx_payload)} "
                    f"dlc={len(rx_payload)} {summary}"
                )
                rows.append(
                    {
                        "time": f"{rx_time:.6f}",
                        "command_name": command.name,
                        "direction": direction,
                        "can_id": f"0x{frame_id:03X}",
                        "dlc": len(rx_payload),
                        "data_hex": rx_payload.hex().upper(),
                        "function": f"0x{command.function:02X}",
                        "command_data": f"0x{command.data & 0xFFFF:04X}",
                        "feedback_kind": decoded.get("feedback_kind", ""),
                        "fb_function": decoded.get("fb_function", ""),
                        "big_board_data_hex": decoded.get("big_board_data_hex", ""),
                        "left_angle_hex": decoded.get("left_angle_hex", ""),
                        "right_angle_hex": decoded.get("right_angle_hex", ""),
                        "left_speed_hex": decoded.get("left_speed_hex", ""),
                        "right_speed_hex": decoded.get("right_speed_hex", ""),
                        "data1_hex": decoded.get("data1_hex", ""),
                        "data2_hex": decoded.get("data2_hex", ""),
                        "feedback_crc": decoded.get("feedback_crc", ""),
                        "feedback_crc_expected": decoded.get("feedback_crc_expected", ""),
                        "feedback_crc_ok": decoded.get("feedback_crc_ok", ""),
                    }
                )
            if feedback_count == 0:
                print("    no 0x002 feedback in timeout window")
    finally:
        probe.close()

    write_rows(csv_path, rows)
    print()
    print(f"Wrote CSV: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
