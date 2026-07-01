#!/usr/bin/python3
"""Rewrite a ROS1 bag through rosbags so connection headers are normalized."""

from __future__ import annotations

import argparse
from pathlib import Path

from rosbags.rosbag1 import Reader, Writer


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("src", type=Path)
    parser.add_argument("dst", type=Path)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.dst.exists():
        raise SystemExit(f"output already exists: {args.dst}")
    args.dst.parent.mkdir(parents=True, exist_ok=True)

    connection_map = {}
    with Reader(args.src) as reader, Writer(args.dst) as writer:
        for conn in reader.connections:
            msgdef = conn.msgdef.data if hasattr(conn.msgdef, "data") else str(conn.msgdef)
            connection_map[conn.id] = writer.add_connection(
                conn.topic,
                conn.msgtype,
                msgdef=msgdef,
                md5sum=conn.digest,
                callerid=conn.ext.callerid,
                latching=conn.ext.latching,
            )

        count = 0
        for conn, timestamp, data in reader.messages():
            writer.write(connection_map[conn.id], timestamp, data)
            count += 1

    print(f"rewritten messages: {count}")
    print(f"output: {args.dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
