"""Continuously send fake actual-pose packets; never sends robot commands."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
import time

import numpy as np
import yaml


def _load_destination(config_path: Path) -> tuple[str, int]:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    return str(config["robot_udp"]["bind_ip"]), int(config["robot_udp"]["port"])


def _matrix(x: float, y: float, z: float) -> list[list[float]]:
    value = np.eye(4, dtype=float)
    value[:3, 3] = [x, y, z]
    return value.tolist()


def build_packet(mode: str, seq: int, x: float, y: float, z: float) -> str:
    timestamp = time.time()
    if mode == "json-matrix":
        return json.dumps(
            {
                "type": "POSE",
                "seq": seq,
                "timestamp": timestamp,
                "T_base_flange": _matrix(x, y, z),
                "state": "READY",
            },
            separators=(",", ":"),
        )
    if mode == "json-abc":
        return json.dumps(
            {
                "type": "POSE",
                "seq": seq,
                "timestamp": timestamp,
                "x": x,
                "y": y,
                "z": z,
                "a": 0.0,
                "b": 0.0,
                "c": 0.0,
                "state": "READY",
            },
            separators=(",", ":"),
        )
    if mode == "delimited-matrix":
        flat = ",".join(str(item) for row in _matrix(x, y, z) for item in row)
        return f"MATRIX;SEQ={seq};TIMESTAMP={timestamp};T={flat};STATE=READY"
    return (
        f"POSE;SEQ={seq};TIMESTAMP={timestamp};X={x};Y={y};Z={z};"
        "A=0;B=0;C=0;STATE=READY"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config.yaml"))
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument(
        "--mode",
        choices=("json-matrix", "json-abc", "delimited-matrix", "delimited-abc"),
        default="json-matrix",
    )
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--x", type=float, default=1000.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=800.0)
    parser.add_argument(
        "--motion-amplitude-mm",
        type=float,
        default=0.0,
        help="Optional sinusoidal X motion for stationary-check testing.",
    )
    args = parser.parse_args()
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")

    config_host, config_port = _load_destination(args.config.resolve())
    host = args.host or config_host
    port = args.port or config_port
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / args.rate_hz
    started = time.monotonic()
    seq = 1
    print(f"Sending {args.mode} actual poses to {host}:{port} at {args.rate_hz:g} Hz")
    try:
        while True:
            x = args.x + args.motion_amplitude_mm * np.sin(2.0 * np.pi * (time.monotonic() - started))
            packet = build_packet(args.mode, seq, float(x), args.y, args.z)
            udp_socket.sendto(packet.encode("utf-8"), (host, port))
            seq += 1
            time.sleep(interval)
    except KeyboardInterrupt:
        return 0
    finally:
        udp_socket.close()


if __name__ == "__main__":
    sys.exit(main())
