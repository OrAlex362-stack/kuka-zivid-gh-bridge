"""Development UDP-ordering fuzz harness for RobotUDPServer.

This uses the in-process packet handler so it is deterministic and does not
require real KUKA hardware. It asserts that accepted same-session poses never
move backward and that a new sender session starts with fresh pose history.
"""

from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_udp_server import RobotUDPServer  # noqa: E402


def _config() -> dict:
    return {
        "bind_ip": "127.0.0.1",
        "port": 0,
        "parser": "auto",
        "history_size": 100,
        "socket_timeout_ms": 20,
        "shutdown_timeout_ms": 1000,
        "max_packet_bytes": 16384,
        "connection_timeout_ms": 1000,
        "max_pose_age_ms": 1000,
        "settle_time_ms": 500,
        "max_translation_motion_mm": 0.5,
        "max_rotation_motion_deg": 0.2,
        "expected_sender_ip": None,
    }


def _packet(session_id: str, seq: int, *, malformed: bool = False) -> bytes:
    if malformed:
        return b"{not-json"
    matrix = np.eye(4)
    matrix[0, 3] = float(seq)
    return json.dumps(
        {
            "type": "POSE",
            "session_id": session_id,
            "seq": seq,
            "T_base_flange": matrix.tolist(),
            "matrix_name": "T_base_flange",
            "source_frame": "flange",
            "target_frame": "base",
            "units": "mm",
        }
    ).encode("utf-8")


def run(seed: int, count: int) -> dict:
    rng = random.Random(seed)
    server = RobotUDPServer(_config(), {"convention": None})
    packets: list[bytes] = [_packet("s1", seq) for seq in range(1, count + 1)]
    fuzzed: list[bytes] = []
    for packet in packets:
        if rng.random() < 0.1:
            continue
        fuzzed.append(packet)
        if rng.random() < 0.2:
            fuzzed.append(packet)
        if rng.random() < 0.05:
            fuzzed.append(_packet("s1", rng.randint(1, count)))
        if rng.random() < 0.03:
            fuzzed.append(_packet("s1", 0, malformed=True))
    rng.shuffle(fuzzed)
    for packet in fuzzed:
        server._handle_packet(packet, ("127.0.0.1", 49152))

    accepted = server.pose_history()
    last = -1
    for pose in accepted:
        if pose.session_id == "s1" and pose.seq is not None:
            if pose.seq <= last:
                raise AssertionError(f"accepted sequence moved backward: {pose.seq} <= {last}")
            last = pose.seq

    old_sample_count = len(server.pose_history())
    server._handle_packet(_packet("s2", 1), ("127.0.0.1", 49152))
    new_history = server.pose_history()
    if len(new_history) >= old_sample_count:
        raise AssertionError("new session inherited old pose history")
    return server.status()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--count", type=int, default=200)
    args = parser.parse_args()
    status = run(args.seed, args.count)
    print(json.dumps(status, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
