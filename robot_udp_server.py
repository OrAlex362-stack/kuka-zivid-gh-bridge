"""Background-only UDP receiver for actual KUKA state/pose messages.

This module has no code for sending robot commands.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from datetime import datetime, timezone
import json
import logging
import socket
import threading
import time
from typing import Any, Callable

import numpy as np
from numpy.typing import NDArray

from bridge_errors import BridgeError
from kuka_pose import kuka_abc_to_matrix
from transform_utils import (
    matrix_to_list,
    rotation_distance_deg,
    translation_distance_mm,
    validate_transform,
)


LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class RobotPose:
    seq: int | None
    robot_timestamp: int | float | str | None
    received_timestamp_utc: str
    received_monotonic: float
    x: float | None
    y: float | None
    z: float | None
    a: float | None
    b: float | None
    c: float | None
    T_base_flange: NDArray[np.float64] | None
    state: str | None
    raw_message: str

    @property
    def matrix_valid(self) -> bool:
        return self.T_base_flange is not None

    def to_dict(self, *, include_raw_message: bool = False) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "seq": self.seq,
            "robot_timestamp": self.robot_timestamp,
            "received_timestamp_utc": self.received_timestamp_utc,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "a": self.a,
            "b": self.b,
            "c": self.c,
            "state": self.state,
            "matrix_valid": self.matrix_valid,
            "T_base_flange": (
                matrix_to_list(self.T_base_flange) if self.T_base_flange is not None else None
            ),
        }
        if include_raw_message:
            payload["raw_message"] = self.raw_message
        return payload


@dataclass(frozen=True, slots=True)
class StationaryStatus:
    exists: bool
    fresh: bool
    stationary: bool
    pose_age_ms: float | None
    history_duration_ms: float
    sample_count: int
    max_translation_motion_mm: float | None
    max_rotation_motion_deg: float | None
    reason: str | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "exists": self.exists,
            "fresh": self.fresh,
            "stationary": self.stationary,
            "pose_age_ms": self.pose_age_ms,
            "history_duration_ms": self.history_duration_ms,
            "sample_count": self.sample_count,
            "max_translation_motion_mm": self.max_translation_motion_mm,
            "max_rotation_motion_deg": self.max_rotation_motion_deg,
            "reason": self.reason,
        }


def _received_times(
    received_timestamp_utc: str | None,
    received_monotonic: float | None,
) -> tuple[str, float]:
    return (
        received_timestamp_utc or datetime.now(timezone.utc).isoformat(),
        time.monotonic() if received_monotonic is None else float(received_monotonic),
    )


def _optional_float(payload: dict[str, Any], key: str) -> float | None:
    value = payload.get(key)
    return None if value is None else float(value)


def _optional_seq(value: Any) -> int | None:
    return None if value in (None, "") else int(value)


def _pose_from_mapping(
    payload: dict[str, Any],
    raw_message: str,
    convention: str | None,
    *,
    received_timestamp_utc: str | None = None,
    received_monotonic: float | None = None,
) -> RobotPose:
    normalized = {str(key).lower(): value for key, value in payload.items()}
    message_type = str(normalized.get("type", "POSE")).upper()
    if message_type not in {"POSE", "MATRIX"}:
        raise ValueError(f"Unsupported UDP message type '{message_type}'.")

    utc_timestamp, monotonic_timestamp = _received_times(
        received_timestamp_utc, received_monotonic
    )
    matrix_value = normalized.get("t_base_flange", normalized.get("matrix"))
    T_base_flange: NDArray[np.float64] | None
    if matrix_value is not None:
        if isinstance(matrix_value, str):
            tokens = [item.strip() for item in matrix_value.replace(" ", ",").split(",")]
            matrix_value = [float(item) for item in tokens if item]
        array = np.asarray(matrix_value, dtype=np.float64)
        if array.size == 16:
            array = array.reshape(4, 4)
        T_base_flange = validate_transform(array, name="T_base_flange from UDP")
    else:
        required = ("x", "y", "z", "a", "b", "c")
        missing = [key.upper() for key in required if normalized.get(key) is None]
        if missing:
            raise ValueError(f"Missing pose fields: {', '.join(missing)}.")
        if convention is None or not str(convention).strip():
            T_base_flange = None
        else:
            T_base_flange = kuka_abc_to_matrix(
                *[float(normalized[key]) for key in required], convention=convention
            )

    return RobotPose(
        seq=_optional_seq(normalized.get("seq")),
        robot_timestamp=normalized.get("timestamp"),
        received_timestamp_utc=utc_timestamp,
        received_monotonic=monotonic_timestamp,
        x=_optional_float(normalized, "x"),
        y=_optional_float(normalized, "y"),
        z=_optional_float(normalized, "z"),
        a=_optional_float(normalized, "a"),
        b=_optional_float(normalized, "b"),
        c=_optional_float(normalized, "c"),
        T_base_flange=T_base_flange,
        state=None if normalized.get("state") is None else str(normalized["state"]),
        raw_message=raw_message,
    )


def parse_json_packet(
    message: str | bytes,
    convention: str | None,
    *,
    received_timestamp_utc: str | None = None,
    received_monotonic: float | None = None,
) -> RobotPose:
    raw_message = message.decode("utf-8") if isinstance(message, bytes) else message
    payload = json.loads(raw_message)
    if not isinstance(payload, dict):
        raise ValueError("JSON robot packet must contain an object.")
    return _pose_from_mapping(
        payload,
        raw_message,
        convention,
        received_timestamp_utc=received_timestamp_utc,
        received_monotonic=received_monotonic,
    )


def parse_delimited_packet(
    message: str | bytes,
    convention: str | None,
    *,
    received_timestamp_utc: str | None = None,
    received_monotonic: float | None = None,
) -> RobotPose:
    raw_message = message.decode("utf-8") if isinstance(message, bytes) else message
    tokens = [token.strip() for token in raw_message.strip().split(";") if token.strip()]
    if not tokens:
        raise ValueError("Empty robot packet.")
    payload: dict[str, Any] = {"type": tokens[0].upper()}
    for token in tokens[1:]:
        if "=" not in token:
            raise ValueError(f"Malformed delimited field '{token}'.")
        key, value = token.split("=", 1)
        payload[key.strip().lower()] = value.strip()

    if "t" in payload:
        payload["t_base_flange"] = payload.pop("t")
    elif all(f"m{row}{column}" in payload for row in range(4) for column in range(4)):
        payload["t_base_flange"] = [
            float(payload[f"m{row}{column}"]) for row in range(4) for column in range(4)
        ]
    return _pose_from_mapping(
        payload,
        raw_message,
        convention,
        received_timestamp_utc=received_timestamp_utc,
        received_monotonic=received_monotonic,
    )


PARSERS: dict[str, Callable[..., RobotPose]] = {
    "json": parse_json_packet,
    "delimited": parse_delimited_packet,
}


def parse_robot_packet(
    message: str | bytes,
    parser: str,
    convention: str | None,
    *,
    received_timestamp_utc: str | None = None,
    received_monotonic: float | None = None,
) -> RobotPose:
    selected = parser.lower()
    raw = message.decode("utf-8") if isinstance(message, bytes) else message
    if selected == "auto":
        selected = "json" if raw.lstrip().startswith("{") else "delimited"
    if selected not in PARSERS:
        raise ValueError(f"Unknown robot UDP parser '{parser}'.")
    return PARSERS[selected](
        raw,
        convention,
        received_timestamp_utc=received_timestamp_utc,
        received_monotonic=received_monotonic,
    )


def poses_stationary(
    poses: list[RobotPose],
    *,
    now_monotonic: float,
    max_pose_age_ms: float,
    settle_time_ms: float,
    max_translation_motion_mm: float,
    max_rotation_motion_deg: float,
) -> StationaryStatus:
    if not poses:
        return StationaryStatus(False, False, False, None, 0.0, 0, None, None, "NO_ROBOT_POSE")

    latest = poses[-1]
    age_ms = max(0.0, (now_monotonic - latest.received_monotonic) * 1000.0)
    fresh = age_ms <= max_pose_age_ms
    if not fresh:
        return StationaryStatus(
            True, False, False, age_ms, 0.0, 0, None, None, "ROBOT_POSE_STALE"
        )
    if latest.T_base_flange is None:
        return StationaryStatus(
            True, True, False, age_ms, 0.0, 0, None, None, "KUKA_CONVENTION_UNVERIFIED"
        )

    cutoff = now_monotonic - settle_time_ms / 1000.0
    matrix_poses = [pose for pose in poses if pose.T_base_flange is not None]
    if not matrix_poses or (settle_time_ms > 0 and matrix_poses[0].received_monotonic > cutoff):
        duration_ms = (
            0.0
            if not matrix_poses
            else (latest.received_monotonic - matrix_poses[0].received_monotonic) * 1000.0
        )
        return StationaryStatus(
            True, True, False, age_ms, duration_ms, len(matrix_poses), None, None, "ROBOT_SETTLING"
        )

    if settle_time_ms <= 0:
        window = [latest]
    else:
        before_cutoff = [pose for pose in matrix_poses if pose.received_monotonic < cutoff]
        window = [pose for pose in matrix_poses if pose.received_monotonic >= cutoff]
        # Retain the closest sample before the boundary. Without this anchor, a
        # jump occurring across the cutoff can disappear from the settle window.
        if before_cutoff:
            window.insert(0, before_cutoff[-1])
        if not window:
            window = [latest]
    transforms = [pose.T_base_flange for pose in window if pose.T_base_flange is not None]
    maximum_translation = 0.0
    maximum_rotation = 0.0
    for index, first in enumerate(transforms):
        for second in transforms[index + 1 :]:
            maximum_translation = max(maximum_translation, translation_distance_mm(first, second))
            maximum_rotation = max(maximum_rotation, rotation_distance_deg(first, second))
    stationary = (
        maximum_translation <= max_translation_motion_mm
        and maximum_rotation <= max_rotation_motion_deg
    )
    duration_ms = (latest.received_monotonic - window[0].received_monotonic) * 1000.0
    return StationaryStatus(
        True,
        True,
        stationary,
        age_ms,
        duration_ms,
        len(window),
        maximum_translation,
        maximum_rotation,
        None if stationary else "ROBOT_MOVING",
    )


class RobotUDPServer:
    """Continuously receive actual robot poses without blocking HTTP requests."""

    def __init__(self, config: dict[str, Any], kuka_pose_config: dict[str, Any]) -> None:
        self.config = config
        self.convention = kuka_pose_config.get("convention")
        self._history: deque[RobotPose] = deque(maxlen=int(config["history_size"]))
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self._running = False
        self._last_sender: tuple[str, int] | None = None
        self._last_datagram_monotonic: float | None = None
        self._received_packets = 0
        self._malformed_packets = 0
        self._last_error: str | None = None

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, name="robot-udp", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        udp_socket = self._socket
        if udp_socket is not None:
            try:
                udp_socket.close()
            except OSError:
                pass
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=float(self.config["shutdown_timeout_ms"]) / 1000.0)
        LOGGER.info("UDP robot receiver stopped")

    def _run(self) -> None:
        bind_ip = str(self.config["bind_ip"])
        port = int(self.config["port"])
        try:
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.settimeout(float(self.config["socket_timeout_ms"]) / 1000.0)
            udp_socket.bind((bind_ip, port))
            self._socket = udp_socket
            self._running = True
            LOGGER.info("UDP robot receiver listening on %s:%d", bind_ip, port)
            while not self._stop_event.is_set():
                try:
                    packet, sender = udp_socket.recvfrom(int(self.config["max_packet_bytes"]))
                except socket.timeout:
                    continue
                except OSError:
                    if self._stop_event.is_set():
                        break
                    raise
                self._handle_packet(packet, sender)
        except Exception as exc:  # The background receiver must never kill the service.
            with self._lock:
                self._last_error = str(exc)
            LOGGER.exception("UDP robot receiver failed")
        finally:
            self._running = False
            self._socket = None

    def _handle_packet(self, packet: bytes, sender: tuple[str, int]) -> None:
        try:
            pose = parse_robot_packet(
                packet,
                str(self.config["parser"]),
                self.convention,
            )
        except Exception as exc:
            with self._lock:
                self._malformed_packets += 1
                self._last_error = str(exc)
                self._last_datagram_monotonic = time.monotonic()
                self._last_sender = sender
            LOGGER.warning("Rejected malformed robot UDP packet from %s:%d: %s", *sender, exc)
            return
        with self._lock:
            was_connected = self._connected_locked(time.monotonic())
            self._history.append(pose)
            self._received_packets += 1
            self._last_datagram_monotonic = pose.received_monotonic
            self._last_sender = sender
            self._last_error = None
        if not was_connected:
            LOGGER.info("Robot UDP stream connected from %s:%d", *sender)

    def inject_pose(self, pose: RobotPose) -> None:
        """Hardware-independent hook for tests and local integration harnesses."""
        copied = replace(
            pose,
            T_base_flange=(None if pose.T_base_flange is None else pose.T_base_flange.copy()),
        )
        with self._lock:
            self._history.append(copied)
            self._received_packets += 1
            self._last_datagram_monotonic = copied.received_monotonic
            self._last_sender = ("mock", 0)

    def latest_pose(self) -> RobotPose | None:
        with self._lock:
            if not self._history:
                return None
            pose = self._history[-1]
            return replace(
                pose,
                T_base_flange=(None if pose.T_base_flange is None else pose.T_base_flange.copy()),
            )

    def pose_history(self) -> list[RobotPose]:
        with self._lock:
            return [
                replace(
                    pose,
                    T_base_flange=(
                        None if pose.T_base_flange is None else pose.T_base_flange.copy()
                    ),
                )
                for pose in self._history
            ]

    def stationary_status(self, *, now_monotonic: float | None = None) -> StationaryStatus:
        return poses_stationary(
            self.pose_history(),
            now_monotonic=time.monotonic() if now_monotonic is None else now_monotonic,
            max_pose_age_ms=float(self.config["max_pose_age_ms"]),
            settle_time_ms=float(self.config["settle_time_ms"]),
            max_translation_motion_mm=float(self.config["max_translation_motion_mm"]),
            max_rotation_motion_deg=float(self.config["max_rotation_motion_deg"]),
        )

    def require_stationary_pose(self) -> RobotPose:
        status = self.stationary_status()
        if not status.exists:
            raise BridgeError("NO_ROBOT_POSE", "No robot pose has been received.", status_code=503)
        if not status.fresh:
            raise BridgeError(
                "ROBOT_POSE_STALE",
                "The latest robot pose is stale.",
                status_code=409,
                details=status.to_dict(),
            )
        if status.reason == "KUKA_CONVENTION_UNVERIFIED":
            raise BridgeError(
                "KUKA_CONVENTION_UNVERIFIED",
                "KUKA rotation convention has not been verified.",
                status_code=422,
            )
        if not status.stationary:
            message = (
                "Robot pose history has not covered the configured settle time."
                if status.reason == "ROBOT_SETTLING"
                else "Robot is moving beyond the configured stationary tolerance."
            )
            raise BridgeError(status.reason or "ROBOT_MOVING", message, details=status.to_dict())
        pose = self.latest_pose()
        if pose is None or pose.T_base_flange is None:  # defensive against concurrent reset
            raise BridgeError("NO_VALID_ROBOT_POSE", "No valid robot transform is available.")
        return pose

    def movement_between(self, before: RobotPose, after: RobotPose) -> dict[str, Any]:
        if before.T_base_flange is None or after.T_base_flange is None:
            return {"valid": False, "moved": True, "reason": "NO_VALID_ROBOT_TRANSFORM"}
        translation = translation_distance_mm(before.T_base_flange, after.T_base_flange)
        rotation = rotation_distance_deg(before.T_base_flange, after.T_base_flange)
        moved = (
            translation > float(self.config["max_translation_motion_mm"])
            or rotation > float(self.config["max_rotation_motion_deg"])
        )
        return {
            "valid": True,
            "moved": moved,
            "translation_motion_mm": translation,
            "rotation_motion_deg": rotation,
        }

    def _connected_locked(self, now_monotonic: float) -> bool:
        if self._last_datagram_monotonic is None:
            return False
        timeout_ms = float(self.config["connection_timeout_ms"])
        return (now_monotonic - self._last_datagram_monotonic) * 1000.0 <= timeout_ms

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        stationary = self.stationary_status(now_monotonic=now)
        latest = self.latest_pose()
        with self._lock:
            connected = self._connected_locked(now)
            sender = self._last_sender
            counters = (self._received_packets, self._malformed_packets, self._last_error)
        raw_pose = None
        if latest is not None:
            raw_pose = {
                "x": latest.x,
                "y": latest.y,
                "z": latest.z,
                "a": latest.a,
                "b": latest.b,
                "c": latest.c,
            }
        return {
            "connected": connected,
            "receiver_running": self._running,
            "pose_valid": latest is not None and latest.matrix_valid and stationary.fresh,
            "pose_age_ms": stationary.pose_age_ms,
            "stationary": stationary.stationary,
            "stationary_reason": stationary.reason,
            "seq": None if latest is None else latest.seq,
            "state": None if latest is None else latest.state,
            "raw_pose": raw_pose,
            "T_base_flange": (
                None
                if latest is None or latest.T_base_flange is None
                else matrix_to_list(latest.T_base_flange)
            ),
            "sender": None if sender is None else {"ip": sender[0], "port": sender[1]},
            "received_packets": counters[0],
            "malformed_packets": counters[1],
            "last_error": counters[2],
        }
