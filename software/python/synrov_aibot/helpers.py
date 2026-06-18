"""Shared utility functions for text, numeric safety and pose handling."""
from __future__ import annotations

from typing import Any, Dict, Iterable, List

from .primitives import json_compatible, normalize_text, safe_float, to_list
from .dataset import canonical_robot
from .safety import MANIP_KEYS, MANIP_POSE_LIMITS, clamp


def mean_abs(values: Iterable[Any]) -> float:
    vals = [abs(safe_float(v)) for v in to_list(values)]
    return sum(vals) / len(vals) if vals else 0.0


def has_any_token(text: Any, tokens: Iterable[str]) -> bool:
    """Return True if any non-empty normalised token appears in *text*."""
    norm = normalize_text(text)
    return any(token in norm for token in (normalize_text(t) for t in to_list(tokens)) if token)


def voice_vector(text: Any, size: int = 12) -> List[float]:
    try:
        length = max(0, int(size))
    except Exception:
        length = 12
    if length <= 0:
        return []

    vec = [0.0] * length
    for i, byte in enumerate(normalize_text(text).encode("utf-8")):
        vec[i % length] += byte / 255.0
    scale = max(1.0, max(abs(v) for v in vec))
    return [v / scale for v in vec]


def format_float(value: Any, digits: int = 2) -> str:
    return f"{safe_float(value):.{int(digits)}f}"


def wrap_signed(value: Any) -> float:
    return ((safe_float(value) + 180.0) % 360.0) - 180.0


def wrap_abs(value: Any) -> float:
    return safe_float(value) % 360.0


_TRAINING_RATE_LIMITS: List[float] = [180.0, 150.0, 150.0, 180.0, 160.0, 180.0, 120.0]


def clamp_manipulator_rate_vector(values: Iterable[Any]) -> List[float]:
    vals = to_list(values)[:7]
    vals.extend([0.0] * max(0, 7 - len(vals)))
    return [clamp(v, -limit, limit) for v, limit in zip(vals, _TRAINING_RATE_LIMITS)]


def manipulator_rate_from_delta_hint(delta: Iterable[Any], dt: float = 0.2) -> List[float]:
    dt_s = max(0.05, min(1.0, safe_float(dt, 0.2)))
    return clamp_manipulator_rate_vector(safe_float(v) / dt_s for v in to_list(delta))


def normalize_training_target(robot: Any, values: Iterable[Any]) -> List[float]:
    name = canonical_robot(robot)
    size = 7 if name == "Manipulator" else 6 if name == "Drone" else 4
    vals = [safe_float(v) for v in to_list(values)[:size]]
    vals.extend([0.0] * (size - len(vals)))
    return vals


def inference_dt_from_period_ms(period_ms: Any) -> float:
    return max(0.12, min(5.0, safe_float(period_ms, 500.0) / 1000.0))


def clamp_member_pose(key: str, value: Any) -> float:
    return clamp(value, *MANIP_POSE_LIMITS.get(key, (0.0, 359.0)))


def normalize_robot_name(value: Any) -> str:
    return canonical_robot(value)


def resolve_active_robot_name(state: Any = None, default: str = "Manipulator") -> str:
    return canonical_robot(getattr(state, "robot", default), default)


def runtime_mode_label(*_args: Any, **_kwargs: Any) -> str:
    return "system_1_0_orchestrated"


def pose_from_state(state: Any) -> Dict[str, float]:
    servos = getattr(state, "servos", {}) or {}
    defaults = {
        "base": 180.0, "upper": 45.0, "fore": 180.0,
        "forearm_roll": 90.0, "wrist_pitch": 95.0, "wrist_rot": 130.0, "grip": 50.0,
    }
    return {
        key: clamp(servos.get(i, defaults[key]), *MANIP_POSE_LIMITS[key])
        for i, key in enumerate(MANIP_KEYS)
    }


def ui_pose_to_armj_values(pose: Dict[str, Any]) -> List[int]:
    src = pose if isinstance(pose, dict) else {}
    return [
        int(round(clamp_member_pose(key, src.get(key, 50.0 if key == "grip" else 180.0))))
        for key in MANIP_KEYS
    ]


def build_armj_command(pose: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "type": "control_intent",
        "control": {
            "robot": "Manipulator",
            "manipulator": {"angles": ui_pose_to_armj_values(pose)},
        },
    }


def nudge_manipulator_pose(pose: Dict[str, Any], key: str, delta: Any) -> Dict[str, float]:
    out = dict(pose or {}) if isinstance(pose, dict) else {}
    current = safe_float(out.get(key, 50.0 if key == "grip" else 180.0))
    if key == "base":
        out[key] = wrap_abs(current + safe_float(delta))
    else:
        out[key] = clamp_member_pose(key, current + safe_float(delta))
    return {
        member: clamp_member_pose(member, out.get(member, 50.0 if member == "grip" else 180.0))
        for member in MANIP_KEYS
    }
