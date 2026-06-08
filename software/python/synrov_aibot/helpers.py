"""Shared utility functions for text, numeric safety and pose handling."""
from __future__ import annotations

import json
import math
import re
import unicodedata
from typing import Any, Dict, Iterable, List

from .dataset import canonical_robot
from .safety import MANIP_KEYS, MANIP_POSE_LIMITS, clamp


# ---------------------------------------------------------------------------
# Numeric helpers
# ---------------------------------------------------------------------------

def safe_float(value: Any, default: float = 0.0) -> float:
    """Return *value* as a finite float, falling back to *default*."""
    try:
        out = float(value)
        return out if math.isfinite(out) else float(default)
    except Exception:
        return float(default)


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(float(value))
    except Exception:
        return int(default)


def mean_abs(values: Iterable[Any]) -> float:
    vals = [abs(safe_float(v)) for v in values or []]
    return sum(vals) / max(1, len(vals))


# ---------------------------------------------------------------------------
# Text helpers
# ---------------------------------------------------------------------------

def normalize_text(value: Any) -> str:
    text = unicodedata.normalize("NFKD", str(value or "").lower())
    text = "".join(ch for ch in text if not unicodedata.combining(ch))
    text = re.sub(r"[^a-z0-9\s]", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def has_any_token(text: Any, tokens: Iterable[str]) -> bool:
    """Return True if any normalised token appears inside normalised *text*."""
    norm = normalize_text(text)
    return any(normalize_text(t) in norm for t in tokens or [])


def voice_vector(text: Any, size: int = 12) -> List[float]:
    norm = normalize_text(text)
    vec = [0.0] * int(size)
    for i, ch in enumerate(norm.encode("utf-8")):
        vec[i % len(vec)] += ch / 255.0
    scale = max(1.0, max(abs(v) for v in vec))
    return [v / scale for v in vec]


def format_float(value: Any, digits: int = 2) -> str:
    return f"{safe_float(value):.{int(digits)}f}"


def json_compatible(value: Any) -> Any:
    try:
        json.dumps(value)
        return value
    except Exception:
        pass
    if isinstance(value, dict):
        return {str(k): json_compatible(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [json_compatible(v) for v in value]
    return str(value)


# ---------------------------------------------------------------------------
# Angle helpers
# ---------------------------------------------------------------------------

def wrap_signed(value: Any) -> float:
    return ((safe_float(value) + 180.0) % 360.0) - 180.0


def wrap_abs(value: Any) -> float:
    return safe_float(value) % 360.0


# ---------------------------------------------------------------------------
# Manipulator rate helpers
# ---------------------------------------------------------------------------

# Per-joint slew limits used during training (deg/s).
_TRAINING_RATE_LIMITS: List[float] = [180.0, 150.0, 150.0, 180.0, 160.0, 180.0, 120.0]


def clamp_manipulator_rate_vector(values: Iterable[Any]) -> List[float]:
    vals = list(values or [])[:7]
    vals += [0.0] * max(0, 7 - len(vals))
    return [clamp(v, -lim, lim) for v, lim in zip(vals, _TRAINING_RATE_LIMITS)]


def manipulator_rate_from_delta_hint(delta: Iterable[Any], dt: float = 0.2) -> List[float]:
    dt = max(0.05, min(1.0, safe_float(dt, 0.2)))
    return clamp_manipulator_rate_vector([safe_float(v) / dt for v in delta or []])


def normalize_training_target(robot: Any, values: Iterable[Any]) -> List[float]:
    name = canonical_robot(robot)
    size = 7 if name == "Manipulator" else 6 if name == "Drone" else 4
    vals = [safe_float(v) for v in list(values or [])[:size]]
    vals += [0.0] * (size - len(vals))
    return vals


def inference_dt_from_period_ms(period_ms: Any) -> float:
    return max(0.12, min(5.0, safe_float(period_ms, 500.0) / 1000.0))


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------

def clamp_member_pose(key: str, value: Any) -> float:
    return clamp(value, *MANIP_POSE_LIMITS.get(key, (0.0, 359.0)))


def normalize_robot_name(value: Any) -> str:
    return canonical_robot(value)


def resolve_active_robot_name(state: Any = None, default: str = "Manipulator") -> str:
    return canonical_robot(getattr(state, "robot", default), default)


def runtime_mode_label(*_args: Any, **_kwargs: Any) -> str:
    return "system_v1_orchestrated"


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
    return [
        int(round(clamp_member_pose(key, (pose or {}).get(key, 50.0 if key == "grip" else 180.0))))
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
    out = dict(pose or {})
    current = safe_float(out.get(key, 50.0 if key == "grip" else 180.0))
    if key == "base":
        out[key] = wrap_abs(current + safe_float(delta))
    else:
        out[key] = clamp_member_pose(key, current + safe_float(delta))
    # Re-clamp every key so callers always get a fully validated pose.
    return {
        k: clamp_member_pose(k, out.get(k, 50.0 if k == "grip" else 180.0))
        for k in MANIP_KEYS
    }
