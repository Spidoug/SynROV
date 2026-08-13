"""Safety clamps and output smoothing for SynROV command execution."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .primitives import safe_float, to_list


def clamp(value: Any, lo: float, hi: float) -> float:
    """Clamp *value* to [*lo*, *hi*] using the shared finite-number rule."""
    low = safe_float(lo, 0.0)
    high = safe_float(hi, low)
    if low > high:
        low, high = high, low
    return safe_float(value, low, low, high)


def _padded(values: Any, size: int, default: float = 0.0) -> List[Any]:
    vals = to_list(values)[:size]
    vals.extend([default] * max(0, size - len(vals)))
    return vals


MANIP_KEYS: List[str] = [
    "base", "upper", "fore", "forearm_roll", "wrist_pitch", "wrist_rot", "grip",
]

MANIPULATOR_HOME_POSE: Dict[str, float] = {
    "base": 180.0,
    "upper": 150.0,
    "fore": 70.0,
    "forearm_roll": 90.0,
    "wrist_pitch": 95.0,
    "wrist_rot": 130.0,
    "grip": 0.0,
}

MANIP_POSE_LIMITS: Dict[str, Tuple[float, float]] = {
    "base":         (0.0, 359.0),
    "upper":        (0.0, 359.0),
    "fore":         (0.0, 359.0),
    "forearm_roll": (0.0, 359.0),
    "wrist_pitch":  (0.0, 359.0),
    "wrist_rot":    (0.0, 359.0),
    "grip":         (0.0, 100.0),
}


MANIP_COMMAND_RATE_DPS: Dict[str, float] = {
    "base":         150.0,
    "upper":        120.0,
    "fore":         120.0,
    "forearm_roll": 170.0,
    "wrist_pitch":  140.0,
    "wrist_rot":    170.0,
    "grip":          90.0,
}

_MANIP_TARGET_LIMITS: List[Tuple[float, float]] = [
    (-180.0, 180.0), (-150.0, 150.0), (-150.0, 150.0),
    (-180.0, 180.0), (-160.0, 160.0), (-180.0, 180.0), (-120.0, 120.0),
]


@dataclass
class SafetyReport:
    robot: str
    source: str
    changed: bool = False
    reason: str = "ok"
    input_values: List[float] = field(default_factory=list)
    output_values: List[float] = field(default_factory=list)


class SynROVSafetyLayer:
    """Final safety layer before a command leaves the Python controller."""

    def __init__(self) -> None:
        self.last_pose: Optional[Dict[str, float]] = None
        self.last_mobile: Dict[str, List[float]] = {}
        self.last_ts: Dict[str, float] = {}
        self.last_report: Optional[SafetyReport] = None
        self.enabled = True


    def _dt(self, key: str, now: Optional[float] = None) -> float:
        """Return elapsed seconds since the last call for *key*, clamped to [0.08, 0.35]."""
        ts = float(now if now is not None else time.time())
        prev = self.last_ts.get(key, 0.0)
        self.last_ts[key] = ts
        return max(0.08, min(0.35, ts - prev if prev else 0.12))

    def _set_report(self, robot: str, source: str, raw: List[float], out: List[float]) -> None:
        changed = any(abs(a - b) > 1e-6 for a, b in zip(raw, out))
        self.last_report = SafetyReport(
            robot,
            source,
            changed,
            "clamped_or_smoothed" if changed else "ok",
            list(raw),
            list(out),
        )


    def sanitize_pose(
        self,
        pose: Dict[str, Any],
        *,
        source: str = "unknown",
        smooth: bool = True,
    ) -> Dict[str, float]:
        """Clamp and optionally slew-rate-limit a manipulator pose dict."""
        src = pose if isinstance(pose, dict) else {}
        raw = {
            key: clamp(src.get(key, 50.0 if key == "grip" else 180.0), *MANIP_POSE_LIMITS[key])
            for key in MANIP_KEYS
        }
        out = dict(raw)

        if self.enabled and smooth and self.last_pose:
            dt = self._dt("Manipulator")
            for key in MANIP_KEYS:
                prev = clamp(self.last_pose.get(key, raw[key]), *MANIP_POSE_LIMITS[key])
                max_delta = MANIP_COMMAND_RATE_DPS.get(key, 120.0) * dt
                delta = raw[key] - prev
                if key == "base":
                    delta = ((delta + 180.0) % 360.0) - 180.0
                    out[key] = (prev + clamp(delta, -max_delta, max_delta)) % 360.0
                else:
                    lo, hi = MANIP_POSE_LIMITS[key]
                    out[key] = clamp(prev + clamp(delta, -max_delta, max_delta), lo, hi)
        else:
            self._dt("Manipulator")

        self.last_pose = dict(out)
        self._set_report("Manipulator", source, [raw[k] for k in MANIP_KEYS], [out[k] for k in MANIP_KEYS])
        return out

    def sanitize_mobile(
        self,
        robot: str,
        values: Iterable[Any],
        *,
        source: str = "unknown",
        smooth: bool = True,
    ) -> List[float]:
        """Clamp and optionally slew-rate-limit vehicle or drone commands (range −1..1)."""
        robot_name = "Drone" if str(robot).lower().startswith("drone") else "Vehicle"
        size = 6 if robot_name == "Drone" else 4
        raw = [clamp(v, -1.0, 1.0) for v in _padded(values, size)]
        out = list(raw)

        if self.enabled and smooth and robot_name in self.last_mobile:
            dt = self._dt(robot_name)
            max_delta = (2.8 if robot_name == "Vehicle" else 2.4) * dt
            prev = _padded(self.last_mobile.get(robot_name), size)
            out = [clamp(p + clamp(v - p, -max_delta, max_delta), -1.0, 1.0) for p, v in zip(prev, raw)]
        else:
            self._dt(robot_name)

        self.last_mobile[robot_name] = list(out)
        self._set_report(robot_name, source, raw, out)
        return out

    def sanitize_target(
        self,
        robot: str,
        values: Optional[List[float]],
        *,
        source: str = "unknown",
    ) -> Optional[List[float]]:
        """Validate a training target row (rates/deltas).  Returns None if input is None."""
        if values is None:
            return None
        name = str(robot or "Manipulator").lower()
        vals = to_list(values)

        if name.startswith("manip"):
            return [clamp(v, lo, hi) for v, (lo, hi) in zip(_padded(vals, 7), _MANIP_TARGET_LIMITS)]
        if name.startswith("drone"):
            return self.sanitize_mobile("Drone", vals, source=source, smooth=False)
        return self.sanitize_mobile("Vehicle", vals, source=source, smooth=False)
