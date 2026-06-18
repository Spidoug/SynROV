"""Safety clamps and output smoothing for SynROV command execution."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

from .primitives import to_list


def clamp(value: Any, lo: float, hi: float) -> float:
    """Clamp *value* to [*lo*, *hi*], coercing invalid values safely."""
    try:
        low = float(lo)
    except Exception:
        low = 0.0
    try:
        high = float(hi)
    except Exception:
        high = low
    if not math.isfinite(low):
        low = 0.0
    if not math.isfinite(high):
        high = low
    if low > high:
        low, high = high, low

    try:
        v = float(value)
    except Exception:
        v = low
    if not math.isfinite(v):
        v = low
    return max(low, min(high, v))


def _padded(values: Any, size: int, default: float = 0.0) -> List[Any]:
    vals = to_list(values)[:size]
    vals.extend([default] * max(0, size - len(vals)))
    return vals


MANIP_KEYS: List[str] = [
    "base", "upper", "fore", "forearm_roll", "wrist_pitch", "wrist_rot", "grip",
]

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


class SynROVBridgeSafetyAdapter:
    """Attaches the safety layer to a live bridge without changing its public API."""

    _INSTALLED_ATTR = "_synrov_bridge_safety_adapter_installed"

    def __init__(
        self,
        safety: SynROVSafetyLayer,
        *,
        source_getter: Optional[Callable[[], str]] = None,
        enabled_getter: Optional[Callable[[], bool]] = None,
    ) -> None:
        self.safety = safety
        self.source_getter = source_getter or (lambda: "unknown")
        self.enabled_getter = enabled_getter or (lambda: True)

    def _source(self) -> str:
        try:
            return str(self.source_getter() or "unknown")
        except Exception:
            return "unknown"

    def _enabled(self) -> bool:
        try:
            return bool(self.enabled_getter())
        except Exception:
            return True

    def _sanitize_manipulator_block(self, manip: Dict[str, Any]) -> Dict[str, Any]:
        raw_angles = _padded(manip.get("angles", []), 7, 180.0)
        pose = {
            key: raw_angles[i] if i < len(raw_angles) else (50.0 if key == "grip" else 180.0)
            for i, key in enumerate(MANIP_KEYS)
        }
        safe_pose = self.safety.sanitize_pose(pose, source=self._source(), smooth=True)
        out = dict(manip)
        out["angles"] = [int(round(safe_pose[key])) for key in MANIP_KEYS]
        return out

    def _sanitize_drive_block(self, drive: Dict[str, Any]) -> Dict[str, Any]:
        vals = self.safety.sanitize_mobile(
            "Vehicle",
            [
                drive.get("throttle", 0.0),
                drive.get("steer", 0.0),
                clamp(drive.get("camPan", 0.0), -90.0, 90.0) / 90.0,
                clamp(drive.get("camTilt", 0.0), -45.0, 45.0) / 45.0,
            ],
            source=self._source(),
            smooth=True,
        )
        out = dict(drive)
        out.update({"throttle": vals[0], "steer": vals[1], "camPan": vals[2] * 90.0, "camTilt": vals[3] * 45.0})
        return out

    def _sanitize_flight_block(self, flight: Dict[str, Any]) -> Dict[str, Any]:
        vals = self.safety.sanitize_mobile(
            "Drone",
            [
                flight.get("throttle", 0.0), flight.get("yaw", 0.0),
                flight.get("pitch", 0.0), flight.get("roll", 0.0),
                flight.get("strafe", 0.0), flight.get("forward", 0.0),
            ],
            source=self._source(),
            smooth=True,
        )
        out = dict(flight)
        out.update({
            "throttle": vals[0],
            "yaw": vals[1],
            "pitch": vals[2],
            "roll": vals[3],
            "strafe": vals[4],
            "forward": vals[5],
        })
        return out

    def _sanitize_control_payload(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        p = dict(payload or {})
        control = p.get("control") if isinstance(p.get("control"), dict) else None
        if not isinstance(control, dict):
            return p

        robot = str(control.get("robot", p.get("robot", "")) or "").lower()
        control = dict(control)

        manip = control.get("manipulator")
        if isinstance(manip, dict) and isinstance(manip.get("angles"), list):
            control["manipulator"] = self._sanitize_manipulator_block(manip)

        drive = control.get("drive")
        if isinstance(drive, dict) and (robot.startswith("vehicle") or "throttle" in drive or "steer" in drive):
            control["drive"] = self._sanitize_drive_block(drive)

        flight = control.get("flight")
        if isinstance(flight, dict) and robot.startswith("drone"):
            control["flight"] = self._sanitize_flight_block(flight)

        p["control"] = control
        return p

    def install(self, bridge: Any) -> Any:
        if bridge is None or getattr(bridge, self._INSTALLED_ATTR, False):
            return bridge

        safety = self.safety
        adapter = self

        original_command_pose = getattr(bridge, "command_manipulator_pose", None)
        original_vehicle = getattr(bridge, "send_vehicle", None)
        original_drone = getattr(bridge, "send_drone", None)
        original_control = getattr(bridge, "send_control", None)

        if callable(original_command_pose):
            def safe_command_manipulator_pose(pose: Dict[str, Any], *args: Any, **kwargs: Any) -> Any:
                safety.enabled = adapter._enabled()
                return original_command_pose(
                    safety.sanitize_pose(
                        dict(pose or {}) if isinstance(pose, dict) else {},
                        source=adapter._source(),
                        smooth=True,
                    ),
                    *args,
                    **kwargs,
                )
            setattr(bridge, "command_manipulator_pose", safe_command_manipulator_pose)

        if callable(original_vehicle):
            def safe_send_vehicle(
                throttle: Any, steer: Any, cam_pan: Any = 0.0, cam_tilt: Any = 0.0,
                *args: Any, **kwargs: Any,
            ) -> Any:
                safety.enabled = adapter._enabled()
                vals = safety.sanitize_mobile(
                    "Vehicle",
                    [throttle, steer, clamp(cam_pan, -90.0, 90.0) / 90.0, clamp(cam_tilt, -45.0, 45.0) / 45.0],
                    source=adapter._source(),
                    smooth=True,
                )
                return original_vehicle(vals[0], vals[1], vals[2] * 90.0, vals[3] * 45.0, *args, **kwargs)
            setattr(bridge, "send_vehicle", safe_send_vehicle)

        if callable(original_drone):
            def safe_send_drone(
                throttle: Any, yaw: Any, pitch: Any, roll: Any, strafe: Any, forward: Any,
                *args: Any, **kwargs: Any,
            ) -> Any:
                safety.enabled = adapter._enabled()
                vals = safety.sanitize_mobile(
                    "Drone",
                    [throttle, yaw, pitch, roll, strafe, forward],
                    source=adapter._source(),
                    smooth=True,
                )
                return original_drone(*vals, *args, **kwargs)
            setattr(bridge, "send_drone", safe_send_drone)

        if callable(original_control):
            def safe_send_control(payload: Dict[str, Any], *args: Any, **kwargs: Any) -> Any:
                safety.enabled = adapter._enabled()
                try:
                    return original_control(adapter._sanitize_control_payload(payload), *args, **kwargs)
                except Exception:
                    return original_control(payload, *args, **kwargs)
            setattr(bridge, "send_control", safe_send_control)

        setattr(bridge, self._INSTALLED_ATTR, True)
        setattr(bridge, "_synrov_bridge_safety_adapter", self)
        return bridge
