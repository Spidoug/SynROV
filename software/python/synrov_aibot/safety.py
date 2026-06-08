"""Safety clamps and output smoothing for SynROV command execution."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple


def clamp(value: Any, lo: float, hi: float) -> float:
    """Clamp *value* to [*lo*, *hi*], coercing non-finite and non-numeric to 0."""
    try:
        v = float(value)
    except Exception:
        v = 0.0
    if v != v:  # NaN check
        v = 0.0
    return max(float(lo), min(float(hi), v))


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

# Command slew limits (deg/s).  Intentionally higher than the learned policy
# rate so the safety layer never conflicts with smooth model output.
MANIP_COMMAND_RATE_DPS: Dict[str, float] = {
    "base":         150.0,
    "upper":        120.0,
    "fore":         120.0,
    "forearm_roll": 170.0,
    "wrist_pitch":  140.0,
    "wrist_rot":    170.0,
    "grip":          90.0,
}


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

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _dt(self, key: str, now: Optional[float] = None) -> float:
        """Return elapsed seconds since the last call for *key*, clamped to [0.08, 0.35]."""
        ts = float(now if now is not None else time.time())
        prev = self.last_ts.get(key, 0.0)
        self.last_ts[key] = ts
        return max(0.08, min(0.35, ts - prev if prev else 0.12))

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def sanitize_pose(
        self,
        pose: Dict[str, Any],
        *,
        source: str = "unknown",
        smooth: bool = True,
    ) -> Dict[str, float]:
        """Clamp and optionally slew-rate-limit a manipulator pose dict."""
        raw = {
            k: clamp(
                (pose or {}).get(k, 50.0 if k == "grip" else 180.0),
                *MANIP_POSE_LIMITS[k],
            )
            for k in MANIP_KEYS
        }
        out = dict(raw)

        if self.enabled and smooth and self.last_pose:
            dt = self._dt("Manipulator")
            for key in MANIP_KEYS:
                prev = clamp(self.last_pose.get(key, raw[key]), *MANIP_POSE_LIMITS[key])
                max_delta = MANIP_COMMAND_RATE_DPS.get(key, 120.0) * dt
                delta = raw[key] - prev
                if key == "base":
                    # Smooth the shortest rotation for the circular base axis.
                    delta = ((delta + 180.0) % 360.0) - 180.0
                    out[key] = (prev + clamp(delta, -max_delta, max_delta)) % 360.0
                else:
                    lo, hi = MANIP_POSE_LIMITS[key]
                    out[key] = clamp(prev + clamp(delta, -max_delta, max_delta), lo, hi)
        else:
            self._dt("Manipulator")

        changed = any(abs(out[k] - raw[k]) > 1e-6 for k in MANIP_KEYS)
        self.last_pose = dict(out)
        self.last_report = SafetyReport(
            "Manipulator",
            source,
            changed,
            "clamped_or_smoothed" if changed else "ok",
            [raw[k] for k in MANIP_KEYS],
            [out[k] for k in MANIP_KEYS],
        )
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
        vals = list(values or [])[:size]
        vals += [0.0] * (size - len(vals))
        raw = [clamp(v, -1.0, 1.0) for v in vals]
        out = list(raw)

        if self.enabled and smooth and robot_name in self.last_mobile:
            dt = self._dt(robot_name)
            max_delta = 2.8 * dt if robot_name == "Vehicle" else 2.4 * dt
            prev = (self.last_mobile.get(robot_name, [0.0] * size) + [0.0] * size)[:size]
            out = [
                clamp(p + clamp(v - p, -max_delta, max_delta), -1.0, 1.0)
                for p, v in zip(prev, raw)
            ]
        else:
            self._dt(robot_name)

        changed = any(abs(a - b) > 1e-6 for a, b in zip(raw, out))
        self.last_mobile[robot_name] = list(out)
        self.last_report = SafetyReport(
            robot_name,
            source,
            changed,
            "clamped_or_smoothed" if changed else "ok",
            raw,
            out,
        )
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
        name = str(robot or "Manipulator")
        vals = list(values or [])

        if name.lower().startswith("manip"):
            vals = vals[:7] + [0.0] * max(0, 7 - len(vals))
            limits = [
                (-180.0, 180.0), (-150.0, 150.0), (-150.0, 150.0),
                (-180.0, 180.0), (-160.0, 160.0), (-180.0, 180.0), (-120.0, 120.0),
            ]
            return [clamp(v, lo, hi) for v, (lo, hi) in zip(vals, limits)]
        if name.lower().startswith("drone"):
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
                    safety.sanitize_pose(dict(pose or {}), source=adapter._source(), smooth=True),
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
                    p = dict(payload or {})
                    control = p.get("control") if isinstance(p.get("control"), dict) else None
                    if not isinstance(control, dict):
                        return original_control(p, *args, **kwargs)

                    robot = str(
                        control.get("robot", p.get("robot", "")) or ""
                    )
                    control = dict(control)

                    manip = control.get("manipulator")
                    if isinstance(manip, dict) and isinstance(manip.get("angles"), list):
                        raw_angles = list(manip["angles"])[:7]
                        pose = {
                            k: raw_angles[i] if i < len(raw_angles) else (50.0 if k == "grip" else 180.0)
                            for i, k in enumerate(MANIP_KEYS)
                        }
                        safe_pose = safety.sanitize_pose(pose, source=adapter._source(), smooth=True)
                        manip = dict(manip)
                        manip["angles"] = [int(round(safe_pose[k])) for k in MANIP_KEYS]
                        control["manipulator"] = manip

                    drive = control.get("drive")
                    if isinstance(drive, dict) and (
                        robot.lower().startswith("vehicle") or "throttle" in drive or "steer" in drive
                    ):
                        vals = safety.sanitize_mobile(
                            "Vehicle",
                            [
                                drive.get("throttle", 0.0),
                                drive.get("steer", 0.0),
                                clamp(drive.get("camPan", 0.0), -90.0, 90.0) / 90.0,
                                clamp(drive.get("camTilt", 0.0), -45.0, 45.0) / 45.0,
                            ],
                            source=adapter._source(),
                            smooth=True,
                        )
                        drive = dict(drive)
                        drive.update({
                            "throttle": vals[0], "steer": vals[1],
                            "camPan": vals[2] * 90.0, "camTilt": vals[3] * 45.0,
                        })
                        control["drive"] = drive

                    flight = control.get("flight")
                    if isinstance(flight, dict) and robot.lower().startswith("drone"):
                        vals = safety.sanitize_mobile(
                            "Drone",
                            [
                                flight.get("throttle", 0.0), flight.get("yaw", 0.0),
                                flight.get("pitch", 0.0), flight.get("roll", 0.0),
                                flight.get("strafe", 0.0), flight.get("forward", 0.0),
                            ],
                            source=adapter._source(),
                            smooth=True,
                        )
                        flight = dict(flight)
                        flight.update({
                            "throttle": vals[0], "yaw": vals[1], "pitch": vals[2],
                            "roll": vals[3], "strafe": vals[4], "forward": vals[5],
                        })
                        control["flight"] = flight

                    p["control"] = control
                    return original_control(p, *args, **kwargs)
                except Exception:
                    return original_control(payload, *args, **kwargs)
            setattr(bridge, "send_control", safe_send_control)

        setattr(bridge, self._INSTALLED_ATTR, True)
        setattr(bridge, "_synrov_bridge_safety_adapter", self)
        return bridge
