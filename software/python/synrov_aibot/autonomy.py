"""Robot-specific command authority and adaptive mission helpers.

The tables here do not create unsolicited motion.  They tune arbitration and
adapt *already requested* missions using available telemetry, keeping manual
and voice control dominant while giving each robot an independent autonomy
profile.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from .robot_types import canonical_robot
from .primitives import safe_float
from .robot_strategy import communication_quality_percent, vehicle_drive_torque_references


@dataclass(frozen=True)
class CommandTuning:
    """Per-command arbitration parameters.

    ``priority_bias`` is added to the source priority only for that robot and
    command. ``lock_scale`` changes the ownership window. ``confidence_floor``
    is the minimum autonomous/model confidence required for the command.
    ``authority`` is metadata for planners/rankers and never scales a direct
    human command by itself.
    """

    authority: float = 1.0
    priority_bias: int = 0
    lock_scale: float = 1.0
    confidence_floor: float = 0.18


DEFAULT_TUNING = CommandTuning()

# Human commands remain dominant; these values mainly distinguish autonomous
# choices that would otherwise tie at the same source priority.
ROBOT_COMMAND_TUNING: Dict[str, Dict[str, CommandTuning]] = {
    "Manipulator": {
        "home": CommandTuning(1.30, 14, 1.30, 0.18),
        "stop_mission": CommandTuning(1.55, 24, 1.55, 0.08),
        "base_left": CommandTuning(1.02, 3, 1.00, 0.24),
        "base_right": CommandTuning(1.02, 3, 1.00, 0.24),
        "arm_up": CommandTuning(1.06, 4, 1.04, 0.25),
        "arm_down": CommandTuning(1.08, 5, 1.06, 0.23),
        "fore_up": CommandTuning(1.04, 3, 1.02, 0.26),
        "fore_down": CommandTuning(1.06, 4, 1.04, 0.24),
        "wrist_up": CommandTuning(0.98, 1, 0.96, 0.28),
        "wrist_down": CommandTuning(1.00, 2, 0.98, 0.26),
        "wrist_left": CommandTuning(0.96, 0, 0.95, 0.28),
        "wrist_right": CommandTuning(0.96, 0, 0.95, 0.28),
        "roll_left": CommandTuning(0.94, -1, 0.94, 0.30),
        "roll_right": CommandTuning(0.94, -1, 0.94, 0.30),
        "grip_open": CommandTuning(1.12, 5, 1.05, 0.24),
        "grip_close": CommandTuning(1.18, 7, 1.10, 0.28),
        "hold_pose": CommandTuning(1.22, 8, 1.15, 0.22),
        "return_home": CommandTuning(1.30, 14, 1.30, 0.20),
        "pick_object": CommandTuning(1.15, 7, 1.10, 0.34),
        "place_object": CommandTuning(1.12, 6, 1.08, 0.32),
        "scan_workspace": CommandTuning(0.92, 1, 0.92, 0.38),
        "inspect_workspace": CommandTuning(0.95, 2, 0.95, 0.36),
        "calibrate_gripper": CommandTuning(0.90, 0, 0.90, 0.45),
        "rhythm_mode": CommandTuning(0.72, -7, 0.82, 0.52),
        "wave": CommandTuning(0.76, -5, 0.85, 0.48),
    },
    "Vehicle": {
        "stop": CommandTuning(1.45, 18, 1.45, 0.10),
        "stop_mission": CommandTuning(1.55, 24, 1.55, 0.08),
        "back": CommandTuning(1.18, 8, 1.12, 0.20),
        "forward": CommandTuning(1.00, 2, 1.00, 0.28),
        "left": CommandTuning(1.02, 3, 1.00, 0.26),
        "right": CommandTuning(1.02, 3, 1.00, 0.26),
        "camera_left": CommandTuning(0.72, -4, 0.80, 0.20),
        "camera_right": CommandTuning(0.72, -4, 0.80, 0.20),
        "camera_up": CommandTuning(0.72, -4, 0.80, 0.20),
        "camera_down": CommandTuning(0.72, -4, 0.80, 0.20),
        "camera_center": CommandTuning(0.78, -2, 0.84, 0.18),
        "lights_toggle": CommandTuning(0.74, -3, 0.82, 0.22),
        "lidar_toggle": CommandTuning(0.82, -1, 0.88, 0.20),
        "hold_position": CommandTuning(1.30, 12, 1.25, 0.16),
        "dock": CommandTuning(1.08, 5, 1.10, 0.42),
        "return_home": CommandTuning(1.10, 6, 1.12, 0.38),
        "follow_target": CommandTuning(0.98, 1, 0.96, 0.44),
        "corridor_scan": CommandTuning(0.90, -1, 0.92, 0.46),
        "perimeter_scan": CommandTuning(0.88, -2, 0.90, 0.48),
        "patrol_area": CommandTuning(0.86, -3, 0.88, 0.50),
        "terrain_inspection": CommandTuning(0.92, 0, 0.94, 0.46),
        "locate_objects": CommandTuning(1.02, 4, 1.02, 0.40),
        "find_exit": CommandTuning(1.08, 6, 1.08, 0.38),
    },
    "Drone": {
        "land": CommandTuning(1.55, 24, 1.60, 0.08),
        "stop_mission": CommandTuning(1.60, 26, 1.65, 0.06),
        "emergency_land": CommandTuning(1.65, 30, 1.80, 0.05),
        "hover": CommandTuning(1.48, 20, 1.50, 0.10),
        "takeoff": CommandTuning(1.18, 8, 1.18, 0.30),
        "up": CommandTuning(1.05, 4, 1.05, 0.28),
        "down": CommandTuning(1.08, 5, 1.08, 0.25),
        "forward": CommandTuning(0.96, 0, 0.96, 0.38),
        "back": CommandTuning(0.98, 1, 0.98, 0.34),
        "left": CommandTuning(0.94, -1, 0.94, 0.38),
        "right": CommandTuning(0.94, -1, 0.94, 0.38),
        "yaw_left": CommandTuning(0.90, -2, 0.92, 0.34),
        "yaw_right": CommandTuning(0.90, -2, 0.92, 0.34),
        "camera_left": CommandTuning(0.68, -5, 0.78, 0.22),
        "camera_right": CommandTuning(0.68, -5, 0.78, 0.22),
        "camera_up": CommandTuning(0.70, -4, 0.80, 0.22),
        "camera_down": CommandTuning(0.72, -3, 0.82, 0.20),
        "camera_center": CommandTuning(0.76, -2, 0.84, 0.18),
        "camera_stream_toggle": CommandTuning(0.72, -3, 0.82, 0.22),
        "altitude_hold": CommandTuning(1.32, 13, 1.30, 0.20),
        "return_home": CommandTuning(1.20, 9, 1.20, 0.36),
        "orbit_point": CommandTuning(0.88, -2, 0.90, 0.52),
        "aerial_scan": CommandTuning(0.86, -3, 0.88, 0.54),
        "search_pattern": CommandTuning(0.82, -4, 0.86, 0.58),
        "terrain_inspection": CommandTuning(0.92, 0, 0.94, 0.48),
        "locate_objects": CommandTuning(1.02, 4, 1.02, 0.42),
        "find_exit": CommandTuning(1.10, 7, 1.10, 0.40),
    },
}


def canonical_command_key(robot: Any, command: Any) -> str:
    """Map core intent names to the robot-specific policy key.

    The Tk runtime prefixes mobile-robot model intents with
    ``vehicle_``/``drone_`` while the dedicated backend uses the shorter
    command names.  Both spellings represent the same authority decision and
    must therefore share exactly the same tuning/learning weight.
    """
    name = canonical_robot(robot)
    key = str(command or "").strip().lower()
    if name == "Vehicle" and key.startswith("vehicle_"):
        key = key[len("vehicle_"):]
    elif name == "Drone" and key.startswith("drone_"):
        key = key[len("drone_"):]
    elif name == "Manipulator":
        key = {
            "arm_left": "base_left",
            "arm_right": "base_right",
            "hold_position": "hold_pose",
            "object_grasp": "pick_object",
            "music_follow": "rhythm_mode",
        }.get(key, key)
    return key


def command_tuning(robot: Any, command: Any) -> CommandTuning:
    name = canonical_robot(robot)
    key = canonical_command_key(name, command)
    return ROBOT_COMMAND_TUNING.get(name, {}).get(key, DEFAULT_TUNING)


def sensor_number(sensors: Mapping[str, Any], keys: Sequence[str], default: Optional[float] = None) -> Optional[float]:
    for key in keys:
        if key not in sensors:
            continue
        try:
            value = float(sensors.get(key))
        except (TypeError, ValueError, OverflowError):
            continue
        if math.isfinite(value):
            return value
    return default


def battery_percent(state: Any) -> Optional[float]:
    sensors = getattr(state, "sensors", {}) or {}
    value = sensor_number(sensors, ("battery_pct", "bat_pct"), None)
    if value is None:
        return None
    return max(0.0, min(100.0, value))


def _perception_forward_obstacle_distance_cm(state: Any, robot: Any) -> Optional[float]:
    """Return the nearest forward obstacle from Processing's 3D stream.

    The dedicated stream reports robot-relative meters. Vehicle forward is +Z
    in Processing, while Drone forward is -Z. A corridor gate prevents nearby
    side/behind geometry from being mistaken for a frontal obstacle.
    """
    obstacles = getattr(state, "perception_obstacles", []) or []
    name = canonical_robot(robot or getattr(state, "robot", None))
    distances = []
    for item in obstacles:
        if not isinstance(item, Mapping):
            continue
        x_m = safe_float(item.get("x_m"), 0.0)
        y_m = safe_float(item.get("y_m"), 0.0)
        z_m = safe_float(item.get("z_m"), 0.0)
        distance_m = safe_float(item.get("distance_m"), -1.0)
        if distance_m <= 0.0:
            continue
        if name == "Drone":
            forward_m = -z_m
            half_width_m = max(1.4, min(5.0, forward_m * 0.22))
            vertical_gate_m = 2.5
        elif name == "Vehicle":
            forward_m = z_m
            half_width_m = max(0.8, min(3.0, forward_m * 0.20))
            vertical_gate_m = 1.8
        else:
            # The Manipulator does not have a single forward driving corridor;
            # its Processing collision guard remains the authoritative source.
            continue
        if forward_m <= 0.0 or abs(x_m) > half_width_m or abs(y_m) > vertical_gate_m:
            continue
        distances.append(distance_m * 100.0)
    return min(distances) if distances else None


def obstacle_distance_cm(state: Any) -> Optional[float]:
    sensors = getattr(state, "sensors", {}) or {}
    values = []
    for key in ("lidar_cm", "range_cm", "sonar_cm", "distance_cm", "an4", "an5"):
        if key in sensors:
            value = safe_float(sensors.get(key), -1.0)
            if value > 0.0:
                values.append(value)

    visual_distance = _perception_forward_obstacle_distance_cm(
        state, getattr(state, "robot", None)
    )
    if visual_distance is not None:
        values.append(visual_distance)
    return min(values) if values else None


def collision_filtered(state: Any, robot: Any) -> bool:
    control = getattr(state, "control", {}) or {}
    name = canonical_robot(robot)
    block = control.get("drive" if name == "Vehicle" else "flight", {})
    return bool(isinstance(block, Mapping) and block.get("collisionFiltered", False))


def adapt_vehicle_motion(
    state: Any,
    throttle: float,
    steer: float,
    *,
    caution_cm: float = 80.0,
    stop_cm: float = 34.0,
) -> Tuple[float, float, Dict[str, Any]]:
    """Adapt an autonomous vehicle step without blocking escape motion."""
    throttle = safe_float(throttle, 0.0, -1.0, 1.0)
    steer = safe_float(steer, 0.0, -1.0, 1.0)
    distance = obstacle_distance_cm(state)
    filtered = collision_filtered(state, "Vehicle")
    meta: Dict[str, Any] = {"distance_cm": distance, "collision_filtered": filtered}
    battery = battery_percent(state)
    link = communication_quality_percent(state)
    drive_torque = vehicle_drive_torque_references(state)
    meta.update({"battery_pct": battery, "communication_quality_pct": link, "drive_torque": drive_torque})

    # Resource state is part of autonomous control, not merely UI telemetry.
    # Torque channels remain references (typically current/raw sensor values),
    # not calibrated N.m unless the hardware provides a calibration curve.
    if battery is not None and battery <= 12.0:
        meta["resource_guard"] = "battery_critical"
        return 0.0, 0.0, meta
    resource_scale = 1.0
    if battery is not None and battery <= 22.0:
        resource_scale = min(resource_scale, 0.58)
        meta["battery_limited"] = True
    if link is not None and link <= 15.0:
        meta["resource_guard"] = "link_critical"
        return 0.0, 0.0, meta
    if link is not None and link <= 35.0:
        resource_scale = min(resource_scale, 0.52)
        meta["link_limited"] = True
    if resource_scale < 1.0:
        throttle *= resource_scale
        steer *= max(0.72, resource_scale)
        meta["resource_speed_scale"] = resource_scale

    # Reverse/escape commands are never suppressed by a front-distance guard.
    if throttle <= 0.0:
        return throttle, steer, meta
    if filtered or (distance is not None and distance <= stop_cm):
        # Use a small reverse + turn so an autonomous mission can free itself.
        direction = -1.0 if steer >= 0.0 else 1.0
        meta["avoidance"] = "escape"
        return -0.12, 0.34 * direction, meta
    if distance is not None and distance < caution_cm:
        scale = max(0.18, min(1.0, (distance - stop_cm) / max(1.0, caution_cm - stop_cm)))
        meta["avoidance"] = "slowdown"
        meta["speed_scale"] = scale
        return throttle * scale, steer, meta
    return throttle, steer, meta


def adapt_drone_motion(
    state: Any,
    flight: Sequence[float],
) -> Tuple[Tuple[float, float, float, float, float, float], Dict[str, Any]]:
    values = [safe_float(v, 0.0, -1.0, 1.0) for v in list(flight)[:6]]
    while len(values) < 6:
        values.append(0.0)
    throttle, yaw, pitch, roll, strafe, forward = values
    meta: Dict[str, Any] = {}

    battery = battery_percent(state)
    if battery is not None:
        meta["battery_pct"] = battery
        if battery <= 10.0:
            meta["recommend_land"] = True
        elif battery <= 18.0:
            # Keep maneuvering authority but avoid climbing/accelerating hard.
            throttle = min(throttle, 0.10)
            strafe *= 0.55
            forward *= 0.55
            meta["low_battery_limited"] = True

    link = communication_quality_percent(state)
    if link is not None:
        meta["communication_quality_pct"] = link
        if link <= 18.0:
            # A critical link does not command an uncontrolled continuation.
            # Mission policy can choose return/land; the motion layer holds.
            yaw = pitch = roll = strafe = forward = 0.0
            throttle = 0.0
            meta["link_guard"] = "critical_hold"
        elif link <= 40.0:
            yaw *= 0.55
            pitch *= 0.55
            roll *= 0.55
            strafe *= 0.48
            forward *= 0.48
            meta["weak_link_limited"] = True

    sensors = getattr(state, "sensors", {}) or {}
    altitude = sensor_number(sensors, ("alt_cm", "altitude_cm", "sonar_down_cm"), None)
    if altitude is not None:
        meta["altitude_cm"] = altitude
        if 0.0 < altitude < 28.0 and throttle < 0.0:
            throttle = max(throttle, -0.06)
            meta["ground_guard"] = True

    if collision_filtered(state, "Drone"):
        # Processing owns the final collision guard.  The AI backs off horizontal
        # demand so it does not repeatedly press into the same obstacle.
        strafe *= 0.25
        forward *= 0.25
        pitch *= 0.35
        roll *= 0.35
        meta["collision_relief"] = True

    return (throttle, yaw, pitch, roll, strafe, forward), meta


__all__ = [
    "CommandTuning",
    "ROBOT_COMMAND_TUNING",
    "canonical_command_key",
    "command_tuning",
    "battery_percent",
    "obstacle_distance_cm",
    "collision_filtered",
    "adapt_vehicle_motion",
    "adapt_drone_motion",
]
