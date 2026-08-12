"""Robot-specialty strategy layer for SynROV AiBot.

This module keeps mission/resource policy separate from actuator code.  It does
not replace firmware safety: it interprets energy, communication quality,
robot-specific torque/load references and Processing 3D context so each robot
can choose a strategy appropriate to its physical specialty.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from .robot_types import canonical_robot
from .primitives import safe_float


def _mapping(value: Any) -> Dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _sensor_number(sensors: Mapping[str, Any], keys: Sequence[str], default: Optional[float] = None) -> Optional[float]:
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


@dataclass(frozen=True)
class RobotSpecialtyStrategy:
    robot: str
    specialties: Tuple[str, ...]
    mission_families: Tuple[str, ...]
    learned_feature_groups: Tuple[str, ...]
    battery_return_pct: float
    battery_critical_pct: float
    link_caution_pct: float
    link_critical_pct: float

    def metadata(self) -> Dict[str, Any]:
        return {
            "robot": self.robot,
            "specialties": list(self.specialties),
            "mission_families": list(self.mission_families),
            "learned_feature_groups": list(self.learned_feature_groups),
            "resource_policy": {
                "battery_return_pct": self.battery_return_pct,
                "battery_critical_pct": self.battery_critical_pct,
                "link_caution_pct": self.link_caution_pct,
                "link_critical_pct": self.link_critical_pct,
            },
        }


STRATEGIES: Dict[str, RobotSpecialtyStrategy] = {
    "Manipulator": RobotSpecialtyStrategy(
        robot="Manipulator",
        specialties=(
            "workspace_inspection",
            "object_identification",
            "dual_imu_pose_reasoning",
            "joint_load_aware_manipulation",
            "grasp_and_place",
            "operator_demonstration_learning",
        ),
        mission_families=("inspect_workspace", "scan_workspace", "pick_object", "place_object", "calibrate_gripper"),
        learned_feature_groups=("joint_pose", "base_imu_mpu2", "gripper_imu_mpu1", "telemetry_heading", "joint_torque_references", "battery", "workspace_camera", "sonar"),
        battery_return_pct=16.0,
        battery_critical_pct=8.0,
        link_caution_pct=0.0,
        link_critical_pct=0.0,
    ),
    "Vehicle": RobotSpecialtyStrategy(
        robot="Vehicle",
        specialties=(
            "terrain_inspection",
            "ground_object_localization",
            "exit_and_corridor_discovery",
            "lidar_obstacle_avoidance",
            "gps_patrol",
            "traction_load_reasoning",
        ),
        mission_families=("terrain_inspection", "locate_objects", "find_exit", "patrol_area", "perimeter_scan", "corridor_scan", "return_home"),
        learned_feature_groups=("front_camera", "chassis_imu", "position", "gps", "telemetry_heading", "lidar", "drive_torque", "battery", "communication_link", "processing_world_3d"),
        battery_return_pct=22.0,
        battery_critical_pct=12.0,
        link_caution_pct=35.0,
        link_critical_pct=15.0,
    ),
    "Drone": RobotSpecialtyStrategy(
        robot="Drone",
        specialties=(
            "aerial_terrain_inspection",
            "aerial_object_localization",
            "three_dimensional_exit_discovery",
            "gps_area_survey",
            "ground_clearance_management",
            "link_aware_flight",
        ),
        mission_families=("terrain_inspection", "locate_objects", "find_exit", "aerial_scan", "search_pattern", "orbit_point", "return_home"),
        learned_feature_groups=("front_camera", "flight_imu", "position_altitude", "gps", "telemetry_heading", "downward_sonar", "battery", "communication_link", "processing_world_3d"),
        battery_return_pct=25.0,
        battery_critical_pct=15.0,
        link_caution_pct=40.0,
        link_critical_pct=18.0,
    ),
}


def strategy_for_robot(robot: Any) -> RobotSpecialtyStrategy:
    return STRATEGIES[canonical_robot(robot)]


def battery_percent(state: Any) -> Optional[float]:
    sensors = _mapping(getattr(state, "sensors", {}))
    value = _sensor_number(sensors, ("battery_pct", "bat_pct"), None)
    if value is None:
        values = _mapping(_mapping(getattr(state, "intelligence_context", {})).get("features"))
        value = safe_float(values.get("battery_pct", -1.0), -1.0)
        if value < 0:
            return None
    return max(0.0, min(100.0, float(value)))


def communication_quality_percent(state: Any) -> Optional[float]:
    sensors = _mapping(getattr(state, "sensors", {}))
    quality_source = str(sensors.get("communication_quality_source", "") or "").strip().lower()
    if quality_source == "unavailable":
        return None
    direct = _sensor_number(sensors, ("communication_quality_pct", "link_quality_pct", "signal_pct", "rssi_pct"), None)
    if direct is not None:
        return max(0.0, min(100.0, direct))
    rssi = _sensor_number(sensors, ("rssi_dbm", "signal_dbm"), None)
    if rssi is not None:
        return max(0.0, min(100.0, (rssi + 100.0) * 2.0))
    link_ms = _sensor_number(sensors, ("link_ms",), None)
    if link_ms is not None and link_ms >= 0.0:
        return max(0.0, min(100.0, 100.0 - max(0.0, link_ms - 40.0) * (100.0 / 760.0)))
    return None


def manipulator_joint_torque_references(state: Any) -> Dict[str, Dict[str, Any]]:
    sensors = _mapping(getattr(state, "sensors", {}))
    aliases = {
        "base": ("base_torque_ref", "base_torque", "base_torque_raw", "base_current_ma", "an2", "an_2"),
        "upper": ("upper_torque_ref", "upper_torque", "upper_arm_torque_raw", "upper_current_ma", "an3", "an_3"),
        "fore": ("fore_torque_ref", "forearm_torque", "forearm_torque_raw", "fore_current_ma", "an4", "an_4"),
        "forearm_roll": ("forearm_roll_torque_ref", "forearm_roll_torque", "forearm_roll_torque_raw", "forearm_roll_current_ma", "an5", "an_5"),
        "wrist_pitch": ("wrist_pitch_torque_ref", "wrist_pitch_current_ma", "wrist_pitch_torque", "wrist_pitch_torque_raw", "ina1_ma"),
        "wrist_rot": ("wrist_rot_torque_ref", "wrist_rot_current_ma", "wrist_rotation_current_ma", "wrist_rot_torque", "ina2_ma"),
        "grip": ("grip_torque_ref", "grip_current_ma", "gripper_current_ma", "grip_torque", "grip_pressure_ma", "ina1_ma"),
    }
    display_names = {
        "an2": "Base torque sensor", "an_2": "Base torque sensor",
        "an3": "Upper arm torque sensor", "an_3": "Upper arm torque sensor",
        "an4": "Forearm vertical torque sensor", "an_4": "Forearm vertical torque sensor",
        "an5": "Forearm rotational torque sensor", "an_5": "Forearm rotational torque sensor",
        "ina1_ma": "Wrist group current 1", "ina2_ma": "Wrist group current 2",
        "grip_pressure_ma": "Gripper pressure/current reference",
    }
    out: Dict[str, Dict[str, Any]] = {}
    for joint, keys in aliases.items():
        source = next((key for key in keys if key in sensors), "")
        out[joint] = {
            "available": bool(source),
            "source_key": source,
            "sensor_name": display_names.get(source, f"{joint} torque reference"),
            "reference": safe_float(sensors.get(source, 0.0), 0.0) if source else 0.0,
            "meaning": "proportional_joint_torque_reference",
            "signal_type": "current_ma" if "ma" in source else "raw_torque_sensor",
            "unit": "raw_reference",
        }
    return out


def vehicle_drive_torque_references(state: Any) -> Dict[str, Any]:
    sensors = _mapping(getattr(state, "sensors", {}))
    left_keys = ("vehicle_left_torque_ref", "left_track_torque", "left_torque", "left_motor_torque", "an2", "an_2")
    right_keys = ("vehicle_right_torque_ref", "right_track_torque", "right_torque", "right_motor_torque", "an3", "an_3")
    left_source = next((key for key in left_keys if key in sensors), "")
    right_source = next((key for key in right_keys if key in sensors), "")
    return {
        "available": bool(left_source or right_source),
        "left_reference": safe_float(sensors.get(left_source, 0.0), 0.0) if left_source else 0.0,
        "right_reference": safe_float(sensors.get(right_source, 0.0), 0.0) if right_source else 0.0,
        "left_source_key": left_source,
        "right_source_key": right_source,
        "meaning": "proportional_drive_torque_reference",
        "unit": "raw_reference",
    }


def mission_resource_status(state: Any, robot: Any, mission: str = "") -> Dict[str, Any]:
    name = canonical_robot(robot)
    profile = strategy_for_robot(name)
    battery = battery_percent(state)
    link = communication_quality_percent(state) if name in {"Vehicle", "Drone"} else None
    status = {
        "battery_pct": battery,
        "communication_quality_pct": link,
        "battery_low": battery is not None and battery <= profile.battery_return_pct,
        "battery_critical": battery is not None and battery <= profile.battery_critical_pct,
        "link_weak": link is not None and link <= profile.link_caution_pct,
        "link_critical": link is not None and link <= profile.link_critical_pct,
        "mission": str(mission or ""),
        "recommended_action": "continue",
    }
    if status["battery_critical"]:
        status["recommended_action"] = "land" if name == "Drone" else ("stop" if name == "Vehicle" else "hold")
    elif status["link_critical"]:
        status["recommended_action"] = "hover" if name == "Drone" else "stop"
    elif status["battery_low"]:
        status["recommended_action"] = "return_home"
    elif status["link_weak"]:
        status["recommended_action"] = "reduce_range_and_speed"
    return status


def _obstacles(state: Any) -> list[Dict[str, float]]:
    raw = list(getattr(state, "perception_obstacles", []) or [])
    out = []
    for item in raw:
        if not isinstance(item, Mapping):
            continue
        out.append({
            "x_m": safe_float(item.get("x_m"), 0.0),
            "y_m": safe_float(item.get("y_m"), 0.0),
            "z_m": safe_float(item.get("z_m"), 0.0),
            "distance_m": safe_float(item.get("distance_m"), 0.0),
        })
    return out


def vehicle_exit_steer(state: Any) -> Tuple[float, Dict[str, Any]]:
    """Choose the least occupied forward corridor from Processing 3D points."""
    bins = {"left": 0.0, "front": 0.0, "right": 0.0}
    nearest = {"left": 999.0, "front": 999.0, "right": 999.0}
    for item in _obstacles(state):
        x, z, dist = item["x_m"], item["z_m"], item["distance_m"]
        if z <= 0.0 or dist <= 0.0 or z > 12.0 or abs(x) > 8.0:
            continue
        key = "left" if x < -1.2 else ("right" if x > 1.2 else "front")
        bins[key] += 1.0 / max(0.35, dist)
        nearest[key] = min(nearest[key], dist)
    scores = {key: nearest[key] - bins[key] * 0.45 for key in bins}
    choice = max(scores, key=scores.get)
    steer = -0.42 if choice == "left" else (0.42 if choice == "right" else 0.0)
    return steer, {"corridor": choice, "corridor_scores": scores, "nearest_m": nearest}


def drone_exit_vector(state: Any) -> Tuple[float, float, float, Dict[str, Any]]:
    """Return strafe, forward and climb suggestions for a free 3D corridor."""
    left = right = front = above = 0.0
    for item in _obstacles(state):
        x, y, z, dist = item["x_m"], item["y_m"], item["z_m"], item["distance_m"]
        if dist <= 0.0 or dist > 14.0:
            continue
        # Drone forward is -Z in the Processing perception coordinate frame.
        if z < -0.5 and abs(x) < 3.0 and abs(y) < 3.0:
            front += 1.0 / max(0.35, dist)
        if x < -0.5:
            left += 1.0 / max(0.35, dist)
        elif x > 0.5:
            right += 1.0 / max(0.35, dist)
        if y < -0.5:
            above += 1.0 / max(0.35, dist)
    strafe = 0.0
    climb = 0.0
    forward = 0.18
    if front > 0.75:
        strafe = 0.24 if left > right else -0.24
        forward = 0.05
        if above < min(left, right) * 0.7:
            climb = 0.10
    return strafe, forward, climb, {"occupancy": {"left": left, "right": right, "front": front, "above": above}}


__all__ = [
    "RobotSpecialtyStrategy",
    "STRATEGIES",
    "strategy_for_robot",
    "battery_percent",
    "communication_quality_percent",
    "manipulator_joint_torque_references",
    "vehicle_drive_torque_references",
    "mission_resource_status",
    "vehicle_exit_steer",
    "drone_exit_vector",
]
