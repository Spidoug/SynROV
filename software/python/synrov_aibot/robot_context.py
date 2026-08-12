"""Robot-specific multimodal context and control ownership for SynROV AiBot.

The Processing perception stream describes what is available in the simulated/
reconstructed 3D world, while runtime telemetry remains authoritative for robot
state.  This module fuses those two streams into one small, deterministic
context per robot so autonomy never has to guess which sensors or human inputs
matter for Manipulator, Vehicle or Drone.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping, Sequence

from .robot_types import canonical_robot
from .primitives import safe_float
from .protocol import ROBOT_INTELLIGENCE_SCHEMA, ROBOT_SENSOR_MANIFEST_SCHEMA, SOFTWARE_VERSION
from .robot_strategy import (
    battery_percent,
    communication_quality_percent,
    manipulator_joint_torque_references,
    mission_resource_status,
    strategy_for_robot,
    vehicle_drive_torque_references,
)

HUMAN_CONTROL_SOURCES = {"keyboard", "joystick", "leap", "leap_motion", "manual", "operator", "ui", "web"}


def _mapping(value: Any) -> Dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _first(mapping: Mapping[str, Any], keys: Iterable[str], default: Any = None) -> Any:
    for key in keys:
        if key in mapping and mapping[key] is not None:
            return mapping[key]
    return default


def _float(mapping: Mapping[str, Any], keys: Iterable[str], default: float = 0.0) -> float:
    return safe_float(_first(mapping, keys, default), default)


def _bool(mapping: Mapping[str, Any], keys: Iterable[str], default: bool = False) -> bool:
    value = _first(mapping, keys, default)
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on", "valid", "ready", "connected"}
    return bool(value)


@dataclass(frozen=True)
class RobotIntelligenceProfile:
    robot: str
    primary_camera_role: str
    primary_state: Sequence[str]
    primary_navigation: Sequence[str]
    primary_safety: Sequence[str]
    interaction_channels: Sequence[str]
    music_policy: str
    processing_sensor_contract: Sequence[str] = ()
    autonomy_capabilities: Sequence[str] = ()
    autonomy_level: str = "high"
    initial_policy: str = "sensor_fusion_mission_orchestrator"
    voice_policy: str = "command_and_mission_input"
    world_3d_policy: str = "context_and_collision_complement"

    def metadata(self) -> Dict[str, Any]:
        return {
            "robot": self.robot,
            "primary_camera_role": self.primary_camera_role,
            "primary_state": list(self.primary_state),
            "primary_navigation": list(self.primary_navigation),
            "primary_safety": list(self.primary_safety),
            "interaction_channels": list(self.interaction_channels),
            "music_policy": self.music_policy,
            "processing_sensor_contract": list(self.processing_sensor_contract),
            "autonomy_capabilities": list(self.autonomy_capabilities),
            "autonomy_level": self.autonomy_level,
            "initial_policy": self.initial_policy,
            "voice_policy": self.voice_policy,
            "world_3d_policy": self.world_3d_policy,
        }


PROFILES: Dict[str, RobotIntelligenceProfile] = {
    "Manipulator": RobotIntelligenceProfile(
        robot="Manipulator",
        primary_camera_role="workspace_camera",
        primary_state=("joint_telemetry", "base_imu", "gripper_imu", "joint_torque_references", "telemetry_heading", "battery"),
        primary_navigation=(),
        primary_safety=("workspace_camera", "sonar", "processing_world_3d"),
        interaction_channels=("voice", "music", "keyboard", "joystick", "leap_motion"),
        music_policy="actuation_allowed_with_operator_enable",
        processing_sensor_contract=(
            "joint_telemetry", "base_imu", "gripper_imu", "joint_torque_references",
            "sonar", "telemetry_heading", "battery", "grip_pressure", "collision", "processing_world_3d", "workspace_camera",
        ),
        autonomy_capabilities=(
            "workspace_inspection", "360_workspace_scan", "object_pick", "object_place",
            "pose_hold", "return_home", "gripper_calibration", "rhythm_motion", "safe_stop",
        ),
    ),
    "Vehicle": RobotIntelligenceProfile(
        robot="Vehicle",
        primary_camera_role="front_camera",
        primary_state=("chassis_imu", "position_telemetry", "drive_torque", "battery", "communication_link"),
        primary_navigation=("gps", "telemetry_heading", "front_camera", "lidar"),
        primary_safety=("lidar", "processing_world_3d"),
        interaction_channels=("voice", "keyboard", "joystick", "leap_motion"),
        music_policy="context_only",
        processing_sensor_contract=(
            "position_telemetry", "chassis_imu", "drive_torque", "lidar", "gps", "telemetry_heading", "battery",
            "communication_link", "collision", "processing_world_3d", "front_camera",
        ),
        autonomy_capabilities=(
            "terrain_inspection", "object_localization", "exit_finding", "area_patrol",
            "perimeter_scan", "corridor_scan", "target_follow", "dock", "hold_position",
            "return_home", "safe_stop",
        ),
    ),
    "Drone": RobotIntelligenceProfile(
        robot="Drone",
        primary_camera_role="front_camera",
        primary_state=("flight_imu", "position_telemetry", "battery", "communication_link"),
        primary_navigation=("gps", "telemetry_heading", "front_camera", "downward_sonar"),
        primary_safety=("downward_sonar", "processing_world_3d"),
        interaction_channels=("voice", "keyboard", "joystick", "leap_motion"),
        music_policy="context_only",
        processing_sensor_contract=(
            "position_telemetry", "flight_imu", "downward_sonar", "gps", "telemetry_heading", "battery",
            "communication_link", "collision", "processing_world_3d", "front_camera",
        ),
        autonomy_capabilities=(
            "terrain_inspection", "object_localization", "exit_finding", "aerial_scan",
            "orbit_point", "search_pattern", "altitude_hold", "return_home",
            "emergency_land", "safe_hover",
        ),
    ),
}


def profile_for_robot(robot: Any) -> RobotIntelligenceProfile:
    return PROFILES[canonical_robot(robot)]


def _valid_sensor_manifest(value: Any) -> Dict[str, Any]:
    manifest = _mapping(value)
    if not manifest:
        return {}
    if str(manifest.get("schema", "") or "") != ROBOT_SENSOR_MANIFEST_SCHEMA:
        return {}
    try:
        version = int(manifest.get("softwareVersion", -1))
    except (TypeError, ValueError):
        return {}
    return manifest if version == SOFTWARE_VERSION else {}


def _processing_sensor_manifest(state: Any) -> Dict[str, Any]:
    scene = _mapping(getattr(state, "perception_scene", {}))
    direct = _valid_sensor_manifest(scene.get("sensorManifest"))
    if direct:
        return direct
    snapshot = _mapping(getattr(state, "snapshot", {}))
    direct = _valid_sensor_manifest(snapshot.get("sensorManifest"))
    if direct:
        return direct
    nested_scene = _mapping(_mapping(snapshot.get("aiPerception")).get("scene"))
    return _valid_sensor_manifest(nested_scene.get("sensorManifest"))


def _processing_inputs(state: Any) -> Dict[str, Any]:
    return _mapping(_processing_sensor_manifest(state).get("inputs"))


def _audio_context(state: Any) -> Dict[str, Any]:
    direct = _mapping(getattr(state, "audio_context", {}))
    if direct:
        return direct
    return _mapping(getattr(state, "voice_context", {}))


def _audio_available(state: Any) -> bool:
    audio = _audio_context(state)
    if not audio:
        return False
    backend = str(audio.get("audio_backend", "") or "").strip().lower()
    age = safe_float(audio.get("audio_age", 999.0), 999.0, 0.0)
    running = bool(audio.get("running", False))
    return running or (backend not in {"", "off", "stopped", "pyaudio_stopped"} and age <= 2.0)


def _audio_summary(state: Any) -> Dict[str, Any]:
    audio = _audio_context(state)
    features = audio.get("audio_features", [])
    if not isinstance(features, (list, tuple)):
        features = []
    return {
        "available": _audio_available(state),
        "speech_text": str(audio.get("text", "") or ""),
        "speech_intent": str(audio.get("intent", "none") or "none"),
        "speech_confidence": safe_float(audio.get("conf", audio.get("confidence", 0.0)), 0.0, 0.0, 1.0),
        "audio_level": safe_float(audio.get("audio_level", 0.0), 0.0, 0.0),
        "audio_meter": safe_float(audio.get("audio_meter", 0.0), 0.0, 0.0),
        "audio_age_s": safe_float(audio.get("audio_age", 999.0), 999.0, 0.0),
        "audio_backend": str(audio.get("audio_backend", "off") or "off"),
        "speech_backend": str(audio.get("speech_backend", "idle") or "idle"),
        "feature_vector": [safe_float(v, 0.0) for v in list(features)[:16]],
    }


def _channel_manifest(state: Any, profile: RobotIntelligenceProfile) -> list[Dict[str, Any]]:
    raw = _processing_sensor_manifest(state).get("channels", [])
    if isinstance(raw, list):
        channels = [dict(item) for item in raw if isinstance(item, Mapping)]
    else:
        channels = []
    if channels:
        camera_available = bool(getattr(state, "robot_camera_available", False))
        audio_available = _audio_available(state)
        camera_name = profile.primary_camera_role
        for channel in channels:
            name = str(channel.get("name", ""))
            if name == "compass":
                channel["name"] = "telemetry_heading"
                channel["role"] = "heading_inside_robot_telemetry"
                channel["transport"] = channel.get("transport", "ws:9000")
            if name == camera_name and camera_available:
                channel["available"] = True
                channel["transport"] = "aibot_robot_camera"
            elif name == "audio_microphone":
                channel["available"] = audio_available
                channel["transport"] = "aibot_local"
        return channels
    # If no live manifest has arrived yet, expose the expected channels as
    # unavailable rather than inventing sensor readings.
    expected = set(profile.primary_state) | set(profile.primary_navigation) | set(profile.primary_safety)
    return [
        {"name": name, "expected": True, "available": False, "authority": "profile", "transport": "unknown"}
        for name in sorted(expected)
    ]


def _telemetry_heading(state: Any, robot: str) -> Dict[str, Any]:
    sensors = _mapping(getattr(state, "sensors", {}))
    values = _mapping(_processing_sensor_manifest(state).get("values"))
    processing_heading = _mapping(values.get("compass"))
    if processing_heading:
        return processing_heading

    heading_keys = {
        "Manipulator": ("compass_heading_deg", "heading_deg", "base_yaw_deg"),
        "Vehicle": ("compass_heading_deg", "heading_deg", "vehicle_yaw_deg"),
        "Drone": ("compass_heading_deg", "heading_deg", "drone_yaw_deg", "att_yaw_deg"),
    }[robot]
    heading_present = any(key in sensors for key in heading_keys[:2])
    raw_available = any(key in sensors for key in ("mag_x", "mag_y", "mag_z"))
    source = str(sensors.get("compass_source", "") or "").strip() or ("runtime_telemetry" if heading_present else "unavailable")
    return {
        "available": bool(heading_present),
        "heading_deg": _float(sensors, heading_keys),
        "mag_x_raw": _float(sensors, ("mag_x",)),
        "mag_y_raw": _float(sensors, ("mag_y",)),
        "mag_z_raw": _float(sensors, ("mag_z",)),
        "magnetometer_raw_available": raw_available,
        "source": source,
        "role": "absolute_heading_and_north_reference",
    }


def _attitude(state: Any, robot: str) -> Dict[str, Any]:
    sensors = _mapping(getattr(state, "sensors", {}))
    intel_values = _mapping(_processing_sensor_manifest(state).get("values"))
    processing_att = _mapping(intel_values.get("attitude"))
    if processing_att:
        return processing_att
    if robot == "Manipulator":
        return {
            "pitch_deg": _float(sensors, ("base_pitch_deg", "manipulator_pitch_deg")),
            "roll_deg": _float(sensors, ("base_roll_deg", "manipulator_roll_deg")),
            "yaw_deg": _float(sensors, ("base_yaw_deg", "heading_deg")),
            "source": "runtime_telemetry",
            "world_grid_level": True,
            "support_surface_tilted": True,
        }
    if robot == "Vehicle":
        return {
            "pitch_deg": _float(sensors, ("vehicle_pitch_deg",)),
            "roll_deg": _float(sensors, ("vehicle_roll_deg",)),
            "yaw_deg": _float(sensors, ("vehicle_yaw_deg", "heading_deg")),
            "source": "runtime_telemetry",
            "world_grid_level": True,
            "support_surface_tilted": True,
        }
    return {
        "pitch_deg": _float(sensors, ("drone_pitch_deg",)),
        "roll_deg": _float(sensors, ("drone_roll_deg",)),
        "yaw_deg": _float(sensors, ("drone_yaw_deg", "att_yaw_deg", "heading_deg")),
        "source": "runtime_telemetry",
        "world_grid_level": True,
        "support_surface_tilted": False,
    }


def _raw_imu(sensors: Mapping[str, Any], prefix: str, role: str, *, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Dict[str, Any]:
    return {
        "role": role,
        "available": any((prefix + suffix) in sensors for suffix in ("ax", "ay", "az", "gx", "gy", "gz")),
        "ax_raw": _float(sensors, (prefix + "ax",)),
        "ay_raw": _float(sensors, (prefix + "ay",)),
        "az_raw": _float(sensors, (prefix + "az",)),
        "gx_raw": _float(sensors, (prefix + "gx",)),
        "gy_raw": _float(sensors, (prefix + "gy",)),
        "gz_raw": _float(sensors, (prefix + "gz",)),
        "roll_deg": safe_float(roll, 0.0),
        "pitch_deg": safe_float(pitch, 0.0),
        "yaw_deg": safe_float(yaw, 0.0),
    }


def _manipulator_imus(state: Any) -> Dict[str, Any]:
    sensors = _mapping(getattr(state, "sensors", {}))
    values = _mapping(_processing_sensor_manifest(state).get("values"))
    base = _mapping(values.get("base_imu"))
    gripper = _mapping(values.get("gripper_imu"))
    if not base:
        base = _raw_imu(
            sensors, "mpu2_", "base_structure",
            roll=_float(sensors, ("base_roll_deg",)),
            pitch=_float(sensors, ("base_pitch_deg",)),
            yaw=_float(sensors, ("base_yaw_deg", "heading_deg")),
        )
    if not gripper:
        gripper = _raw_imu(
            sensors, "mpu1_", "gripper_wrist",
            roll=_float(sensors, ("gripper_roll_deg", "wrist_roll_deg")),
            pitch=_float(sensors, ("gripper_pitch_deg", "wrist_pitch_imu_deg")),
            yaw=_float(sensors, ("wrist_rot_deg", "gimbal_yaw_deg")),
        )
    return {"base_mpu2": base, "gripper_mpu1": gripper}


def _position(state: Any, robot: str) -> Dict[str, Any]:
    sensors = _mapping(getattr(state, "sensors", {}))
    scene = _mapping(getattr(state, "perception_scene", {}))
    pose = _mapping(scene.get("pose"))
    out = {
        "x_m": safe_float(pose.get("x_m", 0.0), 0.0),
        "y_m": safe_float(pose.get("y_m", 0.0), 0.0),
        "z_m": safe_float(pose.get("z_m", 0.0), 0.0),
    }
    if robot == "Manipulator":
        out["joint_deg"] = pose.get("joint_deg", [])
    elif robot == "Vehicle":
        if not pose:
            out["x_scene"] = _float(sensors, ("vehicle_x",))
            out["z_scene"] = _float(sensors, ("vehicle_z",))
    else:
        out["altitude_m"] = safe_float(
            pose.get("altitude_m", _float(sensors, ("alt_cm",), 0.0) / 100.0), 0.0
        )
    return out


def _robot_features(state: Any, robot: str) -> Dict[str, Any]:
    sensors = _mapping(getattr(state, "sensors", {}))
    gps = _mapping(getattr(state, "gps", {}))
    obstacles = list(getattr(state, "perception_obstacles", []) or [])
    battery = battery_percent(state)
    heading = _telemetry_heading(state, robot)
    common: Dict[str, Any] = {
        "attitude": _attitude(state, robot),
        "telemetry": {
            "heading_available": bool(heading.get("available", False)),
            "heading_deg": safe_float(heading.get("heading_deg", 0.0), 0.0),
            "heading_source": str(heading.get("source", "unavailable") or "unavailable"),
            "mag_x_raw": safe_float(heading.get("mag_x_raw", 0.0), 0.0),
            "mag_y_raw": safe_float(heading.get("mag_y_raw", 0.0), 0.0),
            "mag_z_raw": safe_float(heading.get("mag_z_raw", 0.0), 0.0),
            "magnetometer_raw_available": bool(heading.get("magnetometer_raw_available", False)),
        },
        "position": _position(state, robot),
        "gps": gps,
        "battery_pct": battery,
        "nearest_world_obstacles": obstacles[:24],
    }
    if robot == "Manipulator":
        common.update({
            "imus": _manipulator_imus(state),
            "joint_torque_references": manipulator_joint_torque_references(state),
            "sonar_cm": _float(sensors, ("sonar_cm",)),
            "grip_pressure_ma": _float(sensors, ("grip_pressure_ma", "ina1_ma")),
            "grip_pressure_est": _float(sensors, ("grip_pressure_est",)),
            "collision": _bool(sensors, ("collision",)),
            "joint_deg": [
                _float(sensors, ("base_deg",)), _float(sensors, ("upper_deg",)),
                _float(sensors, ("fore_deg",)), _float(sensors, ("forearm_roll_deg",)),
                _float(sensors, ("wrist_pitch_deg",)), _float(sensors, ("wrist_rot_deg",)),
                _float(sensors, ("grip_deg",)),
            ],
        })
    elif robot == "Vehicle":
        common.update({
            "lidar_cm": _float(sensors, ("lidar_cm", "lidar", "range_cm")),
            "drive_torque": vehicle_drive_torque_references(state),
            "communication_quality_pct": communication_quality_percent(state),
            "link_latency_ms": _float(sensors, ("link_ms",), -1.0),
            "camera_pan_deg": _float(sensors, ("vehicle_cam_pan_deg",)),
            "camera_tilt_deg": _float(sensors, ("vehicle_cam_tilt_deg",)),
        })
    else:
        common.update({
            "sonar_down_cm": _float(sensors, ("drone_sonar_down_cm", "sonar_down_cm", "sonar_vertical_cm", "drone_scan_cm", "sonar_cm")),
            "altitude_m": _float(sensors, ("alt_cm",), 0.0) / 100.0,
            "communication_quality_pct": communication_quality_percent(state),
            "link_latency_ms": _float(sensors, ("link_ms",), -1.0),
            "camera_pan_deg": _float(sensors, ("drone_cam_pan_deg",)),
            "camera_tilt_deg": _float(sensors, ("drone_cam_tilt_deg",)),
        })
    return common


def build_robot_intelligence_context(state: Any, *, now: float | None = None) -> Dict[str, Any]:
    ts = float(now if now is not None else time.time())
    robot = canonical_robot(getattr(state, "robot", "Manipulator"))
    profile = profile_for_robot(robot)
    inputs = _processing_inputs(state)
    source = str(inputs.get("active_source", "none") or "none").strip().lower()
    if source == "leap_motion":
        source = "leap"
    human_override = source in HUMAN_CONTROL_SOURCES and source not in {"none", "aibot"}
    channels = _channel_manifest(state, profile)
    missing = [
        str(ch.get("name")) for ch in channels
        if bool(ch.get("expected")) and not bool(ch.get("available"))
        and str(ch.get("authority", "")) in {"primary_vision", "primary_state", "primary_navigation", "primary_safety"}
    ]
    last_tel = safe_float(getattr(state, "last_telemetry_ts", 0.0), 0.0)
    last_perception = safe_float(getattr(state, "last_perception_ts", 0.0), 0.0)
    return {
        "schema": ROBOT_INTELLIGENCE_SCHEMA,
        "softwareVersion": SOFTWARE_VERSION,
        "robot": robot,
        "profile": profile.metadata(),
        "strategy": strategy_for_robot(robot).metadata(),
        "resources": mission_resource_status(state, robot),
        "control": {
            "active_source": source,
            "human_override": human_override,
            "human_demonstration": human_override,
            "demonstration_source": source if human_override else "",
            "autonomy_allowed": not human_override,
            "voice_allowed": True,
            "music_actuation_allowed": robot == "Manipulator" and not human_override,
            "priority_order": [
                "emergency_and_spoken_safety",
                "direct_human",
                "ordinary_voice",
                "mission",
                "vision_object",
                "music_audio",
                "model_autonomy",
            ],
        },
        "audio": _audio_summary(state),
        "channels": channels,
        "missing_primary_channels": missing,
        "features": _robot_features(state, robot),
        "freshness": {
            "telemetry_age_s": max(0.0, ts - last_tel) if last_tel else 999.0,
            "processing_3d_age_s": max(0.0, ts - last_perception) if last_perception else 999.0,
        },
        "camera_fusion": {
            "primary": profile.primary_camera_role,
            "processing_world_3d": "complementary_context",
            "prefer_physical_robot_camera_for_object_or_navigation_vision": True,
        },
    }


def refresh_state_intelligence_context(state: Any, *, now: float | None = None) -> Dict[str, Any]:
    context = build_robot_intelligence_context(state, now=now)
    setattr(state, "intelligence_context", context)
    control = _mapping(context.get("control"))
    setattr(state, "active_input_source", str(control.get("active_source", "none")))
    setattr(state, "human_override", bool(control.get("human_override", False)))
    setattr(state, "sensor_channels", list(context.get("channels", []) or []))
    return context


__all__ = [
    "HUMAN_CONTROL_SOURCES",
    "PROFILES",
    "RobotIntelligenceProfile",
    "build_robot_intelligence_context",
    "profile_for_robot",
    "refresh_state_intelligence_context",
]
