"""Single camera/gimbal contract shared by AiBot control paths."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Tuple

from .primitives import safe_float
from .robot_types import canonical_robot

CAMERA_PAN_MIN_DEG = -60.0
CAMERA_PAN_MAX_DEG = 60.0
CAMERA_TILT_MIN_DEG = -20.0
CAMERA_TILT_MAX_DEG = 20.0
CAMERA_DEFAULTS: Dict[str, Tuple[float, float]] = {
    "Vehicle": (0.0, 0.0),
    "Drone": (0.0, -10.0),
}


@dataclass(frozen=True)
class CameraPose:
    pan_deg: float
    tilt_deg: float


def clamp_pan(value: Any) -> float:
    return max(CAMERA_PAN_MIN_DEG, min(CAMERA_PAN_MAX_DEG, safe_float(value, 0.0)))


def clamp_tilt(value: Any) -> float:
    return max(CAMERA_TILT_MIN_DEG, min(CAMERA_TILT_MAX_DEG, safe_float(value, 0.0)))


def clamp_camera_pose(pan: Any, tilt: Any) -> CameraPose:
    return CameraPose(clamp_pan(pan), clamp_tilt(tilt))


def default_camera_pose(robot: Any) -> CameraPose:
    name = canonical_robot(robot)
    pan, tilt = CAMERA_DEFAULTS.get(name, (0.0, 0.0))
    return CameraPose(pan, tilt)


__all__ = [
    "CAMERA_PAN_MIN_DEG",
    "CAMERA_PAN_MAX_DEG",
    "CAMERA_TILT_MIN_DEG",
    "CAMERA_TILT_MAX_DEG",
    "CAMERA_DEFAULTS",
    "CameraPose",
    "clamp_pan",
    "clamp_tilt",
    "clamp_camera_pose",
    "default_camera_pose",
]
