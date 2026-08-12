"""Canonical robot identifiers for SynROV AiBot Version 1."""
from __future__ import annotations

from typing import Any

ROBOTS = ("Manipulator", "Vehicle", "Drone")
TARGET_DIMS = {"Manipulator": 7, "Vehicle": 4, "Drone": 6}


def canonical_robot(value: Any, default: str = "Manipulator") -> str:
    text = str(value or "").strip().lower()
    if text.startswith(("manip", "arm", "bra", "robot arm")):
        return "Manipulator"
    if text.startswith(("veh", "car", "rover", "veic")):
        return "Vehicle"
    if text.startswith(("drone", "quad", "uav")):
        return "Drone"
    return default if default in ROBOTS else "Manipulator"


__all__ = ["ROBOTS", "TARGET_DIMS", "canonical_robot"]
