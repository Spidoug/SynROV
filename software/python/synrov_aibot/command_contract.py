"""Canonical predefined command semantics shared by SynROV Python backends.

These values describe *intent*, not hardware calibration.  They keep explicit
commands deterministic enough to work even while a learned model is still
weak, while leaving a smaller share of the output available for learned
adaptation.  Processing remains the authority for robot selection, limits and
final execution.
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from .dataset import canonical_robot
from .primitives import safe_float

# Core/legacy GUI voice-intent names.  Dimensions match the existing V1 model
# contract and therefore do not change saved model feature/target shapes.
PREDEFINED_MODEL_PRIORS: Dict[str, Dict[str, Tuple[float, ...]]] = {
    "Manipulator": {
        "arm_up": (0.0, 12.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "arm_down": (0.0, -12.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "arm_left": (18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "arm_right": (-18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "fore_up": (0.0, 0.0, 12.0, 0.0, 0.0, 0.0, 0.0),
        "fore_down": (0.0, 0.0, -12.0, 0.0, 0.0, 0.0, 0.0),
        "roll_left": (0.0, 0.0, 0.0, 14.0, 0.0, 0.0, 0.0),
        "roll_right": (0.0, 0.0, 0.0, -14.0, 0.0, 0.0, 0.0),
        "grip_open": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 22.0),
        "grip_close": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -22.0),
        "wrist_up": (0.0, 0.0, 0.0, 0.0, 14.0, 0.0, 0.0),
        "wrist_down": (0.0, 0.0, 0.0, 0.0, -14.0, 0.0, 0.0),
        "wrist_left": (0.0, 0.0, 0.0, 0.0, 0.0, 16.0, 0.0),
        "wrist_right": (0.0, 0.0, 0.0, 0.0, 0.0, -16.0, 0.0),
    },
    "Vehicle": {
        "vehicle_forward": (0.40, 0.0, 0.0, 0.0),
        "vehicle_back": (-0.40, 0.0, 0.0, 0.0),
        "vehicle_left": (0.14, -0.34, 0.0, 0.0),
        "vehicle_right": (0.14, 0.34, 0.0, 0.0),
        "vehicle_stop": (0.0, 0.0, 0.0, 0.0),
    },
    "Drone": {
        "drone_up": (0.35, 0.0, 0.0, 0.0, 0.0, 0.0),
        "drone_down": (-0.35, 0.0, 0.0, 0.0, 0.0, 0.0),
        "drone_left": (0.0, 0.0, 0.0, 0.0, 0.30, 0.0),
        "drone_right": (0.0, 0.0, 0.0, 0.0, -0.30, 0.0),
        "drone_forward": (0.0, 0.0, 0.0, 0.0, 0.0, 0.30),
        "drone_back": (0.0, 0.0, 0.0, 0.0, 0.0, -0.30),
    },
}

# These commands are semantic actions in Processing and must never be reduced
# to a regression vector.  A zero vector cannot express HOME/TAKEOFF/LAND.
PREDEFINED_ACTIONS: Dict[Tuple[str, str], str] = {
    ("Manipulator", "home"): "home",
    ("Drone", "drone_takeoff"): "takeoff",
    ("Drone", "drone_land"): "land",
}

# Zero is the *correct* actuator command for these intents.  Magnitude gates
# must not discard them as if they were failed predictions.
PREDEFINED_ZERO_COMMANDS = {
    ("Vehicle", "vehicle_stop"),
}


def predefined_action(robot: Any, intent: Any) -> str:
    return PREDEFINED_ACTIONS.get((canonical_robot(robot), str(intent or "").strip()), "")


def predefined_prior(robot: Any, intent: Any) -> Optional[List[float]]:
    values = PREDEFINED_MODEL_PRIORS.get(canonical_robot(robot), {}).get(str(intent or "").strip())
    return list(values) if values is not None else None


def zero_command_is_valid(robot: Any, intent: Any) -> bool:
    return (canonical_robot(robot), str(intent or "").strip()) in PREDEFINED_ZERO_COMMANDS


def blend_with_predefined_prior(
    robot: Any,
    intent: Any,
    prediction: List[float],
    *,
    command_confidence: float = 1.0,
) -> List[float]:
    """Anchor an explicit predefined command while preserving learned nuance.

    At normal speech/text confidence the canonical command contributes roughly
    80–90% of the result.  This prevents an immature model from reversing or
    nulling a known command, yet still permits learned magnitude adaptation.
    """
    prior = predefined_prior(robot, intent)
    if prior is None:
        return [safe_float(v) for v in prediction]
    values = [safe_float(v) for v in prediction]
    if len(values) < len(prior):
        values.extend([0.0] * (len(prior) - len(values)))
    confidence = max(0.0, min(1.0, safe_float(command_confidence, 1.0)))
    prior_weight = 0.78 + 0.12 * confidence
    learned_weight = 1.0 - prior_weight
    return [prior[i] * prior_weight + values[i] * learned_weight for i in range(len(prior))]


def audit_predefined_contract() -> Dict[str, int]:
    return {robot: len(commands) for robot, commands in PREDEFINED_MODEL_PRIORS.items()}


__all__ = [
    "PREDEFINED_MODEL_PRIORS",
    "PREDEFINED_ACTIONS",
    "PREDEFINED_ZERO_COMMANDS",
    "predefined_action",
    "predefined_prior",
    "zero_command_is_valid",
    "blend_with_predefined_prior",
    "audit_predefined_contract",
]
