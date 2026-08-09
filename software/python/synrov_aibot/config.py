"""Application constants, paths and robot key definitions."""
from __future__ import annotations

from pathlib import Path

from .modern_runtime import DEFAULT_URI
from .safety import MANIP_KEYS, MANIP_POSE_LIMITS

APP_NAME = "SynROV AiBot"
APP_DIR = Path(__file__).resolve().parent
DATA_DIR = APP_DIR / "synrov_multimodal_data"
FRAME_DIR = DATA_DIR / "frames"
MODEL_DIR = DATA_DIR / "models"
DATASET_PATH = DATA_DIR / "datasets" / "manipulator_training.jsonl"

PROCESSING_MANIP_LIMITS = MANIP_POSE_LIMITS
MANIP_INCREMENTAL_TARGET_MODE = True
MANIP_MODEL_RATE_LIMIT_DPS = {key: 120.0 for key in MANIP_KEYS}
MANIP_TRAINING_RATE_LIMIT_DPS = {key: 180.0 for key in MANIP_KEYS}
MANIP_MODEL_MIN_DT_S = 0.08
MANIP_MODEL_MAX_DT_S = 0.35
MANIP_HINT_PROPORTIONAL_GAIN = 0.35

VEHICLE_KEYS = ["throttle", "steer", "camPan", "camTilt"]
DRONE_KEYS   = ["throttle", "yaw", "pitch", "roll", "strafe", "forward"]
TELEMETRY_KEYS = ["base", "upper", "fore", "forearm_roll", "wrist_pitch", "wrist_rot", "grip",
                  "battery", "temperature", "distance"]
VOICE_INTENTS  = ["base_left", "base_right", "arm_up", "arm_down", "grip_open", "grip_close", "home", "stop_mission"]
VISION_KEYS    = ["confidence", "dx", "dy", "area", "width", "height", "centered", "close"]
AUDIO_KEYS     = ["level", "peak", "tempo", "beat", "centroid", "zcr", "rms", "energy", "low", "mid", "high", "flux"]

__all__ = [name for name in globals() if name.isupper()]
