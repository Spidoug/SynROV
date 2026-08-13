"""Shared SynROV application protocol.

The transport is intentionally independent from the protocol: Web/AiBot use the
Processing WebSocket service, ROS uses rosbridge, and dedicated image streams may
use their own sockets. Every SynROV message itself uses the same envelope.

SynROV version 1 uses ``softwareVersion: 1`` in every application envelope.
Protocol names, source names, schemas, directories, and variables use stable names.
"""
from __future__ import annotations

import time
import re
from typing import Any, Dict, Iterable, Mapping, Optional, Set, Tuple

SOFTWARE_VERSION = 1
PROTOCOL_NAME = "synrov"

SOURCE_PROCESSING = "processing"
SOURCE_AIBOT = "aibot"
SOURCE_WEB = "web"
SOURCE_ROS = "ros"

MESSAGE_CLIENT = "client"
MESSAGE_SYNC = "sync"
MESSAGE_CONTROL = "control"
MESSAGE_CAMERA = "camera"
MESSAGE_MODE = "mode"
MESSAGE_JOYSTICK = "joystick"
MESSAGE_TOGGLE = "toggle"
MESSAGE_ACTION = "action"
MESSAGE_CONNECT = "connect"
MESSAGE_STATE = "state"
MESSAGE_TELEMETRY = "telemetry"
MESSAGE_PERCEPTION_SUBSCRIPTION = "perception.subscription"
MESSAGE_PERCEPTION_FRAME = "perception.frame"
MESSAGE_CAMERA_CONTROL = "robot_camera.control"
MESSAGE_CAMERA_FRAME = "robot_camera.frame"
MESSAGE_HEARTBEAT = "heartbeat"

# Persistent document identifiers use stable version-1 schema names.
DATASET_SCHEMA = "synrov.training"
SNAPSHOT_SCHEMA = "synrov.snapshot"
COMMANDS_SCHEMA = "synrov.commands"
MODEL_SCHEMA = "synrov.model"
ROBOT_SENSOR_MANIFEST_SCHEMA = "synrov.robot-sensors"
ROBOT_INTELLIGENCE_SCHEMA = "synrov.robot-intelligence"
DANCE_SCHEMA = "synrov.aibot.dance"
SOURCES = frozenset({SOURCE_PROCESSING, SOURCE_AIBOT, SOURCE_WEB, SOURCE_ROS})
LANGUAGE_CODE_RE = re.compile(r"^[a-z]{2,3}(?:-[a-z0-9]{2,8})*$")


def normalize_language(value: Any, default: str = "") -> str:
    """Normalize a Processing language-pack code without hard-coding languages."""
    text = str(value or "").strip().lower().replace("_", "-")
    if LANGUAGE_CODE_RE.fullmatch(text):
        return text
    fallback = str(default or "").strip().lower().replace("_", "-")
    return fallback if LANGUAGE_CODE_RE.fullmatch(fallback) else ""


def message_envelope(
    message_type: str,
    payload: Optional[Mapping[str, Any]],
    *,
    source: str,
    seq: Optional[int] = None,
    timestamp_ms: Optional[int] = None,
) -> Dict[str, Any]:
    """Build the canonical SynROV envelope used by every application transport."""
    envelope: Dict[str, Any] = {
        "protocol": PROTOCOL_NAME,
        "softwareVersion": SOFTWARE_VERSION,
        "messageType": str(message_type or "").strip(),
        "source": str(source or "").strip().lower(),
        "timestampMs": int(time.time() * 1000 if timestamp_ms is None else timestamp_ms),
        "seq": int(0 if seq is None else seq),
        "payload": dict(payload or {}),
    }
    return envelope



def infer_message_type(payload: Mapping[str, Any], default: str = MESSAGE_CONTROL) -> str:
    """Classify an application payload using the shared command vocabulary."""
    if not isinstance(payload, Mapping):
        return default
    if "client" in payload:
        return MESSAGE_CLIENT
    if "sync" in payload:
        return MESSAGE_SYNC
    if "control" in payload:
        return MESSAGE_CONTROL
    if "camera" in payload:
        return MESSAGE_CAMERA
    if "mode" in payload:
        return MESSAGE_MODE
    if "joystickType" in payload:
        return MESSAGE_JOYSTICK
    if "toggle" in payload:
        return MESSAGE_TOGGLE
    if "action" in payload:
        return MESSAGE_ACTION
    if payload.get("connect") is True:
        return MESSAGE_CONNECT
    return default


def envelope_supported(message: Mapping[str, Any], accepted_types: Optional[Iterable[str]] = None) -> bool:
    """Validate the SynROV version-1 application envelope."""
    if not isinstance(message, Mapping):
        return False
    if str(message.get("protocol", "") or "").strip().lower() != PROTOCOL_NAME:
        return False
    try:
        version = int(message.get("softwareVersion", -1))
    except (TypeError, ValueError):
        return False
    if version != SOFTWARE_VERSION:
        return False
    message_type = str(message.get("messageType", "") or "").strip()
    source = str(message.get("source", "") or "").strip().lower()
    if not message_type or source not in SOURCES or not isinstance(message.get("payload"), Mapping):
        return False
    try:
        int(message["timestampMs"])
        int(message["seq"])
    except (KeyError, TypeError, ValueError):
        return False
    if accepted_types is not None:
        allowed: Set[str] = {str(item) for item in accepted_types}
        if message_type not in allowed:
            return False
    return True


def unpack_message(
    message: Mapping[str, Any],
    accepted_types: Optional[Iterable[str]] = None,
) -> Tuple[str, Mapping[str, Any]]:
    """Return ``(messageType, payload)`` or ``("", {})`` when invalid."""
    if not envelope_supported(message, accepted_types):
        return "", {}
    return str(message["messageType"]), message["payload"]  # type: ignore[index]


def iter_state_blocks(payload: Mapping[str, Any]) -> Iterable[Mapping[str, Any]]:
    """Yield state-shaped dictionaries from a state payload."""
    if not isinstance(payload, Mapping):
        return
    yield payload
    for key in ("state", "snapshot", "system"):
        child = payload.get(key)
        if isinstance(child, Mapping):
            yield child
    snapshot = payload.get("snapshot")
    if isinstance(snapshot, Mapping):
        system = snapshot.get("system")
        if isinstance(system, Mapping):
            yield system


def processing_language(payload: Mapping[str, Any], default: str = "") -> str:
    """Extract Processing's language from any supported state payload shape."""
    for block in iter_state_blocks(payload):
        lang = normalize_language(block.get("language"), "")
        if lang:
            return lang
    return normalize_language(default, "")


__all__ = [
    "SOFTWARE_VERSION",
    "PROTOCOL_NAME",
    "SOURCE_PROCESSING",
    "SOURCE_AIBOT",
    "SOURCE_WEB",
    "SOURCE_ROS",
    "MESSAGE_CLIENT",
    "MESSAGE_SYNC",
    "MESSAGE_CONTROL",
    "MESSAGE_CAMERA",
    "MESSAGE_MODE",
    "MESSAGE_JOYSTICK",
    "MESSAGE_TOGGLE",
    "MESSAGE_ACTION",
    "MESSAGE_CONNECT",
    "MESSAGE_STATE",
    "MESSAGE_TELEMETRY",
    "MESSAGE_PERCEPTION_SUBSCRIPTION",
    "MESSAGE_PERCEPTION_FRAME",
    "MESSAGE_CAMERA_CONTROL",
    "MESSAGE_CAMERA_FRAME",
    "MESSAGE_HEARTBEAT",
    "DATASET_SCHEMA",
    "SNAPSHOT_SCHEMA",
    "COMMANDS_SCHEMA",
    "MODEL_SCHEMA",
    "ROBOT_SENSOR_MANIFEST_SCHEMA",
    "ROBOT_INTELLIGENCE_SCHEMA",
    "DANCE_SCHEMA",
    "SOURCES",
    "LANGUAGE_CODE_RE",
    "normalize_language",
    "message_envelope",
    "infer_message_type",
    "envelope_supported",
    "unpack_message",
    "iter_state_blocks",
    "processing_language",
]
