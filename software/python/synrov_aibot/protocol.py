"""SynROV WebSocket protocol V1 helpers.

Processing is the source of truth for robot identity and interface language.
All Python-originated commands use the same V1 envelope accepted by the HTML
console and by the Processing WebSocket command dispatcher.
"""
from __future__ import annotations

import time
from typing import Any, Dict, Iterable, Mapping, Optional

SYNROV_VERSION = 1
CONTROL_SCHEMA = "synrov.control.v1"
STATE_SCHEMA = "synrov.state.v1"
DATASET_SCHEMA = "synrov.training.v1"
SNAPSHOT_SCHEMA = "synrov.snapshot.v1"
COMMANDS_SCHEMA = "synrov.commands.v1"
MODEL_SCHEMA = "synrov.model.v1"
SUPPORTED_LANGUAGES = frozenset({"pt", "en"})


def normalize_language(value: Any, default: str = "") -> str:
    """Return ``pt``/``en`` from a Processing language value."""
    text = str(value or "").strip().lower().replace("_", "-")
    if text.startswith("pt"):
        return "pt"
    if text.startswith("en"):
        return "en"
    return default if default in SUPPORTED_LANGUAGES else ""


def control_envelope(
    payload: Optional[Mapping[str, Any]],
    *,
    source: str,
    seq: Optional[int] = None,
    robot: Optional[str] = None,
    sent_at: Optional[float] = None,
) -> Dict[str, Any]:
    """Attach the mandatory V1 command metadata without replacing caller data."""
    out = dict(payload or {})
    # The V1 transport metadata is authoritative; callers cannot override it.
    out["schema"] = CONTROL_SCHEMA
    out["version"] = SYNROV_VERSION
    out.setdefault("source", source)
    out.setdefault("origin", source)
    out.setdefault("sentAt", float(time.time() if sent_at is None else sent_at))
    if seq is not None:
        out.setdefault("seq", int(seq))
    if robot:
        out.setdefault("detectedRobot", str(robot))
    return out


def iter_state_blocks(payload: Mapping[str, Any]) -> Iterable[Mapping[str, Any]]:
    """Yield state-shaped dictionaries in the order used by SynROV V1."""
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
    """Extract Processing's language from any supported state packet shape."""
    for block in iter_state_blocks(payload):
        lang = normalize_language(block.get("language"), "")
        if lang:
            return lang
    return normalize_language(default, "")


def state_schema_supported(payload: Mapping[str, Any]) -> bool:
    """Return ``True`` only for an explicit SynROV V1 state packet."""
    if not isinstance(payload, Mapping):
        return False
    schema = str(payload.get("schema", "") or "").strip()
    try:
        version = int(payload.get("version", -1))
    except (TypeError, ValueError):
        return False
    return schema == STATE_SCHEMA and version == SYNROV_VERSION


__all__ = [
    "SYNROV_VERSION",
    "CONTROL_SCHEMA",
    "STATE_SCHEMA",
    "DATASET_SCHEMA",
    "SNAPSHOT_SCHEMA",
    "COMMANDS_SCHEMA",
    "MODEL_SCHEMA",
    "SUPPORTED_LANGUAGES",
    "normalize_language",
    "control_envelope",
    "iter_state_blocks",
    "processing_language",
    "state_schema_supported",
]
