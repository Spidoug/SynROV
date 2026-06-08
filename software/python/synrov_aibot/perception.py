"""Canonical perception snapshot layer for SynROV AiBot.

Receives live robot state plus voice/audio/video features and assembles them
into one immutable snapshot so the rest of the AI reasons over a coherent
moment instead of data captured at slightly different times.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


# ---------------------------------------------------------------------------
# Numeric helpers
# ---------------------------------------------------------------------------

def finite_float(
    value: Any,
    default: float = 0.0,
    lo: Optional[float] = None,
    hi: Optional[float] = None,
) -> float:
    """Return *value* as a finite float, falling back to *default* on errors."""
    try:
        out = float(value)
        if not math.isfinite(out):
            out = float(default)
    except Exception:
        out = float(default)
    if lo is not None:
        out = max(float(lo), out)
    if hi is not None:
        out = min(float(hi), out)
    return out


def pad_float_vector(
    values: Any,
    size: int,
    default: float = 0.0,
    lo: Optional[float] = None,
    hi: Optional[float] = None,
) -> List[float]:
    """Convert *values* to a float list of exactly *size* elements."""
    try:
        raw = list(values or [])
    except Exception:
        raw = []
    out = [finite_float(v, default, lo, hi) for v in raw[: max(0, int(size))]]
    padding = int(size) - len(out)
    if padding > 0:
        out.extend([float(default)] * padding)
    return out


def safe_text(value: Any, limit: int = 240) -> str:
    """Return *value* as a UTF-8 string truncated to *limit* characters."""
    try:
        return str(value or "")[:limit]
    except Exception:
        return ""


# ---------------------------------------------------------------------------
# Snapshot dataclasses
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class VoiceSnapshot:
    text: str = ""
    norm: str = ""
    intent: str = "none"
    confidence: float = 0.0
    text_age_s: float = 999.0
    audio_age_s: float = 999.0
    audio_level: float = 0.0
    audio_features: List[float] = field(default_factory=list)

    @property
    def is_fresh(self) -> bool:
        return bool(
            self.intent
            and self.intent != "none"
            and self.text_age_s <= 1.6
            and self.confidence >= 0.15
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "text": self.text,
            "norm": self.norm,
            "intent": self.intent,
            "confidence": self.confidence,
            "text_age_s": self.text_age_s,
            "audio_age_s": self.audio_age_s,
            "audio_level": self.audio_level,
            "audio_features": list(self.audio_features),
            "fresh": self.is_fresh,
        }


@dataclass(frozen=True)
class VisionSnapshot:
    confidence: float = 0.0
    dx: float = 0.0
    dy: float = 0.0
    area: float = 0.0
    width: float = 0.0
    height: float = 0.0
    centered: float = 0.0
    close: float = 0.0
    vector: List[float] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "confidence": self.confidence,
            "dx": self.dx,
            "dy": self.dy,
            "area": self.area,
            "width": self.width,
            "height": self.height,
            "centered": self.centered,
            "close": self.close,
            "vector": list(self.vector),
        }


@dataclass(frozen=True)
class TelemetrySnapshot:
    values: List[float] = field(default_factory=list)
    age_s: float = 999.0
    connected: bool = False
    fresh: bool = False
    robot_reported: str = ""
    simulation: Optional[bool] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "values": list(self.values),
            "age_s": self.age_s,
            "connected": self.connected,
            "fresh": self.fresh,
            "robot_reported": self.robot_reported,
            "simulation": self.simulation,
        }


@dataclass(frozen=True)
class SynROVSnapshot:
    timestamp: float
    robot: str
    source: str
    telemetry: TelemetrySnapshot
    synrov_image_features: List[float]
    webcam_features: List[float]
    voice: VoiceSnapshot
    vision: VisionSnapshot
    target_hint: List[float] = field(default_factory=list)
    mode: str = "unknown"
    metadata: Dict[str, Any] = field(default_factory=dict)

    def feature_vector(self) -> List[float]:
        return (
            list(self.telemetry.values)
            + list(self.synrov_image_features)
            + list(self.webcam_features)
            + list(self.voice.audio_features)
            + list(self.vision.vector)
        )

    def to_record(self) -> Dict[str, Any]:
        return {
            "schema_version": "synrov-snapshot-v2",
            "timestamp": self.timestamp,
            "robot": self.robot,
            "source": self.source,
            "mode": self.mode,
            "telemetry": self.telemetry.to_dict(),
            "synrov_image_features": list(self.synrov_image_features),
            "webcam_features": list(self.webcam_features),
            "voice": self.voice.to_dict(),
            "vision": self.vision.to_dict(),
            "target_hint": list(self.target_hint),
            "metadata": dict(self.metadata),
        }


# ---------------------------------------------------------------------------
# Snapshot builder
# ---------------------------------------------------------------------------

class SynROVSnapshotBuilder:
    """Build one coherent multimodal snapshot per control cycle."""

    def __init__(
        self,
        telemetry_dim: int,
        image_dim: int,
        audio_dim: int,
        vision_dim: int,
        voice_hold_s: float = 1.6,
        audio_hold_s: float = 2.5,
    ) -> None:
        self.telemetry_dim = int(telemetry_dim)
        self.image_dim = int(image_dim)
        self.audio_dim = int(audio_dim)
        self.vision_dim = int(vision_dim)
        self.voice_hold_s = float(voice_hold_s)
        self.audio_hold_s = float(audio_hold_s)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _voice(self, voice_ctx: Optional[Dict[str, Any]]) -> VoiceSnapshot:
        ctx = dict(voice_ctx or {})
        text_age = finite_float(ctx.get("text_age", 999.0), 999.0, 0.0)
        audio_age = finite_float(ctx.get("audio_age", 999.0), 999.0, 0.0)
        audio_level = finite_float(ctx.get("audio_level", 0.0), 0.0, 0.0, 10.0)
        audio_features = pad_float_vector(
            ctx.get("audio_features", []) if audio_age <= self.audio_hold_s else [],
            self.audio_dim,
            0.0,
            -10.0,
            10.0,
        )

        if text_age > self.voice_hold_s:
            return VoiceSnapshot(
                text_age_s=text_age,
                audio_age_s=audio_age,
                audio_level=audio_level if audio_age <= self.audio_hold_s else 0.0,
                audio_features=audio_features,
            )

        return VoiceSnapshot(
            text=safe_text(ctx.get("text", ctx.get("raw", ""))),
            norm=safe_text(ctx.get("norm", ctx.get("text", ""))),
            intent=safe_text(ctx.get("intent", "none"), 80) or "none",
            confidence=finite_float(ctx.get("conf", ctx.get("confidence", 0.0)), 0.0, 0.0, 1.0),
            text_age_s=text_age,
            audio_age_s=audio_age,
            audio_level=audio_level,
            audio_features=audio_features,
        )

    def _vision(
        self,
        vision_obj: Any,
        explicit_vector: Optional[List[float]] = None,
    ) -> VisionSnapshot:
        if explicit_vector is None:
            try:
                explicit_vector = list(vision_obj.as_vector())
            except Exception:
                explicit_vector = []
        vector = pad_float_vector(explicit_vector, self.vision_dim, 0.0, -10.0, 10.0)

        def _attr(name: str, idx: int, lo: float, hi: float) -> float:
            raw = getattr(vision_obj, name, vector[idx] if idx < len(vector) else 0.0)
            return finite_float(raw, 0.0, lo, hi)

        return VisionSnapshot(
            confidence=_attr("confidence", 0, 0.0, 1.0),
            dx=_attr("dx", 1, -1.0, 1.0),
            dy=_attr("dy", 2, -1.0, 1.0),
            area=_attr("area", 3, 0.0, 1.0),
            width=_attr("width", 4, 0.0, 1.0),
            height=_attr("height", 5, 0.0, 1.0),
            centered=_attr("centered", 6, 0.0, 1.0),
            close=_attr("close", 7, 0.0, 1.0),
            vector=vector,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def build(
        self,
        *,
        robot: str,
        source: str,
        state: Any = None,
        telemetry_feat: Any = None,
        synrov_feat: Any = None,
        webcam_feat: Any = None,
        audio_feat: Any = None,
        vision: Any = None,
        voice_ctx: Optional[Dict[str, Any]] = None,
        target_hint: Any = None,
        telemetry_fresh: Optional[bool] = None,
        now: Optional[float] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> SynROVSnapshot:
        ts = float(now if now is not None else time.time())

        telemetry_values = pad_float_vector(
            telemetry_feat, self.telemetry_dim, 0.0, -1_000_000.0, 1_000_000.0
        )
        last_telem_ts = finite_float(getattr(state, "last_telemetry_ts", 0.0), 0.0, 0.0)
        age = ts - last_telem_ts if last_telem_ts else 999.0
        connected = bool(getattr(state, "connected", False))
        fresh = bool(
            telemetry_fresh
            if telemetry_fresh is not None
            else (not connected or age <= 3.0)
        )

        voice = self._voice(voice_ctx)
        if audio_feat is not None:
            # Override audio features while preserving the rest of the voice snapshot.
            voice = VoiceSnapshot(
                text=voice.text,
                norm=voice.norm,
                intent=voice.intent,
                confidence=voice.confidence,
                text_age_s=voice.text_age_s,
                audio_age_s=voice.audio_age_s,
                audio_level=voice.audio_level,
                audio_features=pad_float_vector(audio_feat, self.audio_dim, 0.0, -10.0, 10.0),
            )

        hint_list = list(target_hint or [])
        return SynROVSnapshot(
            timestamp=ts,
            robot=safe_text(robot, 40) or "Manipulator",
            source=safe_text(source, 80) or "observe",
            telemetry=TelemetrySnapshot(
                values=telemetry_values,
                age_s=age,
                connected=connected,
                fresh=fresh,
                robot_reported=safe_text(getattr(state, "robot", ""), 40),
                simulation=(
                    getattr(state, "simulation", None)
                    if isinstance(getattr(state, "simulation", None), (bool, type(None)))
                    else None
                ),
            ),
            synrov_image_features=pad_float_vector(synrov_feat, self.image_dim, 0.0, -10.0, 10.0),
            webcam_features=pad_float_vector(webcam_feat, self.image_dim, 0.0, -10.0, 10.0),
            voice=voice,
            vision=self._vision(vision),
            target_hint=pad_float_vector(hint_list, len(hint_list), 0.0, -100_000.0, 100_000.0),
            mode=safe_text(getattr(state, "mode", "unknown"), 40) or "unknown",
            metadata=dict(metadata or {}),
        )
