"""Schema validation for continuous SynROV multimodal data collection."""
from __future__ import annotations

import json
import time
import unicodedata
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .perception import pad_float_vector, safe_text
from .primitives import safe_float

SCHEMA_VERSION = "synrov-training-v2"
TARGET_DIMS = {"Manipulator": 7, "Vehicle": 4, "Drone": 6}


def _norm_robot_token(value: Any) -> str:
    raw = str(value or "").strip().lower()
    return unicodedata.normalize("NFKD", raw).encode("ascii", "ignore").decode()


def canonical_robot(value: Any, default: str = "Manipulator") -> str:
    """Return the canonical robot name (Manipulator, Vehicle or Drone)."""
    text = _norm_robot_token(value or default)
    if text.startswith(("vehicle", "veh", "rover")) or text in {"car", "ground", "ground_vehicle", "veiculo"}:
        return "Vehicle"
    if text.startswith(("drone", "dro")) or text in {"quad", "uav", "quadcopter"}:
        return "Drone"
    if text.startswith(("manip", "arm", "braco")) or "manipulator" in text:
        return "Manipulator"
    if text != _norm_robot_token(default):
        return canonical_robot(default)
    return "Manipulator"


@dataclass
class ValidationMeta:
    valid: bool
    reason: str = "ok"
    quality_score: float = 0.0
    accepted_for_training: bool = False
    signal: Dict[str, float] = field(default_factory=dict)


class SynROVDatasetValidator:
    """Validate and canonicalise rows before RAM buffering, disk writes or retraining."""

    def __init__(
        self,
        telemetry_dim: int,
        image_dim: int,
        audio_dim: int,
        vision_dim: int,
        target_dims: Optional[Dict[str, int]] = None,
    ) -> None:
        self.telemetry_dim = int(telemetry_dim)
        self.image_dim = int(image_dim)
        self.audio_dim = int(audio_dim)
        self.vision_dim = int(vision_dim)
        self.target_dims = dict(target_dims or TARGET_DIMS)
        self.rows_seen = 0
        self.rows_rejected = 0
        self.last_meta = ValidationMeta(False, "not_started")

    def target_dim(self, robot: str) -> int:
        return int(self.target_dims.get(canonical_robot(robot), 7))

    @staticmethod
    def _abs_stats(values: Any) -> Tuple[float, float]:
        try:
            iterator = iter(values or [])
        except Exception:
            iterator = iter(())
        total = peak = 0.0
        count = 0
        for item in iterator:
            val = abs(safe_float(item))
            total += val
            peak = max(peak, val)
            count += 1
        return (total / count if count else 0.0), peak

    def _signal(self, row: Dict[str, Any]) -> Dict[str, float]:
        def _peak(key: str) -> float:
            return self._abs_stats(row.get(key, []))[1]

        def _mean(key: str) -> float:
            return self._abs_stats(row.get(key, []))[0]

        return {
            "target_mean_abs": _mean("target"),
            "audio_peak_abs": _peak("audio_feat"),
            "vision_peak_abs": _peak("vision_feat"),
            "telemetry_peak_abs": _peak("telemetry_feat"),
        }

    def quality_score(self, row: Dict[str, Any]) -> float:
        signal = self._signal(row)
        source = str(row.get("source", "observe") or "observe").lower()
        voice_conf = safe_float(row.get("voice_confidence", 0.0), 0.0, 0.0, 1.0)

        score = 0.25
        if any(t in source for t in ("manual", "voice", "text", "teacher", "ui", "base")):
            score += 0.30
        if any(t in source for t in ("autonomous", "ai", "model")):
            score -= 0.08
        score += min(0.25, signal["target_mean_abs"] * 0.35)
        score += min(0.10, signal["audio_peak_abs"] * 0.50)
        score += min(0.10, signal["vision_peak_abs"] * 0.12)
        score += min(0.12, voice_conf * 0.12)
        return max(0.0, min(1.0, score))

    def sanitize_row(
        self,
        row: Any,
        *,
        robot_hint: Optional[str] = None,
        strict: bool = False,
    ) -> Optional[Dict[str, Any]]:
        self.rows_seen += 1
        if not isinstance(row, dict):
            self.rows_rejected += 1
            self.last_meta = ValidationMeta(False, "not_a_dict")
            return None

        robot = canonical_robot(row.get("robot", robot_hint or "Manipulator"))
        target_dim = self.target_dim(robot)
        clean: Dict[str, Any] = dict(row)

        clean["schema_version"] = SCHEMA_VERSION
        clean["timestamp"] = safe_float(row.get("timestamp", time.time()), time.time(), 0.0)
        clean["robot"] = robot
        clean["source"] = safe_text(row.get("source", "observe"), 80) or "observe"
        clean["simulation"] = row.get("simulation") if isinstance(row.get("simulation"), (bool, type(None))) else None
        clean["leap_enabled"] = bool(row.get("leap_enabled", False))
        clean["voice_text"] = safe_text(row.get("voice_text", ""), 240)
        clean["voice_norm"] = safe_text(row.get("voice_norm", clean["voice_text"]), 240)
        clean["voice_intent"] = safe_text(row.get("voice_intent", "none"), 80) or "none"
        clean["voice_confidence"] = safe_float(row.get("voice_confidence", 0.0), 0.0, 0.0, 1.0)
        clean["telemetry_feat"] = pad_float_vector(
            row.get("telemetry_feat", []),
            self.telemetry_dim,
            0.0,
            -1_000_000.0,
            1_000_000.0,
        )
        clean["synrov_feat"] = pad_float_vector(row.get("synrov_feat", []), self.image_dim, 0.0, -10.0, 10.0)
        clean["webcam_feat"] = pad_float_vector(row.get("webcam_feat", []), self.image_dim, 0.0, -10.0, 10.0)
        clean["audio_feat"] = pad_float_vector(row.get("audio_feat", []), self.audio_dim, 0.0, -10.0, 10.0)
        clean["vision_feat"] = pad_float_vector(row.get("vision_feat", []), self.vision_dim, 0.0, -10.0, 10.0)
        clean["target"] = pad_float_vector(row.get("target", []), target_dim, 0.0, -100_000.0, 100_000.0)
        clean["frame_path"] = safe_text(row.get("frame_path", ""), 500)

        signal = self._signal(clean)
        quality = self.quality_score(clean)
        has_voice = bool(
            clean["voice_norm"]
            and clean["voice_intent"] != "none"
            and clean["voice_confidence"] >= 0.15
        )
        informative = (
            signal["target_mean_abs"] > 0.004
            or signal["audio_peak_abs"] > 0.015
            or signal["vision_peak_abs"] > 0.12
            or has_voice
        )
        min_quality = 0.28 if strict else 0.18
        accepted = bool(informative and quality >= min_quality)

        if strict and not accepted:
            self.rows_rejected += 1
            self.last_meta = ValidationMeta(False, "low_signal_or_quality", quality, False, signal)
            return None

        clean["quality_score"] = quality
        clean["accepted_for_training"] = accepted
        clean["input_validated"] = True
        clean["input_validation_version"] = SCHEMA_VERSION
        clean["input_signal"] = signal
        self.last_meta = ValidationMeta(True, "ok", quality, accepted, signal)
        return clean

    def sanitize_rows(
        self,
        rows: Iterable[Dict[str, Any]],
        *,
        strict: bool = False,
    ) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
        accepted: List[Dict[str, Any]] = []
        total = invalid = low_quality = 0

        try:
            iterator = iter(rows or [])
        except Exception:
            iterator = iter(())

        for row in iterator:
            total += 1
            hint = row.get("robot") if isinstance(row, dict) else None
            clean = self.sanitize_row(row, robot_hint=hint, strict=strict)
            if clean is None:
                invalid += 1
                continue
            if not clean.get("accepted_for_training", False):
                low_quality += 1
            accepted.append(clean)

        return accepted, {
            "schema_version": SCHEMA_VERSION,
            "rows_total": total,
            "rows_valid": len(accepted),
            "rows_invalid": invalid,
            "rows_low_quality": low_quality,
        }


def read_jsonl(path: Path) -> List[Dict[str, Any]]:
    """Read a JSONL file, skipping blank, missing or malformed lines."""
    out: List[Dict[str, Any]] = []
    try:
        p = Path(path)
    except Exception:
        return out
    if not p.exists() or not p.is_file():
        return out
    try:
        with p.open("r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except Exception:
                    continue
                if isinstance(obj, dict):
                    out.append(obj)
    except OSError:
        return out
    return out
