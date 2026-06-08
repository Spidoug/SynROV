"""Dataset collection, training and inference helpers for SynROV AiBot."""
from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .dataset import SCHEMA_VERSION, TARGET_DIMS, SynROVDatasetValidator, canonical_robot, read_jsonl
from .perception import pad_float_vector
from .policy import PolicyNormalizer, PolicyOutput
from .trainer import TrainingGate, TrainingGateReport


@dataclass
class SampleRecord:
    timestamp: float = field(default_factory=time.time)
    robot: str = "Manipulator"
    source: str = "manual"
    telemetry_feat: List[float] = field(default_factory=list)
    synrov_feat: List[float] = field(default_factory=list)
    webcam_feat: List[float] = field(default_factory=list)
    audio_feat: List[float] = field(default_factory=list)
    vision_feat: List[float] = field(default_factory=list)
    target: List[float] = field(default_factory=list)
    voice_text: str = ""
    voice_intent: str = "none"
    voice_confidence: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class DatasetCollector:
    """Append-only JSONL collector with schema validation."""

    def __init__(
        self,
        path: Path,
        *,
        telemetry_dim: int = 16,
        image_dim: int = 16,
        audio_dim: int = 12,
        vision_dim: int = 8,
    ) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.validator = SynROVDatasetValidator(telemetry_dim, image_dim, audio_dim, vision_dim)

    def append(self, row: Dict[str, Any], *, strict: bool = False) -> Tuple[bool, Optional[Dict[str, Any]]]:
        clean = self.validator.sanitize_row(
            row,
            robot_hint=row.get("robot") if isinstance(row, dict) else None,
            strict=strict,
        )
        if clean is None:
            return False, None
        with self.path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(clean, ensure_ascii=False) + "\n")
        return True, clean

    def collect(self, record: SampleRecord, *, strict: bool = False) -> Tuple[bool, Optional[SampleRecord]]:
        ok, clean = self.append(record.to_dict(), strict=strict)
        if not ok or clean is None:
            return False, None
        return True, SampleRecord(
            timestamp=float(clean.get("timestamp", time.time())),
            robot=str(clean.get("robot", "Manipulator")),
            source=str(clean.get("source", "manual")),
            telemetry_feat=list(clean.get("telemetry_feat", [])),
            synrov_feat=list(clean.get("synrov_feat", [])),
            webcam_feat=list(clean.get("webcam_feat", [])),
            audio_feat=list(clean.get("audio_feat", [])),
            vision_feat=list(clean.get("vision_feat", [])),
            target=list(clean.get("target", [])),
            voice_text=str(clean.get("voice_text", "")),
            voice_intent=str(clean.get("voice_intent", "none")),
            voice_confidence=float(clean.get("voice_confidence", 0.0)),
        )

    def rows(self) -> List[Dict[str, Any]]:
        return load_dataset(self.path)

    def count(self) -> int:
        return load_count(self.path)


# Keep the old name for existing call sites.
BufferedDatasetCollector = DatasetCollector


def load_dataset(path: Path) -> List[Dict[str, Any]]:
    return read_jsonl(Path(path))


def load_count(path: Path) -> int:
    """Return the number of JSONL lines in *path* without loading content into RAM."""
    p = Path(path)
    if not p.exists():
        return 0
    try:
        with p.open("r", encoding="utf-8") as fh:
            return sum(1 for _ in fh)
    except Exception:
        return 0


def vector_l2(values: Iterable[Any]) -> float:
    vals = [float(v or 0.0) for v in values or []]
    return math.sqrt(sum(v * v for v in vals))


def cosine_similarity(a: Iterable[Any], b: Iterable[Any]) -> float:
    av = [float(v or 0.0) for v in a or []]
    bv = [float(v or 0.0) for v in b or []]
    n = min(len(av), len(bv))
    if n <= 0:
        return 0.0
    av, bv = av[:n], bv[:n]
    da = vector_l2(av)
    db = vector_l2(bv)
    if da <= 1e-12 or db <= 1e-12:
        return 0.0
    return max(-1.0, min(1.0, sum(x * y for x, y in zip(av, bv)) / (da * db)))


def _feature(row: Dict[str, Any]) -> List[float]:
    return (
        list(row.get("telemetry_feat", []))
        + list(row.get("synrov_feat", []))
        + list(row.get("webcam_feat", []))
        + list(row.get("audio_feat", []))
        + list(row.get("vision_feat", []))
    )


def prepare_training_rows(
    rows: List[Dict[str, Any]],
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    validator = SynROVDatasetValidator(16, 16, 12, 8)
    sanitized, meta = validator.sanitize_rows(rows, strict=False)
    selected = [row for row in sanitized if row.get("accepted_for_training", True)]
    gate = TrainingGate()
    report = gate.evaluate(selected)
    meta.update({
        "training_gate_allowed": report.allowed,
        "training_gate_reason": report.reason,
        "rows_selected": report.rows_selected,
        "trusted_rows": report.trusted_rows,
        "autonomous_rows": report.autonomous_rows,
    })
    return selected, meta


def train_model(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    selected, meta = prepare_training_rows(rows)
    if not selected:
        return {"ok": False, "reason": "no_training_rows", "metadata": meta}

    robot = canonical_robot(selected[-1].get("robot", "Manipulator"))
    target_dim = TARGET_DIMS.get(robot, 7)
    X = [_feature(row) for row in selected]
    y = [
        pad_float_vector(row.get("target", []), target_dim, 0.0, -100_000.0, 100_000.0)
        for row in selected
    ]
    package: Dict[str, Any] = {
        "ok": True,
        "robot": robot,
        "rows": len(selected),
        "metadata": meta,
        "target_dim": target_dim,
    }
    try:
        from sklearn.ensemble import RandomForestRegressor  # type: ignore
        model = RandomForestRegressor(n_estimators=80, random_state=42, min_samples_leaf=2)
        model.fit(X, y)
        package["model"] = model
        package["model_type"] = "RandomForestRegressor"
    except Exception as exc:
        # Safe fallback: average target — runtime stays predictable without sklearn.
        avg = [sum(t[i] for t in y) / len(y) for i in range(target_dim)]
        package.update({
            "model": None,
            "model_type": "mean_target",
            "mean_target": avg,
            "warning": str(exc),
        })
    return package


def build_inference_feature(*parts: Iterable[Any]) -> List[float]:
    out: List[float] = []
    for part in parts:
        out.extend(float(v or 0.0) for v in part or [])
    return out


def clamp_infer_period_ms(value: Any, default: int = 500) -> int:
    try:
        val = int(float(value))
    except Exception:
        val = int(default)
    return max(120, min(5000, val))


def compute_feature_familiarity(
    feature: Iterable[Any],
    rows: Iterable[Dict[str, Any]],
    limit: int = 60,
) -> float:
    fv = list(feature or [])
    sims = [cosine_similarity(fv, _feature(row)) for row in list(rows or [])[-limit:]]
    return max(sims) if sims else 0.0


def compute_tree_agreement(predictions: Iterable[Iterable[Any]]) -> float:
    preds = [list(p or []) for p in predictions or []]
    if len(preds) < 2:
        return 1.0 if preds else 0.0
    base = preds[0]
    sims = [cosine_similarity(base, p) for p in preds[1:]]
    return sum(sims) / max(1, len(sims))


def estimate_prediction_confidence(
    *,
    familiarity: float = 0.0,
    agreement: float = 0.0,
    gate_ok: bool = True,
) -> float:
    conf = 0.20 + 0.45 * max(0.0, familiarity) + 0.30 * max(0.0, agreement)
    if not gate_ok:
        conf *= 0.45
    return max(0.0, min(1.0, conf))


def spontaneity_offsets(size: int, strength: float = 0.05) -> List[float]:
    return [
        math.sin(time.time() * (0.7 + i * 0.17)) * float(strength)
        for i in range(max(0, int(size)))
    ]


def should_enable_spontaneity(
    confidence: float,
    novelty: float = 0.0,
    enabled: bool = False,
) -> bool:
    return bool(enabled and confidence >= 0.35 and novelty >= 0.10)


def blend_predictions(a: Iterable[Any], b: Iterable[Any], alpha: float = 0.5) -> List[float]:
    av = list(a or [])
    bv = list(b or [])
    n = max(len(av), len(bv))
    av = pad_float_vector(av, n)
    bv = pad_float_vector(bv, n)
    alpha = max(0.0, min(1.0, float(alpha)))
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(av, bv)]


def observed_shadow_vector(values: Iterable[Any]) -> List[float]:
    return [float(v or 0.0) for v in values or []]


def evaluate_shadow_score(predicted: Iterable[Any], observed: Iterable[Any]) -> float:
    return max(0.0, min(1.0, (cosine_similarity(predicted, observed) + 1.0) / 2.0))
