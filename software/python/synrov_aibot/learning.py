"""Dataset collection, training and inference helpers for SynROV AiBot."""
from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .dataset import SCHEMA_VERSION, TARGET_DIMS, SynROVDatasetValidator, canonical_robot, read_jsonl
from .primitives import safe_float, to_list
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


BufferedDatasetCollector = DatasetCollector


def load_dataset(path: Path) -> List[Dict[str, Any]]:
    return read_jsonl(Path(path))


def load_count(path: Path) -> int:
    """Return the number of JSONL rows in *path* without loading content into RAM."""
    p = Path(path)
    if not p.exists() or not p.is_file():
        return 0

    count = 0
    has_data = False
    last_byte = b""
    try:
        with p.open("rb") as fh:
            for chunk in iter(lambda: fh.read(1024 * 1024), b""):
                if not chunk:
                    continue
                has_data = True
                count += chunk.count(b"\n")
                last_byte = chunk[-1:]
    except OSError:
        return 0
    return count + (1 if has_data and last_byte != b"\n" else 0)


def vector_l2(values: Iterable[Any]) -> float:
    total = 0.0
    for value in to_list(values):
        v = safe_float(value, 0.0)
        total += v * v
    return math.sqrt(total)


def cosine_similarity(a: Iterable[Any], b: Iterable[Any]) -> float:
    av = [safe_float(v, 0.0) for v in to_list(a)]
    bv = [safe_float(v, 0.0) for v in to_list(b)]
    n = min(len(av), len(bv))
    if n <= 0:
        return 0.0

    dot = sum(av[i] * bv[i] for i in range(n))
    da = math.sqrt(sum(av[i] * av[i] for i in range(n)))
    db = math.sqrt(sum(bv[i] * bv[i] for i in range(n)))
    if da <= 1e-12 or db <= 1e-12:
        return 0.0
    return max(-1.0, min(1.0, dot / (da * db)))


def _feature(row: Dict[str, Any]) -> List[float]:
    out: List[float] = []
    for key in ("telemetry_feat", "synrov_feat", "webcam_feat", "audio_feat", "vision_feat"):
        out.extend(safe_float(v, 0.0) for v in to_list(row.get(key, [])))
    return out


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


def _active_robot_from_rows(rows: List[Dict[str, Any]]) -> str:
    for row in reversed(rows):
        if isinstance(row, dict):
            return canonical_robot(row.get("robot", "Manipulator"))
    return "Manipulator"


def train_model(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    selected, meta = prepare_training_rows(rows)
    if not selected:
        return {"ok": False, "reason": "no_training_rows", "metadata": meta}

    robot = _active_robot_from_rows(selected)
    selected = [row for row in selected if canonical_robot(row.get("robot", robot)) == robot]
    if not selected:
        meta["training_gate_reason"] = "no_rows_for_active_robot"
        return {"ok": False, "reason": "no_rows_for_active_robot", "metadata": meta}

    robot_gate = TrainingGate().evaluate(selected)
    meta.update({
        "training_robot_gate_allowed": robot_gate.allowed,
        "training_robot_gate_reason": robot_gate.reason,
        "training_robot_rows_selected": robot_gate.rows_selected,
        "training_robot_trusted_rows": robot_gate.trusted_rows,
        "training_robot_autonomous_rows": robot_gate.autonomous_rows,
    })
    if not robot_gate.allowed:
        return {"ok": False, "reason": robot_gate.reason, "metadata": meta}

    target_dim = TARGET_DIMS.get(robot, 7)
    meta["training_robot"] = robot
    meta["training_robot_rows"] = len(selected)
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
        from sklearn.ensemble import RandomForestRegressor

        model = RandomForestRegressor(
            n_estimators=64,
            random_state=42,
            min_samples_leaf=2,
            n_jobs=-1,
        )
        model.fit(X, y)
        package["model"] = model
        package["model_type"] = "RandomForestRegressor"
    except Exception as exc:

        avg = [sum(target[i] for target in y) / len(y) for i in range(target_dim)]
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
        out.extend(safe_float(v, 0.0) for v in to_list(part))
    return out


def clamp_infer_period_ms(value: Any, default: int = 500) -> int:
    return max(120, min(5000, int(round(safe_float(value, float(default))))))


def compute_feature_familiarity(
    feature: Iterable[Any],
    rows: Iterable[Dict[str, Any]],
    limit: int = 60,
) -> float:
    fv = to_list(feature)
    recent_rows: deque = deque(maxlen=max(1, int(limit)))
    try:
        iterator = iter(rows or [])
    except Exception:
        iterator = iter(())
    for row in iterator:
        if isinstance(row, dict):
            recent_rows.append(row)

    best = 0.0
    for row in recent_rows:
        best = max(best, cosine_similarity(fv, _feature(row)))
    return best


def compute_tree_agreement(predictions: Iterable[Iterable[Any]]) -> float:
    preds = [to_list(p) for p in to_list(predictions)]
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
    conf = 0.20 + 0.45 * max(0.0, safe_float(familiarity)) + 0.30 * max(0.0, safe_float(agreement))
    if not gate_ok:
        conf *= 0.45
    return max(0.0, min(1.0, conf))


def spontaneity_offsets(size: int, strength: float = 0.05) -> List[float]:
    now = time.time()
    amp = safe_float(strength, 0.05)
    return [math.sin(now * (0.7 + i * 0.17)) * amp for i in range(max(0, int(size)))]


def should_enable_spontaneity(
    confidence: float,
    novelty: float = 0.0,
    enabled: bool = False,
) -> bool:
    return bool(enabled and safe_float(confidence) >= 0.35 and safe_float(novelty) >= 0.10)


def blend_predictions(a: Iterable[Any], b: Iterable[Any], alpha: float = 0.5) -> List[float]:
    av = to_list(a)
    bv = to_list(b)
    n = max(len(av), len(bv))
    av = pad_float_vector(av, n)
    bv = pad_float_vector(bv, n)
    mix = safe_float(alpha, 0.5, 0.0, 1.0)
    return [(1.0 - mix) * x + mix * y for x, y in zip(av, bv)]


def observed_shadow_vector(values: Iterable[Any]) -> List[float]:
    return [safe_float(v, 0.0) for v in to_list(values)]


def evaluate_shadow_score(predicted: Iterable[Any], observed: Iterable[Any]) -> float:
    return max(0.0, min(1.0, (cosine_similarity(predicted, observed) + 1.0) / 2.0))
