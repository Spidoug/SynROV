"""Training gates for online SynROV retraining."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List


@dataclass
class TrainingGateReport:
    allowed: bool
    reason: str
    rows_total: int
    rows_selected: int
    trusted_rows: int
    autonomous_rows: int
    metadata: Dict[str, Any] = field(default_factory=dict)


class TrainingGate:
    """Rejects retraining runs likely to collapse the model into autonomous noise."""

    def __init__(self, min_rows: int = 20, min_trusted_rows: int = 3, max_autonomous_share: float = 0.18) -> None:
        self.min_rows = int(min_rows)
        self.min_trusted_rows = int(min_trusted_rows)
        self.max_autonomous_share = float(max_autonomous_share)
        self.last_report = TrainingGateReport(False, "not_started", 0, 0, 0, 0)

    def evaluate(self, rows: Iterable[Dict[str, Any]]) -> TrainingGateReport:
        data = [r for r in rows or [] if isinstance(r, dict)]
        selected = [r for r in data if r.get("accepted_for_training", True)]
        trusted_tokens = ("manual", "voice", "text", "teacher", "ui", "base", "seed", "guided")
        autonomous_tokens = ("ai", "model", "autonomous", "vision", "music", "curiosity")
        trusted = 0
        autonomous = 0
        for row in selected:
            source = str(row.get("source", "") or "").lower()
            if any(t in source for t in trusted_tokens):
                trusted += 1
            if any(t in source for t in autonomous_tokens):
                autonomous += 1
        rows_selected = len(selected)
        if rows_selected < self.min_rows:
            report = TrainingGateReport(False, "not_enough_selected_rows", len(data), rows_selected, trusted, autonomous)
        elif trusted < self.min_trusted_rows:
            report = TrainingGateReport(False, "not_enough_trusted_rows", len(data), rows_selected, trusted, autonomous)
        elif rows_selected and autonomous / max(1, rows_selected) > self.max_autonomous_share:
            report = TrainingGateReport(False, "too_many_autonomous_rows", len(data), rows_selected, trusted, autonomous, {"autonomous_share": autonomous / max(1, rows_selected)})
        else:
            report = TrainingGateReport(True, "ok", len(data), rows_selected, trusted, autonomous, {"autonomous_share": autonomous / max(1, rows_selected) if rows_selected else 0.0})
        self.last_report = report
        return report
