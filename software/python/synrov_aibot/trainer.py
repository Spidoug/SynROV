"""Training gates for online SynROV retraining."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable


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

    TRUSTED_TOKENS = ("manual", "voice", "text", "teacher", "ui", "base", "seed", "guided")
    AUTONOMOUS_TOKENS = ("ai", "model", "autonomous", "vision", "music", "curiosity")

    def __init__(self, min_rows: int = 20, min_trusted_rows: int = 3, max_autonomous_share: float = 0.18) -> None:
        self.min_rows = int(min_rows)
        self.min_trusted_rows = int(min_trusted_rows)
        self.max_autonomous_share = float(max_autonomous_share)
        self.last_report = TrainingGateReport(False, "not_started", 0, 0, 0, 0)

    def evaluate(self, rows: Iterable[Dict[str, Any]]) -> TrainingGateReport:
        total = selected = trusted = autonomous = 0

        try:
            iterator = iter(rows or [])
        except Exception:
            iterator = iter(())

        for row in iterator:
            if not isinstance(row, dict):
                continue
            total += 1
            if not row.get("accepted_for_training", True):
                continue
            selected += 1
            source = str(row.get("source", "") or "").lower()
            if any(token in source for token in self.TRUSTED_TOKENS):
                trusted += 1
            if any(token in source for token in self.AUTONOMOUS_TOKENS):
                autonomous += 1

        autonomous_share = autonomous / max(1, selected)
        metadata = {"autonomous_share": autonomous_share}

        if selected < self.min_rows:
            report = TrainingGateReport(False, "not_enough_selected_rows", total, selected, trusted, autonomous)
        elif trusted < self.min_trusted_rows:
            report = TrainingGateReport(False, "not_enough_trusted_rows", total, selected, trusted, autonomous)
        elif autonomous_share > self.max_autonomous_share:
            report = TrainingGateReport(
                False,
                "too_many_autonomous_rows",
                total,
                selected,
                trusted,
                autonomous,
                metadata,
            )
        else:
            report = TrainingGateReport(True, "ok", total, selected, trusted, autonomous, metadata)

        self.last_report = report
        return report
