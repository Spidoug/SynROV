"""Lightweight replay buffer for debugging autonomous SynROV decisions."""
from __future__ import annotations

import json
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, List, Optional


class ReplayBuffer:
    def __init__(self, maxlen: int = 600) -> None:
        self.buffer: Deque[Dict[str, Any]] = deque(maxlen=max(10, int(maxlen)))

    def record(self, snapshot: Any, decision: Any = None, result: Optional[bool] = None) -> None:
        try:
            snap = snapshot.to_record() if hasattr(snapshot, "to_record") else dict(snapshot or {})
        except Exception:
            snap = {"raw": str(snapshot)[:500]}
        if decision is not None:
            try:
                dec = decision.__dict__
            except Exception:
                dec = {"raw": str(decision)[:240]}
        else:
            dec = None
        self.buffer.append({"timestamp": time.time(), "snapshot": snap, "decision": dec, "result": result})

    def recent(self, n: int = 20) -> List[Dict[str, Any]]:
        return list(self.buffer)[-max(0, int(n)):]

    def save_jsonl(self, path: Path) -> int:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        rows = list(self.buffer)
        with p.open("w", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(row, ensure_ascii=False) + "\n")
        return len(rows)

    @staticmethod
    def load_jsonl(path: Path) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        p = Path(path)
        if not p.exists():
            return out
        with p.open("r", encoding="utf-8") as handle:
            for line in handle:
                try:
                    obj = json.loads(line)
                    if isinstance(obj, dict):
                        out.append(obj)
                except Exception:
                    continue
        return out
