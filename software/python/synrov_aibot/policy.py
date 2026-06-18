"""Policy helpers: normalise model/planner output to the active SynROV target dimension."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional

from .dataset import TARGET_DIMS, canonical_robot
from .perception import pad_float_vector
from .primitives import safe_float


@dataclass(frozen=True)
class PolicyOutput:
    robot: str
    source: str
    target: List[float]
    confidence: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)


class PolicyNormalizer:
    """Normalise any model/planner output to the active SynROV target dimension."""

    def __init__(self, target_dims: Optional[Dict[str, int]] = None) -> None:
        self.target_dims = dict(target_dims or TARGET_DIMS)

    def normalize(
        self,
        robot: Any,
        target: Iterable[Any],
        *,
        source: str = "model",
        confidence: float = 0.0,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> PolicyOutput:
        name = canonical_robot(robot)
        size = int(self.target_dims.get(name, 7))
        return PolicyOutput(
            robot=name,
            source=str(source or "model"),
            target=pad_float_vector(target, size, 0.0, -100_000.0, 100_000.0),
            confidence=safe_float(confidence, 0.0, 0.0, 1.0),
            metadata=dict(metadata or {}),
        )
