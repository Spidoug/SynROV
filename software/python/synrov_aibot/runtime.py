"""Runtime exports for the SynROV interface."""
from __future__ import annotations

from typing import Any

from .main import main
from .modern_runtime import ModernSynROVBridge as SynROVBridge
from .modern_runtime import ModernSynROVState as SynROVState


def __getattr__(name: str) -> Any:
    """Load heavyweight GUI symbols only when callers explicitly request them."""
    if name == "SynROV":
        from .core import SynROV

        return SynROV
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def synrov_identification_text(state: SynROVState | None = None) -> str:
    if state is None:
        return "SynROV: disconnected"
    return f"SynROV: robot={state.robot} status={state.status} telemetry={state.telemetry_counter}"


__all__ = ["main", "SynROVBridge", "SynROVState", "SynROV", "synrov_identification_text"]
