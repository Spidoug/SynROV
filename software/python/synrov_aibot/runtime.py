"""Runtime compatibility layer for the SynROV interface."""
from __future__ import annotations

from .main import main
from .modern_runtime import ModernSynROVBridge as SynROVBridge, ModernSynROVState as SynROVState
from .core import SynROV


def synrov_identification_text(state: SynROVState | None = None) -> str:
    if state is None:
        return "SynROV: disconnected"
    return f"SynROV: robot={state.robot} status={state.status} telemetry={state.telemetry_counter}"


__all__ = ["main", "SynROVBridge", "SynROVState", "SynROV", "synrov_identification_text"]
