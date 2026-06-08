"""Bridge exports for the SynROV dedicated AI runtime."""
from __future__ import annotations

from .modern_runtime import ModernSynROVBridge, ModernSynROVState
from .safety import SynROVBridgeSafetyAdapter, SynROVSafetyLayer

# Public name used throughout the codebase.
SynROVBridge = ModernSynROVBridge

__all__ = [
    "SynROVBridge",
    "ModernSynROVBridge",
    "ModernSynROVState",
    "SynROVSafetyLayer",
    "SynROVBridgeSafetyAdapter",
]
