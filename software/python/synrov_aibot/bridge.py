"""WebSocket bridge and command transmission layer."""

from ._legacy import SynROVBridge
try:
    from ._legacy import EnhancedSynROVBridge  # type: ignore
except Exception:  # pragma: no cover - optional in some builds
    EnhancedSynROVBridge = None  # type: ignore

__all__ = ["SynROVBridge", "EnhancedSynROVBridge"]
