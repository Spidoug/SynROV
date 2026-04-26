"""Tkinter UI classes for the SynROV AiBot application."""

from ._legacy import (
    MultimodalAutoLearnStudio,
    CompactMultimodalAutoLearnStudio,
    CompactDragAutolearnStudio,
    CompactResponsiveAutoLearnStudio,
    SynROVSimplificadoPT,
)
try:
    from ._legacy import ScrollablePane, DraggableCard  # type: ignore
except Exception:  # pragma: no cover
    ScrollablePane = None  # type: ignore
    DraggableCard = None  # type: ignore
