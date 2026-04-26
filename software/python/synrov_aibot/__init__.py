"""SynROV AiBot modular package.

The stable public modules are split by concern (bridge, learning, media, ui).
The compatibility core is kept in _legacy.py so the original incremental UI
inheritance order remains exactly preserved.
"""

from .runtime import (
    main,
    SynROVBridge,
    SynROVState,
    RichVoiceParser,
    VoiceAudioEngine,
    WebcamSource,
    VisionInfo,
    DatasetCollector,
    SampleRecord,
    SynROVSimplificadoPT,
    synrov_identification_text,
)

__all__ = [
    "main",
    "SynROVBridge",
    "SynROVState",
    "RichVoiceParser",
    "VoiceAudioEngine",
    "WebcamSource",
    "VisionInfo",
    "DatasetCollector",
    "SampleRecord",
    "SynROVSimplificadoPT",
    "synrov_identification_text",
]
