"""Voice parser and microphone/audio recognition engine."""

from ._legacy import RichVoiceParser, VoiceAudioEngine
try:
    from ._legacy import IntentAwareVoiceParser, ImprovedVoiceAudioEngine, ModeloIntencaoPorVozPoucasAmostras  # type: ignore
except Exception:  # pragma: no cover
    IntentAwareVoiceParser = None  # type: ignore
    ImprovedVoiceAudioEngine = None  # type: ignore
    ModeloIntencaoPorVozPoucasAmostras = None  # type: ignore

__all__ = [
    "RichVoiceParser",
    "VoiceAudioEngine",
    "IntentAwareVoiceParser",
    "ImprovedVoiceAudioEngine",
    "ModeloIntencaoPorVozPoucasAmostras",
]
