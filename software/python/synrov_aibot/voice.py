"""Voice parsing and optional Google SpeechRecognition backend."""
from __future__ import annotations

import threading
import time
from typing import Any, Dict

from .helpers import normalize_text
from .robot_ai import DRONE_PROFILE, MANIPULATOR_PROFILE, VEHICLE_PROFILE, match_alias


class RichVoiceParser:
    """Stateless intent parser backed by the three robot profiles."""

    def __init__(self) -> None:
        self.profiles = [MANIPULATOR_PROFILE, VEHICLE_PROFILE, DRONE_PROFILE]

    def parse(self, text: Any) -> Dict[str, Any]:
        best = (0, "Manipulator", "none", "")
        for profile in self.profiles:
            for name, aliases in profile.intents.items():
                score = match_alias(text, aliases)
                if score > best[0]:
                    best = (score, profile.robot, "intent", name)
            for name, aliases in profile.missions.items():
                score = match_alias(text, aliases)
                if score > best[0]:
                    best = (score, profile.robot, "mission", name)
        return {
            "text": str(text or ""),
            "norm": normalize_text(text),
            "robot": best[1],
            "kind": best[2],
            "intent": best[3] or "none",
            "conf": min(1.0, best[0] / 1000.0),
        }


class GoogleVoiceAudioEngine:
    """One-shot recogniser wrapper used by compatibility imports."""

    def __init__(self) -> None:
        self.running = False
        self.last_text = ""
        self.last_error = ""
        self.last_ts = 0.0
        self.parser = RichVoiceParser()

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def inject_text(self, text: str) -> None:
        self.last_text = str(text or "")
        self.last_ts = time.time()

    def listen_once_async(self, callback: Any, language: str = "pt-BR") -> None:
        def worker() -> None:
            try:
                import speech_recognition as sr  # type: ignore
                rec = sr.Recognizer()
                with sr.Microphone() as src:
                    rec.adjust_for_ambient_noise(src, duration=0.35)
                    audio = rec.listen(src, timeout=4, phrase_time_limit=5)
                text = rec.recognize_google(audio, language=language)
                self.inject_text(text)
                callback(text, None)
            except Exception as exc:
                self.last_error = str(exc)
                callback("", exc)
        threading.Thread(target=worker, daemon=True).start()

    def context(self) -> Dict[str, Any]:
        parsed = self.parser.parse(self.last_text)
        return {
            "raw": self.last_text,
            "text": self.last_text,
            "norm": parsed.get("norm", ""),
            "intent": parsed.get("intent", "none"),
            "conf": parsed.get("conf", 0.0),
            "text_age": time.time() - self.last_ts if self.last_ts else 999.0,
            "audio_features": [0.0] * 12,
            "audio_level": 0.0,
            "audio_age": 999.0,
            "error": self.last_error,
            "speech_backend": "google_optional",
        }


# Compatibility aliases.
VoiceAudioEngine = GoogleVoiceAudioEngine
ImprovedVoiceAudioEngine = GoogleVoiceAudioEngine

__all__ = [
    "RichVoiceParser",
    "GoogleVoiceAudioEngine",
    "VoiceAudioEngine",
    "ImprovedVoiceAudioEngine",
]
