"""Canonical microphone and speech input for SynROV AiBot Version 1.

The current path uses ``sounddevice`` directly for PCM capture, avoiding the
implicit PyAudio dependency of ``speech_recognition.Microphone``.  Speech is
segmented locally with a small energy-based VAD and then sent to the selected
SpeechRecognition recognizer.  The active recognition language can be changed
while the microphone is running; the next phrase uses the new language.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional

try:
    import sounddevice as sd
except Exception:  # optional until application startup validation
    sd = None

try:
    import speech_recognition as sr
except Exception:  # optional until application startup validation
    sr = None


SAMPLE_RATE = 16_000
SAMPLE_WIDTH = 2
BLOCK_FRAMES = 1_600  # 100 ms at 16 kHz
MIN_PHRASE_S = 0.24
END_SILENCE_S = 0.70
MAX_PHRASE_S = 7.0
CALIBRATION_S = 0.70
MIN_ENERGY_THRESHOLD = 220.0
ENERGY_MULTIPLIER = 2.15
DEFAULT_RECOGNITION_LOCALE = "en-US"


def recognition_language(value: str) -> str:
    """Normalize the locale supplied by the active language package.

    No language-specific branch lives here: every installed package publishes
    its own recognition locale (for example ``pt-BR`` or ``ja-JP``).
    """
    text = str(value or "").strip().replace("_", "-")
    if not text:
        return DEFAULT_RECOGNITION_LOCALE
    parts = [part for part in text.split("-") if part]
    if not parts:
        return DEFAULT_RECOGNITION_LOCALE
    language = parts[0].lower()
    if len(parts) >= 2:
        region = parts[1].upper() if len(parts[1]) in (2, 3) else parts[1]
        return "-".join([language, region, *parts[2:]])
    return language


def pcm16_rms(data: bytes) -> float:
    """Compute little-endian signed 16-bit PCM RMS without NumPy."""
    if not data or len(data) < 2:
        return 0.0
    usable = len(data) - (len(data) % 2)
    view = memoryview(data[:usable]).cast("h")
    if not view:
        return 0.0
    total = 0.0
    for sample in view:
        value = float(sample)
        total += value * value
    return math.sqrt(total / len(view))


class VoiceInput:
    """Continuous microphone input with live language switching."""

    def __init__(self, on_text: Callable[[str], None], on_status: Optional[Callable[[str], None]] = None) -> None:
        self.on_text = on_text
        self.on_status = on_status or (lambda _text: None)
        self.running = False
        self._language = DEFAULT_RECOGNITION_LOCALE
        self._language_lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.device_name = ""
        self.last_energy = 0.0
        self.energy_threshold = MIN_ENERGY_THRESHOLD

    @property
    def available(self) -> bool:
        return sr is not None and sd is not None

    @property
    def language(self) -> str:
        with self._language_lock:
            return self._language

    def set_language(self, language: str) -> str:
        normalized = recognition_language(language)
        with self._language_lock:
            self._language = normalized
        if self.running:
            self.on_status(f"language:{normalized}")
        return normalized

    def start(self, language: str = DEFAULT_RECOGNITION_LOCALE) -> None:
        self.set_language(language)
        if self.running:
            return
        if sr is None:
            raise RuntimeError("SpeechRecognition is not available")
        if sd is None:
            raise RuntimeError("sounddevice is not available")
        self._stop.clear()
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="synrov-voice")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.running = False

    def _read_block(self, stream) -> bytes:
        data, overflowed = stream.read(BLOCK_FRAMES)
        if overflowed:
            self.on_status("overflow")
        return bytes(data)

    def _calibrate(self, stream) -> float:
        samples = []
        blocks = max(2, int(CALIBRATION_S * SAMPLE_RATE / BLOCK_FRAMES))
        self.on_status("calibrating")
        for _ in range(blocks):
            if self._stop.is_set():
                break
            level = pcm16_rms(self._read_block(stream))
            if level > 0:
                samples.append(level)
        if not samples:
            return MIN_ENERGY_THRESHOLD
        samples.sort()
        # Median is much less sensitive to a transient sound during startup.
        median = samples[len(samples) // 2]
        return max(MIN_ENERGY_THRESHOLD, median * ENERGY_MULTIPLIER)

    def _capture_phrase(self, stream) -> Optional[bytes]:
        block_s = BLOCK_FRAMES / SAMPLE_RATE
        pre_roll = b""
        while not self._stop.is_set():
            block = self._read_block(stream)
            energy = pcm16_rms(block)
            self.last_energy = energy
            # Slowly follow a quieter room, but never chase active speech.
            if energy < self.energy_threshold * 0.70:
                floor = max(MIN_ENERGY_THRESHOLD, energy * ENERGY_MULTIPLIER)
                self.energy_threshold = self.energy_threshold * 0.985 + floor * 0.015
            pre_roll = block
            if energy >= self.energy_threshold:
                break

        if self._stop.is_set():
            return None

        chunks = [pre_roll]
        speech_s = block_s
        silence_s = 0.0
        total_s = block_s
        while not self._stop.is_set() and total_s < MAX_PHRASE_S:
            block = self._read_block(stream)
            chunks.append(block)
            energy = pcm16_rms(block)
            self.last_energy = energy
            total_s += block_s
            if energy >= self.energy_threshold * 0.72:
                speech_s += block_s
                silence_s = 0.0
            else:
                silence_s += block_s
                if silence_s >= END_SILENCE_S and speech_s >= MIN_PHRASE_S:
                    break
        if speech_s < MIN_PHRASE_S:
            return None
        return b"".join(chunks)

    def _loop(self) -> None:
        recognizer = sr.Recognizer()
        try:
            info = sd.query_devices(kind="input")
            self.device_name = str(info.get("name", "default microphone")) if isinstance(info, dict) else str(info)
            stream = sd.RawInputStream(
                samplerate=SAMPLE_RATE,
                blocksize=BLOCK_FRAMES,
                channels=1,
                dtype="int16",
                latency="low",
            )
        except Exception as exc:
            self.running = False
            self.on_status(f"microphone_error:{exc}")
            return

        try:
            with stream:
                self.energy_threshold = self._calibrate(stream)
                self.on_status(f"listening:{self.device_name}")
                while not self._stop.is_set():
                    try:
                        pcm = self._capture_phrase(stream)
                    except Exception as exc:
                        self.on_status(f"microphone_error:{exc}")
                        time.sleep(0.15)
                        continue
                    if not pcm:
                        continue
                    self.on_status("recognizing")
                    audio = sr.AudioData(pcm, SAMPLE_RATE, SAMPLE_WIDTH)
                    try:
                        text = recognizer.recognize_google(audio, language=self.language)
                    except sr.UnknownValueError:
                        self.on_status("unrecognized")
                        self.on_status(f"listening:{self.device_name}")
                        continue
                    except sr.RequestError as exc:
                        self.on_status(f"recognition_error:{exc}")
                        time.sleep(0.35)
                        self.on_status(f"listening:{self.device_name}")
                        continue
                    except Exception as exc:
                        self.on_status(f"recognition_error:{exc}")
                        self.on_status(f"listening:{self.device_name}")
                        continue
                    text = str(text or "").strip()
                    if text:
                        self.on_status(f"heard:{text}")
                        self.on_text(text)
                    self.on_status(f"listening:{self.device_name}")
        finally:
            self.running = False
            self.on_status("stopped")


__all__ = [
    "VoiceInput", "recognition_language", "pcm16_rms", "SAMPLE_RATE", "SAMPLE_WIDTH",
    "DEFAULT_RECOGNITION_LOCALE",
]
