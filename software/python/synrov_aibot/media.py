"""Audio, vision and webcam utilities for the SynROV runtime."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, List, Optional

from .perception import (
    SynROVSnapshot,
    SynROVSnapshotBuilder,
    TelemetrySnapshot,
    VisionSnapshot,
    VoiceSnapshot,
)
from .primitives import safe_float


@dataclass
class VisionInfo:
    confidence: float = 0.0
    dx: float = 0.0
    dy: float = 0.0
    area: float = 0.0
    width: float = 0.0
    height: float = 0.0
    centered: float = 0.0
    close: float = 0.0

    def as_vector(self) -> List[float]:
        return [self.confidence, self.dx, self.dy, self.area, self.width, self.height, self.centered, self.close]


class WebcamSource:
    """Small optional OpenCV webcam wrapper."""

    def __init__(self, index: int = 0) -> None:
        self.index = int(index)
        self._cap: Any = None

    def open(self) -> bool:
        try:
            import cv2
            self._cap = cv2.VideoCapture(self.index)
            return bool(self._cap and self._cap.isOpened())
        except Exception:
            self._cap = None
            return False

    def read(self) -> Optional[Any]:
        if self._cap is None and not self.open():
            return None
        try:
            ok, frame = self._cap.read()
            return frame if ok else None
        except Exception:
            return None

    def close(self) -> None:
        try:
            if self._cap is not None:
                self._cap.release()
        finally:
            self._cap = None


def _vector_size(size: Any, default: int) -> int:
    try:
        return max(0, int(size))
    except Exception:
        return default


def extract_audio_features(samples: Any, size: int = 12) -> List[float]:
    length = _vector_size(size, 12)
    if length <= 0:
        return []
    try:
        vals = [safe_float(v) for v in ([] if samples is None else list(samples))]
    except Exception:
        vals = []
    if not vals:
        return [0.0] * length

    rms = math.sqrt(sum(v * v for v in vals) / len(vals))
    peak = max(abs(v) for v in vals)
    mean = sum(vals) / len(vals)
    zcr = sum(1 for a, b in zip(vals, vals[1:]) if (a >= 0) != (b >= 0)) / max(1, len(vals) - 1)
    base = [rms, peak, mean, zcr]
    base.extend([0.0] * max(0, length - len(base)))
    return base[:length]


def pil_to_feature_vector(image: Any, size: int = 16) -> List[float]:
    length = _vector_size(size, 16)
    if length <= 0:
        return []
    try:
        side = max(1, int(math.ceil(math.sqrt(length))))
        img = image.convert("L").resize((side, side))
        vals = [p / 255.0 for p in list(img.getdata())]
    except Exception:
        vals = []
    vals = vals[:length]
    vals.extend([0.0] * max(0, length - len(vals)))
    return vals


def detect_object_from_pil(image: Any) -> VisionInfo:
    try:
        img = image.convert("L")
        w, h = img.size
        data = list(img.getdata())
        if not data:
            return VisionInfo()

        threshold = sum(data) / len(data)
        xs: List[int] = []
        ys: List[int] = []
        for i, px in enumerate(data):
            if px > threshold:
                xs.append(i % w)
                ys.append(i // w)
        if not xs:
            return VisionInfo()

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        cx = (min_x + max_x) / (2.0 * max(1, w))
        cy = (min_y + max_y) / (2.0 * max(1, h))
        bw = (max_x - min_x + 1) / max(1, w)
        bh = (max_y - min_y + 1) / max(1, h)
        area = len(xs) / max(1, w * h)
        dx = (cx - 0.5) * 2.0
        dy = (cy - 0.5) * 2.0
        centered = 1.0 - min(1.0, abs(dx) + abs(dy))
        close = min(1.0, area * 4.0)
        return VisionInfo(0.5, dx, dy, area, bw, bh, centered, close)
    except Exception:
        return VisionInfo()


__all__ = [
    "VisionInfo",
    "WebcamSource",
    "extract_audio_features",
    "pil_to_feature_vector",
    "detect_object_from_pil",
    "SynROVSnapshot",
    "SynROVSnapshotBuilder",
    "VoiceSnapshot",
    "VisionSnapshot",
    "TelemetrySnapshot",
]
