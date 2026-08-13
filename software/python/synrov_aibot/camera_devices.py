"""Physical camera discovery and capture for SynROV AiBot.

The GUI and runtime use this module as the single authority for local webcams.
It validates real devices before exposing them, supports an automatic choice,
and uses platform-appropriate OpenCV capture backends with safe fallbacks.
"""
from __future__ import annotations

import os
import platform
from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Sequence, Tuple

try:
    import cv2
except Exception:  # optional dependency
    cv2 = None

try:
    from PIL import Image
except Exception:  # optional dependency
    Image = None


@dataclass(frozen=True)
class CameraDevice:
    index: int
    backend: int
    backend_name: str
    width: int = 0
    height: int = 0

    @property
    def label(self) -> str:
        """Language-neutral device identity; UI code localizes the visible prefix."""
        size = f" · {self.width}x{self.height}" if self.width > 0 and self.height > 0 else ""
        return f"Camera {self.index} · {self.backend_name}{size}"


def _backend_candidates() -> List[Tuple[int, str]]:
    if cv2 is None:
        return []
    system = platform.system().lower()
    candidates: List[Tuple[int, str]] = []
    if system == "windows":
        for attr, name in (("CAP_DSHOW", "DirectShow"), ("CAP_MSMF", "Media Foundation")):
            value = getattr(cv2, attr, None)
            if value is not None:
                candidates.append((int(value), name))
    elif system == "darwin":
        value = getattr(cv2, "CAP_AVFOUNDATION", None)
        if value is not None:
            candidates.append((int(value), "AVFoundation"))
    else:
        value = getattr(cv2, "CAP_V4L2", None)
        if value is not None:
            candidates.append((int(value), "V4L2"))
    candidates.append((int(getattr(cv2, "CAP_ANY", 0)), "OpenCV Auto"))

    out: List[Tuple[int, str]] = []
    seen = set()
    for backend, name in candidates:
        if backend in seen:
            continue
        seen.add(backend)
        out.append((backend, name))
    return out


def _open_capture(index: int, *, read_probe: bool = True) -> Tuple[Any, Optional[CameraDevice]]:
    if cv2 is None:
        return None, None
    for backend, backend_name in _backend_candidates():
        cap = None
        try:
            cap = cv2.VideoCapture(int(index), int(backend)) if backend else cv2.VideoCapture(int(index))
            if cap is None or not cap.isOpened():
                if cap is not None:
                    cap.release()
                continue
            try:
                cap.set(getattr(cv2, "CAP_PROP_BUFFERSIZE", 38), 1)
            except Exception:
                pass
            ok = True
            frame = None
            if read_probe:
                ok, frame = cap.read()
            if not ok:
                cap.release()
                continue
            width = int(cap.get(getattr(cv2, "CAP_PROP_FRAME_WIDTH", 3)) or 0)
            height = int(cap.get(getattr(cv2, "CAP_PROP_FRAME_HEIGHT", 4)) or 0)
            if frame is not None:
                try:
                    height, width = int(frame.shape[0]), int(frame.shape[1])
                except Exception:
                    pass
            return cap, CameraDevice(int(index), int(backend), backend_name, width, height)
        except Exception:
            try:
                if cap is not None:
                    cap.release()
            except Exception:
                pass
    return None, None


def discover_camera_devices(max_index: int = 10) -> List[CameraDevice]:
    """Return only camera indices that can be opened and deliver a frame."""
    if cv2 is None:
        return []
    limit = max(1, min(32, int(max_index)))
    devices: List[CameraDevice] = []
    # On Linux, avoid probing clearly absent /dev/video nodes when possible.
    existing_linux = None
    if platform.system().lower() == "linux":
        existing_linux = {i for i in range(limit) if os.path.exists(f"/dev/video{i}")}
        if not existing_linux:
            return []
    for index in range(limit):
        if existing_linux is not None and existing_linux and index not in existing_linux:
            continue
        cap, device = _open_capture(index, read_probe=True)
        try:
            if cap is not None:
                cap.release()
        except Exception:
            pass
        if device is not None:
            devices.append(device)
    return devices


class WebcamSource:
    """Validated webcam source shared by all AiBot components."""

    AUTO_INDEX = -1

    def __init__(self) -> None:
        self.cap: Any = None
        self.enabled = False
        self.index = self.AUTO_INDEX
        self.device: Optional[CameraDevice] = None
        self.last_frame_pil: Any = None
        self.last_error = ""

    @property
    def backend_name(self) -> str:
        return self.device.backend_name if self.device is not None else "none"

    def start(self, index: int = AUTO_INDEX, devices: Optional[Sequence[CameraDevice]] = None) -> CameraDevice:
        if cv2 is None or Image is None:
            raise RuntimeError("opencv-python and pillow are required for webcam support")
        self.stop()
        wanted = int(index)
        candidates: Iterable[int]
        if wanted >= 0:
            candidates = (wanted,)
        else:
            known = list(devices) if devices is not None else discover_camera_devices()
            candidates = [device.index for device in known]
            if not candidates:
                if platform.system().lower() == "linux" and not any(os.path.exists(f"/dev/video{i}") for i in range(10)):
                    self.last_error = "no physical camera available"
                    raise RuntimeError(self.last_error)
                candidates = range(10)

        tried: List[int] = []
        for camera_index in candidates:
            camera_index = int(camera_index)
            if camera_index in tried:
                continue
            tried.append(camera_index)
            cap, device = _open_capture(camera_index, read_probe=True)
            if cap is None or device is None:
                continue
            self.cap = cap
            self.device = device
            self.enabled = True
            self.index = camera_index
            self.last_error = ""
            # Request a useful live-view resolution without assuming support.
            try:
                cap.set(getattr(cv2, "CAP_PROP_FRAME_WIDTH", 3), 1280)
                cap.set(getattr(cv2, "CAP_PROP_FRAME_HEIGHT", 4), 720)
            except Exception:
                pass
            return device

        self.last_error = (
            f"could not open camera {wanted}" if wanted >= 0
            else "no physical camera available"
        )
        raise RuntimeError(self.last_error)

    def stop(self) -> None:
        self.enabled = False
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = None
        self.device = None
        self.last_frame_pil = None

    def update(self) -> Optional[Any]:
        if not self.enabled or self.cap is None or cv2 is None or Image is None:
            return self.last_frame_pil
        try:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.last_error = "camera_frame_unavailable"
                return self.last_frame_pil
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.last_frame_pil = Image.fromarray(rgb)
            self.last_error = ""
        except Exception as exc:
            self.last_error = str(exc)
        return self.last_frame_pil


__all__ = ["CameraDevice", "WebcamSource", "discover_camera_devices"]
