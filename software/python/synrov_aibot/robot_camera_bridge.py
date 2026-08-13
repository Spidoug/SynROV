"""AiBot -> Processing robot-camera bridge (port 9002).

The bridge is intentionally independent from control (9000) and perception
(9001). Processing may request Vehicle/Drone camera view even when no physical
camera exists: AiBot then publishes the latest SynROV perception image, or a
small generated fallback frame. When an enabled webcam supplies a frame, that
physical image automatically becomes the preferred source.
"""
from __future__ import annotations

import base64
import io
import json
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional
from urllib.parse import urlsplit, urlunsplit

from .protocol import (
    MESSAGE_CAMERA_CONTROL, MESSAGE_CAMERA_FRAME, SOURCE_AIBOT,
    envelope_supported, message_envelope, unpack_message,
)

try:
    import websocket
except Exception:  # pragma: no cover - optional runtime dependency
    websocket = None

try:
    from PIL import Image, ImageDraw
except Exception:  # pragma: no cover
    Image = None
    ImageDraw = None

SUPPORTED_ROBOTS = {"Vehicle": "vehicle", "Drone": "drone"}


def camera_uri_from_control_uri(control_uri: str) -> str:
    """Map ws(s)://host:9000/... to the dedicated ws(s)://host:9002/ URI."""
    raw = str(control_uri or "ws://127.0.0.1:9000/").strip()
    try:
        parsed = urlsplit(raw)
        host = parsed.hostname or "127.0.0.1"
        if ":" in host and not host.startswith("["):
            host = f"[{host}]"
        netloc = f"{host}:9002"
        return urlunsplit((parsed.scheme or "ws", netloc, "/", "", ""))
    except Exception:
        return "ws://127.0.0.1:9002/"


def _copy_image(image: Any) -> Any:
    if image is None:
        return None
    try:
        return image.copy()
    except Exception:
        return image


@dataclass
class CameraFrame:
    robot: str
    image: Any
    source: str
    ts: float


class AiBotRobotCameraBridge:
    """Latest-frame WebSocket publisher controlled by Processing messages."""

    def __init__(self, *, width: int = 640, height: int = 360, jpeg_quality: int = 72) -> None:
        self.width = max(160, int(width))
        self.height = max(90, int(height))
        self.jpeg_quality = max(35, min(92, int(jpeg_quality)))
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._control_uri = "ws://127.0.0.1:9000/"
        self._latest: Dict[str, CameraFrame] = {}
        self._enabled: Dict[str, bool] = {"vehicle": False, "drone": False}
        self._target_fps: Dict[str, float] = {"vehicle": 12.0, "drone": 12.0}
        self._seq: Dict[str, int] = {"vehicle": 0, "drone": 0}
        self._last_send: Dict[str, float] = {"vehicle": 0.0, "drone": 0.0}
        self._connected = False
        self._last_source: Dict[str, str] = {"vehicle": "fallback", "drone": "fallback"}

    @property
    def connected(self) -> bool:
        return bool(self._connected)

    def start(self) -> None:
        if websocket is None or self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="synrov-camera-9002", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._thread = None

    def submit(
        self,
        *,
        control_uri: str,
        robot: str,
        physical_image: Any = None,
        fallback_image: Any = None,
    ) -> None:
        key = SUPPORTED_ROBOTS.get(str(robot or ""))
        if key is None:
            return
        source = "physical_webcam" if physical_image is not None else "synrov_fallback"
        image = physical_image if physical_image is not None else fallback_image
        with self._lock:
            self._control_uri = str(control_uri or self._control_uri)
            self._latest[key] = CameraFrame(key, _copy_image(image), source, time.time())
            self._last_source[key] = source if image is not None else "generated_fallback"
        self.start()

    def status(self, robot: str) -> Dict[str, Any]:
        key = SUPPORTED_ROBOTS.get(str(robot or ""), str(robot or "").lower())
        with self._lock:
            return {
                "connected": self.connected,
                "enabled": bool(self._enabled.get(key, False)),
                "source": self._last_source.get(key, "generated_fallback"),
                "port": 9002,
            }

    def _placeholder(self, robot: str) -> Any:
        if Image is None:
            return None
        image = Image.new("RGB", (self.width, self.height), (22, 27, 34))
        if ImageDraw is not None:
            try:
                draw = ImageDraw.Draw(image)
                draw.text((24, 24), f"SynROV AiBot | {robot.title()} camera", fill=(235, 238, 242))
                draw.text((24, 52), "No physical camera - AiBot fallback stream", fill=(180, 188, 198))
                draw.text((24, 80), time.strftime("%Y-%m-%d %H:%M:%S"), fill=(145, 154, 166))
            except Exception:
                pass
        return image

    def _jpeg(self, frame: CameraFrame) -> Optional[bytes]:
        image = frame.image if frame.image is not None else self._placeholder(frame.robot)
        if image is None or Image is None:
            return None
        try:
            if not isinstance(image, Image.Image):
                image = Image.fromarray(image)
            if image.mode != "RGB":
                image = image.convert("RGB")
            image = image.copy()
            image.thumbnail((self.width, self.height))
            canvas = Image.new("RGB", (self.width, self.height), (0, 0, 0))
            x = max(0, (self.width - image.width) // 2)
            y = max(0, (self.height - image.height) // 2)
            canvas.paste(image, (x, y))
            buf = io.BytesIO()
            canvas.save(buf, format="JPEG", quality=self.jpeg_quality, optimize=False)
            return buf.getvalue()
        except Exception:
            return None

    def _handle_control(self, raw: Any) -> None:
        if not isinstance(raw, str) or not raw:
            return
        try:
            msg = json.loads(raw)
        except Exception:
            return
        if not envelope_supported(msg, {MESSAGE_CAMERA_CONTROL}):
            return
        _, payload = unpack_message(msg, {MESSAGE_CAMERA_CONTROL})
        robot = str(payload.get("robot", "")).lower()
        if robot not in self._enabled:
            return
        enabled = bool(payload.get("enabled", False))
        action = str(payload.get("action", "")).lower()
        if action == "start":
            enabled = True
        elif action == "stop":
            enabled = False
        fps = payload.get("target_fps")
        if isinstance(fps, (int, float)) and fps > 0:
            self._target_fps[robot] = max(1.0, min(24.0, float(fps)))
        self._enabled[robot] = enabled

    def _send_due(self, ws: Any) -> None:
        now = time.monotonic()
        for robot in ("vehicle", "drone"):
            if not self._enabled.get(robot, False):
                continue
            period = 1.0 / max(1.0, self._target_fps.get(robot, 12.0))
            if now - self._last_send.get(robot, 0.0) < period:
                continue
            with self._lock:
                frame = self._latest.get(robot)
                source = self._last_source.get(robot, "generated_fallback")
            if frame is None:
                frame = CameraFrame(robot, None, "generated_fallback", time.time())
                source = "generated_fallback"
            jpeg = self._jpeg(frame)
            if not jpeg:
                continue
            seq = self._seq[robot]
            payload = {
                "robot": robot,
                "encoding": "jpeg_base64",
                "width": self.width,
                "height": self.height,
                "cameraSource": source,
                "fallback": source != "physical_webcam",
                "image": base64.b64encode(jpeg).decode("ascii"),
            }
            envelope = message_envelope(
                MESSAGE_CAMERA_FRAME, payload, source=SOURCE_AIBOT, seq=seq
            )
            ws.send(json.dumps(envelope, separators=(",", ":")))
            self._seq[robot] = seq + 1
            self._last_send[robot] = now

    def _run(self) -> None:
        retry = 0.35
        ws = None
        current_uri = ""
        while not self._stop.is_set():
            try:
                with self._lock:
                    desired_uri = camera_uri_from_control_uri(self._control_uri)
                if ws is None or desired_uri != current_uri:
                    if ws is not None:
                        try:
                            ws.close()
                        except Exception:
                            pass
                    ws = websocket.create_connection(desired_uri, timeout=0.8)
                    ws.settimeout(0.025)
                    current_uri = desired_uri
                    self._connected = True
                    retry = 0.35
                try:
                    raw = ws.recv()
                    self._handle_control(raw)
                except websocket.WebSocketTimeoutException:
                    pass
                self._send_due(ws)
                time.sleep(0.008)
            except Exception:
                self._connected = False
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
                ws = None
                self._stop.wait(retry)
                retry = min(3.0, retry * 1.45)
        self._connected = False
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass
