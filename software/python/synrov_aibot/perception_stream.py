"""Dedicated Processing 3D perception stream for SynROV AiBot.

Control/telemetry remains on port 9000. This client connects to port 9001 and
uses the same SynROV application envelope as every other Processing channel.
The dedicated socket exists only to isolate image bandwidth from robot commands.
"""
from __future__ import annotations

import base64
import io
import json
import re
import threading
import time
from typing import Any, Callable, Optional
from urllib.parse import urlsplit, urlunsplit

from .protocol import (
    MESSAGE_PERCEPTION_FRAME,
    MESSAGE_PERCEPTION_SUBSCRIPTION,
    SOURCE_AIBOT,
    envelope_supported,
    message_envelope,
    unpack_message,
)
from .robot_context import refresh_state_intelligence_context

try:
    import websocket
except ImportError:  # pragma: no cover - optional dependency at import time
    websocket = None

try:
    from PIL import Image
except ImportError:  # pragma: no cover - optional dependency at import time
    Image = None

AI_PERCEPTION_PORT = 9001


def derive_perception_uri(control_uri: str, port: int = AI_PERCEPTION_PORT) -> str:
    """Return the dedicated perception URI for a control WebSocket URI."""
    raw = str(control_uri or "").strip() or "ws://127.0.0.1:9000/"
    if not re.match(r"^wss?://", raw, flags=re.IGNORECASE):
        raw = "ws://" + raw
    parts = urlsplit(raw)
    hostname = parts.hostname or "127.0.0.1"
    if ":" in hostname and not hostname.startswith("["):
        hostname = f"[{hostname}]"
    netloc = f"{hostname}:{int(port)}"
    return urlunsplit((parts.scheme or "ws", netloc, "/", "", ""))


class ProcessingPerceptionClient:
    """Reconnectable, read-mostly client for Processing's AI perception stream."""

    def __init__(
        self,
        state: Any,
        emit: Optional[Callable[[str, Any], None]] = None,
        *,
        max_frame_bytes: int = 8 * 1024 * 1024,
    ) -> None:
        self.state = state
        self.emit = emit or (lambda _kind, _payload=None: None)
        self.max_frame_bytes = int(max_frame_bytes)
        self.uri = derive_perception_uri("ws://127.0.0.1:9000/")
        self.connected = False
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._ws_app: Any = None
        self._runner_thread: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._run_token = 0
        self._send_seq = 0
        self._last_frame_event_ts = 0.0

    def _send_raw(self, payload: dict[str, Any]) -> bool:
        ws_app = self._ws_app
        if ws_app is None:
            return False
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        with self._lock:
            try:
                ws_app.send(data)
            except Exception:
                self.connected = False
                return False
        return True

    def _stream_message(self, action: str) -> dict[str, Any]:
        self._send_seq += 1
        return message_envelope(
            MESSAGE_PERCEPTION_SUBSCRIPTION,
            {"action": action, "stream": "ai_perception", "robot": getattr(self.state, "robot", None)},
            source=SOURCE_AIBOT,
            seq=self._send_seq,
        )

    def _heartbeat_loop(self, token: int) -> None:
        while not self._stop.wait(0.8) and token == self._run_token:
            if self.connected:
                self._send_raw(self._stream_message("ping"))

    def _decode_frame(self, frame_block: Any) -> Any:
        if Image is None or not isinstance(frame_block, dict):
            return None
        encoded = str(frame_block.get("data", "") or "").strip()
        if not encoded:
            return None
        if len(encoded) > self.max_frame_bytes * 2:
            raise ValueError("perception frame payload too large")
        raw = base64.b64decode(re.sub(r"\s+", "", encoded), validate=False)
        if len(raw) > self.max_frame_bytes:
            raise ValueError("decoded perception frame too large")
        return Image.open(io.BytesIO(raw)).convert("RGB")

    def _handle_message(self, message: str) -> None:
        try:
            envelope = json.loads(message)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(envelope, dict) or not envelope_supported(envelope, {MESSAGE_PERCEPTION_FRAME}):
            return
        _, payload = unpack_message(envelope, {MESSAGE_PERCEPTION_FRAME})

        now = time.time()
        scene = payload.get("scene") if isinstance(payload.get("scene"), dict) else {}
        runtime = payload.get("runtime") if isinstance(payload.get("runtime"), dict) else {}
        obstacles = scene.get("nearest_obstacles") if isinstance(scene.get("nearest_obstacles"), list) else []

        setattr(self.state, "perception_connected", True)
        setattr(self.state, "last_perception_ts", now)
        setattr(self.state, "perception_counter", int(getattr(self.state, "perception_counter", 0)) + 1)
        setattr(self.state, "perception_scene", dict(scene))
        setattr(self.state, "perception_runtime", dict(runtime))
        setattr(self.state, "perception_obstacles", list(obstacles))
        setattr(self.state, "perception_seq", int(envelope.get("seq", 0) or 0))
        refresh_state_intelligence_context(self.state, now=now)

        snapshot = getattr(self.state, "snapshot", None)
        if isinstance(snapshot, dict):
            snapshot["aiPerception"] = {
                "scene": dict(scene),
                "obstacles": list(obstacles),
                "seq": int(envelope.get("seq", 0) or 0),
                "timestamp_ms": int(envelope.get("timestampMs", 0) or 0),
            }

        try:
            image = self._decode_frame(payload.get("frame"))
        except (ValueError, TypeError, OSError) as exc:
            self.emit("log", f"[perception/frame] decode error: {exc}")
            image = None

        if image is not None:
            self.state.last_frame_pil = image
            self.state.last_frame_ts = now
            self.state.frame_counter = int(getattr(self.state, "frame_counter", 0)) + 1
            if now - self._last_frame_event_ts >= 0.25:
                self._last_frame_event_ts = now
                self.emit("frame", image)

        self.emit("perception", None)

    def connect(self, control_uri: str) -> None:
        if websocket is None:
            return
        self.disconnect()
        self.uri = derive_perception_uri(control_uri)
        self._stop.clear()
        self._run_token += 1
        token = self._run_token

        def on_open(_ws: Any) -> None:
            self.connected = True
            setattr(self.state, "perception_connected", True)
            self._send_raw(self._stream_message("subscribe"))
            self.emit("log", f"[perception/ws] connected to {self.uri}")

        def on_message(_ws: Any, message: str) -> None:
            self._handle_message(message)

        def on_error(_ws: Any, error: Any) -> None:
            self.connected = False
            setattr(self.state, "perception_connected", False)
            self.emit("log", f"[perception/ws] error: {error}")

        def on_close(_ws: Any, _code: Any, _msg: Any) -> None:
            self.connected = False
            setattr(self.state, "perception_connected", False)

        def runner() -> None:
            while not self._stop.is_set() and token == self._run_token:
                try:
                    self._ws_app = websocket.WebSocketApp(
                        self.uri,
                        on_open=on_open,
                        on_message=on_message,
                        on_error=on_error,
                        on_close=on_close,
                    )
                    self._ws_app.run_forever(http_proxy_host=None, http_proxy_port=None)
                except Exception as exc:
                    self.emit("log", f"[perception/ws] failed: {exc}")
                if self._stop.wait(0.7):
                    break

        self._runner_thread = threading.Thread(target=runner, daemon=True, name="synrov-perception-ws")
        self._runner_thread.start()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            args=(token,),
            daemon=True,
            name="synrov-perception-heartbeat",
        )
        self._heartbeat_thread.start()

    def disconnect(self) -> None:
        self._run_token += 1
        self._stop.set()
        if self.connected:
            self._send_raw(self._stream_message("disconnect"))
        ws_app = self._ws_app
        self._ws_app = None
        if ws_app is not None:
            try:
                ws_app.close()
            except Exception:
                pass
        self.connected = False
        setattr(self.state, "perception_connected", False)


__all__ = [
    "AI_PERCEPTION_PORT",
    "derive_perception_uri",
    "ProcessingPerceptionClient",
]
