"""SynROV dedicated runtime — robot identity resolved from WebSocket messages."""
from __future__ import annotations

import argparse
import base64
import io
import json
import queue
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

try:
    import websocket  # type: ignore
except Exception:  # pragma: no cover
    websocket = None  # type: ignore

try:
    from PIL import Image  # type: ignore
except Exception:  # pragma: no cover
    Image = None  # type: ignore

from .dataset import canonical_robot
from .orchestrator import SynROVOrchestrator
from .robot_ai import CommandResult, RobotAIRegistry
from .safety import MANIP_KEYS, MANIP_POSE_LIMITS, SynROVSafetyLayer, clamp

DEFAULT_URI = "ws://127.0.0.1:9000/"


@dataclass
class ModernSynROVState:
    connected: bool = False
    status: str = "disconnected"
    robot: str = "Manipulator"
    detected_robot_raw: str = ""
    sensors: Dict[str, Any] = field(default_factory=dict)
    control: Dict[str, Any] = field(default_factory=dict)
    snapshot: Dict[str, Any] = field(default_factory=dict)
    servos: Dict[int, float] = field(default_factory=dict)
    last_msg_ts: float = 0.0
    last_telemetry_ts: float = 0.0
    telemetry_counter: int = 0
    last_frame_pil: Any = None
    last_frame_ts: float = 0.0
    frame_counter: int = 0
    last_runtime: str = ""


class ModernSynROVBridge:
    """WebSocket bridge whose robot identity comes from SynROV, not a UI selector."""

    ROBOT_KEYS = (
        "robot", "detectedRobot", "detected_robot",
        "activeRobot", "active_robot", "robotName", "robot_name",
        "selectedRobot", "selected_robot",
    )

    def __init__(self, event_queue: Optional["queue.Queue[Tuple[str, Any]]"] = None) -> None:
        self.event_queue: "queue.Queue[Tuple[str, Any]]" = event_queue or queue.Queue()
        self.state = ModernSynROVState()
        self.uri = DEFAULT_URI
        self._ws_app = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._control_lock = threading.Lock()
        self._send_seq = 0
        self._last_control_send_ts = 0.0
        self._control_send_interval_s = 0.08

        # Non-blocking command sender: commands are queued here and the
        # background thread handles rate-limiting without ever blocking the
        # caller (typically the UI thread).  maxsize=4: when full the oldest
        # pending command is discarded so the robot always acts on the latest
        # intent.
        self._control_queue: queue.Queue = queue.Queue(maxsize=4)
        self._control_sender_thread = threading.Thread(
            target=self._control_sender_loop, daemon=True
        )
        self._control_sender_thread.start()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _control_sender_loop(self) -> None:
        """Background thread: drain _control_queue with rate limiting."""
        while not self._stop.is_set():
            try:
                payload = self._control_queue.get(timeout=0.25)
            except queue.Empty:
                continue

            with self._control_lock:
                wait_s = self._control_send_interval_s - (
                    time.time() - self._last_control_send_ts
                )
                if wait_s > 0:
                    time.sleep(min(wait_s, self._control_send_interval_s))
                self._last_control_send_ts = time.time()
                out = dict(payload or {})
                out.setdefault("type", "control_intent")
                out.setdefault("controlSource", "synrov_dedicated_ai")
                try:
                    self.send_json(out)
                except Exception:
                    pass

    def _emit(self, kind: str, payload: Any = None) -> None:
        try:
            self.event_queue.put((kind, payload))
        except Exception:
            pass

    def _heartbeat_loop(self) -> None:
        while not self._stop.is_set():
            time.sleep(0.6)
            if not self.state.connected:
                continue
            try:
                self.send_json({"client": "ping", "runtime": "synrov_dedicated_ai"})
                if time.time() - self.state.last_telemetry_ts > 1.5:
                    self.send_json({"sync": True, "requestRobotIdentity": True})
            except Exception:
                pass

    def _robot_from_block(self, block: Dict[str, Any]) -> Optional[str]:
        for key in self.ROBOT_KEYS:
            if block.get(key):
                return str(block[key])
        control = block.get("control") if isinstance(block.get("control"), dict) else None
        if isinstance(control, dict):
            for key in self.ROBOT_KEYS:
                if control.get(key):
                    return str(control[key])
            if isinstance(control.get("manipulator"), dict):
                return "Manipulator"
            if isinstance(control.get("drive"), dict):
                return "Vehicle"
            if isinstance(control.get("flight"), dict):
                return "Drone"
        if isinstance(block.get("manipulator"), dict):
            return "Manipulator"
        if isinstance(block.get("drive"), dict):
            return "Vehicle"
        if isinstance(block.get("flight"), dict):
            return "Drone"
        return None

    def _handle_message(self, message: str) -> None:
        try:
            payload = json.loads(message)
        except Exception:
            self._emit("log", f"[runtime/ws/raw] {str(message)[:200]}")
            return
        if not isinstance(payload, dict):
            return

        changed = False
        blocks: List[Dict[str, Any]] = [payload]
        for key in ("snapshot", "state"):
            if isinstance(payload.get(key), dict):
                blocks.append(payload[key])

        for block in blocks:
            robot_raw = self._robot_from_block(block)
            if robot_raw:
                self.state.detected_robot_raw = robot_raw
                self.state.robot = canonical_robot(robot_raw, self.state.robot)
                changed = True
            if isinstance(block.get("sensors"), dict):
                self.state.sensors.update(block["sensors"])
                self.state.telemetry_counter += 1
                self.state.last_telemetry_ts = time.time()
                changed = True
            if isinstance(block.get("control"), dict):
                self.state.control.update(block["control"])
                changed = True
            if isinstance(block.get("snapshot"), dict):
                self.state.snapshot.update(block["snapshot"])
                changed = True

        if "servo" in payload and "angle" in payload:
            try:
                self.state.servos[int(float(payload["servo"]))] = float(payload["angle"])
                self.state.telemetry_counter += 1
                self.state.last_telemetry_ts = time.time()
                changed = True
            except Exception:
                pass

        frame_payload = (
            payload.get("frame")
            or payload.get("dataUrl")
            or payload.get("image")
        )
        if frame_payload:
            self._handle_frame(frame_payload)
            changed = True

        if changed:
            self.state.connected = True
            self.state.status = "connected"
            self._emit("state", self.state)

    def _handle_frame(self, image_payload: Any) -> None:
        if Image is None:
            return
        try:
            text = str(image_payload).strip()
            if text.startswith("data:image/") and "," in text:
                text = text.split(",", 1)[1]
            raw = base64.b64decode(re.sub(r"\s+", "", text), validate=False)
            self.state.last_frame_pil = Image.open(io.BytesIO(raw)).convert("RGB")
            self.state.last_frame_ts = time.time()
            self.state.frame_counter += 1
            self._emit("frame", self.state.last_frame_pil)
        except Exception as exc:
            self._emit("log", f"[runtime/frame] decode error: {exc}")

    def _with_metadata(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        self._send_seq += 1
        out = dict(payload or {})
        out.setdefault("source", "synrov_dedicated_ai")
        out.setdefault("origin", "synrov_dedicated_ai")
        out.setdefault("detectedRobot", self.state.robot)
        out.setdefault("sentAt", time.time())
        out.setdefault("seq", self._send_seq)
        return out

    # ------------------------------------------------------------------
    # Public API — connection
    # ------------------------------------------------------------------

    def connect(self, uri: str = DEFAULT_URI) -> None:
        if websocket is None:
            raise RuntimeError("websocket-client is not installed")
        self.disconnect()
        self.uri = str(uri or DEFAULT_URI).strip() or DEFAULT_URI
        self._stop.clear()
        if not getattr(self, "_control_sender_thread", None) or not self._control_sender_thread.is_alive():
            self._control_sender_thread = threading.Thread(
                target=self._control_sender_loop, daemon=True
            )
            self._control_sender_thread.start()
        self.state.status = "connecting"
        self._emit("state", self.state)

        def on_open(ws: Any) -> None:
            self.state.connected = True
            self.state.status = "connected"
            now = time.time()
            self.state.last_msg_ts = now
            self.state.last_telemetry_ts = now
            self.send_json({"client": "connect", "runtime": "synrov_dedicated_ai"})
            self.send_json({"sync": True, "requestRobotIdentity": True})
            self._emit("log", f"[runtime/ws] connected to {self.uri}")
            self._emit("state", self.state)

        def on_message(ws: Any, message: str) -> None:
            self.state.last_msg_ts = time.time()
            self._handle_message(message)

        def on_error(ws: Any, error: Any) -> None:
            self.state.connected = False
            self.state.status = "error"
            self._emit("log", f"[runtime/ws] error: {error}")
            self._emit("state", self.state)

        def on_close(ws: Any, code: Any, msg: Any) -> None:
            self.state.connected = False
            self.state.status = "disconnected"
            self._emit("log", f"[runtime/ws] disconnected ({code}, {msg})")
            self._emit("state", self.state)

        def runner() -> None:
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
                self._emit("log", f"[runtime/ws] failed: {exc}")

        threading.Thread(target=runner, daemon=True).start()
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()

    def disconnect(self) -> None:
        self._stop.set()
        try:
            self.send_json({"client": "disconnect", "runtime": "synrov_dedicated_ai"})
        except Exception:
            pass
        if self._ws_app is not None:
            try:
                self._ws_app.close()
            except Exception:
                pass
        self.state.connected = False
        self.state.status = "disconnected"

    # ------------------------------------------------------------------
    # Public API — messaging
    # ------------------------------------------------------------------

    def send_json(self, payload: Dict[str, Any]) -> None:
        if self._ws_app is None:
            return
        with self._lock:
            self._ws_app.send(json.dumps(self._with_metadata(payload), ensure_ascii=False))

    def send_control(self, payload: Dict[str, Any]) -> None:
        """Enqueue a control command for the background sender.

        Never blocks.  If the queue is full (robot is being commanded faster
        than 12.5 Hz) the oldest pending command is discarded so the newest
        intent always wins.
        """
        self.state.last_runtime = json.dumps(payload, ensure_ascii=False)
        try:
            while self._control_queue.full():
                self._control_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self._control_queue.put_nowait(payload)
        except queue.Full:
            pass  # unreachable after the drain above

    # ------------------------------------------------------------------
    # Public API — robot commands
    # ------------------------------------------------------------------

    def command_manipulator_pose(self, pose: Dict[str, Any]) -> str:
        safe = {
            k: clamp((pose or {}).get(k, 50.0 if k == "grip" else 180.0), *MANIP_POSE_LIMITS[k])
            for k in MANIP_KEYS
        }
        angles = [int(round(safe[k])) for k in MANIP_KEYS]
        self.state.servos.update({i: float(v) for i, v in enumerate(angles)})
        self.send_control({
            "type": "control_intent",
            "control": {"robot": "Manipulator", "manipulator": {"angles": angles}},
        })
        return "CONTROL:Manipulator"

    def manip_home(self) -> None:
        self.state.servos.update({0: 180.0, 1: 45.0, 2: 180.0, 3: 90.0, 4: 95.0, 5: 130.0, 6: 50.0})
        self.send_control({"type": "control_intent", "control": {"robot": "Manipulator", "action": "home"}})

    def send_vehicle(
        self,
        throttle: float,
        steer: float,
        cam_pan: float = 0.0,
        cam_tilt: float = 0.0,
    ) -> str:
        drive = {
            "mode": "arc",
            "throttle": clamp(throttle, -1, 1),
            "steer": clamp(steer, -1, 1),
            "camPan": clamp(cam_pan, -90, 90),
            "camTilt": clamp(cam_tilt, -45, 45),
        }
        self.state.control.update({"robot": "Vehicle", "drive": drive})
        self.send_control({"type": "control_intent", "control": {"robot": "Vehicle", "drive": drive}})
        return "CONTROL:Vehicle"

    def send_drone(
        self,
        throttle: float,
        yaw: float,
        pitch: float,
        roll: float,
        strafe: float,
        forward: float,
    ) -> str:
        flight = {
            "throttle": clamp(throttle, -1, 1),
            "yaw": clamp(yaw, -1, 1),
            "pitch": clamp(pitch, -1, 1),
            "roll": clamp(roll, -1, 1),
            "strafe": clamp(strafe, -1, 1),
            "forward": clamp(forward, -1, 1),
        }
        self.state.control.update({"robot": "Drone", "flight": flight})
        self.send_control({"type": "control_intent", "control": {"robot": "Drone", "flight": flight}})
        return "CONTROL:Drone"

    def drone_takeoff(self) -> None:
        self.send_control({"type": "control_intent", "control": {"robot": "Drone", "action": "takeoff"}})

    def drone_land(self) -> None:
        self.send_control({"type": "control_intent", "control": {"robot": "Drone", "action": "land"}})


class SynROVDedicatedRuntime:
    """High-level runtime combining bridge, safety, orchestrator and AI registry."""

    def __init__(self, bridge: Optional[ModernSynROVBridge] = None) -> None:
        self.bridge = bridge or ModernSynROVBridge()
        self.safety = SynROVSafetyLayer()
        self.orchestrator = SynROVOrchestrator()
        self.registry = RobotAIRegistry(self.safety)
        self.last_result = CommandResult(False, "Manipulator", reason="not_started")

    @property
    def active_robot(self) -> str:
        return canonical_robot(self.bridge.state.robot)

    def execute_text(self, text: str) -> CommandResult:
        result = self.registry.execute_text(text, self.bridge, self.orchestrator)
        self.last_result = result
        self.bridge._emit("command_result", result)
        return result

    def tick(self) -> CommandResult:
        self.orchestrator.begin_cycle()
        result = self.registry.tick_active(self.bridge, self.orchestrator)
        if result.ok:
            self.last_result = result
            self.bridge._emit("mission_tick", result)
        return result

    def stop_active(self) -> None:
        self.registry.stop_active(self.bridge, self.orchestrator)

    def status(self) -> Dict[str, Any]:
        return {
            "active_robot": self.active_robot,
            "detected_robot_raw": self.bridge.state.detected_robot_raw,
            "connected": self.bridge.state.connected,
            "models": self.registry.metadata(),
            "orchestrator": self.orchestrator.health(),
            "last_result": self.last_result.__dict__,
        }


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="SynROV dedicated runtime")
    parser.add_argument("--uri", default=DEFAULT_URI, help="SynROV WebSocket URI")
    parser.add_argument("--loop", action="store_true", help="Keep the mission tick loop active")
    args = parser.parse_args(argv)

    bridge = ModernSynROVBridge()
    runtime = SynROVDedicatedRuntime(bridge)
    bridge.connect(args.uri)
    print("SynROV dedicated AI runtime active. Robot selection comes from WebSocket identification.")

    try:
        while args.loop:
            while not bridge.event_queue.empty():
                kind, payload = bridge.event_queue.get_nowait()
                if kind == "log":
                    print(payload)
                elif kind == "state":
                    print(f"[state] robot={bridge.state.robot} connected={bridge.state.connected}")
            runtime.tick()
            time.sleep(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.disconnect()


if __name__ == "__main__":
    main()
