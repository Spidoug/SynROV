"""SynROV dedicated runtime — Processing owns robot identity and language."""
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
    import websocket
except ImportError:
    websocket = None

_WS_ERRORS = (OSError, RuntimeError, ValueError)
if websocket is not None and hasattr(websocket, "WebSocketException"):
    _WS_ERRORS = _WS_ERRORS + (websocket.WebSocketException,)

try:
    from PIL import Image
except ImportError:
    Image = None

from .dataset import canonical_robot
from .orchestrator import SynROVOrchestrator
from .protocol import control_envelope, processing_language, state_schema_supported
from .robot_ai import CommandResult, RobotAIRegistry
from .safety import MANIP_KEYS, MANIP_POSE_LIMITS, SynROVSafetyLayer, clamp

DEFAULT_URI = "ws://127.0.0.1:9000/"


@dataclass
class ModernSynROVState:
    connected: bool = False
    status: str = "disconnected"
    robot: str = "Manipulator"
    detected_robot_raw: str = ""
    language: str = ""
    sensors: Dict[str, Any] = field(default_factory=dict)
    control: Dict[str, Any] = field(default_factory=dict)
    snapshot: Dict[str, Any] = field(default_factory=dict)
    gps: Dict[str, Any] = field(default_factory=dict)
    servos: Dict[int, float] = field(default_factory=dict)
    last_msg_ts: float = 0.0
    last_telemetry_ts: float = 0.0
    telemetry_counter: int = 0
    last_frame_pil: Any = None
    last_frame_ts: float = 0.0
    frame_counter: int = 0
    last_runtime: str = ""


class ModernSynROVBridge:
    """V1 bridge using the same command shapes as the SynROV HTML console."""

    ROBOT_KEYS = (
        "robot",
        "detectedRobot",
        "detected_robot",
        "activeRobot",
        "active_robot",
        "robotName",
        "robot_name",
        "selectedRobot",
        "selected_robot",
    )

    def __init__(self, event_queue: Optional["queue.Queue[Tuple[str, Any]]"] = None) -> None:
        self.event_queue: "queue.Queue[Tuple[str, Any]]" = event_queue or queue.Queue()
        self.state = ModernSynROVState()
        self.uri = DEFAULT_URI
        self._ws_app: Any = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._control_lock = threading.Lock()
        self._send_seq = 0
        self._last_control_send_ts = 0.0
        self._control_send_interval_s = 0.08
        self._run_token = 0
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._runner_thread: Optional[threading.Thread] = None
        self._control_sender_thread: Optional[threading.Thread] = None
        self._max_frame_bytes = 8 * 1024 * 1024
        self._control_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=4)

    def _emit(self, kind: str, payload: Any = None) -> None:
        self.event_queue.put((kind, payload))

    def _ensure_control_sender(self, token: int) -> None:
        if self._control_sender_thread is not None and self._control_sender_thread.is_alive():
            return
        self._control_sender_thread = threading.Thread(
            target=self._control_sender_loop,
            args=(token,),
            daemon=True,
            name="synrov-control-sender",
        )
        self._control_sender_thread.start()

    def _control_sender_loop(self, token: int) -> None:
        while not self._stop.is_set() and token == self._run_token:
            try:
                payload = self._control_queue.get(timeout=0.25)
            except queue.Empty:
                continue
            self._send_control_now(payload)

    def _send_control_now(self, payload: Dict[str, Any]) -> None:
        with self._control_lock:
            wait_s = self._control_send_interval_s - (time.monotonic() - self._last_control_send_ts)
            if wait_s > 0:
                time.sleep(min(wait_s, self._control_send_interval_s))
            self._last_control_send_ts = time.monotonic()
            out = dict(payload or {})
            if "control" in out:
                out.setdefault("type", "control_intent")
                out.setdefault("controlSource", "synrov_dedicated_ai")
            self.send_json(out)

    def _heartbeat_loop(self, token: int) -> None:
        while not self._stop.wait(0.6) and token == self._run_token:
            if not self.state.connected:
                continue
            self.send_json({"client": "ping", "runtime": "synrov_dedicated_ai"})
            if time.time() - self.state.last_telemetry_ts > 1.5:
                self.send_json({"sync": True, "requestRobotIdentity": True})

    def _robot_from_block(self, block: Dict[str, Any]) -> Optional[str]:
        for key in self.ROBOT_KEYS:
            if block.get(key):
                return str(block[key])
        control = block.get("control")
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

    def _merge_block(self, block: Dict[str, Any]) -> Tuple[bool, bool]:
        changed = False
        telemetry = False
        robot_raw = self._robot_from_block(block)
        if robot_raw:
            self.state.detected_robot_raw = robot_raw
            self.state.robot = canonical_robot(robot_raw, self.state.robot)
            changed = True

        for key, target in (("sensors", self.state.sensors), ("control", self.state.control), ("gps", self.state.gps)):
            value = block.get(key)
            if isinstance(value, dict):
                target.update(value)
                changed = True
                telemetry = True

        snapshot = block.get("snapshot")
        if isinstance(snapshot, dict):
            self.state.snapshot.update(snapshot)
            changed = True
            telemetry = True

        language = processing_language(block, self.state.language)
        if language and language != self.state.language:
            self.state.language = language
            changed = True
        return changed, telemetry

    def _handle_message(self, message: str) -> None:
        try:
            payload = json.loads(message)
        except (TypeError, json.JSONDecodeError):
            self._emit("log", f"[runtime/ws/raw] {str(message)[:200]}")
            return
        if not isinstance(payload, dict):
            return
        if not state_schema_supported(payload):
            self._emit("log", f"[runtime/ws] ignored state schema={payload.get('schema')!r}")
            return

        changed = False
        got_telemetry = False
        blocks: List[Dict[str, Any]] = [payload]
        for key in ("snapshot", "state"):
            child = payload.get(key)
            if isinstance(child, dict):
                blocks.append(child)
                system = child.get("system")
                if isinstance(system, dict):
                    blocks.append(system)

        for block in blocks:
            block_changed, block_telemetry = self._merge_block(block)
            changed = changed or block_changed
            got_telemetry = got_telemetry or block_telemetry

        if "servo" in payload and "angle" in payload:
            try:
                servo = int(float(payload["servo"]))
                angle = float(payload["angle"])
            except (TypeError, ValueError, OverflowError):
                servo = -1
                angle = 0.0
            if servo >= 0:
                self.state.servos[servo] = angle
                got_telemetry = True
                changed = True

        if got_telemetry:
            self.state.telemetry_counter += 1
            self.state.last_telemetry_ts = time.time()

        frame_payload = payload.get("frame") or payload.get("dataUrl") or payload.get("image")
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
            encoded = re.sub(r"\s+", "", text)
            if len(encoded) > self._max_frame_bytes * 2:
                raise ValueError("frame payload too large")
            raw = base64.b64decode(encoded, validate=False)
            if len(raw) > self._max_frame_bytes:
                raise ValueError("decoded frame too large")
            image = Image.open(io.BytesIO(raw)).convert("RGB")
        except (ValueError, TypeError, OSError) as exc:
            self._emit("log", f"[runtime/frame] decode error: {exc}")
            return
        self.state.last_frame_pil = image
        self.state.last_frame_ts = time.time()
        self.state.frame_counter += 1
        self._emit("frame", image)

    def _with_metadata(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        self._send_seq += 1
        return control_envelope(
            payload,
            source="synrov_dedicated_ai",
            seq=self._send_seq,
            robot=self.state.robot,
        )

    def connect(self, uri: str = DEFAULT_URI) -> None:
        if websocket is None:
            raise RuntimeError("websocket-client is not installed")
        self.disconnect()
        self.uri = str(uri or DEFAULT_URI).strip() or DEFAULT_URI
        self._stop.clear()
        self._run_token += 1
        token = self._run_token
        self._ensure_control_sender(token)
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
            except _WS_ERRORS as exc:
                self._emit("log", f"[runtime/ws] failed: {exc}")

        self._runner_thread = threading.Thread(target=runner, daemon=True, name="synrov-ws-runner")
        self._runner_thread.start()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            args=(token,),
            daemon=True,
            name="synrov-ws-heartbeat",
        )
        self._heartbeat_thread.start()

    def disconnect(self) -> None:
        if self.state.connected and self._ws_app is not None:
            self.send_json({"client": "disconnect", "runtime": "synrov_dedicated_ai"})
        self._run_token += 1
        self._stop.set()
        while True:
            try:
                self._control_queue.get_nowait()
            except queue.Empty:
                break
        ws_app = self._ws_app
        self._ws_app = None
        if ws_app is not None:
            try:
                ws_app.close()
            except _WS_ERRORS:
                pass
        self.state.connected = False
        self.state.status = "disconnected"

    def send_json(self, payload: Dict[str, Any]) -> bool:
        ws_app = self._ws_app
        if ws_app is None:
            return False
        data = json.dumps(self._with_metadata(payload), ensure_ascii=False)
        with self._lock:
            try:
                ws_app.send(data)
            except _WS_ERRORS as exc:
                self.state.connected = False
                self.state.status = "send_error"
                self._emit("log", f"[runtime/ws] send failed: {exc}")
                return False
        return True

    def send_control(self, payload: Dict[str, Any], *, immediate: Optional[bool] = None) -> None:
        """Send robot control, coalescing only continuous motion packets."""
        out = dict(payload or {})
        self.state.last_runtime = json.dumps(out, ensure_ascii=False)
        if immediate is None:
            immediate = any(key in out for key in ("action", "mode", "toggle", "connect", "joystickType", "camera"))
        if immediate:
            self._send_control_now(out)
            return
        while self._control_queue.full():
            try:
                self._control_queue.get_nowait()
            except queue.Empty:
                break
        try:
            self._control_queue.put_nowait(out)
        except queue.Full:
            # A concurrent sender may have refilled the queue; newest intent is
            # already represented by another packet, so dropping here is safe.
            return

    def command_manipulator_pose(
        self,
        pose: Dict[str, Any],
        *,
        neutral: Optional[List[bool]] = None,
        duty: Optional[List[int]] = None,
    ) -> str:
        safe_pose = {
            key: clamp((pose or {}).get(key, 50.0 if key == "grip" else 180.0), *MANIP_POSE_LIMITS[key])
            for key in MANIP_KEYS
        }
        angles = [int(round(safe_pose[key])) for key in MANIP_KEYS]
        manipulator: Dict[str, Any] = {"angles": angles}
        if neutral is not None:
            values = [bool(v) for v in list(neutral)[:7]]
            manipulator["neutral"] = values + [False] * (7 - len(values))
        if duty is not None:
            values = [int(round(clamp(v, 0, 100))) for v in list(duty)[:4]]
            manipulator["duty"] = values + [0] * (4 - len(values))
        self.state.servos.update({index: float(value) for index, value in enumerate(angles)})
        self.send_control({"type": "control_intent", "control": {"robot": "Manipulator", "manipulator": manipulator}})
        return "CONTROL:Manipulator"

    def set_manipulator_auto_torque(self, enabled: bool) -> None:
        self.send_control({
            "type": "control_intent",
            "control": {"robot": "Manipulator", "manipulator": {"autoTorque": bool(enabled)}},
        }, immediate=True)

    def manip_home(self) -> None:
        self.state.servos.update({0: 180.0, 1: 150.0, 2: 70.0, 3: 90.0, 4: 95.0, 5: 130.0, 6: 0.0})
        self.send_control({"action": "home", "robot": "Manipulator"}, immediate=True)

    def _merge_control_state(self, robot: str, section: str, values: Dict[str, Any]) -> None:
        self.state.control["robot"] = robot
        current = self.state.control.get(section)
        if not isinstance(current, dict):
            current = {}
            self.state.control[section] = current
        current.update(values)

    def _merge_camera_state(self, **values: Any) -> None:
        camera = self.state.control.get("camera")
        if not isinstance(camera, dict):
            camera = {}
            self.state.control["camera"] = camera
        camera.update({key: value for key, value in values.items() if value is not None})

    def send_vehicle(
        self,
        throttle: float,
        steer: float,
        cam_pan: Optional[float] = None,
        cam_tilt: Optional[float] = None,
        *,
        pivot: Optional[float] = None,
        lights: Optional[bool] = None,
        lidar_scan: Optional[bool] = None,
    ) -> str:
        drive: Dict[str, Any] = {
            "throttle": clamp(throttle, -1, 1),
            "steer": clamp(steer, -1, 1),
        }
        if cam_pan is not None:
            drive["camPan"] = clamp(cam_pan, -90, 90)
        if cam_tilt is not None:
            drive["camTilt"] = clamp(cam_tilt, -45, 45)
        if pivot is not None:
            drive["pivot"] = clamp(pivot, -1, 1)
        if lights is not None:
            drive["lights"] = bool(lights)
        if lidar_scan is not None:
            drive["lidarScan"] = bool(lidar_scan)
        self._merge_control_state("Vehicle", "drive", drive)
        camera_values: Dict[str, Any] = {}
        if cam_pan is not None:
            camera_values["pan"] = drive["camPan"]
        if cam_tilt is not None:
            camera_values["tilt"] = drive["camTilt"]
        if camera_values:
            self._merge_camera_state(**camera_values)
        if lights is not None:
            self.state.control["lights"] = bool(lights)
        if lidar_scan is not None:
            self.state.control["lidarScan"] = bool(lidar_scan)
        self.send_control({"type": "control_intent", "control": {"robot": "Vehicle", "drive": drive}})
        return "CONTROL:Vehicle"

    def set_vehicle_lights(self, enabled: bool) -> None:
        value = bool(enabled)
        self.state.control["robot"] = "Vehicle"
        self.state.control["lights"] = value
        self._merge_control_state("Vehicle", "drive", {"lights": value})
        self.send_control({"control": {"robot": "Vehicle", "drive": {"lights": value}}}, immediate=True)

    def set_vehicle_lidar_scan(self, enabled: bool) -> None:
        value = bool(enabled)
        self.state.control["robot"] = "Vehicle"
        self.state.control["lidarScan"] = value
        self._merge_control_state("Vehicle", "drive", {"lidarScan": value})
        self.send_control({"control": {"robot": "Vehicle", "drive": {"lidarScan": value}}}, immediate=True)

    def send_drone(
        self,
        throttle: float,
        yaw: float,
        pitch: float,
        roll: float,
        strafe: float,
        forward: float,
        *,
        cam_pan: Optional[float] = None,
        cam_tilt: Optional[float] = None,
        camera_streaming: Optional[bool] = None,
    ) -> str:
        flight: Dict[str, Any] = {
            "throttle": clamp(throttle, -1, 1),
            "yaw": clamp(yaw, -1, 1),
            "pitch": clamp(pitch, -1, 1),
            "roll": clamp(roll, -1, 1),
            "strafe": clamp(strafe, -1, 1),
            "forward": clamp(forward, -1, 1),
        }
        if cam_pan is not None:
            flight["camPan"] = clamp(cam_pan, -90, 90)
        if cam_tilt is not None:
            flight["camTilt"] = clamp(cam_tilt, -90, 90)
        if camera_streaming is not None:
            flight["cameraStreaming"] = bool(camera_streaming)
        self._merge_control_state("Drone", "flight", flight)
        camera_values: Dict[str, Any] = {}
        if cam_pan is not None:
            camera_values["pan"] = flight["camPan"]
        if cam_tilt is not None:
            camera_values["tilt"] = flight["camTilt"]
        if camera_streaming is not None:
            camera_values["streaming"] = bool(camera_streaming)
        if camera_values:
            self._merge_camera_state(**camera_values)
        self.send_control({"type": "control_intent", "control": {"robot": "Drone", "flight": flight}})
        return "CONTROL:Drone"

    def drone_takeoff(self) -> None:
        self.send_control({"action": "takeoff", "robot": "Drone"}, immediate=True)

    def drone_land(self) -> None:
        self.send_control({"action": "land", "robot": "Drone"}, immediate=True)

    def set_drone_camera_streaming(self, enabled: bool) -> None:
        value = bool(enabled)
        self.state.control["robot"] = "Drone"
        self._merge_control_state("Drone", "flight", {"cameraStreaming": value})
        self._merge_camera_state(streaming=value)
        self.send_control({"control": {"robot": "Drone", "flight": {"cameraStreaming": value}}}, immediate=True)

    def set_camera(self, robot: Any, *, pan: Optional[float] = None, tilt: Optional[float] = None, stream: Optional[bool] = None) -> None:
        name = canonical_robot(robot)
        if name == "Manipulator":
            return
        self.state.control["robot"] = name
        if name == "Vehicle":
            drive: Dict[str, Any] = {}
            camera_values: Dict[str, Any] = {}
            if pan is not None:
                drive["camPan"] = clamp(pan, -90, 90)
                camera_values["pan"] = drive["camPan"]
            if tilt is not None:
                drive["camTilt"] = clamp(tilt, -45, 45)
                camera_values["tilt"] = drive["camTilt"]
            if drive:
                self._merge_control_state(name, "drive", drive)
                self._merge_camera_state(**camera_values)
                self.send_control({"control": {"robot": name, "drive": drive}}, immediate=True)
            return
        flight: Dict[str, Any] = {}
        camera_values: Dict[str, Any] = {}
        if pan is not None:
            flight["camPan"] = clamp(pan, -90, 90)
            camera_values["pan"] = flight["camPan"]
        if tilt is not None:
            flight["camTilt"] = clamp(tilt, -90, 90)
            camera_values["tilt"] = flight["camTilt"]
        if stream is not None:
            flight["cameraStreaming"] = bool(stream)
            camera_values["streaming"] = bool(stream)
        if flight:
            self._merge_control_state(name, "flight", flight)
            self._merge_camera_state(**camera_values)
            self.send_control({"control": {"robot": name, "flight": flight}}, immediate=True)

    def set_mode(self, robot: Any) -> None:
        self.send_control({"mode": canonical_robot(robot)}, immediate=True)

    def set_joystick_type(self, robot: Any, joystick_type: int) -> None:
        value = max(1, min(3, int(joystick_type)))
        self.send_control({"joystickType": value, "robot": canonical_robot(robot)}, immediate=True)

    def toggle_processing_feature(self, feature: str) -> None:
        key = str(feature or "").strip().lower()
        if key in {"leap", "joystick", "collision", "trace"}:
            self.send_control({"toggle": key}, immediate=True)

    def play_action(self, index: int) -> None:
        value = int(index)
        if 1 <= value <= 5:
            self.send_control({"action": value}, immediate=True)

    def request_sync(self) -> None:
        self.send_control({"sync": True}, immediate=True)

    def request_hardware_connect(self) -> None:
        self.send_control({"connect": True}, immediate=True)


class SynROVDedicatedRuntime:
    """High-level runtime combining bridge, safety, per-robot AI and arbitration."""

    def __init__(self, bridge: Optional[ModernSynROVBridge] = None) -> None:
        self.bridge = bridge or ModernSynROVBridge()
        self.safety = SynROVSafetyLayer()
        self.orchestrator = SynROVOrchestrator()
        self.registry = RobotAIRegistry(self.safety)
        self.last_result = CommandResult(False, "Manipulator", reason="not_started")

    @property
    def active_robot(self) -> str:
        return canonical_robot(self.bridge.state.robot)

    @property
    def language(self) -> str:
        return self.bridge.state.language

    def execute_text(self, text: str) -> CommandResult:
        result = self.registry.execute_text(text, self.bridge, self.orchestrator)
        self.last_result = result
        self.bridge._emit("command_result", result)
        return result

    def tick(self) -> CommandResult:
        self.orchestrator.begin_cycle(self.active_robot)
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
            "language": self.language,
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
    print("SynROV dedicated AI runtime active. Robot and language come from Processing WebSocket state.")

    try:
        while True:
            while True:
                try:
                    kind, payload = bridge.event_queue.get_nowait()
                except queue.Empty:
                    break
                if kind == "log":
                    print(payload)
                elif kind == "state":
                    print(f"[state] robot={bridge.state.robot} lang={bridge.state.language or '?'} connected={bridge.state.connected}")
            if args.loop:
                runtime.tick()
            time.sleep(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.disconnect()


if __name__ == "__main__":
    main()
