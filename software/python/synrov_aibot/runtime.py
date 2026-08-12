"""Canonical SynROV AiBot runtime.

This is the only control runtime used by the GUI and headless entry point.
Processing and AiBot exchange the shared SynROV application envelope.
"""
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

from .camera_devices import CameraDevice, WebcamSource, discover_camera_devices
from .robot_types import canonical_robot
from .primitives import safe_float
from .orchestrator import SynROVOrchestrator
from .protocol import (
    MESSAGE_STATE, SOFTWARE_VERSION, SOURCE_AIBOT, envelope_supported, infer_message_type,
    message_envelope, processing_language, unpack_message,
)
from .perception_stream import ProcessingPerceptionClient
from .robot_ai import CommandResult, RobotAIRegistry
from .robot_context import refresh_state_intelligence_context
from .robot_camera_bridge import AiBotRobotCameraBridge
from .safety import MANIP_KEYS, MANIP_POSE_LIMITS, SynROVSafetyLayer, clamp
from .camera_contract import (
    CAMERA_PAN_MIN_DEG, CAMERA_PAN_MAX_DEG, CAMERA_TILT_MIN_DEG, CAMERA_TILT_MAX_DEG,
    clamp_pan, clamp_tilt, default_camera_pose,
)
from .media import detect_object_from_pil
from .robot_catalog import audit_catalog, all_specs_for_robot

DEFAULT_URI = "ws://127.0.0.1:9000/"


@dataclass
class SynROVState:
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
    perception_connected: bool = False
    last_perception_ts: float = 0.0
    perception_counter: int = 0
    perception_seq: int = 0
    perception_scene: Dict[str, Any] = field(default_factory=dict)
    perception_runtime: Dict[str, Any] = field(default_factory=dict)
    perception_obstacles: List[Dict[str, Any]] = field(default_factory=list)
    intelligence_context: Dict[str, Any] = field(default_factory=dict)
    sensor_channels: List[Dict[str, Any]] = field(default_factory=list)
    active_input_source: str = "none"
    human_override: bool = False
    robot_camera_available: bool = False
    robot_camera_frame_ts: float = 0.0
    audio_context: Dict[str, Any] = field(default_factory=dict)
    vision_info: Dict[str, Any] = field(default_factory=dict)
    last_runtime: str = ""


class SynROVBridge:
    """Bridge using the same protocol and command shapes as the HTML console."""

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
        self.state = SynROVState()
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
        self._auto_torque_thread: Optional[threading.Thread] = None
        self._auto_torque_baseline: List[Optional[float]] = [None, None, None, None]
        self._auto_torque_filtered: List[float] = [0.0, 0.0, 0.0, 0.0]
        self._auto_torque_last_duty: List[int] = [-1, -1, -1, -1]
        self._auto_torque_active_last = False
        self._max_frame_bytes = 8 * 1024 * 1024
        self._control_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=4)
        self.perception_client = ProcessingPerceptionClient(self.state, self._emit)

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
                out.setdefault("controlSource", "aibot")
            self.send_json(out)

    def _reset_auto_torque_controller(self) -> None:
        self._auto_torque_baseline = [None, None, None, None]
        self._auto_torque_filtered = [0.0, 0.0, 0.0, 0.0]
        self._auto_torque_last_duty = [-1, -1, -1, -1]

    def _auto_torque_enabled(self) -> bool:
        if not self.state.connected or canonical_robot(self.state.robot) != "Manipulator":
            return False
        manipulator = self.state.control.get("manipulator")
        return bool(manipulator.get("autoTorque", False)) if isinstance(manipulator, dict) else False

    def _auto_torque_sensor(self, index: int) -> Optional[float]:
        keys = (
            ("base_current_ma", "base_torque", "base_torque_raw", "an2", "an_2", "an_1"),
            ("upper_current_ma", "upper_torque", "upper_arm_torque_raw", "an3", "an_3", "an_2"),
            ("fore_current_ma", "forearm_torque", "forearm_torque_raw", "an4", "an_4", "an_3"),
            ("forearm_roll_current_ma", "forearm_roll_torque", "forearm_roll_torque_raw", "an5", "an_5", "an_4"),
        )[max(0, min(3, int(index)))]
        for key in keys:
            if key not in self.state.sensors:
                continue
            try:
                return float(self.state.sensors[key])
            except (TypeError, ValueError, OverflowError):
                continue
        return None

    def _auto_torque_command_seed(self) -> List[int]:
        manipulator = self.state.control.get("manipulator")
        if not isinstance(manipulator, dict):
            return [0, 0, 0, 0]
        values = manipulator.get("dutyCommand", [])
        if not isinstance(values, list):
            return [0, 0, 0, 0]
        out = [int(round(clamp(v, 0, 100))) for v in values[:4]]
        return out + [0] * (4 - len(out))

    def _auto_torque_neutral(self) -> List[bool]:
        manipulator = self.state.control.get("manipulator")
        if not isinstance(manipulator, dict):
            return [False, False, False, False]
        values = manipulator.get("neutral", [])
        if not isinstance(values, list):
            return [False, False, False, False]
        out = [bool(v) for v in values[:4]]
        return out + [False] * (4 - len(out))

    def _compute_auto_torque_duty(self) -> Optional[List[int]]:
        # Keep the same control law as Processing so authority can move between
        # hosts without a discontinuous change in behavior.
        min_pct = 12
        deadband = 8.0
        response_range = 420.0
        seed = self._auto_torque_command_seed()
        neutral = self._auto_torque_neutral()
        values = list(seed)
        available = 0
        for index in range(4):
            raw = self._auto_torque_sensor(index)
            if raw is None:
                if neutral[index]:
                    values[index] = 0
                continue
            available += 1
            baseline = self._auto_torque_baseline[index]
            if baseline is None:
                baseline = raw
                self._auto_torque_baseline[index] = raw
                self._auto_torque_filtered[index] = raw
            filtered = self._auto_torque_filtered[index] + (raw - self._auto_torque_filtered[index]) * 0.32
            self._auto_torque_filtered[index] = filtered
            response = abs(filtered - float(baseline))
            if response < deadband:
                self._auto_torque_baseline[index] = float(baseline) + (filtered - float(baseline)) * 0.02
                response = 0.0
            ratio = max(0.0, min(1.0, response / response_range))
            computed = int(round(min_pct + ratio * (100 - min_pct)))
            if neutral[index]:
                computed = 0
            previous = self._auto_torque_last_duty[index]
            if previous >= 0:
                computed = int(round(previous + (computed - previous) * 0.45))
            values[index] = max(0, min(100, computed))
        return values if available else None

    def _auto_torque_loop(self, token: int) -> None:
        while not self._stop.wait(0.09) and token == self._run_token:
            active = self._auto_torque_enabled()
            if active != self._auto_torque_active_last:
                self._auto_torque_active_last = active
                self._reset_auto_torque_controller()
            if not active:
                continue
            # Stale current telemetry must never drive a fresh PWM command.
            if not self.state.last_telemetry_ts or time.time() - self.state.last_telemetry_ts > 1.5:
                continue
            duty = self._compute_auto_torque_duty()
            if duty is None:
                continue
            if duty == self._auto_torque_last_duty:
                continue
            self._auto_torque_last_duty = list(duty)
            self.command_manipulator_duty(duty)

    def _heartbeat_loop(self, token: int) -> None:
        while not self._stop.wait(0.6) and token == self._run_token:
            if not self.state.connected:
                continue
            self.send_json({"client": "ping", "runtime": "synrov_aibot", "visualStream": False})
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
            previous_language = self.state.language
            self.state.language = language
            changed = True
            self._emit("language", {"language": language, "previous": previous_language})
        return changed, telemetry

    def _handle_message(self, message: str) -> None:
        try:
            payload = json.loads(message)
        except (TypeError, json.JSONDecodeError):
            self._emit("log", f"[runtime/ws/raw] {str(message)[:200]}")
            return
        if not isinstance(payload, dict) or not envelope_supported(payload, {MESSAGE_STATE}):
            self._emit("log", "[runtime/ws] ignored invalid SynROV state envelope")
            return
        _, state_payload = unpack_message(payload, {MESSAGE_STATE})
        state_payload = dict(state_payload)

        changed = False
        got_telemetry = False
        blocks: List[Dict[str, Any]] = [state_payload]
        for key in ("snapshot", "state"):
            child = state_payload.get(key)
            if isinstance(child, dict):
                blocks.append(child)
                system = child.get("system")
                if isinstance(system, dict):
                    blocks.append(system)

        for block in blocks:
            block_changed, block_telemetry = self._merge_block(block)
            changed = changed or block_changed
            got_telemetry = got_telemetry or block_telemetry

        if "servo" in state_payload and "angle" in state_payload:
            try:
                servo = int(float(state_payload["servo"]))
                angle = float(state_payload["angle"])
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
            refresh_state_intelligence_context(self.state, now=self.state.last_telemetry_ts)

        frame_payload = state_payload.get("frame") or state_payload.get("dataUrl") or state_payload.get("image")
        if frame_payload and not self.perception_client.connected:
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
        return message_envelope(
            infer_message_type(payload),
            payload,
            source=SOURCE_AIBOT,
            seq=self._send_seq,
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
        self._auto_torque_active_last = False
        self._reset_auto_torque_controller()
        self._auto_torque_thread = threading.Thread(
            target=self._auto_torque_loop,
            args=(token,),
            daemon=True,
            name="synrov-aibot-auto-torque",
        )
        self._auto_torque_thread.start()
        self.state.status = "connecting"
        self._emit("state", self.state)

        def on_open(ws: Any) -> None:
            self.state.connected = True
            self.state.status = "connected"
            now = time.time()
            self.state.last_msg_ts = now
            self.state.last_telemetry_ts = now
            self.send_json({"client": "connect", "runtime": "synrov_aibot", "visualStream": False})
            self.send_json({"sync": True, "requestRobotIdentity": True})
            self.perception_client.connect(self.uri)
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
            self.send_json({"client": "disconnect", "runtime": "synrov_aibot", "visualStream": False})
        self._run_token += 1
        self._stop.set()
        self._auto_torque_active_last = False
        self._reset_auto_torque_controller()
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
        self.perception_client.disconnect()
        self.state.connected = False
        self.state.status = "disconnected"
        self.state.perception_connected = False

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

    def command_manipulator_duty(self, duty: List[int]) -> str:
        values = [int(round(clamp(v, 0, 100))) for v in list(duty)[:4]]
        values += [0] * (4 - len(values))
        self.send_control({
            "type": "control_intent",
            "control": {"robot": "Manipulator", "manipulator": {"duty": values}},
        }, immediate=True)
        return "CONTROL:Manipulator:AutoTorque"

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
            drive["camPan"] = clamp_pan(cam_pan)
        if cam_tilt is not None:
            drive["camTilt"] = clamp_tilt(cam_tilt)
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

    def drone_flight_status(self) -> Dict[str, Any]:
        flight = self.state.control.get("flight", {}) if isinstance(self.state.control, dict) else {}
        if not isinstance(flight, dict):
            flight = {}
        phase = str(flight.get("phase", "") or "").strip().lower()
        if "flightReady" in flight:
            ready = bool(flight.get("flightReady"))
        else:
            altitude = 0.0
            for key in ("alt_cm", "altitude_cm", "sonar_down_cm"):
                if key in self.state.sensors:
                    altitude = max(altitude, safe_float(self.state.sensors.get(key), 0.0))
            ready = altitude >= 24.0 or phase == "airborne"
        airborne = bool(flight.get("airborne", ready))
        if not phase:
            phase = "airborne" if ready else "taking_off" if airborne else "grounded"
        return {"phase": phase, "flight_ready": ready, "airborne": airborne}

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
        status = self.drone_flight_status()
        bounded_throttle = clamp(throttle, -1, 1)
        bounded_yaw = clamp(yaw, -1, 1)
        bounded_pitch = clamp(pitch, -1, 1)
        bounded_roll = clamp(roll, -1, 1)
        bounded_strafe = clamp(strafe, -1, 1)
        bounded_forward = clamp(forward, -1, 1)
        filtered_by_flight_state = False
        if not status["flight_ready"]:
            if bounded_throttle < 0.0 and not status["airborne"]:
                bounded_throttle = 0.0
            if any(abs(value) > 1e-6 for value in (bounded_yaw, bounded_pitch, bounded_roll, bounded_strafe, bounded_forward)):
                filtered_by_flight_state = True
            bounded_yaw = bounded_pitch = bounded_roll = bounded_strafe = bounded_forward = 0.0

        flight: Dict[str, Any] = {
            "throttle": bounded_throttle,
            "yaw": bounded_yaw,
            "pitch": bounded_pitch,
            "roll": bounded_roll,
            "strafe": bounded_strafe,
            "forward": bounded_forward,
        }
        if filtered_by_flight_state:
            flight["flightStateFiltered"] = True
        if cam_pan is not None:
            flight["camPan"] = clamp_pan(cam_pan)
        if cam_tilt is not None:
            flight["camTilt"] = clamp_tilt(cam_tilt)
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
        status = self.drone_flight_status()
        if not status["flight_ready"]:
            self._merge_control_state("Drone", "flight", {
                "airborne": bool(status["airborne"]), "flightReady": False, "phase": "taking_off",
                "yaw": 0.0, "pitch": 0.0, "roll": 0.0, "strafe": 0.0, "forward": 0.0,
            })
        self.send_control({"action": "takeoff", "robot": "Drone"}, immediate=True)

    def drone_land(self) -> None:
        self._merge_control_state("Drone", "flight", {
            "flightReady": False, "phase": "landing",
            "yaw": 0.0, "pitch": 0.0, "roll": 0.0, "strafe": 0.0, "forward": 0.0,
        })
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
                drive["camPan"] = clamp_pan(pan)
                camera_values["pan"] = drive["camPan"]
            if tilt is not None:
                drive["camTilt"] = clamp_tilt(tilt)
                camera_values["tilt"] = drive["camTilt"]
            if drive:
                self._merge_control_state(name, "drive", drive)
                self._merge_camera_state(**camera_values)
                self.send_control({"control": {"robot": name, "drive": drive}}, immediate=True)
            return
        flight: Dict[str, Any] = {}
        camera_values: Dict[str, Any] = {}
        if pan is not None:
            flight["camPan"] = clamp_pan(pan)
            camera_values["pan"] = flight["camPan"]
        if tilt is not None:
            flight["camTilt"] = clamp_tilt(tilt)
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


class SynROVRuntime:
    """Single high-level AiBot runtime used by GUI and headless mode."""

    def __init__(self, bridge: Optional[SynROVBridge] = None, camera_index: Optional[int] = None) -> None:
        self.bridge = bridge or SynROVBridge()
        self.safety = SynROVSafetyLayer()
        self.orchestrator = SynROVOrchestrator()
        self.registry = RobotAIRegistry(self.safety)
        self.camera_bridge = AiBotRobotCameraBridge(width=640, height=360, jpeg_quality=72)
        self.camera_bridge.start()
        self.webcam = WebcamSource()
        self.camera_devices: List[CameraDevice] = []
        self.camera_index = WebcamSource.AUTO_INDEX if camera_index is None else int(camera_index)
        self.adaptive_enabled = True
        self.music_enabled = False
        self.vision_enabled = True
        self.webcam_enabled = camera_index is not None
        # Internal safeguards. They intentionally have no user checkbox.
        self.shadow_validation = True
        self.learning_protection = True
        self.last_result = CommandResult(False, "Manipulator", reason="not_started")
        if self.webcam_enabled:
            self.refresh_camera_devices()
            self.set_webcam(True, self.camera_index)

    @property
    def active_robot(self) -> str:
        return canonical_robot(self.bridge.state.robot)

    @property
    def language(self) -> str:
        return self.bridge.state.language

    def refresh_camera_devices(self) -> List[CameraDevice]:
        self.camera_devices = discover_camera_devices(max_index=12)
        return list(self.camera_devices)

    def set_adaptive(self, enabled: bool) -> None:
        self.adaptive_enabled = bool(enabled)
        if not self.adaptive_enabled:
            self.registry.stop_active(self.bridge, self.orchestrator)

    def set_music(self, enabled: bool) -> None:
        # Music actuation is intentionally limited to Manipulator.
        self.music_enabled = bool(enabled) and self.active_robot == "Manipulator"

    def set_vision(self, enabled: bool) -> None:
        self.vision_enabled = bool(enabled)
        if not self.vision_enabled:
            self.bridge.state.vision_info = {}

    def set_webcam(self, enabled: bool, index: Optional[int] = None) -> Optional[CameraDevice]:
        self.webcam_enabled = bool(enabled)
        if index is not None:
            self.camera_index = int(index)
        if not self.webcam_enabled:
            self.webcam.stop()
            return None
        if not self.camera_devices:
            self.refresh_camera_devices()
        # If a selected USB index disappeared, use Auto instead of a stale id.
        available = {device.index for device in self.camera_devices}
        if self.camera_index >= 0 and self.camera_index not in available:
            self.camera_index = WebcamSource.AUTO_INDEX
        try:
            return self.webcam.start(self.camera_index, self.camera_devices)
        except Exception as exc:
            self.webcam.stop()
            self.bridge._emit("log", f"[camera] {exc}")
            return None

    def command_specs(self, robot: Optional[Any] = None):
        return all_specs_for_robot(robot or self.active_robot)

    def execute_text(self, text: str) -> CommandResult:
        ai = self.registry.get(self.active_robot)
        plan = ai.resolve_plan(text)
        if not plan:
            result = CommandResult(False, self.active_robot, reason="command_not_understood_with_confidence")
        elif any(name == "rhythm_mode" for _, name, _ in plan) and not self.music_enabled:
            result = CommandResult(False, self.active_robot, "mission", "rhythm_mode", "music_disabled")
        else:
            result = self.registry.execute_text(text, self.bridge, self.orchestrator)
        self.last_result = result
        self.bridge._emit("command_result", result)
        return result

    def tick(self) -> CommandResult:
        physical_image = self.webcam.update() if self.webcam_enabled and self.webcam.enabled else None

        # Port 9002 is independent from the physical camera. A Processing frame
        # or generated frame remains the camera source when webcam is disabled.
        if self.active_robot in {"Vehicle", "Drone"}:
            self.camera_bridge.submit(
                control_uri=self.bridge.uri,
                robot=self.active_robot,
                physical_image=physical_image,
                fallback_image=self.bridge.state.last_frame_pil,
            )

        if self.vision_enabled:
            source_image = physical_image or self.bridge.state.last_frame_pil
            if source_image is not None:
                info = detect_object_from_pil(source_image)
                self.bridge.state.vision_info = {
                    "confidence": info.confidence, "dx": info.dx, "dy": info.dy,
                    "area": info.area, "width": info.width, "height": info.height,
                    "centered": bool(info.centered >= 0.55), "close": bool(info.close >= 0.55),
                    "source": "physical_webcam" if physical_image is not None else "processing_frame",
                }

        self.orchestrator.begin_cycle(self.active_robot)
        context = refresh_state_intelligence_context(self.bridge.state)
        self.orchestrator.sync_input_context(self.active_robot, context)
        if not self.adaptive_enabled:
            return CommandResult(False, self.active_robot, reason="adaptive_disabled")

        result = self.registry.tick_active(self.bridge, self.orchestrator)
        if result.ok:
            self.last_result = result
            self.bridge._emit("mission_tick", result)
        return result

    def stop_active(self) -> None:
        self.registry.stop_active(self.bridge, self.orchestrator)

    def learn_alias(self, phrase: str, kind: str, intent: str) -> bool:
        return self.registry.learn_alias(self.active_robot, phrase, kind, intent)

    def status(self) -> Dict[str, Any]:
        intelligence = refresh_state_intelligence_context(self.bridge.state)
        return {
            "softwareVersion": SOFTWARE_VERSION,
            "active_robot": self.active_robot,
            "detected_robot_raw": self.bridge.state.detected_robot_raw,
            "language": self.language,
            "connected": self.bridge.state.connected,
            "features": {
                "adaptive": self.adaptive_enabled,
                "music": self.music_enabled,
                "vision": self.vision_enabled,
                "webcam": self.webcam_enabled,
                "shadow_validation": self.shadow_validation,
                "learning_protection": self.learning_protection,
            },
            "camera": {
                "selected_index": self.camera_index,
                "active_index": self.webcam.index if self.webcam.enabled else None,
                "backend": self.webcam.backend_name,
                "pan_min_deg": CAMERA_PAN_MIN_DEG,
                "pan_max_deg": CAMERA_PAN_MAX_DEG,
                "tilt_min_deg": CAMERA_TILT_MIN_DEG,
                "tilt_max_deg": CAMERA_TILT_MAX_DEG,
                "default": default_camera_pose(self.active_robot).__dict__,
            },
            "catalog": audit_catalog().get(self.active_robot, {}),
            "models": self.registry.metadata(),
            "intelligence": intelligence,
            "orchestrator": self.orchestrator.health(),
            "robot_camera_9002": self.camera_bridge.status(self.active_robot),
            "last_result": self.last_result.__dict__,
        }

    def close(self) -> None:
        self.webcam.stop()
        self.camera_bridge.stop()


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="SynROV AiBot Version 1 runtime")
    parser.add_argument("--uri", default=DEFAULT_URI, help="SynROV WebSocket URI")
    parser.add_argument("--loop", action="store_true", help="Keep the mission tick loop active")
    parser.add_argument("--camera", default="off", help="Physical camera: off, auto, or a numeric index")
    args = parser.parse_args(argv)

    camera_arg = str(args.camera or "off").strip().lower()
    camera_index: Optional[int]
    if camera_arg in {"", "off", "none", "disabled"}:
        camera_index = None
    elif camera_arg in {"auto", "automatic"}:
        camera_index = WebcamSource.AUTO_INDEX
    else:
        try:
            camera_index = max(0, int(camera_arg))
        except ValueError as exc:
            raise SystemExit("--camera must be off, auto, or a non-negative camera index") from exc

    bridge = SynROVBridge()
    runtime = SynROVRuntime(bridge, camera_index=camera_index)
    bridge.connect(args.uri)
    print("SynROV AiBot runtime active. Robot and language are synchronized with Processing.")

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
        runtime.close()
        bridge.disconnect()


if __name__ == "__main__":
    main()
