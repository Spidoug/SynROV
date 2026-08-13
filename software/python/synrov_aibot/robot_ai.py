"""Dedicated robot AIs for Manipulator, Vehicle and Drone.

Each robot owns its command vocabulary, mission state, model/dataset paths and
arbitration tuning.  Robot identity is supplied by Processing over WebSocket;
no Python-side selector is used by this backend.
"""
from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Tuple
from types import SimpleNamespace

from .adaptive_policy import AdaptiveMissionPolicy
from .autonomy import adapt_drone_motion, adapt_vehicle_motion, command_tuning, obstacle_distance_cm
from .robot_types import TARGET_DIMS, canonical_robot
from .dance import AdaptiveDanceModel
from .orchestrator import SynROVOrchestrator
from .robot_context import profile_for_robot, refresh_state_intelligence_context
from .robot_strategy import (
    drone_exit_vector,
    manipulator_joint_torque_references,
    mission_resource_status,
    strategy_for_robot,
    vehicle_exit_steer,
)
from .primitives import normalize_text, safe_float
from .robot_catalog import aliases_for_robot
from .camera_contract import default_camera_pose
from .cognitive import LongContextMemory, contextual_resolve, split_explicit_plan
from .protocol import COMMANDS_SCHEMA, SOFTWARE_VERSION
from .safety import MANIP_KEYS, MANIP_POSE_LIMITS, MANIPULATOR_HOME_POSE, SynROVSafetyLayer, clamp


def _vision_target(state: Any) -> Dict[str, Any]:
    """Return the latest physical-camera object observation when available."""
    raw = getattr(state, "vision_info", None)
    if raw is None:
        raw = getattr(state, "last_vision", None)
    if isinstance(raw, Mapping):
        get = raw.get
    elif raw is not None:
        get = lambda key, default=None: getattr(raw, key, default)
    else:
        return {"available": False, "confidence": 0.0, "dx": 0.0, "dy": 0.0, "area": 0.0, "centered": False, "close": False, "label": "none"}
    confidence = safe_float(get("confidence", 0.0), 0.0)
    return {
        "available": confidence > 0.0,
        "confidence": confidence,
        "dx": safe_float(get("dx", 0.0), 0.0),
        "dy": safe_float(get("dy", 0.0), 0.0),
        "area": safe_float(get("area", 0.0), 0.0),
        "centered": bool(get("centered", False)),
        "close": bool(get("close", False)),
        "label": str(get("label", "object") or "object"),
    }


def _angle_error_deg(target_deg: float, current_deg: float) -> float:
    return (float(target_deg) - float(current_deg) + 180.0) % 360.0 - 180.0


def _local_navigation_pose(state: Any, robot: Any) -> Optional[Dict[str, Any]]:
    """Return Processing-local x/z/yaw for safe origin navigation.

    The perception stream publishes metric local pose. The normal state channel
    also publishes scene coordinates, so return-home keeps working when the
    perception stream is temporarily unavailable. Both coordinate systems share
    the same Processing origin (the robot's home at runtime reset).
    """
    canonical = canonical_robot(robot)
    if canonical not in {"Vehicle", "Drone"}:
        return None
    scene = getattr(state, "perception_scene", {}) or {}
    pose = scene.get("pose", {}) if isinstance(scene, Mapping) else {}
    if isinstance(pose, Mapping) and "x_m" in pose and "z_m" in pose:
        return {
            "x": safe_float(pose.get("x_m"), 0.0),
            "z": safe_float(pose.get("z_m"), 0.0),
            "yaw_deg": safe_float(pose.get("yaw_deg"), 0.0),
            "units": "meters",
        }
    sensors = getattr(state, "sensors", {}) or {}
    x_key, z_key, yaw_key = (
        ("vehicle_x", "vehicle_z", "vehicle_yaw_deg")
        if canonical == "Vehicle" else
        ("drone_x", "drone_z", "drone_yaw_deg")
    )
    if isinstance(sensors, Mapping) and (x_key in sensors or z_key in sensors):
        return {
            "x": safe_float(sensors.get(x_key), 0.0),
            "z": safe_float(sensors.get(z_key), 0.0),
            "yaw_deg": safe_float(sensors.get(yaw_key, sensors.get("heading_deg", 0.0)), 0.0),
            "units": "scene",
        }
    return None


def match_alias(text: Any, aliases: Iterable[str]) -> int:
    query = normalize_text(text)
    best = 0
    for alias in aliases or ():
        normalized = normalize_text(alias)
        if not normalized:
            continue
        if query == normalized:
            best = max(best, 1000 + len(normalized))
        elif normalized in query:
            best = max(best, 100 + len(normalized))
    return best


@dataclass(frozen=True)
class RobotModelSlot:
    robot: str
    target_dim: int
    model_dir: Path
    policy_path: Path
    dataset_path: Path

    @classmethod
    def build(cls, robot: Any, root: Optional[Path] = None) -> "RobotModelSlot":
        name = canonical_robot(robot)
        base = Path(root or Path(__file__).resolve().parent / "runtime_data")
        key = name.lower()
        model_dir = base / "models" / key
        dataset_path = base / "datasets" / f"{key}_training.jsonl"
        model_dir.mkdir(parents=True, exist_ok=True)
        dataset_path.parent.mkdir(parents=True, exist_ok=True)
        return cls(
            robot=name,
            target_dim=int(TARGET_DIMS.get(name, 7)),
            model_dir=model_dir,
            policy_path=model_dir / f"{key}_policy.pkl",
            dataset_path=dataset_path,
        )

    def health(self) -> Dict[str, Any]:
        return {
            "robot": self.robot,
            "target_dim": self.target_dim,
            "model_dir": str(self.model_dir),
            "policy_path": str(self.policy_path),
            "policy_exists": self.policy_path.is_file(),
            "dataset_path": str(self.dataset_path),
            "dataset_exists": self.dataset_path.is_file(),
        }


@dataclass(frozen=True)
class RobotProfile:
    robot: str
    target_dim: int
    music_enabled: bool
    intents: Mapping[str, Tuple[str, ...]]
    missions: Mapping[str, Tuple[str, ...]]


@dataclass
class MissionState:
    name: str
    created_ts: float = field(default_factory=time.time)
    text: str = ""
    data: Dict[str, Any] = field(default_factory=dict)


class LearnedCommandMemory:
    """Persistent teachable aliases, isolated by canonical robot name."""

    def __init__(self, path: Optional[Path] = None) -> None:
        self.path = Path(
            path
            or Path(__file__).resolve().parent / "runtime_data" / "learned_commands.json"
        )
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._data: Dict[str, Dict[str, Dict[str, str]]] = {}
        self.load()

    def load(self) -> None:
        raw: Any = {}
        if self.path.is_file():
            try:
                raw = json.loads(self.path.read_text(encoding="utf-8"))
            except (OSError, UnicodeError, json.JSONDecodeError):
                raw = {}

        clean: Dict[str, Dict[str, Dict[str, str]]] = {}
        payload = raw.get("robots", {}) if isinstance(raw, dict) else {}
        if (
            isinstance(raw, dict)
            and raw.get("schema") == COMMANDS_SCHEMA
            and raw.get("softwareVersion") == SOFTWARE_VERSION
            and isinstance(payload, dict)
        ):
            for robot, entries in payload.items():
                canonical = canonical_robot(robot)
                if not isinstance(entries, dict):
                    continue
                robot_entries = clean.setdefault(canonical, {})
                for phrase, item in entries.items():
                    if not isinstance(item, dict):
                        continue
                    kind = str(item.get("kind", "")).strip().lower()
                    name = str(item.get("name", "")).strip()
                    phrase_key = normalize_text(phrase)
                    if phrase_key and kind in {"intent", "mission"} and name:
                        robot_entries[phrase_key] = {"kind": kind, "name": name}
        self._data = clean

    def save(self) -> bool:
        try:
            payload = {"schema": COMMANDS_SCHEMA, "softwareVersion": SOFTWARE_VERSION, "robots": self._data}
            self.path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            return True
        except (OSError, UnicodeError):
            return False

    def learn(self, robot: Any, phrase: Any, kind: str, name: str) -> bool:
        canonical = canonical_robot(robot)
        phrase_key = normalize_text(phrase)
        kind_key = str(kind or "").strip().lower()
        command_name = str(name or "").strip()
        if not phrase_key or kind_key not in {"intent", "mission"} or not command_name:
            return False
        self._data.setdefault(canonical, {})[phrase_key] = {"kind": kind_key, "name": command_name}
        return self.save()

    def forget(self, robot: Any, phrase: Any) -> bool:
        canonical = canonical_robot(robot)
        phrase_key = normalize_text(phrase)
        entries = self._data.get(canonical, {})
        if not phrase_key or phrase_key not in entries:
            return False
        del entries[phrase_key]
        return self.save()

    def resolve(self, robot: Any, text: Any) -> Tuple[str, str]:
        canonical = canonical_robot(robot)
        query = normalize_text(text)
        best_score = 0
        best_kind = "none"
        best_name = ""
        for phrase, item in self._data.get(canonical, {}).items():
            if query == phrase:
                score = 2000 + len(phrase)
            elif phrase and (phrase in query or query in phrase):
                score = 300 + min(len(phrase), len(query))
            else:
                continue
            if score > best_score:
                best_score = score
                best_kind = str(item.get("kind", "none"))
                best_name = str(item.get("name", ""))
        return (best_kind, best_name) if best_score else ("none", "")

    def list_for_robot(self, robot: Any) -> Dict[str, Dict[str, str]]:
        return dict(self._data.get(canonical_robot(robot), {}))

    def all(self) -> Dict[str, Dict[str, Dict[str, str]]]:
        return {robot: dict(entries) for robot, entries in self._data.items()}


@dataclass(frozen=True)
class CommandResult:
    ok: bool
    robot: str
    kind: str = "none"
    name: str = ""
    reason: str = ""
    metadata: Dict[str, Any] = field(default_factory=dict)


def manip_pose(state: Any) -> Dict[str, float]:
    sensors = getattr(state, "sensors", {}) or {}
    servos = getattr(state, "servos", {}) or {}
    defaults = dict(MANIPULATOR_HOME_POSE)
    sensor_names = {
        "base": "base_deg",
        "upper": "upper_deg",
        "fore": "fore_deg",
        "forearm_roll": "forearm_roll_deg",
        "wrist_pitch": "wrist_pitch_deg",
        "wrist_rot": "wrist_rot_deg",
        "grip": "grip_deg",
    }
    return {
        key: clamp(servos.get(index, sensors.get(sensor_names[key], defaults[key])), *MANIP_POSE_LIMITS[key])
        for index, key in enumerate(MANIP_KEYS)
    }


def _profile(robot: str, target_dim: int, music_enabled: bool) -> RobotProfile:
    return RobotProfile(
        robot=robot,
        target_dim=target_dim,
        music_enabled=music_enabled,
        intents=aliases_for_robot(robot, "intent"),
        missions=aliases_for_robot(robot, "mission"),
    )


MANIPULATOR_PROFILE = _profile("Manipulator", 7, True)
VEHICLE_PROFILE = _profile("Vehicle", 4, False)
DRONE_PROFILE = _profile("Drone", 6, False)


class DedicatedRobotAI:
    """Common lifecycle; actuator/mission implementations live in subclasses."""

    def __init__(
        self,
        profile: RobotProfile,
        safety: SynROVSafetyLayer,
        *,
        model_root: Optional[Path] = None,
        memory: Optional[LearnedCommandMemory] = None,
        context_memory: Optional[LongContextMemory] = None,
    ) -> None:
        self.profile = profile
        self.safety = safety
        self.memory = memory
        self.context_memory = context_memory
        self.model_slot = RobotModelSlot.build(profile.robot, model_root)
        self.policy = AdaptiveMissionPolicy(profile.robot, self.model_slot.policy_path, self.model_slot.dataset_path)
        self.active_mission: Optional[MissionState] = None
        self.last_command_source = ""
        self.last_command_name = ""
        self.last_command_ts = 0.0
        self.pending_plan: list[Tuple[str, str, str]] = []
        self.plan_not_before_ts = 0.0

    @property
    def robot(self) -> str:
        return self.profile.robot

    def metadata(self) -> Dict[str, Any]:
        intelligence = profile_for_robot(self.robot).metadata()
        return {
            **self.model_slot.health(),
            "music_enabled": self.profile.music_enabled,
            "learned_commands": len(self.memory.list_for_robot(self.robot)) if self.memory else 0,
            "long_context": self.context_memory.metadata(self.robot) if self.context_memory else {"episodes": 0},
            "active_mission": self.active_mission.name if self.active_mission else "",
            "queued_plan_steps": len(self.pending_plan),
            "intelligence_profile": intelligence,
            "autonomy_level": intelligence.get("autonomy_level", "high"),
            "initial_policy": intelligence.get("initial_policy", "sensor_fusion_mission_orchestrator"),
            "processing_sensor_contract": list(intelligence.get("processing_sensor_contract", [])),
            "autonomy_capabilities": list(intelligence.get("autonomy_capabilities", [])),
            "intents": {name: list(aliases) for name, aliases in self.profile.intents.items()},
            "missions": {name: list(aliases) for name, aliases in self.profile.missions.items()},
            "specialty_strategy": strategy_for_robot(self.robot).metadata(),
            "adaptive_policy": self.policy.metadata(),
        }

    def resolve(self, text: Any) -> Tuple[str, str]:
        if self.memory is not None:
            learned_kind, learned_name = self.memory.resolve(self.robot, text)
            if learned_kind != "none" and learned_name:
                return learned_kind, learned_name

        best_score = 0.0
        best_kind = "none"
        best_name = ""
        for kind, catalog in (("mission", self.profile.missions), ("intent", self.profile.intents)):
            for name, aliases in catalog.items():
                raw_score = match_alias(text, aliases)
                if not raw_score:
                    continue
                score = float(raw_score) + command_tuning(self.robot, name).authority
                if score > best_score:
                    best_score = score
                    best_kind = kind
                    best_name = name
        if best_score:
            return best_kind, best_name

        available = {("intent", name) for name in self.profile.intents} | {("mission", name) for name in self.profile.missions}
        if self.context_memory is not None:
            historical = self.context_memory.suggest(self.robot, text, available)
            if historical.kind != "none":
                return historical.kind, historical.name

        semantic = contextual_resolve(self.robot, text, self.profile.intents, self.profile.missions)
        if semantic.kind != "none":
            return semantic.kind, semantic.name
        return "none", ""

    def resolve_plan(self, text: Any) -> list[Tuple[str, str, str]]:
        def resolve_clauses(clauses: Iterable[str]) -> list[Tuple[str, str, str]]:
            plan: list[Tuple[str, str, str]] = []
            for clause in clauses:
                kind, name = self.resolve(clause)
                if kind == "none" or not name:
                    return []
                plan.append((kind, name, clause))
            return plan

        clauses = split_explicit_plan(text)
        if not clauses:
            return []
        # Natural requests often join two independently meaningful goals with a
        # plain "e"/"and". Split that conjunction only when every resulting
        # clause independently resolves to an audited skill. Trying this before
        # whole-sentence semantic resolution prevents a broad first goal from
        # swallowing the second one.
        if len(clauses) == 1:
            natural = tuple(
                part.strip(" ,.")
                for part in re.split(r"\s+(?:e|and)\s+", str(text or ""), flags=re.IGNORECASE)
                if normalize_text(part)
            )
            if 1 < len(natural) <= 4:
                natural_plan = resolve_clauses(natural)
                if natural_plan:
                    return natural_plan

        plan = resolve_clauses(clauses)
        return plan

    def _record_context_result(self, text: Any, result: CommandResult, bridge: Any) -> None:
        if self.context_memory is None or result.kind not in {"intent", "mission"} or not result.name:
            return
        # A mission start is not yet evidence of mission success; terminal mission
        # outcomes are recorded by tick(). Failed starts are useful negative context.
        if result.kind == "mission" and result.ok and result.reason == "started":
            return
        self.context_memory.record(
            self.robot, text or result.name, result.kind, result.name, ok=result.ok,
            reason=result.reason, state=getattr(bridge, "state", None), metadata=result.metadata,
        )

    def _execute_resolved(
        self, kind: str, name: str, source_text: str, bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        if kind == "intent":
            result = self.execute_intent(name, bridge, orchestrator)
        elif kind == "mission":
            result = self.start_mission(name, bridge, source_text, orchestrator)
        else:
            result = CommandResult(False, self.robot, reason="unrecognized_for_active_robot", metadata={"text": source_text})
        self._record_context_result(source_text, result, bridge)
        return result

    def intent_dwell_seconds(self, name: str) -> float:
        if name in {"stop", "hover", "land", "takeoff", "home"}:
            return 0.0
        if name.startswith("camera_") or name in {"lights_toggle", "lidar_toggle"}:
            return 0.18
        return 0.45

    def _telemetry_fresh(self, bridge: Any) -> bool:
        state = getattr(bridge, "state", None)
        if state is None:
            return False
        ts = safe_float(getattr(state, "last_telemetry_ts", 0.0), 0.0)
        return bool(getattr(state, "connected", False) and ts and (time.time() - ts) <= 2.5)

    def _allowed(
        self,
        orchestrator: Optional[SynROVOrchestrator],
        source: str,
        command: str,
        *,
        bridge: Any,
        confidence: float = 1.0,
    ) -> bool:
        if orchestrator is None:
            return True
        state = getattr(bridge, "state", None)
        if state is not None:
            context = refresh_state_intelligence_context(state)
            orchestrator.sync_input_context(self.robot, context)
        age = time.time() - self.last_command_ts if self.last_command_ts else 999.0
        arbiter_source = source
        if source == "voice" and command in {"stop", "hover", "emergency_land", "stop_mission"}:
            # Spoken safety commands must remain effective even while a manual
            # controller is active. They enter the same high-priority safety
            # lane as physical emergency actions, while ordinary voice motion
            # remains below keyboard/joystick/Leap ownership.
            arbiter_source = "safety"
        return orchestrator.action_allowed(
            arbiter_source,
            robot=self.robot,
            command=command,
            telemetry_fresh=self._telemetry_fresh(bridge),
            mission_active=self.active_mission is not None and source != "mission",
            last_command_source=self.last_command_source,
            last_command_age_s=age,
            confidence=confidence,
        ).allowed

    def _record(self, source: str, command: str, orchestrator: Optional[SynROVOrchestrator]) -> None:
        self.last_command_source = source
        self.last_command_name = command
        self.last_command_ts = time.time()
        if orchestrator is not None:
            orchestrator.register_action(
                source,
                self.last_command_ts,
                robot=self.robot,
                command=command,
            )

    def cancel_mission(self, reason: str = "cancelled") -> Optional[str]:
        mission = self.active_mission
        self.active_mission = None
        if reason in {"stopped", "cancelled", "robot_changed"}:
            self.pending_plan.clear()
        return mission.name if mission else None

    def execute_text(
        self,
        text: str,
        bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        plan = self.resolve_plan(text)
        if not plan:
            return CommandResult(False, self.robot, reason="unrecognized_for_active_robot", metadata={"text": text})
        self.pending_plan = list(plan[1:])
        kind, name, clause = plan[0]
        result = self._execute_resolved(kind, name, clause, bridge, orchestrator)
        if len(plan) > 1:
            metadata = dict(result.metadata)
            metadata["cognitive_plan"] = [{"kind": k, "name": n, "text": c} for k, n, c in plan]
            result = CommandResult(result.ok, result.robot, result.kind, result.name, result.reason, metadata)
        return result

    def start_mission(
        self,
        name: str,
        bridge: Any,
        text: str = "",
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        if name == "stop_mission":
            previous = self.cancel_mission("stopped")
            self.stop(bridge, "mission", orchestrator)
            return CommandResult(True, self.robot, "mission", name, "stopped", {"previous": previous})
        if name not in self.profile.missions:
            return CommandResult(False, self.robot, "mission", name, "unknown_mission")
        if name == "rhythm_mode" and not self.profile.music_enabled:
            return CommandResult(False, self.robot, "mission", name, "music_only_for_manipulator")
        resource = mission_resource_status(getattr(bridge, "state", None), self.robot, name)
        if resource.get("battery_critical"):
            return CommandResult(False, self.robot, "mission", name, "battery_critical", resource)
        if resource.get("link_critical") and self.robot in {"Vehicle", "Drone"}:
            return CommandResult(False, self.robot, "mission", name, "communication_link_critical", resource)
        if not self._allowed(orchestrator, "voice", name, bridge=bridge):
            return CommandResult(False, self.robot, "mission", name, "blocked_by_priority")
        self.active_mission = MissionState(name, text=text)
        learning = self.policy.begin_mission(name, self.active_mission.data)
        self._record("mission", name, orchestrator)
        metadata = self.metadata()
        metadata["learning"] = learning
        return CommandResult(True, self.robot, "mission", name, "started", metadata)

    def execute_intent(
        self,
        name: str,
        bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        if name not in self.profile.intents:
            return CommandResult(False, self.robot, "intent", name, "unknown_intent")
        if not self._allowed(orchestrator, "voice", name, bridge=bridge):
            return CommandResult(False, self.robot, "intent", name, "blocked_by_priority")
        result = self._execute_intent(name, bridge)
        if result.ok:
            self._record("voice", name, orchestrator)
            self.plan_not_before_ts = max(self.plan_not_before_ts, time.time() + self.intent_dwell_seconds(name))
        return result

    def tick(
        self,
        bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        mission = self.active_mission
        if mission is None:
            if self.pending_plan:
                if time.time() < self.plan_not_before_ts:
                    return CommandResult(False, self.robot, reason="plan_dwell")
                kind, name, clause = self.pending_plan.pop(0)
                return self._execute_resolved(kind, name, clause, bridge, orchestrator)
            return CommandResult(False, self.robot, reason="no_active_mission")
        if not self._allowed(orchestrator, "mission", mission.name, bridge=bridge):
            return CommandResult(False, self.robot, "mission", mission.name, "blocked_by_priority")

        resource = mission_resource_status(getattr(bridge, "state", None), self.robot, mission.name)
        if resource.get("battery_critical"):
            if self.robot == "Drone":
                bridge.drone_land()
            elif self.robot == "Vehicle":
                bridge.send_vehicle(0, 0)
            else:
                bridge.command_manipulator_pose(manip_pose(getattr(bridge, "state", None)))
            self.active_mission = None
            result = CommandResult(True, self.robot, "mission", mission.name, "stopped_battery_critical", resource)
            self._record_context_result(mission.text or mission.name, result, bridge)
            return result
        if resource.get("link_critical") and self.robot in {"Vehicle", "Drone"}:
            if self.robot == "Drone":
                bridge.send_drone(0, 0, 0, 0, 0, 0)
            else:
                bridge.send_vehicle(0, 0)
            return CommandResult(True, self.robot, "mission", mission.name, "paused_link_critical", resource)

        now = time.time()
        elapsed = now - mission.created_ts
        # Strategy selection happens in bounded time segments. Robot-specific
        # mission code reads this value but can only choose from pre-approved
        # variants; raw motor exploration is never permitted.
        self.policy.variant_for_tick(mission.name, mission.data, elapsed)
        duration, metadata = self._tick_mission(mission, bridge, elapsed, now)
        if duration is None:
            self.active_mission = None
            return CommandResult(False, self.robot, "mission", mission.name, "unknown_mission")

        metadata = dict(metadata or {})
        learning = self.policy.observe(
            mission.name,
            mission.data,
            getattr(bridge, "state", None),
            metadata,
            elapsed,
            force=bool(duration <= 0.0 or self.active_mission is None),
        )
        metadata["learning"] = learning
        self._record("mission", mission.name, orchestrator)
        if duration <= 0.0 or self.active_mission is None:
            self.active_mission = None
            result = CommandResult(True, self.robot, "mission", mission.name, "done", metadata)
            self._record_context_result(mission.text or mission.name, result, bridge)
            return result
        if elapsed >= duration:
            self.active_mission = None
            self.stop(bridge, "mission", orchestrator)
            result = CommandResult(True, self.robot, "mission", mission.name, "done", metadata)
            self._record_context_result(mission.text or mission.name, result, bridge)
            return result
        return CommandResult(
            True,
            self.robot,
            "mission",
            mission.name,
            "running",
            {"elapsed_s": elapsed, **metadata},
        )

    def _execute_intent(self, name: str, bridge: Any) -> CommandResult:
        raise NotImplementedError

    def _tick_mission(
        self,
        mission: MissionState,
        bridge: Any,
        elapsed: float,
        now: float,
    ) -> Tuple[Optional[float], Dict[str, Any]]:
        raise NotImplementedError

    def stop(
        self,
        bridge: Any,
        source: str = "manual",
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> None:
        raise NotImplementedError


class ManipulatorAI(DedicatedRobotAI):
    STEP_DEG = 24.0

    def __init__(
        self,
        profile: RobotProfile,
        safety: SynROVSafetyLayer,
        *,
        model_root: Optional[Path] = None,
        memory: Optional[LearnedCommandMemory] = None,
        context_memory: Optional[LongContextMemory] = None,
    ) -> None:
        super().__init__(profile, safety, model_root=model_root, memory=memory, context_memory=context_memory)
        dance_path = self.model_slot.model_dir.parent.parent / "synrov_dance_model.json"
        self.dance_model = AdaptiveDanceModel(dance_path)

    @staticmethod
    def _beat_state(bridge: Any) -> Optional[Any]:
        state = getattr(bridge, "state", None)
        beat = getattr(state, "beat_state", None)
        if beat is not None:
            return beat
        snapshot = getattr(state, "snapshot", {}) or {}
        raw = snapshot.get("beat") if isinstance(snapshot, dict) else None
        return SimpleNamespace(**raw) if isinstance(raw, dict) else None

    def _execute_intent(self, name: str, bridge: Any) -> CommandResult:
        if name == "home":
            self.cancel_mission("cancelled")
            bridge.manip_home()
            return CommandResult(True, self.robot, "intent", name, "home_sent")

        # Canonical physical convention used by every input source:
        # 0° = North and numeric base yaw increases clockwise. Therefore
        # LEFT decreases the yaw target and RIGHT increases it. Absolute base
        # angles pass through unchanged.
        delta: Dict[str, Tuple[str, float]] = {
            "base_left": ("base", -self.STEP_DEG),
            "base_right": ("base", +self.STEP_DEG),
            # Keyboard is the canonical physical reference:
            # S/F/G increase the corresponding joint and mean UP;
            # W/R/T decrease it and mean DOWN.
            "arm_up": ("upper", +self.STEP_DEG),
            "arm_down": ("upper", -self.STEP_DEG),
            "fore_up": ("fore", +self.STEP_DEG),
            "fore_down": ("fore", -self.STEP_DEG),
            "wrist_up": ("wrist_pitch", +self.STEP_DEG),
            "wrist_down": ("wrist_pitch", -self.STEP_DEG),
            "wrist_left": ("wrist_rot", +self.STEP_DEG),
            "wrist_right": ("wrist_rot", -self.STEP_DEG),
            "roll_left": ("forearm_roll", +self.STEP_DEG),
            "roll_right": ("forearm_roll", -self.STEP_DEG),
            "grip_open": ("grip", +self.STEP_DEG),
            "grip_close": ("grip", -self.STEP_DEG),
        }
        if name not in delta:
            return CommandResult(False, self.robot, "intent", name, "unknown_intent")
        key, amount = delta[name]
        pose = manip_pose(getattr(bridge, "state", None))
        lo, hi = MANIP_POSE_LIMITS[key]
        pose[key] = (pose[key] + amount) % 360.0 if key == "base" else clamp(pose[key] + amount, lo, hi)
        bridge.command_manipulator_pose(self.safety.sanitize_pose(pose, source="voice", smooth=True))
        return CommandResult(True, self.robot, "intent", name, "pose_sent")

    def _tick_mission(
        self,
        mission: MissionState,
        bridge: Any,
        elapsed: float,
        now: float,
    ) -> Tuple[Optional[float], Dict[str, Any]]:
        pose = manip_pose(getattr(bridge, "state", None))
        name = mission.name
        duration: Optional[float]

        if name == "return_home":
            bridge.manip_home()
            self.active_mission = None
            return 0.0, {}
        variant = str(mission.data.get("_learning_variant", "balanced") or "balanced")
        if name == "hold_pose":
            duration = 10.0
        elif name == "scan_workspace":
            duration = 18.0
            start_base = safe_float(mission.data.setdefault("scan_start_base", pose.get("base", 180.0)), 180.0)
            progress = clamp(elapsed / duration, 0.0, 1.0)
            phase = progress * math.tau
            amplitudes = {
                "precision": (12.0, 14.0, 22.0, 17.0, 28.0),
                "wide": (22.0, 26.0, 42.0, 30.0, 52.0),
            }.get(variant, (18.0, 20.0, 34.0, 24.0, 40.0))
            upper_amp, fore_amp, roll_amp, wrist_amp, wrist_rot_amp = amplitudes
            pose["base"] = (start_base - 360.0 * progress) % 360.0
            pose["upper"] = clamp(68 + upper_amp * math.sin(phase), 0, 359)
            pose["fore"] = clamp(154 + fore_amp * math.sin(phase + math.pi / 3.0), 0, 359)
            pose["forearm_roll"] = clamp(90 + roll_amp * math.sin(phase * 1.5), 0, 359)
            pose["wrist_pitch"] = clamp(108 + wrist_amp * math.sin(phase * 2.0 + math.pi / 6.0), 0, 359)
            pose["wrist_rot"] = clamp(130 + wrist_rot_amp * math.sin(phase * 1.25 + math.pi / 4.0), 0, 359)
        elif name == "inspect_workspace":
            if variant == "wrist_focus":
                base_amp, upper_amp, wrist_amp = 0.7, 8.0, 24.0
            elif variant == "base_focus":
                base_amp, upper_amp, wrist_amp = 2.2, 10.0, 10.0
            else:
                base_amp, upper_amp, wrist_amp = 1.4, 12.0, 16.0
            pose["base"] = (pose["base"] + base_amp * math.sin(now * 0.55)) % 360.0
            pose["upper"] = clamp(60 + upper_amp * math.sin(now * 0.42), 0, 359)
            pose["wrist_pitch"] = clamp(112 + wrist_amp * math.sin(now * 0.68), 0, 359)
            duration = 24.0
        elif name == "pick_object":
            pose.update({
                "upper": 42 if elapsed < 4.4 else 72,
                "fore": 130 if elapsed < 4.4 else 168,
                "wrist_pitch": 82 if elapsed < 4.4 else 110,
                "grip": 78 if elapsed < 2.2 else 25,
            })
            duration = 8.5
        elif name == "place_object":
            pose.update({
                "upper": 52,
                "fore": 144,
                "wrist_pitch": 92,
                "grip": 82 if elapsed > 2 else pose["grip"],
            })
            duration = 6.5
        elif name == "calibrate_gripper":
            pose["grip"] = 15 if int(elapsed * 1.6) % 2 == 0 else 85
            duration = 6.0
        elif name == "rhythm_mode":
            beat = self._beat_state(bridge)
            if beat is None or not bool(getattr(beat, "active", False)) or safe_float(getattr(beat, "confidence", 0.0), 0.0) < 0.54:
                # No synthetic dance without a real beat. Send HOME only once
                # while waiting, instead of flooding Processing while silent.
                if mission.data.get("rhythm_wait_home_sent"):
                    return 30.0, {"beat_wait": True}
                mission.data["rhythm_wait_home_sent"] = True
                pose = dict(MANIPULATOR_HOME_POSE)
            elif self.dance_model.due("Manipulator", beat, now=now):
                mission.data["rhythm_wait_home_sent"] = False
                pose = self.dance_model.manipulator_pose(beat, now=now)
            else:
                return 30.0, {"beat_wait": False, "dance_throttled": True}
            duration = 30.0
        elif name == "wave":
            # Wave is a gripper gesture. Keep every other joint in the exact
            # HOME pose so a greeting can never drag the arm away from neutral.
            pose = dict(MANIPULATOR_HOME_POSE)
            pose["grip"] = clamp(50.0 + 28.0 * math.sin(now * 5.0), 20.0, 80.0)
            duration = 5.0
        else:
            return None, {}

        source = "music" if name == "rhythm_mode" else "mission"
        bridge.command_manipulator_pose(self.safety.sanitize_pose(pose, source=source, smooth=True))
        state = getattr(bridge, "state", None)
        torque_refs = manipulator_joint_torque_references(state)
        metadata = {
            "joint_torque_references": torque_refs,
            "strategy": "dual_imu_joint_load_aware_manipulation",
            "strategy_variant": variant,
        }
        if name in {"scan_workspace", "inspect_workspace"}:
            # Vision outcome gives the bounded strategy learner a real signal:
            # scan variants that expose/center useful targets earn more reward.
            metadata["vision"] = _vision_target(state)
        return duration, metadata

    def stop(self, bridge: Any, source: str = "manual", orchestrator: Optional[SynROVOrchestrator] = None) -> None:
        bridge.command_manipulator_pose(manip_pose(getattr(bridge, "state", None)))
        self._record(source, "hold_pose", orchestrator)


class VehicleAI(DedicatedRobotAI):
    def _vehicle_control(self, bridge: Any) -> Dict[str, Any]:
        control = getattr(getattr(bridge, "state", None), "control", {}) or {}
        return control if isinstance(control, dict) else {}

    def _execute_intent(self, name: str, bridge: Any) -> CommandResult:
        drive_mapping: Dict[str, Tuple[float, float]] = {
            "forward": (0.34, 0.00),
            "back": (-0.28, 0.00),
            "left": (0.14, -0.34),
            "right": (0.14, 0.34),
        }
        if name in drive_mapping:
            bridge.send_vehicle(*drive_mapping[name], pivot=0.0)
            return CommandResult(True, self.robot, "intent", name, "drive_sent")
        if name == "pivot_left":
            bridge.send_vehicle(0.0, 0.0, pivot=-0.34)
            return CommandResult(True, self.robot, "intent", name, "pivot_sent")
        if name == "pivot_right":
            bridge.send_vehicle(0.0, 0.0, pivot=0.34)
            return CommandResult(True, self.robot, "intent", name, "pivot_sent")
        if name == "stop":
            bridge.send_vehicle(0.0, 0.0, pivot=0.0)
            return CommandResult(True, self.robot, "intent", name, "drive_sent")

        # Vehicle camera follows the Processing keyboard exactly:
        # Z/X = left/right and R/F = down/up.
        camera_delta = {
            "camera_left": (5.0, 0.0),
            "camera_right": (-5.0, 0.0),
            "camera_up": (0.0, 5.0),
            "camera_down": (0.0, -5.0),
        }
        if name in camera_delta:
            pan_delta, tilt_delta = camera_delta[name]
            camera = self._vehicle_control(bridge).get("camera", {})
            if not isinstance(camera, dict):
                camera = {}
            pan = safe_float(camera.get("pan", 0.0), 0.0) + pan_delta
            tilt = safe_float(camera.get("tilt", 0.0), 0.0) + tilt_delta
            bridge.set_camera("Vehicle", pan=pan, tilt=tilt)
            return CommandResult(True, self.robot, "intent", name, "camera_sent")
        if name == "camera_center":
            pose = default_camera_pose("Vehicle")
            bridge.set_camera("Vehicle", pan=pose.pan_deg, tilt=pose.tilt_deg)
            return CommandResult(True, self.robot, "intent", name, "camera_centered")
        if name == "camera_toggle":
            camera = self._vehicle_control(bridge).get("camera", {})
            current = bool(camera.get("enabled", False)) if isinstance(camera, dict) else False
            bridge.set_camera_enabled("Vehicle", not current)
            return CommandResult(True, self.robot, "intent", name, "camera_toggled", {"enabled": not current})
        if name == "lights_toggle":
            current = bool(self._vehicle_control(bridge).get("lights", False))
            bridge.set_vehicle_lights(not current)
            return CommandResult(True, self.robot, "intent", name, "lights_toggled", {"enabled": not current})
        if name == "lidar_toggle":
            current = bool(self._vehicle_control(bridge).get("lidarScan", False))
            bridge.set_vehicle_lidar_scan(not current)
            return CommandResult(True, self.robot, "intent", name, "lidar_toggled", {"enabled": not current})
        return CommandResult(False, self.robot, "intent", name, "unknown_intent")

    def _send_adaptive(
        self,
        bridge: Any,
        throttle: float,
        steer: float,
        cam_pan: Optional[float] = None,
        cam_tilt: Optional[float] = None,
    ) -> Dict[str, Any]:
        throttle, steer, metadata = adapt_vehicle_motion(getattr(bridge, "state", None), throttle, steer)
        if cam_pan is None and cam_tilt is None:
            bridge.send_vehicle(throttle, steer)
        else:
            bridge.send_vehicle(throttle, steer, cam_pan, cam_tilt)
        return metadata

    def _tick_mission(
        self,
        mission: MissionState,
        bridge: Any,
        elapsed: float,
        now: float,
    ) -> Tuple[Optional[float], Dict[str, Any]]:
        name = mission.name
        variant = str(mission.data.get("_learning_variant", "balanced") or "balanced")

        if name == "hold_position":
            bridge.send_vehicle(0, 0)
            return 12.0, {"strategy": "hold_position", "strategy_variant": variant}

        if name == "patrol_area":
            if variant == "careful":
                throttle, steer_amp, camera_amp = 0.13, 0.18, 18.0
            elif variant == "wide_sweep":
                throttle, steer_amp, camera_amp = 0.17, 0.32, 42.0
            else:
                throttle, steer_amp, camera_amp = 0.18, 0.24, 25.0
            metadata = self._send_adaptive(bridge, throttle, steer_amp * math.sin(now * 0.42), camera_amp * math.sin(now * 0.5), 0)
            metadata.update({"strategy": "adaptive_patrol", "strategy_variant": variant})
            return 30.0, metadata

        if name == "perimeter_scan":
            if variant == "careful":
                throttle, steer, camera_amp = 0.085, 0.32, 28.0
            elif variant == "wide_camera":
                throttle, steer, camera_amp = 0.11, 0.40, 55.0
            else:
                throttle, steer, camera_amp = 0.12, 0.42, 38.0
            metadata = self._send_adaptive(bridge, throttle, steer, camera_amp * math.sin(now * 0.7), 0)
            metadata.update({"strategy": "adaptive_perimeter_scan", "strategy_variant": variant})
            return 22.0, metadata

        if name == "corridor_scan":
            if variant == "careful":
                throttle, steer_amp, camera_amp = 0.115, 0.045, 34.0
            elif variant == "camera_sweep":
                throttle, steer_amp, camera_amp = 0.145, 0.055, 58.0
            else:
                throttle, steer_amp, camera_amp = 0.16, 0.06, 50.0
            metadata = self._send_adaptive(bridge, throttle, steer_amp * math.sin(now * 0.75), camera_amp * math.sin(now * 0.9), 10)
            metadata.update({"strategy": "adaptive_corridor_scan", "strategy_variant": variant})
            return 28.0, metadata

        if name == "follow_target":
            state = getattr(bridge, "state", None)
            vision = _vision_target(state)
            if vision["confidence"] >= 0.30:
                gain = 0.56 if variant == "careful" else (0.82 if variant == "responsive" else 0.70)
                max_steer = 0.30 if variant == "careful" else (0.48 if variant == "responsive" else 0.40)
                steer = clamp(vision["dx"] * gain, -max_steer, max_steer)
                base_throttle = 0.11 if variant == "careful" else 0.15
                throttle = clamp(base_throttle - vision["area"] * 1.15, 0.02, base_throttle)
                metadata = self._send_adaptive(bridge, throttle, steer, clamp(vision["dx"] * 55.0, -55.0, 55.0), -2.0)
                metadata.update({"strategy": "visual_target_follow", "strategy_variant": variant, "vision": vision})
                return 24.0, metadata
            search_amp = 0.12 if variant == "careful" else (0.28 if variant == "responsive" else 0.20)
            metadata = self._send_adaptive(bridge, 0.06, search_amp * math.sin(now * 0.72), 48.0 * math.sin(now * 0.55), -2.0)
            metadata.update({"strategy": "visual_target_reacquire", "strategy_variant": variant, "vision": vision})
            return 24.0, metadata

        if name == "dock":
            distance = obstacle_distance_cm(getattr(bridge, "state", None))
            if distance is not None and distance <= 22.0:
                bridge.send_vehicle(0, 0, 0, -8)
                self.active_mission = None
                return 0.0, {"dock_distance_cm": distance, "dock_reached": True, "strategy": "range_dock", "strategy_variant": variant}
            throttle = 0.10 if elapsed < 5.0 else 0.04
            metadata = self._send_adaptive(bridge, throttle, 0, 0, -8)
            metadata.update({"dock_distance_cm": distance, "strategy": "range_dock", "strategy_variant": variant})
            return 8.0, metadata

        if name == "terrain_inspection":
            lane_phase = int(elapsed // 9.0)
            if variant == "careful":
                throttle, lane_steer, turn_steer, camera_amp = 0.12, 0.045, 0.30, 32.0
            elif variant == "wide_sweep":
                throttle, lane_steer, turn_steer, camera_amp = 0.16, 0.075, 0.44, 55.0
            else:
                throttle, lane_steer, turn_steer, camera_amp = 0.17, 0.06, 0.38, 42.0
            steer = lane_steer * math.sin(now * 0.7)
            if (elapsed % 9.0) > 7.4:
                steer = turn_steer if lane_phase % 2 == 0 else -turn_steer
            metadata = self._send_adaptive(bridge, throttle, steer, camera_amp * math.sin(now * 0.32), -4.0)
            metadata.update({"strategy": "systematic_ground_sweep", "strategy_variant": variant, "lane": lane_phase})
            return 72.0, metadata

        if name == "locate_objects":
            state = getattr(bridge, "state", None)
            vision = _vision_target(state)
            if vision["confidence"] >= 0.35:
                steer_gain = 0.60 if variant == "slow_scan" else (0.82 if variant == "wide_scan" else 0.72)
                steer = clamp(vision["dx"] * steer_gain, -0.42, 0.42)
                throttle_max = 0.12 if variant == "slow_scan" else 0.17
                throttle = clamp(throttle_max - vision["area"] * 1.6, 0.025, throttle_max)
                metadata = self._send_adaptive(bridge, throttle, steer, clamp(vision["dx"] * 55.0, -55.0, 55.0), -2.0)
                metadata.update({"strategy": "camera_target_approach", "strategy_variant": variant, "vision": vision})
                if vision["centered"] and vision["close"]:
                    bridge.send_vehicle(0, 0)
                    self.active_mission = None
                    metadata["object_localized"] = True
                    return 0.0, metadata
                return 55.0, metadata
            if variant == "slow_scan":
                throttle, steer_amp, camera_amp, rate = 0.055, 0.16, 45.0, 0.32
            elif variant == "wide_scan":
                throttle, steer_amp, camera_amp, rate = 0.075, 0.34, 62.0, 0.52
            else:
                throttle, steer_amp, camera_amp, rate = 0.075, 0.24, 58.0, 0.42
            metadata = self._send_adaptive(bridge, throttle, steer_amp * math.sin(now * rate), camera_amp * math.sin(now * 0.58), -2.0)
            metadata.update({"strategy": "camera_lidar_search", "strategy_variant": variant, "vision": vision})
            return 55.0, metadata

        if name == "find_exit":
            steer, corridor = vehicle_exit_steer(getattr(bridge, "state", None))
            steer_scale = 0.78 if variant == "careful" else (1.12 if variant == "decisive" else 1.0)
            steer = clamp(steer * steer_scale, -0.48, 0.48)
            straight_throttle = 0.11 if variant == "careful" else (0.17 if variant == "decisive" else 0.15)
            turn_throttle = 0.065 if variant == "careful" else (0.10 if variant == "decisive" else 0.085)
            throttle = straight_throttle if abs(steer) < 0.1 else turn_throttle
            metadata = self._send_adaptive(bridge, throttle, steer, clamp(-steer * 80.0, -55.0, 55.0), 0.0)
            metadata.update({"strategy": "lidar_3d_free_corridor", "strategy_variant": variant, **corridor})
            return 60.0, metadata

        if name == "return_home":
            nav = _local_navigation_pose(getattr(bridge, "state", None), self.robot)
            if nav is None:
                bridge.send_vehicle(0, 0, pivot=0.0)
                return 75.0, {"strategy": "origin_navigation", "strategy_variant": variant, "navigation_available": False}
            dx, dz = -nav["x"], -nav["z"]
            distance = math.hypot(dx, dz)
            arrival = 0.22 if nav["units"] == "meters" else 12.0
            if distance <= arrival:
                bridge.send_vehicle(0, 0, pivot=0.0)
                self.active_mission = None
                return 0.0, {"strategy": "origin_navigation", "strategy_variant": variant, "arrived_home": True, "distance": distance, "units": nav["units"]}
            desired_yaw = math.degrees(math.atan2(dx, dz))
            yaw_error = _angle_error_deg(desired_yaw, nav["yaw_deg"])
            # Processing's Vehicle semantic steering is intentionally opposite
            # the local yaw sign: negative steer/pivot turns toward +yaw.
            steer = clamp(-yaw_error / 60.0, -0.42, 0.42)
            if abs(yaw_error) > 32.0:
                bridge.send_vehicle(0.0, 0.0, pivot=steer)
                metadata = {"avoidance": False}
                phase = "align_heading"
            else:
                throttle = 0.10 if variant == "careful" else 0.15
                metadata = self._send_adaptive(bridge, throttle, steer)
                phase = "approach_origin"
            metadata.update({
                "strategy": "origin_navigation", "strategy_variant": variant, "phase": phase,
                "distance": distance, "units": nav["units"], "yaw_error_deg": yaw_error,
            })
            return 75.0, metadata
        return None, {}

    def stop(self, bridge: Any, source: str = "manual", orchestrator: Optional[SynROVOrchestrator] = None) -> None:
        bridge.send_vehicle(0, 0, pivot=0.0)
        self._record(source, "stop", orchestrator)


class DroneAI(DedicatedRobotAI):
    FLIGHT_REQUIRED_MISSIONS = {
        "aerial_scan", "orbit_point", "search_pattern", "altitude_hold",
        "terrain_inspection", "locate_objects", "find_exit", "return_home",
    }
    FLIGHT_REQUIRED_INTENTS = {"forward", "back", "left", "right", "yaw_left", "yaw_right", "down"}

    def intent_dwell_seconds(self, name: str) -> float:
        if name in {"hover", "land", "takeoff"}:
            return 0.0
        if name.startswith("camera_"):
            return 0.18
        if name in self.FLIGHT_REQUIRED_INTENTS or name == "up":
            return 1.0
        return super().intent_dwell_seconds(name)

    def _flight_status(self, bridge: Any) -> Dict[str, Any]:
        state = getattr(bridge, "state", None)
        control = getattr(state, "control", {}) or {}
        flight = control.get("flight", {}) if isinstance(control, Mapping) else {}
        if not isinstance(flight, Mapping):
            flight = {}
        phase = str(flight.get("phase", "") or "").strip().lower()
        sensors = getattr(state, "sensors", {}) or {}
        altitude = 0.0
        for key in ("alt_cm", "altitude_cm", "sonar_down_cm"):
            if key in sensors:
                altitude = max(altitude, safe_float(sensors.get(key), 0.0))
        throttle = safe_float(flight.get("throttle", 0.0), 0.0)
        ready = bool(flight.get("flightReady", False)) or phase in {"taking_off", "airborne"} or throttle > 0.02 or altitude >= 24.0
        if phase == "landing":
            ready = False
        airborne = bool(flight.get("airborne", False)) or altitude > 3.0
        if not phase:
            phase = "airborne" if ready else "taking_off" if airborne else "grounded"
        return {"phase": phase, "flight_ready": ready, "airborne": airborne}

    def tick(
        self,
        bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        status = self._flight_status(bridge)
        deferred = str(getattr(self, "_deferred_flight_intent", "") or "")
        if deferred:
            if not status["flight_ready"]:
                if status["phase"] not in {"taking_off", "landing"}:
                    bridge.drone_takeoff()
                return CommandResult(True, self.robot, "intent", deferred, "waiting_for_takeoff", {"flight": status})
            self._deferred_flight_intent = ""
            return self.execute_intent(deferred, bridge, orchestrator)

        mission = self.active_mission
        if mission is not None and mission.name in self.FLIGHT_REQUIRED_MISSIONS:
            if not status["flight_ready"]:
                if status["phase"] != "taking_off":
                    bridge.drone_takeoff()
                mission.data["_preflight_pending"] = True
                return CommandResult(True, self.robot, "mission", mission.name, "waiting_for_takeoff", {"flight": status, "prerequisite": "takeoff"})
            if mission.data.pop("_preflight_pending", False):
                mission.created_ts = time.time()

        # Explicit plans such as "decole, depois avance" can consume the next
        # step as soon as takeoff/climb command authority is established.
        if self.pending_plan and not status["flight_ready"] and self.last_command_name == "takeoff":
            return CommandResult(True, self.robot, "intent", "takeoff", "waiting_for_takeoff", {"flight": status, "queued_steps": len(self.pending_plan)})
        return super().tick(bridge, orchestrator)

    def _drone_control(self, bridge: Any) -> Dict[str, Any]:
        control = getattr(getattr(bridge, "state", None), "control", {}) or {}
        return control if isinstance(control, dict) else {}

    def _execute_intent(self, name: str, bridge: Any) -> CommandResult:
        if name == "takeoff":
            self._deferred_flight_intent = ""
            bridge.drone_takeoff()
            return CommandResult(True, self.robot, "intent", name, "takeoff_sent")
        if name == "land":
            self._deferred_flight_intent = ""
            bridge.drone_land()
            return CommandResult(True, self.robot, "intent", name, "land_sent")
        if name == "hover":
            self._deferred_flight_intent = ""

        flight_status = self._flight_status(bridge)
        if name in self.FLIGHT_REQUIRED_INTENTS and not flight_status["flight_ready"]:
            if name == "down" and flight_status["phase"] == "grounded":
                return CommandResult(False, self.robot, "intent", name, "already_grounded", {"flight": flight_status})
            self._deferred_flight_intent = name
            if flight_status["phase"] != "taking_off":
                bridge.drone_takeoff()
            return CommandResult(True, self.robot, "intent", name, "takeoff_then_intent_queued", {"flight": flight_status})

        mapping: Dict[str, Tuple[float, float, float, float, float, float]] = {
            "up": (0.30, 0.00, 0.00, 0.00, 0.00, 0.00),
            "down": (-0.24, 0.00, 0.00, 0.00, 0.00, 0.00),
            "forward": (0.00, 0.00, 0.00, 0.00, 0.00, 0.30),
            "back": (0.00, 0.00, 0.00, 0.00, 0.00, -0.26),
            "left": (0.00, 0.00, 0.00, 0.00, 0.22, 0.00),
            "right": (0.00, 0.00, 0.00, 0.00, -0.22, 0.00),
            # AiBot resolves the operator's visual reference here; the
            # canonical yaw field itself is never remapped by the protocol.
            "yaw_left": (0.00, 0.22, 0.00, 0.00, 0.00, 0.00),
            "yaw_right": (0.00, -0.22, 0.00, 0.00, 0.00, 0.00),
            "hover": (0.00, 0.00, 0.00, 0.00, 0.00, 0.00),
        }
        if name in mapping:
            bridge.send_drone(*mapping[name])
            return CommandResult(True, self.robot, "intent", name, "flight_sent")

        # Drone camera follows the Processing keyboard exactly:
        # Z/X = left/right and R/F = down/up.
        camera_delta = {
            "camera_left": (5.0, 0.0),
            "camera_right": (-5.0, 0.0),
            "camera_up": (0.0, 5.0),
            "camera_down": (0.0, -5.0),
        }
        if name in camera_delta:
            pan_delta, tilt_delta = camera_delta[name]
            camera = self._drone_control(bridge).get("camera", {})
            if not isinstance(camera, dict):
                camera = {}
            pan = safe_float(camera.get("pan", 0.0), 0.0) + pan_delta
            tilt = safe_float(camera.get("tilt", 0.0), 0.0) + tilt_delta
            bridge.set_camera("Drone", pan=pan, tilt=tilt)
            return CommandResult(True, self.robot, "intent", name, "camera_sent")
        if name == "camera_center":
            pose = default_camera_pose("Drone")
            bridge.set_camera("Drone", pan=pose.pan_deg, tilt=pose.tilt_deg)
            return CommandResult(True, self.robot, "intent", name, "camera_centered")
        if name == "camera_toggle":
            camera = self._drone_control(bridge).get("camera", {})
            current = bool(camera.get("enabled", False)) if isinstance(camera, dict) else False
            bridge.set_camera_enabled("Drone", not current)
            return CommandResult(True, self.robot, "intent", name, "camera_toggled", {"enabled": not current})
        return CommandResult(False, self.robot, "intent", name, "unknown_intent")

    def _send_adaptive(self, bridge: Any, flight: Tuple[float, float, float, float, float, float]) -> Dict[str, Any]:
        values, metadata = adapt_drone_motion(getattr(bridge, "state", None), flight)
        if metadata.get("recommend_land"):
            bridge.drone_land()
            self.active_mission = None
            metadata["autonomous_action"] = "land_low_battery"
            return metadata
        bridge.send_drone(*values)
        return metadata

    def _tick_mission(
        self,
        mission: MissionState,
        bridge: Any,
        elapsed: float,
        now: float,
    ) -> Tuple[Optional[float], Dict[str, Any]]:
        name = mission.name
        variant = str(mission.data.get("_learning_variant", "balanced") or "balanced")

        if name == "aerial_scan":
            if variant == "careful":
                yaw, strafe, forward = 0.16, 0.025, 0.025
            elif variant == "wide_scan":
                yaw, strafe, forward = 0.28, 0.08, 0.055
            else:
                yaw, strafe, forward = 0.24, 0.04, 0.04
            metadata = self._send_adaptive(bridge, (0, yaw, 0, 0, strafe * math.sin(now * 0.5), forward))
            metadata.update({"strategy": "adaptive_aerial_scan", "strategy_variant": variant})
            return 18.0, metadata

        if name == "orbit_point":
            if variant == "careful":
                yaw, roll_amp, strafe, forward = 0.13, 0.05, 0.13, 0.085
            elif variant == "wide_orbit":
                yaw, roll_amp, strafe, forward = 0.20, 0.10, 0.22, 0.14
            else:
                yaw, roll_amp, strafe, forward = 0.18, 0.08, 0.18, 0.12
            metadata = self._send_adaptive(bridge, (0, yaw, 0, roll_amp * math.sin(now * 0.6), strafe, forward))
            metadata.update({"strategy": "adaptive_orbit", "strategy_variant": variant})
            return 22.0, metadata

        if name == "search_pattern":
            if variant == "slow_scan":
                yaw_amp, strafe_amp, forward = 0.065, 0.14, 0.10
            elif variant == "wide_scan":
                yaw_amp, strafe_amp, forward = 0.14, 0.28, 0.15
            else:
                yaw_amp, strafe_amp, forward = 0.10, 0.22, 0.16
            metadata = self._send_adaptive(bridge, (0, yaw_amp * math.sin(now * 0.32), -0.04, 0, strafe_amp * math.sin(now * 0.55), forward))
            metadata.update({"strategy": "adaptive_search_pattern", "strategy_variant": variant})
            return 30.0, metadata

        if name == "altitude_hold":
            state = getattr(bridge, "state", None)
            sensors = getattr(state, "sensors", {}) or {}
            current_alt = None
            for key in ("alt_cm", "altitude_cm", "sonar_down_cm"):
                if key in sensors:
                    current_alt = safe_float(sensors.get(key), 0.0)
                    if current_alt > 0:
                        break
            target_alt = safe_float(mission.data.get("target_alt_cm", 0.0), 0.0)
            if target_alt <= 0.0 and current_alt and current_alt > 0.0:
                target_alt = current_alt
                mission.data["target_alt_cm"] = target_alt
            throttle = clamp((target_alt - current_alt) * 0.008, -0.14, 0.14) if target_alt > 0.0 and current_alt else 0.0
            metadata = self._send_adaptive(bridge, (throttle, 0, 0, 0, 0, 0))
            metadata.update({"target_alt_cm": target_alt, "current_alt_cm": current_alt, "strategy": "sonar_altitude_hold", "strategy_variant": variant})
            return 16.0, metadata

        if name == "terrain_inspection":
            band = int(elapsed // 12.0)
            if variant == "careful":
                strafe_amp, forward, yaw_amp = 0.11, 0.12, 0.025
            elif variant == "wide_sweep":
                strafe_amp, forward, yaw_amp = 0.21, 0.16, 0.05
            else:
                strafe_amp, forward, yaw_amp = 0.16, 0.17, 0.035
            strafe = strafe_amp if band % 2 == 0 else -strafe_amp
            yaw = yaw_amp * math.sin(now * 0.25)
            metadata = self._send_adaptive(bridge, (0.0, yaw, 0.0, 0.0, strafe, forward))
            metadata.update({"strategy": "aerial_lawnmower_survey", "strategy_variant": variant, "band": band})
            return 84.0, metadata

        if name == "locate_objects":
            state = getattr(bridge, "state", None)
            vision = _vision_target(state)
            if vision["confidence"] >= 0.35:
                yaw_gain = 0.34 if variant == "slow_scan" else (0.52 if variant == "wide_scan" else 0.44)
                yaw_limit = 0.22 if variant == "slow_scan" else 0.28
                yaw = clamp(vision["dx"] * yaw_gain, -yaw_limit, yaw_limit)
                throttle = clamp(-vision["dy"] * 0.16, -0.08, 0.08)
                forward_max = 0.11 if variant == "slow_scan" else 0.15
                forward = clamp(forward_max - vision["area"] * 1.25, 0.02, forward_max)
                metadata = self._send_adaptive(bridge, (throttle, yaw, 0.0, 0.0, 0.0, forward))
                metadata.update({"strategy": "front_camera_object_localization", "strategy_variant": variant, "vision": vision})
                if vision["centered"] and vision["close"]:
                    bridge.send_drone(0, 0, 0, 0, 0, 0)
                    self.active_mission = None
                    metadata["object_localized"] = True
                    return 0.0, metadata
                return 60.0, metadata
            if variant == "slow_scan":
                yaw, strafe_amp, forward = 0.10, 0.08, 0.04
            elif variant == "wide_scan":
                yaw, strafe_amp, forward = 0.20, 0.18, 0.06
            else:
                yaw, strafe_amp, forward = 0.16, 0.12, 0.055
            metadata = self._send_adaptive(bridge, (0.0, yaw, 0.0, 0.0, strafe_amp * math.sin(now * 0.43), forward))
            metadata.update({"strategy": "yaw_strafe_visual_search", "strategy_variant": variant, "vision": vision})
            return 60.0, metadata

        if name == "find_exit":
            strafe, forward, climb, corridor = drone_exit_vector(getattr(bridge, "state", None))
            if variant == "careful":
                strafe *= 0.72
                forward *= 0.68
                climb *= 0.72
                yaw_amp = 0.025
            elif variant == "decisive":
                strafe = clamp(strafe * 1.12, -0.30, 0.30)
                forward = clamp(forward * 1.10, 0.0, 0.22)
                climb = clamp(climb * 1.08, -0.14, 0.14)
                yaw_amp = 0.06
            else:
                yaw_amp = 0.045
            metadata = self._send_adaptive(bridge, (climb, yaw_amp * math.sin(now * 0.35), 0.0, 0.0, strafe, forward))
            metadata.update({"strategy": "three_dimensional_free_corridor", "strategy_variant": variant, **corridor})
            return 65.0, metadata

        if name == "return_home":
            nav = _local_navigation_pose(getattr(bridge, "state", None), self.robot)
            if nav is None:
                bridge.send_drone(0, 0, 0, 0, 0, 0)
                return 90.0, {"strategy": "origin_navigation", "strategy_variant": variant, "navigation_available": False}
            dx, dz = -nav["x"], -nav["z"]
            distance = math.hypot(dx, dz)
            arrival = 0.28 if nav["units"] == "meters" else 15.0
            if distance <= arrival:
                bridge.send_drone(0, 0, 0, 0, 0, 0)
                self.active_mission = None
                return 0.0, {"strategy": "origin_navigation", "strategy_variant": variant, "arrived_home": True, "distance": distance, "units": nav["units"]}
            # Drone forward points toward +Z at yaw=0 in Processing, matching
            # the local navigation integration used by the 3D runtime.
            desired_yaw = math.degrees(math.atan2(dx, dz))
            yaw_error = _angle_error_deg(desired_yaw, nav["yaw_deg"])
            yaw_cmd = clamp(yaw_error / 70.0, -0.30, 0.30)
            forward = 0.0 if abs(yaw_error) > 38.0 else (0.08 if variant == "careful" else 0.14)
            metadata = self._send_adaptive(bridge, (0.0, yaw_cmd, 0.0, 0.0, 0.0, forward))
            metadata.update({
                "strategy": "origin_navigation", "strategy_variant": variant,
                "phase": "align_heading" if forward == 0.0 else "approach_origin",
                "distance": distance, "units": nav["units"], "yaw_error_deg": yaw_error,
            })
            return 90.0, metadata
        if name == "emergency_land":
            bridge.drone_land()
            self.active_mission = None
            return 0.0, {"autonomous_action": "emergency_land", "strategy": "emergency_land", "strategy_variant": variant}
        return None, {}

    def stop(self, bridge: Any, source: str = "manual", orchestrator: Optional[SynROVOrchestrator] = None) -> None:
        self._deferred_flight_intent = ""
        if source == "manual":
            self.pending_plan.clear()
        bridge.send_drone(0, 0, 0, 0, 0, 0)
        self._record(source, "hover", orchestrator)


class RobotAIRegistry:
    """Own exactly one independent AI instance and model slot per robot."""

    def __init__(
        self,
        safety: Optional[SynROVSafetyLayer] = None,
        *,
        model_root: Optional[Path] = None,
        memory_path: Optional[Path] = None,
        context_memory_path: Optional[Path] = None,
    ) -> None:
        self.safety = safety or SynROVSafetyLayer()
        self.memory = LearnedCommandMemory(memory_path)
        if context_memory_path is None and memory_path is not None:
            context_memory_path = Path(memory_path).with_name("long_context.jsonl")
        self.context_memory = LongContextMemory(context_memory_path)
        self.ais: Dict[str, DedicatedRobotAI] = {
            "Manipulator": ManipulatorAI(MANIPULATOR_PROFILE, self.safety, model_root=model_root, memory=self.memory, context_memory=self.context_memory),
            "Vehicle": VehicleAI(VEHICLE_PROFILE, self.safety, model_root=model_root, memory=self.memory, context_memory=self.context_memory),
            "Drone": DroneAI(DRONE_PROFILE, self.safety, model_root=model_root, memory=self.memory, context_memory=self.context_memory),
        }
        self._active_robot = ""

    def get(self, robot: Any) -> DedicatedRobotAI:
        return self.ais[canonical_robot(robot)]

    def sync_active_robot(self, bridge: Any) -> DedicatedRobotAI:
        current = canonical_robot(getattr(getattr(bridge, "state", None), "robot", "Manipulator"))
        if self._active_robot and self._active_robot != current:
            # Do not send a stop to the previously active robot here: that would make
            # Processing select the previous mode again.  Only cancel stale Python
            # autonomy so it can never resume after a later robot switch.
            self.ais[self._active_robot].cancel_mission("robot_changed")
        self._active_robot = current
        return self.ais[current]

    def _active_ai(self, bridge: Any) -> DedicatedRobotAI:
        return self.sync_active_robot(bridge)

    def execute_text(self, text: str, bridge: Any, orchestrator: Optional[SynROVOrchestrator] = None) -> CommandResult:
        return self._active_ai(bridge).execute_text(text, bridge, orchestrator)

    def tick_active(self, bridge: Any, orchestrator: Optional[SynROVOrchestrator] = None) -> CommandResult:
        return self._active_ai(bridge).tick(bridge, orchestrator)

    def stop_active(self, bridge: Any, orchestrator: Optional[SynROVOrchestrator] = None) -> None:
        self._active_ai(bridge).stop(bridge, "manual", orchestrator)

    def learn_alias(self, robot: Any, phrase: Any, kind: str, name: str) -> bool:
        profile = self.get(robot).profile
        if kind == "intent" and name not in profile.intents:
            return False
        if kind == "mission" and name not in profile.missions:
            return False
        return self.memory.learn(robot, phrase, kind, name)

    def forget_alias(self, robot: Any, phrase: Any) -> bool:
        return self.memory.forget(robot, phrase)

    def learned_aliases(self, robot: Any) -> Dict[str, Dict[str, str]]:
        return self.memory.list_for_robot(robot)

    def metadata(self) -> Dict[str, Any]:
        return {name: ai.metadata() for name, ai in self.ais.items()}


__all__ = [
    "CommandResult",
    "DedicatedRobotAI",
    "ManipulatorAI",
    "VehicleAI",
    "DroneAI",
    "RobotAIRegistry",
    "RobotModelSlot",
    "RobotProfile",
    "MissionState",
    "LearnedCommandMemory",
    "MANIPULATOR_PROFILE",
    "VEHICLE_PROFILE",
    "DRONE_PROFILE",
    "match_alias",
    "manip_pose",
]
