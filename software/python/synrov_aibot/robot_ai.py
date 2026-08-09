"""Dedicated robot AIs for Manipulator, Vehicle and Drone.

Each robot owns its command vocabulary, mission state, model/dataset paths and
arbitration tuning.  Robot identity is supplied by Processing over WebSocket;
no Python-side selector is used by this backend.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Tuple
from types import SimpleNamespace

from .autonomy import adapt_drone_motion, adapt_vehicle_motion, command_tuning, obstacle_distance_cm
from .dataset import TARGET_DIMS, canonical_robot
from .dance import AdaptiveDanceModel, HOME_POSE as DANCE_HOME_POSE
from .orchestrator import SynROVOrchestrator
from .primitives import normalize_text, safe_float
from .protocol import COMMANDS_SCHEMA, SYNROV_VERSION
from .safety import MANIP_KEYS, MANIP_POSE_LIMITS, SynROVSafetyLayer, clamp


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
        base = Path(root or Path(__file__).resolve().parent / "synrov_multimodal_data")
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
            or Path(__file__).resolve().parent / "synrov_multimodal_data" / "learned_commands.json"
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
            and raw.get("version") == SYNROV_VERSION
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
            payload = {"schema": COMMANDS_SCHEMA, "version": SYNROV_VERSION, "robots": self._data}
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
    defaults = {
        "base": 180.0,
        "upper": 45.0,
        "fore": 180.0,
        "forearm_roll": 90.0,
        "wrist_pitch": 95.0,
        "wrist_rot": 130.0,
        "grip": 50.0,
    }
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


MANIPULATOR_PROFILE = RobotProfile(
    "Manipulator",
    7,
    True,
    intents={
        "base_left": ("base esquerda", "virar base esquerda", "girar base esquerda", "rotacionar base esquerda", "base left"),
        "base_right": ("base direita", "virar base direita", "girar base direita", "rotacionar base direita", "base right"),
        "arm_up": ("subir braço", "subir braco", "levantar braço", "levantar braco", "arm up"),
        "arm_down": ("descer braço", "descer braco", "baixar braço", "baixar braco", "arm down"),
        "fore_up": ("subir antebraço", "subir antebraco", "antebraço para cima", "forearm up"),
        "fore_down": ("descer antebraço", "descer antebraco", "antebraço para baixo", "forearm down"),
        "wrist_up": ("subir punho", "punho para cima", "levantar punho", "wrist up"),
        "wrist_down": ("descer punho", "punho para baixo", "abaixar punho", "wrist down"),
        "wrist_left": ("punho esquerda", "girar punho esquerda", "rotacionar punho esquerda", "wrist left"),
        "wrist_right": ("punho direita", "girar punho direita", "rotacionar punho direita", "wrist right"),
        "roll_left": ("rolar antebraço esquerda", "rolar antebraco esquerda", "girar antebraço esquerda", "forearm roll left"),
        "roll_right": ("rolar antebraço direita", "rolar antebraco direita", "girar antebraço direita", "forearm roll right"),
        "grip_open": ("abrir garra", "abrir pinça", "abrir pinca", "soltar objeto", "open grip"),
        "grip_close": ("fechar garra", "fechar pinça", "fechar pinca", "close grip"),
        "home": ("home", "posição inicial", "posicao inicial", "voltar home", "return home"),
    },
    missions={
        "inspect_workspace": ("missão inspeção", "missao inspecao", "inspecionar bancada", "inspect workspace"),
        "scan_workspace": ("scan 360", "varredura 360", "girar 360", "varrer bancada", "fazer varredura 360", "varrer 360", "varrer 360 graus", "360 degree sweep", "full scan"),
        "pick_object": ("pegar objeto", "missão pegar objeto", "pegar objeto missão", "capturar objeto", "agarrar objeto", "grab object", "grasp object"),
        "place_object": ("colocar objeto", "depositar objeto", "place object"),
        "hold_pose": ("manter posição", "manter posicao", "hold pose"),
        "return_home": ("retornar home", "missão home", "missao home"),
        "calibrate_gripper": ("calibrar garra", "testar garra"),
        "rhythm_mode": ("modo música", "modo musica", "modo ritmo", "dançar", "dancar", "seguir ritmo"),
        "wave": ("acenar", "dar tchau"),
        "stop_mission": ("parar missão", "parar missao", "cancelar missão", "cancelar missao", "stop mission"),
    },
)

VEHICLE_PROFILE = RobotProfile(
    "Vehicle",
    4,
    False,
    intents={
        "forward": ("veículo frente", "veiculo frente", "andar para frente", "avançar", "avancar"),
        "back": ("veículo ré", "veiculo re", "dar ré", "dar re", "recuar"),
        "left": ("veículo esquerda", "veiculo esquerda", "virar esquerda", "curva esquerda"),
        "right": ("veículo direita", "veiculo direita", "virar direita", "curva direita"),
        "stop": ("parar veículo", "parar veiculo", "frear", "vehicle stop", "stop"),
        "camera_left": ("câmera esquerda", "camera esquerda", "olhar esquerda"),
        "camera_right": ("câmera direita", "camera direita", "olhar direita"),
        "camera_up": ("câmera cima", "camera cima", "olhar para cima"),
        "camera_down": ("câmera baixo", "camera baixo", "olhar para baixo"),
        "camera_center": ("centralizar câmera", "centralizar camera", "camera center"),
        "lights_toggle": ("alternar luzes", "ligar ou desligar luzes", "toggle lights"),
        "lidar_toggle": ("alternar lidar", "alternar scan", "toggle lidar", "toggle scan"),
    },
    missions={
        "patrol_area": ("missão patrulha", "missao patrulha", "patrulhar área", "patrulhar area"),
        "perimeter_scan": ("varrer perímetro", "varrer perimetro", "scan perimetro"),
        "corridor_scan": ("inspecionar corredor", "varrer corredor"),
        "follow_target": ("seguir alvo terrestre", "seguir alvo", "seguir objeto"),
        "dock": ("docar", "estacionar", "aproximar base"),
        "hold_position": ("manter posição", "manter posicao", "parar e observar"),
        "return_home": ("retornar home", "voltar base"),
        "stop_mission": ("parar missão", "parar missao", "cancelar missão", "cancelar missao"),
    },
)

DRONE_PROFILE = RobotProfile(
    "Drone",
    6,
    False,
    intents={
        "takeoff": ("decolar", "drone decolar", "takeoff"),
        "land": ("pousar", "drone pousar", "aterrissar", "landing"),
        "up": ("subir drone", "drone subir", "drone up"),
        "down": ("descer drone", "drone descer", "drone down"),
        "forward": ("drone frente", "ir para frente", "drone forward"),
        "back": ("drone ré", "drone re", "drone trás", "drone tras"),
        "left": ("drone esquerda", "deslocar esquerda"),
        "right": ("drone direita", "deslocar direita"),
        "yaw_left": ("girar drone esquerda", "yaw esquerda"),
        "yaw_right": ("girar drone direita", "yaw direita"),
        "hover": ("pairar", "estabilizar drone", "ficar pairando", "hover"),
        "camera_left": ("câmera drone esquerda", "camera drone esquerda"),
        "camera_right": ("câmera drone direita", "camera drone direita"),
        "camera_up": ("subir câmera drone", "subir camera drone"),
        "camera_down": ("descer câmera drone", "descer camera drone"),
        "camera_center": ("centralizar câmera drone", "centralizar camera drone"),
        "camera_stream_toggle": ("alternar câmera drone", "alternar camera drone", "toggle drone camera"),
    },
    missions={
        "aerial_scan": ("missão varredura aérea", "missao varredura aerea", "scan aéreo", "scan aereo"),
        "orbit_point": ("orbitar ponto", "orbitar alvo"),
        "search_pattern": ("buscar alvo", "padrão de busca", "padrao de busca"),
        "altitude_hold": ("manter altitude", "altitude hold"),
        "return_home": ("retornar home", "voltar base"),
        "emergency_land": ("pouso de emergência", "pouso de emergencia"),
        "stop_mission": ("parar missão", "parar missao", "cancelar missão", "cancelar missao"),
    },
)


class DedicatedRobotAI:
    """Common lifecycle; actuator/mission implementations live in subclasses."""

    def __init__(
        self,
        profile: RobotProfile,
        safety: SynROVSafetyLayer,
        *,
        model_root: Optional[Path] = None,
        memory: Optional[LearnedCommandMemory] = None,
    ) -> None:
        self.profile = profile
        self.safety = safety
        self.memory = memory
        self.model_slot = RobotModelSlot.build(profile.robot, model_root)
        self.active_mission: Optional[MissionState] = None
        self.last_command_source = ""
        self.last_command_name = ""
        self.last_command_ts = 0.0

    @property
    def robot(self) -> str:
        return self.profile.robot

    def metadata(self) -> Dict[str, Any]:
        return {
            **self.model_slot.health(),
            "music_enabled": self.profile.music_enabled,
            "learned_commands": len(self.memory.list_for_robot(self.robot)) if self.memory else 0,
            "active_mission": self.active_mission.name if self.active_mission else "",
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
                # Authority only resolves ambiguous phrases; it does not change
                # direct actuator magnitude.
                score = float(raw_score) + command_tuning(self.robot, name).authority
                if score > best_score:
                    best_score = score
                    best_kind = kind
                    best_name = name
        return (best_kind, best_name) if best_score else ("none", "")

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
        age = time.time() - self.last_command_ts if self.last_command_ts else 999.0
        return orchestrator.action_allowed(
            source,
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
        return mission.name if mission else None

    def execute_text(
        self,
        text: str,
        bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        kind, name = self.resolve(text)
        if kind == "intent":
            return self.execute_intent(name, bridge, orchestrator)
        if kind == "mission":
            return self.start_mission(name, bridge, text, orchestrator)
        return CommandResult(False, self.robot, reason="unrecognized_for_active_robot", metadata={"text": text})

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
        if not self._allowed(orchestrator, "voice", name, bridge=bridge):
            return CommandResult(False, self.robot, "mission", name, "blocked_by_priority")
        self.active_mission = MissionState(name, text=text)
        self._record("mission", name, orchestrator)
        return CommandResult(True, self.robot, "mission", name, "started", self.metadata())

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
        return result

    def tick(
        self,
        bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        mission = self.active_mission
        if mission is None:
            return CommandResult(False, self.robot, reason="no_active_mission")
        if not self._allowed(orchestrator, "mission", mission.name, bridge=bridge):
            return CommandResult(False, self.robot, "mission", mission.name, "blocked_by_priority")

        now = time.time()
        elapsed = now - mission.created_ts
        duration, metadata = self._tick_mission(mission, bridge, elapsed, now)
        if duration is None:
            self.active_mission = None
            return CommandResult(False, self.robot, "mission", mission.name, "unknown_mission")

        self._record("mission", mission.name, orchestrator)
        if duration <= 0.0 or self.active_mission is None:
            self.active_mission = None
            return CommandResult(True, self.robot, "mission", mission.name, "done", metadata)
        if elapsed >= duration:
            self.active_mission = None
            self.stop(bridge, "mission", orchestrator)
            return CommandResult(True, self.robot, "mission", mission.name, "done", metadata)
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
    ) -> None:
        super().__init__(profile, safety, model_root=model_root, memory=memory)
        dance_path = self.model_slot.model_dir.parent.parent / "synrov_dance_model_v1.json"
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
            bridge.manip_home()
            return CommandResult(True, self.robot, "intent", name, "home_sent")

        delta: Dict[str, Tuple[str, float]] = {
            "base_left": ("base", +self.STEP_DEG),
            "base_right": ("base", -self.STEP_DEG),
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
        if name == "hold_pose":
            duration = 10.0
        elif name == "scan_workspace":
            duration = 18.0
            start_base = safe_float(mission.data.setdefault("scan_start_base", pose.get("base", 180.0)), 180.0)
            progress = clamp(elapsed / duration, 0.0, 1.0)
            phase = progress * math.tau
            pose["base"] = (start_base + 360.0 * progress) % 360.0
            pose["upper"] = clamp(68 + 18 * math.sin(phase), 0, 359)
            pose["fore"] = clamp(154 + 20 * math.sin(phase + math.pi / 3.0), 0, 359)
            pose["forearm_roll"] = clamp(90 + 34 * math.sin(phase * 1.5), 0, 359)
            pose["wrist_pitch"] = clamp(108 + 24 * math.sin(phase * 2.0 + math.pi / 6.0), 0, 359)
            pose["wrist_rot"] = clamp(130 + 40 * math.sin(phase * 1.25 + math.pi / 4.0), 0, 359)
        elif name == "inspect_workspace":
            pose["base"] = (pose["base"] + 1.4 * math.sin(now * 0.55)) % 360.0
            pose["upper"] = clamp(60 + 12 * math.sin(now * 0.42), 0, 359)
            pose["wrist_pitch"] = clamp(112 + 16 * math.sin(now * 0.68), 0, 359)
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
                pose = dict(DANCE_HOME_POSE)
            elif self.dance_model.due("Manipulator", beat, now=now):
                mission.data["rhythm_wait_home_sent"] = False
                pose = self.dance_model.manipulator_pose(beat, now=now)
            else:
                return 30.0, {"beat_wait": False, "dance_throttled": True}
            duration = 30.0
        elif name == "wave":
            # Wave is a gripper gesture. Keep every other joint in the exact
            # HOME pose so a greeting can never drag the arm away from neutral.
            pose = {
                "base": 180.0,
                "upper": 45.0,
                "fore": 180.0,
                "forearm_roll": 90.0,
                "wrist_pitch": 95.0,
                "wrist_rot": 130.0,
                "grip": clamp(50.0 + 28.0 * math.sin(now * 5.0), 20.0, 80.0),
            }
            duration = 5.0
        else:
            return None, {}

        source = "music" if name == "rhythm_mode" else "mission"
        bridge.command_manipulator_pose(self.safety.sanitize_pose(pose, source=source, smooth=True))
        return duration, {}

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
            "stop": (0.00, 0.00),
        }
        if name in drive_mapping:
            bridge.send_vehicle(*drive_mapping[name])
            return CommandResult(True, self.robot, "intent", name, "drive_sent")

        camera_delta = {
            "camera_left": (5.0, 0.0),
            "camera_right": (-5.0, 0.0),
            "camera_up": (0.0, -5.0),
            "camera_down": (0.0, 5.0),
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
            bridge.set_camera("Vehicle", pan=0.0, tilt=0.0)
            return CommandResult(True, self.robot, "intent", name, "camera_centered")
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
        if name == "hold_position":
            bridge.send_vehicle(0, 0)
            return 12.0, {}
        if name == "patrol_area":
            return 30.0, self._send_adaptive(bridge, 0.18, 0.24 * math.sin(now * 0.42), 25 * math.sin(now * 0.5), 0)
        if name == "perimeter_scan":
            return 22.0, self._send_adaptive(bridge, 0.12, 0.42, 38 * math.sin(now * 0.7), 0)
        if name == "corridor_scan":
            return 28.0, self._send_adaptive(bridge, 0.16, 0.06 * math.sin(now * 0.75), 50 * math.sin(now * 0.9), 10)
        if name == "follow_target":
            return 24.0, self._send_adaptive(bridge, 0.16, 0.18 * math.sin(now * 0.85), 22 * math.sin(now * 0.6), 0)
        if name == "dock":
            distance = obstacle_distance_cm(getattr(bridge, "state", None))
            if distance is not None and distance <= 22.0:
                bridge.send_vehicle(0, 0, 0, -8)
                self.active_mission = None
                return 0.0, {"dock_distance_cm": distance, "dock_reached": True}
            throttle = 0.10 if elapsed < 5.0 else 0.04
            metadata = self._send_adaptive(bridge, throttle, 0, 0, -8)
            metadata["dock_distance_cm"] = distance
            return 8.0, metadata
        if name == "return_home":
            # Reverse remains an escape-capable command and is never stopped by
            # the forward obstacle guard.
            return 8.0, self._send_adaptive(bridge, -0.18, 0)
        return None, {}

    def stop(self, bridge: Any, source: str = "manual", orchestrator: Optional[SynROVOrchestrator] = None) -> None:
        bridge.send_vehicle(0, 0)
        self._record(source, "stop", orchestrator)


class DroneAI(DedicatedRobotAI):
    def _drone_control(self, bridge: Any) -> Dict[str, Any]:
        control = getattr(getattr(bridge, "state", None), "control", {}) or {}
        return control if isinstance(control, dict) else {}

    def _execute_intent(self, name: str, bridge: Any) -> CommandResult:
        if name == "takeoff":
            bridge.drone_takeoff()
            return CommandResult(True, self.robot, "intent", name, "takeoff_sent")
        if name == "land":
            bridge.drone_land()
            return CommandResult(True, self.robot, "intent", name, "land_sent")

        mapping: Dict[str, Tuple[float, float, float, float, float, float]] = {
            "up": (0.30, 0.00, 0.00, 0.00, 0.00, 0.00),
            "down": (-0.24, 0.00, 0.00, 0.00, 0.00, 0.00),
            "forward": (0.00, 0.00, 0.00, 0.00, 0.00, 0.30),
            "back": (0.00, 0.00, 0.00, 0.00, 0.00, -0.26),
            "left": (0.00, 0.00, 0.00, 0.00, 0.22, 0.00),
            "right": (0.00, 0.00, 0.00, 0.00, -0.22, 0.00),
            "yaw_left": (0.00, 0.22, 0.00, 0.00, 0.00, 0.00),
            "yaw_right": (0.00, -0.22, 0.00, 0.00, 0.00, 0.00),
            "hover": (0.00, 0.00, 0.00, 0.00, 0.00, 0.00),
        }
        if name in mapping:
            bridge.send_drone(*mapping[name])
            return CommandResult(True, self.robot, "intent", name, "flight_sent")

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
            bridge.set_camera("Drone", pan=0.0, tilt=0.0)
            return CommandResult(True, self.robot, "intent", name, "camera_centered")
        if name == "camera_stream_toggle":
            camera = self._drone_control(bridge).get("camera", {})
            current = bool(camera.get("streaming", False)) if isinstance(camera, dict) else False
            bridge.set_drone_camera_streaming(not current)
            return CommandResult(True, self.robot, "intent", name, "camera_stream_toggled", {"enabled": not current})
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
        if name == "aerial_scan":
            return 18.0, self._send_adaptive(bridge, (0, 0.24, 0, 0, 0.04 * math.sin(now * 0.5), 0.04))
        if name == "orbit_point":
            return 22.0, self._send_adaptive(bridge, (0, 0.18, 0, 0.08 * math.sin(now * 0.6), 0.18, 0.12))
        if name == "search_pattern":
            return 30.0, self._send_adaptive(bridge, (0, 0.10 * math.sin(now * 0.32), -0.04, 0, 0.22 * math.sin(now * 0.55), 0.16))
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
            metadata.update({"target_alt_cm": target_alt, "current_alt_cm": current_alt})
            return 16.0, metadata
        if name == "return_home":
            return 10.0, self._send_adaptive(bridge, (0, 0, 0.04, 0, 0, -0.18))
        if name == "emergency_land":
            bridge.drone_land()
            self.active_mission = None
            return 0.0, {"autonomous_action": "emergency_land"}
        return None, {}

    def stop(self, bridge: Any, source: str = "manual", orchestrator: Optional[SynROVOrchestrator] = None) -> None:
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
    ) -> None:
        self.safety = safety or SynROVSafetyLayer()
        self.memory = LearnedCommandMemory(memory_path)
        self.ais: Dict[str, DedicatedRobotAI] = {
            "Manipulator": ManipulatorAI(MANIPULATOR_PROFILE, self.safety, model_root=model_root, memory=self.memory),
            "Vehicle": VehicleAI(VEHICLE_PROFILE, self.safety, model_root=model_root, memory=self.memory),
            "Drone": DroneAI(DRONE_PROFILE, self.safety, model_root=model_root, memory=self.memory),
        }
        self._active_robot = ""

    def get(self, robot: Any) -> DedicatedRobotAI:
        return self.ais[canonical_robot(robot)]

    def sync_active_robot(self, bridge: Any) -> DedicatedRobotAI:
        current = canonical_robot(getattr(getattr(bridge, "state", None), "robot", "Manipulator"))
        if self._active_robot and self._active_robot != current:
            # Do not send a stop to the old robot here: that would make
            # Processing select the old mode again.  Only cancel stale Python
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
