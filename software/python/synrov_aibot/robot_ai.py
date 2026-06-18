"""Dedicated AI profiles for Manipulator, Vehicle and Drone.

The runtime selects one profile based on the robot identity received through
the SynROV WebSocket.  Each robot owns its own intents, missions, model slot
and runtime state.  Music/rhythm is only available on the Manipulator profile.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Tuple

from .primitives import normalize_text
from .dataset import TARGET_DIMS, canonical_robot
from .orchestrator import SynROVOrchestrator
from .safety import MANIP_KEYS, MANIP_POSE_LIMITS, SynROVSafetyLayer, clamp


def match_alias(text: Any, aliases: Iterable[str]) -> int:
    t = normalize_text(text)
    best = 0
    for alias in aliases or []:
        a = normalize_text(alias)
        if t == a:
            best = max(best, 1000 + len(a))
        elif a and a in t:
            best = max(best, 100 + len(a))
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
            name,
            int(TARGET_DIMS.get(name, 7)),
            model_dir,
            model_dir / f"{key}_policy.pkl",
            dataset_path,
        )


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


class LearnedCommandMemory:
    """Persistent, user-teachable command aliases for each robot profile.

    The fixed base profiles keep the robot safe and predictable.  This memory
    adds small learned language mappings such as "pegar a peça vermelha" →
    Manipulator mission pick_object without changing source code.
    """

    def __init__(self, path: Optional[Path] = None) -> None:
        base = Path(
            path
            or Path(__file__).resolve().parent / "synrov_multimodal_data" / "learned_commands.json"
        )
        self.path = base
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._data: Dict[str, Dict[str, Dict[str, str]]] = {}
        self.load()


    def load(self) -> None:
        try:
            raw = json.loads(self.path.read_text("utf-8")) if self.path.exists() else {}
        except Exception:
            raw = {}

        clean: Dict[str, Dict[str, Dict[str, str]]] = {}
        if isinstance(raw, dict):
            for robot, entries in raw.items():
                r = canonical_robot(robot)
                clean.setdefault(r, {})
                if isinstance(entries, dict):
                    for phrase, item in entries.items():
                        if not isinstance(item, dict):
                            continue
                        kind = str(item.get("kind", "")).strip().lower()
                        name = str(item.get("name", "")).strip()
                        if kind in {"intent", "mission"} and name:
                            clean[r][normalize_text(phrase)] = {"kind": kind, "name": name}
        self._data = clean

    def save(self) -> None:
        try:
            self.path.write_text(json.dumps(self._data, ensure_ascii=False, indent=2), "utf-8")
        except Exception:
            pass

    def learn(self, robot: Any, phrase: Any, kind: str, name: str) -> bool:
        r = canonical_robot(robot)
        key = normalize_text(phrase)
        k = str(kind or "").strip().lower()
        n = str(name or "").strip()
        if not key or k not in {"intent", "mission"} or not n:
            return False
        self._data.setdefault(r, {})[key] = {"kind": k, "name": n}
        self.save()
        return True

    def forget(self, robot: Any, phrase: Any) -> bool:
        r = canonical_robot(robot)
        key = normalize_text(phrase)
        if key and key in self._data.get(r, {}):
            del self._data[r][key]
            self.save()
            return True
        return False

    def resolve(self, robot: Any, text: Any) -> Tuple[str, str]:
        r = canonical_robot(robot)
        query = normalize_text(text)
        best = (0, "none", "")
        for phrase, item in self._data.get(r, {}).items():
            if not phrase:
                continue
            if query == phrase:
                score = 2000 + len(phrase)
            elif phrase in query or query in phrase:
                score = 300 + min(len(phrase), len(query))
            else:
                score = 0
            if score > best[0]:
                best = (score, str(item.get("kind", "none")), str(item.get("name", "")))
        return (best[1], best[2]) if best[0] else ("none", "")

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
        "base": 180, "upper": 45, "fore": 180,
        "forearm_roll": 90, "wrist_pitch": 95, "wrist_rot": 130, "grip": 50,
    }
    sensor_names = {
        "base": "base_deg", "upper": "upper_deg", "fore": "fore_deg",
        "forearm_roll": "forearm_roll_deg", "wrist_pitch": "wrist_pitch_deg",
        "wrist_rot": "wrist_rot_deg", "grip": "grip_deg",
    }
    return {
        k: clamp(servos.get(i, sensors.get(sensor_names[k], defaults[k])), *MANIP_POSE_LIMITS[k])
        for i, k in enumerate(MANIP_KEYS)
    }


MANIPULATOR_PROFILE = RobotProfile(
    "Manipulator", 7, True,
    intents={
        "base_left": (
            "base esquerda",
            "virar base esquerda",
            "girar base esquerda",
            "rotacionar base esquerda",
            "base left",
        ),
        "base_right": (
            "base direita",
            "virar base direita",
            "girar base direita",
            "rotacionar base direita",
            "base right",
        ),
        "arm_up":       ("subir braço", "subir braco", "levantar braço", "levantar braco", "arm up"),
        "arm_down":     ("descer braço", "descer braco", "baixar braço", "baixar braco", "arm down"),
        "fore_up":      ("subir antebraço", "subir antebraco", "antebraço para cima", "forearm up"),
        "fore_down":    ("descer antebraço", "descer antebraco", "antebraço para baixo", "forearm down"),
        "wrist_up":     ("subir punho", "punho para cima", "levantar punho", "wrist up"),
        "wrist_down":   ("descer punho", "punho para baixo", "abaixar punho", "wrist down"),
        "wrist_left":   ("punho esquerda", "girar punho esquerda", "rotacionar punho esquerda", "wrist left"),
        "wrist_right":  ("punho direita", "girar punho direita", "rotacionar punho direita", "wrist right"),
        "roll_left": (
            "rolar antebraço esquerda",
            "rolar antebraco esquerda",
            "girar antebraço esquerda",
            "forearm roll left",
        ),
        "roll_right": (
            "rolar antebraço direita",
            "rolar antebraco direita",
            "girar antebraço direita",
            "forearm roll right",
        ),
        "grip_open":    ("abrir garra", "abrir pinça", "abrir pinca", "soltar objeto", "open grip"),
        "grip_close":   ("fechar garra", "fechar pinça", "fechar pinca", "pegar objeto", "close grip"),
        "home":         ("home", "posição inicial", "posicao inicial", "voltar home", "return home"),
    },
    missions={
        "inspect_workspace":  ("missão inspeção", "missao inspecao", "inspecionar bancada", "inspect workspace"),
        "scan_workspace":     ("scan 360", "varredura 360", "girar 360", "varrer bancada", "fazer varredura 360", "varrer 360", "varrer 360 graus", "360 degree sweep", "full scan"),
        "pick_object":        ("missão pegar objeto", "pegar objeto missão", "capturar objeto", "agarrar objeto"),
        "place_object":       ("colocar objeto", "depositar objeto", "place object"),
        "hold_pose":          ("manter posição", "manter posicao", "hold pose"),
        "return_home":        ("retornar home", "missão home", "missao home"),
        "calibrate_gripper":  ("calibrar garra", "testar garra"),
        "rhythm_mode":        ("modo música", "modo musica", "modo ritmo", "dançar", "dancar", "seguir ritmo"),
        "wave":               ("acenar", "dar tchau"),
        "stop_mission":       ("parar missão", "parar missao", "cancelar missão", "cancelar missao", "stop mission"),
    },
)

VEHICLE_PROFILE = RobotProfile(
    "Vehicle", 4, False,
    intents={
        "forward":      ("veículo frente", "veiculo frente", "andar para frente", "avançar", "avancar"),
        "back":         ("veículo ré", "veiculo re", "dar ré", "dar re", "recuar"),
        "left":         ("veículo esquerda", "veiculo esquerda", "virar esquerda", "curva esquerda"),
        "right":        ("veículo direita", "veiculo direita", "virar direita", "curva direita"),
        "stop":         ("parar veículo", "parar veiculo", "frear", "vehicle stop", "stop"),
        "camera_left":  ("câmera esquerda", "camera esquerda", "olhar esquerda"),
        "camera_right": ("câmera direita", "camera direita", "olhar direita"),
        "camera_up":    ("câmera cima", "camera cima", "olhar para cima"),
        "camera_down":  ("câmera baixo", "camera baixo", "olhar para baixo"),
    },
    missions={
        "patrol_area":    ("missão patrulha", "missao patrulha", "patrulhar área", "patrulhar area"),
        "perimeter_scan": ("varrer perímetro", "varrer perimetro", "scan perimetro"),
        "corridor_scan":  ("inspecionar corredor", "varrer corredor"),
        "follow_target":  ("seguir alvo terrestre", "seguir alvo", "seguir objeto"),
        "dock":           ("docar", "estacionar", "aproximar base"),
        "hold_position":  ("manter posição", "manter posicao", "parar e observar"),
        "return_home":    ("retornar home", "voltar base"),
        "stop_mission":   ("parar missão", "parar missao", "cancelar missão", "cancelar missao"),
    },
)

DRONE_PROFILE = RobotProfile(
    "Drone", 6, False,
    intents={
        "takeoff":   ("decolar", "drone decolar", "takeoff"),
        "land":      ("pousar", "drone pousar", "aterrissar", "landing"),
        "up":        ("subir drone", "drone subir", "drone up"),
        "down":      ("descer drone", "drone descer", "drone down"),
        "forward":   ("drone frente", "ir para frente", "drone forward"),
        "back":      ("drone ré", "drone re", "drone trás", "drone tras"),
        "left":      ("drone esquerda", "deslocar esquerda"),
        "right":     ("drone direita", "deslocar direita"),
        "yaw_left":  ("girar drone esquerda", "yaw esquerda"),
        "yaw_right": ("girar drone direita", "yaw direita"),
        "hover":     ("pairar", "manter altitude", "hover"),
    },
    missions={
        "aerial_scan":    ("missão varredura aérea", "missao varredura aerea", "scan aéreo", "scan aereo"),
        "orbit_point":    ("orbitar ponto", "orbitar alvo"),
        "search_pattern": ("buscar alvo", "padrão de busca", "padrao de busca"),
        "altitude_hold":  ("manter altitude", "altitude hold"),
        "return_home":    ("retornar home", "voltar base"),
        "emergency_land": ("pouso de emergência", "pouso de emergencia"),
        "stop_mission":   ("parar missão", "parar missao", "cancelar missão", "cancelar missao"),
    },
)


class DedicatedRobotAI:
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
        self.last_command_ts = 0.0

    @property
    def robot(self) -> str:
        return self.profile.robot

    def metadata(self) -> Dict[str, Any]:
        return {
            "robot": self.robot,
            "target_dim": self.profile.target_dim,
            "music_enabled": self.profile.music_enabled,
            "policy_path": str(self.model_slot.policy_path),
            "dataset_path": str(self.model_slot.dataset_path),
            "learned_commands": len(self.memory.list_for_robot(self.robot)) if self.memory else 0,
        }

    def resolve(self, text: Any) -> Tuple[str, str]:
        if self.memory is not None:
            learned_kind, learned_name = self.memory.resolve(self.robot, text)
            if learned_kind != "none" and learned_name:
                return learned_kind, learned_name
        best = (0, "none", "")
        for name, aliases in self.profile.missions.items():
            score = match_alias(text, aliases)
            if score > best[0]:
                best = (score, "mission", name)
        for name, aliases in self.profile.intents.items():
            score = match_alias(text, aliases)
            if score > best[0]:
                best = (score, "intent", name)
        return (best[1], best[2]) if best[0] else ("none", "")


    def _allowed(self, orchestrator: Optional[SynROVOrchestrator], source: str) -> bool:
        if orchestrator is None:
            return True
        age = time.time() - self.last_command_ts if self.last_command_ts else 999.0
        return orchestrator.action_allowed(
            source,
            telemetry_fresh=True,
            mission_active=self.active_mission is not None and source != "mission",
            last_command_source=self.last_command_source,
            last_command_age_s=age,
        ).allowed

    def _record(self, source: str, orchestrator: Optional[SynROVOrchestrator]) -> None:
        self.last_command_source = source
        self.last_command_ts = time.time()
        if orchestrator:
            orchestrator.register_action(source, self.last_command_ts)


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
            self.active_mission = None
            self.stop(bridge, "mission", orchestrator)
            return CommandResult(True, self.robot, "mission", name, "stopped")
        if name == "rhythm_mode" and not self.profile.music_enabled:
            return CommandResult(False, self.robot, "mission", name, "music_only_for_manipulator")
        if not self._allowed(orchestrator, "voice"):
            return CommandResult(False, self.robot, "mission", name, "blocked_by_priority")
        self.active_mission = MissionState(name, text=text)
        self._record("mission", orchestrator)
        return CommandResult(True, self.robot, "mission", name, "started", self.metadata())

    def execute_intent(
        self,
        name: str,
        bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        if not self._allowed(orchestrator, "voice"):
            return CommandResult(False, self.robot, "intent", name, "blocked_by_priority")
        if self.robot == "Manipulator":
            result = self._intent_manipulator(name, bridge)
        elif self.robot == "Vehicle":
            result = self._intent_vehicle(name, bridge)
        else:
            result = self._intent_drone(name, bridge)
        if result.ok:
            self._record("voice", orchestrator)
        return result


    def _intent_manipulator(self, name: str, bridge: Any) -> CommandResult:
        if name == "home":
            bridge.manip_home()
            return CommandResult(True, self.robot, "intent", name, "home_sent")

        step = 24.0


        delta: Dict[str, Tuple[str, float]] = {
            "base_left":    ("base",         +step),
            "base_right":   ("base",         -step),
            "arm_up":       ("upper",        +step),
            "arm_down":     ("upper",        -step),
            "fore_up":      ("fore",         +step),
            "fore_down":    ("fore",         -step),
            "wrist_up":     ("wrist_pitch",  +step),
            "wrist_down":   ("wrist_pitch",  -step),
            "wrist_left":   ("wrist_rot",    +step),
            "wrist_right":  ("wrist_rot",    -step),
            "roll_left":    ("forearm_roll", -step),
            "roll_right":   ("forearm_roll", +step),
            "grip_open":    ("grip",         +step),
            "grip_close":   ("grip",         -step),
        }
        if name not in delta:
            return CommandResult(False, self.robot, "intent", name, "unknown_intent")

        key, amount = delta[name]
        pose = manip_pose(getattr(bridge, "state", None))
        lo, hi = MANIP_POSE_LIMITS[key]
        if key == "base":
            pose[key] = (pose[key] + amount) % 360.0
        else:
            pose[key] = clamp(pose[key] + amount, lo, hi)
        bridge.command_manipulator_pose(self.safety.sanitize_pose(pose, source="voice", smooth=True))
        return CommandResult(True, self.robot, "intent", name, "pose_sent")

    def _intent_vehicle(self, name: str, bridge: Any) -> CommandResult:
        mapping: Dict[str, Tuple[float, float, float, float]] = {
            "forward":       ( 0.34,  0.00,   0,   0),
            "back":          (-0.28,  0.00,   0,   0),
            "left":          ( 0.14, -0.34,   0,   0),
            "right":         ( 0.14,  0.34,   0,   0),
            "stop":          ( 0.00,  0.00,   0,   0),
            "camera_left":   ( 0.00,  0.00, -45,   0),
            "camera_right":  ( 0.00,  0.00,  45,   0),
            "camera_up":     ( 0.00,  0.00,   0,  20),
            "camera_down":   ( 0.00,  0.00,   0, -20),
        }
        if name not in mapping:
            return CommandResult(False, self.robot, "intent", name, "unknown_intent")
        bridge.send_vehicle(*mapping[name])
        return CommandResult(True, self.robot, "intent", name, "drive_sent")

    def _intent_drone(self, name: str, bridge: Any) -> CommandResult:
        if name == "takeoff":
            bridge.drone_takeoff()
            return CommandResult(True, self.robot, "intent", name, "takeoff_sent")
        if name == "land":
            bridge.drone_land()
            return CommandResult(True, self.robot, "intent", name, "land_sent")

        mapping: Dict[str, Tuple[float, float, float, float, float, float]] = {
            "up":        ( 0.30,  0.00,  0.00, 0.00,  0.00,  0.00),
            "down":      (-0.24,  0.00,  0.00, 0.00,  0.00,  0.00),
            "forward":   ( 0.00,  0.00, -0.08, 0.00,  0.00,  0.30),
            "back":      ( 0.00,  0.00,  0.08, 0.00,  0.00, -0.26),
            "left":      ( 0.00,  0.00,  0.00, 0.00, -0.22,  0.00),
            "right":     ( 0.00,  0.00,  0.00, 0.00,  0.22,  0.00),
            "yaw_left":  ( 0.00, -0.22,  0.00, 0.00,  0.00,  0.00),
            "yaw_right": ( 0.00,  0.22,  0.00, 0.00,  0.00,  0.00),
            "hover":     ( 0.00,  0.00,  0.00, 0.00,  0.00,  0.00),
        }
        if name not in mapping:
            return CommandResult(False, self.robot, "intent", name, "unknown_intent")
        bridge.send_drone(*mapping[name])
        return CommandResult(True, self.robot, "intent", name, "flight_sent")


    def stop(
        self,
        bridge: Any,
        source: str = "manual",
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> None:
        if self.robot == "Manipulator":
            bridge.command_manipulator_pose(manip_pose(getattr(bridge, "state", None)))
        elif self.robot == "Vehicle":
            bridge.send_vehicle(0, 0, 0, 0)
        else:
            bridge.send_drone(0, 0, 0, 0, 0, 0)
        self._record(source, orchestrator)


    def tick(
        self,
        bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        mission = self.active_mission
        if mission is None:
            return CommandResult(False, self.robot, reason="no_active_mission")

        elapsed = time.time() - mission.created_ts
        t = time.time()
        done = 10.0

        if self.robot == "Manipulator":
            done = self._tick_manipulator(mission, bridge, elapsed, t)
        elif self.robot == "Vehicle":
            done = self._tick_vehicle(mission, bridge, t)
        else:
            done = self._tick_drone(mission, bridge, t)

        if done is None:

            self.active_mission = None
            return CommandResult(False, self.robot, "mission", mission.name, "unknown_mission")

        self._record("mission", orchestrator)
        if done <= 0.0 or self.active_mission is None:
            self.active_mission = None
            return CommandResult(True, self.robot, "mission", mission.name, "done")
        if elapsed >= done:
            self.active_mission = None
            self.stop(bridge, "mission", orchestrator)
            return CommandResult(True, self.robot, "mission", mission.name, "done")
        return CommandResult(True, self.robot, "mission", mission.name, "running", {"elapsed_s": elapsed})

    def _tick_manipulator(
        self, mission: MissionState, bridge: Any, elapsed: float, t: float,
    ) -> Optional[float]:
        pose = manip_pose(getattr(bridge, "state", None))
        name = mission.name
        done: Optional[float] = None

        if name == "return_home":
            bridge.manip_home()
            self.active_mission = None
            return 0.0

        if name == "hold_pose":
            done = 10.0
        elif name == "scan_workspace":
            duration = 18.0
            if not hasattr(mission, "scan_start_base"):
                mission.scan_start_base = pose.get("base", 180.0)
            progress = clamp(elapsed / duration, 0.0, 1.0)
            phase = progress * math.tau
            pose["base"] = (float(getattr(mission, "scan_start_base", 180.0)) + 360.0 * progress) % 360
            pose["upper"] = clamp(68 + 18 * math.sin(phase), 0, 359)
            pose["fore"] = clamp(154 + 20 * math.sin(phase + math.pi / 3.0), 0, 359)
            pose["forearm_roll"] = clamp(90 + 34 * math.sin(phase * 1.5), 0, 359)
            pose["wrist_pitch"] = clamp(108 + 24 * math.sin(phase * 2.0 + math.pi / 6.0), 0, 359)
            pose["wrist_rot"] = clamp(130 + 40 * math.sin(phase * 1.25 + math.pi / 4.0), 0, 359)
            done = duration
        elif name == "inspect_workspace":
            pose["base"] = (pose["base"] + 1.4 * math.sin(t * 0.55)) % 360
            pose["upper"] = clamp(60 + 12 * math.sin(t * 0.42), 0, 359)
            pose["wrist_pitch"] = clamp(112 + 16 * math.sin(t * 0.68), 0, 359)
            done = 24.0
        elif name == "pick_object":
            pose.update({
                "upper":       42 if elapsed < 4.4 else 72,
                "fore":       130 if elapsed < 4.4 else 168,
                "wrist_pitch": 82 if elapsed < 4.4 else 110,
                "grip":        78 if elapsed < 2.2 else 25,
            })
            done = 8.5
        elif name == "place_object":
            pose.update({
                "upper": 52, "fore": 144, "wrist_pitch": 92,
                "grip": 82 if elapsed > 2 else pose["grip"],
            })
            done = 6.5
        elif name == "calibrate_gripper":
            pose["grip"] = 15 if int(elapsed * 1.6) % 2 == 0 else 85
            done = 6.0
        elif name == "rhythm_mode":
            pose.update({
                "base":       (180.0 + clamp(20 * math.sin(t * 1.25), -32, 32)) % 360,
                "forearm_roll": clamp(90  + 28 * math.sin(t * 2.00), 0, 359),
                "wrist_pitch":  clamp(108 + 22 * math.sin(t * 1.65), 0, 359),
                "wrist_rot":    clamp(130 + 32 * math.sin(t * 2.40), 0, 359),
            })
            done = 30.0
        elif name == "wave":
            pose.update({
                "upper": 72, "fore": 150,
                "wrist_rot": clamp(130 + 38 * math.sin(t * 4), 0, 359),
            })
            done = 7.0
        else:
            return None

        source = "music" if name == "rhythm_mode" else "mission"
        bridge.command_manipulator_pose(self.safety.sanitize_pose(pose, source=source, smooth=True))
        return done

    def _tick_vehicle(self, mission: MissionState, bridge: Any, t: float) -> Optional[float]:
        name = mission.name
        if name == "hold_position":
            bridge.send_vehicle(0, 0, 0, 0)
            return 12.0
        if name == "patrol_area":
            bridge.send_vehicle(0.18, 0.24 * math.sin(t * 0.42), 25 * math.sin(t * 0.5), 0)
            return 30.0
        if name == "perimeter_scan":
            bridge.send_vehicle(0.12, 0.42, 38 * math.sin(t * 0.7), 0)
            return 22.0
        if name == "corridor_scan":
            bridge.send_vehicle(0.16, 0.06 * math.sin(t * 0.75), 50 * math.sin(t * 0.9), 10)
            return 28.0
        if name == "follow_target":
            bridge.send_vehicle(0.16, 0.18 * math.sin(t * 0.85), 22 * math.sin(t * 0.6), 0)
            return 24.0
        if name == "dock":
            bridge.send_vehicle(0.10 if (time.time() - self.active_mission.created_ts) < 5 else 0.04, 0, 0, -8)
            return 8.0
        if name == "return_home":
            bridge.send_vehicle(-0.18, 0, 0, 0)
            return 8.0
        return None

    def _tick_drone(self, mission: MissionState, bridge: Any, t: float) -> Optional[float]:
        name = mission.name
        if name == "aerial_scan":
            bridge.send_drone(0, 0.24, 0, 0, 0.04 * math.sin(t * 0.5), 0.04)
            return 18.0
        if name == "orbit_point":
            bridge.send_drone(0, 0.18, 0, 0.08 * math.sin(t * 0.6), 0.18, 0.12)
            return 22.0
        if name == "search_pattern":
            bridge.send_drone(0, 0.10 * math.sin(t * 0.32), -0.04, 0, 0.22 * math.sin(t * 0.55), 0.16)
            return 30.0
        if name == "altitude_hold":
            bridge.send_drone(0, 0, 0, 0, 0, 0)
            return 16.0
        if name == "return_home":
            bridge.send_drone(0, 0, 0.04, 0, 0, -0.18)
            return 10.0
        if name == "emergency_land":
            bridge.drone_land()
            self.active_mission = None
            return 0.0
        return None


class RobotAIRegistry:
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
            "Manipulator": DedicatedRobotAI(
                MANIPULATOR_PROFILE,
                self.safety,
                model_root=model_root,
                memory=self.memory,
            ),
            "Vehicle": DedicatedRobotAI(
                VEHICLE_PROFILE,
                self.safety,
                model_root=model_root,
                memory=self.memory,
            ),
            "Drone": DedicatedRobotAI(
                DRONE_PROFILE,
                self.safety,
                model_root=model_root,
                memory=self.memory,
            ),
        }

    def get(self, robot: Any) -> DedicatedRobotAI:
        return self.ais[canonical_robot(robot)]

    def _active_ai(self, bridge: Any) -> DedicatedRobotAI:
        return self.get(getattr(getattr(bridge, "state", None), "robot", "Manipulator"))

    def execute_text(
        self,
        text: str,
        bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        return self._active_ai(bridge).execute_text(text, bridge, orchestrator)

    def tick_active(
        self,
        bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> CommandResult:
        return self._active_ai(bridge).tick(bridge, orchestrator)

    def stop_active(
        self,
        bridge: Any,
        orchestrator: Optional[SynROVOrchestrator] = None,
    ) -> None:
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
