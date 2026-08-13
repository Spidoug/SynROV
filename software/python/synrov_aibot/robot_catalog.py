"""Robot-specific command and intention catalog for SynROV AiBot Version 1.

There are no global commands.  Every phrase resolves only inside the currently
selected robot catalog.  ``command`` is the phrase shown to the operator;
``intent`` is the stable semantic identifier executed by that robot AI.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping, Tuple

from .primitives import normalize_text
from .robot_types import ROBOTS, canonical_robot
from .ui_i18n import DEFAULT_LANGUAGE, text as ui_text


@dataclass(frozen=True)
class CommandSpec:
    intent: str
    command_pt: str
    command_en: str
    aliases: Tuple[str, ...]
    description_pt: str
    description_en: str
    key_hint: str = ""
    kind: str = "command"

    def command(self, language: str = DEFAULT_LANGUAGE) -> str:
        label = ui_text(language, self.command_en)
        return f"{label} [{self.key_hint}]" if self.key_hint else label

    def description(self, language: str = DEFAULT_LANGUAGE) -> str:
        return ui_text(language, self.description_en)


@dataclass(frozen=True)
class MissionSpec:
    intent: str
    command_pt: str
    command_en: str
    aliases: Tuple[str, ...]
    description_pt: str
    description_en: str
    kind: str = "mission"

    def command(self, language: str = DEFAULT_LANGUAGE) -> str:
        return ui_text(language, self.command_en)

    def description(self, language: str = DEFAULT_LANGUAGE) -> str:
        return ui_text(language, self.description_en)


def _c(intent: str, pt: str, en: str, aliases: Iterable[str], dpt: str, den: str, key_hint: str = "") -> CommandSpec:
    vals = tuple(dict.fromkeys([pt, en, *aliases]))
    return CommandSpec(intent, pt, en, vals, dpt, den, key_hint)


def _m(intent: str, pt: str, en: str, aliases: Iterable[str], dpt: str, den: str) -> MissionSpec:
    vals = tuple(dict.fromkeys([pt, en, *aliases]))
    return MissionSpec(intent, pt, en, vals, dpt, den)


COMMANDS: Dict[str, Tuple[CommandSpec, ...]] = {
    "Manipulator": (
        _c("base_left", "girar base esquerda", "rotate base left", ("base esquerda", "virar base esquerda", "rotacionar base esquerda", "base left"), "Gira somente a base para a esquerda.", "Rotates only the base to the left.", "Z"),
        _c("base_right", "girar base direita", "rotate base right", ("base direita", "virar base direita", "rotacionar base direita", "base right"), "Gira somente a base para a direita.", "Rotates only the base to the right.", "X"),
        _c("arm_up", "subir braço", "arm up", ("subir braco", "levantar braço", "levantar braco"), "Eleva o braço superior.", "Raises the upper arm.", "S"),
        _c("arm_down", "descer braço", "arm down", ("descer braco", "baixar braço", "baixar braco"), "Abaixa o braço superior.", "Lowers the upper arm.", "W"),
        _c("fore_up", "subir antebraço", "forearm up", ("subir antebraco", "antebraço para cima"), "Eleva o antebraço.", "Raises the forearm.", "F"),
        _c("fore_down", "descer antebraço", "forearm down", ("descer antebraco", "antebraço para baixo"), "Abaixa o antebraço.", "Lowers the forearm.", "R"),
        _c("wrist_up", "subir punho", "wrist up", ("punho para cima", "levantar punho"), "Inclina o punho para cima.", "Tilts the wrist upward.", "G"),
        _c("wrist_down", "descer punho", "wrist down", ("punho para baixo", "abaixar punho"), "Inclina o punho para baixo.", "Tilts the wrist downward.", "T"),
        _c("wrist_left", "girar punho esquerda", "wrist left", ("punho esquerda", "rotacionar punho esquerda"), "Gira o punho para a esquerda.", "Rotates the wrist left.", "Q"),
        _c("wrist_right", "girar punho direita", "wrist right", ("punho direita", "rotacionar punho direita"), "Gira o punho para a direita.", "Rotates the wrist right.", "E"),
        _c("roll_left", "rolar antebraço esquerda", "forearm roll left", ("rolar antebraco esquerda", "girar antebraço esquerda"), "Rotaciona o antebraço para a esquerda.", "Rolls the forearm left.", "A"),
        _c("roll_right", "rolar antebraço direita", "forearm roll right", ("rolar antebraco direita", "girar antebraço direita"), "Rotaciona o antebraço para a direita.", "Rolls the forearm right.", "D"),
        _c("grip_open", "abrir garra", "open grip", ("abrir pinça", "abrir pinca", "soltar objeto"), "Abre a garra.", "Opens the gripper.", "Y"),
        _c("grip_close", "fechar garra", "close grip", ("fechar pinça", "fechar pinca"), "Fecha a garra.", "Closes the gripper.", "H"),
        _c("home", "posição inicial", "return home", ("home", "posicao inicial", "voltar home"), "Retorna o Manipulator à posição inicial.", "Returns the Manipulator to its home pose."),
    ),
    "Vehicle": (
        _c("forward", "andar para frente", "vehicle forward", ("veículo frente", "veiculo frente", "avançar", "avancar"), "Move o veículo para frente.", "Moves the vehicle forward.", "W"),
        _c("back", "dar ré", "vehicle reverse", ("veículo ré", "veiculo re", "dar re", "recuar"), "Move o veículo para trás.", "Moves the vehicle backward.", "S"),
        _c("left", "virar esquerda", "vehicle left", ("veículo esquerda", "veiculo esquerda", "curva esquerda"), "Conduz o veículo para a esquerda.", "Steers the vehicle left.", "A"),
        _c("right", "virar direita", "vehicle right", ("veículo direita", "veiculo direita", "curva direita"), "Conduz o veículo para a direita.", "Steers the vehicle right.", "D"),
        _c("pivot_left", "girar no eixo esquerda", "pivot left", ("pivot esquerda", "giro esquerda", "spin left"), "Gira o Vehicle no próprio eixo para a esquerda.", "Pivots the Vehicle left in place.", "Q"),
        _c("pivot_right", "girar no eixo direita", "pivot right", ("pivot direita", "giro direita", "spin right"), "Gira o Vehicle no próprio eixo para a direita.", "Pivots the Vehicle right in place.", "E"),
        _c("stop", "parar veículo", "vehicle stop", ("parar veiculo", "frear", "stop"), "Interrompe o movimento do veículo.", "Stops vehicle motion."),
        _c("camera_left", "câmera esquerda", "camera left", ("camera esquerda", "olhar esquerda"), "Move o gimbal da câmera para a esquerda.", "Moves the camera gimbal left.", "Z"),
        _c("camera_right", "câmera direita", "camera right", ("camera direita", "olhar direita"), "Move o gimbal da câmera para a direita.", "Moves the camera gimbal right.", "X"),
        _c("camera_up", "câmera cima", "camera up", ("camera cima", "olhar para cima"), "Move o gimbal da câmera para cima.", "Moves the camera gimbal upward.", "F"),
        _c("camera_down", "câmera baixo", "camera down", ("camera baixo", "olhar para baixo"), "Move o gimbal da câmera para baixo.", "Moves the camera gimbal downward.", "R"),
        _c("camera_center", "centralizar câmera", "camera center", ("centralizar camera",), "Retorna o gimbal ao centro.", "Returns the gimbal to center."),
        _c("camera_toggle", "alternar câmera", "toggle camera", ("alternar camera",), "Alterna a câmera do Vehicle.", "Toggles the Vehicle camera."),
        _c("lights_toggle", "alternar luzes", "toggle lights", ("ligar ou desligar luzes",), "Alterna as luzes do Vehicle.", "Toggles Vehicle lights."),
        _c("lidar_toggle", "alternar lidar", "toggle lidar", ("alternar scan", "toggle scan"), "Alterna a varredura LiDAR.", "Toggles LiDAR scanning."),
    ),
    "Drone": (
        _c("takeoff", "decolar", "takeoff", ("drone decolar",), "Inicia decolagem controlada.", "Starts controlled takeoff."),
        _c("land", "pousar", "land", ("drone pousar", "aterrissar", "landing"), "Inicia pouso controlado.", "Starts controlled landing."),
        _c("up", "subir drone", "drone up", ("drone subir",), "Aumenta altitude.", "Increases altitude.", "T"),
        _c("down", "descer drone", "drone down", ("drone descer",), "Reduz altitude.", "Decreases altitude.", "G"),
        _c("forward", "drone frente", "drone forward", ("ir para frente",), "Desloca o Drone para frente.", "Moves the Drone forward.", "W"),
        _c("back", "drone ré", "drone back", ("drone re", "drone trás", "drone tras"), "Desloca o Drone para trás.", "Moves the Drone backward.", "S"),
        _c("left", "drone esquerda", "drone left", ("deslocar esquerda",), "Desloca lateralmente para a esquerda.", "Strafes left.", "A"),
        _c("right", "drone direita", "drone right", ("deslocar direita",), "Desloca lateralmente para a direita.", "Strafes right.", "D"),
        _c("yaw_left", "girar drone esquerda", "yaw left", ("yaw esquerda",), "Gira o Drone em yaw para a esquerda.", "Yaws the Drone left.", "E"),
        _c("yaw_right", "girar drone direita", "yaw right", ("yaw direita",), "Gira o Drone em yaw para a direita.", "Yaws the Drone right.", "Q"),
        _c("hover", "pairar", "hover", ("estabilizar drone", "ficar pairando"), "Zera translação e mantém estabilização.", "Stops translation and keeps stabilization."),
        _c("camera_left", "câmera drone esquerda", "drone camera left", ("camera drone esquerda",), "Move o gimbal do Drone para a esquerda.", "Moves the Drone gimbal left.", "Z"),
        _c("camera_right", "câmera drone direita", "drone camera right", ("camera drone direita",), "Move o gimbal do Drone para a direita.", "Moves the Drone gimbal right.", "X"),
        _c("camera_up", "subir câmera drone", "drone camera up", ("subir camera drone",), "Move o gimbal do Drone para cima.", "Moves the Drone gimbal upward.", "F"),
        _c("camera_down", "descer câmera drone", "drone camera down", ("descer camera drone",), "Move o gimbal do Drone para baixo.", "Moves the Drone gimbal downward.", "R"),
        _c("camera_center", "centralizar câmera drone", "drone camera center", ("centralizar camera drone",), "Retorna o gimbal do Drone ao centro.", "Returns the Drone gimbal to center."),
        _c("camera_toggle", "alternar câmera drone", "toggle drone camera", ("alternar camera drone",), "Alterna a câmera do Drone.", "Toggles the Drone camera."),
    ),
}

MISSIONS: Dict[str, Tuple[MissionSpec, ...]] = {
    "Manipulator": (
        _m("inspect_workspace", "inspecionar bancada", "inspect workspace", ("missão inspeção", "missao inspecao"), "Inspeciona a área de trabalho usando sensores e visão.", "Inspects the workspace using sensors and vision."),
        _m("scan_workspace", "varredura 360", "360 degree sweep", ("scan 360", "girar 360", "varrer bancada", "fazer varredura 360", "varrer 360", "varrer 360 graus", "full scan"), "Executa varredura completa da área de trabalho.", "Performs a complete workspace sweep."),
        _m("pick_object", "pegar objeto", "pick object", ("missão pegar objeto", "pegar objeto missão", "capturar objeto", "agarrar objeto", "grab object", "grasp object"), "Localiza, aproxima e segura um objeto.", "Locates, approaches and grips an object."),
        _m("place_object", "colocar objeto", "place object", ("depositar objeto",), "Posiciona e libera um objeto.", "Positions and releases an object."),
        _m("hold_pose", "manter posição", "hold pose", ("manter posicao",), "Mantém a pose atual.", "Holds the current pose."),
        _m("return_home", "retornar home", "return home mission", ("missão home", "missao home"), "Retorna autonomamente à pose inicial.", "Autonomously returns to the home pose."),
        _m("calibrate_gripper", "calibrar garra", "calibrate gripper", ("testar garra",), "Executa ciclo seguro de calibração da garra.", "Runs a safe gripper calibration cycle."),
        _m("rhythm_mode", "modo ritmo", "rhythm mode", ("modo música", "modo musica", "dançar", "dancar", "seguir ritmo"), "Permite resposta motora a música somente no Manipulator.", "Allows music-driven motion only on the Manipulator."),
        _m("wave", "acenar", "wave", ("dar tchau",), "Executa gesto de aceno.", "Performs a waving gesture."),
        _m("stop_mission", "parar missão", "stop mission", ("parar missao", "cancelar missão", "cancelar missao"), "Cancela a missão ativa.", "Cancels the active mission."),
    ),
    "Vehicle": (
        _m("terrain_inspection", "inspecionar terreno", "terrain inspection", ("inspeção de terreno", "inspecao de terreno", "inspect terrain"), "Navega inspecionando terreno e obstáculos.", "Navigates while inspecting terrain and obstacles."),
        _m("locate_objects", "localizar objetos", "locate objects", ("procurar objetos", "encontrar objetos", "find objects"), "Procura objetos usando visão e contexto 3D.", "Searches for objects using vision and 3D context."),
        _m("find_exit", "encontrar saída", "find exit", ("descobrir saída", "descobrir saida", "encontrar saida", "discover exit"), "Procura uma rota de saída segura.", "Searches for a safe exit route."),
        _m("patrol_area", "patrulhar área", "patrol area", ("missão patrulha", "missao patrulha", "patrulhar area"), "Patrulha a área usando navegação e rumo da telemetria.", "Patrols the area using navigation and telemetry heading."),
        _m("perimeter_scan", "varrer perímetro", "perimeter scan", ("varrer perimetro", "scan perimetro"), "Percorre e observa o perímetro.", "Traverses and observes the perimeter."),
        _m("corridor_scan", "inspecionar corredor", "corridor scan", ("varrer corredor",), "Percorre corredor mantendo distância segura.", "Scans a corridor while keeping safe clearance."),
        _m("follow_target", "seguir alvo", "follow target", ("seguir alvo terrestre", "seguir objeto"), "Segue alvo terrestre detectado.", "Follows a detected ground target."),
        _m("dock", "estacionar", "dock", ("docar", "aproximar base"), "Aproxima e estaciona na base.", "Approaches and docks at the base."),
        _m("hold_position", "manter posição", "hold position", ("manter posicao", "parar e observar"), "Para e mantém observação local.", "Stops and observes locally."),
        _m("return_home", "voltar base", "return home", ("retornar home", "voltar para base", "retornar base", "ir para base"), "Retorna à posição de origem usando navegação.", "Returns to the origin using navigation."),
        _m("stop_mission", "parar missão", "stop mission", ("parar missao", "cancelar missão", "cancelar missao"), "Cancela a missão ativa e para o veículo.", "Cancels the active mission and stops the vehicle."),
    ),
    "Drone": (
        _m("terrain_inspection", "inspeção aérea do terreno", "aerial terrain inspection", ("inspeção de terreno", "inspecao de terreno", "inspecionar terreno", "terrain inspection"), "Inspeciona terreno em voo usando sensores e visão.", "Inspects terrain in flight using sensors and vision."),
        _m("locate_objects", "localizar objetos", "locate objects", ("procurar objetos", "encontrar objetos", "find objects"), "Procura objetos a partir do ar.", "Searches for objects from the air."),
        _m("find_exit", "encontrar saída", "find exit", ("descobrir saída", "descobrir saida", "encontrar saida", "discover exit"), "Procura uma saída/rota segura a partir do ar.", "Searches for a safe exit/route from the air."),
        _m("aerial_scan", "varredura aérea", "aerial scan", ("missão varredura aérea", "missao varredura aerea", "scan aéreo", "scan aereo"), "Executa padrão de varredura aérea.", "Runs an aerial scan pattern."),
        _m("orbit_point", "orbitar ponto", "orbit point", ("orbitar alvo",), "Orbita um ponto mantendo observação.", "Orbits a point while observing it."),
        _m("search_pattern", "padrão de busca", "search pattern", ("buscar alvo", "padrao de busca"), "Executa padrão autônomo de busca.", "Runs an autonomous search pattern."),
        _m("altitude_hold", "manter altitude", "altitude hold", (), "Mantém a altitude atual com estabilização.", "Maintains current altitude with stabilization."),
        _m("return_home", "voltar base", "return home", ("retornar home", "voltar para base", "retornar base", "ir para base"), "Retorna à origem usando a pose local do Processing e o rumo da telemetria.", "Returns to origin using local Processing pose and telemetry heading."),
        _m("emergency_land", "pouso de emergência", "emergency land", ("pouso de emergencia",), "Executa pouso imediato com prioridade de segurança.", "Performs an immediate safety-priority landing."),
        _m("stop_mission", "parar missão", "stop mission", ("parar missao", "cancelar missão", "cancelar missao"), "Cancela a missão e entra em hover seguro.", "Cancels the mission and enters safe hover."),
    ),
}


def commands_for_robot(robot: Any) -> Tuple[CommandSpec, ...]:
    return COMMANDS[canonical_robot(robot)]


def missions_for_robot(robot: Any) -> Tuple[MissionSpec, ...]:
    return MISSIONS[canonical_robot(robot)]


def all_specs_for_robot(robot: Any) -> Tuple[CommandSpec | MissionSpec, ...]:
    name = canonical_robot(robot)
    return (*COMMANDS[name], *MISSIONS[name])


def aliases_for_robot(robot: Any, kind: str) -> Mapping[str, Tuple[str, ...]]:
    specs = commands_for_robot(robot) if kind == "intent" else missions_for_robot(robot)
    return {spec.intent: spec.aliases for spec in specs}


def resolve_catalog(robot: Any, text: Any) -> Tuple[str, str, int]:
    name = canonical_robot(robot)
    query = normalize_text(text)
    best = ("none", "", 0)
    for kind, specs in (("intent", COMMANDS[name]), ("mission", MISSIONS[name])):
        for spec in specs:
            for alias in spec.aliases:
                phrase = normalize_text(alias)
                if not phrase:
                    continue
                score = 0
                if query == phrase:
                    score = 2000 + len(phrase)
                elif phrase in query:
                    score = 300 + len(phrase)
                if score > best[2]:
                    best = (kind, spec.intent, score)
    return best


def audit_catalog() -> Dict[str, Dict[str, int]]:
    return {
        robot: {"commands": len(COMMANDS[robot]), "missions": len(MISSIONS[robot])}
        for robot in ROBOTS
    }


__all__ = [
    "CommandSpec", "MissionSpec", "COMMANDS", "MISSIONS", "commands_for_robot",
    "missions_for_robot", "all_specs_for_robot", "aliases_for_robot",
    "resolve_catalog", "audit_catalog",
]
