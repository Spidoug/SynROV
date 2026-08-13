"""Adaptive contextual reasoning for the three SynROV robots.

This module deliberately keeps actuation inside the audited robot skill catalog,
while allowing novel natural-language goals, explicit multi-step plans and
persistent long-context experience to select and sequence those safe skills.
"""
from __future__ import annotations

import json
import math
import re
import time
from collections import deque
from dataclasses import dataclass
from difflib import SequenceMatcher
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, Mapping, Optional, Sequence, Tuple

from .primitives import normalize_text, safe_float
from .robot_types import canonical_robot


@dataclass(frozen=True)
class CognitiveResolution:
    kind: str = "none"
    name: str = ""
    confidence: float = 0.0
    reason: str = "unresolved"


# Concept vocabulary supplements the literal catalog aliases. It is intentionally
# semantic rather than actuator-specific: unfamiliar phrasing resolves to an
# audited goal/skill and never directly invents raw motor commands.
CONCEPT_HINTS: Dict[str, Dict[str, Dict[str, Tuple[str, ...]]]] = {
    "Manipulator": {
        "mission": {
            "inspect_workspace": ("observar", "examinar", "analisar bancada", "verificar area", "inspecionar area"),
            "scan_workspace": ("mapear bancada", "varredura completa", "olhar ao redor", "scan completo"),
            "pick_object": ("apanhar", "agarrar", "segurar objeto", "capturar", "recolher", "pegar item", "pegar", "pegue", "apanhe", "pegar coisa"),
            "place_object": ("depositar", "deixar objeto", "soltar no local", "posicionar item"),
            "hold_pose": ("ficar parado", "manter postura", "nao se mover"),
            "return_home": ("voltar ao inicio", "ir para origem", "retornar origem"),
        },
        "intent": {
            "grip_open": ("liberar", "soltar", "abrir pinca"),
            "grip_close": ("prender", "segurar", "fechar pinca"),
            "home": ("zerar braco", "inicio do braco"),
        },
    },
    "Vehicle": {
        "mission": {
            "terrain_inspection": ("explorar terreno", "avaliar terreno", "reconhecer terreno", "inspecionar caminho"),
            "locate_objects": ("procurar coisa", "achar objeto", "buscar objeto", "encontrar item", "localizar alvo"),
            "find_exit": ("achar saida", "buscar rota de fuga", "sair daqui", "encontrar caminho para fora"),
            "patrol_area": ("rondar", "vigiar area", "patrulhar local", "fazer ronda"),
            "perimeter_scan": ("verificar perimetro", "contornar area", "mapear borda"),
            "corridor_scan": ("examinar corredor", "percorrer corredor", "mapear corredor"),
            "follow_target": ("acompanhar alvo", "seguir objeto", "perseguir alvo"),
            "dock": ("voltar para doca", "estacionar na base", "acoplar na base"),
            "hold_position": ("ficar parado", "parar e observar", "manter local"),
            "return_home": ("voltar para origem", "retornar base", "ir para casa"),
        },
        "intent": {
            "stop": ("pare agora", "imobilizar", "frear tudo"),
            "forward": ("seguir em frente", "ir adiante"),
            "back": ("andar para tras", "voltar um pouco"),
        },
    },
    "Drone": {
        "mission": {
            "terrain_inspection": ("reconhecimento aereo", "examinar terreno do alto", "mapear terreno", "inspecionar area pelo ar"),
            "locate_objects": ("procurar coisa", "achar objeto", "buscar objeto", "encontrar item", "localizar alvo do ar"),
            "find_exit": ("achar saida", "buscar rota segura", "encontrar caminho para fora"),
            "aerial_scan": ("varrer area", "mapear do alto", "fazer reconhecimento", "scan do alto"),
            "orbit_point": ("circular alvo", "dar volta no ponto", "contornar alvo"),
            "search_pattern": ("fazer busca", "vasculhar area", "procurar alvo", "pente fino aereo"),
            "altitude_hold": ("ficar nessa altura", "segurar altura", "nao mudar altitude"),
            "return_home": ("voltar para origem", "retornar base", "ir para casa"),
            "emergency_land": ("pouse imediatamente", "descer de emergencia", "pousar agora"),
        },
        "intent": {
            "takeoff": ("levantar voo", "decolar agora", "decole", "suba e decole", "sair do chao"),
            "land": ("pousar", "pouse", "aterrisse", "ir para o chao"),
            "hover": ("ficar parado no ar", "pairar no lugar"),
            "forward": ("voar adiante", "seguir em frente", "avance", "avance para frente"),
            "back": ("voar para tras", "recuar no ar"),
            "left": ("deslocar para esquerda", "ir lateralmente esquerda"),
            "right": ("deslocar para direita", "ir lateralmente direita"),
        },
    },
}

_STOPWORDS = {
    "a", "o", "as", "os", "um", "uma", "de", "do", "da", "dos", "das", "para", "por", "com",
    "e", "que", "eu", "voce", "robo", "robot", "drone", "veiculo", "vehicle", "manipulador", "manipulator",
    "please", "the", "a", "an", "to", "and", "then", "now", "agora", "favor", "quero", "preciso",
}


def _tokens(value: Any) -> set[str]:
    return {token for token in normalize_text(value).split() if len(token) > 1 and token not in _STOPWORDS}


def _phrase_similarity(query: str, phrase: str) -> float:
    q = normalize_text(query)
    p = normalize_text(phrase)
    if not q or not p:
        return 0.0
    if q == p:
        return 1.0
    qt, pt = _tokens(q), _tokens(p)
    overlap = len(qt & pt) / max(1, len(pt))
    jaccard = len(qt & pt) / max(1, len(qt | pt))
    sequence = SequenceMatcher(None, q, p).ratio()
    soft_scores = []
    for expected in pt:
        best_token = max((SequenceMatcher(None, expected, actual).ratio() for actual in qt), default=0.0)
        soft_scores.append(best_token if best_token >= 0.70 else 0.0)
    soft_coverage = sum(soft_scores) / max(1, len(pt))
    contains = 0.92 if p in q or q in p else 0.0
    return max(contains, 0.36 * overlap + 0.14 * jaccard + 0.18 * sequence + 0.32 * soft_coverage)


def split_explicit_plan(text: Any) -> Tuple[str, ...]:
    """Split only explicit sequencing language; ordinary conjunctions stay intact."""
    raw = str(text or "").strip()
    if not raw:
        return ()
    parts = re.split(
        r"\s*(?:;|\b(?:e\s+depois|depois|em\s+seguida|logo\s+depois|entao|then|and\s+then|after\s+that|next)\b)\s*",
        raw,
        flags=re.IGNORECASE,
    )
    return tuple(part.strip(" ,.") for part in parts if normalize_text(part))


def _candidate_phrases(robot: str, kind: str, name: str, aliases: Iterable[str]) -> Tuple[str, ...]:
    hints = CONCEPT_HINTS.get(robot, {}).get(kind, {}).get(name, ())
    return tuple(dict.fromkeys([name.replace("_", " "), *aliases, *hints]))


def contextual_resolve(robot: Any, text: Any, intents: Mapping[str, Sequence[str]], missions: Mapping[str, Sequence[str]]) -> CognitiveResolution:
    canonical = canonical_robot(robot)
    query = normalize_text(text)
    if not query:
        return CognitiveResolution()

    best = CognitiveResolution()
    runner_up = 0.0
    for kind, catalog in (("mission", missions), ("intent", intents)):
        for name, aliases in catalog.items():
            score = max((_phrase_similarity(query, phrase) for phrase in _candidate_phrases(canonical, kind, name, aliases)), default=0.0)
            # Prefer mission-level reasoning for broad goal language while direct
            # actuator commands require slightly more literal evidence.
            if kind == "mission" and len(_tokens(query)) >= 3:
                score += 0.025
            if score > best.confidence:
                runner_up = best.confidence
                best = CognitiveResolution(kind, name, min(1.0, score), "semantic_goal")
            elif score > runner_up:
                runner_up = score

    threshold = 0.50 if best.kind == "mission" else 0.56
    # Ambiguous free-form commands are not guessed. This preserves safety while
    # still accepting novel phrasing with a clear semantic winner.
    if best.confidence < threshold or (best.confidence - runner_up) < 0.055:
        return CognitiveResolution()
    return best


def compact_state_snapshot(state: Any, robot: Any) -> Dict[str, Any]:
    """Persist a bounded, non-image state summary useful for later decisions."""
    canonical = canonical_robot(robot)
    sensors = getattr(state, "sensors", {}) or {}
    control = getattr(state, "control", {}) or {}
    intelligence = getattr(state, "intelligence_context", {}) or {}
    out: Dict[str, Any] = {
        "robot": canonical,
        "connected": bool(getattr(state, "connected", False)),
        "active_input_source": str(getattr(state, "active_input_source", "none") or "none"),
        "human_override": bool(getattr(state, "human_override", False)),
    }
    for key in ("battery_pct", "bat_pct", "alt_cm", "altitude_cm", "sonar_down_cm", "lidar_cm", "range_cm", "sonar_cm"):
        if key in sensors:
            value = safe_float(sensors.get(key), math.nan)
            if math.isfinite(value):
                out[key] = round(value, 3)
    if isinstance(control, Mapping):
        block_name = "flight" if canonical == "Drone" else "drive" if canonical == "Vehicle" else "manipulator"
        block = control.get(block_name, {})
        if isinstance(block, Mapping):
            for key in ("airborne", "flightReady", "phase", "collisionFiltered", "throttle", "steer", "forward", "strafe", "yaw"):
                if key in block and isinstance(block.get(key), (bool, int, float, str)):
                    out[key] = block.get(key)
    if isinstance(intelligence, Mapping):
        features = intelligence.get("features", {})
        if isinstance(features, Mapping):
            position = features.get("position")
            if isinstance(position, Mapping):
                out["position"] = {k: safe_float(position.get(k), 0.0) for k in ("x_m", "y_m", "z_m", "x_scene", "y_scene", "z_scene", "altitude_m") if k in position}
    return out


class LongContextMemory:
    """Persistent cross-session episodic memory, isolated per robot."""

    def __init__(self, path: Optional[Path] = None, *, max_episodes: int = 4000) -> None:
        self.path = Path(path or Path(__file__).resolve().parent / "runtime_data" / "long_context.jsonl")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.max_episodes = max(200, int(max_episodes))
        self._episodes: Deque[Dict[str, Any]] = deque(maxlen=self.max_episodes)
        self._load()

    def _load(self) -> None:
        if not self.path.is_file():
            return
        try:
            lines = self.path.read_text(encoding="utf-8").splitlines()[-self.max_episodes :]
        except (OSError, UnicodeError):
            return
        for line in lines:
            try:
                item = json.loads(line)
            except (TypeError, json.JSONDecodeError):
                continue
            if isinstance(item, dict) and item.get("robot") in {"Manipulator", "Vehicle", "Drone"}:
                self._episodes.append(item)

    def record(
        self,
        robot: Any,
        text: Any,
        kind: str,
        name: str,
        *,
        ok: bool,
        reason: str = "",
        state: Any = None,
        metadata: Optional[Mapping[str, Any]] = None,
    ) -> None:
        canonical = canonical_robot(robot)
        phrase = normalize_text(text)
        if not phrase or kind not in {"intent", "mission"} or not name:
            return
        item: Dict[str, Any] = {
            "ts": round(time.time(), 3),
            "robot": canonical,
            "text": phrase[:320],
            "kind": kind,
            "name": str(name)[:96],
            "ok": bool(ok),
            "reason": str(reason or "")[:120],
            "state": compact_state_snapshot(state, canonical) if state is not None else {},
        }
        if metadata:
            safe_meta: Dict[str, Any] = {}
            for key in ("strategy", "strategy_variant", "object_localized", "avoidance", "resource_guard", "autonomous_action"):
                value = metadata.get(key)
                if isinstance(value, (str, bool, int, float)):
                    safe_meta[key] = value
            if safe_meta:
                item["meta"] = safe_meta
        self._episodes.append(item)
        try:
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n")
            # Periodically bound the on-disk history as well as RAM.
            if self.path.stat().st_size > 5_000_000:
                self.compact()
        except (OSError, UnicodeError):
            pass

    def compact(self) -> None:
        try:
            payload = "".join(json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n" for item in self._episodes)
            self.path.write_text(payload, encoding="utf-8")
        except (OSError, UnicodeError):
            pass

    def suggest(self, robot: Any, text: Any, available: set[Tuple[str, str]]) -> CognitiveResolution:
        canonical = canonical_robot(robot)
        query = normalize_text(text)
        if not query:
            return CognitiveResolution()
        best = CognitiveResolution()
        now = time.time()
        for item in reversed(self._episodes):
            if item.get("robot") != canonical or not item.get("ok", False):
                continue
            kind = str(item.get("kind", ""))
            name = str(item.get("name", ""))
            if (kind, name) not in available:
                continue
            score = _phrase_similarity(query, str(item.get("text", "")))
            if score < 0.62:
                continue
            age_days = max(0.0, (now - safe_float(item.get("ts"), now)) / 86400.0)
            recency = max(0.0, 0.035 * math.exp(-age_days / 120.0))
            confidence = min(1.0, score + recency)
            if confidence > best.confidence:
                best = CognitiveResolution(kind, name, confidence, "long_context")
        return best if best.confidence >= 0.66 else CognitiveResolution()

    def relevant(self, robot: Any, text: Any, limit: int = 8) -> Tuple[Dict[str, Any], ...]:
        canonical = canonical_robot(robot)
        query = normalize_text(text)
        scored = []
        for item in self._episodes:
            if item.get("robot") != canonical:
                continue
            score = _phrase_similarity(query, str(item.get("text", ""))) if query else 0.0
            if score > 0.20:
                scored.append((score, safe_float(item.get("ts"), 0.0), item))
        scored.sort(key=lambda row: (row[0], row[1]), reverse=True)
        return tuple(dict(row[2]) for row in scored[: max(1, int(limit))])

    def metadata(self, robot: Any) -> Dict[str, Any]:
        canonical = canonical_robot(robot)
        count = sum(1 for item in self._episodes if item.get("robot") == canonical)
        return {"episodes": count, "capacity": self.max_episodes, "path": str(self.path)}


__all__ = [
    "CognitiveResolution", "LongContextMemory", "compact_state_snapshot",
    "contextual_resolve", "split_explicit_plan",
]
