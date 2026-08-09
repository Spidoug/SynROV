"""Beat-driven dance policy with a tiny persistent preference model.

The module never stores raw audio, frames or telemetry.  It starts from a
predefined choreography model and learns only compact per-style preference
scores from confirmed beats.  This keeps rhythm mode adaptive without creating
continuous datasets or background training load.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

from .primitives import safe_float

DANCE_SCHEMA = "synrov.aibot.dance.v1"
HOME_POSE: Dict[str, float] = {
    "base": 180.0,
    "upper": 45.0,
    "fore": 180.0,
    "forearm_roll": 90.0,
    "wrist_pitch": 95.0,
    "wrist_rot": 130.0,
    "grip": 50.0,
}


@dataclass(frozen=True)
class DanceStyle:
    name: str
    min_bpm: float
    max_bpm: float
    amplitude: float
    phase: float = 0.0


DEFAULT_STYLES: Mapping[str, Tuple[DanceStyle, ...]] = {
    "Manipulator": (
        DanceStyle("grip_wave", 55.0, 150.0, 1.00, 0.0),
        DanceStyle("wrist_groove", 70.0, 180.0, 0.82, math.pi / 3.0),
        DanceStyle("roll_groove", 85.0, 210.0, 0.72, math.pi / 2.0),
    ),
    "Vehicle": (
        DanceStyle("pivot_soft", 55.0, 145.0, 0.55, 0.0),
        DanceStyle("sway", 75.0, 180.0, 0.48, math.pi / 2.0),
    ),
    "Drone": (
        DanceStyle("yaw_hover", 55.0, 150.0, 0.30, 0.0),
        DanceStyle("roll_hover", 75.0, 180.0, 0.22, math.pi / 2.0),
    ),
}


@dataclass
class DanceState:
    style: str = ""
    beat_index: int = 0
    last_emit_ts: float = 0.0
    last_switch_ts: float = 0.0
    last_save_ts: float = 0.0
    updates_since_save: int = 0
    scores: Dict[str, float] = field(default_factory=dict)


class AdaptiveDanceModel:
    """Small online selector seeded by predefined choreography styles.

    Learning is intentionally bounded: only style scores are updated on
    confirmed beats.  No raw signal is persisted and disk writes are throttled.
    """

    def __init__(self, path: Optional[Path] = None) -> None:
        self.path = Path(path) if path is not None else None
        self.states: Dict[str, DanceState] = {}
        self._load()

    @staticmethod
    def _robot(robot: Any) -> str:
        value = str(robot or "Manipulator").strip().lower()
        if value.startswith("veh"):
            return "Vehicle"
        if value.startswith("dro"):
            return "Drone"
        return "Manipulator"

    def _state(self, robot: str) -> DanceState:
        robot = self._robot(robot)
        state = self.states.setdefault(robot, DanceState())
        for style in DEFAULT_STYLES[robot]:
            state.scores.setdefault(style.name, 1.0)
        return state

    def _load(self) -> None:
        if self.path is None or not self.path.is_file():
            return
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError):
            return
        if not isinstance(raw, dict) or raw.get("schema") != DANCE_SCHEMA:
            return
        robots = raw.get("robots", {})
        if not isinstance(robots, dict):
            return
        for robot, item in robots.items():
            key = self._robot(robot)
            if not isinstance(item, dict):
                continue
            scores = item.get("scores", {})
            clean_scores = {
                str(name): max(0.25, min(4.0, safe_float(score, 1.0)))
                for name, score in (scores.items() if isinstance(scores, dict) else ())
            }
            self.states[key] = DanceState(style=str(item.get("style", "") or ""), scores=clean_scores)

    def save(self, force: bool = False, now: Optional[float] = None) -> bool:
        if self.path is None:
            return False
        # Opening and closing AiBot without a confirmed beat must not create a
        # dance file. Persistence starts only after the rhythm model was used.
        if not self.states and not self.path.exists():
            return False
        now = time.time() if now is None else float(now)
        dirty = any(state.updates_since_save > 0 for state in self.states.values())
        if not dirty and not force:
            return False
        last_save = max((state.last_save_ts for state in self.states.values()), default=0.0)
        updates = sum(state.updates_since_save for state in self.states.values())
        if not force and updates < 48 and (now - last_save) < 60.0:
            return False
        payload = {
            "schema": DANCE_SCHEMA,
            "version": 1,
            "updated_at": now,
            "robots": {
                robot: {"style": state.style, "scores": dict(state.scores)}
                for robot, state in self.states.items()
            },
        }
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(".tmp")
            tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            tmp.replace(self.path)
        except (OSError, UnicodeError):
            return False
        for state in self.states.values():
            state.last_save_ts = now
            state.updates_since_save = 0
        return True

    @staticmethod
    def _style_fit(style: DanceStyle, bpm: float, energy: float) -> float:
        bpm = max(0.0, safe_float(bpm, 0.0))
        energy = max(0.0, min(1.5, safe_float(energy, 0.0)))
        if bpm <= 0.0:
            tempo_fit = 0.55
        elif style.min_bpm <= bpm <= style.max_bpm:
            tempo_fit = 1.0
        else:
            distance = min(abs(bpm - style.min_bpm), abs(bpm - style.max_bpm))
            tempo_fit = max(0.15, 1.0 - distance / 90.0)
        return tempo_fit * (0.72 + 0.28 * min(1.0, energy * 8.0))

    def _select_style(self, robot: str, bpm: float, energy: float, now: float) -> DanceStyle:
        state = self._state(robot)
        styles = DEFAULT_STYLES[robot]
        ranked = sorted(
            styles,
            key=lambda style: self._style_fit(style, bpm, energy) * state.scores.get(style.name, 1.0),
            reverse=True,
        )
        current = next((style for style in styles if style.name == state.style), None)
        should_switch = current is None or (now - state.last_switch_ts) >= 5.0
        chosen = ranked[0] if should_switch else current
        if chosen.name != state.style:
            state.style = chosen.name
            state.last_switch_ts = now
        return chosen

    def observe_beat(self, robot: Any, beat: Any, now: Optional[float] = None) -> DanceStyle:
        robot = self._robot(robot)
        now = time.time() if now is None else float(now)
        bpm = safe_float(getattr(beat, "bpm", 0.0), 0.0)
        energy = safe_float(getattr(beat, "energy", 0.0), 0.0)
        confidence = max(0.0, min(1.0, safe_float(getattr(beat, "confidence", 0.0), 0.0)))
        state = self._state(robot)
        style = self._select_style(robot, bpm, energy, now)
        if bool(getattr(beat, "beat_now", False)):
            state.beat_index += 1
            fit = self._style_fit(style, bpm, energy)
            reward = 0.65 * confidence + 0.35 * fit
            old = state.scores.get(style.name, 1.0)
            state.scores[style.name] = max(0.25, min(4.0, old * 0.965 + (0.75 + reward) * 0.035))
            state.updates_since_save += 1
            self.save(force=False, now=now)
        return style

    def due(self, robot: Any, beat: Any, now: Optional[float] = None) -> bool:
        robot = self._robot(robot)
        now = time.time() if now is None else float(now)
        if not bool(getattr(beat, "active", False)):
            return False
        state = self._state(robot)
        bpm = safe_float(getattr(beat, "bpm", 0.0), 0.0)
        period = 60.0 / bpm if bpm > 1.0 else 0.50
        interval = max(0.16, min(0.55, period * 0.50))
        if bool(getattr(beat, "beat_now", False)) or (now - state.last_emit_ts) >= interval:
            state.last_emit_ts = now
            return True
        return False

    def manipulator_pose(self, beat: Any, now: Optional[float] = None) -> Dict[str, float]:
        now = time.time() if now is None else float(now)
        style = self.observe_beat("Manipulator", beat, now)
        state = self._state("Manipulator")
        pose = dict(HOME_POSE)
        phase = (state.beat_index % 8) * (math.tau / 8.0) + style.phase
        energy = max(0.0, min(1.0, safe_float(getattr(beat, "energy", 0.0), 0.0) * 8.0))
        amp = style.amplitude * (0.55 + 0.45 * energy)
        if style.name == "grip_wave":
            pose["grip"] = max(18.0, min(82.0, 50.0 + 30.0 * amp * math.sin(phase)))
            pose["wrist_rot"] = max(100.0, min(160.0, 130.0 + 12.0 * amp * math.sin(phase * 0.5)))
        elif style.name == "wrist_groove":
            pose["wrist_pitch"] = max(72.0, min(118.0, 95.0 + 20.0 * amp * math.sin(phase)))
            pose["wrist_rot"] = max(100.0, min(160.0, 130.0 + 24.0 * amp * math.sin(phase + math.pi / 2.0)))
            pose["grip"] = max(28.0, min(72.0, 50.0 + 18.0 * amp * math.sin(phase * 0.5)))
        else:
            pose["forearm_roll"] = max(62.0, min(118.0, 90.0 + 25.0 * amp * math.sin(phase)))
            pose["wrist_rot"] = max(100.0, min(160.0, 130.0 + 20.0 * amp * math.sin(phase + math.pi / 2.0)))
            pose["grip"] = max(30.0, min(70.0, 50.0 + 16.0 * amp * math.sin(phase)))
        return pose

    def vehicle_values(self, beat: Any, now: Optional[float] = None) -> Tuple[float, float, float, float]:
        now = time.time() if now is None else float(now)
        style = self.observe_beat("Vehicle", beat, now)
        state = self._state("Vehicle")
        phase = (state.beat_index % 8) * (math.tau / 8.0) + style.phase
        amp = style.amplitude
        if style.name == "pivot_soft":
            return 0.0, max(-0.22, min(0.22, 0.20 * amp * math.sin(phase))), 0.0, 0.0
        return 0.05 * (1.0 if math.sin(phase) >= 0 else -1.0), max(-0.16, min(0.16, 0.15 * amp * math.sin(phase))), 0.0, 0.0

    def drone_values(self, beat: Any, now: Optional[float] = None) -> Tuple[float, float, float, float, float, float]:
        now = time.time() if now is None else float(now)
        style = self.observe_beat("Drone", beat, now)
        state = self._state("Drone")
        phase = (state.beat_index % 8) * (math.tau / 8.0) + style.phase
        amp = style.amplitude
        if style.name == "roll_hover":
            return 0.0, 0.0, 0.0, max(-0.10, min(0.10, 0.09 * amp * math.sin(phase))), 0.0, 0.0
        return 0.0, max(-0.10, min(0.10, 0.09 * amp * math.sin(phase))), 0.0, 0.0, 0.0, 0.0
