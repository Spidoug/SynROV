"""Priority and event orchestration for the SynROV multimodal AI runtime.

Human input keeps stable precedence.  Autonomous ownership is tracked per
robot so a recent command for Vehicle never blocks Drone or Manipulator after
a hardware/mode switch.  Per-command tuning comes from ``autonomy.py``.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from .autonomy import command_tuning
from .dataset import canonical_robot
from .primitives import normalize_identifier, safe_float

AUTONOMOUS_SOURCES = {"ai", "model", "autonomy", "curiosity", "vision", "object", "music", "audio"}
HUMAN_SOURCES = {"manual", "operator", "ui", "teacher", "voice", "text", "typed"}


def _source_group(source: Any) -> str:
    src = normalize_identifier(source) or "observe"
    if src.startswith("mission"):
        return "mission"
    if src.startswith("auto"):
        return "autonomy"
    if src.startswith("model"):
        return "model"
    return src


@dataclass(frozen=True)
class ActionDecision:
    allowed: bool
    source: str
    reason: str = ""
    priority: int = 0
    lock_s: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)


class PriorityArbiter:
    """Deterministic source priority with robot/command-specific bias."""

    DEFAULT_PRIORITIES: Dict[str, int] = {
        "emergency": 150,
        "safety": 145,
        "manual": 125,
        "operator": 125,
        "ui": 122,
        "teacher": 122,
        "voice": 116,
        "text": 112,
        "typed": 112,
        "mission": 92,
        "mission_step": 92,
        "vision": 76,
        "object": 76,
        "music": 68,
        "audio": 68,
        "ai": 56,
        "model": 56,
        "autonomy": 56,
        "curiosity": 30,
        "observe": 12,
        "telemetry": 12,
    }

    DEFAULT_LOCKS: Dict[str, float] = {
        "emergency": 1.80,
        "safety": 1.50,
        "manual": 0.72,
        "operator": 0.72,
        "ui": 0.65,
        "teacher": 0.65,
        "voice": 1.25,
        "text": 1.05,
        "typed": 1.05,
        "mission": 0.34,
        "mission_step": 0.34,
        "vision": 0.26,
        "object": 0.26,
        "music": 0.22,
        "audio": 0.22,
        "ai": 0.42,
        "model": 0.42,
        "autonomy": 0.42,
        "curiosity": 0.18,
        "observe": 0.0,
        "telemetry": 0.0,
    }

    def priority(self, source: Any, *, robot: Any = None, command: Any = None) -> int:
        base = int(self.DEFAULT_PRIORITIES.get(_source_group(source), 40))
        if robot is None or not command:
            return base
        return base + int(command_tuning(robot, command).priority_bias)

    def lock_seconds(self, source: Any, *, robot: Any = None, command: Any = None) -> float:
        base = float(self.DEFAULT_LOCKS.get(_source_group(source), 0.35))
        if robot is None or not command:
            return base
        return max(0.0, base * float(command_tuning(robot, command).lock_scale))


@dataclass
class RuntimeState:
    last_action_ts: Dict[str, float] = field(default_factory=dict)
    last_decision: Optional[ActionDecision] = None
    last_blocked: Optional[ActionDecision] = None
    last_command_source: str = ""
    last_command_name: str = ""
    last_command_ts: float = 0.0
    cycle_id: int = 0
    automation_locked: bool = False


class SynROVOrchestrator:
    """Central arbiter with independent ownership state for every robot."""

    def __init__(self, arbiter: Optional[PriorityArbiter] = None) -> None:
        self.arbiter = arbiter or PriorityArbiter()
        self.state = RuntimeState()  # compatibility/global diagnostics
        self.robot_states: Dict[str, RuntimeState] = {
            name: RuntimeState() for name in ("Manipulator", "Vehicle", "Drone")
        }

    def _state_for(self, robot: Any = None) -> RuntimeState:
        if robot is None:
            return self.state
        return self.robot_states[canonical_robot(robot)]

    def priority(self, source: Any, *, robot: Any = None, command: Any = None) -> int:
        return self.arbiter.priority(source, robot=robot, command=command)

    def begin_cycle(self, robot: Any = None) -> int:
        self.state.cycle_id += 1
        if robot is not None:
            robot_state = self._state_for(robot)
            robot_state.cycle_id += 1
            return robot_state.cycle_id
        return self.state.cycle_id

    def set_automation_locked(self, locked: bool) -> None:
        value = bool(locked)
        self.state.automation_locked = value
        for robot_state in self.robot_states.values():
            robot_state.automation_locked = value

    def register_external_command(
        self,
        source: Any,
        timestamp: Optional[float] = None,
        *,
        robot: Any = None,
        command: Any = None,
    ) -> None:
        self.register_action(source, timestamp, robot=robot, command=command)

    def register_action(
        self,
        source: Any,
        timestamp: Optional[float] = None,
        *,
        robot: Any = None,
        command: Any = None,
    ) -> None:
        src = _source_group(source) or "unknown"
        ts = float(timestamp if timestamp is not None else time.time())
        name = str(command or "")
        targets = [self.state]
        if robot is not None:
            targets.append(self._state_for(robot))
        for runtime_state in targets:
            runtime_state.last_action_ts[src] = ts
            runtime_state.last_command_source = src
            runtime_state.last_command_name = name
            runtime_state.last_command_ts = ts

    def _decision(
        self,
        allowed: bool,
        src: str,
        reason: str,
        priority: int,
        lock_s: float,
        *,
        runtime_state: RuntimeState,
        **metadata: Any,
    ) -> ActionDecision:
        decision = ActionDecision(allowed, src, reason, priority, lock_s, dict(metadata))
        if allowed:
            runtime_state.last_decision = decision
            self.state.last_decision = decision
        else:
            runtime_state.last_blocked = decision
            self.state.last_blocked = decision
        return decision

    def action_allowed(
        self,
        source: Any,
        *,
        robot: Any = None,
        command: Any = None,
        voice_ctx: Optional[Dict[str, Any]] = None,
        telemetry_fresh: bool = True,
        mission_active: bool = False,
        last_command_source: Any = None,
        last_command_age_s: Optional[float] = None,
        confidence: float = 1.0,
        now: Optional[float] = None,
    ) -> ActionDecision:
        ts = float(now if now is not None else time.time())
        src = _source_group(source) or "unknown"
        runtime_state = self._state_for(robot)
        priority = self.priority(src, robot=robot, command=command)
        lock_s = self.arbiter.lock_seconds(src, robot=robot, command=command)
        conf = safe_float(confidence, 1.0, 0.0, 1.0)
        tuning = command_tuning(robot, command) if robot is not None and command else None
        confidence_floor = float(tuning.confidence_floor) if tuning is not None else 0.18

        if runtime_state.automation_locked and src in AUTONOMOUS_SOURCES:
            return self._decision(False, src, "automation_locked", priority, lock_s, runtime_state=runtime_state)

        ctx = dict(voice_ctx or {})
        voice_intent = normalize_identifier(ctx.get("intent", "none")) or "none"
        voice_age = safe_float(ctx.get("text_age", ctx.get("text_age_s", 999.0)), 999.0, 0.0)
        voice_conf = safe_float(ctx.get("conf", ctx.get("confidence", 0.0)), 0.0, 0.0, 1.0)
        fresh_voice = voice_intent != "none" and voice_age <= 1.45 and voice_conf >= 0.15

        if src in {"ai", "model", "autonomy", "vision", "object", "music", "audio"} and not telemetry_fresh:
            return self._decision(False, src, "stale_telemetry", priority, lock_s, runtime_state=runtime_state)

        if conf < confidence_floor and src in AUTONOMOUS_SOURCES:
            return self._decision(
                False,
                src,
                "low_confidence",
                priority,
                lock_s,
                runtime_state=runtime_state,
                confidence=conf,
                confidence_floor=confidence_floor,
            )

        if mission_active and src in {"ai", "model", "autonomy", "curiosity"}:
            return self._decision(False, src, "mission_has_control", priority, lock_s, runtime_state=runtime_state)

        if src in {"ai", "model", "autonomy", "music", "audio", "vision", "object"} and fresh_voice:
            allowed_voice_intents = {
                "music_follow", "follow_music", "object_grasp", "object_follow", "follow_object", "none",
            }
            if not (src in {"music", "audio", "vision", "object"} and voice_intent in allowed_voice_intents):
                return self._decision(
                    False,
                    src,
                    "fresh_voice_priority",
                    priority,
                    lock_s,
                    runtime_state=runtime_state,
                    voice_intent=voice_intent,
                )

        prior_src = _source_group(
            last_command_source if last_command_source is not None else runtime_state.last_command_source
        )
        if last_command_age_s is None:
            last_ts = runtime_state.last_command_ts
            last_command_age_s = ts - last_ts if last_ts else 999.0
        if prior_src and prior_src != src:
            prior_command = runtime_state.last_command_name
            prior_priority = self.priority(prior_src, robot=robot, command=prior_command)
            prior_lock = self.arbiter.lock_seconds(prior_src, robot=robot, command=prior_command)
            priority_window = max(lock_s, prior_lock)
            if last_command_age_s <= priority_window and prior_priority > priority:
                return self._decision(
                    False,
                    src,
                    f"recent_{prior_src}_priority",
                    priority,
                    lock_s,
                    runtime_state=runtime_state,
                    prior_priority=prior_priority,
                    age_s=float(last_command_age_s),
                )

        return self._decision(
            True,
            src,
            "allowed",
            priority,
            lock_s,
            runtime_state=runtime_state,
            confidence=conf,
            robot=canonical_robot(robot) if robot is not None else None,
            command=str(command or ""),
        )

    def health(self) -> Dict[str, Any]:
        def serialize(runtime_state: RuntimeState) -> Dict[str, Any]:
            return {
                "cycle_id": runtime_state.cycle_id,
                "automation_locked": runtime_state.automation_locked,
                "last_action_ts": dict(runtime_state.last_action_ts),
                "last_command_source": runtime_state.last_command_source,
                "last_command_name": runtime_state.last_command_name,
                "last_command_ts": runtime_state.last_command_ts,
                "last_decision": runtime_state.last_decision.__dict__ if runtime_state.last_decision else None,
                "last_blocked": runtime_state.last_blocked.__dict__ if runtime_state.last_blocked else None,
            }

        out = serialize(self.state)
        out["robots"] = {name: serialize(state) for name, state in self.robot_states.items()}
        return out
