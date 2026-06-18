"""Priority and event orchestration for the SynROV multimodal AI runtime.

This module is intentionally small and deterministic.  It gives human commands
stable precedence, lets missions run smoothly between human interventions, and
blocks autonomous sources when the user enables the automation lock.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from .primitives import normalize_identifier


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
    """Deterministic arbiter: one source owns the robot during a short window."""

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

    def priority(self, source: Any) -> int:
        return int(self.DEFAULT_PRIORITIES.get(_source_group(source), 40))

    def lock_seconds(self, source: Any) -> float:
        return float(self.DEFAULT_LOCKS.get(_source_group(source), 0.35))


@dataclass
class RuntimeState:
    last_action_ts: Dict[str, float] = field(default_factory=dict)
    last_decision: Optional[ActionDecision] = None
    last_blocked: Optional[ActionDecision] = None
    last_command_source: str = ""
    last_command_ts: float = 0.0
    cycle_id: int = 0
    automation_locked: bool = False


class SynROVOrchestrator:
    """Central arbiter for voice, autonomy, vision, music and UI commands."""

    def __init__(self, arbiter: Optional[PriorityArbiter] = None) -> None:
        self.arbiter = arbiter or PriorityArbiter()
        self.state = RuntimeState()

    def priority(self, source: Any) -> int:
        return self.arbiter.priority(source)

    def begin_cycle(self) -> int:
        self.state.cycle_id += 1
        return self.state.cycle_id

    def set_automation_locked(self, locked: bool) -> None:
        self.state.automation_locked = bool(locked)

    def register_external_command(self, source: Any, timestamp: Optional[float] = None) -> None:
        self.register_action(source, timestamp)

    def register_action(self, source: Any, timestamp: Optional[float] = None) -> None:
        src = _source_group(source) or "unknown"
        ts = float(timestamp if timestamp is not None else time.time())
        self.state.last_action_ts[src] = ts
        self.state.last_command_source = src
        self.state.last_command_ts = ts

    def _decision(
        self,
        allowed: bool,
        src: str,
        reason: str,
        priority: int,
        lock_s: float,
        **metadata: Any,
    ) -> ActionDecision:
        decision = ActionDecision(allowed, src, reason, priority, lock_s, dict(metadata))
        if allowed:
            self.state.last_decision = decision
        else:
            self.state.last_blocked = decision
        return decision

    def action_allowed(
        self,
        source: Any,
        *,
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
        priority = self.priority(src)
        lock_s = self.arbiter.lock_seconds(src)
        conf = max(0.0, min(1.0, float(confidence if confidence is not None else 1.0)))

        if self.state.automation_locked and src in AUTONOMOUS_SOURCES:
            return self._decision(False, src, "automation_locked", priority, lock_s)

        ctx = dict(voice_ctx or {})
        voice_intent = normalize_identifier(ctx.get("intent", "none")) or "none"
        try:
            voice_age = float(ctx.get("text_age", ctx.get("text_age_s", 999.0)))
        except Exception:
            voice_age = 999.0
        try:
            voice_conf = float(ctx.get("conf", ctx.get("confidence", 0.0)))
        except Exception:
            voice_conf = 0.0
        fresh_voice = voice_intent != "none" and voice_age <= 1.45 and voice_conf >= 0.15

        if src in {"ai", "model", "autonomy", "vision", "object", "music", "audio"} and not telemetry_fresh:
            return self._decision(False, src, "stale_telemetry", priority, lock_s)

        if conf < 0.18 and src in AUTONOMOUS_SOURCES:
            return self._decision(False, src, "low_confidence", priority, lock_s, confidence=conf)

        if mission_active and src in {"ai", "model", "autonomy", "curiosity"}:
            return self._decision(False, src, "mission_has_control", priority, lock_s)

        if src in {"ai", "model", "autonomy", "music", "audio", "vision", "object"} and fresh_voice:
            allowed_voice_intents = {
                "music_follow", "follow_music", "object_grasp", "object_follow", "follow_object",
                "none",
            }
            if src in {"music", "audio", "vision", "object"} and voice_intent in allowed_voice_intents:
                pass
            else:
                return self._decision(False, src, "fresh_voice_priority", priority, lock_s, voice_intent=voice_intent)

        prior_src = _source_group(
            last_command_source if last_command_source is not None else self.state.last_command_source
        )
        if last_command_age_s is None:
            last_ts = self.state.last_command_ts
            last_command_age_s = ts - last_ts if last_ts else 999.0
        if prior_src and prior_src != src:
            prior_priority = self.priority(prior_src)
            priority_window = max(lock_s, self.arbiter.lock_seconds(prior_src))
            if last_command_age_s <= priority_window and prior_priority > priority:
                return self._decision(
                    False,
                    src,
                    f"recent_{prior_src}_priority",
                    priority,
                    lock_s,
                    prior_priority=prior_priority,
                    age_s=float(last_command_age_s),
                )

        return self._decision(True, src, "allowed", priority, lock_s, confidence=conf)

    def health(self) -> Dict[str, Any]:
        return {
            "cycle_id": self.state.cycle_id,
            "automation_locked": self.state.automation_locked,
            "last_action_ts": dict(self.state.last_action_ts),
            "last_decision": self.state.last_decision.__dict__ if self.state.last_decision else None,
            "last_blocked": self.state.last_blocked.__dict__ if self.state.last_blocked else None,
        }
