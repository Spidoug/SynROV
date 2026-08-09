from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any, Deque, Dict, Iterable, List, Mapping, Optional

from .primitives import normalize_text, safe_float

ROBOT_NAMES = ("Manipulator", "Vehicle", "Drone")


@dataclass(frozen=True)
class RobotPerformanceProfile:
    """Factory profile used by the adaptive runtime.

    The values intentionally favor responsive control while keeping expensive
    perception/training work event-driven.  ``auto_collect`` means collection is
    armed; the ActivityManager decides whether an individual sample is useful.
    """

    robot: str
    ai_gain: float
    music_gain: float
    object_gain: float
    min_confidence: float
    spontaneity_strength: float
    infer_active_ms: int
    infer_idle_ms: int
    feature_active_s: float
    feature_idle_s: float
    min_train_rows: int
    retrain_new_rows: int
    sample_cooldown_s: float
    autonomous_sample_cooldown_s: float
    autonomous_samples_per_minute: int
    frame_sample_min_s: float
    idle_after_s: float
    quiet_before_train_s: float
    train_cooldown_s: float
    autonomous_share: float
    audio_personality_strength: float
    software_image_training: bool
    dataset_ram_batch: int
    dataset_ram_flush_s: float
    auto_collect: bool = True
    auto_retrain: bool = True
    model_follow: bool = True
    direct_voice: bool = True
    music_follow: bool = True
    object_follow: bool = True
    spontaneity: bool = True
    autonomous_learning: bool = True
    audio_personality: bool = True

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


_FACTORY_PROFILES: Dict[str, RobotPerformanceProfile] = {
    "Manipulator": RobotPerformanceProfile(
        robot="Manipulator",
        ai_gain=0.62,
        music_gain=0.28,
        object_gain=0.54,
        min_confidence=0.64,
        spontaneity_strength=0.055,
        infer_active_ms=260,
        infer_idle_ms=1250,
        feature_active_s=0.30,
        feature_idle_s=1.40,
        min_train_rows=120,
        retrain_new_rows=72,
        sample_cooldown_s=0.38,
        autonomous_sample_cooldown_s=2.20,
        autonomous_samples_per_minute=12,
        frame_sample_min_s=3.8,
        idle_after_s=2.2,
        quiet_before_train_s=3.0,
        train_cooldown_s=105.0,
        autonomous_share=0.10,
        audio_personality_strength=0.18,
        software_image_training=True,
        dataset_ram_batch=192,
        dataset_ram_flush_s=18.0,
    ),
    "Vehicle": RobotPerformanceProfile(
        robot="Vehicle",
        ai_gain=0.66,
        music_gain=0.12,
        object_gain=0.58,
        min_confidence=0.72,
        spontaneity_strength=0.032,
        infer_active_ms=285,
        infer_idle_ms=1450,
        feature_active_s=0.34,
        feature_idle_s=1.70,
        min_train_rows=100,
        retrain_new_rows=88,
        sample_cooldown_s=0.42,
        autonomous_sample_cooldown_s=2.80,
        autonomous_samples_per_minute=10,
        frame_sample_min_s=4.4,
        idle_after_s=2.4,
        quiet_before_train_s=3.4,
        train_cooldown_s=120.0,
        autonomous_share=0.08,
        audio_personality_strength=0.075,
        software_image_training=False,
        dataset_ram_batch=224,
        dataset_ram_flush_s=20.0,
    ),
    "Drone": RobotPerformanceProfile(
        robot="Drone",
        ai_gain=0.56,
        music_gain=0.08,
        object_gain=0.46,
        min_confidence=0.78,
        spontaneity_strength=0.020,
        infer_active_ms=320,
        infer_idle_ms=1650,
        feature_active_s=0.38,
        feature_idle_s=1.95,
        min_train_rows=120,
        retrain_new_rows=104,
        sample_cooldown_s=0.46,
        autonomous_sample_cooldown_s=3.20,
        autonomous_samples_per_minute=8,
        frame_sample_min_s=5.0,
        idle_after_s=2.6,
        quiet_before_train_s=3.8,
        train_cooldown_s=135.0,
        autonomous_share=0.055,
        audio_personality_strength=0.050,
        software_image_training=False,
        dataset_ram_batch=224,
        dataset_ram_flush_s=22.0,
    ),
}


def factory_profile(robot: str) -> RobotPerformanceProfile:
    key = str(robot or "Manipulator").strip().lower()
    if key.startswith("veh"):
        return _FACTORY_PROFILES["Vehicle"]
    if key.startswith("dro"):
        return _FACTORY_PROFILES["Drone"]
    return _FACTORY_PROFILES["Manipulator"]


def factory_profiles() -> Dict[str, Dict[str, Any]]:
    return {name: profile.to_dict() for name, profile in _FACTORY_PROFILES.items()}


@dataclass
class BeatState:
    active: bool = False
    beat_now: bool = False
    confidence: float = 0.0
    bpm: float = 0.0
    energy: float = 0.0
    last_beat_ts: float = 0.0


class BeatDetector:
    """Small adaptive onset/tempo detector for the spontaneity music gate.

    A loud signal alone is deliberately insufficient.  Music becomes active only
    after a short sequence of regularly spaced energy onsets, which prevents
    speech, fan noise or one transient from enabling the music behavior.
    """

    def __init__(self) -> None:
        self.energy_floor = 0.004
        self.prev_energy = 0.0
        self.onsets: Deque[float] = deque(maxlen=8)
        self.last_onset_ts = 0.0
        self.period_s = 0.0
        self.state = BeatState()

    @staticmethod
    def _median(values: Iterable[float]) -> float:
        data = sorted(float(v) for v in values)
        if not data:
            return 0.0
        mid = len(data) // 2
        return data[mid] if len(data) % 2 else 0.5 * (data[mid - 1] + data[mid])

    def update(self, audio_feat: Optional[List[float]], now: Optional[float] = None) -> BeatState:
        now = time.time() if now is None else float(now)
        feat = list(audio_feat or [])
        feat.extend([0.0] * max(0, 8 - len(feat)))
        rms = max(0.0, safe_float(feat[0], 0.0))
        peak = max(0.0, safe_float(feat[1], 0.0))
        low = max(0.0, safe_float(feat[4], 0.0))
        mid = max(0.0, safe_float(feat[5], 0.0))
        pulse = safe_float(feat[7], 0.0)
        tonal_energy = max(rms, min(peak, rms * 2.8 + 0.02), 0.55 * low + 0.25 * mid)
        energy = min(2.0, max(0.0, tonal_energy))

        rise = max(0.0, energy - self.prev_energy)
        onset_strength = max(0.0, pulse, rise)
        floor = max(0.0015, self.energy_floor)
        energy_gate = max(0.008, floor * 1.42)
        onset_gate = max(0.0032, floor * 0.22)
        refractory_ok = (now - self.last_onset_ts) >= 0.20
        candidate = energy >= energy_gate and onset_strength >= onset_gate and refractory_ok

        # Adapt primarily on non-onset frames so a real kick does not immediately
        # raise the threshold that is supposed to detect it.
        if candidate:
            self.energy_floor = 0.995 * floor + 0.005 * min(energy, floor * 1.8)
        else:
            bounded = min(energy, max(0.012, floor * 1.35))
            self.energy_floor = 0.975 * floor + 0.025 * bounded

        beat_now = False
        confidence = self.state.confidence
        bpm = self.state.bpm
        if candidate:
            beat_now = True
            self.last_onset_ts = now
            self.onsets.append(now)
            intervals = [b - a for a, b in zip(self.onsets, list(self.onsets)[1:])]
            valid = [d for d in intervals[-5:] if 0.25 <= d <= 1.25]
            if len(valid) >= 2:
                period = self._median(valid)
                spread = self._median(abs(v - period) for v in valid)
                regularity = max(0.0, 1.0 - (spread / max(0.06, period * 0.30)))
                support = min(1.0, len(valid) / 4.0)
                confidence = min(1.0, 0.58 * regularity + 0.42 * support)
                if regularity >= 0.48 and confidence >= 0.54:
                    self.period_s = period
                    bpm = 60.0 / period if period > 0 else 0.0
                    self.state.active = True

        expiry = max(1.05, min(2.8, (self.period_s * 2.45) if self.period_s > 0 else 1.35))
        if self.state.active and now - self.last_onset_ts > expiry:
            self.state.active = False
            confidence *= 0.45
            bpm = 0.0
            self.period_s = 0.0
            self.onsets.clear()
        elif not self.state.active and now - self.last_onset_ts > 1.5:
            confidence *= 0.75

        self.prev_energy = energy
        self.state = BeatState(
            active=bool(self.state.active),
            beat_now=beat_now,
            confidence=max(0.0, min(1.0, confidence)),
            bpm=max(0.0, bpm),
            energy=energy,
            last_beat_ts=self.last_onset_ts,
        )
        return self.state


@dataclass
class CollectionDecision:
    allow: bool
    save_frame: bool = False
    trusted: bool = False
    novel: bool = False
    reason: str = "idle"


class ActivityManager:
    """Event-driven activity policy shared by collection, inference and training."""

    TRUSTED_SOURCES = {"manual", "operator", "teacher", "guided", "voice", "text", "typed", "joystick", "leap", "ui"}
    AUTONOMOUS_SOURCES = {"ai", "model", "ai_spontaneous", "curiosity", "personality_audio", "personality_audio_startle"}
    VISUAL_SOURCES = {"vision", "object", "object_grasp"}

    def __init__(self) -> None:
        self.beat = BeatDetector()
        self.last_activity_ts = time.time()
        self.last_runtime_telemetry: Dict[str, List[float]] = {}
        self.last_saved_telemetry: Dict[str, List[float]] = {}
        self.last_saved_target: Dict[str, List[float]] = {}
        self.last_saved_source: Dict[str, str] = {}
        self.last_sample_ts: Dict[str, float] = {}
        self.last_frame_sample_ts: Dict[str, float] = {}
        self.autonomous_sample_times: Dict[str, Deque[float]] = {name: deque() for name in ROBOT_NAMES}
        self.last_model_ts: Dict[str, float] = {}
        self.last_train_ts: Dict[str, float] = {}
        self.accepted_since_train: Dict[str, int] = {name: 0 for name in ROBOT_NAMES}
        self.trusted_since_train: Dict[str, int] = {name: 0 for name in ROBOT_NAMES}
        self.novel_since_train: Dict[str, int] = {name: 0 for name in ROBOT_NAMES}
        self.last_reject_reason = ""

    @staticmethod
    def _robot(robot: str) -> str:
        return factory_profile(robot).robot

    @staticmethod
    def _vector(values: Optional[List[float]]) -> List[float]:
        out: List[float] = []
        for value in list(values or []):
            number = safe_float(value, 0.0)
            if math.isfinite(number):
                out.append(number)
            else:
                out.append(0.0)
        return out

    @classmethod
    def _mean_abs(cls, values: Optional[List[float]]) -> float:
        vec = cls._vector(values)
        return sum(abs(v) for v in vec) / len(vec) if vec else 0.0

    @classmethod
    def _mean_delta(cls, a: Optional[List[float]], b: Optional[List[float]]) -> float:
        va, vb = cls._vector(a), cls._vector(b)
        size = max(len(va), len(vb))
        if size <= 0:
            return 0.0
        va.extend([0.0] * (size - len(va)))
        vb.extend([0.0] * (size - len(vb)))
        return sum(abs(x - y) for x, y in zip(va, vb)) / size

    def observe_audio(self, audio_feat: Optional[List[float]], now: Optional[float] = None) -> BeatState:
        return self.beat.update(audio_feat, now=now)

    def observe_runtime(
        self,
        robot: str,
        telemetry: Optional[List[float]],
        voice_ctx: Optional[Mapping[str, Any]] = None,
        vision_confidence: float = 0.0,
        last_command_age_s: float = 999.0,
        mission_active: bool = False,
        now: Optional[float] = None,
    ) -> None:
        now = time.time() if now is None else float(now)
        robot = self._robot(robot)
        telemetry_vec = self._vector(telemetry)
        previous = self.last_runtime_telemetry.get(robot)
        telemetry_delta = self._mean_delta(telemetry_vec, previous)
        self.last_runtime_telemetry[robot] = telemetry_vec
        voice = dict(voice_ctx or {})
        fresh_voice = safe_float(voice.get("text_age", 999.0), 999.0) <= 1.25 and (
            normalize_text(str(voice.get("norm", ""))) or str(voice.get("intent", "none")) != "none"
        )
        telemetry_active = telemetry_delta > (0.030 if robot == "Manipulator" else 0.006)
        if fresh_voice or mission_active or last_command_age_s <= 0.95 or telemetry_active or self.beat.state.active:
            self.last_activity_ts = now

    def is_idle(self, robot: str, now: Optional[float] = None) -> bool:
        now = time.time() if now is None else float(now)
        return (now - self.last_activity_ts) >= factory_profile(robot).idle_after_s

    def feature_period_s(self, robot: str, visual_active: bool = False, now: Optional[float] = None) -> float:
        profile = factory_profile(robot)
        if visual_active or not self.is_idle(robot, now=now):
            return profile.feature_active_s
        return profile.feature_idle_s

    def model_due(self, robot: str, context_active: bool = False, now: Optional[float] = None) -> bool:
        now = time.time() if now is None else float(now)
        robot = self._robot(robot)
        profile = factory_profile(robot)
        interval = profile.infer_active_ms / 1000.0 if (context_active or not self.is_idle(robot, now)) else profile.infer_idle_ms / 1000.0
        last = self.last_model_ts.get(robot, 0.0)
        if now - last < interval:
            return False
        self.last_model_ts[robot] = now
        return True

    def collection_decision(
        self,
        robot: str,
        source: str,
        target: Optional[List[float]],
        telemetry: Optional[List[float]],
        voice_ctx: Optional[Mapping[str, Any]],
        vision_confidence: float,
        now: Optional[float] = None,
        image_available: bool = False,
    ) -> CollectionDecision:
        now = time.time() if now is None else float(now)
        robot = self._robot(robot)
        profile = factory_profile(robot)
        src = normalize_text(str(source or "observe")).replace(" ", "_") or "observe"
        if src.startswith("adaptive_"):
            src = src[len("adaptive_"):] or "ai"
        elif src.startswith("curiosity"):
            src = "curiosity"
        target_vec = self._vector(target)
        telemetry_vec = self._vector(telemetry)
        voice = dict(voice_ctx or {})
        voice_fresh = safe_float(voice.get("text_age", 999.0), 999.0) <= 1.25 and (
            str(voice.get("intent", "none")) != "none" or len(normalize_text(str(voice.get("norm", "")))) >= 3
        )
        threshold = 0.22 if robot == "Manipulator" else 0.014
        target_significant = self._mean_abs(target_vec) > threshold
        last_target = self.last_saved_target.get(robot)
        target_delta = self._mean_delta(target_vec, last_target)
        telemetry_delta = self._mean_delta(telemetry_vec, self.last_saved_telemetry.get(robot))
        source_changed = src != self.last_saved_source.get(robot, "")
        trusted = src in self.TRUSTED_SOURCES or voice_fresh
        autonomous = src in self.AUTONOMOUS_SOURCES
        visual = src in self.VISUAL_SOURCES or safe_float(vision_confidence, 0.0) >= 0.56
        music = src == "music"
        stop_transition = last_target is not None and self._mean_abs(last_target) > threshold and not target_significant
        novel = source_changed or stop_transition or target_delta > (0.16 if robot == "Manipulator" else 0.022) or telemetry_delta > (0.035 if robot == "Manipulator" else 0.008) or voice_fresh

        if music and not self.beat.state.active:
            self.last_reject_reason = "music_without_beat"
            return CollectionDecision(False, reason=self.last_reject_reason)
        if src in {"observe", "telemetry", "idle", "none"} and not (target_significant or stop_transition or voice_fresh):
            self.last_reject_reason = "idle_telemetry"
            return CollectionDecision(False, reason=self.last_reject_reason)
        if visual and not (target_significant or stop_transition or voice_fresh):
            self.last_reject_reason = "static_vision"
            return CollectionDecision(False, reason=self.last_reject_reason)
        if autonomous and not (target_significant or stop_transition) and not voice_fresh:
            self.last_reject_reason = "autonomy_without_action"
            return CollectionDecision(False, reason=self.last_reject_reason)

        last_ts = self.last_sample_ts.get(robot, 0.0)
        auto_limited = autonomous or music or visual
        if auto_limited:
            recent = self.autonomous_sample_times.setdefault(robot, deque())
            while recent and now - recent[0] > 60.0:
                recent.popleft()
            if len(recent) >= profile.autonomous_samples_per_minute:
                self.last_reject_reason = "autonomous_sample_budget"
                return CollectionDecision(False, reason=self.last_reject_reason)
        cooldown = profile.autonomous_sample_cooldown_s if auto_limited else profile.sample_cooldown_s
        if now - last_ts < cooldown and not voice_fresh and not stop_transition:
            self.last_reject_reason = "sample_cooldown"
            return CollectionDecision(False, reason=self.last_reject_reason)
        if (autonomous or music or visual) and not novel and not self.beat.state.beat_now:
            self.last_reject_reason = "repeated_autonomy"
            return CollectionDecision(False, reason=self.last_reject_reason)
        if not (trusted or target_significant or stop_transition or visual or music):
            self.last_reject_reason = "no_learning_signal"
            return CollectionDecision(False, reason=self.last_reject_reason)

        self.last_sample_ts[robot] = now
        if autonomous or music or visual:
            self.autonomous_sample_times.setdefault(robot, deque()).append(now)
        self.last_saved_target[robot] = target_vec
        self.last_saved_telemetry[robot] = telemetry_vec
        self.last_saved_source[robot] = src
        self.accepted_since_train[robot] = self.accepted_since_train.get(robot, 0) + 1
        if trusted:
            self.trusted_since_train[robot] = self.trusted_since_train.get(robot, 0) + 1
        if novel:
            self.novel_since_train[robot] = self.novel_since_train.get(robot, 0) + 1
        self.last_activity_ts = now

        save_frame = False
        frame_relevant = image_available and (visual or voice_fresh or (trusted and target_significant))
        if frame_relevant and now - self.last_frame_sample_ts.get(robot, 0.0) >= profile.frame_sample_min_s:
            save_frame = True
            self.last_frame_sample_ts[robot] = now
        self.last_reject_reason = "accepted"
        return CollectionDecision(True, save_frame=save_frame, trusted=trusted, novel=novel, reason="accepted")

    def should_train(
        self,
        robot: str,
        row_count: int,
        last_train_rows: int,
        training: bool,
        model_exists: bool,
        auto_retrain: bool = True,
        now: Optional[float] = None,
    ) -> bool:
        now = time.time() if now is None else float(now)
        robot = self._robot(robot)
        profile = factory_profile(robot)
        if not auto_retrain or training or row_count < profile.min_train_rows:
            return False
        if now - self.last_activity_ts < profile.quiet_before_train_s:
            return False
        if now - self.last_train_ts.get(robot, 0.0) < profile.train_cooldown_s:
            return False
        new_rows = max(0, int(row_count) - int(last_train_rows))
        accepted = self.accepted_since_train.get(robot, 0)
        trusted = self.trusted_since_train.get(robot, 0)
        novel = self.novel_since_train.get(robot, 0)
        if not model_exists:
            return row_count >= profile.min_train_rows and (accepted >= 8 or last_train_rows <= 0)
        if new_rows < profile.retrain_new_rows:
            return False
        # Require real novelty/quality from this session; a large old dataset by
        # itself is not a reason to keep retraining forever.
        return accepted >= 14 and novel >= 7 and (trusted >= 3 or accepted >= 28)

    def note_train_started(self, robot: str, now: Optional[float] = None) -> None:
        robot = self._robot(robot)
        self.last_train_ts[robot] = time.time() if now is None else float(now)

    def note_train_completed(self, robot: str) -> None:
        robot = self._robot(robot)
        self.accepted_since_train[robot] = 0
        self.trusted_since_train[robot] = 0
        self.novel_since_train[robot] = 0

    def stats(self, robot: str) -> Dict[str, Any]:
        robot = self._robot(robot)
        return {
            "robot": robot,
            "idle": self.is_idle(robot),
            "beat_active": self.beat.state.active,
            "beat_confidence": self.beat.state.confidence,
            "beat_bpm": self.beat.state.bpm,
            "accepted_since_train": self.accepted_since_train.get(robot, 0),
            "trusted_since_train": self.trusted_since_train.get(robot, 0),
            "novel_since_train": self.novel_since_train.get(robot, 0),
            "last_reject_reason": self.last_reject_reason,
        }
