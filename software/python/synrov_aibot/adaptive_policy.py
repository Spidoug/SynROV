"""Bounded persistent mission-strategy learning for SynROV AiBot.

The learner never invents actuator commands.  It selects only among explicitly
approved strategy variants supplied by the robot AI, observes mission/sensor
outcomes, and stores aggregate performance plus a compact JSONL experience
trail.  Firmware/Processing safety gates remain the final authority.
"""
from __future__ import annotations

import json
import math
import pickle
import time
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

from .primitives import safe_float
from .protocol import SOFTWARE_VERSION
from .robot_types import canonical_robot


SAFE_MISSION_VARIANTS: Dict[str, Dict[str, Tuple[str, ...]]] = {
    "Manipulator": {
        "scan_workspace": ("balanced", "precision", "wide"),
        "inspect_workspace": ("balanced", "wrist_focus", "base_focus"),
    },
    "Vehicle": {
        "patrol_area": ("balanced", "careful", "wide_sweep"),
        "perimeter_scan": ("balanced", "careful", "wide_camera"),
        "corridor_scan": ("balanced", "careful", "camera_sweep"),
        "follow_target": ("balanced", "careful", "responsive"),
        "terrain_inspection": ("balanced", "careful", "wide_sweep"),
        "locate_objects": ("balanced", "slow_scan", "wide_scan"),
        "find_exit": ("balanced", "careful", "decisive"),
    },
    "Drone": {
        "aerial_scan": ("balanced", "careful", "wide_scan"),
        "orbit_point": ("balanced", "careful", "wide_orbit"),
        "search_pattern": ("balanced", "slow_scan", "wide_scan"),
        "terrain_inspection": ("balanced", "careful", "wide_sweep"),
        "locate_objects": ("balanced", "slow_scan", "wide_scan"),
        "find_exit": ("balanced", "careful", "decisive"),
    },
}


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _finite(value: Any, default: float = 0.0) -> float:
    value = safe_float(value, default)
    return value if math.isfinite(value) else default


class AdaptiveMissionPolicy:
    """Small UCB learner over bounded, pre-approved mission strategies."""

    SEGMENT_SECONDS = 6.0
    OBSERVE_INTERVAL_SECONDS = 1.0
    UCB_EXPLORATION = 0.34

    def __init__(self, robot: Any, policy_path: Path, dataset_path: Path) -> None:
        self.robot = canonical_robot(robot)
        self.policy_path = Path(policy_path)
        self.dataset_path = Path(dataset_path)
        self.variants = SAFE_MISSION_VARIANTS.get(self.robot, {})
        self.state: Dict[str, Any] = {
            "softwareVersion": SOFTWARE_VERSION,
            "robot": self.robot,
            "episodes": 0,
            "missions": {},
            "updated_ts": 0.0,
        }
        self._load()

    def _load(self) -> None:
        if not self.policy_path.is_file():
            return
        try:
            with self.policy_path.open("rb") as fh:
                raw = pickle.load(fh)
            if not isinstance(raw, dict) or raw.get("robot") != self.robot:
                return
            if int(raw.get("softwareVersion", 0)) != SOFTWARE_VERSION:
                return
            self.state.update(raw)
            if not isinstance(self.state.get("missions"), dict):
                self.state["missions"] = {}
        except Exception:
            # An invalid model file must never prevent AiBot start.
            return

    def _save(self) -> None:
        try:
            self.policy_path.parent.mkdir(parents=True, exist_ok=True)
            self.state["updated_ts"] = time.time()
            tmp = self.policy_path.with_suffix(self.policy_path.suffix + ".tmp")
            with tmp.open("wb") as fh:
                pickle.dump(self.state, fh, protocol=pickle.HIGHEST_PROTOCOL)
            tmp.replace(self.policy_path)
        except Exception:
            pass

    def _mission_stats(self, mission: str) -> Dict[str, Any]:
        missions = self.state.setdefault("missions", {})
        stats = missions.setdefault(mission, {"segments": 0, "variants": {}})
        if not isinstance(stats.get("variants"), dict):
            stats["variants"] = {}
        for variant in self.variants_for(mission):
            stats["variants"].setdefault(
                variant,
                {"selections": 0, "observations": 0, "mean_reward": 0.0, "reward_sum": 0.0},
            )
        return stats

    def variants_for(self, mission: str) -> Tuple[str, ...]:
        values = tuple(self.variants.get(str(mission or ""), ()))
        return values or ("default",)

    def begin_mission(self, mission: str, mission_data: Dict[str, Any]) -> Dict[str, Any]:
        self.state["episodes"] = int(self.state.get("episodes", 0)) + 1
        mission_data["_learning_segment"] = -1
        mission_data["_learning_variant"] = ""
        mission_data["_learning_last_observe"] = 0.0
        mission_data["_learning_variant_changed"] = False
        variant = self.variant_for_tick(mission, mission_data, 0.0)
        self._save()
        return self.snapshot(mission, variant, exploring=True)

    def _select_variant(self, mission: str) -> Tuple[str, bool]:
        variants = self.variants_for(mission)
        stats = self._mission_stats(mission)
        vstats = stats["variants"]

        # Try every safe strategy before exploiting learned performance.
        for variant in variants:
            if int(vstats[variant].get("observations", 0)) <= 0:
                return variant, True

        total = sum(max(1, int(vstats[v].get("observations", 0))) for v in variants)
        best_variant = variants[0]
        best_score = -1e9
        for variant in variants:
            item = vstats[variant]
            count = max(1, int(item.get("observations", 0)))
            mean = _finite(item.get("mean_reward", 0.0), 0.0)
            bonus = self.UCB_EXPLORATION * math.sqrt(math.log(total + 1.0) / count)
            score = mean + bonus
            if score > best_score:
                best_score = score
                best_variant = variant
        return best_variant, False

    def variant_for_tick(self, mission: str, mission_data: Dict[str, Any], elapsed: float) -> str:
        segment = max(0, int(max(0.0, elapsed) // self.SEGMENT_SECONDS))
        previous = int(mission_data.get("_learning_segment", -1))
        if segment == previous and mission_data.get("_learning_variant"):
            mission_data["_learning_variant_changed"] = False
            return str(mission_data["_learning_variant"])

        variant, exploring = self._select_variant(mission)
        stats = self._mission_stats(mission)
        stats["segments"] = int(stats.get("segments", 0)) + 1
        item = stats["variants"][variant]
        item["selections"] = int(item.get("selections", 0)) + 1
        mission_data["_learning_segment"] = segment
        mission_data["_learning_variant"] = variant
        mission_data["_learning_exploring"] = bool(exploring)
        mission_data["_learning_variant_changed"] = True
        self._save()
        return variant

    def _reward(self, mission: str, state: Any, metadata: Mapping[str, Any]) -> float:
        reward = 0.06  # small reward for a safe, successfully executed mission tick
        vision = _mapping(metadata.get("vision"))
        if vision:
            confidence = max(0.0, min(1.0, _finite(vision.get("confidence"), 0.0)))
            reward += 0.70 * confidence
            reward -= 0.20 * min(1.0, abs(_finite(vision.get("dx"), 0.0)))
            if bool(vision.get("centered")):
                reward += 0.28
            if bool(vision.get("close")):
                reward += 0.34
        if bool(metadata.get("object_localized")):
            reward += 1.35

        distance = metadata.get("distance_cm")
        if distance is not None:
            d = max(0.0, _finite(distance, 0.0))
            reward += 0.16 * min(1.0, d / 120.0)

        avoidance = str(metadata.get("avoidance", "") or "")
        if avoidance == "escape":
            reward -= 0.42
        elif avoidance == "slowdown":
            reward -= 0.10
        if bool(metadata.get("collision_filtered")) or bool(metadata.get("collision_relief")):
            reward -= 0.28
        if metadata.get("resource_guard") or metadata.get("link_guard"):
            reward -= 0.65
        if bool(metadata.get("ground_guard")):
            reward -= 0.15

        if mission == "find_exit":
            scores = _mapping(metadata.get("corridor_scores"))
            if scores:
                finite_scores = [_finite(value, 0.0) for value in scores.values()]
                if finite_scores:
                    reward += 0.10 * max(-1.0, min(1.0, max(finite_scores) / 8.0))
            occupancy = _mapping(metadata.get("occupancy"))
            if occupancy:
                front = max(0.0, _finite(occupancy.get("front"), 0.0))
                reward += 0.18 / (1.0 + front)

        # Resource quality contributes gently; safety gates still dominate.
        battery = metadata.get("battery_pct")
        if battery is not None:
            reward += 0.05 * max(0.0, min(1.0, _finite(battery, 0.0) / 100.0))
        link = metadata.get("communication_quality_pct")
        if link is not None:
            reward += 0.05 * max(0.0, min(1.0, _finite(link, 0.0) / 100.0))

        return max(-1.5, min(2.5, reward))

    def _context(self, state: Any, metadata: Mapping[str, Any]) -> Dict[str, Any]:
        sensors = _mapping(getattr(state, "sensors", {}))
        vision = _mapping(metadata.get("vision"))
        return {
            "battery_pct": metadata.get("battery_pct", sensors.get("battery_pct")),
            "communication_quality_pct": metadata.get(
                "communication_quality_pct", sensors.get("communication_quality_pct")
            ),
            "distance_cm": metadata.get("distance_cm"),
            "vision_confidence": vision.get("confidence"),
            "vision_dx": vision.get("dx"),
            "vision_area": vision.get("area"),
            "collision_filtered": bool(metadata.get("collision_filtered", False)),
            "strategy": str(metadata.get("strategy", "") or ""),
        }

    def observe(
        self,
        mission: str,
        mission_data: Dict[str, Any],
        state: Any,
        metadata: Mapping[str, Any],
        elapsed: float,
        *,
        force: bool = False,
    ) -> Dict[str, Any]:
        variant = str(mission_data.get("_learning_variant") or self.variant_for_tick(mission, mission_data, elapsed))
        now = time.time()
        last = _finite(mission_data.get("_learning_last_observe", 0.0), 0.0)
        if not force and last and (now - last) < self.OBSERVE_INTERVAL_SECONDS:
            return self.snapshot(
                mission,
                variant,
                exploring=bool(mission_data.get("_learning_exploring", False)),
                changed=bool(mission_data.get("_learning_variant_changed", False)),
            )

        reward = self._reward(mission, state, metadata)
        stats = self._mission_stats(mission)
        item = stats["variants"].setdefault(
            variant, {"selections": 0, "observations": 0, "mean_reward": 0.0, "reward_sum": 0.0}
        )
        count = int(item.get("observations", 0)) + 1
        total = _finite(item.get("reward_sum", 0.0), 0.0) + reward
        item["observations"] = count
        item["reward_sum"] = total
        item["mean_reward"] = total / max(1, count)
        item["last_reward"] = reward
        item["updated_ts"] = now
        mission_data["_learning_last_observe"] = now
        self._save()

        record = {
            "timestamp": now,
            "robot": self.robot,
            "mission": mission,
            "segment": int(mission_data.get("_learning_segment", 0)),
            "variant": variant,
            "reward": reward,
            "elapsed_s": max(0.0, float(elapsed)),
            "context": self._context(state, metadata),
        }
        try:
            self.dataset_path.parent.mkdir(parents=True, exist_ok=True)
            with self.dataset_path.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
        except Exception:
            pass

        return self.snapshot(
            mission,
            variant,
            exploring=bool(mission_data.get("_learning_exploring", False)),
            changed=bool(mission_data.get("_learning_variant_changed", False)),
            reward=reward,
        )

    def snapshot(
        self,
        mission: str,
        variant: str,
        *,
        exploring: bool = False,
        changed: bool = False,
        reward: float | None = None,
    ) -> Dict[str, Any]:
        stats = self._mission_stats(mission)
        item = stats["variants"].get(variant, {})
        out = {
            "enabled": len(self.variants_for(mission)) > 1,
            "mode": "bounded_ucb_strategy_learning",
            "variant": variant,
            "available_variants": list(self.variants_for(mission)),
            "exploring": bool(exploring),
            "variant_changed": bool(changed),
            "observations": int(item.get("observations", 0)),
            "mean_reward": _finite(item.get("mean_reward", 0.0), 0.0),
            "policy_path": str(self.policy_path),
            "dataset_path": str(self.dataset_path),
        }
        if reward is not None:
            out["reward"] = float(reward)
        return out

    def metadata(self) -> Dict[str, Any]:
        return {
            "learning_mode": "bounded_ucb_strategy_learning",
            "episodes": int(self.state.get("episodes", 0)),
            "trainable_missions": sorted(self.variants.keys()),
            "policy_exists": self.policy_path.is_file(),
            "dataset_exists": self.dataset_path.is_file(),
            "mission_stats": self.state.get("missions", {}),
        }


__all__ = ["AdaptiveMissionPolicy", "SAFE_MISSION_VARIANTS"]
