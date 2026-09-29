# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Stateful target association for the MDBOT worker follower."""

from __future__ import annotations

from dataclasses import dataclass
import math


Point2D = tuple[float, float]


@dataclass(frozen=True)
class TargetLockConfig:
    """Tuning values for conservative worker target association."""

    search_confirm_frames: int = 20
    search_match_distance_m: float = 0.45
    search_max_missed_frames: int = 3
    match_gate_m: float = 0.55
    ambiguity_proximity_m: float = 0.65
    ambiguity_score_margin_m: float = 0.12
    reacquire_confirm_frames: int = 2
    prediction_hold_frames: int = 10
    ambiguous_prediction_hold_frames: int = 2
    max_missed_frames: int = 16
    max_ambiguous_frames: int = 50
    position_alpha: float = 0.65
    velocity_alpha: float = 0.20
    max_target_speed_mps: float = 2.0


@dataclass(frozen=True)
class TargetLockResult:
    """One target-association result returned to the ROS-facing node."""

    state: str
    position: Point2D | None
    target_id: int | None
    confidence: float


class RobustTargetLock:
    """Keep one worker locked and bridge brief uncertain detections safely."""

    def __init__(self, config: TargetLockConfig) -> None:
        """Initialize target, candidate, and motion-estimation state."""
        self.config = config
        self.state = "SEARCH"
        self.position: Point2D | None = None
        self.velocity: Point2D = (0.0, 0.0)
        self.target_id: int | None = None
        self.next_target_id = 1
        self.last_update_sec: float | None = None
        self.candidate_position: Point2D | None = None
        self.candidate_frames = 0
        self.search_missed_frames = 0
        self.reacquire_position: Point2D | None = None
        self.reacquire_frames = 0
        self.missed_frames = 0
        self.ambiguous_frames = 0

    def search(
        self,
        detections: list[Point2D],
        now_sec: float,
    ) -> TargetLockResult:
        """Confirm that the same nearby detection persists before locking it."""
        if not detections:
            self.search_missed_frames += 1
            if self.search_missed_frames > max(
                0,
                int(self.config.search_max_missed_frames),
            ):
                self._clear_search_candidate()
            self.state = "SEARCH"
            required = max(1, int(self.config.search_confirm_frames))
            confidence = min(1.0, self.candidate_frames / required)
            return self._result(None, confidence)

        self.search_missed_frames = 0

        nearest = min(detections, key=self._distance_from_origin)
        if self.candidate_position is None:
            self._start_search_candidate(nearest)
        else:
            matched = min(
                detections,
                key=lambda point: self._distance(point, self.candidate_position),
            )
            if (
                self._distance(matched, self.candidate_position)
                <= self.config.search_match_distance_m
            ):
                self.candidate_position = matched
                self.candidate_frames += 1
            else:
                self._start_search_candidate(nearest)

        required = max(1, int(self.config.search_confirm_frames))
        confidence = min(1.0, self.candidate_frames / required)
        if self.candidate_frames < required:
            self.state = "SEARCH"
            return self._result(None, confidence)

        self.position = self.candidate_position
        self.velocity = (0.0, 0.0)
        self.target_id = self.next_target_id
        self.next_target_id += 1
        self.last_update_sec = float(now_sec)
        self.state = "LOCKED"
        self.missed_frames = 0
        self.ambiguous_frames = 0
        self._clear_search_candidate()
        return self._result(self.position, 1.0)

    def update(
        self,
        detections: list[Point2D],
        now_sec: float,
    ) -> TargetLockResult:
        """Associate detections with the locked worker using motion continuity."""
        if self.position is None or self.target_id is None:
            self.reset()
            return self._result(None, 0.0)

        dt = self._bounded_dt(now_sec)
        predicted = self._predict(dt)
        gated = sorted(
            (
                (self._distance(point, predicted), point)
                for point in detections
                if self._distance(point, predicted) <= self._dynamic_gate(dt)
            ),
            key=lambda item: item[0],
        )

        if not gated:
            return self._handle_missing(predicted, now_sec)

        if self._association_is_ambiguous(gated):
            return self._handle_ambiguous(predicted, now_sec)

        measurement = gated[0][1]
        if self.state in {"AMBIGUOUS", "OCCLUDED"}:
            return self._confirm_reacquisition(measurement, predicted, now_sec, dt)

        return self._accept_measurement(measurement, predicted, now_sec, dt)

    def reset(self) -> None:
        """Return to SEARCH without retaining a stale worker identity."""
        self.state = "SEARCH"
        self.position = None
        self.velocity = (0.0, 0.0)
        self.target_id = None
        self.last_update_sec = None
        self.reacquire_position = None
        self.reacquire_frames = 0
        self.missed_frames = 0
        self.ambiguous_frames = 0
        self._clear_search_candidate()

    def predicted_position(self, now_sec: float) -> Point2D | None:
        """Return the bounded constant-velocity prediction for diagnostics."""
        if self.position is None:
            return None
        return self._predict(self._bounded_dt(now_sec))

    def _handle_missing(
        self,
        predicted: Point2D,
        now_sec: float,
    ) -> TargetLockResult:
        self.missed_frames += 1
        self.state = "OCCLUDED"
        self.position = predicted
        self.velocity = (self.velocity[0] * 0.90, self.velocity[1] * 0.90)
        self.last_update_sec = float(now_sec)
        self._clear_reacquisition()
        if self.missed_frames > max(0, int(self.config.max_missed_frames)):
            self.state = "LOST"
            self.position = None
            self.velocity = (0.0, 0.0)
            self.target_id = None
            return self._result(None, 0.0)
        prediction_hold_frames = max(
            0,
            int(self.config.prediction_hold_frames),
        )
        if self.missed_frames <= prediction_hold_frames:
            confidence = 1.0 - self.missed_frames / (prediction_hold_frames + 1)
            return self._result(predicted, confidence)
        return self._result(None, 0.0)

    def _handle_ambiguous(
        self,
        predicted: Point2D,
        now_sec: float,
    ) -> TargetLockResult:
        self.ambiguous_frames += 1
        self.state = "AMBIGUOUS"
        self.position = predicted
        self.velocity = (self.velocity[0] * 0.95, self.velocity[1] * 0.95)
        self.last_update_sec = float(now_sec)
        self._clear_reacquisition()
        if self.ambiguous_frames > max(0, int(self.config.max_ambiguous_frames)):
            self.state = "LOST"
            self.position = None
            self.velocity = (0.0, 0.0)
            self.target_id = None
            return self._result(None, 0.0)
        prediction_hold_frames = max(
            0,
            int(self.config.ambiguous_prediction_hold_frames),
        )
        if self.ambiguous_frames <= prediction_hold_frames:
            confidence = 0.5 * (
                1.0 - self.ambiguous_frames / (prediction_hold_frames + 1)
            )
            return self._result(predicted, confidence)
        return self._result(None, 0.0)

    def _confirm_reacquisition(
        self,
        measurement: Point2D,
        predicted: Point2D,
        now_sec: float,
        dt: float,
    ) -> TargetLockResult:
        if (
            self.reacquire_position is None
            or self._distance(measurement, self.reacquire_position)
            > self.config.search_match_distance_m
        ):
            self.reacquire_position = measurement
            self.reacquire_frames = 1
        else:
            self.reacquire_position = measurement
            self.reacquire_frames += 1

        required = max(1, int(self.config.reacquire_confirm_frames))
        if self.reacquire_frames < required:
            self.position = predicted
            self.last_update_sec = float(now_sec)
            return self._result(predicted, self.reacquire_frames / required)

        self._clear_reacquisition()
        return self._accept_measurement(measurement, predicted, now_sec, dt)

    def _accept_measurement(
        self,
        measurement: Point2D,
        predicted: Point2D,
        now_sec: float,
        dt: float,
    ) -> TargetLockResult:
        previous = self.position or predicted
        alpha = min(1.0, max(0.0, float(self.config.position_alpha)))
        filtered = (
            alpha * measurement[0] + (1.0 - alpha) * predicted[0],
            alpha * measurement[1] + (1.0 - alpha) * predicted[1],
        )
        measured_velocity = (
            (filtered[0] - previous[0]) / max(dt, 1e-3),
            (filtered[1] - previous[1]) / max(dt, 1e-3),
        )
        measured_velocity = self._limit_vector(
            measured_velocity,
            max(0.0, float(self.config.max_target_speed_mps)),
        )
        velocity_alpha = min(1.0, max(0.0, float(self.config.velocity_alpha)))
        self.velocity = (
            velocity_alpha * measured_velocity[0]
            + (1.0 - velocity_alpha) * self.velocity[0],
            velocity_alpha * measured_velocity[1]
            + (1.0 - velocity_alpha) * self.velocity[1],
        )
        self.position = filtered
        self.last_update_sec = float(now_sec)
        self.state = "LOCKED"
        self.missed_frames = 0
        self.ambiguous_frames = 0
        return self._result(filtered, 1.0)

    def _association_is_ambiguous(
        self,
        gated: list[tuple[float, Point2D]],
    ) -> bool:
        if len(gated) < 2:
            return False
        best_score, best = gated[0]
        second_score, second = gated[1]
        score_gap = second_score - best_score
        score_margin = max(
            0.0,
            float(self.config.ambiguity_score_margin_m),
        )
        close_people = self._distance(best, second) <= max(
            0.0,
            float(self.config.ambiguity_proximity_m),
        )
        if close_people:
            return score_gap <= score_margin
        return score_gap <= score_margin * 0.5

    def _bounded_dt(self, now_sec: float) -> float:
        if self.last_update_sec is None:
            return 0.05
        return max(0.01, min(0.25, float(now_sec) - self.last_update_sec))

    def _predict(self, dt: float) -> Point2D:
        position = self.position or (0.0, 0.0)
        return (
            position[0] + self.velocity[0] * dt,
            position[1] + self.velocity[1] * dt,
        )

    def _dynamic_gate(self, dt: float) -> float:
        speed = math.hypot(*self.velocity)
        return max(0.05, float(self.config.match_gate_m)) + speed * dt

    def _result(
        self,
        position: Point2D | None,
        confidence: float,
    ) -> TargetLockResult:
        return TargetLockResult(
            state=self.state,
            position=position,
            target_id=self.target_id,
            confidence=max(0.0, min(1.0, float(confidence))),
        )

    def _start_search_candidate(self, point: Point2D) -> None:
        self.candidate_position = point
        self.candidate_frames = 1

    def _clear_search_candidate(self) -> None:
        self.candidate_position = None
        self.candidate_frames = 0
        self.search_missed_frames = 0

    def _clear_reacquisition(self) -> None:
        self.reacquire_position = None
        self.reacquire_frames = 0

    @staticmethod
    def _distance(first: Point2D, second: Point2D) -> float:
        return math.hypot(first[0] - second[0], first[1] - second[1])

    @staticmethod
    def _distance_from_origin(point: Point2D) -> float:
        return math.hypot(point[0], point[1])

    @staticmethod
    def _limit_vector(vector: Point2D, max_length: float) -> Point2D:
        length = math.hypot(*vector)
        if max_length <= 0.0 or length <= max_length:
            return vector
        scale = max_length / max(length, 1e-6)
        return vector[0] * scale, vector[1] * scale
