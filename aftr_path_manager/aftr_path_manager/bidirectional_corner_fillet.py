# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Direction-symmetric corner fillets for recorded replay paths."""

from dataclasses import dataclass
import math

from aftr_path_manager.path_types import RecordedPose


@dataclass(frozen=True)
class CornerFilletSummary:
    """Describe the result without making path feasibility decisions."""

    poses: list[RecordedPose]
    detected_corners: int
    modified_corners: int
    minimum_applied_radius_m: float
    degraded_corners: int = 0
    maximum_curvature_rate_per_m2: float = 0.0
    applied_radii_m: tuple[float, ...] = ()


@dataclass(frozen=True)
class _Corner:
    center_s: float
    heading_change: float


@dataclass(frozen=True)
class _Replacement:
    start_s: float
    end_s: float
    samples: list[RecordedPose]
    applied_radius_m: float


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _cumulative_lengths(poses: list[RecordedPose]) -> list[float]:
    lengths = [0.0]
    for start, end in zip(poses, poses[1:]):
        lengths.append(lengths[-1] + math.hypot(end.x - start.x, end.y - start.y))
    return lengths


def _segment_headings(poses: list[RecordedPose]) -> list[float]:
    headings = []
    for start, end in zip(poses, poses[1:]):
        headings.append(math.atan2(end.y - start.y, end.x - start.x))
    return headings


def _detect_corners(
    poses: list[RecordedPose],
    cumulative: list[float],
    heading_threshold_rad: float,
    merge_gap_m: float,
) -> list[_Corner]:
    headings = _segment_headings(poses)
    turn_samples = []
    for index in range(1, len(headings)):
        delta = _normalize_angle(headings[index] - headings[index - 1])
        if abs(delta) >= math.radians(0.35):
            turn_samples.append((cumulative[index], delta))

    if not turn_samples:
        return []

    groups = [[turn_samples[0]]]
    for sample in turn_samples[1:]:
        if sample[0] - groups[-1][-1][0] <= merge_gap_m:
            groups[-1].append(sample)
        else:
            groups.append([sample])

    corners = []
    for group in groups:
        total_turn = sum(sample[1] for sample in group)
        if abs(total_turn) < heading_threshold_rad:
            continue
        weight = sum(abs(sample[1]) for sample in group)
        center_s = sum(sample[0] * abs(sample[1]) for sample in group) / weight
        corners.append(_Corner(center_s=center_s, heading_change=total_turn))
    return corners


def _pose_at_s(
    poses: list[RecordedPose],
    cumulative: list[float],
    distance_s: float,
) -> RecordedPose:
    distance_s = min(max(distance_s, 0.0), cumulative[-1])
    segment_index = 0
    while (
        segment_index + 1 < len(cumulative)
        and cumulative[segment_index + 1] < distance_s
    ):
        segment_index += 1

    if segment_index >= len(poses) - 1:
        return poses[-1]

    start = poses[segment_index]
    end = poses[segment_index + 1]
    segment_length = cumulative[segment_index + 1] - cumulative[segment_index]
    ratio = 0.0 if segment_length <= 1e-9 else (
        distance_s - cumulative[segment_index]
    ) / segment_length
    return RecordedPose(
        stamp_sec=start.stamp_sec + (end.stamp_sec - start.stamp_sec) * ratio,
        frame_id=start.frame_id,
        x=start.x + (end.x - start.x) * ratio,
        y=start.y + (end.y - start.y) * ratio,
        yaw=math.atan2(end.y - start.y, end.x - start.x),
    )


def _unit_direction(
    poses: list[RecordedPose],
    cumulative: list[float],
    center_s: float,
    sample_distance_m: float,
) -> tuple[float, float]:
    before = _pose_at_s(poses, cumulative, center_s - sample_distance_m)
    after = _pose_at_s(poses, cumulative, center_s + sample_distance_m)
    dx = after.x - before.x
    dy = after.y - before.y
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return 1.0, 0.0
    return dx / length, dy / length


def _point_segment_distance(
    x: float,
    y: float,
    start: RecordedPose,
    end: RecordedPose,
) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    squared_length = dx * dx + dy * dy
    if squared_length <= 1e-12:
        return math.hypot(x - start.x, y - start.y)
    ratio = ((x - start.x) * dx + (y - start.y) * dy) / squared_length
    ratio = min(max(ratio, 0.0), 1.0)
    return math.hypot(x - (start.x + ratio * dx), y - (start.y + ratio * dy))


def _maximum_distance_from_path(
    samples: list[RecordedPose],
    poses: list[RecordedPose],
) -> float:
    return max(
        min(
            _point_segment_distance(sample.x, sample.y, start, end)
            for start, end in zip(poses, poses[1:])
        )
        for sample in samples
    )


def _quintic_transition_samples(
    poses: list[RecordedPose],
    cumulative: list[float],
    center_s: float,
    tangent_distance_m: float,
    radius_m: float,
    step_m: float,
    tangent_sample_m: float,
    outward_overshoot_m: float = 0.0,
) -> list[RecordedPose]:
    """
    Build a tangent- and curvature-continuous corner transition.

    The first three and last three control points are collinear and equally
    spaced.  This makes the second derivative zero at both endpoints, so the
    curve joins a straight approach with zero endpoint curvature instead of
    commanding an instantaneous steering step.  The resulting quintic is a
    practical clothoid-like transition that remains exactly reversible.
    """
    start_s = center_s - tangent_distance_m
    end_s = center_s + tangent_distance_m
    start = _pose_at_s(poses, cumulative, start_s)
    end = _pose_at_s(poses, cumulative, end_s)
    start_direction = _unit_direction(
        poses, cumulative, start_s, tangent_sample_m
    )
    end_direction = _unit_direction(poses, cumulative, end_s, tangent_sample_m)
    direction_angle = abs(
        _normalize_angle(
            math.atan2(end_direction[1], end_direction[0])
            - math.atan2(start_direction[1], start_direction[0])
        )
    )
    cubic_handle_length = (
        (4.0 / 3.0) * radius_m * math.tan(direction_angle / 4.0)
    )
    # Match the endpoint derivative magnitude of the usual cubic circular-arc
    # approximation: 5*h ~= 3*cubic_handle.  P2=2*P1-P0 (and its exit-side
    # equivalent) gives zero endpoint curvature.
    base_handle_length = 0.60 * cubic_handle_length

    def control_points_between(
        segment_start,
        segment_end,
        start_vector,
        end_vector,
        start_handle_m,
        end_handle_m,
    ):
        """Place Bezier controls between the incoming and outgoing straights."""
        return [
            (segment_start.x, segment_start.y),
            (
                segment_start.x + start_handle_m * start_vector[0],
                segment_start.y + start_handle_m * start_vector[1],
            ),
            (
                segment_start.x + 2.0 * start_handle_m * start_vector[0],
                segment_start.y + 2.0 * start_handle_m * start_vector[1],
            ),
            (
                segment_end.x - 2.0 * end_handle_m * end_vector[0],
                segment_end.y - 2.0 * end_handle_m * end_vector[1],
            ),
            (
                segment_end.x - end_handle_m * end_vector[0],
                segment_end.y - end_handle_m * end_vector[1],
            ),
            (segment_end.x, segment_end.y),
        ]

    def sample_control_points(control_points, segment_start, segment_end, length_m):
        """Sample the turn defined by its control points."""
        sample_count = max(2, int(math.ceil(max(length_m, step_m) / step_m)))
        segment_samples = []
        for index in range(sample_count + 1):
            t = index / sample_count
            one_minus_t = 1.0 - t
            weights = (
                one_minus_t**5,
                5.0 * one_minus_t**4 * t,
                10.0 * one_minus_t**3 * t**2,
                10.0 * one_minus_t**2 * t**3,
                5.0 * one_minus_t * t**4,
                t**5,
            )
            segment_samples.append(
                RecordedPose(
                    stamp_sec=(
                        segment_start.stamp_sec
                        + (segment_end.stamp_sec - segment_start.stamp_sec) * t
                    ),
                    frame_id=segment_start.frame_id,
                    x=sum(
                        weight * point[0]
                        for weight, point in zip(weights, control_points)
                    ),
                    y=sum(
                        weight * point[1]
                        for weight, point in zip(weights, control_points)
                    ),
                    yaw=0.0,
                )
            )
        return segment_samples

    if outward_overshoot_m > 0.0 and tangent_distance_m >= 1.20:
        # Continue farther along the approach tangent, but keep the outgoing
        # controls on their straight. This creates the requested broad outside
        # arc without the loop caused by extending both sides past the corner.
        start_handle_length = max(
            base_handle_length,
            0.5 * tangent_distance_m + outward_overshoot_m,
        )
        end_handle_length = min(
            0.5 * tangent_distance_m,
            max(base_handle_length, 0.45 * tangent_distance_m),
        )
    else:
        start_handle_length = base_handle_length
        end_handle_length = base_handle_length
    control_points = control_points_between(
        start,
        end,
        start_direction,
        end_direction,
        start_handle_length,
        end_handle_length,
    )
    samples = sample_control_points(
        control_points,
        start,
        end,
        2.0 * tangent_distance_m,
    )

    for index in range(len(samples) - 1):
        samples[index].yaw = math.atan2(
            samples[index + 1].y - samples[index].y,
            samples[index + 1].x - samples[index].x,
        )
    samples[-1].yaw = samples[-2].yaw
    return samples


def _signed_discrete_curvatures(poses: list[RecordedPose]) -> list[float]:
    """Estimate signed curvature while treating path endpoints as straight."""
    if len(poses) < 3:
        return [0.0] * len(poses)

    curvatures = [0.0]
    for first, middle, last in zip(poses, poses[1:], poses[2:]):
        first_length = math.hypot(middle.x - first.x, middle.y - first.y)
        second_length = math.hypot(last.x - middle.x, last.y - middle.y)
        chord_length = math.hypot(last.x - first.x, last.y - first.y)
        denominator = first_length * second_length * chord_length
        if denominator <= 1e-9:
            curvatures.append(0.0)
            continue
        cross = (
            (middle.x - first.x) * (last.y - middle.y)
            - (middle.y - first.y) * (last.x - middle.x)
        )
        curvatures.append(2.0 * cross / denominator)
    curvatures.append(0.0)
    return curvatures


def _minimum_discrete_radius(poses: list[RecordedPose]) -> float:
    maximum_curvature = max(
        (abs(value) for value in _signed_discrete_curvatures(poses)),
        default=0.0,
    )
    return 0.0 if maximum_curvature <= 1e-9 else 1.0 / maximum_curvature


def _maximum_curvature_rate(poses: list[RecordedPose]) -> float:
    curvatures = _signed_discrete_curvatures(poses)
    maximum_rate = 0.0
    for start, end, start_curvature, end_curvature in zip(
        poses,
        poses[1:],
        curvatures,
        curvatures[1:],
    ):
        distance = math.hypot(end.x - start.x, end.y - start.y)
        if distance <= 1e-6:
            continue
        maximum_rate = max(
            maximum_rate,
            abs(end_curvature - start_curvature) / distance,
        )
    return maximum_rate


def apply_bidirectional_corner_fillets(
    poses: list[RecordedPose],
    target_radius_m: float,
    heading_threshold_rad: float,
    max_deviation_m: float,
    resample_step_m: float,
    merge_gap_m: float = 0.30,
    minimum_radius_m: float = 0.0,
    sample_is_safe=None,
    outward_overshoot_m: float = 0.0,
) -> CornerFilletSummary:
    """
    Round major corners equally in forward and reverse path order.

    A short or constrained corner reduces its applied radius. It never rejects
    the route, because this function only improves replay geometry.
    """
    source = list(poses)
    if len(source) < 3 or target_radius_m <= 0.0:
        return CornerFilletSummary(source, 0, 0, 0.0)

    cumulative = _cumulative_lengths(source)
    if cumulative[-1] <= 1e-6:
        return CornerFilletSummary(source, 0, 0, 0.0)

    corners = _detect_corners(
        source,
        cumulative,
        max(heading_threshold_rad, math.radians(1.0)),
        max(merge_gap_m, resample_step_m),
    )
    replacements = []
    for index, corner in enumerate(corners):
        turn_angle = min(abs(corner.heading_change), math.radians(150.0))
        tangent_factor = math.tan(turn_angle / 2.0)
        if tangent_factor <= 1e-6:
            continue

        previous_center = corners[index - 1].center_s if index > 0 else 0.0
        next_center = (
            corners[index + 1].center_s
            if index + 1 < len(corners)
            else cumulative[-1]
        )
        distance_before = corner.center_s - previous_center
        distance_after = next_center - corner.center_s
        # Adjacent corners share at most 96% of their connecting straight.  A
        # route endpoint does not have another fillet competing for that space,
        # so use most of the available approach instead of the old 48% cap.
        available_before = distance_before * (0.85 if index == 0 else 0.48)
        available_after = distance_after * (
            0.85 if index + 1 == len(corners) else 0.48
        )
        available = min(available_before, available_after)
        minimum_tangent_distance = max(2.0 * resample_step_m, 0.10)
        if available < minimum_tangent_distance:
            continue

        # Recorded routes are not ideal polylines: a tangent point can land on
        # a slightly distorted portion of the saved path.  Consequently, one
        # formula-derived tangent length can produce a much tighter curve than
        # requested. Search the usable straight length and choose from the
        # measured curve geometry instead.  This is performed only once when a
        # replay starts, so the extra samples do not affect controller runtime.
        preferred_tangent_distance = min(
            1.40 * target_radius_m * tangent_factor,
            available,
        )
        search_start = minimum_tangent_distance
        candidate_distances = {
            search_start,
            preferred_tangent_distance,
            available,
        }
        search_intervals = 32
        for search_index in range(search_intervals + 1):
            ratio = search_index / search_intervals
            candidate_distances.add(
                search_start
                + (available - search_start) * ratio
            )

        feasible_candidates = []
        for tangent_distance in sorted(candidate_distances):
            radius = tangent_distance / tangent_factor
            tangent_sample = max(
                resample_step_m, min(0.20, tangent_distance * 0.20)
            )
            requested_overshoot_m = max(0.0, outward_overshoot_m)
            overshoot_trials = [
                requested_overshoot_m * factor
                for factor in (1.0, 0.75, 0.50, 0.25)
            ]
            overshoot_trials.append(0.0)
            for candidate_overshoot_m in dict.fromkeys(overshoot_trials):
                if candidate_overshoot_m > 0.0 and tangent_distance < 1.20:
                    continue
                samples = _quintic_transition_samples(
                    source,
                    cumulative,
                    corner.center_s,
                    tangent_distance,
                    radius,
                    max(resample_step_m, 0.02),
                    tangent_sample,
                    outward_overshoot_m=candidate_overshoot_m,
                )
                deviation_m = _maximum_distance_from_path(samples, source)
                if max_deviation_m > 0.0 and deviation_m > max_deviation_m:
                    continue
                if sample_is_safe is not None and not all(
                    sample_is_safe(sample) for sample in samples
                ):
                    continue
                feasible_candidates.append(
                    (
                        _minimum_discrete_radius(samples),
                        _maximum_curvature_rate(samples),
                        deviation_m,
                        tangent_distance,
                        samples,
                        candidate_overshoot_m,
                    )
                )
                # For this transition length, retain the largest map-safe
                # outside arc and avoid needless lower-overshoot evaluations.
                break

        if not feasible_candidates:
            # The map-cleaned source path remains usable. Do not introduce a
            # new curve through occupied clearance merely to satisfy smoothing.
            continue

        radius_safe_candidates = [
            candidate
            for candidate in feasible_candidates
            if candidate[0] >= max(0.0, minimum_radius_m)
        ]
        selection_pool = radius_safe_candidates or feasible_candidates
        largest_feasible_overshoot_m = max(
            candidate[5] for candidate in selection_pool
        )
        overshoot_candidates = [
            candidate
            for candidate in selection_pool
            if candidate[5] >= largest_feasible_overshoot_m - 1e-9
        ]
        preferred_candidates = [
            candidate
            for candidate in overshoot_candidates
            if candidate[3] >= preferred_tangent_distance - 1e-9
        ]
        target_candidates = [
            candidate
            for candidate in preferred_candidates
            if candidate[0] >= target_radius_m
        ]
        if target_candidates:
            # Use the candidate nearest the requested radius. A numerically
            # almost-straight candidate is not inherently a better corner and
            # can consume excessive path length without improving tracking.
            selected = min(
                target_candidates,
                key=lambda candidate: (
                    candidate[0] - target_radius_m,
                    candidate[1],
                    candidate[2],
                    candidate[3],
                ),
            )
        elif preferred_candidates:
            # A short corridor segment may make the target radius impossible.
            # Keep the route traversable and use the best smooth radius that
            # fits instead of restoring an instantaneous heading change.
            selected = max(
                preferred_candidates,
                key=lambda candidate: (
                    candidate[0],
                    -candidate[1],
                    -candidate[2],
                ),
            )
        else:
            # Clearance or deviation may require a transition shorter than the
            # nominal starting length. Use the longest feasible candidate; this
            # avoids treating a three-sample curve as a falsely large radius.
            selected = max(
                overshoot_candidates,
                key=lambda candidate: (
                    candidate[3],
                    candidate[0],
                    -candidate[1],
                ),
            )
        applied_radius_m, _, _, tangent_distance, samples, _ = selected

        replacements.append(
            _Replacement(
                start_s=corner.center_s - tangent_distance,
                end_s=corner.center_s + tangent_distance,
                samples=samples,
                applied_radius_m=applied_radius_m,
            )
        )

    if not replacements:
        return CornerFilletSummary(source, len(corners), 0, 0.0)

    output = []
    source_index = 0
    boundary_tolerance = 1e-6
    for replacement in replacements:
        while (
            source_index < len(source)
            and cumulative[source_index]
            < replacement.start_s - boundary_tolerance
        ):
            output.append(source[source_index])
            source_index += 1
        if output and math.hypot(
            output[-1].x - replacement.samples[0].x,
            output[-1].y - replacement.samples[0].y,
        ) <= 1e-6:
            output.pop()
        output.extend(replacement.samples)
        while (
            source_index < len(source)
            and cumulative[source_index]
            <= replacement.end_s + boundary_tolerance
        ):
            source_index += 1
    output.extend(source[source_index:])

    minimum_applied_radius = min(
        replacement.applied_radius_m for replacement in replacements
    )
    return CornerFilletSummary(
        poses=output,
        detected_corners=len(corners),
        modified_corners=len(replacements),
        minimum_applied_radius_m=minimum_applied_radius,
        degraded_corners=sum(
            replacement.applied_radius_m < max(0.0, target_radius_m)
            for replacement in replacements
        ),
        maximum_curvature_rate_per_m2=max(
            _maximum_curvature_rate(replacement.samples)
            for replacement in replacements
        ),
        applied_radii_m=tuple(
            replacement.applied_radius_m for replacement in replacements
        ),
    )
