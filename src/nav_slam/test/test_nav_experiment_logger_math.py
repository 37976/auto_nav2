# Copyright 2026 xu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Test timestamp synchronization and SE(2) metric helpers."""

import csv
import math
from collections import deque

import pytest

from nav_slam.map_odom_corrector import (
    _correction_innovation,
    _tracking_innovation_allowed,
)
from nav_slam.nav_experiment_goal_runner import parse_goals
from nav_slam.nav_experiment_logger import (
    NavExperimentLogger,
    _composite_pose_error,
    _interpolate_pose,
    _interpolate_sample_pose,
    _joint_accuracy_satisfied,
    _pose_difference,
    _relative_pose,
    _sample_standard_deviation,
)


def test_interpolate_pose_uses_estimate_timestamp():
    """Truth poses are interpolated at the estimate timestamp."""
    history = deque([
        (1.0, 0.0, 0.0, 0.0),
        (1.2, 2.0, 4.0, math.radians(20.0)),
    ])

    pose = _interpolate_pose(history, 1.1, 0.11)

    assert pose is not None
    assert pose[0] == pytest.approx(1.0)
    assert pose[1] == pytest.approx(2.0)
    assert math.degrees(pose[2]) == pytest.approx(10.0)
    assert pose[3] == pytest.approx(0.2)


def test_interpolate_pose_rejects_unsynchronized_truth():
    """Truth outside the synchronization tolerance is rejected."""
    history = deque([(1.0, 0.0, 0.0, 0.0)])

    assert _interpolate_pose(history, 1.2, 0.1) is None


def test_relative_pose_is_expressed_in_first_pose_frame():
    """Relative translation is represented in the first pose frame."""
    first = {"gt_x_m": 1.0, "gt_y_m": 2.0, "gt_yaw": math.pi / 2.0}
    second = {"gt_x_m": 1.0, "gt_y_m": 3.0, "gt_yaw": math.pi / 2.0}

    dx, dy, dyaw = _relative_pose(first, second, "gt")

    assert dx == pytest.approx(1.0)
    assert dy == pytest.approx(0.0)
    assert dyaw == pytest.approx(0.0)


def test_rpe_pose_is_interpolated_at_exact_interval():
    """Irregular trajectory samples still evaluate the exact target time."""
    samples = [
        {"time_s": 0.0, "gt_x_m": 0.0, "gt_y_m": 0.0, "gt_yaw": 0.0},
        {"time_s": 0.8, "gt_x_m": 0.8, "gt_y_m": 0.0, "gt_yaw": 0.0},
        {"time_s": 1.2, "gt_x_m": 1.2, "gt_y_m": 0.0, "gt_yaw": 0.0},
    ]

    pose = _interpolate_sample_pose(samples, 0, 1.0, "gt")

    assert pose is not None
    assert pose["gt_x_m"] == pytest.approx(1.0)


def test_composite_pose_error_has_equal_normalized_weights():
    """Both errors at their allowed limits produce a combined error of one."""
    assert _composite_pose_error(0.5, 5.0, 0.5, 5.0) == pytest.approx(1.0)
    assert _composite_pose_error(0.5, 0.0, 0.5, 5.0) == pytest.approx(
        math.sqrt(0.5)
    )


def test_joint_accuracy_uses_absolute_yaw_error():
    """A large negative wrapped yaw error must not pass the joint gate."""
    assert _joint_accuracy_satisfied(0.5, -5.0, 0.5, 5.0)
    assert not _joint_accuracy_satisfied(0.1, -6.0, 0.5, 5.0)
    assert not _joint_accuracy_satisfied(0.6, 0.0, 0.5, 5.0)


def test_pose_difference_wraps_yaw_for_initial_stability_gate():
    """The post-localization gate handles yaw across the +/-180-degree seam."""
    translation, yaw = _pose_difference(
        (0.0, 0.0, math.radians(179.0)),
        (0.3, 0.4, math.radians(-179.0)),
    )

    assert translation == pytest.approx(0.5)
    assert yaw == pytest.approx(2.0)


def test_repeated_run_standard_deviation_is_sample_std():
    """Group summaries use n-1 sample standard deviation."""
    assert _sample_standard_deviation([1.0]) is None
    assert _sample_standard_deviation([1.0, 3.0]) == pytest.approx(
        math.sqrt(2.0)
    )


def test_correction_innovation_wraps_yaw_and_uses_translation_norm():
    """Accepted-correction maxima use the target relative to current TF."""
    translation, yaw = _correction_innovation(
        (1.0, 2.0, math.radians(179.0)),
        (1.3, 2.4, math.radians(-179.0)),
    )

    assert translation == pytest.approx(0.5)
    assert math.degrees(yaw) == pytest.approx(2.0)


def test_tracking_innovation_gate_rejects_recorded_symmetric_map_alias():
    """Both mutually consistent false observations from the run are rejected."""
    reference = (3.916126, -1.389290, math.radians(-159.486645))
    false_candidates = [
        (9.926, 7.536, math.radians(20.961)),
        (10.087068, 7.584735, math.radians(20.936848)),
    ]

    for false_candidate in false_candidates:
        assert not _tracking_innovation_allowed(
            reference, false_candidate, 0.50, math.radians(5.0)
        )


def test_tracking_innovation_gate_keeps_normal_recorded_correction():
    """Observed normal gated corrections remain inside the new safety bound."""
    reference = (3.916126, -1.389290, math.radians(-159.486645))
    normal_candidate = (4.113649, -1.376752, math.radians(-159.132553))

    assert _tracking_innovation_allowed(
        reference, normal_candidate, 0.50, math.radians(5.0)
    )


def test_zero_tracking_innovation_limits_disable_gate():
    """Other launch modes retain their previous behavior when limits are zero."""
    assert _tracking_innovation_allowed(
        (0.0, 0.0, 0.0), (10.0, 10.0, math.pi), 0.0, 0.0
    )


def test_goal_sequence_parser_uses_degrees():
    """Experiment goal strings are deterministic and human-readable."""
    goals = parse_goals("1.0,2.0,90; -1.0,0.5,-45")

    assert len(goals) == 2
    assert goals[0].x == pytest.approx(1.0)
    assert goals[0].y == pytest.approx(2.0)
    assert goals[0].yaw_rad == pytest.approx(math.pi / 2.0)
    assert goals[1].yaw_rad == pytest.approx(-math.pi / 4.0)


def test_goal_sequence_parser_rejects_incomplete_goal():
    """Every target must specify x, y, and yaw in degrees."""
    with pytest.raises(ValueError):
        parse_goals("1.0,2.0")


def test_group_summary_writes_mean_and_sample_std(tmp_path):
    """Paper table aggregation contains both mean and standard deviation."""
    summaries = [
        {
            "experiment_group": "gated",
            "paper_statistics_eligible": True,
            "initial_estimate_stabilized": True,
            "sample_count": 10,
            "ate_rmse_m": 1.0,
        },
        {
            "experiment_group": "gated",
            "paper_statistics_eligible": True,
            "initial_estimate_stabilized": True,
            "sample_count": 12,
            "ate_rmse_m": 3.0,
        },
        {
            "experiment_group": "gated",
            "paper_statistics_eligible": False,
            "initial_estimate_stabilized": True,
            "sample_count": 2,
            "ate_rmse_m": 99.0,
        },
    ]

    NavExperimentLogger._write_group_summary(str(tmp_path), summaries)

    with open(
        tmp_path / "group_summary.csv", newline="", encoding="utf-8"
    ) as file:
        row = next(csv.DictReader(file))
    assert float(row["ate_rmse_m_mean"]) == pytest.approx(2.0)
    assert float(row["ate_rmse_m_std"]) == pytest.approx(math.sqrt(2.0))
    assert int(row["run_count_total"]) == 3
    assert int(row["run_count_eligible"]) == 2


def test_group_summary_excludes_legacy_unstabilized_alignment(tmp_path):
    """Old runs without the post-global stability gate cannot enter paper stats."""
    summaries = [{
        "experiment_group": "gated",
        "paper_statistics_eligible": True,
        "sample_count": 10,
        "ate_rmse_m": 99.0,
    }]

    NavExperimentLogger._write_group_summary(str(tmp_path), summaries)

    with open(
        tmp_path / "group_summary.csv", newline="", encoding="utf-8"
    ) as file:
        row = next(csv.DictReader(file))
    assert int(row["run_count_eligible"]) == 0
    assert row["ate_rmse_m_mean"] == ""
