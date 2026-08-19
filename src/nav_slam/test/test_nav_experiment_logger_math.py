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

import math
from collections import deque

import pytest

from nav_slam.nav_experiment_logger import _interpolate_pose, _relative_pose


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
