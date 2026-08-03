import math

import numpy as np
import pytest

from control_actuator.inverseKinematics import StewartPlatformIK
from control_actuator.inverseKinematics_2 import StewartPlatformIK as LegacyIK


def test_neutral_pose_produces_six_nearly_equal_extensions():
    ik = StewartPlatformIK()

    extensions = ik.solve([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

    assert extensions.shape == (6,)
    assert np.ptp(extensions) < 1e-5
    assert extensions == pytest.approx(np.full(6, 0.000325), abs=1e-5)


def test_positive_vertical_motion_increases_all_extensions():
    ik = StewartPlatformIK()
    neutral = ik.solve([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

    raised = ik.solve([0.0, 0.0, 0.020], [0.0, 0.0, 0.0, 1.0])

    assert np.all(raised > neutral)
    assert raised - neutral == pytest.approx(np.full(6, 0.019745), abs=1e-5)


def test_nearly_unit_quaternion_is_normalized_before_use():
    ik = StewartPlatformIK()
    half_angle = math.pi / 8.0
    quaternion = [math.sin(half_angle), 0.0, 0.0, math.cos(half_angle)]

    normalized = ik.solve([0.0, 0.0, 0.05], quaternion)
    slightly_scaled = ik.solve(
        [0.0, 0.0, 0.05], np.multiply(quaternion, 1.0005)
    )

    assert slightly_scaled == pytest.approx(normalized)


def test_zero_quaternion_is_rejected():
    ik = StewartPlatformIK()

    with pytest.raises(ValueError, match='nonzero'):
        ik.solve([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])


def test_non_unit_quaternion_is_rejected():
    ik = StewartPlatformIK()

    with pytest.raises(ValueError, match='unit magnitude'):
        ik.solve([0.0, 0.0, 0.0], [0.0, 0.0, 0.01, 0.0])


def test_legacy_import_uses_corrected_solver():
    assert LegacyIK is StewartPlatformIK
