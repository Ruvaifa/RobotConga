"""Unit tests for BSplinePath and PathManager."""

import math
import pytest

from robot_conga.bspline_path import BSplinePath, PathManager
from robot_conga.models import Pose


def test_bspline_path_basic():
    """Test BSplinePath evaluation with 4 straight waypoints."""
    waypoints = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    path = BSplinePath(waypoints)

    assert path.total_length > 2.8
    # Point at origin
    x0, y0 = path.point(0.0)
    assert x0 == pytest.approx(0.0, abs=1e-3)
    assert y0 == pytest.approx(0.0, abs=1e-3)

    # Tangent heading along +x should be approx 0
    th = path.heading(1.0)
    assert th == pytest.approx(0.0, abs=1e-2)

    # Curvature of straight line should be approx 0
    kappa = path.curvature(1.0)
    assert kappa == pytest.approx(0.0, abs=1e-2)


def test_bspline_path_steep_ascent_frame_rotation():
    """Test steep vertical segment triggers 90-degree frame rotation (Paper Section IV-C)."""
    # Vertical line along y axis: dx = 0, dy > 0
    waypoints = [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (0.0, 3.0)]
    path = BSplinePath(waypoints)

    # Frame rotation should be pi/2
    assert path.frame_angle == pytest.approx(math.pi / 2.0)

    # Evaluated points should accurately track along y axis
    x_mid, y_mid = path.point(1.5)
    assert x_mid == pytest.approx(0.0, abs=1e-2)
    assert y_mid == pytest.approx(1.5, abs=0.1)


def test_bspline_path_insufficient_waypoints():
    """Verify ValueError is raised if fewer than 4 waypoints are given."""
    with pytest.raises(ValueError, match="At least 4 waypoints required"):
        BSplinePath([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)])


def test_path_manager_lifecycle():
    """Test PathManager updating and reference state generation."""
    pm = PathManager()
    assert pm.total_length == 0.0
    assert pm.get_reference_state(1.0, 0.1) is None

    # Update with valid waypoints
    pts = [(0.0, 0.0), (1.0, 0.5), (2.0, 1.0), (3.0, 1.5)]
    assert pm.update_path(pts) is True
    assert pm.total_length > 0.0

    # Get reference state at s = 1.0 m, speed 0.1 m/s
    ref = pm.get_reference_state(1.0, 0.1)
    assert ref is not None
    assert ref.s == pytest.approx(1.0)
    assert ref.twist.linear == pytest.approx(0.1)
    assert math.isfinite(ref.pose.x)
    assert math.isfinite(ref.pose.y)
    assert math.isfinite(ref.pose.theta)
    assert math.isfinite(ref.twist.angular)

    # Negative s should return None
    assert pm.get_reference_state(-0.5, 0.1) is None

