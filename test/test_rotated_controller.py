"""Tests for the frame-rotated Eq. (8) controller (any path heading, full circles)."""

import math
import statistics

import numpy as np
import pytest

from robot_conga import (
    CirclePath,
    CongaController,
    CongaSimulator,
    ControllerSingularityError,
    Pose,
    ReferenceState,
    RotatedCongaController,
    SimulatorConfig,
    StraightPath,
    Twist,
    compute_conga_metrics,
    singularity_fallback,
)


@pytest.mark.parametrize("heading_deg", [0, 60, 90, 120, 180, -90, -150])
def test_straight_path_any_heading_converges(heading_deg):
    """20 cm lateral offset converges below 5 cm within 40 s, default noise and delay."""
    th = math.radians(heading_deg)
    start = Pose(x=-0.2 * math.sin(th), y=0.2 * math.cos(th), theta=th)  # 20 cm to the left
    sim = CongaSimulator(
        path=StraightPath(0.0, 0.0, th),
        num_robots=1,
        v_cmd=0.1,
        initial_leader_s=0.0,
        initial_poses=[start],
        config=SimulatorConfig(seed=0),
    )
    errors = sim.run(40.0).get_position_errors(0)
    # Mean true error over the last 5 s; noise alone keeps it at ~1-4 cm.
    assert statistics.mean(errors[-500:]) < 0.05


def test_plain_controller_fails_where_rotated_succeeds():
    """Documents finding 1: plain Eq. (8) is singular on a path heading -90 deg."""
    th = -math.pi / 2
    sim = CongaSimulator(
        path=StraightPath(0.0, 0.0, th),
        num_robots=1,
        initial_leader_s=0.0,
        initial_poses=[Pose(0.2, 0.0, th)],
        controllers=[CongaController()],
        config=SimulatorConfig(seed=0),
    )
    with pytest.raises(ControllerSingularityError):
        sim.run(40.0)


def test_full_circle_three_robots():
    """3 robots complete a full circle (R=1.5, v=0.1) with small tracking error."""
    sim = CongaSimulator(
        path=CirclePath(0.0, 0.0, 1.5, start_angle=-math.pi / 2),
        num_robots=3,
        spacing=1.0,
        v_cmd=0.1,
        config=SimulatorConfig(seed=0),
    )
    result = sim.run(120.0)  # leader covers 12 m > one 9.42 m lap; last follower > 1 lap too
    assert result.steps[-1].reference_states[-1].s > 2 * math.pi * 1.5
    metrics = compute_conga_metrics(result, start_time=10.0)
    assert max(m.position_rmse for m in metrics.robot_metrics) < 0.05
    assert max(m.max_position_error for m in metrics.robot_metrics) < 0.15


def test_frame_does_not_chatter_near_45_deg():
    """theta* jittering around 45 deg (the frame boundary) switches the frame at most once."""
    controller = RotatedCongaController()
    rng = np.random.default_rng(0)
    alphas = []
    for _ in range(2000):
        theta_star = math.radians(45.0 + rng.normal(0.0, 3.0))
        controller.update_frame(theta_star)
        alphas.append(controller.alpha)
    switches = sum(1 for a, b in zip(alphas, alphas[1:]) if a != b)
    assert switches <= 1


def test_frame_switches_after_hysteresis():
    controller = RotatedCongaController(hysteresis=math.radians(10.0))
    assert controller.update_frame(math.radians(40.0)) == pytest.approx(0.0)
    assert controller.update_frame(math.radians(54.0)) == pytest.approx(0.0)  # inside band
    assert controller.update_frame(math.radians(56.0)) == pytest.approx(math.pi / 2)
    assert controller.update_frame(math.radians(36.0)) == pytest.approx(math.pi / 2)  # inside band
    assert controller.update_frame(math.radians(34.0)) == pytest.approx(0.0)


def test_rotated_matches_plain_near_x_axis():
    """With alpha = 0 the rotated controller is exactly Eq. (8)."""
    ref = ReferenceState(pose=Pose(0.0, 0.0, 0.2), twist=Twist(0.1, 0.02), s=1.0, curvature=0.2)
    actual = Pose(0.03, -0.05, 0.1)
    a = CongaController().compute(actual, ref)
    b = RotatedCongaController().compute(actual, ref)
    assert (a.linear, a.angular) == pytest.approx((b.linear, b.angular))


def test_singularity_fallback_turns_toward_reference():
    ref = ReferenceState(pose=Pose(0.0, 0.0, 0.0), twist=Twist(0.1, 0.0), s=0.0, curvature=0.0)
    cmd = singularity_fallback(Pose(0.0, 0.0, math.pi / 2), ref)
    assert cmd.linear == 0.0
    assert cmd.angular < 0.0  # heading too far left -> turn right
