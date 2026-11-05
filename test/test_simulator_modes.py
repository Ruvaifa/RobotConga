"""Tests for the startup prefix, the time-delay baseline and the safety gate."""

import math

import pytest

from robot_conga import (
    CongaSimulator,
    PrefixedPath,
    SafetyGate,
    SCurvePath,
    SimulatorConfig,
    StraightPath,
    compute_conga_metrics,
)


def test_prefixed_path_geometry():
    base = SCurvePath(x0=1.0, y0=2.0, theta0=math.pi / 2)
    path = PrefixedPath(base, 2.0)
    assert path.point(0.0) == pytest.approx((1.0, 0.0))  # 2 m behind start along -heading
    assert path.point(1.0) == pytest.approx((1.0, 1.0))
    assert path.point(2.0) == pytest.approx(base.point(0.0))
    assert path.point(3.5) == pytest.approx(base.point(1.5))
    assert path.heading(0.5) == pytest.approx(math.pi / 2)
    assert path.curvature(0.5) == 0.0


def test_startup_prefix_places_all_robots_on_path():
    base = SCurvePath()
    sim = CongaSimulator(base, num_robots=3, spacing=1.0, startup_prefix=True,
                         config=SimulatorConfig(seed=0))
    refs = sim.propagator.propagate(sim.leader_s)
    assert sim.leader_s == pytest.approx(2.0)
    assert [r.s for r in refs] == pytest.approx([2.0, 1.0, 0.0])
    assert (refs[0].x, refs[0].y) == pytest.approx(base.point(0.0))  # leader at the original start
    assert (refs[2].x, refs[2].y) == pytest.approx((-2.0, 0.0))  # last robot 2 d behind it
    metrics = compute_conga_metrics(sim.run(40.0))
    assert max(m.position_rmse for m in metrics.robot_metrics) < 0.06


def _leader_stops(sim, t):
    sim.v_cmd = 0.0 if 20.0 <= t < 40.0 else 0.1


def _min_spacing(following):
    sim = CongaSimulator(StraightPath(), num_robots=3, spacing=1.0, v_cmd=0.1,
                         following=following, config=SimulatorConfig(seed=0))
    result = sim.run(60.0, on_step=_leader_stops)
    return min(min(d) for d in result.get_inter_robot_distances().values())


def test_conga_keeps_spacing_when_leader_stops():
    assert _min_spacing("conga") > 0.85


def test_time_delay_baseline_collapses_spacing_when_leader_stops():
    # 20 s stop > tau_1 = 10 s: follower 1 drives up to where the leader stopped
    assert _min_spacing("time_delay") < 0.2


def test_time_delay_matches_conga_at_constant_speed():
    refs = {}
    for mode in ("conga", "time_delay"):
        sim = CongaSimulator(StraightPath(), num_robots=3, v_cmd=0.1, following=mode,
                             config=SimulatorConfig(seed=0))
        refs[mode] = [r.s for r in sim.run(5.0).steps[-1].reference_states]
    assert refs["conga"] == pytest.approx(refs["time_delay"], abs=1e-6)


def test_unknown_following_mode_rejected():
    with pytest.raises(ValueError):
        CongaSimulator(StraightPath(), following="psychic")


def test_gate_ratio():
    gate = SafetyGate(slow_error=0.1, stop_error=0.3, min_ratio=0.0)
    assert gate.ratio(0.05) == 1.0
    assert gate.ratio(0.2) == pytest.approx(0.5)
    assert gate.ratio(0.5) == 0.0


def _stalled_run(gate):
    def stall(sim, t):
        sim.robots[1].stalled = 20.0 <= t < 35.0
    sim = CongaSimulator(StraightPath(), num_robots=3, spacing=1.0, v_cmd=0.1, gate=gate,
                         config=SimulatorConfig(seed=0))
    return sim.run(60.0, on_step=stall)


def test_gate_prevents_pileup_behind_stalled_robot():
    off = _stalled_run(None)
    on = _stalled_run(SafetyGate())
    assert min(off.get_inter_robot_distances()[(1, 2)]) < 0.2
    assert min(on.get_inter_robot_distances()[(1, 2)]) > 0.5
    assert min(step.speed_ratio for step in on.steps) < 0.2  # the gate actually engaged
    assert on.steps[-1].speed_ratio == 1.0  # and released after the stall
