"""Tests for the ROS-free path transport, e-stop range check and CSV log round trip."""

import math

import numpy as np
import pytest

from robot_conga import CirclePath, CongaSimulator, SimulatorConfig, compute_conga_metrics
from robot_conga.transport import (
    POSE_STAMPED_BYTES,
    LeaderPathRecorder,
    SampledPath,
    csv_header,
    csv_row,
    front_min_range,
    load_log_csv,
)


def _drive_circle(rec, radius=1.5, laps=3.0, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    for phi in np.arange(0.0, 2 * math.pi * laps, 0.005):
        x = radius * math.sin(phi) + rng.normal(0, noise)
        y = radius * (1 - math.cos(phi)) + rng.normal(0, noise)
        rec.add(x, y, phi)
    return x, y


def test_recorder_window_and_message_size():
    rec = LeaderPathRecorder(min_dist=0.05, window=4.0, max_points=400)
    _drive_circle(rec, laps=3.0)  # ~28 m driven
    s = [p[0] for p in rec.points]
    assert s[-1] > 25.0
    assert s[-1] - s[0] <= 4.0 + 0.06
    assert len(rec.points) * POSE_STAMPED_BYTES < 65000 / 4  # well under the DDS cap


def test_recorder_respects_hard_cap():
    rec = LeaderPathRecorder(min_dist=0.05, window=1000.0, max_points=50)
    _drive_circle(rec, laps=1.0)
    assert len(rec.points) == 50


def test_startup_prefix_puts_s0_at_last_robot():
    rec = LeaderPathRecorder(min_dist=0.05, prefix_length=2.0)
    rec.add(1.0, 1.0, math.pi / 2)  # leader at (1, 1) facing +y
    s0, x0, y0 = rec.points[0]
    assert (s0, x0, y0) == pytest.approx((0.0, 1.0, -1.0))
    assert rec.leader_s(1.0, 1.0) == pytest.approx(2.0)


def test_sampled_path_matches_circle_and_shares_leader_s():
    rec = LeaderPathRecorder(min_dist=0.05, window=4.0)
    x, y = _drive_circle(rec, radius=1.5, laps=1.2, noise=0.003)
    s, xs, ys = zip(*rec.points)
    path = SampledPath(s, xs, ys, smoothing_std=0.005)
    s_l = rec.leader_s(x, y)
    # Leader's own s maps back to (near) its position on the followers' representation
    px, py = path.point(min(s_l, path.s_end))
    assert math.hypot(px - x, py - y) < 0.06
    # Points follower-distance behind are on the circle, with the right curvature
    for back in (1.0, 2.0, 3.0):
        ref = path.reference(s_l - back, 0.1)
        r = math.hypot(ref.x - 0.0, ref.y - 1.5)
        assert r == pytest.approx(1.5, abs=0.01)
        assert ref.curvature == pytest.approx(1 / 1.5, rel=0.15)
        assert ref.twist.angular == pytest.approx(0.1 * ref.curvature)
    assert path.reference(path.s0 - 0.1, 0.1) is None  # outside the window


def test_front_min_range_uses_front_sector_only():
    # 360 beams, 1 deg each, starting at -180 deg; obstacle at +90 deg (side) and -90 deg
    ranges = [5.0] * 360
    ranges[270] = 0.2  # +90 deg
    ranges[90] = 0.4   # -90 deg
    ranges[91] = float("nan")
    a0, inc = -math.pi, math.radians(1.0)
    # TurtleBot 4: RPLIDAR yawed +90 deg, so robot-front is -90 deg in the scan frame
    assert front_min_range(ranges, a0, inc, math.radians(-90), math.radians(30)) == pytest.approx(0.4)
    assert front_min_range(ranges, a0, inc, 0.0, math.radians(30)) == pytest.approx(5.0)
    assert front_min_range([0.0, 0.01], 0.0, 0.1, 0.0, 1.0, range_min=0.15) == math.inf


def test_csv_round_trip_gives_same_metrics(tmp_path):
    sim = CongaSimulator(CirclePath(radius=1.5), num_robots=3, spacing=1.0, config=SimulatorConfig(seed=0))
    result = sim.run(10.0)
    path = tmp_path / "log.csv"
    with open(path, "w", newline="") as f:
        import csv
        w = csv.writer(f)
        w.writerow(csv_header(3))
        for step in result.steps:
            w.writerow(csv_row(step.time, step.true_poses, [r.pose for r in step.reference_states],
                               [r.s for r in step.reference_states], step.command_twists))
    loaded = load_log_csv(str(path), desired_spacing=1.0)
    a, b = compute_conga_metrics(result), compute_conga_metrics(loaded)
    assert [m.position_rmse for m in a.robot_metrics] == pytest.approx([m.position_rmse for m in b.robot_metrics])
    assert a.spacing_rmse == pytest.approx(b.spacing_rmse)
