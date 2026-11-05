"""ROS-free pieces of the leader -> follower path transport, e-stop and CSV logging.

Path transport: the leader records world-frame waypoints every `min_dist` metres and tags
each with its arc length s (published in PoseStamped.position.z, unused for planar robots).
Followers fit smoothing splines x(s), y(s) over those same waypoints, so the leader's s_L
and the followers' s_i live on one shared parameterisation, and nothing drifts between a
chord sum on one side and a spline length on the other. Only a bounded recent window is sent.
"""

import csv
import math
from collections import deque
from typing import List, Optional, Sequence, Tuple

import numpy as np

from robot_conga.geometry import wrap_to_pi
from robot_conga.models import Pose, ReferenceState, Twist
from robot_conga.path import Path

# Serialized nav_msgs/Path cost per PoseStamped: header (8 B stamp + ~12 B frame_id) + 7 doubles.
POSE_STAMPED_BYTES = 80


class LeaderPathRecorder:
    """Leader-side waypoint list with cumulative arc length, trimmed to a recent window."""

    def __init__(
        self,
        min_dist: float = 0.05,
        window: float = 4.0,
        max_points: int = 400,
        prefix_length: float = 0.0,
    ) -> None:
        """
        Args:
            min_dist: Minimum spacing between recorded waypoints (m).
            window: Keep waypoints with s >= s_last - window (m). Must exceed (N-1)*d.
            max_points: Hard cap on waypoints (400 * 80 B = 32 KB, half the 65 KB DDS limit).
            prefix_length: Straight lead-in recorded behind the first pose, (N-1)*d, so the
                followers lined up behind the leader start on the path (s = 0 at the last one).
        """
        self.min_dist = float(min_dist)
        self.window = float(window)
        self.prefix_length = float(prefix_length)
        self.points: deque = deque(maxlen=int(max_points))  # (s, x, y)

    def add(self, x: float, y: float, theta: float) -> bool:
        """Record a leader pose; returns True if a new waypoint was appended."""
        if not self.points:
            n = int(round(self.prefix_length / self.min_dist))
            for k in range(n):
                back = self.prefix_length - k * self.prefix_length / n
                self.points.append((self.prefix_length - back, x - back * math.cos(theta), y - back * math.sin(theta)))
            self.points.append((self.prefix_length, x, y))
            return True
        s_last, x_last, y_last = self.points[-1]
        step = math.hypot(x - x_last, y - y_last)
        if step < self.min_dist:
            return False
        self.points.append((s_last + step, x, y))
        while self.points[0][0] < self.points[-1][0] - self.window:
            self.points.popleft()
        return True

    def leader_s(self, x: float, y: float) -> float:
        """Leader arc length: last waypoint's s plus the (< min_dist) distance beyond it."""
        s_last, x_last, y_last = self.points[-1]
        return s_last + math.hypot(x - x_last, y - y_last)


class SampledPath(Path):
    """Path from leader waypoints (s, x, y) via smoothing splines x(s), y(s).

    s is absolute (the leader's arc length), so the window may start at s0 > 0.
    Queries outside [s0, s_end] are clamped; callers should check `s0` first.
    """

    def __init__(self, s: Sequence[float], x: Sequence[float], y: Sequence[float], smoothing_std: float = 0.01) -> None:
        from scipy.interpolate import UnivariateSpline

        if len(s) < 4:
            raise ValueError(f"At least 4 waypoints required, got {len(s)}")
        s_arr = np.asarray(s, dtype=float)
        smooth = len(s_arr) * smoothing_std ** 2  # residual budget: ~smoothing_std per point
        self._fx = UnivariateSpline(s_arr, np.asarray(x, dtype=float), k=3, s=smooth)
        self._fy = UnivariateSpline(s_arr, np.asarray(y, dtype=float), k=3, s=smooth)
        self.s0 = float(s_arr[0])
        self.s_end = float(s_arr[-1])

    def _clamp(self, s: float) -> float:
        self._validate_s(s)
        return min(max(s, self.s0), self.s_end)

    def point(self, s: float) -> Tuple[float, float]:
        s = self._clamp(s)
        return float(self._fx(s)), float(self._fy(s))

    def heading(self, s: float) -> float:
        s = self._clamp(s)
        return wrap_to_pi(math.atan2(float(self._fy(s, 1)), float(self._fx(s, 1))))

    def curvature(self, s: float) -> float:
        s = self._clamp(s)
        dx, dy = float(self._fx(s, 1)), float(self._fy(s, 1))
        ddx, ddy = float(self._fx(s, 2)), float(self._fy(s, 2))
        speed_sq = dx * dx + dy * dy
        return (dx * ddy - dy * ddx) / speed_sq ** 1.5 if speed_sq > 1e-6 else 0.0

    def reference(self, s: float, v: float) -> Optional[ReferenceState]:
        """Reference state at absolute arc length s, or None if s is outside the window."""
        if s < self.s0 or s < 0.0:
            return None
        st = self.evaluate(min(s, self.s_end))
        return ReferenceState(pose=st.pose, twist=Twist(v, v * st.curvature), s=s, curvature=st.curvature)


def front_min_range(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    front_angle: float,
    half_width: float,
    range_min: float = 0.0,
    range_max: float = math.inf,
) -> float:
    """Smallest valid range within +/-half_width of front_angle (scan frame, radians); inf if none."""
    best = math.inf
    for k, r in enumerate(ranges):
        if not (range_min <= r <= range_max) or not math.isfinite(r):
            continue
        if abs(wrap_to_pi(angle_min + k * angle_increment - front_angle)) <= half_width:
            best = min(best, r)
    return best


# --- CSV log shared by the ROS logger node and scripts/analyze_log.py ---------------------

ROBOT_FIELDS = ("x", "y", "theta", "x_ref", "y_ref", "theta_ref", "s", "u_cmd", "omega_cmd")


def csv_header(num_robots: int) -> List[str]:
    cols = ["t"] + [f"r{i}_{f}" for i in range(num_robots) for f in ROBOT_FIELDS]
    return cols + [f"d_{i}_{i + 1}" for i in range(num_robots - 1)]


def csv_row(t: float, poses: Sequence[Pose], refs: Sequence[Pose], s: Sequence[float], cmds: Sequence[Twist]) -> List[float]:
    row = [t]
    for p, r, si, c in zip(poses, refs, s, cmds):
        row += [p.x, p.y, p.theta, r.x, r.y, r.theta, si, c.linear, c.angular]
    return row + [math.hypot(a.x - b.x, a.y - b.y) for a, b in zip(poses, poses[1:])]


def load_log_csv(path: str, desired_spacing: float):
    """Turn a logger CSV into a SimulationResult, so the simulator's metrics and plots apply."""
    from robot_conga.simulator import SimulationResult, SimulationStep

    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path} has no data rows")
    n = sum(1 for k in rows[0] if k.endswith("_x_ref"))
    result = SimulationResult(num_robots=n, desired_spacing=desired_spacing)
    t0 = float(rows[0]["t"])
    for row in rows:
        g = lambda i, f: float(row[f"r{i}_{f}"])  # noqa: E731
        poses = [Pose(g(i, "x"), g(i, "y"), g(i, "theta")) for i in range(n)]
        refs = [
            ReferenceState(Pose(g(i, "x_ref"), g(i, "y_ref"), g(i, "theta_ref")), Twist(0.0, 0.0), g(i, "s"), 0.0)
            for i in range(n)
        ]
        cmds = [Twist(g(i, "u_cmd"), g(i, "omega_cmd")) for i in range(n)]
        t = float(row["t"]) - t0
        result.times.append(t)
        result.steps.append(SimulationStep(t, refs[0].s, poses, poses, refs, cmds, cmds))
    return result
