"""Parametric B-spline path implementation and dynamic path manager for Robot Conga."""

import math
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np

try:
    from scipy.interpolate import splev, splprep
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

from robot_conga.geometry import wrap_to_pi
from robot_conga.models import PathState, Pose, ReferenceState, Twist
from robot_conga.path import Path


class BSplinePath(Path):
    """Arc-length parameterized B-spline curve fitted to 2D waypoints.

    Implements the steep-ascent coordinate rotation check from Section IV.C of the
    Robot Conga paper (Tiwari & Nath, 2025) to prevent numerical issues on near-vertical segments.
    """

    def __init__(
        self,
        waypoints: Sequence[Union[Tuple[float, float], Pose]],
        smoothing: float = 0.0,
        samples: int = 1000,
    ) -> None:
        """Initialize B-spline path from waypoints.

        Args:
            waypoints: Ordered list of (x, y) coordinates or Pose objects. Must have >= 4 points.
            smoothing: Smoothing parameter for spline fitting (default 0.0 for interpolation).
            samples: Number of discretization steps for arc-length computation.

        Raises:
            ImportError: If scipy is not installed.
            ValueError: If fewer than 4 waypoints are provided.
        """
        if not HAS_SCIPY:
            raise ImportError("scipy is required for BSplinePath (pip install scipy).")

        if len(waypoints) < 4:
            raise ValueError(f"At least 4 waypoints required for cubic B-spline, got {len(waypoints)}")

        pts = []
        for p in waypoints:
            if isinstance(p, (tuple, list)):
                pts.append((float(p[0]), float(p[1])))
            elif hasattr(p, "x") and hasattr(p, "y"):
                pts.append((float(p.x), float(p.y)))
            else:
                raise ValueError(f"Unsupported waypoint format: {p}")
        self._raw_pts = np.array(pts)

        # Steep-ascent check (Section IV-C):
        # If segments have near-zero dx and nonzero dy, rotate coordinate frame by 90 deg
        dx = np.diff(self._raw_pts[:, 0])
        dy = np.diff(self._raw_pts[:, 1])
        if np.any(np.abs(dx) < 1e-3) and np.any(np.abs(dy) > 1e-3):
            self.frame_angle = math.pi / 2.0
        else:
            self.frame_angle = 0.0

        rot_pts = self._rotate_points(self._raw_pts, -self.frame_angle)

        self.tck, _ = splprep([rot_pts[:, 0], rot_pts[:, 1]], s=smoothing)

        fine_u = np.linspace(0.0, 1.0, samples)
        x_fine, y_fine = splev(fine_u, self.tck)
        diffs = np.sqrt(np.diff(x_fine) ** 2 + np.diff(y_fine) ** 2)
        self.s_vals = np.insert(np.cumsum(diffs), 0, 0.0)
        self.total_length = float(self.s_vals[-1])

    @staticmethod
    def _rotate_points(pts: np.ndarray, angle: float) -> np.ndarray:
        c, s = math.cos(angle), math.sin(angle)
        R = np.array(((c, -s), (s, c)))
        return np.dot(pts, R.T)

    def _eval_derivatives(self, s: float) -> Tuple[float, float, float, float, float, float]:
        self._validate_s(s)
        s_clamped = min(s, self.total_length)
        u = float(np.interp(s_clamped, self.s_vals, np.linspace(0.0, 1.0, len(self.s_vals))))

        out = splev(u, self.tck)
        out_d1 = splev(u, self.tck, der=1)
        out_d2 = splev(u, self.tck, der=2)

        x_rot, y_rot = float(out[0]), float(out[1])
        dx_rot, dy_rot = float(out_d1[0]), float(out_d1[1])
        ddx_rot, ddy_rot = float(out_d2[0]), float(out_d2[1])

        # Rotate back to global frame
        c, s_ang = math.cos(self.frame_angle), math.sin(self.frame_angle)
        x = x_rot * c - y_rot * s_ang
        y = x_rot * s_ang + y_rot * c

        dx = dx_rot * c - dy_rot * s_ang
        dy = dx_rot * s_ang + dy_rot * c

        ddx = ddx_rot * c - ddy_rot * s_ang
        ddy = ddx_rot * s_ang + ddy_rot * c

        return x, y, dx, dy, ddx, ddy

    def point(self, s: float) -> Tuple[float, float]:
        x, y, _, _, _, _ = self._eval_derivatives(s)
        return x, y

    def heading(self, s: float) -> float:
        _, _, dx, dy, _, _ = self._eval_derivatives(s)
        return wrap_to_pi(math.atan2(dy, dx))

    def curvature(self, s: float) -> float:
        _, _, dx, dy, ddx, ddy = self._eval_derivatives(s)
        speed_sq = dx ** 2 + dy ** 2
        if speed_sq > 1e-6:
            return float((dx * ddy - dy * ddx) / (speed_sq ** 1.5))
        return 0.0


class PathManager:
    """Dynamic path updater that accumulates incoming waypoints from a leader node."""

    def __init__(self) -> None:
        self.current_path: Optional[BSplinePath] = None
        self.num_points: int = 0

    def update_path(self, points: Sequence[Union[Tuple[float, float], Pose]]) -> bool:
        """Update reference path from newly received waypoints.

        Args:
            points: Sequence of waypoints (x, y) or Pose.

        Returns:
            True if path was successfully updated, False otherwise.
        """
        if len(points) < 4:
            return False
        try:
            self.current_path = BSplinePath(points)
            self.num_points = len(points)
            return True
        except Exception:
            return False

    @property
    def total_length(self) -> float:
        """Total arc length of the current path in meters."""
        return self.current_path.total_length if self.current_path else 0.0

    def get_reference_state(self, s: float, v_cmd: float) -> Optional[ReferenceState]:
        """Compute reference state (Pose, Twist, s, curvature) for a given arc length.

        Args:
            s: Target arc length along path (meters).
            v_cmd: Commanded reference linear speed (m/s).

        Returns:
            ReferenceState or None if path is uninitialized or s < 0.
        """
        if self.current_path is None or self.total_length <= 0.0:
            return None
        if s < 0.0:
            return None

        state = self.current_path.evaluate(min(s, self.total_length))
        u_star = float(v_cmd)
        omega_star = float(v_cmd * state.curvature)
        return ReferenceState(
            pose=Pose(x=state.x, y=state.y, theta=state.theta),
            twist=Twist(linear=u_star, angular=omega_star),
            s=state.s,
            curvature=state.curvature,
        )

