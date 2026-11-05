"""Robot Conga: A Leader-Follower Walking Approach to Sequential Path Following in Multi-Agent Systems."""

from robot_conga.bspline_path import BSplinePath, PathManager
from robot_conga.controller import (
    CongaController,
    ControllerSingularityError,
    RotatedCongaController,
    singularity_fallback,
)
from robot_conga.geometry import angle_difference, clamp, distance, wrap_to_pi
from robot_conga.metrics import (
    CongaSummary,
    RobotMetrics,
    compute_conga_metrics,
    heading_error,
    inter_robot_distance,
    max_error,
    max_spacing_deviation,
    position_error,
    rmse,
    spacing_error,
)
from robot_conga.models import PathState, Pose, ReferenceState, Twist
from robot_conga.path import CirclePath, Path, PrefixedPath, SCurvePath, StraightPath
from robot_conga.propagator import ReferencePropagator
from robot_conga.simulator import (
    CongaSimulator,
    RobotSimulator,
    SimulationResult,
    SimulationStep,
    SafetyGate,
    SimulatorConfig,
)

__all__ = [
    "Pose",
    "Twist",
    "PathState",
    "ReferenceState",
    "wrap_to_pi",
    "clamp",
    "distance",
    "angle_difference",
    "Path",
    "StraightPath",
    "CirclePath",
    "SCurvePath",
    "PrefixedPath",
    "BSplinePath",
    "PathManager",
    "ReferencePropagator",
    "CongaController",
    "ControllerSingularityError",
    "RotatedCongaController",
    "singularity_fallback",
    "SimulatorConfig",
    "SafetyGate",
    "RobotSimulator",
    "CongaSimulator",
    "SimulationStep",
    "SimulationResult",
    "position_error",
    "heading_error",
    "rmse",
    "max_error",
    "inter_robot_distance",
    "spacing_error",
    "max_spacing_deviation",
    "RobotMetrics",
    "CongaSummary",
    "compute_conga_metrics",
]
