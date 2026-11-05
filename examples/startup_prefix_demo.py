"""Demo: start a 3-robot line ON the path using a straight startup prefix.

Robots are placed in a line behind the leader at spacing d. A straight lead-in of
(N-1)*d is prepended to the path so s = 0 is at the last robot and the leader starts at
s = (N-1)*d. Every robot therefore starts on its own reference (the paper's k = 0 setup)
instead of sitting at s < 0 and jumping d ahead when the leader has moved far enough.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_conga import CongaSimulator, SCurvePath, SimulatorConfig, compute_conga_metrics
from robot_conga.plotting import plot_xy_trajectories


def main() -> None:
    num_robots, d = 3, 1.0
    path = SCurvePath(radius=2.0, sweep_angle=math.pi / 2, straight_entry=0.5, straight_exit=2.0)
    sim = CongaSimulator(
        path=path, num_robots=num_robots, spacing=d, v_cmd=0.1,
        startup_prefix=True, config=SimulatorConfig(seed=0),
    )
    refs = sim.propagator.propagate(sim.leader_s)
    print("=== Startup prefix: initial references (all on the path) ===")
    for i, r in enumerate(refs):
        print(f"  robot {i}: s = {r.s:.2f} m at ({r.x:+.2f}, {r.y:+.2f})")

    result = sim.run(80.0)
    metrics = compute_conga_metrics(result)
    print("Per-robot max position error from t = 0 (no warm-up excluded):")
    for m in metrics.robot_metrics:
        print(f"  robot {m.robot_idx}: max {m.max_position_error * 100:.1f} cm, RMSE {m.position_rmse * 100:.1f} cm")

    out = Path("results") / "startup_prefix.png"
    plot_xy_trajectories(result, str(out), title="Startup prefix: 3 robots start on the path")
    print(f"Plot saved to: {out}")


if __name__ == "__main__":
    main()
