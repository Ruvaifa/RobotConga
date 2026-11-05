"""Demo: safety gate on vs off when a follower is physically blocked.

Follower 1 is held in place from t = 20 s to 35 s. Without the gate the references keep
moving, so follower 2 drives into follower 1. With the gate, s_L slows to a crawl once any
robot's measured tracking error passes 10 cm, and recovers when follower 1 is released.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_conga import CongaSimulator, SafetyGate, SimulatorConfig, StraightPath, compute_conga_metrics
from robot_conga.plotting import plot_spacing_comparison


def stall_follower_1(sim: CongaSimulator, t: float) -> None:
    sim.robots[1].stalled = 20.0 <= t < 35.0


def main() -> None:
    results = {}
    print(f"{'gate':<5} {'max track err':>14} {'min dist':>9} {'spacing RMSE':>13} {'leader s':>9}")
    for gate in (None, SafetyGate()):
        sim = CongaSimulator(
            path=StraightPath(), num_robots=3, spacing=1.0, v_cmd=0.1, gate=gate, config=SimulatorConfig(seed=0),
        )
        result = sim.run(70.0, on_step=stall_follower_1)
        label = "Gate off" if gate is None else f"Gate on (slow at {gate.slow_error} m, floor {gate.min_ratio} at {gate.stop_error} m)"
        results[label] = result
        m = compute_conga_metrics(result)
        worst_track = max(r.max_position_error for r in m.robot_metrics)
        min_dist = min(min(d) for d in result.get_inter_robot_distances().values())
        worst_spacing = max(m.spacing_rmse.values())
        print(f"{'on' if gate else 'off':<5} {worst_track * 100:11.1f} cm {min_dist * 100:6.1f} cm "
              f"{worst_spacing * 100:10.1f} cm {result.steps[-1].leader_s:7.2f} m")

    out = Path("results") / "safety_gate.png"
    plot_spacing_comparison(results, str(out), title="Follower 1 blocked from 20 s to 35 s")
    print(f"Plot saved to: {out}")


if __name__ == "__main__":
    main()
