"""Demo: Robot Conga (arc-length spacing) vs time-delayed following.

Baseline: follower i tracks the leader's reference from tau_i = i*d/v seconds ago,
with tau fixed from the nominal speed. The leader cruises, slows, stops and speeds up.
Conga keeps the spacing at d; time-delay spacing scales with speed and collapses to
zero when the leader stops for longer than tau. This is the paper's main claim, which
the paper itself does not test against a baseline.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_conga import CongaSimulator, SCurvePath, SimulatorConfig, compute_conga_metrics
from robot_conga.plotting import plot_spacing_comparison


def leader_speed(sim: CongaSimulator, t: float) -> None:
    """Cruise 0.10, slow to 0.05, stop 15 s, speed up to 0.20, cruise 0.10 (all <= 0.25 m/s)."""
    sim.v_cmd = 0.10 if t < 20 else 0.05 if t < 35 else 0.0 if t < 50 else 0.20 if t < 65 else 0.10


def main() -> None:
    results = {}
    print(f"{'mode':<12} {'pair':<7} {'spacing RMSE':>13} {'max dev':>9} {'min dist':>9}")
    for mode, label in (("conga", "Robot Conga (s_i = s_L - i d)"), ("time_delay", "Time-delayed following (tau = i d / v)")):
        sim = CongaSimulator(
            path=SCurvePath(radius=2.5, straight_entry=2.0, straight_exit=1.0),
            num_robots=3, spacing=1.0, v_cmd=0.1, following=mode, config=SimulatorConfig(seed=0),
        )
        result = sim.run(90.0, on_step=leader_speed)
        results[label] = result
        m = compute_conga_metrics(result)
        dists = result.get_inter_robot_distances()
        for pair in m.spacing_rmse:
            print(f"{mode:<12} {pair[0]}-{pair[1]:<5} {m.spacing_rmse[pair] * 100:11.1f} cm "
                  f"{m.max_spacing_deviation[pair] * 100:6.1f} cm {min(dists[pair]) * 100:6.1f} cm")

    out = Path("results") / "time_delay_vs_conga.png"
    plot_spacing_comparison(results, str(out), title="Leader cruises, slows (20 s), stops (35 s), speeds up (50 s)")
    print(f"Plot saved to: {out}")


if __name__ == "__main__":
    main()
