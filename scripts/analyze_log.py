"""Turn a Robot Conga logger CSV (hardware or gz run) into the simulator's metrics and plots.

Usage:
    python scripts/analyze_log.py conga_log_123.csv --spacing 1.0 --out results/hardware --warmup 5
"""

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_conga import compute_conga_metrics
from robot_conga.plotting import generate_all_plots
from robot_conga.transport import load_log_csv


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--spacing", type=float, default=1.0, help="desired inter-robot spacing d (m)")
    ap.add_argument("--out", default="results/hardware", help="directory for the plots")
    ap.add_argument("--warmup", type=float, default=0.0, help="seconds excluded from the metrics")
    args = ap.parse_args()

    result = load_log_csv(args.csv, args.spacing)
    m = compute_conga_metrics(result, start_time=args.warmup)
    print(f"{len(result.times)} samples over {result.times[-1]:.1f} s, {result.num_robots} robots")
    for r in m.robot_metrics:
        role = "leader" if r.robot_idx == 0 else f"follower {r.robot_idx}"
        print(f"  {role:<11} pos RMSE {r.position_rmse * 100:5.1f} cm  max {r.max_position_error * 100:5.1f} cm  "
              f"heading RMSE {math.degrees(r.heading_rmse):4.1f} deg")
    dists = result.get_inter_robot_distances()
    for pair, rmse in m.spacing_rmse.items():
        print(f"  pair {pair[0]}-{pair[1]}   spacing RMSE {rmse * 100:5.1f} cm  max dev "
              f"{m.max_spacing_deviation[pair] * 100:5.1f} cm  min dist {min(dists[pair]) * 100:5.1f} cm")
    generate_all_plots(result, output_dir=args.out)
    print(f"Plots written to {args.out}/")


if __name__ == "__main__":
    main()
