"""Demo: Circular trajectory tracking with Robot Conga controller."""

import math
import sys
from pathlib import Path

# Add project root to sys.path if not installed
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_conga import (
    CirclePath,
    CongaSimulator,
    SimulatorConfig,
    compute_conga_metrics,
)
from robot_conga.plotting import plot_xy_trajectories


def main() -> None:
    print("=== Robot Conga: Circular Trajectory Tracking Experiment ===")
    v_cmd = 0.10  # 0.1 m/s
    radius = 2.0  # R = 2 m as specified in implementation plan
    curvature = 1.0 / radius
    omega_star = v_cmd * curvature
    print(f"Parameters: R = {radius} m, Curvature = {curvature} 1/m, omega* = {omega_star:.4f} rad/s")
    print("Full circle: every heading, including +/-90 deg where plain Eq. (8) is singular.")
    print("The default RotatedCongaController evaluates Eq. (8) in a 90-deg-snapped frame.")

    start_phi = -math.pi / 2  # bottom of the circle, heading +x
    path = CirclePath(center_x=0.0, center_y=radius, radius=radius, start_angle=start_phi, direction=1)

    config = SimulatorConfig(
        dt=0.01,
        actuation_delay=0.100,  # 100 ms actuation delay
        position_noise_std=0.02,  # 2 cm measurement noise
        heading_noise_std=math.radians(2.0),  # 2 deg heading noise
        measurement_frequency=15.0,  # 15 Hz sensor rate
        seed=100,
    )

    sim = CongaSimulator(
        path=path,
        num_robots=1,
        spacing=1.0,
        v_cmd=v_cmd,
        initial_leader_s=0.0,
        config=config,
    )

    duration = 2 * math.pi * radius / v_cmd + 5.0  # one full lap plus a margin
    print(f"Simulating full circle for {duration:.0f} seconds...")
    result = sim.run(duration=duration)

    metrics = compute_conga_metrics(result, start_time=5.0)
    lead_m = metrics.robot_metrics[0]
    print(f"Position RMSE:        {lead_m.position_rmse * 1000:.2f} mm")
    print(f"Max Position Error:   {lead_m.max_position_error * 1000:.2f} mm")
    print(f"Heading RMSE:         {math.degrees(lead_m.heading_rmse):.2f} deg")

    output_path = Path("results") / "circle_demo.png"
    plot_xy_trajectories(result, str(output_path), title=f"Full Circle Tracking (R={radius}m, Vcmd={v_cmd}m/s)")
    print(f"Plot saved to: {output_path}")


if __name__ == "__main__":
    main()
