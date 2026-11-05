# Robot Conga: Sequential Path Following for Multi-Agent Systems

A ROS 2 / Python implementation of the **Robot Conga** leader-follower sequential path following strategy for multi-agent non-holonomic systems, based on the research paper:

> **"Robot Conga: A Leader-Follower Walking Approach to Sequential Path Following in Multi-Agent Systems"**  
> Pranav Tiwari\*, Soumyodipta Nath\* (Cyber-Physical Systems, Indian Institute of Science, Bengaluru)  
> [arXiv:2509.16482](https://arxiv.org/abs/2509.16482) | [Project Website](https://robot-conga.github.io)

---

## 1. Problem Statement

In multi-agent robotic systems (such as warehouse AGVs, industrial pipeline inspection units, hospital disinfection teams, and exploratory rovers), multiple robots frequently need to traverse constrained corridors, aisles, or shared pathways in an orderly, single-file fashion.

Traditional formation control strategies generally enforce rigid geometric shapes (triangles, lines abreast) or rely on time-parameterized trajectories and time-delay following ($t - \tau$). In practice, these methods suffer from severe drawbacks:
- **Synchronization Brittleness:** Heterogeneous agents, wheel slippage, or localized actuation delays cause agents to lose phase synchronization.
- **Corner Cutting / Distortion:** Rigid formations cannot negotiate narrow winding corridors.
- **Cumulative Latency:** Delayed-time followers accumulate errors when the leader slows down, accelerates, or stops.

**Robot Conga** addresses this challenge by formulating **sequential path following**: all agents must follow the *exact same spatial path*, maintaining a fixed *spatial separation* ($d$) along the path, dynamically guided by the leader.

---

## 2. Robot Conga Concept

Robot Conga is a centralized leader-follower trajectory tracking framework. The leader progresses along a shared reference curve, and reference states for all follower robots are propagated along the curve as a function of the leader's **arc-length displacement**, rather than elapsed time:

$$s_i = s_L - i \cdot d, \quad i = 0, 1, \dots, N-1$$

where:
- $i = 0$ is the leader ($s_0 = s_L$),
- $i > 0$ are the followers,
- $s_L$ is the leader's current continuous arc length along the path,
- $d$ is the commanded inter-robot spatial separation (e.g., $1.0\text{ m}$).

Each robot receives a reference state $(x_i^{\ast}, y_i^{\ast}, \theta_i^{\ast}, u_i^{\ast}, \omega_i^{\ast})$ corresponding strictly to its arc length $s_i$. When the leader pauses ($u_L = 0$), $s_L$ stops advancing, and all followers naturally come to rest at their designated spatial offsets without time-lapse accumulation.

---

## 3. Spatial Propagation vs. Time-Delay Following

| Characteristic | Traditional Time-Delay Following ($t - \tau$) | Robot Conga Spatial Propagation ($s_L - i \cdot d$) |
|---|---|---|
| **Progression Variable** | Time $t \ge 0$ | Arc length $s \ge 0$ along curve |
| **Leader Stop Behavior** | Followers keep moving for $\tau$ seconds before stopping | Followers pause synchronously with path displacement |
| **Velocity Variations** | Spacing stretches at high speed, compresses at low speed | Spacing remains constant ($d$ meters) regardless of speed |
| **Heterogeneous Dynamics** | Requires identical acceleration profiles to prevent collisions | Accommodates differing platform dynamics natively |
| **Path Fidelity** | Waypoint-history playback can drift off-path | Every robot references the analytic path curve |

---

## 4. Mathematical Model: Unicycle Kinematics

Each mobile robot is modeled as a non-holonomic unicycle system:

$$\dot{x}(t) = u(t) \cos[\theta(t)]$$

$$\dot{y}(t) = u(t) \sin[\theta(t)]$$

$$\dot{\theta}(t) = \omega(t)$$

where:
- $(x(t), y(t))$ represents 2D position in the global coordinate frame,
- $\theta(t) \in (-\pi, \pi]$ represents the global heading angle,
- $u(t)$ is the forward linear velocity control input,
- $\omega(t)$ is the angular velocity control input.

For each virtual reference agent $i$, the reference trajectory satisfies identical unicycle kinematics:

$$\dot{x}_i^{\ast}(t) = u_i^{\ast}(t) \cos[\theta_i^{\ast}(t)]$$

$$\dot{y}_i^{\ast}(t) = u_i^{\ast}(t) \sin[\theta_i^{\ast}(t)]$$

$$\dot{\theta}_i^{\ast}(t) = \omega_i^{\ast}(t)$$

---

## 5. Tracking Error

For each agent, the tracking error vector $\mathbf{e}(t) = [e_1(t), e_2(t), e_3(t)]^{T}$ is defined in the global frame:

$$e_1(t) = x(t) - x^{\ast}(t)$$

$$e_2(t) = y(t) - y^{\ast}(t)$$

$$e_3(t) = \theta(t) - \theta^{\ast}(t)$$

Taking the time derivative yields the open-loop error dynamics:

$$\dot{e}_1 = u \cos(\theta) - u^{\ast} \cos(\theta^{\ast})$$

$$\dot{e}_2 = u \sin(\theta) - u^{\ast} \sin(\theta^{\ast})$$

$$\dot{e}_3 = \omega - \omega^{\ast}$$

> **Implementation Note:** Normalizing $e_3$ to $[-\pi, \pi)$ via `wrap_to_pi` is an essential engineering safeguard implemented in code to prevent angle wind-up across the $\pm \pi$ branch cut.

---

## 6. Robot Conga Controller (Equation 8)

The paper employs a nonlinear state-feedback control law derived via Lyapunov stability analysis (Ailon & Zohar, 2007; Tiwari & Nath, 2025):

$$u(t) = \frac{u^{\ast}(t) \cos[\theta^{\ast}(t)] - \lambda_3 e_1(t)}{\cos[\theta^{\ast}(t) + e_3(t)]}$$

$$\omega(t) = \omega^{\ast}(t) - \lambda_2 e_3(t) - \lambda_1 e_2(t)$$

where $\lambda_1, \lambda_2, \lambda_3 > 0$ are positive feedback gains.

### Closed-Loop Error Dynamics

Substituting the controller into the unicycle error dynamics yields:

$$\dot{e}_1 = -\lambda_3 e_1$$

$$\dot{e}_2 = -\lambda_3 e_1 \tan(\theta^{\ast}) + \frac{u^{\ast} \sin(e_3)}{\cos(\theta^{\ast} + e_3)}$$

$$\dot{e}_3 = -\lambda_1 e_2 - \lambda_2 e_3$$

Under persistent forward motion ($u^{\ast} > 0$), the Lyapunov candidate function:

$$V(\mathbf{e}) = \frac{1}{2} \left( \delta e_1^2 + \delta_1 e_2^2 + 2 e_2 e_3 + e_3^2 \right)$$

satisfies $\dot{V}(\mathbf{e}) < 0$ in a local domain $\mathcal{D}$ for appropriately chosen constants $\delta, \delta_1 > 0$, establishing local exponential stability $\mathbf{e} \to \mathbf{0}$.

### Paper Tuning Parameters (Table II)

| Robot Platform | $\lambda_1$ (Lateral) | $\lambda_2$ (Heading) | $\lambda_3$ (Longitudinal) |
|---|---|---|---|
| **TurtleBot3** | 4.5 | 7.5 | 2.5 |
| **Laikago Quadruped** | 4.5 | 1.5 | 2.5 |
| **Heterogeneous (Mixed)** | 5.0 | 1.0 | 1.5 |

---

## 7. Arc-Length Propagation

Given a path parameterized by arc length $s$, the reference state for robot $i$ is calculated from the leader's spatial progress $s_L$:

$$s_i = s_L - i \cdot d$$

The continuous path model evaluates:
- $\mathbf{p}(s_i) = (x(s_i), y(s_i))$
- $\theta(s_i) = \operatorname{atan2}(y'(s_i), x'(s_i))$
- $\kappa(s_i) = \theta'(s_i)$ (signed path curvature)

Negative arc-length positions ($s_i < 0$) are rejected explicitly by `ReferencePropagator`, preventing followers from evaluating uninitialized path regions prior to the origin.

---

## 8. Reference Velocity Calculation

The reference linear velocity along the trajectory is commanded in real time (e.g., via operator input or mission planner):

$$u_i^{\ast} = V_{\text{cmd}}$$

The reference angular velocity is computed directly from path curvature $\kappa$:

$$\omega_i^{\ast} = V_{\text{cmd}} \cdot \kappa(s_i)$$

This formulation is mathematically equivalent to the paper's instantaneous radius of curvature formulation:

$$\omega_i^{\ast} = \frac{u_i^{\ast}}{\text{IROC}(s_i)}, \quad \text{where } \text{IROC} = \frac{1}{\kappa}$$

- For a **StraightPath**: $\kappa = 0 \implies \omega^{\ast} = 0$.
- For a **CirclePath** of radius $R$: $\kappa = \pm 1/R \implies \omega^{\ast} = \pm V_{\text{cmd}} / R$.

---

## 9. Software Architecture

`robot_conga` is a ROS 2 **Jazzy** `ament_python` package. The library (everything except the `*_node.py`, `ros_common.py` and `launch_common.py` modules) runs without ROS:

```
robot_conga/
├── package.xml / setup.py / setup.cfg / resource/robot_conga
├── robot_conga/
│   ├── geometry.py, models.py    # wrap_to_pi, clamp, Pose, Twist, ReferenceState
│   ├── path.py                   # Straight, Circle, SCurve, PrefixedPath (startup lead-in)
│   ├── bspline_path.py           # BSplinePath, PathManager (offline spline from waypoints)
│   ├── propagator.py             # ReferencePropagator: s_i = s_L - i d
│   ├── controller.py             # CongaController (Eq. 8), RotatedCongaController (default),
│   │                             #   singularity_fallback
│   ├── simulator.py              # unicycles + delay + noise; conga / time_delay following;
│   │                             #   SafetyGate; on_step hook (speed profiles, stalls)
│   ├── metrics.py, plotting.py   # RMSE / spacing metrics, plots
│   ├── transport.py              # leader path window + s tagging, SampledPath, lidar e-stop
│   │                             #   check, CSV log format (all ROS-free and unit-tested)
│   ├── ros_common.py             # WorldPoseSource (odom+TF / PoseStamped), cmd_vel publisher
│   ├── leader_node.py            # records path, publishes /conga/path + /conga/leader_state
│   ├── follower_node.py          # rotated Eq. 8 + watchdog + lidar e-stop
│   ├── logger_node.py            # CSV logger for hardware / gz runs
│   └── launch_common.py          # node set shared by both launch files
├── launch/
│   ├── conga_launch.py           # Gazebo (gz, turtlebot4_gz_bringup) + Conga nodes
│   └── conga_hardware_launch.py  # real robots only, no simulation
├── hardware/                     # phase0_bringup.sh (env + checks), fastdds.xml
├── scripts/analyze_log.py        # logger CSV -> same metrics and plots as the simulator
├── examples/                     # simulation experiments (see section 15)
└── test/                         # pytest suite (62 tests)
```

### What changed in v0.2 (review fixes)

| Finding | Fix |
|---|---|
| Eq. 8 diverges or is singular for path headings beyond about ±75° (cos θ denominator; the −λ1 e2 term has the wrong sign toward −x). A full circle fails. | `RotatedCongaController` evaluates Eq. 8 in a frame rotated by α, the multiple of 90° nearest θ*, switching only when \|θ* − α\| > 45° + 10° hysteresis. Positions are rotated by −α and α is subtracted from both headings; u and ω are body-frame and pass through unchanged. It is the default in the simulator and the follower node. |
| Each robot's odom frame starts at its own power-on pose. | Shared `world` frame: `WorldPoseSource` takes either a world-frame PoseStamped topic or odom + static TF `world → tb4_i/odom` from the hand-measured start pose (odom is zeroed at node start). |
| Followers start at s < 0 and jump d ahead. | Startup prefix: a straight lead-in of (N−1)d behind the leader, so s = 0 is at the last robot and the leader starts at s = (N−1)d (`PrefixedPath`, `startup_prefix=True`, and in the leader node). |
| A singularity freezes the follower permanently. | Last-resort `singularity_fallback`: u = 0, turn toward θ*, so the next cycle can use Eq. 8 again. |
| Unbounded `nav_msgs/Path`; interpolating spline through noisy history; chord-sum s_L ≠ follower spline length. | The leader sends only a recent window (default (N−1)d + 2 m, hard cap 400 poses ≈ 32 KB), each pose carrying its arc length s in `position.z`. Followers fit smoothing splines x(s), y(s) (smoothing > 0) over the same points, so s_L and s_i share one parameterization. Leader state is a stamped `Vector3Stamped` (s_L, u_L, ω_L). |
| Jazzy: TwistStamped, gz rename, mixed namespaces. | `cmd_vel_type` parameter (default TwistStamped); `turtlebot4_gz_bringup` (`sim.launch.py`, `turtlebot4_spawn.launch.py`); `tb4_i` namespaces everywhere. |
| No safety behaviour. | Follower watchdog (pose or leader state older than 0.2 s → zero velocity), lidar e-stop (< 0.3 m in a ±30° front sector, or no scan for 0.5 s → zero velocity), 0.25 m/s hard speed cap, and an optional `SafetyGate` in the simulator. |

### Running on ROS 2 Jazzy

```bash
colcon build --symlink-install && source install/setup.bash

# Gazebo (gz): world + 3 TurtleBot 4s + Conga nodes; drive tb4_0 with teleop
ros2 launch robot_conga conga_launch.py num_robots:=3 delta_s:=1.0

# Real robots: measure start poses (x, y, yaw in degrees) in your world frame
ros2 launch robot_conga conga_hardware_launch.py num_robots:=3 delta_s:=1.0 \
    start_poses:='0,0,0;-1,0,0;-2,0,0' cmd_vel_type:=TwistStamped pose_source:=odom
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:=/tb4_0/cmd_vel -p stamped:=true

# Afterwards: metrics and plots from the logger CSV
python scripts/analyze_log.py conga_log_<time>.csv --spacing 1.0 --out results/hardware
```

Topics: `/conga/path` (nav_msgs/Path, transient-local), `/conga/leader_state` (Vector3Stamped: x = s_L, y = u_L, z = ω_L), `/tb4_i/conga/pose` and `/tb4_i/conga/reference` (PoseStamped in `world`, reference z = s_i), `/tb4_i/cmd_vel`. With `pose_source:=topic`, each node instead reads `/tb4_i/world_pose` (PoseStamped, e.g. from an overhead-camera tracker, in any frame with a TF to `world`).

The nodes were smoke-tested on ROS 2 Humble (WSL) against simulated unicycle robots publishing fake odom and scan: followers start on the path, track the leader in the world frame, stop on the watchdog and the lidar e-stop, and the logger CSV feeds `analyze_log.py`. They have **not** yet run on real TurtleBot 4s or in gz.

---

## 10. Simulator & Dynamic Pipeline

The simulator implements a high-fidelity unicycle simulation loop:

```
Reference Path ──► ReferencePropagator (s_i = s_L - i*d)
                          │
                          ▼ ReferenceState (x*, y*, theta*, u*, omega*)
                     Controller (Equation 8)
                          │
                          ▼ Commanded Twist (u_cmd, omega_cmd)
                    Actuation Delay Queue (FIFO, default 100 ms)
                          │
                          ▼ Delayed Target Twist
                    Acceleration Limiter (linear & angular ramp)
                          │
                          ▼ Actual Applied Twist (u, omega)
                   Unicycle Dynamic Integration (dt = 0.01 s)
                          │
                          ▼ True Pose (x, y, theta)
                 Sensor Noise & 15 Hz Sampling
                          │
                          ▼ Measured Pose (x_meas, y_meas, theta_meas)
                          └──────────► Controller Feedback
```

---

## 11. Noise and Delay Model

To reflect physical indoor robotic platforms (e.g. TurtleBot 4), the simulator incorporates realistic physical effects:

1. **Actuation Latency:** Command delay buffer implementing a FIFO queue (default $100\text{ ms}$). Commands issued at time $t$ take effect at $t + \tau_{\text{delay}}$.
2. **Sensor Measurement Noise:** Zero-mean additive Gaussian white noise applied to global localization measurements:
   - Position noise: $\sigma_{\text{pos}} = 0.02\text{ m}$ ($2\text{ cm}$).
   - Heading noise: $\sigma_{\theta} = 2.0^{\circ}$ ($0.0349\text{ rad}$).
   - *The true robot kinematic state remains completely separate from the noisy measured state.*
3. **Discrete Measurement Rate:** Sensor updates occur at $15\text{ Hz}$ ($66.7\text{ ms}$ interval), with sample-and-hold between cycles.
4. **Velocity & Acceleration Saturation:**
   - Linear speed clamped to $[-0.25, 0.25]\text{ m/s}$.
   - Angular speed clamped to $[-1.50, 1.50]\text{ rad/s}$.
   - Linear acceleration clamped to $1.0\text{ m/s}^2$.
   - Angular acceleration clamped to $3.0\text{ rad/s}^2$.

---

## 12. Paper Theory vs. Engineering Implementation Choices

| Aspect | Directly from Paper (Tiwari & Nath, 2025) | Engineering Implementation Choice |
|---|---|---|
| **Controller Equation** | Eq. (8): feedback law for $u$ and $\omega$ | `CongaController.compute` exactly; `RotatedCongaController` (default) evaluates it in a 90°-snapped rotated frame |
| **Gains** | $\lambda_1=4.5, \lambda_2=7.5, \lambda_3=2.5$ for TurtleBot | Configurable constructor / node parameters |
| **Spatial Spacing** | $s_i = s_L - i \cdot d$ | `ReferencePropagator`; startup prefix puts all robots on the path at $k = 0$ |
| **Singularity** | Section IV.C rotates the spline frame for steep segments | Not sufficient (the singularity is in the controller's global-frame errors); controller-frame rotation instead, with `ControllerSingularityError` + turn-in-place fallback as a last resort |
| **Heading Normalization**| Not explicitly noted in Eq. (3) | `wrap_to_pi(e3)` |
| **Simulator Dynamics** | Unicycle kinematics (Eq. 1) | Euler integration, $dt=0.01\text{ s}$ |
| **Actuation Delay** | Motivation in Section I | $100\text{ ms}$ FIFO delay queue |
| **Measurement Noise** | Mocap/UWB/vision assumed | Gaussian ($\sigma_{\text{pos}}=2\text{ cm}, \sigma_{\theta}=2^{\circ}$) at $15\text{ Hz}$ |
| **Velocity Limits** | Not specified | $v_{\text{max}}=0.25\text{ m/s}$ (hard cap in the nodes), $\omega_{\text{max}}=1.50\text{ rad/s}$ |
| **Baseline** | Claims an advantage over time-delay following but never tests it | `following="time_delay"` baseline in the simulator, compared in `examples/time_delay_comparison.py` |

---

## 13. Results

All numbers come from the simulator with the default noise (2 cm, 2°, 15 Hz) and 100 ms delay, seed 0.

**Any heading.** 20 cm lateral offset on a straight path; mean position error over t = 35–40 s:

| Path heading | 0° | 60° | 90° | 120° | 180° | −90° | −150° |
|---|---|---|---|---|---|---|---|
| Plain Eq. 8 | 2.3 cm | 4.1 cm | 4.3 m (diverged) | 3.1 m (diverged) | singular | singular | singular |
| Rotated (default) | 2.3 cm | 1.1 cm | 2.7 cm | 4.3 cm | 1.2 cm | 1.2 cm | 2.4 cm |

Full circle, 3 robots (R = 1.5 m, v = 0.1 m/s, d = 1 m): position RMSE 1.7–2.1 cm, max 5.5–10 cm. Plain Eq. 8 hits the singularity.

**Conga vs time-delayed following** (`examples/time_delay_comparison.py`, `results/time_delay_vs_conga.png`). 3 robots, d = 1 m. The leader cruises at 0.10 m/s, slows to 0.05, stops for 15 s, then runs at 0.20. The baseline replays the leader's reference from τ_i = i d / v ago, with v the nominal 0.10 m/s:

| Following | Spacing RMSE | Max deviation | Min distance |
|---|---|---|---|
| Robot Conga | 1.9–2.2 cm | 7.1 cm | 93 cm |
| Time delay | 51 cm | 98–102 cm | **1–2 cm (collision)** |

**Safety gate** (`examples/safety_gate_comparison.py`, `results/safety_gate.png`). Follower 1 is physically held from 20 s to 35 s:

| Gate | Max tracking error | Min distance | Worst spacing RMSE |
|---|---|---|---|
| Off | 150 cm | **1.6 cm** (follower 2 drives into follower 1) | 52 cm |
| On (slows above 10 cm, 10 % floor at 30 cm) | 38 cm | 62 cm | 14 cm |

**Startup prefix** (`examples/startup_prefix_demo.py`, `results/startup_prefix.png`): robots lined up behind the leader start on their references. RMSE is 2.8–3.5 cm from t = 0, with no warm-up excluded.

---

## 14. Current Limitations

1. **The noise floor depends on the frame offset.** With 2 cm / 2° noise and 100 ms delay, the paper's gains make the heading weave by about ±15° (±1° without noise). The steady error is about 2.5 cm when θ* is aligned with the rotated frame and about 4–5 cm at a 30° offset. In about 15 % of random seeds, the 30°-offset cases end above 5 cm. Lower λ2, or a frame that rotates continuously with θ*, would help, but either changes the paper's controller.
2. **Lateral convergence needs motion.** At u* = 0, Eq. 8 cannot reduce lateral error (Brockett). The safety gate therefore slows to a 10 % floor by default instead of stopping, which avoids a deadlock when the large error is lateral. `SafetyGate(min_ratio=0)` gives a hard stop.
3. **The safety gate is simulator-only.** In ROS each follower only stops itself (watchdog / e-stop); the leader is driven by a person and does not wait for the line.
4. **Odom drift.** With `pose_source:=odom`, the world frame is only as good as each robot's odometry plus the hand-measured start pose, and errors grow over a run. An external tracker (`pose_source:=topic`) removes this; an overhead-camera node is not included.
5. **Watchdog vs Wi-Fi.** A 0.2 s timeout on odom (≈20 Hz) and on leader state over Wi-Fi can trigger on jitter and make followers stutter. If that happens, raise `watchdog_timeout`.
6. **Unverified hardware assumptions:** the cmd_vel type (default TwistStamped), the RPLIDAR front at −90° in the scan frame (taken from the TB4 Jazzy URDF), and odom QoS (subscribed best-effort, which matches either publisher setting).
7. **Path transport shortcut:** the arc length s rides in `PoseStamped.position.z`, which avoids a custom message package. This only works for planar robots.

---

## 15. Verification & Testing

```bash
pytest -q                                   # 62 passed
colcon test --packages-select robot_conga --python-testing pytest
```

Examples (all write to `results/`):

```bash
python examples/conga_demo.py              # 4-robot S-curve, 5 plots
python examples/circle_demo.py             # full circle (was a ±45° arc before the rotated controller)
python examples/straight_demo.py
python examples/singularity_demo.py        # plain Eq. 8 singularity guard
python examples/startup_prefix_demo.py     # robots start on the path
python examples/time_delay_comparison.py   # Conga vs time-delayed following
python examples/safety_gate_comparison.py  # gate on vs off with a blocked follower
```

### Recommended hardware test order

1. Per robot: run `hardware/phase0_bringup.sh tb4_i`, check the odom rate and `ros2 topic info -v /tb4_i/cmd_vel` (type), and teleop with the matching `stamped` flag.
2. E-stop direction: with one follower running, hold a hand in front and confirm `Stopped: obstacle` appears in its log; a hand at the side must not trigger it.
3. World frame: launch with all robots placed, then compare `ros2 topic echo /tb4_i/conga/pose` against tape-measured positions. Drive the leader 2 m and re-measure to see odom drift.
4. One follower, straight line, d = 1 m, 0.1 m/s, with someone ready at the e-stop / teleop.
5. Watchdog: kill the leader node mid-run; the followers should stop within 0.2 s.
6. Turns through ±90° and 180° (the rotated-controller case), then a full circle.
7. Three robots, then leader slow / stop / speed-up, to reproduce the spacing comparison on hardware with `analyze_log.py`.
