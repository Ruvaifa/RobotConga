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

The package is organized as a clean ROS 2 Humble `ament_python` package that runs completely independently of ROS runtime libraries:

```
robot_conga/
├── package.xml                   # ROS 2 package manifest (format 3)
├── setup.py                      # ament_python build definition
├── setup.cfg                     # Script directories & pytest configuration
├── resource/robot_conga          # Ament resource marker
├── robot_conga/                  # Reusable mathematical library
│   ├── __init__.py               # Public API exports
│   ├── geometry.py               # wrap_to_pi, clamp, distance, angle_difference
│   ├── models.py                 # Pose, Twist, PathState, ReferenceState
│   ├── path.py                   # Path, StraightPath, CirclePath, SCurvePath
│   ├── propagator.py             # ReferencePropagator (spatial arc length)
│   ├── controller.py             # CongaController (Eq. 8, singularity guard)
│   ├── simulator.py              # RobotSimulator, CongaSimulator, delay, noise
│   ├── metrics.py                # RMSE, max error, spacing error analysis
│   └── plotting.py               # Matplotlib trajectory and error visualizer
├── test/                         # Comprehensive pytest test suite (31 tests)
│   ├── test_geometry.py
│   ├── test_path.py
│   ├── test_propagator.py
│   ├── test_controller.py
│   ├── test_simulator.py
│   └── test_metrics.py
├── examples/                     # Ready-to-run simulation experiments
│   ├── straight_demo.py          # Straight-line tracking (Vcmd = 0.1 m/s)
│   ├── circle_demo.py            # Circular arc tracking (R = 2.0 m)
│   ├── conga_demo.py             # 4-robot Conga line (generates 5 plots)
│   └── singularity_demo.py       # Denominator singularity detection
└── results/                      # Generated performance plots (.png)
```

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
   - Linear speed clamped to $[-0.30, 0.30]\text{ m/s}$.
   - Angular speed clamped to $[-1.50, 1.50]\text{ rad/s}$.
   - Linear acceleration clamped to $1.0\text{ m/s}^2$.
   - Angular acceleration clamped to $3.0\text{ rad/s}^2$.

---

## 12. Paper Theory vs. Engineering Implementation Choices

| Aspect | Directly from Paper (Tiwari & Nath, 2025) | Engineering Implementation Choice |
|---|---|---|
| **Controller Equation** | Eq. (8): feedback law for $u$ and $\omega$ | Exact drop-in implementation in `CongaController.compute` |
| **Gains** | $\lambda_1=4.5, \lambda_2=7.5, \lambda_3=2.5$ for TurtleBot | Fully configurable constructor parameters with validation |
| **Spatial Spacing** | $s_i = s_L - i \cdot d$ | `ReferencePropagator` with explicit negative-s bounds checking |
| **Singularity Guard** | Discussed in Section IV.C (frame rotation) | Explicit `singularity_threshold` and `ControllerSingularityError` |
| **Heading Normalization**| Not explicitly noted in Eq. (3) | `wrap_to_pi(e3)` safeguarding against $\pm \pi$ angle wind-up |
| **Simulator Dynamics** | Unicycle kinematics (Eq. 1) | RK1 kinematic integrator with configurable $dt=0.01\text{s}$ |
| **Actuation Delay** | Acknowledged in Section I as a motivation | $100\text{ ms}$ FIFO delay queue |
| **Measurement Noise** | Paper assumes mocap/UWB/vision | Gaussian noise ($\sigma_{\text{pos}}=2\text{ cm}, \sigma_{\theta}=2^{\circ}$) at $15\text{ Hz}$ |
| **Velocity Limits** | Not specified in paper | $v_{\text{max}}=0.30\text{ m/s}, \omega_{\text{max}}=1.50\text{ rad/s}$ for TurtleBot |

---

## 13. Current Limitations

1. **Y-Axis Coordinate Singularity:** Equation (8) contains $\cos(\theta^{\ast} + e_3) = \cos(\theta)$ in the denominator. When the robot's heading approaches $\pm \pi/2$ (traveling parallel to the global Y-axis), the denominator approaches zero. The controller detects this condition and raises `ControllerSingularityError`. The paper notes a local coordinate frame rotation mechanism (Section IV.C), which will be implemented in Phase 3.
2. **Stationary Lateral Controllability:** By Brockett's Theorem, a driftless non-holonomic unicycle cannot be stabilized to a fixed point in 3D $(x, y, \theta)$ via continuous pure-state feedback. When $u^{\ast} = 0$, $\dot{e}_2 = 0$. Persistent forward progression ($V_{\text{cmd}} > 0$) is mathematically required for lateral error convergence.
3. **Centralized Global State Assumption:** The current phase assumes centralized knowledge of agent positions in a global coordinate frame (e.g. Motion Capture or UWB localization).

---

## 14. Verification & Testing

### Running Unit Tests

The test suite covers geometry, path generation, propagation, controller stability, unicycle simulation, and metrics.

Using direct `pytest`:
```bash
pytest -v
```

Using `colcon` (ROS 2 workspace build & test):
```bash
colcon build --symlink-install
colcon test --packages-select robot_conga --python-testing pytest
colcon test-result --verbose
```

Result: **31 passed, 0 failures, 0 errors**.

### Running Example Demonstrations

```bash
# Straight-line tracking experiment (Vcmd = 0.1 m/s)
python examples/straight_demo.py

# Circular trajectory experiment (R = 2.0 m)
python examples/circle_demo.py

# 4-robot Conga multi-agent simulation (generates all 5 plots)
python examples/conga_demo.py

# Controller singularity detection demo
python examples/singularity_demo.py
```

### Generated Output Plots (`results/`)

Running `examples/conga_demo.py` generates the following plots:
1. `results/xy_trajectory.png`: Ground truth trajectories of leader and 3 followers along the reference path.
2. `results/position_error.png`: Euclidean tracking error vs. time for each robot.
3. `results/heading_error.png`: Absolute heading error (degrees) vs. time for each robot.
4. `results/inter_robot_spacing.png`: Inter-robot spacing between consecutive pairs compared to the $1.0\text{ m}$ reference line.
5. `results/arc_length_position.png`: Temporal progression of arc lengths $s_i(t)$.

---

## 15. Future Roadmap (Phases 3 & 4)

- **Phase 3:** Dynamic local-frame rotation to resolve the Y-axis singularity ($\theta \approx \pm \pi/2$), dynamic B-spline path generation from joystick steering inputs.
- **Phase 4:** ROS 2 Humble node implementation (`/cmd_vel` publishers, `/odom` subscribers, TF2 transformations, Nav2 integration, and physical TurtleBot 4 deployment).
