# Project: Control and Trajectory Tracking for Autonomous Vehicles

**Udacity Self-Driving Car Engineer Nanodegree – Control Module**

## Objectives

* Design and integrate a PID controller in C++.
* Apply the PID controller to trajectory tracking (steering and throttle).
* Test the controller in CARLA and evaluate it with plots and a run recording.
* Answer the required discussion questions (Q1–Q5).

## Tasks

### Build the PID Controller (`pid_controller.cpp` / `pid_controller.h`)

* Initialise a PID controller instance.
* Compute the PID error terms for a given cross-track / speed error.
* Evaluate the PID expression with output saturation and integral anti-windup.
* Provide a function to update the time-delta.
* Verify Step 1: with no useful error / zero gains, the ego vehicle stays stationary.

### Use PID Controller for Vehicle Throttle (`main.cpp`)

* Implement the throttle PID and split positive/negative output into throttle/brake.
* Initialise experimental gains and tune until speed tracking is acceptable.

### Use PID Controller for Vehicle Steering (`main.cpp`)

* Implement the steering PID from heading error to a look-ahead path point.
* Initialise experimental gains (with steering limits tightened to ±0.60 rad).
* Tune until lane tracking past the parked cars is acceptable.

### Evaluate Controller Efficiency (`plot_pid_save.py`, video)

* Log throttle / steering error and commands each cycle.
* Plot the logged values and discuss the results.
* Answer the discussion questions below.

---

## 1. Introduction

This project implements two independent PID loops that drive the CARLA ego
vehicle along the trajectory produced by the provided path planner: one loop
produces throttle/brake from speed error, and one produces steering from heading
error. The work follows the same shape as the Planning module write-up —
implement the TODOs, tune, evaluate in simulation, then discuss results.

Steps completed:

1. Design the PID class in C++.
2. Integrate it with CARLA through `main.cpp` and the provided simulator client.
3. Tune gains by hand (P → I → D, then clip steering limits).
4. Log errors, produce plots, and record a run video.
5. Discuss where tracking succeeds and where it fails.

---

## 2. Implementation

### 2.1 Step 1 — PID object

The `PID` class stores gains (`k_p`, `k_i`, `k_d`), output limits, the three
error terms, and `delta_t`.

* `init_controller` stores gains/limits and precomputes integral anti-windup bounds.
* `update_error(cte)` sets proportional error, a guarded derivative
  `(cte - prev) / delta_t`, and a trapezoidal integral that is then clamped.
* `total_error()` returns `k_p*error_p + k_i*error_i + k_d*error_d`, saturated.
* `update_delta_time(dt)` stores the period used by D and I.

With no error signal (or all gains zero), `total_error()` is zero and the car
does not move — the Step 1 check.

![Step 1: ego vehicle stationary with zero controller output](step1_car_stationary.png)

### 2.2 Step 2 — Throttle

```cpp
double target_speed = v_points.empty() ? 0.0 : v_points.back();
double error_throttle = target_speed - velocity;
```

The last element of `v_points` is the planner’s look-ahead speed. The error is
that target minus measured speed. Positive PID output is throttle; negative is
brake. Limits are `[-1, 1]`.

### 2.3 Step 3 — Steering

```cpp
// Look-ahead (~5 m) along the trajectory from the closest point
desired_yaw = angle_between_points(x_position, y_position,
                                   x_points[idx_target], y_points[idx_target]);
error_steer = normalize_angle(desired_yaw - yaw_vehicle);
```

Desired heading is the bearing from the car to a point ~5 m ahead on the path
(not the closest point alone). Subtracting the **measured vehicle heading**
(`yaw_vehicle`) gives the heading error; `normalize_angle` keeps it in
`[-π, π]`. Output is limited to ±0.60 rad (~±34°), inside the rubric’s ±1.2 and
closer to a realistic steering range.

### 2.4 Final gains

| Loop     | Kp   | Ki     | Kd    | Output limits |
|----------|------|--------|-------|---------------|
| Steering | 0.30 | 0.0025 | 0.17  | [-0.60, 0.60] |
| Throttle | 0.21 | 0.01   | 0.080 | [-1.0, 1.0]   |

Manual tuning: start from zero, raise Kp until the response tracks, add Kd to
damp overshoot, then a small Ki to reduce steady-state bias. Steering limits
were reduced from ±1.2 to ±0.60 following the same rationale used in accepted
Control write-ups (realistic turning angle). Throttle Ki is larger than the
common mentor seed `0.0006` so the integral can close speed error within a short
run given the coarse `delta_t` from `difftime()`.

### 2.5 Local simulator notes (disclosed)

Two small changes were required in the provided `simulatorAPI.py` so the PID
receives usable signals on a local Windows/WSL + 60 FPS CARLA setup:

1. **`wait_time = delta_t` (0.05 s)** — with `wait_time = 0`, the trajectory
   cursor advances every rendered frame, so at ~60 FPS the reference runs about
   three times real time. Gating to `delta_t` keeps path advance at real time.
2. **`yaw_vehicle`** — the original `yaw` field is the planned path heading at
   the cursor, not the car’s heading. Steering needs measured heading, so the
   client also sends `t.rotation.yaw` and `main.cpp` uses that for the error.

These are environment / interface fixes, not changes to the PID math. The
controller TODOs remain in `pid_controller.*` and `main.cpp`.

---

## 3. Results

Logged run: **146** control iterations (this recording session). Plots generated
with `plot_pid_save.py` from `steer_pid_data.txt` and `throttle_pid_data.txt`.

<img src="pid_run_corrected.gif" width="100%" alt="Figure 1. PID controller tracking the planned trajectory in CARLA.">

Full recording: [`pid_run_corrected.mp4`](pid_run_corrected.mp4).

### Throttle (driving phase)

![Throttle error, brake, and throttle output](throttle_plot_driving.png)

After the initial rise, speed error trends down from roughly 1 m/s toward
~0.2 m/s while throttle sits near 0.30–0.35. Brake is rarely used during cruise.
That is the expected P-dominated speed loop with a modest I contribution from
`Ki = 0.01`. Brief spikes appear when the planner changes target speed (slow /
stop / resume).

### Steering (driving phase)

![Steering error and steering output](steer_plot_driving.png)

Through about iteration 120 the steering error stays small (typically well under
±0.15 rad) and the output tracks it at roughly `Kp` scale — the car stays in
lane past the parked cars. Near iterations 123–128 the error jumps above 1 rad
as the car resumes through a junction while the path turns; the controller
responds but does not recover a clean track, and the planner later reports
“No spirals generated.” A perfect full-course run is not required; the
drivable stretch shows the PID doing its job.

### Full-run plots

![Steering full run](steer_plot_full.png)

![Throttle full run](throttle_plot_full.png)

### Note on `delta_t`

`main.cpp` still uses `time()` / `difftime()`, so `delta_t` is 0 on most cycles
and 1 on second boundaries. Derivative and integral therefore act
intermittently. That is a starter-code limitation; a
`std::chrono::steady_clock` experiment was explored separately and is summarised
in the discussion answers.

---

## 4. Discussion questions (Q1–Q5)

### Q1. Add the plots to your report and explain them

See Section 3. In short: throttle error decays toward a small residual while
throttle holds a steady cruise command; steering error is small while tracking
the lane and grows sharply only when the junction resume turns the reference
faster than the car can follow. Steering limits of ±0.60 prevent unrealistic
commands without stopping that late failure by themselves.

### Q2. What is the effect of the PID according to the plots? How does each part affect the command?

* **P** — command proportional to present error; dominant term that pulls the
  car toward target speed / heading. Too large → oscillation; too small → lag.
* **I** — accumulates past error to remove steady-state offset (e.g. persistent
  speed deficit). Too large → windup / slow weave; we clamp the integral
  (anti-windup). With one-second `delta_t`, I only grows on second boundaries,
  so Ki must be sized accordingly (`0.01` on throttle here).
* **D** — responds to rate of change of error and damps overshoot. With coarse
  `delta_t` it is inactive most cycles and appears as occasional jumps in the
  output-to-error ratio rather than continuous smoothing.

### Q3. How would you design a way to automatically tune the PID parameters?

* **Twiddle / coordinate ascent** on a fixed scenario, minimising a cost such as
  ∫(cross-track² + speed_error² + λ·effort) dt.
* **Ziegler–Nichols** from ultimate gain and oscillation period.
* **Black-box search** (grid, random, Bayesian optimisation, CMA-ES) in
  simulation with the same cost.
* Keep the scenario and cost repeatable so comparisons are fair. Automated
  tuning was not required for this submission; gains were set by hand.

### Q4. PID is model-free — pros and cons

**Pros:** no vehicle model required; simple to implement and run in real time;
easy to understand and retune; widely used.

**Cons:** reacts after error appears (no horizon); gains are operating-point
specific; no explicit comfort / actuator constraints beyond saturation; two
decoupled loops ignore longitudinal–lateral coupling. For sharp junction
resumes, model-based methods (e.g. MPC) or geometric laws (pure pursuit /
Stanley) are more robust.

### Q5. (Optional) What would you do to improve the controller?

* Source `delta_t` from `steady_clock` and **re-tune Kd/Ki** (untuned, a finer
  clock made actuation harsher without fixing the junction).
* Keep / refine look-ahead (already used) or switch steering to Stanley /
  pure pursuit with speed-dependent look-ahead.
* Gain-schedule on speed and curvature.
* Add curvature / load feed-forward so PID only corrects residual error.
* Longer term: MPC with actuator and comfort constraints.

---

## 5. Closing remarks

### Alternatives

* Model-based control (nonlinear MPC).
* Geometric lateral control (Stanley / pure pursuit) with the existing throttle PID.

### Extensions

* Ablate P / PD / PID to show each term’s contribution on the same scenario.
* Twiddle or Ziegler–Nichols for automatic gains.

### Credits

Assignment prepared for the Udacity Self-Driving Car Engineer Nanodegree
(Control course). Structure of this write-up follows common accepted Control
and Planning project READMEs for this nanodegree.

### References

1. Berjoza, D. Research in Kinematics of Turn For Vehicles and Semitrailers.
   Engineering for Rural Development, 2008.
2. Farag, W. A. Complex Trajectory Tracking Using PID Control for Autonomous
   Driving. Int. J. Intelligent Transportation Systems Research, 2020.
3. Udacity nd013 Control starter / CARLA trajectory-tracking project materials.
