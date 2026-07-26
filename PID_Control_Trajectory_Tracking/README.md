# Project: Control and Trajectory Tracking for Autonomous Vehicles

## Overview

This project implements a PID controller in C++ and uses it to track a planned
trajectory in the CARLA simulator. Two independent PID loops drive the ego
vehicle: one produces throttle/brake commands from the speed error, and one
produces steering commands from the heading error. The trajectory itself comes
from the motion planner provided with the starter code; the work here is the
controller that follows it.

## Repository layout

| Path | Purpose |
|------|---------|
| `project/pid_controller/pid_controller.h` / `.cpp` | The `PID` class (gains, error terms, saturation, anti-windup) |
| `project/pid_controller/main.cpp` | Controller integration, error computation, data logging |
| `project/plot_pid.py` | Evaluation plotting script from the starter code |
| `project/plot_pid_save.py` | Headless variant that writes the plots to PNG |
| `project/steer_pid_data.txt` | Logged steering error and output, one row per iteration |
| `project/throttle_pid_data.txt` | Logged speed error, brake output and throttle output |
| `project/report_pid_control.md` | Technical report and the required discussion questions |
| `project/pid_run_recording.mp4` | Screen recording of the run analysed below |

## Implementation

### Step 1 — the PID class

`init_controller(k_p, k_i, k_d, lim_max_output, lim_min_output)` stores the three
gains and the output saturation limits, and precomputes the clamp bounds used
for integral anti-windup.

`update_error(cte)` updates the three error terms each cycle. The proportional
term is the cross-track error itself. The derivative term is
`(cte - previous) / delta_t`, guarded so that a zero or near-zero `delta_t`
yields zero rather than a division blow-up. The integral term accumulates
trapezoidally and is then clamped to the anti-windup range.

`total_error()` returns `k_p*error_p + k_i*error_i + k_d*error_d`, saturated to
the configured output limits. `update_delta_time(dt)` stores the period used by
the derivative and integral terms.

With no error signal fed in, `total_error()` is zero and the car does not move,
which is the Step 1 acceptance check.

### Step 2 — throttle control

```cpp
double target_speed = v_points.empty() ? 0.0 : v_points.back();
double error_throttle = target_speed - velocity;
```

The last element of `v_points` is the speed the planner wants the car to reach
along the current trajectory, so the error is that target minus the measured
velocity. Using the look-ahead target rather than the speed at the closest point
keeps the loop commanding forward motion even if the local trajectory
momentarily degenerates, which avoids a zero-throttle deadlock. The PID output
is split into throttle for positive values and brake for negative ones.

### Step 3 — steering control

```cpp
double desired_yaw = angle_between_points(x_position, y_position,
                                          x_points[idx_closest_point],
                                          y_points[idx_closest_point]);
double error_steer = normalize_angle(desired_yaw - yaw);
```

The desired heading is the angle from the car's current position to the nearest
planned trajectory point. Subtracting the actual heading gives the heading
error, and `normalize_angle` keeps it in `[-PI, PI]` so the car always turns the
short way round.

### Final gains

Set in `main.cpp`:

| Loop | Kp | Ki | Kd | Output limits |
|------|-----|--------|-------|---------------|
| Steering | 0.30 | 0.0025 | 0.17 | [-0.60, 0.60] |
| Throttle | 0.21 | 0.0006 | 0.080 | [-1.0, 1.0] |

The steering limits are tightened from the suggested ±1.2 rad to ±0.60 rad,
since ±1.2 rad is about ±69°, well beyond the steering range of a real vehicle.
In practice the steering output peaked at about -0.42 during the logged run, so
this limit was not the binding constraint.

The gains were found by manual tuning: raise Kp until the response tracks,
add Kd to damp the resulting overshoot, then add a small Ki to remove
steady-state bias. No automated search was used; see the report for how that
would be done.

## Building

The build depends on CARLA's C++ client library. `CMakeLists.txt` expects the
CARLA source tree at `/opt/carla-source` and Eigen at
`project/pid_controller/eigen-3.3.7`.

```bash
cd project/pid_controller/build
cmake .. && make -j$(nproc)
```

This produces `project/pid_controller/build/pid_controller`.

## Running

This setup is WSL2 for the controller and Windows for the simulator, using
CARLA 0.9.9.4. The controller binary runs under Linux, while `run_main_pid.sh`
drives the simulator through the Windows CARLA PythonAPI egg with Windows
Python 3.7.

**1. Start CARLA on the Windows side.** From a PowerShell prompt:

```powershell
Start-Process -FilePath "C:\Users\jayan\Downloads\CARLA_0.9.9.4\WindowsNoEditor\CarlaUE4.exe" -WorkingDirectory "C:\Users\jayan\Downloads\CARLA_0.9.9.4\WindowsNoEditor"
```

Or equivalently from WSL:

```bash
cd /mnt/c && powershell.exe -NoProfile -Command "Start-Process -FilePath 'C:\Users\jayan\Downloads\CARLA_0.9.9.4\WindowsNoEditor\CarlaUE4.exe' -WorkingDirectory 'C:\Users\jayan\Downloads\CARLA_0.9.9.4\WindowsNoEditor'"
```

Do **not** launch CARLA with `cmd.exe /C start` while the working directory is a
`\\wsl.localhost\...` UNC path. Windows cannot represent that directory, and the
interop call hangs without returning control to the shell — it also ignores
Ctrl+C, so the only way out is killing the terminal. Moving to `/mnt/c` first,
as above, avoids this.

Wait for the map to finish rendering before continuing.

**2. Start the controller.**

```bash
cd project
./run_main_pid.sh
```

Confirm only one CARLA instance is running first (`tasklist.exe | grep -i carla`).
With two instances the controller connects to whichever holds port 2000, which
may not be the window you are watching.

## Evaluating the results

```bash
cd project
MPLCONFIGDIR=.mplcache python3 plot_pid_save.py
```

This reads the two logged data files and writes four PNGs. `plot_pid.py` from
the starter code produces the same plots interactively via `plt.show()`; note
that it calls `pd.read_csv(..., delim_whitespace=True)`, which pandas 3.0
removed, so it needs an older pandas than the one in this environment.

## Results

The logged run is 926 iterations long. The recording of it is at
`project/pid_run_recording.mp4`.

### Steering

![Steering error and output, driving phase](project/steer_plot_driving.png)

The steering error stays within roughly ±0.2 rad through iteration 137, with one
excursion to -0.33 rad near iteration 44 that recovers cleanly. The output
(orange) tracks the error (blue) at about 0.27 times its magnitude — the
proportional gain of 0.30 offset slightly by the accumulated integral term.
Because `delta_t` is derived from a one-second-resolution clock, the derivative
and integral terms only act on whole-second boundaries rather than every cycle;
the report discusses this and its consequences in detail.

Between iterations 138 and 144 the error collapses to -1.53 rad and then settles
at about -1.19 rad for the rest of the run. The car is resuming from a
planner-commanded stop at a junction when the path turns sharply, so the
steering reference — the closest trajectory point — swings almost perpendicular
within a few cycles. The controller answers with -0.459 against a -0.60 limit,
so it never reaches its own steering authority. The planner then reports "No
spirals generated" continuously and tracking cannot recover; section 4 of the
report works through the evidence.

### Throttle

![Throttle error and output, driving phase](project/throttle_plot_driving.png)

The speed error spikes to about 3 m/s at iterations 12, 68 and 136 whenever the
car falls behind the planned speed, and the throttle output answers each spike
with a rise to about 0.64 before settling near 0.26 as the error decays to
around 1.2 m/s. Brake output stays at essentially zero because the car is
generally under the target speed. This is the expected proportional response of
the throttle loop.

### Full-run views

The full-run plots are dominated by the post-collapse flatline and are included
for completeness: `project/steer_plot_full.png` and
`project/throttle_plot_full.png`.

## Discussion

The controller tracks the planned trajectory accurately over the drivable
stretch. It fails at a sharp junction turn taken on resume from a stop, where
the closest-point steering reference swings almost perpendicular and the
controller does not reach its own steering limit. That combination — reference
quality and steering authority — rather than the planner alone is what ends the
run; section 4 of the report sets out the evidence. A perfect trajectory is not
expected for this project.

One follow-up experiment is also documented there. The starter code derives the
PID update period from `time()`, whose one-second resolution leaves `delta_t` at
zero on most cycles. Re-deriving it from `std::chrono::steady_clock` was
implemented and tested on the `chrono-timing-fix` branch: it does activate the
derivative term, but with the gains unchanged it makes the actuation markedly
harsher without improving tracking or preventing the collision, so it is not
part of the submitted build. Section 5 of the report gives the numbers.

The full discussion — the effect of each PID term, how the gains could be tuned
automatically, the trade-offs of a model-free controller, and what would improve
it — is in [`project/report_pid_control.md`](project/report_pid_control.md).

## Possible extensions

- Tune the gains automatically with Twiddle, Ziegler–Nichols, or a black-box
  optimiser against a scalar cost combining tracking error, control effort and
  jerk.
- Replace the steering loop with a model-based law such as Stanley or
  pure-pursuit, which anticipates path curvature instead of only reacting to it.
- Schedule the gains on speed and curvature so one tuning holds across the whole
  operating envelope.
- Move to MPC, which optimises over a horizon subject to actuator and comfort
  constraints.
