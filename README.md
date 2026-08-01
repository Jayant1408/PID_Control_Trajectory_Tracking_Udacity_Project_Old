# Project: Control and Trajectory Tracking for Autonomous Vehicles

**Udacity Self-Driving Car Engineer Nanodegree – Control Module**

## Objectives

* Design and integrate a PID controller in C++.
* Apply it to trajectory tracking (steering and throttle) in CARLA.
* Tune gains, log errors, plot results, and record a run.
* Answer the required discussion questions.

## Tasks

### Build the PID Controller (`pid_controller.cpp`)

* ✅ Initialise a PID instance; compute P/I/D terms; evaluate with saturation / anti-windup.
* ✅ Update `delta_t`; verify Step 1 (stationary car with no useful control).

### Throttle PID (`main.cpp`)

* ✅ Speed error = planner look-ahead speed − measured velocity; throttle/brake split; tune gains.

### Steering PID (`main.cpp`)

* ✅ Heading error to a ~5 m look-ahead path point using measured vehicle yaw; tune gains; ±0.60 rad limits.

### Evaluate (`plot_pid_save.py`, video, report)

* ✅ Log and plot steering / throttle; discuss results; answer Q1–Q5 in the report.

## Repository layout

| Path | Purpose |
|------|---------|
| `project/pid_controller/pid_controller.h` / `.cpp` | PID class |
| `project/pid_controller/main.cpp` | Integration, errors, gains, logging |
| `project/simulatorAPI.py` | CARLA client (two local fixes: see report §2.5) |
| `project/plot_pid_save.py` | Headless plots → PNG |
| `project/steer_pid_data.txt` / `throttle_pid_data.txt` | Logged run |
| `project/report_pid_control.md` | Full write-up and Q1–Q5 |
| `project/pid_run_corrected.mp4` | Screen recording of the evaluated run |
| `project/steer_plot_*.png` / `throttle_plot_*.png` | Evaluation figures |

## Final gains

```cpp
pid_steer.init_controller(0.3, 0.0025, 0.17, 0.60, -0.60);
pid_throttle.init_controller(0.21, 0.01, 0.080, 1.0, -1.0);
```

Steering limits are ±0.60 rad (inside the rubric ±1.2). Tuning was manual: raise Kp, add Kd, then Ki.

## Building

```bash
cd project/pid_controller/build
cmake .. && make -j$(nproc)
```

## Running (WSL controller + Windows CARLA 0.9.9.4)

**1. Kill leftovers**

```bash
pkill -f pid_controller; pkill -f simulatorAPI.py
taskkill.exe /F /IM python.exe 2>/dev/null
taskkill.exe /F /IM CarlaUE4-Win64-Shipping.exe 2>/dev/null
taskkill.exe /F /IM CarlaUE4.exe 2>/dev/null
```

**2. Start CARLA** (from WSL)

```bash
cd /mnt/c && powershell.exe -NoProfile -Command "Start-Process -FilePath 'C:\Users\jayan\Downloads\CARLA_0.9.9.4\WindowsNoEditor\CarlaUE4.exe' -WorkingDirectory 'C:\Users\jayan\Downloads\CARLA_0.9.9.4\WindowsNoEditor'"
```

Wait for the map to load.

**3. Run the controller**

```bash
cd project && ./run_main_pid.sh
```

Optional countdown before start (for screen recording): `./record_and_run.sh`

## Evaluating

```bash
cd project
MPLCONFIGDIR=.mplcache python3 plot_pid_save.py
```

## Results (summary)

The logged session used for the report is about **146** iterations. Steering
error stays small through the parked-car stretch; throttle error decays toward
a small residual under cruise. Near a junction resume the path turns sharply,
steering error grows above 1 rad, and tracking is lost (“No spirals generated”).
A perfect full-course trajectory is not required; the controller tracks the
drivable stretch. Full plot commentary and Q1–Q5 are in
[`project/report_pid_control.md`](project/report_pid_control.md).

<img src="project/pid_run_corrected.gif" width="100%" alt="Figure 1. PID controller tracking the planned trajectory in CARLA (pygame window).">

Full-length recording: [`project/pid_run_corrected.mp4`](project/pid_run_corrected.mp4).

![Steering, driving phase](project/steer_plot_driving.png)

![Throttle, driving phase](project/throttle_plot_driving.png)

## Local simulator notes

On this machine, `simulatorAPI.py` includes two disclosed fixes so the PID sees
correct timing and heading: `wait_time = delta_t`, and a `yaw_vehicle` field.
Details are in the report §2.5.

## Closing remarks

**Alternatives:** Stanley / pure pursuit laterally; MPC longer term.

**Extensions:** P/PD/PID ablation; Twiddle or Ziegler–Nichols for gains; finer
`delta_t` with a re-tuned Kd.

## Credits

Udacity Self-Driving Car Engineer Nanodegree (Control). Write-up structure
aligned with accepted Control/Planning project READMEs for this course.
