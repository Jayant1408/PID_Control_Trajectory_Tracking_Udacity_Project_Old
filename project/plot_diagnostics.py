"""Plot the steering-reference diagnostics logged to steer_debug.log.

These figures are evidence for the two harness defects described in the report:
the reference trajectory drifting away from the vehicle, and the controller
being fed the planned path's heading instead of the car's measured heading.

Usage (from the project directory, after a run):
    MPLCONFIGDIR=.mplcache python3 plot_diagnostics.py
"""
import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd

LOG = "steer_debug.log"


def read_log():
    df = pd.read_csv(LOG, sep=r"\s+", header=0)
    print(f"{LOG}: {len(df)} rows, columns: {list(df.columns)}")
    return df


def save(fig, fname):
    fig.tight_layout()
    fig.savefig(fname, dpi=130)
    plt.close(fig)
    print("wrote", fname)


def plot_reference_distance(df):
    fig, ax = plt.subplots()
    ax.plot(df["i"], df["dist_closest"], label="nearest trajectory point")
    ax.plot(df["i"], df["dist_target"], label="look-ahead target point",
            alpha=0.7)
    ax.axhline(1.0, color="red", linestyle=":", linewidth=1,
               label="degenerate-reference threshold (1 m)")
    ax.set_title("Distance from vehicle to its steering reference")
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Distance (m)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    save(fig, "reference_distance.png")


def plot_yaw_sources(df):
    # Prefer the derived heading (clean route); fall back to the old harness field.
    yaw_col = "yaw_estimate" if "yaw_estimate" in df.columns else "yaw_vehicle"
    fig, (ax_top, ax_bot) = plt.subplots(2, 1, sharex=True,
                                         figsize=(7.0, 6.0))
    ax_top.plot(df["i"], df[yaw_col], label=f"{yaw_col} (used for steering)")
    ax_top.plot(df["i"], df["yaw_path"], label="yaw_path (planned)",
                alpha=0.8)
    ax_top.set_title("Heading signal: steering yaw vs planned path yaw")
    ax_top.set_ylabel("Yaw (rad)")
    ax_top.legend()
    ax_top.grid(True, alpha=0.3)

    ax_bot.plot(df["i"], df["yaw_path"] - df[yaw_col], color="crimson")
    ax_bot.axhline(0.0, color="black", linewidth=0.8)
    ax_bot.set_title("Difference between planned and steering heading")
    ax_bot.set_xlabel("Iteration")
    ax_bot.set_ylabel(f"yaw_path - {yaw_col} (rad)")
    ax_bot.grid(True, alpha=0.3)
    save(fig, "yaw_signal_comparison.png")


def plot_speed_tracking(df):
    fig, ax = plt.subplots()
    ax.plot(df["i"], df["target_speed"], label="target speed (planner)")
    ax.plot(df["i"], df["velocity"], label="measured speed")
    ax.plot(df["i"], df["throttle"], label="throttle command", alpha=0.6)
    ax.set_title("Speed tracking and throttle command")
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Speed (m/s) / normalized throttle")
    ax.legend()
    ax.grid(True, alpha=0.3)
    save(fig, "speed_tracking.png")


def report_summary(df):
    """Print the numbers quoted in the report so they stay verifiable."""
    yaw_col = "yaw_estimate" if "yaw_estimate" in df.columns else "yaw_vehicle"
    gap = (df["yaw_path"] - df[yaw_col]).abs()
    opposite = ((df["yaw_path"] * df[yaw_col]) < 0).sum()
    cruise = df[df["target_speed"] > 2.5]
    print("--- summary ---")
    print(f"iterations                     : {len(df)}")
    print(f"max |steer_output|             : {df['steer_output'].abs().max():.4f}")
    print(f"max |error_steer| (rad)        : {df['error_steer'].abs().max():.4f}")
    print(f"max dist_closest (m)           : {df['dist_closest'].max():.2f}")
    print(f"final dist_closest (m)         : {df['dist_closest'].iloc[-1]:.2f}")
    print(f"max |yaw_path - {yaw_col}|   : {gap.max():.4f} rad")
    print(f"cycles where the two disagree in sign: {opposite} of {len(df)}")
    if not cruise.empty:
        deficit = (cruise["target_speed"] - cruise["velocity"]).mean()
        print(f"mean speed deficit at cruise   : {deficit:.3f} m/s")
    for mode, label in ((1, "path tangent"), (2, "no reference")):
        n = (df["ref_mode"] == mode).sum()
        print(f"cycles using fallback {mode} ({label}): {n}")


def main():
    df = read_log()
    plot_reference_distance(df)
    plot_yaw_sources(df)
    plot_speed_tracking(df)
    report_summary(df)


if __name__ == "__main__":
    main()
