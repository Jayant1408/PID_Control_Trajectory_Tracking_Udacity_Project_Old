"""Generate PID evaluation plots (Step 4) and save them as PNG files.

Headless-friendly version of plot_pid.py: uses the Agg backend and writes
figures to disk instead of calling plt.show(). Produces both a full-run view
and a zoomed view of the driving phase (before the vehicle stalls).
"""
import matplotlib
matplotlib.use("Agg")

import pandas as pd
import matplotlib.pyplot as plt


def read_steer_data():
    df = pd.read_csv("steer_pid_data.txt", sep=r"\s+",
                     header=None, usecols=[0, 1, 2])
    df.columns = ["Iteration", "Error Steering", "Steering Output"]
    return df


def read_throttle_data():
    df = pd.read_csv("throttle_pid_data.txt", sep=r"\s+",
                     header=None, usecols=[0, 1, 2, 3])
    df.columns = ["Iteration", "Error Throttle", "Brake Output", "Throttle Output"]
    return df


def save_steer(df, n_rows, fname, title):
    d = df if n_rows < 0 else df[:n_rows]
    ax = d.plot(x="Iteration", y=["Error Steering", "Steering Output"], kind="line")
    ax.set_title(title)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Steering (rad error / normalized output)")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(fname, dpi=130)
    plt.close()
    print("wrote", fname)


def save_throttle(df, n_rows, fname, title):
    d = df if n_rows < 0 else df[:n_rows]
    ax = d.plot(x="Iteration",
                y=["Error Throttle", "Brake Output", "Throttle Output"], kind="line")
    ax.set_title(title)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Throttle (m/s error / normalized output)")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(fname, dpi=130)
    plt.close()
    print("wrote", fname)


def main():
    steer = read_steer_data()
    throttle = read_throttle_data()
    print(f"steer rows: {len(steer)}, throttle rows: {len(throttle)}")

    drive_n = 150  # driving phase before the vehicle stalls

    save_steer(steer, -1, "steer_plot_full.png", "Steering: error and output (full run)")
    save_throttle(throttle, -1, "throttle_plot_full.png", "Throttle: error and output (full run)")
    save_steer(steer, drive_n, "steer_plot_driving.png",
               f"Steering: error and output (driving phase, first {drive_n})")
    save_throttle(throttle, drive_n, "throttle_plot_driving.png",
                  f"Throttle: error and output (driving phase, first {drive_n})")


if __name__ == "__main__":
    main()
