"""Print summary stats for the report. Run from project/: python3 summarize_run.py"""
import pandas as pd

s = pd.read_csv("steer_pid_data.txt", sep=r"\s+", header=None, names=["i", "e", "o"])
t = pd.read_csv("throttle_pid_data.txt", sep=r"\s+", header=None, names=["i", "e", "b", "th"])
print(f"steer rows: {len(s)}")
print(f"throttle rows: {len(t)}")
print(f"steer |error| max: {s.e.abs().max():.4f} at i={int(s.loc[s.e.abs().idxmax(), 'i'])}")
print(f"steer |output| max: {s.o.abs().max():.4f}")
sat = s.o.abs() >= 0.59
print(f"first |steer_out|>=0.59: {int(s.loc[sat, 'i'].iloc[0]) if sat.any() else None}")
for thr in (0.3, 0.5, 1.0):
    m = s.e.abs() >= thr
    print(f"first |error|>= {thr}: {int(s.loc[m, 'i'].iloc[0]) if m.any() else None}")
d = s[s.i <= 120]
print(f"i<=120 |error| mean={d.e.abs().mean():.4f} max={d.e.abs().max():.4f}")
print(f"i<=120 |output| mean={d.o.abs().mean():.4f}")
td = t[t.i <= 120]
print(f"i<=120 throttle err mean={td.e.mean():.4f} throttle mean={td.th.mean():.4f} brake max={td.b.max():.4f}")
# speed proxy: target ~3 when err~e and velocity = target - e approximately when target=3
# from throttle: e = target - v => v = target - e; when e around 1, v around 2
print(f"last 20 steer errors: {s.e.tail(5).tolist()}")
print(f"last 20 steer outs: {s.o.tail(5).tolist()}")
