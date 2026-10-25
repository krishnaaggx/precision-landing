#!/usr/bin/env python3
"""
Log files ka graph: error vs time, aur upar se path.

  pip install matplotlib
  python3 plot_logs.py p_only.csv kp15.csv ff.csv
  -> compare.png banega
"""
import csv
import sys

import matplotlib.pyplot as plt


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def main():
    files = sys.argv[1:]
    if not files:
        raise SystemExit("Usage: python3 plot_logs.py log1.csv log2.csv ...")

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(17, 5))
    for path in files:
        d = load(path)
        name = path.rsplit(".", 1)[0]
        ax1.plot(d["t"], d["error"], label=name)
        ax2.plot(d["t"], d["alt"], label=name)
        line, = ax3.plot(d["y"], d["x"], label=f"{name} (drone)")
        ax3.plot(d["ty"], d["tx"], "--", color=line.get_color(), alpha=0.5)

    ax1.set(title="Target se error", xlabel="time (s)", ylabel="error (m)")
    ax1.axhline(0.5, color="gray", ls=":", lw=1)
    ax2.set(title="Height", xlabel="time (s)", ylabel="height (m)")
    ax3.set(title="Upar se path (dashed = target)", xlabel="East (m)", ylabel="North (m)")
    ax3.set_aspect("equal", adjustable="datalim")
    for ax in (ax1, ax2, ax3):
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig("compare.png", dpi=150)
    print("Saved compare.png")


if __name__ == "__main__":
    main()
