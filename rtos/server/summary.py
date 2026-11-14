# reads every step test csv in tune_logs and puts them all in one figure:
#   left:  steady speed vs duty for each wheel (this is the feedforward curve)
#   right: every step response on the same axes
# also prints a table of steady speed and time constant per test
# usage: .venv\Scripts\python summary.py   (saves tune_logs/summary.png and shows it)
#
# pick which tests to use by the time in their file name, so air and floor sets dont get mixed:
#   summary.py --after 20261113_1239 --before 20261113_1246 --out floor_8.3V --title "floor, 8.3 V battery"
# if a duty was run more than once in the window the newest run is used

import argparse
import csv
import glob
import os

import matplotlib.pyplot as plt

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tune_logs")
PRE_MS = 200    # the robot brakes this long before the step starts (STEP_PRE_MS in motor.c)


def file_time(path):
    # step_<duty>_<YYYYMMDD>_<HHMMSS>.csv -> "YYYYMMDD_HHMMSS"
    parts = os.path.basename(path)[:-4].split("_")
    return f"{parts[-2]}_{parts[-1]}"


def load(path):
    with open(path) as f:
        rows = [[int(x) for x in r] for r in list(csv.reader(f))[1:]]
    run = [r for r in rows if r[1] != 0]
    if not run:
        return None
    return {"duty": run[0][1], "rows": rows, "run": run, "time": file_time(path)}


def steady(run, col):
    # average over the second half of the run, after it has settled
    half = run[len(run) // 2:]
    return sum(r[col] for r in half) / len(half)


def time_constant(run, col, final):
    # time to reach 63% of the final speed
    for r in run:
        if abs(r[col]) >= 0.632 * abs(final):
            return r[0] - PRE_MS
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--after", default="", help="only tests at or after this time, like 20261113_1239")
    ap.add_argument("--before", default="99999999_999999", help="only tests before this time")
    ap.add_argument("--out", default="", help="added to the png name: summary_<out>.png")
    ap.add_argument("--title", default="", help="added to the plot titles")
    args = ap.parse_args()

    paths = [p for p in glob.glob(os.path.join(LOG_DIR, "step_*.csv"))
             if args.after <= file_time(p) < args.before]
    tests = [t for t in (load(p) for p in paths) if t]
    if not tests:
        print(f"no step test csvs in {LOG_DIR} in that time window")
        return

    # if a duty was run more than once keep the newest one
    by_duty = {}
    for t in sorted(tests, key=lambda t: t["time"]):
        by_duty[t["duty"]] = t
    tests = sorted(by_duty.values(), key=lambda t: t["duty"])

    print(f"{'duty':>5} {'left c/s':>9} {'right c/s':>10} {'R/L':>5} {'tau L':>7} {'tau R':>7}")
    duties, lefts, rights = [], [], []
    for t in tests:
        left = steady(t["run"], 2)
        right = steady(t["run"], 3)
        tau_l = time_constant(t["run"], 2, left)
        tau_r = time_constant(t["run"], 3, right)
        ratio = right / left if left else 0
        print(f"{t['duty']:>4}% {left:>9.0f} {right:>10.0f} {ratio:>5.2f} {str(tau_l) + ' ms':>7} {str(tau_r) + ' ms':>7}")
        duties.append(t["duty"])
        lefts.append(left)
        rights.append(right)

    fig, (ff_ax, step_ax) = plt.subplots(1, 2, figsize=(14, 5.5))

    ff_ax.plot(duties, lefts, "o-", label="left")
    ff_ax.plot(duties, rights, "s-", label="right")
    ff_ax.axhline(0, color="gray", linewidth=0.8)
    ff_ax.axvline(0, color="gray", linewidth=0.8)
    ff_ax.set_xlabel("duty (%)")
    ff_ax.set_ylabel("steady speed (counts/s)")
    suffix = f" ({args.title})" if args.title else ""
    ff_ax.set_title("steady speed vs duty (feedforward curve)" + suffix)
    ff_ax.grid(True, alpha=0.3)
    ff_ax.legend()

    cmap = plt.get_cmap("viridis")
    for i, t in enumerate(tests):
        color = cmap(i / max(1, len(tests) - 1))
        tt = [r[0] - PRE_MS for r in t["rows"]]
        step_ax.plot(tt, [r[2] for r in t["rows"]], color=color, label=f"{t['duty']}%")
        step_ax.plot(tt, [r[3] for r in t["rows"]], color=color, linestyle="--")
    step_ax.set_xlabel("time since step (ms)")
    step_ax.set_ylabel("speed (counts/s)")
    step_ax.set_title("step responses (solid = left, dashed = right)" + suffix)
    step_ax.grid(True, alpha=0.3)
    step_ax.legend(title="duty", fontsize=8)

    fig.tight_layout()
    out = os.path.join(LOG_DIR, f"summary_{args.out}.png" if args.out else "summary.png")
    fig.savefig(out, dpi=120)
    print(f"saved {out}")
    plt.show()


if __name__ == "__main__":
    main()
