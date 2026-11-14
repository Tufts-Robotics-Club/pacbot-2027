# runs motor step tests on the robot, saves each one to a csv and plots it
# usage: .venv\Scripts\python tune.py   (stop console.py/server.py first since they use the same port)
#
#   step <duty%> [ms]   brake 0.2 s, run both wheels at duty% for ms (default 1000), brake 0.5 s,
#                       then save tune_logs/step_<duty>_<time>.csv and show speed vs time
#   vstep <mm/s> [ms]   same but the robot holds a target speed (speed control) instead of a fixed duty,
#                       saves tune_logs/vstep_<speed>_<time>.csv and shows target vs measured speed
#   anything else       sent to the robot as is (d 30, s, x ...)
#   q                   quit

import asyncio
import csv
import os
import queue
import sys
import threading
import time

import matplotlib.pyplot as plt
from websockets.asyncio.server import serve

PORT = 8765
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tune_logs")

robot = None            # the robots connection once it connects
loop = None             # the websocket thread's event loop
log_lines = queue.Queue()   # "log ..." lines from the robot, read by the main thread


async def handle_robot(ws):
    global robot
    robot = ws
    print(f"\nrobot connected from {ws.remote_address[0]}")
    try:
        async for msg in ws:
            if msg.startswith("log"):
                log_lines.put(msg)
            elif msg.startswith("{") or msg.startswith("enc"):
                pass    # imu and encoder counts would drown out everything else
            else:
                print(f"robot: {msg}")
    except Exception as e:
        print(f"connection error: {e}")
    finally:
        if robot is ws:
            robot = None
        print("robot disconnected, waiting for it to come back...")


async def serve_forever():
    try:
        async with serve(handle_robot, "0.0.0.0", PORT):
            await asyncio.Future()
    except OSError as e:
        print(f"couldnt listen on port {PORT} ({e}). close console.py/server.py first")
        os._exit(1)


# the websocket server gets its own thread so the main thread is free for typing and matplotlib windows
def start_server():
    global loop
    loop = asyncio.new_event_loop()
    threading.Thread(target=loop.run_until_complete, args=(serve_forever(),), daemon=True).start()


def send(text):
    if robot is None:
        print("no robot connected yet")
        return False
    asyncio.run_coroutine_threadsafe(robot.send(text), loop).result(timeout=2)
    return True


def run_step(value, duration_ms, speed=False):
    # throw away anything left over from a cancelled test
    while not log_lines.empty():
        log_lines.get_nowait()

    if not send(f"{'p' if speed else 'o'} {value} {duration_ms}"):
        return

    rows = []
    deadline = time.time() + (200 + duration_ms + 500) / 1000 + 3
    while time.time() < deadline:
        try:
            line = log_lines.get(timeout=0.5)
        except queue.Empty:
            continue
        if line == "log done":
            break
        parts = line.split()
        # duty steps send 4 numbers, speed steps send 6 (the last two are the duty the PI chose)
        if len(parts) in (5, 7):
            rows.append([float(p) for p in parts[1:]])
    else:
        print("test didnt finish (cancelled or lost connection), plotting what came in")

    if not rows:
        print("no data came back")
        return

    os.makedirs(LOG_DIR, exist_ok=True)
    prefix = "vstep" if speed else "step"
    path = os.path.join(LOG_DIR, f"{prefix}_{value}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        if speed:
            w.writerow(["t_ms", "target_mm_per_s", "left_mm_per_s", "right_mm_per_s", "left_duty", "right_duty"][:len(rows[0])])
        else:
            w.writerow(["t_ms", "duty", "left_counts_per_s", "right_counts_per_s"])
        w.writerows(rows)
    print(f"saved {len(rows)} samples to {path}")

    if speed:
        plot_speed(rows, f"speed step: {value} mm/s for {duration_ms} ms", path.replace(".csv", ".png"))
    else:
        plot(rows, f"open loop step: {value}% for {duration_ms} ms", path.replace(".csv", ".png"))


def plot(rows, title, png_path):
    t = [r[0] for r in rows]
    fig, speed_ax = plt.subplots(figsize=(9, 5))
    speed_ax.plot(t, [r[2] for r in rows], label="left speed")
    speed_ax.plot(t, [r[3] for r in rows], label="right speed")
    speed_ax.set_xlabel("time (ms)")
    speed_ax.set_ylabel("measured speed (counts/s)")
    speed_ax.grid(True, alpha=0.3)

    duty_ax = speed_ax.twinx()
    # each sample covers the 20 ms before it, so the duty is drawn over that interval too
    duty_ax.step(t, [r[1] for r in rows], where="pre", color="gray", linestyle="--", label="commanded duty")
    duty_ax.set_ylabel("duty (%)")

    lines = speed_ax.get_legend_handles_labels()
    duty_lines = duty_ax.get_legend_handles_labels()
    speed_ax.legend(lines[0] + duty_lines[0], lines[1] + duty_lines[1], loc="upper right")
    speed_ax.set_title(title)
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    print(f"saved plot to {png_path}")
    plt.show()   # close the window to keep typing


def plot_speed(rows, title, png_path):
    t = [r[0] for r in rows]
    has_duty = len(rows[0]) >= 6   # newer firmware also sends the duty the PI chose
    if has_duty:
        fig, (ax, duty_ax) = plt.subplots(2, 1, figsize=(9, 7), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    else:
        fig, ax = plt.subplots(figsize=(9, 5))

    # top: target vs measured speed. each sample covers the 20 ms before it, so the target is drawn over that interval too
    ax.step(t, [r[1] for r in rows], where="pre", color="gray", linestyle="--", label="target speed")
    ax.plot(t, [r[2] for r in rows], label="left speed")
    ax.plot(t, [r[3] for r in rows], label="right speed")
    ax.set_ylabel("speed (mm/s)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    ax.set_title(title)

    # bottom: the duty the controller sent each motor (feedforward + PI correction)
    if has_duty:
        duty_ax.plot(t, [r[4] for r in rows], label="left duty")
        duty_ax.plot(t, [r[5] for r in rows], label="right duty")
        duty_ax.set_ylabel("duty (%)")
        duty_ax.set_xlabel("time (ms)")
        duty_ax.grid(True, alpha=0.3)
        duty_ax.legend(loc="upper right")
    else:
        ax.set_xlabel("time (ms)")
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    print(f"saved plot to {png_path}")
    plt.show()   # close the window to keep typing


def main():
    start_server()
    print(f"waiting for the robot on port {PORT}. type 'step <duty%> [ms]', 'vstep <mm/s> [ms]', any robot command, or q to quit")

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        if line == "q":
            break
        parts = line.split()
        if parts[0] in ("step", "vstep"):
            speed = parts[0] == "vstep"
            try:
                value = int(parts[1])
                duration_ms = int(parts[2]) if len(parts) > 2 else 1000
            except (IndexError, ValueError):
                print("usage: step <duty%> [ms]   or   vstep <mm/s> [ms]")
                continue
            if not speed and not -100 <= value <= 100:
                print("step is a duty in %, -100..100. did you mean vstep (speed in mm/s)?")
                continue
            run_step(value, duration_ms, speed)
        elif send(line):
            print(f"sent: {line}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
