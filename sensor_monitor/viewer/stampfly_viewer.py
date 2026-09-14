#!/usr/bin/env python3
"""StampFly センサーモニターのリアルタイム表示

使い方:
    python3 stampfly_viewer.py                 # ポート自動検出
    python3 stampfly_viewer.py --log run1.csv  # CSV にも保存
    python3 stampfly_viewer.py --port /dev/cu.usbmodem83101 --window 20
"""

import argparse
import collections
import csv
import glob
import sys
import threading
import time

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

FIELDS = [
    "t_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "roll_deg", "pitch_deg",
    "mx_uT", "my_uT", "mz_uT", "heading_deg", "press_hPa", "temp_C", "baro_alt_m",
    "tof_bottom_mm", "tof_front_mm", "vbat_V",
]
RATE_HZ = 50


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("StampFly が見つかりません（/dev/cu.usbmodem* なし）")
    return ports[0]


class Reader(threading.Thread):
    """シリアルを読み続け、最新 N サンプルをリングバッファに保持する"""

    def __init__(self, port, maxlen, log_path):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.buf = {k: collections.deque(maxlen=maxlen) for k in FIELDS}
        self.lock = threading.Lock()
        self.messages = collections.deque(maxlen=6)
        self.log_file = open(log_path, "w", newline="") if log_path else None
        self.log = csv.writer(self.log_file) if self.log_file else None
        if self.log:
            self.log.writerow(FIELDS)
        self.count = 0

    def run(self):
        while True:
            try:
                line = self.ser.readline().decode(errors="replace").strip()
            except serial.SerialException as e:
                self.messages.append(f"serial error: {e}")
                time.sleep(1)
                continue
            if line.startswith("D,"):
                parts = line[2:].split(",")
                if len(parts) != len(FIELDS):
                    continue
                try:
                    values = [float(v) for v in parts]
                except ValueError:
                    continue
                with self.lock:
                    for k, v in zip(FIELDS, values):
                        self.buf[k].append(v)
                    self.count += 1
                if self.log:
                    self.log.writerow(parts)
            elif line.startswith("#") and not line.startswith("#HEADER"):
                self.messages.append(line[1:])

    def snapshot(self):
        with self.lock:
            return {k: list(v) for k, v in self.buf.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--window", type=float, default=10.0, help="表示する秒数")
    ap.add_argument("--log", default=None, help="CSV 保存先")
    args = ap.parse_args()

    port = args.port or find_port()
    reader = Reader(port, int(args.window * RATE_HZ), args.log)
    reader.start()

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.canvas.manager.set_window_title(f"StampFly sensor monitor - {port}")
    (ax_att, ax_gyro, ax_acc), (ax_alt, ax_mag, ax_txt) = axes

    def setup(ax, title, ylabel, series):
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
        lines = {k: ax.plot([], [], label=label, lw=1.2)[0] for k, label in series}
        ax.legend(loc="upper left", fontsize=8)
        return lines

    lines = {}
    lines.update(setup(ax_att, "Attitude", "deg", [("roll_deg", "roll"), ("pitch_deg", "pitch")]))
    lines.update(setup(ax_gyro, "Gyro (body)", "deg/s", [("gx_dps", "x"), ("gy_dps", "y"), ("gz_dps", "z")]))
    lines.update(setup(ax_acc, "Accel (body)", "G", [("ax_g", "x"), ("ay_g", "y"), ("az_g", "z")]))
    lines.update(setup(ax_alt, "Distance / Altitude", "m",
                       [("tof_bottom_mm", "ToF bottom"), ("tof_front_mm", "ToF front"),
                        ("baro_alt_m", "baro (rel.)")]))
    lines.update(setup(ax_mag, "Magnetometer (uncalibrated)", "uT",
                       [("mx_uT", "x"), ("my_uT", "y"), ("mz_uT", "z")]))
    for ax in (ax_att, ax_gyro, ax_acc, ax_alt, ax_mag):
        ax.set_xlabel("time [s]")

    ax_txt.axis("off")
    text = ax_txt.text(0.0, 1.0, "waiting for data...", va="top", family="monospace", fontsize=11)

    def update(_):
        d = reader.snapshot()
        if not d["t_ms"]:
            return
        t = [(v - d["t_ms"][-1]) / 1000.0 for v in d["t_ms"]]
        for k, line in lines.items():
            y = d[k]
            if k.startswith("tof_"):  # mm → m、測定なし(-1)は欠損
                y = [v / 1000.0 if v >= 0 else float("nan") for v in y]
            line.set_data(t, y)
        for ax in (ax_att, ax_gyro, ax_acc, ax_alt, ax_mag):
            ax.set_xlim(-args.window, 0)
            ax.relim()
            ax.autoscale_view(scalex=False)

        last = {k: v[-1] for k, v in d.items()}

        def tof(mm):
            return "  --- " if mm < 0 else f"{mm:5.0f} mm"

        text.set_text(
            f"Roll      {last['roll_deg']:7.1f} deg\n"
            f"Pitch     {last['pitch_deg']:7.1f} deg\n"
            f"Heading   {last['heading_deg']:7.1f} deg\n"
            f"\n"
            f"ToF bottom {tof(last['tof_bottom_mm'])}\n"
            f"ToF front  {tof(last['tof_front_mm'])}\n"
            f"Baro alt  {last['baro_alt_m']:7.2f} m\n"
            f"Pressure  {last['press_hPa']:7.2f} hPa\n"
            f"Temp      {last['temp_C']:7.2f} C\n"
            f"Battery   {last['vbat_V']:7.2f} V\n"
            f"\n"
            f"samples   {reader.count}\n"
            + "\n".join(m[:48] for m in reader.messages)
        )

    anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)  # noqa: F841
    fig.tight_layout()
    plt.show()
    if reader.log_file:
        reader.log_file.close()


if __name__ == "__main__":
    main()
