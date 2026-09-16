#!/usr/bin/env python3
"""キーボードで StampFly を操作し、機体の状態を表示する

    python3 teleop.py                  # ポート自動検出
    python3 teleop.py --port /dev/cu.usbserial-xxxx --log run.csv

M5GO の中継ファーム（bridge/）と組み合わせて使う。
PC → M5GO → StampFly の経路で 50Hz の指令を送り続ける。

キー操作:
    スペース  離陸 / 着陸（トグル）
    w / s     前 / 後ろ
    a / d     左 / 右
    q / e     左回り / 右回り
    r / f     上昇 / 下降（自動高度モードでは目標高度の増減）
    x         すべて中立（その場で止まる）
    m         高度モードの切り替え（自動 ⇔ 手動）
    z         送信停止（機体は自動着陸する）
    Ctrl-C    終了（中立を送ってから切断）

キーは押した瞬間だけ効き、0.4秒で自動的に中立へ戻る。押しっぱなしにすると
その間だけ動き続ける。

安全について:
    * 指令が 0.3 秒途切れると M5GO が送信を止め、機体は自動着陸する。
    * M5GO のボタン C（送信停止）、C 長押し（即時停止）はいつでも使える。
    * Atom JoyStick を電源に入れておけば、手動操縦に切り替えられる。
"""

import argparse
import csv
import glob
import select
import sys
import termios
import time
import tty

import serial

SEND_HZ = 50
DECAY_S = 0.4          # キーを離してから中立に戻るまで
STEP = 0.35            # 1回の操作で入れる量（-1〜1）
THROTTLE_STEP = 0.5    # 高度の増減
ALT_AUTO, ALT_MANUAL = 4, 5

MODE_NAMES = {0: "INIT", 1: "CALIB", 2: "FLIGHT", 3: "PARKING",
              4: "LOG", 5: "LANDING", 6: "FLIP"}


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbserial-*") + glob.glob("/dev/cu.wchusbserial*"))
    if not ports:
        sys.exit("M5GO が見つかりません。USB で接続して、--port で指定してください")
    return ports[0]


class Axis:
    """押した瞬間だけ値が入り、離すと中立に戻る軸"""

    def __init__(self):
        self.value = 0.0
        self.t_set = 0.0

    def set(self, v):
        self.value = v
        self.t_set = time.time()

    def get(self):
        if self.value != 0.0 and time.time() - self.t_set > DECAY_S:
            self.value = 0.0
        return self.value


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--log", default=None, help="テレメトリを CSV に保存")
    args = ap.parse_args()

    port = args.port or find_port()
    ser = serial.Serial(port, 115200, timeout=0)
    time.sleep(0.3)

    throttle, aileron, elevator, rudder = Axis(), Axis(), Axis(), Axis()
    alt_mode = ALT_AUTO
    arm_pulse = False
    transmitting = True
    telem = {}
    last_telem_line = ""

    log_file = writer = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        writer = csv.writer(log_file)
        writer.writerow(["t_s", "roll", "pitch", "yaw", "voltage", "altitude",
                         "mode", "alt_flag", "front_mm", "thrust", "duty_fl", "duty_rr"])

    old_term = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(f"接続: {port}   スペース=離陸/着陸  wasd=移動  qe=旋回  rf=高度  x=中立  z=停止  Ctrl-C=終了")

    rx = ""
    t_next = time.time()
    try:
        while True:
            # ---- キー入力 ----
            while select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == " ":
                    arm_pulse = True
                    transmitting = True
                elif key == "w":
                    elevator.set(STEP)
                elif key == "s":
                    elevator.set(-STEP)
                elif key == "d":
                    aileron.set(STEP)
                elif key == "a":
                    aileron.set(-STEP)
                elif key == "e":
                    rudder.set(STEP)
                elif key == "q":
                    rudder.set(-STEP)
                elif key == "r":
                    throttle.set(THROTTLE_STEP)
                elif key == "f":
                    throttle.set(-THROTTLE_STEP)
                elif key == "x":
                    for ax in (throttle, aileron, elevator, rudder):
                        ax.set(0.0)
                elif key == "m":
                    alt_mode = ALT_MANUAL if alt_mode == ALT_AUTO else ALT_AUTO
                elif key == "z":
                    transmitting = False
                elif key == "\x03":
                    raise KeyboardInterrupt

            # ---- 指令の送信（50Hz） ----
            now = time.time()
            if now >= t_next:
                t_next = now + 1.0 / SEND_HZ
                if transmitting:
                    cmd = (f"C,{throttle.get():.3f},{aileron.get():.3f},{elevator.get():.3f},"
                           f"{rudder.get():.3f},{1 if arm_pulse else 0},0,0,{alt_mode}\n")
                    ser.write(cmd.encode())
                    arm_pulse = False

            # ---- 機体からの返信 ----
            data = ser.read(4096).decode(errors="replace")
            if data:
                rx += data
                while "\n" in rx:
                    line, rx = rx.split("\n", 1)
                    line = line.strip()
                    if line.startswith("T,"):
                        p = line.split(",")
                        if len(p) >= 13:
                            telem = {"t": float(p[1]), "roll": float(p[2]), "pitch": float(p[3]),
                                     "yaw": float(p[4]), "v": float(p[5]), "alt": float(p[6]),
                                     "mode": int(p[7]), "alt_flag": int(p[8]), "front": int(p[9]),
                                     "thrust": float(p[10]), "fl": float(p[11]), "rr": float(p[12])}
                            if writer:
                                writer.writerow(p[1:13])
                    elif line.startswith("#") or line.startswith("S,"):
                        last_telem_line = line

            # ---- 画面表示（10Hz） ----
            if telem and int(now * 10) % 2 == 0:
                sys.stdout.write(
                    f"\r{MODE_NAMES.get(telem['mode'], '?'):8s} "
                    f"{telem['v']:4.2f}V h{telem['alt']:5.2f}m "
                    f"R{telem['roll']:+5.1f} P{telem['pitch']:+5.1f} Y{telem['yaw']:+6.1f} | "
                    f"thr{throttle.get():+.2f} ail{aileron.get():+.2f} "
                    f"ele{elevator.get():+.2f} rud{rudder.get():+.2f} | "
                    f"FL{telem['fl']:.2f} RR{telem['rr']:.2f} "
                    f"{'ALT_AUTO' if alt_mode == ALT_AUTO else 'ALT_MAN '} "
                    f"{'送信中' if transmitting else '停止中'}   ")
                sys.stdout.flush()
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        for _ in range(5):  # 中立を送ってから終わる
            ser.write(f"C,0,0,0,0,0,0,0,{alt_mode}\n".encode())
            time.sleep(0.02)
        ser.close()
        if log_file:
            log_file.close()
        print("\n終了しました。機体は指令が途切れると自動着陸します。")
        if last_telem_line:
            print("最後の状態:", last_telem_line)


if __name__ == "__main__":
    main()
