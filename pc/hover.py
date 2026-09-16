#!/usr/bin/env python3
"""OAK-D で測った位置をもとに、StampFly を1点でホバリングさせる

    # まず飛ばさずに確認（指令は計算するが送らない。マーカーを手で動かして見る）
    python3 hover.py --dry-run

    # 実際に飛ばす
    python3 hover.py --log hover1.csv

前提:
    * `oakd/track_world.py --calibrate` で座標系を決めてあること
    * M5GO に bridge/ のファームが入っていて、USB で繋がっていること
    * 機体にマーカー（既定 id 2, 2.8cm）が貼ってあること

しくみ:
    カメラで測った位置 → 目標との差 → PID → 傾ける角度 → スティック値 → M5GO → 機体
    高さは機体の自動高度モードに任せ、PC は目標高度を少しずつ動かすだけ。
    姿勢の安定化（速い制御）は機体が 400Hz で行う。PC は位置だけを見る（30Hz）。

キー操作:
    スペース  離陸 / 着陸
    w/s/a/d   目標位置を前後左右に 10cm ずつ動かす
    r / f     目標高度を 10cm 上下
    h         いまの位置を目標にする（その場で止まる）
    z         送信停止（機体は自動着陸）
    Ctrl-C    終了

安全のしくみ:
    * マーカーを見失って 0.5 秒たつと送信を止める → 機体は自動着陸する
    * 決めた範囲（既定 ±0.8m、高さ 1.2m）を出たら送信を止める
    * 傾ける角度は既定 8 度まで
    * M5GO のボタン C（停止）、C 長押し（即時停止）はいつでも効く
    * **Atom JoyStick の電源は切っておくこと**（同時に送信すると指令が打ち消される）
"""

import argparse
import csv
import glob
import pathlib
import select
import shutil
import sys
import termios
import time
import tty

import cv2
import depthai as dai
import numpy as np
import serial

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "oakd"))
from marker_common import (RESOLUTIONS, build_pipeline, latest, make_detector,  # noqa: E402
                           marker_object_points, solve_pose, wrap_deg)

MODE_NAMES = {0: "INIT", 1: "CALIB", 2: "FLIGHT", 3: "PARKING",
              4: "LOG", 5: "LANDING", 6: "FLIP"}
DEG_PER_STICK = 36.0   # スティック 1.0 で 36 度（実測で確認済み）
ALT_AUTO = 4


class PID:
    """位置のずれから傾ける角度を出す。微分は速度から直接作る（雑音に強い）"""

    def __init__(self, kp, ki, kd, i_limit=5.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i = 0.0
        self.i_limit = i_limit

    def reset(self):
        self.i = 0.0

    def update(self, err, vel, dt):
        self.i = float(np.clip(self.i + err * dt, -self.i_limit, self.i_limit))
        return self.kp * err + self.ki * self.i - self.kd * vel


def to_command(ax_deg, ay_deg, head_deg, max_tilt, flip_roll=False, flip_pitch=False):
    """世界座標で「+x へ ax 度、+y へ ay 度 傾けたい」を、機体から見た指令に直す

    機体は自分の向き（head）を向いているので、世界座標の要求を機体の前後左右へ回す。
    戻り値: (aileron, elevator)  どちらも -1〜1
    """
    h = np.radians(head_deg)
    right_deg = ax_deg * np.cos(h) + ay_deg * np.sin(h)
    fwd_deg = -ax_deg * np.sin(h) + ay_deg * np.cos(h)
    right_deg = float(np.clip(right_deg, -max_tilt, max_tilt))
    fwd_deg = float(np.clip(fwd_deg, -max_tilt, max_tilt))
    ail = right_deg / DEG_PER_STICK * (-1 if flip_roll else 1)
    ele = fwd_deg / DEG_PER_STICK * (-1 if flip_pitch else 1)
    return ail, ele


class Tracker:
    """OAK-D でマーカーを追い、床基準の位置と向きを返す"""

    def __init__(self, args):
        self.width, self.height = RESOLUTIONS[args.res]
        self.detector = make_detector()
        self.size_m = args.marker_size / 100.0
        self.obj = marker_object_points(self.size_m)
        self.marker_id = args.marker_id
        self.prev_R = None

        calib = json_load(args.calib)
        self.R_world = np.array(calib["R_ref"]).T
        self.t_ref = np.array(calib["t_ref"])

        self.device = dai.Device(build_pipeline(self.width, self.height, args.fps,
                                                args.exposure, args.iso))
        c = self.device.readCalibration()
        self.K = np.array(c.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, self.width, self.height))
        self.dist = np.array(c.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))[:8]
        self.queue = self.device.getOutputQueue("video", 1, blocking=False)
        self.usb = str(self.device.getUsbSpeed())

    def read(self):
        """(見つかったか, 位置[m], 向き[deg], 遅延[ms], 画像)"""
        pkt = latest(self.queue)
        latency = (dai.Clock.now() - pkt.getTimestamp()).total_seconds() * 1000.0
        gray = pkt.getFrame()[:self.height, :]
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            for c, mid in zip(corners, ids.ravel()):
                if int(mid) != self.marker_id:
                    continue
                rvec, tvec, _ = solve_pose(self.obj, c[0], self.K, self.dist, self.prev_R)
                if rvec is None:
                    continue
                R, _ = cv2.Rodrigues(rvec)
                self.prev_R = R
                pos = self.R_world @ (tvec.reshape(3) - self.t_ref)
                Rw = self.R_world @ R
                head = np.degrees(np.arctan2(Rw[1, 0], Rw[0, 0]))
                return True, pos, head, latency, gray
        return False, None, None, latency, gray

    def close(self):
        self.device.close()


def json_load(path):
    import json
    p = pathlib.Path(path)
    if not p.exists():
        sys.exit(f"座標系のファイルがありません: {p}\n"
                 f"先に  python3 oakd/track_world.py --calibrate  を実行してください")
    return json.loads(p.read_text())


def find_bridge_port():
    ports = sorted(glob.glob("/dev/cu.usbserial-*") + glob.glob("/dev/cu.wchusbserial*"))
    if not ports:
        sys.exit("M5GO が見つかりません。--bridge-port で指定してください")
    return ports[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="指令を計算するが送らない（安全な確認用）")
    ap.add_argument("--bridge-port", default=None)
    ap.add_argument("--calib", default=str(pathlib.Path(__file__).resolve().parent.parent
                                           / "oakd" / "world_calib.json"))
    ap.add_argument("--marker-id", type=int, default=2)
    ap.add_argument("--marker-size", type=float, default=2.8, help="機体マーカーの一辺 [cm]")
    ap.add_argument("--res", choices=sorted(RESOLUTIONS), default="1080p")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--exposure", type=float, default=6.0)
    ap.add_argument("--iso", type=int, default=800)
    # 制御
    ap.add_argument("--kp", type=float, default=6.0, help="位置のずれ1mあたり何度傾けるか")
    ap.add_argument("--ki", type=float, default=0.5)
    ap.add_argument("--kd", type=float, default=3.0, help="速度1m/sあたり何度戻すか")
    ap.add_argument("--max-tilt", type=float, default=8.0, help="傾ける角度の上限 [deg]")
    ap.add_argument("--kz", type=float, default=1.5, help="高さのずれ1mあたりのスロットル量")
    ap.add_argument("--target-z", type=float, default=0.5, help="目標高度 [m]")
    # 安全
    ap.add_argument("--fence-xy", type=float, default=0.8, help="この範囲を出たら停止 [m]")
    ap.add_argument("--fence-z", type=float, default=1.2, help="この高さを超えたら停止 [m]")
    ap.add_argument("--lost-grace", type=float, default=0.5, help="見失ってから停止するまで [s]")
    ap.add_argument("--flip-roll", action="store_true", help="左右が逆に動くとき指定")
    ap.add_argument("--flip-pitch", action="store_true", help="前後が逆に動くとき指定")
    ap.add_argument("--gui", action="store_true", help="カメラ映像も表示する")
    ap.add_argument("--log", default=None)
    ap.add_argument("--seconds", type=float, default=None, help="この秒数で自動終了（動作確認用）")
    args = ap.parse_args()

    tracker = Tracker(args)
    print(f"カメラ USB {tracker.usb}  {tracker.width}x{tracker.height}@{args.fps}")

    ser = None
    if not args.dry_run:
        port = args.bridge_port or find_bridge_port()
        ser = serial.Serial(port, 115200, timeout=0)
        print(f"中継機: {port}")
    else:
        print("お試しモード: 指令は計算するだけで送りません")

    pid_x, pid_y = PID(args.kp, args.ki, args.kd), PID(args.kp, args.ki, args.kd)
    target = np.array([0.0, 0.0, args.target_z])
    flying = False          # 離陸指令を出したかどうか（PC側の認識）
    transmitting = True
    arm_pulse = False
    arm_count = 0
    stop_reason = ""

    pos_f = None
    vel = np.zeros(3)
    t_prev = None
    last_seen = time.time()
    telem = {}
    rx = ""

    log_file = writer = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        writer = csv.writer(log_file)
        writer.writerow(["t_s", "x", "y", "z", "head", "tx", "ty", "tz",
                         "ail", "ele", "thr", "seen", "lat_ms"])

    interactive = sys.stdin.isatty()
    old_term = termios.tcgetattr(sys.stdin) if interactive else None
    if interactive:
        tty.setcbreak(sys.stdin.fileno())
    print("スペース=離陸/着陸  wasd=目標移動  rf=目標高度  h=その場で止まる  z=停止  Ctrl-C=終了")
    t0 = time.time()
    last_draw = 0.0

    try:
        while True:
            seen, pos, head, latency, gray = tracker.read()
            now = time.time() - t0

            # ---- キー入力 ----
            while interactive and select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == " ":
                    arm_pulse = True
                    arm_count += 1
                    flying = not flying
                    pid_x.reset()
                    pid_y.reset()
                    transmitting = True
                    stop_reason = ""
                elif key == "w":
                    target[1] += 0.1
                elif key == "s":
                    target[1] -= 0.1
                elif key == "d":
                    target[0] += 0.1
                elif key == "a":
                    target[0] -= 0.1
                elif key == "r":
                    target[2] += 0.1
                elif key == "f":
                    target[2] -= 0.1
                elif key == "h" and pos_f is not None:
                    target = pos_f.copy()
                elif key == "z":
                    transmitting = False
                    stop_reason = "手動停止"
                elif key == "\x03":
                    raise KeyboardInterrupt

            # ---- 位置の更新（見失ったときは前の値を保つ） ----
            if seen:
                last_seen = time.time()
                dt = (now - t_prev) if t_prev is not None else 0.0
                if pos_f is None or dt <= 0 or dt > 0.5:
                    pos_f, vel = pos, np.zeros(3)
                else:
                    a = 0.5
                    new = a * pos + (1 - a) * pos_f
                    vel = 0.5 * vel + 0.5 * (new - pos_f) / dt
                    pos_f = new
                t_prev = now

            # ---- 安全の判定 ----
            lost = (time.time() - last_seen) > args.lost_grace
            if lost and transmitting and flying:
                transmitting = False
                stop_reason = "マーカーを見失った"
            if pos_f is not None:
                if abs(pos_f[0]) > args.fence_xy or abs(pos_f[1]) > args.fence_xy:
                    if transmitting and flying:
                        transmitting = False
                        stop_reason = "範囲外(水平)"
                if pos_f[2] > args.fence_z:
                    if transmitting and flying:
                        transmitting = False
                        stop_reason = "範囲外(高さ)"

            # ---- 位置制御 ----
            # 飛んでいなくても計算はする（お試しモードで向きを確認できるように）。
            # 送信するのは離陸後だけ。
            ail = ele = thr = 0.0
            if pos_f is not None:
                if not flying:
                    pid_x.reset()   # 飛んでいない間は積分を溜めない
                    pid_y.reset()
                err = target - pos_f
                dt = max(1.0 / args.fps, 1e-3)
                # 世界座標での必要な傾き
                ax = pid_x.update(err[0], vel[0], dt)   # +x 方向へ動きたい量 [deg]
                ay = pid_y.update(err[1], vel[1], dt)
                # 機体の向きに合わせて、機体から見た前後左右の指令に直す
                ail, ele = to_command(ax, ay, head if head is not None else 0.0,
                                      args.max_tilt, args.flip_roll, args.flip_pitch)
                thr = float(np.clip(args.kz * (target[2] - pos_f[2]), -0.5, 0.5))

            # ---- 送信（離陸後だけスティックを送る） ----
            if ser is not None and transmitting:
                s_thr, s_ail, s_ele = (thr, ail, ele) if flying else (0.0, 0.0, 0.0)
                ser.write(f"C,{s_thr:.3f},{s_ail:.3f},{s_ele:.3f},0,"
                          f"{1 if arm_pulse else 0},0,0,{ALT_AUTO}\n".encode())
            arm_pulse = False

            # ---- 機体からの返信 ----
            if ser is not None:
                data = ser.read(4096).decode(errors="replace")
                if data:
                    rx += data
                    while "\n" in rx:
                        line, rx = rx.split("\n", 1)
                        if line.startswith("T,"):
                            p = line.strip().split(",")
                            if len(p) >= 13:
                                telem = {"v": float(p[5]), "alt": float(p[6]), "mode": int(p[7])}
                                telem["t_recv"] = time.time()

            # ---- 表示（1行に収める） ----
            if now - last_draw > 0.2:
                last_draw = now
                pp = pos_f if pos_f is not None else np.array([np.nan] * 3)
                mode_txt = MODE_NAMES.get(telem.get("mode"), "----") if telem else "通信なし"
                link = "OK" if telem and time.time() - telem.get("t_recv", 0) < 1.0 else "--"
                line = (f"{mode_txt:7s} 機体{link} "
                        f"{'FLY' if flying else 'IDLE'}{'TX' if transmitting else '停止'} "
                        f"arm{arm_count} | "
                        f"{'見' if seen else '×'} x{pp[0]:+.2f} y{pp[1]:+.2f} z{pp[2]:+.2f} | "
                        f"a{ail:+.2f} e{ele:+.2f} t{thr:+.2f} "
                        f"{('%.2fV' % telem['v']) if telem else ''} {stop_reason}")
                width = shutil.get_terminal_size((100, 24)).columns
                sys.stdout.write("\r" + line[:width - 1] + "\033[K")
                sys.stdout.flush()

            if writer:
                p = pos_f if pos_f is not None else np.array([np.nan] * 3)
                writer.writerow([f"{now:.3f}"] + [f"{v:.4f}" for v in p] +
                                [f"{head:.1f}" if head is not None else ""] +
                                [f"{v:.3f}" for v in target] +
                                [f"{ail:.3f}", f"{ele:.3f}", f"{thr:.3f}", int(seen), f"{latency:.0f}"])

            if args.seconds is not None and now > args.seconds:
                break

            if args.gui:
                view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                cv2.putText(view, f"{'SEEN' if seen else 'LOST'} {stop_reason}", (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0) if seen else (0, 0, 255), 2)
                cv2.imshow("hover", view)
                cv2.waitKey(1)
    except KeyboardInterrupt:
        pass
    finally:
        if interactive:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        if ser is not None:
            for _ in range(5):  # 中立を送ってから終了（機体は自動着陸に入る）
                ser.write(f"C,0,0,0,0,0,0,0,{ALT_AUTO}\n".encode())
                time.sleep(0.02)
            ser.close()
        tracker.close()
        if log_file:
            log_file.close()
        if args.gui:
            cv2.destroyAllWindows()
        print("\n終了しました。" + (f" 停止理由: {stop_reason}" if stop_reason else ""))


if __name__ == "__main__":
    main()
