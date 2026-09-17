#!/usr/bin/env python3
"""OAK-D で測った位置を使い、Tello EDU を離陸地点の真上に留める

    python3 tello/hover_oakd.py --seconds 20 --log ~/Desktop/tello_pos1.csv

事前に:
    * oakd/track_world.py --calibrate で座標系を作っておく（カメラを動かしたら作り直す）
    * Tello を映像の中央に置く（oakd/view.py の十字）。上面にマーカー id 1
    * Mac の Wi-Fi を Tello に、OAK-D を USB3 に繋ぐ

流れ:
    1. 離陸し、下向きToFで --height まで降りる
    2. 前に20cm動いて戻り、その移動をカメラで見て「機体の前方向」を世界座標で実測する
       （StampFly では向きの思い込みが -90度ずれていて暴走した。その対策）
    3. 離陸地点の真上へ戻るように、水平の速度指令を出し続ける
       高さは Tello 自身の高度維持に任せる

役割の分担:
    Tello は自分の下向きカメラで速度をほぼ0に保つ（単体で水平ずれ ±2cm 程度）。
    PC はその上に「目標とのずれ × ゲイン」を速度指令として足すだけ。

安全のための自動着陸:
    * 目標から --fence 以上離れた
    * マーカーを --lost-land 秒以上見失った（1秒未満の見失いは指令0で待つ）
    * Ctrl-C
"""

import argparse
import csv
import json
import pathlib
import sys
import threading
import time

import cv2
import depthai as dai
import numpy as np
from djitellopy import Tello

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "oakd"))
from marker_common import build_pipeline, latest, make_detector, wrap_deg  # noqa: E402
from track_world import detect_poses  # noqa: E402


class Tracker(threading.Thread):
    """OAK-D でマーカーを追い、最新の世界座標を持っておく"""

    def __init__(self, calib_path, marker_id, marker_cm, exposure_ms, iso):
        super().__init__(daemon=True)
        data = json.loads(pathlib.Path(calib_path).read_text())
        self.R_ref = np.array(data["R_ref"])
        self.t_ref = np.array(data["t_ref"])
        self.calib_created = data["created"]
        self.marker_id = marker_id
        self.sizes = {marker_id: marker_cm / 100.0}
        self.exposure_ms = exposure_ms
        self.iso = iso
        self.sample = None        # (撮影時刻 time.time() 基準, pos[3], heading_deg)
        self.fps = 0.0
        self.error = None
        self.ready = threading.Event()
        self.stop = False

    def run(self):
        try:
            self._loop()
        except Exception as e:     # メイン側で気づけるように保存する
            self.error = e
            self.ready.set()

    def _loop(self):
        width, height = 1920, 1080
        detector = make_detector()
        with dai.Device(build_pipeline(width, height, 30, self.exposure_ms, self.iso)) as device:
            calib = device.readCalibration()
            K = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height))
            dist = np.array(calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))[:8]
            self.usb = str(device.getUsbSpeed())
            q = device.getOutputQueue("video", 1, blocking=False)
            R_world = self.R_ref.T
            prev_R = {}
            n, t_fps = 0, time.time()
            self.ready.set()
            while not self.stop:
                pkt = latest(q)
                latency = (dai.Clock.now() - pkt.getTimestamp()).total_seconds()
                gray = pkt.getFrame()[:height, :]
                poses = detect_poses(detector, gray, K, dist, self.sizes, prev_R)
                n += 1
                if time.time() - t_fps > 1.0:
                    self.fps = n / (time.time() - t_fps)
                    n, t_fps = 0, time.time()
                if self.marker_id not in poses:
                    continue
                R_d, t_d, _, _ = poses[self.marker_id]
                pos = R_world @ (t_d - self.t_ref)
                R_dw = R_world @ R_d
                head = float(np.degrees(np.arctan2(R_dw[1, 0], R_dw[0, 0])))
                self.sample = (time.time() - latency, pos, head)


def wait_sample(tracker, max_age, timeout):
    """新しい位置が来るまで待つ"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = tracker.sample
        if s is not None and time.time() - s[0] < max_age:
            return s
        time.sleep(0.02)
    return None


def average_position(tracker, seconds):
    """静止中の位置と向きを平均する"""
    pts, heads = [], []
    last = None
    t0 = time.time()
    while time.time() - t0 < seconds:
        s = tracker.sample
        if s is not None and s is not last and time.time() - s[0] < 0.2:
            pts.append(s[1])
            heads.append(s[2])
            last = s
        time.sleep(0.02)
    if len(pts) < 3:
        return None, None
    h = np.radians(heads)
    return np.mean(pts, axis=0), float(np.degrees(np.arctan2(np.sin(h).mean(), np.cos(h).mean())))


def to_body(vx, vy, fwd_deg):
    """世界座標の速度 → Tello の (右, 前)

    前ベクトル = (cos h, sin h)、右ベクトル = (sin h, -cos h)（z 上向きの右手系）
    """
    h = np.radians(fwd_deg)
    fwd = vx * np.cos(h) + vy * np.sin(h)
    right = vx * np.sin(h) - vy * np.cos(h)
    return right, fwd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20, help="位置制御する時間 [s]")
    ap.add_argument("--height", type=int, default=40, help="ホバリング高さ（下向きToF）[cm]")
    ap.add_argument("--kp", type=float, default=1.0,
                    help="ずれに対する速度指令のゲイン。1.0 なら 10cm ずれで指令10（≒10cm/s）")
    ap.add_argument("--max-cmd", type=int, default=25, help="水平指令の上限（rc、最大100）")
    ap.add_argument("--deadband", type=float, default=0.02, help="これ以内のずれは直さない [m]")
    ap.add_argument("--fence", type=float, default=0.5, help="目標からこれ以上離れたら着陸 [m]")
    ap.add_argument("--lost-land", type=float, default=2.0, help="これ以上見失ったら着陸 [s]")
    ap.add_argument("--marker-id", type=int, default=1)
    ap.add_argument("--marker-cm", type=float, default=4.0)
    ap.add_argument("--exposure", type=float, default=6.0, help="手動露出 [ms]")
    ap.add_argument("--iso", type=int, default=800)
    ap.add_argument("--calib", default=str(ROOT / "oakd" / "world_calib.json"))
    ap.add_argument("--log", default=None)
    args = ap.parse_args()

    # --- カメラ ---
    tracker = Tracker(args.calib, args.marker_id, args.marker_cm, args.exposure, args.iso)
    tracker.start()
    tracker.ready.wait(20)
    if tracker.error:
        sys.exit(f"OAK-D を開けません: {tracker.error}")
    print(f"OAK-D: USB {tracker.usb}  座標系 {tracker.calib_created}")
    if tracker.usb.endswith("HIGH"):
        print("警告: USB 2 接続。遅延が大きいので USB 3 に挿し直すこと")

    s = wait_sample(tracker, 0.3, 5)
    if s is None:
        sys.exit(f"マーカー id {args.marker_id} が見えません。Tello の置き場所を確認してください")
    ground, _ = average_position(tracker, 1.0)
    target = ground[:2].copy()
    print(f"離陸地点 x{ground[0]:+.3f} y{ground[1]:+.3f} z{ground[2]:+.3f} m → ここを目標にします")

    # --- Tello ---
    tello = Tello()
    print("Tello に接続中…")
    tello.connect()
    bat = tello.get_battery()
    print(f"バッテリー {bat}%")
    if bat < 30:
        sys.exit("残量が少ないので中止します")

    writer = log_file = None
    if args.log:
        log_file = open(pathlib.Path(args.log).expanduser(), "w", newline="")
        writer = csv.writer(log_file)
        writer.writerow(["t_s", "phase", "x_m", "y_m", "z_m", "head_deg", "age_ms",
                         "err_x_m", "err_y_m", "cmd_right", "cmd_fwd", "tof_cm", "bat"])

    t_start = time.time()
    reason = "時間終了"
    stats = []
    try:
        # 1. 離陸して高さを合わせる
        print("離陸します（Ctrl-C で着陸）")
        tello.takeoff()
        t_adj = time.time()
        while time.time() - t_adj < 8:
            tof = tello.get_distance_tof()
            if 0 < tof <= 500:
                err = args.height - tof
                if abs(err) <= 5:
                    break
                tello.send_rc_control(0, 0, int(np.clip(err, -30, 30)), 0)
            time.sleep(0.05)
        tello.send_rc_control(0, 0, 0, 0)
        time.sleep(1.0)

        # 2. 前方向を実測する
        print("前に20cm動いて戻り、機体の向きを測ります")
        p0, head0 = average_position(tracker, 1.0)
        if p0 is None:
            raise RuntimeError("浮いた状態でマーカーが見えません（カメラの画面外に出ていないか確認）")
        tello.move_forward(20)
        time.sleep(0.5)
        p1, head1 = average_position(tracker, 1.0)
        if p1 is None:
            raise RuntimeError("前進後にマーカーが見えません")
        d = p1[:2] - p0[:2]
        dist = float(np.linalg.norm(d))
        fwd_world = float(np.degrees(np.arctan2(d[1], d[0])))
        offset = wrap_deg(fwd_world - head1)   # マーカーの向き → 機体の前方向
        print(f"  移動 {dist * 100:.1f}cm  前方向 {fwd_world:+.1f}度  "
              f"マーカーとの差 {offset:+.1f}度")
        if not 0.10 <= dist <= 0.35:
            raise RuntimeError(f"移動量 {dist * 100:.1f}cm が 20cm からかけ離れています。"
                               f"マーカーの大きさ(--marker-cm)を確認してください")
        tello.move_back(20)
        time.sleep(0.5)

        # 3. 位置制御
        print(f"位置制御を {args.seconds:.0f} 秒行います。目標 x{target[0]:+.3f} y{target[1]:+.3f}")
        t0 = time.time()
        last_ok = time.time()
        next_print = 0.0
        while time.time() - t0 < args.seconds:
            now = time.time()
            s = tracker.sample
            age = now - s[0] if s is not None else 99.0
            cmd_r = cmd_f = 0
            err = np.array([np.nan, np.nan])
            if age < 0.3:
                last_ok = now
                pos, head = s[1], s[2]
                err = target - pos[:2]
                dist_err = float(np.linalg.norm(err))
                if dist_err > args.fence:
                    reason = f"目標から {dist_err * 100:.0f}cm 離れた"
                    break
                if dist_err > args.deadband:
                    v = err * args.kp * 100          # rc 値（≒cm/s）
                    right, fwd = to_body(v[0], v[1], head + offset)
                    cmd_r = int(np.clip(right, -args.max_cmd, args.max_cmd))
                    cmd_f = int(np.clip(fwd, -args.max_cmd, args.max_cmd))
                stats.append(dist_err)
            elif now - last_ok > args.lost_land:
                reason = f"マーカーを {now - last_ok:.1f} 秒見失った"
                break
            tello.send_rc_control(cmd_r, cmd_f, 0, 0)

            if writer:
                st = tello.get_current_state()
                p = s[1] if s is not None else [np.nan] * 3
                writer.writerow([f"{now - t_start:.2f}", "ctrl", *(f"{v:.4f}" for v in p),
                                 f"{s[2]:.1f}" if s is not None else "", f"{age * 1000:.0f}",
                                 f"{err[0]:.4f}", f"{err[1]:.4f}", cmd_r, cmd_f,
                                 st.get("tof", 0), st.get("bat", 0)])
            if now - t0 > next_print:
                next_print = now - t0 + 0.5
                if age < 0.3:
                    sys.stdout.write(f"\r{now - t0:5.1f}s  ずれ x{err[0] * 100:+5.1f} y{err[1] * 100:+5.1f}cm"
                                     f"  指令 右{cmd_r:+3d} 前{cmd_f:+3d}  カメラ{tracker.fps:3.0f}fps   ")
                else:
                    sys.stdout.write(f"\r{now - t0:5.1f}s  マーカーを見失い中（指令0）"
                                     f"                          ")
                sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        reason = "Ctrl-C"
    except Exception as e:
        reason = f"エラー: {e}"
    finally:
        print(f"\n着陸します（{reason}）")
        try:
            tello.send_rc_control(0, 0, 0, 0)
            tello.land()
        except Exception as e:
            print(f"着陸の指令に失敗: {e}")
        tracker.stop = True
        if log_file:
            log_file.close()
        tello.end()

    if stats:
        a = np.array(stats) * 100
        print(f"\n目標からのずれ: 平均 {a.mean():.1f}cm  最大 {a.max():.1f}cm  "
              f"（{len(a)}サンプル）")


if __name__ == "__main__":
    main()
