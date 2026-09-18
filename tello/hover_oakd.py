#!/usr/bin/env python3
"""OAK-D で測った位置を使い、Tello EDU を決めた場所に留める／決めた点を順に通す

    python3 tello/hover_oakd.py --seconds 20 --log ~/Desktop/tello_pos1.csv
    python3 tello/hover_oakd.py --waypoints "0,0 0.3,0 0.3,0.3 0,0.3 0,0" --dwell 12

事前に:
    * oakd/track_world.py --calibrate で座標系を作っておく（カメラを動かしたら作り直す）
    * Tello を映像の中央に置く（oakd/view.py の十字）。上面にマーカー id 1
    * Mac の Wi-Fi を Tello に、OAK-D を USB3 に繋ぐ

流れ:
    1. 離陸し、下向きToFで --height まで降りる
    2. 少し（8cm）前に動き、その移動をカメラで見て「機体の前方向」を世界座標で実測する
       （StampFly では向きの思い込みが -90度ずれていて暴走した。その対策）
    3. 目標へ戻るように、水平の速度指令を出し続ける（P+I 制御）
       高さは Tello 自身の高度維持に任せる
       --waypoints を付けると、目標を --dwell 秒ごとに次の点へ移していく
       （離陸前に、通る点すべてが浮いたときも画面に入るかを確かめる）

役割の分担:
    Tello は自分の下向きカメラで速度をほぼ0に保つ（単体で水平ずれ ±2cm 程度）。
    PC はその上に「目標とのずれ × ゲイン」を速度指令として足すだけ。

安全のための自動着陸:
    * 目標から --fence 以上離れた
    * マーカーを --lost-land 秒以上見失った（見失っている間は指令0で Tello 自身のホバリングに任せる）
    * OAK-D が落ちて --camera-down-land 秒以内に復帰しない（落ちたら自動で開き直す）
    * Ctrl-C
"""

import argparse
import atexit
import csv
import json
import logging
import pathlib
import sys
import threading
import time

import cv2
import depthai as dai
import numpy as np
from djitellopy import Tello

Tello.LOGGER.setLevel(logging.WARNING)   # 送信のたびに出る INFO を止める

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
        self.pixel = None         # 映像内のマーカー中心（0〜1）
        self.K = None
        self.last_frame = 0.0     # 最後に映像が届いた時刻（マーカーの有無に関係なく）
        self.error = None
        self.crashes = 0          # 途中で落ちて開き直した回数
        self.down = False         # 開き直している最中
        self.ready = threading.Event()
        self.stop = False

    def run(self):
        # OAK-D は飛行中に落ちることがある（device has crashed）。落ちたら開き直す。
        # その間 Tello は自分でホバリングしているので、数秒の中断なら飛行を続けられる
        while not self.stop:
            try:
                self._loop()
            except Exception as e:
                self.error = e
                if not self.ready.is_set():   # 最初から開けないときはメイン側で終了する
                    self.ready.set()
                    return
                self.crashes += 1
                self.down = True
                time.sleep(0.5)

    def _loop(self):
        width, height = 1920, 1080
        detector = make_detector()
        with dai.Device(build_pipeline(width, height, 30, self.exposure_ms, self.iso)) as device:
            calib = device.readCalibration()
            K = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height))
            dist = np.array(calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))[:8]
            self.K = K
            self.usb = str(device.getUsbSpeed())
            q = device.getOutputQueue("video", 1, blocking=False)
            R_world = self.R_ref.T
            prev_R = {}
            n, t_fps = 0, time.time()
            self.down = False
            self.ready.set()
            while not self.stop:
                pkt = latest(q)
                latency = (dai.Clock.now() - pkt.getTimestamp()).total_seconds()
                self.last_frame = time.time()
                gray = pkt.getFrame()[:height, :]
                poses = detect_poses(detector, gray, K, dist, self.sizes, prev_R)
                n += 1
                if time.time() - t_fps > 1.0:
                    self.fps = n / (time.time() - t_fps)
                    n, t_fps = 0, time.time()
                if self.marker_id not in poses:
                    continue
                R_d, t_d, _, corners = poses[self.marker_id]
                self.pixel = corners.reshape(-1, 2).mean(axis=0) / (width, height)  # 0〜1
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


def image_ratio(tracker, p_world):
    """世界座標の点が映像のどこに写るか（横, 縦、0〜1）"""
    pc = tracker.R_ref @ np.asarray(p_world) + tracker.t_ref
    x = tracker.K @ pc
    return x[0] / x[2] / 1920, x[1] / x[2] / 1080


def probe_direction(tello, tracker, rc_fwd, dist_m, timeout, settle=2.0):
    """rc_fwd の向きへ押して、実際に動いた向き（世界座標 deg）とマーカーの向きを返す

    短い移動では機体自身のふらつき（数cm）が方向の誤差になり、数十度ずれるので、
    押しながら経路の点を集め、直線をあてはめて向きを出す。

    押し始めの数cm は、前の動きの勢いが残っていて向きが当てにならない
    （前へ押したあとすぐ後ろへ押すと、まだ前へ流れている分が混ざる。実測で
    後ろへ 4.3cm しか動かず、向きが 173度 ひっくり返った）。そのため
    - 押す前に settle 秒、指令0 で止まるのを待つ
    - 始点から 3cm 離れるまでの点は使わない
    """
    # 止まるのを待ちながら、機体が勝手に流される速さを測る。
    # 15cm 進む間に横へ3cm 流されるだけで向きが11度ずれるので、これを差し引く
    t_settle = time.time()
    drift_pts, last = [], None
    while time.time() - t_settle < settle:
        tello.send_rc_control(0, 0, 0, 0)     # 止まるのを待つ（無指令だと15秒で着陸する）
        s = tracker.sample
        if s is not None and time.time() - s[0] < 0.2 and s is not last:
            last = s
            drift_pts.append((time.time(), s[1][:2].copy()))
        time.sleep(0.05)
    drift = np.zeros(2)
    if len(drift_pts) >= 8 and drift_pts[-1][0] - drift_pts[0][0] > 0.8:
        tt = np.array([q[0] for q in drift_pts]); tt -= tt.mean()
        pp = np.array([q[1] for q in drift_pts])
        drift = (tt @ (pp - pp.mean(axis=0))) / (tt @ tt)      # 最小二乗の傾き [m/s]

    if wait_sample(tracker, 0.3, 5.0) is None:      # 落ち着くのを待つ（離陸直後は流れる）
        where = (f"最後に見えたのは 横 {tracker.pixel[0] * 100:.0f}% 縦 {tracker.pixel[1] * 100:.0f}%"
                 if tracker.pixel is not None else "一度も見えていません")
        raise RuntimeError(f"浮いた状態でマーカーが見えません。{where}\n"
                           f"  離陸で約80cmまで上がったときに画面から出た可能性があります")
    p0, _ = average_position(tracker, 0.7)
    if p0 is None:
        raise RuntimeError("浮いた状態でマーカーが見えません（カメラの画面外に出ていないか確認）")
    pts, heads, last = [], [], None
    far = 0.0
    t_push = time.time()
    while time.time() - t_push < timeout:
        tello.send_rc_control(0, rc_fwd, 0, 0)
        s = tracker.sample
        if s is not None and time.time() - s[0] < 0.2 and s is not last:
            last = s
            q = s[1][:2] - drift * (time.time() - t_push)     # 流される分を取り除く
            d = float(np.linalg.norm(q - p0[:2]))
            far = max(far, d)
            if d >= 0.03:                     # 勢いが残っている区間は使わない
                pts.append(q)
                heads.append(s[2])
            if d >= dist_m:
                break
        time.sleep(0.05)
    tello.send_rc_control(0, 0, 0, 0)
    if last is None:
        raise RuntimeError("押している間にマーカーを見失いました")
    if far < dist_m * 0.6 or len(pts) < 5:
        raise RuntimeError(
            f"{far * 100:.1f}cm しか動けませんでした（目標 {dist_m * 100:.0f}cm）。"
            f"向きが決められないので中止します。\n"
            f"  --probe-cmd を大きくするか、--probe-dist を小さくしてください")
    pts = np.array(pts)
    net = pts[-1] - p0[:2]
    dist = float(np.linalg.norm(net))
    # 経路の点に直線をあてはめる（主成分）。向きは実際に進んだ側に合わせる
    v = np.linalg.svd(pts - pts.mean(axis=0))[2][0]
    if v @ net < 0:
        v = -v
    deg = float(np.degrees(np.arctan2(v[1], v[0])))
    h = np.radians(heads)
    head = float(np.degrees(np.arctan2(np.sin(h).mean(), np.cos(h).mean())))
    return deg, head, dist, float(np.linalg.norm(drift)) * 100


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
    ap.add_argument("--waypoints", default=None,
                    help="通過する点を離陸地点からのずれ [m] で並べる。"
                         "例 \"0,0 0.3,0 0.3,0.3 0,0.3 0,0\" で一辺30cmの正方形。"
                         "x は基準マーカーの矢印の向き。省略すると離陸地点に留まる")
    ap.add_argument("--dwell", type=float, default=10.0,
                    help="1つの点に留まる時間 [s]。整定に10秒以上かかるので短くしすぎない")
    ap.add_argument("--height", type=int, default=40, help="ホバリング高さ（下向きToF）[cm]")
    ap.add_argument("--kp", type=float, default=1.0,
                    help="ずれに対する速度指令のゲイン。1.0 なら 10cm ずれで指令10（≒10cm/s）")
    ap.add_argument("--ki", type=float, default=0.35,
                    help="積分ゲイン。同じ向きのずれが残り続けたときに指令を足していく。"
                         "0 で比例のみ（定常偏差が -7cm 残った）")
    ap.add_argument("--kd", type=float, default=0.6,
                    help="微分ゲイン（ブレーキ）。機体の速度[cm/s]にこれを掛けた分を指令から引く。"
                         "0 だと目標を通り過ぎて振動が止まらない")
    ap.add_argument("--move-speed", type=float, default=0.15,
                    help="目標を次の点へ動かす速さ [m/s]。一瞬で飛ばすと行き過ぎる")
    ap.add_argument("--i-band", type=float, default=0.10,
                    help="積分を溜めるずれの範囲 [m]。これより離れている間は溜めるのを"
                         "やめる（すでに溜めた分は出し続ける）")
    ap.add_argument("--max-i", type=float, default=18.0,
                    help="積分が出せる指令の上限（rc）。溜まりすぎての行き過ぎを防ぐ。"
                         "10 では外乱の強い日に押し負けて、到達後 毎秒1cm 離れていった")
    ap.add_argument("--max-cmd", type=int, default=25, help="水平指令の上限（rc、最大100）")
    ap.add_argument("--deadband", type=float, default=0.02, help="これ以内のずれは直さない [m]")
    ap.add_argument("--fence", type=float, default=0.5, help="目標からこれ以上離れたら着陸 [m]")
    ap.add_argument("--camera-down-land", type=float, default=8.0,
                    help="カメラが落ちて開き直している間、これ以上続いたら着陸 [s]")
    ap.add_argument("--lost-land", type=float, default=3.0, help="これ以上見失ったら着陸 [s]")
    ap.add_argument("--probe-cmd", type=int, default=20, help="向きを測るときの前進指令（rc）")
    ap.add_argument("--probe-dist", type=float, default=0.20,
                    help="向きを測るために動かす距離 [m]。短いとふらつきで向きがずれる")
    ap.add_argument("--probe-agree", type=float, default=20.0,
                    help="前後2回の測定に許す食い違い [deg]。超えたら飛ばさない")
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
    if tracker.error and not tracker.is_alive():
        sys.exit(f"OAK-D を開けません: {tracker.error}")
    def close_camera():            # 途中で終わってもカメラを正しく閉じる（閉じないと OAK-D が異常終了する）
        tracker.stop = True
        tracker.join(timeout=3)
    atexit.register(close_camera)
    print(f"OAK-D: USB {tracker.usb}  座標系 {tracker.calib_created}")
    if tracker.usb.endswith("HIGH"):
        print("警告: USB 2 接続。遅延が大きいので USB 3 に挿し直すこと")

    s = wait_sample(tracker, 0.3, 5)
    if s is None:
        sys.exit(f"マーカー id {args.marker_id} が見えません。Tello の置き場所を確認してください")
    ground, _ = average_position(tracker, 1.0)
    # カメラは真下を向いていないので、浮くと映像の中で位置がずれる。
    # 地上ではなく「ホバリング高さでどこに写るか」で置き場所を判定する
    z_hover = ground[2] + args.height / 100.0

    # 通る点を決める（離陸地点からのずれ [m]）
    if args.waypoints:
        try:
            offsets = [np.array([float(a), float(b)]) for a, b in
                       (w.split(",") for w in args.waypoints.split())]
        except ValueError:
            sys.exit('--waypoints の書き方が違います。例: "0,0 0.3,0 0.3,0.3 0,0.3 0,0"')
    else:
        offsets = [np.zeros(2)]
    targets = [ground[:2] + d for d in offsets]

    # 浮いたとき画面から出ないか、通る点すべてで確かめる（出ると見失って着陸する）
    print(f"ホバリング時の映像内の位置（予測、中央は 50%）")
    for i, (tg, d) in enumerate(zip(targets, offsets)):
        u, v = image_ratio(tracker, [tg[0], tg[1], z_hover])
        ng = not (0.3 <= u <= 0.7 and 0.3 <= v <= 0.7)
        print(f"  {i + 1}. ずれ x{d[0] * 100:+5.0f} y{d[1] * 100:+5.0f}cm → "
              f"横 {u * 100:3.0f}%  縦 {v * 100:3.0f}%" + ("   ← 画面の外へ出ます" if ng else ""))
    # Tello は離陸で必ず約80cmまで上がる（高さは指定できない）。そこでも入るか確かめる
    z_takeoff = ground[2] + 0.80
    u, v = image_ratio(tracker, [ground[0], ground[1], z_takeoff])
    print(f"  離陸直後（約80cm）→ 横 {u * 100:3.0f}%  縦 {v * 100:3.0f}%"
          + ("   ← 画面の外へ出ます" if not (0.3 <= u <= 0.7 and 0.3 <= v <= 0.7) else ""))
    takeoff_bad = not (0.3 <= u <= 0.7 and 0.3 <= v <= 0.7)

    # 向きの測定で前後に --probe-dist 動くぶんも、画面に入るか確かめる
    d = args.probe_dist
    probe_pts = [ground[:2] + np.array(v) for v in ((d, 0), (-d, 0), (0, d), (0, -d))]
    for tg in probe_pts:
        u, v = image_ratio(tracker, [tg[0], tg[1], z_hover])
        if not (0.3 <= u <= 0.7 and 0.3 <= v <= 0.7):
            print(f"  向きの測定（離陸地点から {d * 100:.0f}cm）→ 横 {u * 100:3.0f}%  "
                  f"縦 {v * 100:3.0f}%   ← 画面の外へ出ます")

    bad = [i for i, tg in enumerate(list(targets) + probe_pts)
           if not all(0.3 <= r <= 0.7 for r in image_ratio(tracker, [tg[0], tg[1], z_hover]))]
    if takeoff_bad and not bad:
        sys.exit("離陸直後（約80cm）に画面の外へ出ます。Tello を画面の中央寄りに置いてください")
    if bad:
        # 「離陸地点を中央に」では足りない。離陸地点が中央でも遠い端だけ
        # はみ出すことがあり、そのときは「動かさなくてよい」と答えてしまう。
        # 通る点すべてが最も余裕をもって収まる置き場所を探す
        def margin(g):
            m = 1.0
            for tg in [g + d for d in offsets] + [g + np.array(v) for v in
                                                  ((d, 0), (-d, 0), (0, d), (0, -d))]:
                for r in image_ratio(tracker, [tg[0], tg[1], z_hover]):
                    m = min(m, r - 0.3, 0.7 - r)
            return m
        best = max(((margin(ground[:2] + np.array([ax, ay])), ax, ay)
                    for ax in np.arange(-0.8, 0.801, 0.01)
                    for ay in np.arange(-0.8, 0.801, 0.01)), key=lambda q: q[0])
        m, ax, ay = best
        if m > 0:
            sys.exit(f"{len(bad)}個の点が画面の外です。\n"
                     f"  Tello を x 方向に {ax * 100:+.0f}cm、y 方向に {ay * 100:+.0f}cm "
                     f"動かしてください（基準マーカーの矢印の向きが +）")
        sys.exit(f"{len(bad)}個の点が画面の外です。どこに置いても収まりません。\n"
                 f"  経路を小さくするか、--probe-dist を小さくするか、"
                 f"カメラを遠ざけてください（python3 tello/place.py で上限がわかります）")
    total = args.dwell * len(targets) if args.waypoints else args.seconds
    print(f"離陸地点 x{ground[0]:+.3f} y{ground[1]:+.3f} z{ground[2]:+.3f} m"
          f"  通る点 {len(targets)}個  所要 {total:.0f}秒")

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
        writer.writerow(["t_s", "phase", "wp", "tgt_x_m", "tgt_y_m",
                         "x_m", "y_m", "z_m", "head_deg", "age_ms", "i_cmd",
                         "err_x_m", "err_y_m", "cmd_right", "cmd_fwd", "tof_cm", "bat", "cam_fps"])

    t_start = time.time()
    reason = "時間終了"
    stats = []
    try:
        # 1. 離陸して高さを合わせる
        print("離陸します（Ctrl-C で着陸）")
        tello.takeoff()
        t_adj = time.time()
        seen = total = 0            # 降りている間、マーカーが見えていた割合
        while time.time() - t_adj < 12:
            total += 1
            sm = tracker.sample
            if sm is not None and time.time() - sm[0] < 0.3:
                seen += 1
            tof = tello.get_distance_tof()
            if 0 < tof <= 500:
                err = args.height - tof
                if abs(err) <= 3:
                    break
                # 誤差に比例させるだけでは、近づくと指令が弱すぎて止まってしまう
                speed = int(np.clip(err * 1.5, -30, 30))
                if abs(speed) < 8:
                    speed = 8 if speed > 0 else -8
                tello.send_rc_control(0, 0, speed, 0)
            time.sleep(0.05)
        tello.send_rc_control(0, 0, 0, 0)
        if total:
            note = ""
            if seen < total * 0.8 and tracker.pixel is not None:
                note = (f"  最後に見えたのは 横 {tracker.pixel[0] * 100:.0f}% "
                        f"縦 {tracker.pixel[1] * 100:.0f}%")
            print(f"降下中にマーカーが見えていた割合 {seen / total * 100:.0f}%{note}")
        time.sleep(1.0)

        # 2. 前方向を実測する（前と後ろの2回。食い違ったら飛ばさない）
        #    「マーカーとの差」は貼り方で決まる固定値なので、2回の測定は一致するはず。
        #    1回だけだと、ふらつきで数十度ずれても気づけず、機体が目標のまわりを
        #    回り続ける（実測: 半径20cm・周期7秒の円を描いて画面外へ出た）
        print("前後に動かして、機体の向きを測ります")
        fwd_a, head_a, dist_a, dr_a = probe_direction(tello, tracker, args.probe_cmd,
                                                      args.probe_dist, 4.0, settle=2.0)
        off_a = wrap_deg(fwd_a - head_a)
        back, head_b, dist_b, dr_b = probe_direction(tello, tracker, -args.probe_cmd,
                                                     args.probe_dist, 4.0, settle=2.0)
        off_b = wrap_deg(back + 180.0 - head_b)
        print(f"  前へ {dist_a * 100:.1f}cm → マーカーとの差 {off_a:+.1f}度（流れ {dr_a:.1f}cm/s）")
        print(f"  後へ {dist_b * 100:.1f}cm → マーカーとの差 {off_b:+.1f}度（流れ {dr_b:.1f}cm/s）")
        gap = abs(wrap_deg(off_a - off_b))
        if gap > args.probe_agree:
            raise RuntimeError(
                f"2回の測定が {gap:.0f}度 食い違っています（許容 {args.probe_agree:.0f}度）。"
                f"向きを間違えたまま飛ばすと目標のまわりを回り続けるので中止します。\n"
                f"  マーカーが傾いて読めていないか、押している間に流されていないか確認してください")
        o = np.radians([off_a, off_b])
        offset = float(np.degrees(np.arctan2(np.sin(o).mean(), np.cos(o).mean())))
        print(f"  採用: マーカーとの差 {offset:+.1f}度（食い違い {gap:.0f}度）")
        time.sleep(0.5)

        # 3. 位置制御
        wp_time = args.dwell if args.waypoints else args.seconds
        print(f"位置制御を {wp_time * len(targets):.0f} 秒行います"
              f"（{len(targets)}個の点を各 {wp_time:.0f} 秒）")
        t0 = time.time()
        last_ok = time.time()
        next_print = 0.0
        integral = np.zeros(2)      # ずれの積み上げ（世界座標, m*s）
        t_prev = time.time()
        wp = -1
        target = targets[0].copy()      # なめらかに動かす、いまの目標
        goal = targets[0]               # 向かっている点
        recent = []                     # 速度を出すための (時刻, 位置)
        while time.time() - t0 < wp_time * len(targets):
            now = time.time()
            dt = now - t_prev
            t_prev = now
            # 目標の切り替え。積分はそのまま引き継ぐ（外乱を打ち消す分は次の点でも要る）
            i_wp = min(int((now - t0) / wp_time), len(targets) - 1)
            if i_wp != wp:
                wp = i_wp
                goal = targets[wp]
                if len(targets) > 1:
                    d = offsets[wp]
                    print(f"\n  {wp + 1}/{len(targets)} 点目へ "
                          f"（離陸地点から x{d[0] * 100:+.0f} y{d[1] * 100:+.0f}cm）")
            # 目標を goal へ --move-speed で近づける（一瞬で飛ばすと行き過ぎる）
            to_goal = goal - target
            step = args.move_speed * dt
            target = goal.copy() if np.linalg.norm(to_goal) <= step else target + to_goal / np.linalg.norm(to_goal) * step

            s = tracker.sample
            age = now - s[0] if s is not None else 99.0
            cam_down = tracker.down or now - tracker.last_frame > 1.0   # 映像そのものが止まっている
            cmd_r = cmd_f = 0
            i_size = 0.0
            err = np.array([np.nan, np.nan])
            if age < 0.3:
                last_ok = now
                pos, head = s[1], s[2]
                err = target - pos[:2]
                dist_err = float(np.linalg.norm(err))
                if dist_err > args.fence:
                    reason = f"目標から {dist_err * 100:.0f}cm 離れた"
                    break
                # 機体の速度［cm/s］（0.2秒前との差。1フレーム差では雑音が乗る）
                recent.append((now, pos[:2].copy()))
                while len(recent) > 2 and now - recent[0][0] > 0.2:
                    recent.pop(0)
                vel = ((pos[:2] - recent[0][1]) / (now - recent[0][0]) * 100
                       if now - recent[0][0] > 0.05 else np.zeros(2))

                if dist_err > args.deadband:
                    # 比例だけだと、機体を押し続ける外乱と釣り合う分のずれが残る
                    # （実測で x に -7cm）。同じ向きのずれを積み上げて、その分を足す。
                    # ただし移動中に溜めると行き過ぎるので、目標の近くでだけ効かせる
                    if args.ki > 0:
                        # 目標から離れている間は「溜めるのをやめる」だけにする。
                        # 効果ごと0にすると、離れ始めた瞬間に外乱を打ち消す力が消えて
                        # さらに離れる（実測: ずれ8cmのまま積分が10.7→5.7と減っていった）。
                        # 外乱は機体がどこにいても同じ向きなので、打ち消す分は出し続ける
                        if dist_err < args.i_band:
                            integral += err * dt
                        i_cmd = np.clip(integral * args.ki * 100, -args.max_i, args.max_i)
                        integral = i_cmd / (args.ki * 100)  # 上限で頭打ちにして溜め込みを防ぐ
                    else:
                        i_cmd = np.zeros(2)

                    # 速度の分を引く（ブレーキ）。これが無いと目標を通り過ぎて振動が続く
                    v = err * args.kp * 100 + i_cmd - vel * args.kd
                    right, fwd = to_body(v[0], v[1], head + offset)
                    cmd_r = int(np.clip(right, -args.max_cmd, args.max_cmd))
                    cmd_f = int(np.clip(fwd, -args.max_cmd, args.max_cmd))
                # 積分の大きさは、ずれが小さくて指令を出さないときも記録する
                # （そうしないとログ上で 0 に落ちて、消えたように見える）
                i_size = float(np.linalg.norm(integral) * args.ki * 100)
                stats.append(dist_err)
            elif now - last_ok > (args.camera_down_land if cam_down else args.lost_land):
                reason = (f"カメラが復帰しない（{now - last_ok:.1f} 秒）" if cam_down
                          else f"マーカーを {now - last_ok:.1f} 秒見失った")
                break
            if age >= 0.3:
                integral[:] = 0      # 見失っている間に溜めると、復帰した瞬間に暴れる
                recent.clear()       # 古い位置との差で速度を出さない
            tello.send_rc_control(cmd_r, cmd_f, 0, 0)

            if writer:
                st = tello.get_current_state()
                p = s[1] if s is not None else [np.nan] * 3
                writer.writerow([f"{now - t_start:.2f}", "ctrl", wp + 1,
                                 f"{target[0]:.4f}", f"{target[1]:.4f}",
                                 *(f"{v:.4f}" for v in p),
                                 f"{s[2]:.1f}" if s is not None else "", f"{age * 1000:.0f}",
                                 f"{i_size:.1f}",
                                 f"{err[0]:.4f}", f"{err[1]:.4f}", cmd_r, cmd_f,
                                 st.get("tof", 0), st.get("bat", 0), f"{tracker.fps:.0f}"])
            if now - t0 > next_print:
                next_print = now - t0 + 0.5
                if age < 0.3:
                    sys.stdout.write(f"\r{now - t0:5.1f}s  ずれ x{err[0] * 100:+5.1f} y{err[1] * 100:+5.1f}cm"
                                     f"  指令 右{cmd_r:+3d} 前{cmd_f:+3d}  カメラ{tracker.fps:3.0f}fps   ")
                else:
                    what = "カメラの映像が止まっている" if cam_down else "マーカーを見失い中（画面外？）"
                    sys.stdout.write(f"\r{now - t0:5.1f}s  {what}（指令0）"
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
        tracker.join(timeout=3)     # カメラを閉じてから終わる（終了時の異常終了を防ぐ）
        if log_file:
            log_file.close()
        tello.end()

    if tracker.crashes:
        print(f"注意: 飛行中に OAK-D が {tracker.crashes} 回落ちて開き直しました")
    if stats:
        a = np.array(stats) * 100
        print(f"\n目標からのずれ: 平均 {a.mean():.1f}cm  最大 {a.max():.1f}cm  "
              f"（{len(a)}サンプル）")


if __name__ == "__main__":
    main()
