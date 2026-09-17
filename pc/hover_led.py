#!/usr/bin/env python3
"""LED を追いかけて StampFly を1点でホバリングさせる

マーカー方式（hover.py）は、機体が浮いて傾くとカメラから読めなくなった。
こちらは機体の LED を左右の白黒カメラで捉えて三角測量するので、
機体がどれだけ傾いても位置を見失わない。

    python3 hover_led.py --dry-run     # 飛ばさずに位置と指令だけ確認
    python3 hover_led.py --log h.csv   # 実際に飛ばす

準備:
    * 床の基準マーカー(id 0, 5cm)を飛行場所の中央に置く（座標系のため）
    * M5GO に bridge/ のファームを入れ、USB で接続する
    * Atom JoyStick の電源は切る
    * 機体はリセットボタンを押してから、動かさずに初期化を待つ

向き（ヨー角）について:
    LED は点なので向きが分からない。機体のジャイロ（テレメトリの yaw）を使う。
    起動時に「機体の前を、床マーカーの赤い矢印(+x)に合わせて置く」ことで
    基準を合わせる。c キーでいつでも取り直せる。

キー操作:
    スペース  離陸 / 着陸（M5GO の A ボタンでも可）
    w/s/a/d   目標位置を前後左右に 10cm ずつ動かす
    r / f     目標高度を 10cm 上下
    h         いまの位置を目標にする
    c         向きの基準を取り直す（機体を +x に向けて置いた状態で）
    z         送信停止（機体は自動着陸）
    Ctrl-C    終了
"""

import argparse
import csv
import glob
import pathlib
import queue as queue_mod
import shutil
import sys
import termios
import threading
import time
import tty

import numpy as np
import serial

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "oakd"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from led_stereo import LedStereo  # noqa: E402
from marker_common import wrap_deg  # noqa: E402
from hover import PID, to_command, DEG_PER_STICK  # noqa: E402,F401

ALT_AUTO, ALT_MANUAL = 4, 5
MODE_NAMES = {0: "INIT", 1: "CALIB", 2: "FLIGHT", 3: "PARKING",
              4: "LOG", 5: "LANDING", 6: "FLIP"}


def find_bridge_port():
    ports = sorted(glob.glob("/dev/cu.usbserial-*") + glob.glob("/dev/cu.wchusbserial*"))
    if not ports:
        sys.exit("M5GO が見つかりません。--bridge-port で指定してください")
    return ports[0]


def start_key_reader():
    keys = queue_mod.Queue()

    def run():
        while True:
            ch = sys.stdin.read(1)
            if not ch:
                break
            keys.put(ch)

    threading.Thread(target=run, daemon=True).start()
    return keys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="指令を送らない（確認用）")
    ap.add_argument("--bridge-port", default=None)
    # カメラ
    ap.add_argument("--exposure", type=int, default=700, help="LED を撮る露出 [us]")
    ap.add_argument("--iso", type=int, default=400)
    ap.add_argument("--threshold", type=int, default=200, help="光点とみなす明るさ")
    ap.add_argument("--led-offset-z", type=float, default=0.05,
                    help="機体が報告する高度と LED の高さの差 [m]")
    ap.add_argument("--z-tol", type=float, default=0.6,
                    help="報告された高度からこれ以上離れた光は機体とみなさない [m]")
    ap.add_argument("--max-area", type=int, default=300,
                    help="これより大きい光は LED でないとみなす（窓や白い紙の除外）")
    ap.add_argument("--fps", type=int, default=60)
    # 制御
    ap.add_argument("--kp", type=float, default=8.0, help="位置のずれ1mあたり何度傾けるか")
    ap.add_argument("--ki", type=float, default=2.0,
                    help="機体が持つ傾きの偏りを打ち消す項。小さいと流され続ける")
    ap.add_argument("--kd", type=float, default=6.0, help="速度1m/sあたり何度戻すか")
    ap.add_argument("--max-tilt", type=float, default=8.0, help="傾ける角度の上限 [deg]")
    # 高さ（手動高度モードでは PC が推力そのものを決める）
    ap.add_argument("--alt-mode", choices=("manual", "auto"), default="manual",
                    help="manual: 高さもPCが制御しLEDは黄色のまま（推奨）。"
                         "auto: 機体任せ。LEDが暗い紫になり見失いやすい")
    ap.add_argument("--hover-thr", type=float, default=0.41,
                    help="ホバリングに必要なスロットル（電圧で変わる。3.8Vで約0.41）")
    ap.add_argument("--kz", type=float, default=0.6, help="高さのずれ1mあたりのスロットル量")
    ap.add_argument("--kvz", type=float, default=0.35, help="上下の速度に対するブレーキ")
    ap.add_argument("--thr-min", type=float, default=0.25)
    ap.add_argument("--thr-max", type=float, default=0.60)
    ap.add_argument("--ramp", type=float, default=1.5, help="離陸時にスロットルを上げる時間 [s]")
    ap.add_argument("--target-z", type=float, default=0.25, help="目標高度 [m]（LEDの高さ基準）")
    ap.add_argument("--slew", type=float, default=0.05, help="指令の1フレームあたりの変化上限")
    ap.add_argument("--settle", type=float, default=2.0, help="離陸してから制御を始めるまで [s]")
    # 向き
    ap.add_argument("--yaw-sign", type=float, default=-1.0,
                    help="機体のyawと世界座標の回転の向きの対応（-1 か 1）")
    # 安全
    ap.add_argument("--fence-xy", type=float, default=0.5, help="この範囲を出たら停止 [m]")
    ap.add_argument("--fence-z", type=float, default=0.8, help="この高さを超えたら停止 [m]")
    ap.add_argument("--lost-grace", type=float, default=0.5, help="見失ってから停止するまで [s]")
    ap.add_argument("--neutral-after", type=float, default=0.15)
    ap.add_argument("--log", default=None)
    args = ap.parse_args()

    tracker = LedStereo(fps=args.fps, led_exposure_us=args.exposure, led_iso=args.iso,
                        threshold=args.threshold, max_area=args.max_area)
    print(f"高度: {'PCが制御（LEDは黄色のまま）' if args.alt_mode == 'manual' else '機体任せ'}")
    print(f"カメラ USB {tracker.usb}  基線長 {np.linalg.norm(tracker.T_lr)*100:.1f} cm")
    print("床の基準マーカー(id 0)で座標系を作ります…")
    if not tracker.calibrate_world():
        sys.exit("基準マーカーが見つかりません。カメラに写っているか確認してください")
    tracker.led_mode()
    print("座標系ができました。LED 追跡に切り替えます")

    ser = None
    try:
        port = args.bridge_port or find_bridge_port()
        ser = serial.Serial(port, 115200, timeout=0)
        print(f"中継機: {port}" + ("（お試しモード: 受信のみ）" if args.dry_run else ""))
    except (SystemExit, serial.SerialException) as e:
        if not args.dry_run:
            raise
        print(f"中継機なしで続けます（{e}）")

    alt_mode = ALT_MANUAL if args.alt_mode == "manual" else ALT_AUTO
    pid_x, pid_y = PID(args.kp, args.ki, args.kd), PID(args.kp, args.ki, args.kd)
    pid_z = PID(args.kz, 0.15, args.kvz, i_limit=0.3)
    target = np.array([0.0, 0.0, args.target_z])
    flying = False
    transmitting = True
    arm_pulse = False
    arm_count = 0
    stop_reason = ""
    t_takeoff = None
    yaw_zero = None          # 機体の yaw の基準（+x を向いたときの値）
    telem = {}
    rx = ""

    pos_f = None
    vel = np.zeros(3)
    t_prev = None
    last_seen = time.time()
    ail_prev = ele_prev = 0.0

    log_file = writer = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        writer = csv.writer(log_file)
        writer.writerow(["t_s", "x", "y", "z", "head", "tx", "ty", "tz",
                         "ail", "ele", "thr", "seen", "mode", "volt", "flying", "nblob"])

    interactive = sys.stdin.isatty()
    old_term = termios.tcgetattr(sys.stdin) if interactive else None
    if interactive:
        tty.setcbreak(sys.stdin.fileno())
    keys = start_key_reader() if interactive else queue_mod.Queue()
    print(f"キーボード入力: {'有効' if interactive else '無効（M5GOのボタンで操作してください）'}")
    print("向きの基準は、機体を +x に向けて離陸させれば自動で取れます"
          "（c キー、または M5GO の B ボタンでも取れます）")
    print("スペース=離陸/着陸  wasd=目標移動  rf=目標高度  h=その場  z=停止  Ctrl-C=終了")

    t0 = time.time()
    last_draw = 0.0
    try:
        while True:
            # 機体が報告している高度（下向きToF）と照合して、別の光へのロックオンを防ぐ
            expect_z = None
            if telem.get("alt") is not None and time.time() - telem.get("t_recv", 0) < 1.0:
                expect_z = telem["alt"] + args.led_offset_z
            pos, blobs = tracker.world_position(pos_f, expect_z, args.z_tol)
            seen = pos is not None
            now = time.time() - t0

            # ---- キー入力 ----
            while not keys.empty():
                key = keys.get()
                if key == " ":
                    arm_pulse = True
                    arm_count += 1
                    pid_x.reset()
                    pid_y.reset()
                    transmitting = True
                    stop_reason = "お試しモードなので離陸しません" if args.dry_run else ""
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
                elif key == "c":
                    if telem.get("yaw") is not None:
                        yaw_zero = args.yaw_sign * telem["yaw"]
                        stop_reason = "向きの基準を取りました"
                    else:
                        stop_reason = "テレメトリが来ていません"
                elif key == "z":
                    transmitting = False
                    stop_reason = "手動停止"
                elif key == "\x03":
                    raise KeyboardInterrupt

            # ---- 位置と速度 ----
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

            # ---- 機体の向き ----
            head = None
            if telem.get("yaw") is not None and yaw_zero is not None:
                head = wrap_deg(args.yaw_sign * telem["yaw"] - yaw_zero)

            # ---- 安全の判定 ----
            lost_s = time.time() - last_seen
            if flying and transmitting:
                if lost_s > args.lost_grace:
                    transmitting, stop_reason = False, "LEDを見失った"
                elif pos_f is not None:
                    if abs(pos_f[0]) > args.fence_xy or abs(pos_f[1]) > args.fence_xy:
                        transmitting, stop_reason = False, "範囲外(水平)"
                    elif pos_f[2] > args.fence_z:
                        transmitting, stop_reason = False, "範囲外(高さ)"

            # ---- 位置制御 ----
            ail = ele = thr = 0.0
            # 手動高度モードでは機体が自動で浮かないので、静定待ちはしない
            settling = (alt_mode == ALT_AUTO and flying and t_takeoff is not None
                        and (time.time() - t_takeoff) < args.settle)
            if pos_f is not None and not settling and lost_s < args.neutral_after:
                if not flying:
                    pid_x.reset()
                    pid_y.reset()
                err = target - pos_f
                dt = max(1.0 / args.fps, 1e-3)
                ax = pid_x.update(err[0], vel[0], dt)
                ay = pid_y.update(err[1], vel[1], dt)
                ail, ele = to_command(ax, ay, head if head is not None else 0.0, args.max_tilt)
                if alt_mode == ALT_MANUAL:
                    # ホバリングに必要な分を土台にして、ずれと上下の速度で補正する
                    thr = args.hover_thr + pid_z.update(target[2] - pos_f[2], vel[2], dt)
                    if t_takeoff is not None:   # 離陸直後はゆっくり立ち上げる
                        ramp = min(1.0, (time.time() - t_takeoff) / max(args.ramp, 1e-3))
                        thr *= ramp
                    thr = float(np.clip(thr, 0.0 if not flying else args.thr_min, args.thr_max))
                else:
                    thr = float(np.clip(args.kz * (target[2] - pos_f[2]), -0.5, 0.5))

            ail = float(np.clip(ail, ail_prev - args.slew, ail_prev + args.slew))
            ele = float(np.clip(ele, ele_prev - args.slew, ele_prev + args.slew))
            ail_prev, ele_prev = ail, ele

            # ---- 送信 ----
            if ser is not None and transmitting and not args.dry_run:
                s_thr, s_ail, s_ele = (thr, ail, ele) if flying else (0.0, 0.0, 0.0)
                ser.write(f"C,{s_thr:.3f},{s_ail:.3f},{s_ele:.3f},0,"
                          f"{1 if arm_pulse else 0},0,0,{alt_mode}\n".encode())
            arm_pulse = False

            # ---- 機体からの返信 ----
            if ser is not None:
                data = ser.read(4096).decode(errors="replace")
                if data:
                    rx += data
                    while "\n" in rx:
                        line, rx = rx.split("\n", 1)
                        if line.startswith("#BTN,"):
                            btn = line.strip().split(",")[-1]
                            if btn == "B":
                                # 飛行前: 向きの基準を取る / 飛行中: いまの位置を目標にする
                                if not flying and telem.get("yaw") is not None:
                                    yaw_zero = args.yaw_sign * telem["yaw"]
                                    stop_reason = "向きの基準を取りました(Bボタン)"
                                elif flying and pos_f is not None:
                                    target = pos_f.copy()
                                    stop_reason = "その場を目標にしました(Bボタン)"
                            elif btn == "C":
                                transmitting = False
                                stop_reason = "M5GOのCボタンで停止"
                        elif line.startswith("T,"):
                            p = line.strip().split(",")
                            if len(p) >= 13:
                                telem = {"yaw": float(p[4]), "v": float(p[5]),
                                         "alt": float(p[6]), "mode": int(p[7]),
                                         "t_recv": time.time()}
                                drone_flying = telem["mode"] in (2, 6)
                                if drone_flying != flying:
                                    flying = drone_flying
                                    t_takeoff = time.time() if flying else None
                                    if flying and yaw_zero is None:
                                        # 基準を取っていなければ離陸時の向きを 0 とする
                                        yaw_zero = args.yaw_sign * telem["yaw"]
                                    if not flying:
                                        pid_x.reset()
                                        pid_y.reset()
                                        pid_z.reset()
                                    sys.stdout.write("\n★ 機体が" +
                                                     ("離陸しました\n" if flying else "着陸しました\n"))

            # ---- 表示 ----
            if now - last_draw > 0.2:
                last_draw = now
                pp = pos_f if pos_f is not None else np.array([np.nan] * 3)
                mode_txt = MODE_NAMES.get(telem.get("mode"), "----") if telem else "通信なし"
                line = (f"{mode_txt:7s} {'試' if args.dry_run else ('TX' if transmitting else '--')} "
                        f"arm{arm_count} {'見' if seen else '×'} "
                        f"x{pp[0]:+.2f} y{pp[1]:+.2f} z{pp[2]:+.2f} "
                        f"{('h%+.0f' % head) if head is not None else 'h--'} "
                        f"a{ail:+.2f} e{ele:+.2f} t{thr:+.2f} "
                        f"{('%.1fV' % telem['v']) if telem else ''} "
                        f"{'静定' if settling else ''}{stop_reason}")
                width = min(shutil.get_terminal_size((80, 24)).columns - 2, 76)
                sys.stdout.write("\r" + line[:width] + "\033[K")
                sys.stdout.flush()

            if writer:
                pp = pos_f if pos_f is not None else np.array([np.nan] * 3)
                writer.writerow([f"{now:.3f}"] + [f"{v:.4f}" for v in pp] +
                                [f"{head:.1f}" if head is not None else ""] +
                                [f"{v:.3f}" for v in target] +
                                [f"{ail:.3f}", f"{ele:.3f}", f"{thr:.3f}", int(seen),
                                 telem.get("mode", ""), f"{telem['v']:.2f}" if telem else "",
                                 int(flying), len(blobs[0]) if blobs else 0])
    except KeyboardInterrupt:
        pass
    finally:
        if interactive:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        if ser is not None:
            for _ in range(5):
                ser.write(f"C,0,0,0,0,0,0,0,{alt_mode}\n".encode())
                time.sleep(0.02)
            ser.close()
        tracker.close()
        if log_file:
            log_file.close()
        print("\n終了しました。" + (f" 停止理由: {stop_reason}" if stop_reason else ""))


if __name__ == "__main__":
    main()
