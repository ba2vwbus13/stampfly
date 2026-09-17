#!/usr/bin/env python3
"""Tello EDU を単体でホバリングさせ、その安定度を測る

    python3 hover_test.py             # 10秒ホバリングして着陸
    python3 hover_test.py --seconds 20 --log hover.csv

StampFly では「機体が自力で安定するか」の基準がないまま外部制御に進んだため、
問題の切り分けに苦労した。Tello は下向きカメラで自分の位置を保てるので、
まずこの基準を作る。ここが安定していれば、外部カメラを繋いだあとに
挙動がおかしくなったとき「こちら側の問題」と即断できる。

安全のために:
    * 周囲2m を空ける。プロペラガードを付ける
    * バッテリー残量 30% 以上で行う
    * Ctrl-C で着陸する（緊急時は手で押さえずに Ctrl-C）
"""

import argparse
import csv
import sys
import time

import numpy as np
from djitellopy import Tello


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10, help="ホバリングさせる時間 [s]")
    ap.add_argument("--height", type=int, default=40,
                    help="ホバリングする高さ [cm]。離陸は必ず約80cmまで上がるので、"
                         "その後この高さまで降りる（下向きToFで確認。30cm以上を推奨）")
    ap.add_argument("--log", default=None)
    args = ap.parse_args()

    tello = Tello()
    print("接続中…")
    tello.connect()
    battery = tello.get_battery()
    print(f"バッテリー {battery}%")
    if battery < 30:
        sys.exit("残量が少ないので中止します。充電してください")

    writer = log_file = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        writer = csv.writer(log_file)
        writer.writerow(["t_s", "h_cm", "tof_cm", "roll", "pitch", "yaw", "vgx", "vgy", "vgz", "bat"])

    samples = []
    print(f"離陸します。{args.seconds:.0f}秒ホバリングして着陸します")
    try:
        tello.takeoff()   # 仕様で必ず約80cmまで上がる（高さは指定できない）
        # 目標の高さまで、下向きToFを見ながらゆっくり降りる/上がる
        t_adj = time.time()
        while time.time() - t_adj < 8:
            tof = tello.get_distance_tof()
            if tof <= 0 or tof > 500:        # 測定範囲外の値は無視
                time.sleep(0.05)
                continue
            err = args.height - tof
            if abs(err) <= 5:
                break
            speed = int(max(-30, min(30, err)))   # 最大30cm/s
            tello.send_rc_control(0, 0, speed, 0)
            sys.stdout.write(f"\r高さを調整中: ToF {tof}cm → 目標 {args.height}cm   ")
            sys.stdout.flush()
            time.sleep(0.05)
        tello.send_rc_control(0, 0, 0, 0)
        print()

        t0 = time.time()
        while time.time() - t0 < args.seconds:
            s = tello.get_current_state()
            t = time.time() - t0
            row = [s.get(k, 0) for k in ("h", "tof", "roll", "pitch", "yaw", "vgx", "vgy", "vgz", "bat")]
            samples.append(row)
            if writer:
                writer.writerow([f"{t:.2f}"] + row)
            sys.stdout.write(f"\r{t:5.1f}s  高度{row[0]:3d}cm ToF{row[1]:3d}cm  "
                             f"R{row[2]:+3d} P{row[3]:+3d} Y{row[4]:+4d}  "
                             f"速度({row[5]:+3d},{row[6]:+3d},{row[7]:+3d})  残量{row[8]}%   ")
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n中断します")
    finally:
        print("\n着陸します")
        try:
            tello.land()
        except Exception as e:
            print(f"着陸の指令に失敗: {e}")
        if log_file:
            log_file.close()
        tello.end()

    if samples:
        a = np.array(samples, dtype=float)
        print(f"\nホバリングの安定度（{len(a)}サンプル）")
        print(f"  高度     : 平均{a[:,0].mean():.0f}cm  ばらつき{a[:,0].std():.1f}cm")
        print(f"  姿勢     : ロール{a[:,2].mean():+.1f}±{a[:,2].std():.1f}度  "
              f"ピッチ{a[:,3].mean():+.1f}±{a[:,3].std():.1f}度")
        print(f"  水平速度 : |vx|平均{np.abs(a[:,5]).mean():.1f}  |vy|平均{np.abs(a[:,6]).mean():.1f} cm/s")
        print("\n  水平速度が平均 5cm/s 以下なら、その場に留まれている（外部制御の土台になる）")


if __name__ == "__main__":
    main()
