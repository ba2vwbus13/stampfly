#!/usr/bin/env python3
"""床に置いた基準マーカーを原点として、機体の位置と向きを求める

準備:
  1. 床（飛ばす場所の中央）に基準マーカー id 0 (5cm) を平らに置く
  2. カメラを 1.2〜1.5m 離し、斜め上から 40〜60 度見下ろす向きに固定する
  3. 基準マーカーと機体の両方がカメラに写ることを確認する

使い方:
    python3 track_world.py --calibrate          # 座標系を決めて保存（最初に1回）
    python3 track_world.py                      # 保存した座標系で追跡
    python3 track_world.py --csv flight.csv     # ログも保存

カメラを動かしたら、必ず --calibrate をやり直すこと。

座標系（基準マーカー基準）:
    x = マーカーの右方向 [m]
    y = マーカーの上方向（床の上を奥へ）[m]
    z = 高さ（床から上が正）[m]
    heading = 機体マーカーの向き [deg]（基準マーカーと同じ向きで 0）
"""

import argparse
import csv
import json
import pathlib
import sys
import time

import cv2
import depthai as dai
import numpy as np

from marker_common import (RESOLUTIONS, average_rotation, build_pipeline, latest,
                           make_detector, marker_object_points, solve_pose, wrap_deg)

DEFAULT_CALIB = pathlib.Path(__file__).with_name("world_calib.json")


def detect_poses(detector, gray, K, dist, sizes, prev_R):
    """写っているマーカーの姿勢を返す {id: (R, t, 誤差比, 四隅)}"""
    corners, ids, _ = detector.detectMarkers(gray)
    out = {}
    if ids is None:
        return out
    for c, marker_id in zip(corners, ids.ravel()):
        marker_id = int(marker_id)
        if marker_id not in sizes:
            continue
        rvec, tvec, ratio = solve_pose(marker_object_points(sizes[marker_id]), c[0], K, dist,
                                       prev_R.get(marker_id))
        if rvec is None:
            continue
        R, _ = cv2.Rodrigues(rvec)
        prev_R[marker_id] = R
        out[marker_id] = (R, tvec.reshape(3), ratio, c)
    return out


def calibrate(device, detector, K, dist, args, width, height):
    """基準マーカーを何フレームか見て、カメラ→世界の変換を決める"""
    q = device.getOutputQueue("video", 1, blocking=False)
    sizes = {args.ref_id: args.ref_size / 100.0}
    prev_R, rots, trans = {}, [], []
    print(f"基準マーカー id={args.ref_id} ({args.ref_size:.1f}cm) を探しています…")
    t0 = time.time()
    while len(rots) < args.calib_frames:
        if time.time() - t0 > 30:
            sys.exit("基準マーカーが見つかりません。カメラに写っているか、--ref-size が正しいか確認してください")
        gray = latest(q).getFrame()[:height, :]
        poses = detect_poses(detector, gray, K, dist, sizes, prev_R)
        if args.ref_id in poses:
            R, t, _, _ = poses[args.ref_id]
            rots.append(R)
            trans.append(t)

    R_ref = average_rotation(rots)
    t_ref = np.mean(trans, axis=0)
    spread = np.std(trans, axis=0) * 1000
    print(f"基準マーカーまで {np.linalg.norm(t_ref):.3f} m  "
          f"ばらつき X{spread[0]:.1f} Y{spread[1]:.1f} Z{spread[2]:.1f} mm")

    data = {"R_ref": R_ref.tolist(), "t_ref": t_ref.tolist(),
            "ref_id": args.ref_id, "ref_size_cm": args.ref_size,
            "res": args.res, "created": time.strftime("%Y-%m-%d %H:%M:%S")}
    args.calib.write_text(json.dumps(data, indent=2))
    print("保存:", args.calib)
    return R_ref, t_ref


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drone-id", type=int, default=2)
    ap.add_argument("--drone-size", type=float, default=2.8, help="機体マーカーの一辺 [cm]")
    ap.add_argument("--ref-id", type=int, default=0)
    ap.add_argument("--ref-size", type=float, default=5.0, help="基準マーカーの一辺 [cm]")
    ap.add_argument("--calib", type=pathlib.Path, default=DEFAULT_CALIB)
    ap.add_argument("--calibrate", action="store_true", help="座標系を決め直す")
    ap.add_argument("--calib-frames", type=int, default=60)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--res", choices=sorted(RESOLUTIONS), default="1080p")
    ap.add_argument("--exposure", type=float, default=6.0, help="手動露出 [ms]。0 で自動")
    ap.add_argument("--iso", type=int, default=800)
    ap.add_argument("--max-jump", type=float, default=0.6, help="1フレームで許す移動量 [m]")
    ap.add_argument("--max-yaw-rate", type=float, default=400.0, help="1秒で許す回転 [deg/s]")
    ap.add_argument("--alpha", type=float, default=0.5, help="平滑化の強さ (0〜1、小さいほど滑らか)")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--csv", default=None)
    ap.add_argument("--seconds", type=float, default=None)
    args = ap.parse_args()

    width, height = RESOLUTIONS[args.res]
    detector = make_detector()
    sizes = {args.drone_id: args.drone_size / 100.0, args.ref_id: args.ref_size / 100.0}

    writer = csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["t_s", "x_m", "y_m", "z_m", "heading_deg",
                         "vx", "vy", "vz", "lat_ms", "rejected"])

    with dai.Device(build_pipeline(width, height, args.fps, args.exposure, args.iso)) as device:
        calib = device.readCalibration()
        K = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height), dtype=np.float64)
        dist = np.array(calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A), dtype=np.float64)[:8]
        usb = device.getUsbSpeed()
        print(f"USB {usb}  {width}x{height}@{args.fps}")
        if str(usb).endswith("HIGH"):
            print("警告: USB 2 で接続されている。USB 3 にすると遅延が大きく減る")

        if args.calibrate or not args.calib.exists():
            R_ref, t_ref = calibrate(device, detector, K, dist, args, width, height)
        else:
            data = json.loads(args.calib.read_text())
            R_ref = np.array(data["R_ref"])
            t_ref = np.array(data["t_ref"])
            print(f"座標系を読み込み: {args.calib.name} ({data['created']})")

        R_world = R_ref.T  # カメラ座標 → 世界座標

        print("t[s]     x[m]    y[m]    z[m]  head[deg]   速さ[m/s]  lat[ms]")
        q = device.getOutputQueue("video", 1, blocking=False)
        prev_R = {}
        t0 = time.time()
        last_print = 0.0
        # 平滑化と外れ値除去のための状態
        pos_f = None
        head_f = None
        vel = np.zeros(3)
        t_prev = None
        rejected = 0
        frames = 0

        try:
            while True:
                pkt = latest(q)
                frames += 1
                latency_ms = (dai.Clock.now() - pkt.getTimestamp()).total_seconds() * 1000.0
                gray = pkt.getFrame()[:height, :]
                now = time.time() - t0

                poses = detect_poses(detector, gray, K, dist, sizes, prev_R)
                found = args.drone_id in poses
                reject = False

                if found:
                    R_d, t_d, ratio, corners = poses[args.drone_id]
                    pos = R_world @ (t_d - t_ref)          # 世界座標での位置
                    R_dw = R_world @ R_d                    # 世界座標での姿勢
                    head = np.degrees(np.arctan2(R_dw[1, 0], R_dw[0, 0]))

                    dt = (now - t_prev) if t_prev is not None else 0.0
                    if pos_f is not None and dt > 0:
                        # ありえない飛びは捨てる（見失い後の復帰は通す）
                        if np.linalg.norm(pos - pos_f) > args.max_jump and dt < 0.5:
                            reject = True
                        elif abs(wrap_deg(head - head_f)) > args.max_yaw_rate * dt and dt < 0.5:
                            # 位置は使い、向きだけ前の値を保つ
                            head = head_f
                            reject = True

                    if not reject or pos_f is None:
                        if pos_f is None or dt <= 0 or dt > 0.5:
                            pos_f, head_f, vel = pos, head, np.zeros(3)
                        else:
                            a = args.alpha
                            new_pos = a * pos + (1 - a) * pos_f
                            vel = (new_pos - pos_f) / dt
                            pos_f = new_pos
                            head_f = head_f + a * wrap_deg(head - head_f)
                        t_prev = now
                    else:
                        rejected += 1

                if now - last_print > 0.2:
                    last_print = now
                    if found and pos_f is not None:
                        print(f"{now:6.2f} {pos_f[0]:7.3f} {pos_f[1]:7.3f} {pos_f[2]:7.3f} "
                              f"{wrap_deg(head_f):9.1f} {np.linalg.norm(vel):9.2f} {latency_ms:8.0f}"
                              + ("  (外れ値)" if reject else ""))
                    else:
                        print(f"{now:6.2f}  機体マーカーが見つかりません"
                              f"                          {latency_ms:8.0f}")

                if writer and found and pos_f is not None:
                    writer.writerow([f"{now:.3f}"] + [f"{v:.4f}" for v in pos_f] +
                                    [f"{wrap_deg(head_f):.2f}"] + [f"{v:.3f}" for v in vel] +
                                    [f"{latency_ms:.1f}", int(reject)])

                if not args.no_gui:
                    view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                    for marker_id, (R, t, ratio, c) in poses.items():
                        cv2.aruco.drawDetectedMarkers(view, [c], np.array([[marker_id]]))
                        cv2.drawFrameAxes(view, K, dist, cv2.Rodrigues(R)[0], t, sizes[marker_id] * 0.8)
                    if found and pos_f is not None:
                        cv2.putText(view, f"x{pos_f[0]:+.2f} y{pos_f[1]:+.2f} z{pos_f[2]:+.2f} m  "
                                          f"head{wrap_deg(head_f):+.0f}",
                                    (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 2)
                    cv2.putText(view, f"latency {latency_ms:.0f} ms  {frames / max(now, 1e-3):.0f} fps  "
                                      f"rejected {rejected}",
                                (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
                    cv2.imshow("StampFly world tracking (q で終了)", view)
                    if cv2.waitKey(1) == ord("q"):
                        break

                if args.seconds is not None and now > args.seconds:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            print(f"平均 {frames / max(time.time() - t0, 1e-3):.1f} fps  外れ値 {rejected} 回")
            if csv_file:
                csv_file.close()
            if not args.no_gui:
                cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
