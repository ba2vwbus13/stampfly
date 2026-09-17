#!/usr/bin/env python3
"""カメラの設置を決めるための確認用ビューア

    python3 view.py              # 映像を表示（q で終了）
    python3 view.py --no-gui     # 数値だけ（画面なしの環境用）

映っているマーカーに枠を描き、id・見かけの大きさ（px）・距離の目安を出す。
座標系の設定（world_calib.json）は不要なので、設置場所を決める段階で使える。

見かたの目安:
    * マーカーは 25px 以上あれば安定して検出できる
    * 機体を飛ばす高さまで持ち上げても、枠が消えないこと
    * 飛行させる空間が画面の中央に来るようにカメラを向けること
"""

import argparse
import pathlib
import sys
import time

import cv2
import depthai as dai
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from marker_common import RESOLUTIONS, build_pipeline, latest, make_detector  # noqa: E402

MARKER_CM = {0: 5.0, 1: 4.0, 2: 3.0}  # 既定の大きさ（距離の目安の計算用）


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--res", choices=sorted(RESOLUTIONS), default="1080p")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--exposure", type=float, default=6.0, help="手動露出 [ms]。0 で自動")
    ap.add_argument("--iso", type=int, default=800)
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--seconds", type=float, default=None)
    args = ap.parse_args()

    width, height = RESOLUTIONS[args.res]
    detector = make_detector()

    with dai.Device(build_pipeline(width, height, args.fps, args.exposure, args.iso)) as device:
        calib = device.readCalibration()
        K = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height))
        fx = K[0, 0]
        print(f"USB {device.getUsbSpeed()}  {width}x{height}@{args.fps}")
        if str(device.getUsbSpeed()).endswith("HIGH"):
            print("警告: USB 2 接続。遅延が増えるが、設置場所の確認には使える")
        print("q で終了。マーカーは 25px 以上あれば安定して検出できる")

        q = device.getOutputQueue("video", 1, blocking=False)
        t0 = time.time()
        last_print = 0.0
        while True:
            pkt = latest(q)
            gray = pkt.getFrame()[:height, :]
            corners, ids, _ = detector.detectMarkers(gray)
            now = time.time() - t0

            found = []
            if ids is not None:
                for c, marker_id in zip(corners, ids.ravel()):
                    marker_id = int(marker_id)
                    px = float(np.linalg.norm(c[0][0] - c[0][1]))
                    cx, cy = c[0].mean(axis=0)
                    size_cm = MARKER_CM.get(marker_id)
                    dist = (fx * size_cm / 100.0 / px) if (size_cm and px > 0) else float("nan")
                    found.append((marker_id, cx, cy, px, dist))

            if now - last_print > 0.5:
                last_print = now
                if found:
                    txt = "  ".join(f"id{m}:{p:.0f}px {d:.2f}m" for m, _, _, p, d in found)
                else:
                    txt = "マーカーが見つかりません"
                sys.stdout.write("\r" + txt[:76] + "\033[K")
                sys.stdout.flush()

            if not args.no_gui:
                view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                # 画面の中央に十字を描く（ここに飛行空間が来るのが理想）
                cv2.drawMarker(view, (width // 2, height // 2), (80, 80, 255),
                               cv2.MARKER_CROSS, 60, 2)
                if found:
                    cv2.aruco.drawDetectedMarkers(view, corners, ids)
                for i, (m, cx, cy, px, dist) in enumerate(found):
                    ok = px >= 25
                    cv2.putText(view, f"id{m} {px:.0f}px {dist:.2f}m {'OK' if ok else '小さすぎ'}",
                                (int(cx) + 20, int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 255, 0) if ok else (0, 165, 255), 2)
                cv2.putText(view, "q:終了  中央の十字に飛行空間が来るように向ける",
                            (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
                cv2.imshow("OAK-D setup view", view)
                if cv2.waitKey(1) == ord("q"):
                    break

            if args.seconds is not None and now > args.seconds:
                break

        if not args.no_gui:
            cv2.destroyAllWindows()
    print()


if __name__ == "__main__":
    main()
