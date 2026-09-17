#!/usr/bin/env python3
"""マーカーの貼り付け角のずれを測って保存する

機体に貼ったマーカーが、機体の前方向に対して何度ずれているかを測る。
ここがずれていると、位置制御の「左へ戻れ」が「前へ進め」になり、
機体が加速して飛び去る。

手順:
  1. 機体を床に置き、**機体の前（USB端子の反対側）を、床の基準マーカーの
     赤い矢印（+x）と同じ向き**に合わせる
  2. このプログラムを実行する

    python3 measure_yaw_offset.py            # 測って world_calib.json に保存
    python3 measure_yaw_offset.py --dry-run  # 測るだけ（保存しない）

保存した値は hover.py が自動で読み込む。--yaw-offset で上書きもできる。
"""

import argparse
import json
import pathlib
import sys
import time

import cv2
import depthai as dai
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from marker_common import (RESOLUTIONS, build_pipeline, latest, make_detector,  # noqa: E402
                           marker_object_points, solve_pose, wrap_deg)

DEFAULT_CALIB = pathlib.Path(__file__).with_name("world_calib.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drone-id", type=int, default=2)
    ap.add_argument("--drone-size", type=float, default=2.8)
    ap.add_argument("--calib", type=pathlib.Path, default=DEFAULT_CALIB)
    ap.add_argument("--res", choices=sorted(RESOLUTIONS), default="1080p")
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if not args.calib.exists():
        sys.exit(f"座標系のファイルがありません: {args.calib}\n"
                 f"先に  python3 track_world.py --calib-only  を実行してください")
    calib = json.loads(args.calib.read_text())
    R_world = np.array(calib["R_ref"]).T

    width, height = RESOLUTIONS[args.res]
    detector = make_detector()
    obj = marker_object_points(args.drone_size / 100.0)

    print("機体の前（USB端子の反対側）を、床マーカーの赤い矢印と同じ向きに置いてください")
    print("測定中…")

    with dai.Device(build_pipeline(width, height, 30, 6.0, 800)) as device:
        c = device.readCalibration()
        K = np.array(c.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height))
        dist = np.array(c.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))[:8]
        q = device.getOutputQueue("video", 1, blocking=False)

        heads, prev_R = [], None
        t0 = time.time()
        while len(heads) < args.frames:
            if time.time() - t0 > 30:
                sys.exit("機体のマーカーが見つかりません。カメラに写っているか確認してください")
            gray = latest(q).getFrame()[:height, :]
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is None:
                continue
            for cc, marker_id in zip(corners, ids.ravel()):
                if int(marker_id) != args.drone_id:
                    continue
                rvec, tvec, _ = solve_pose(obj, cc[0], K, dist, prev_R)
                if rvec is None:
                    continue
                R, _ = cv2.Rodrigues(rvec)
                prev_R = R
                Rw = R_world @ R
                heads.append(np.degrees(np.arctan2(Rw[1, 0], Rw[0, 0])))

    # 角度の平均（±180度をまたぐ場合に備えてベクトルで平均する）
    rad = np.radians(heads)
    offset = float(np.degrees(np.arctan2(np.sin(rad).mean(), np.cos(rad).mean())))
    spread = float(np.std([wrap_deg(h - offset) for h in heads]))
    print(f"\n測定結果: マーカーのX軸は、機体の前方向から {offset:+.1f} 度ずれています"
          f"（ばらつき {spread:.1f} 度、{len(heads)} 回の平均）")

    if spread > 10:
        print("警告: ばらつきが大きいです。機体が動いていないか、マーカーがよく見えるか確認してください")

    if args.dry_run:
        print("--dry-run のため保存しません")
        return 0

    calib["yaw_offset"] = offset
    args.calib.write_text(json.dumps(calib, indent=2))
    print(f"保存しました: {args.calib.name} の yaw_offset")
    print("hover.py はこの値を自動で読み込みます")
    return 0


if __name__ == "__main__":
    sys.exit(main())
