#!/usr/bin/env python3
"""OAK-D の動作確認: カラーと深度を1フレーム取得し、画面中央までの距離を表示する

    python3 check_device.py [--out 保存先ディレクトリ]

画面表示はせず、PNG を保存する（ヘッドレスでも動く）。
"""

import argparse
import pathlib

import cv2
import depthai as dai
import numpy as np

WIDTH, HEIGHT = 640, 400


def build_pipeline():
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setPreviewSize(WIDTH, HEIGHT)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    for mono, socket in ((left, dai.CameraBoardSocket.CAM_B), (right, dai.CameraBoardSocket.CAM_C)):
        mono.setBoardSocket(socket)
        mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)       # 遮蔽部の誤差を減らす
    stereo.setSubpixel(True)             # 遠距離の分解能を上げる
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # カラー画像と画素を揃える
    stereo.setOutputSize(WIDTH, HEIGHT)  # カラーのプレビューと同じ大きさにする
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam.preview.link(xout_rgb.input)

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    return pipeline


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=".", help="PNG の保存先")
    args = ap.parse_args()
    out_dir = pathlib.Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    with dai.Device(build_pipeline()) as device:
        print("MXID      :", device.getMxId())
        print("USB speed :", device.getUsbSpeed())
        q_rgb = device.getOutputQueue("rgb", 4, blocking=False)
        q_depth = device.getOutputQueue("depth", 4, blocking=False)

        # 露出と深度が落ち着くまで数フレーム捨てる
        for _ in range(30):
            rgb = q_rgb.get().getCvFrame()
            depth = q_depth.get().getFrame()  # uint16, 単位 mm

        h, w = depth.shape
        center = depth[h // 2, w // 2]
        valid = depth[depth > 0]
        print(f"深度 {w}x{h} / カラー {rgb.shape[1]}x{rgb.shape[0]}")
        print(f"画面中央までの距離: {center / 1000:.3f} m  (0 は測定不能)")
        if valid.size:
            print(f"有効画素 {100 * valid.size / depth.size:.1f}%  "
                  f"最小 {valid.min() / 1000:.2f} m  中央値 {np.median(valid) / 1000:.2f} m  "
                  f"最大 {valid.max() / 1000:.2f} m")

        # 深度を見やすい色に変換して保存
        vis = np.clip(depth.astype(np.float32) / 4000.0 * 255.0, 0, 255).astype(np.uint8)
        vis = cv2.applyColorMap(vis, cv2.COLORMAP_TURBO)
        vis[depth == 0] = 0  # 測定不能な画素は黒
        cv2.imwrite(str(out_dir / "oakd_color.png"), rgb)
        cv2.imwrite(str(out_dir / "oakd_depth.png"), vis)
        print("保存:", out_dir / "oakd_color.png", ",", out_dir / "oakd_depth.png")


if __name__ == "__main__":
    main()
