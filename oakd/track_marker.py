#!/usr/bin/env python3
"""OAK-D で ArUco マーカーを追跡し、3次元位置と向きをリアルタイム表示する

    python3 track_marker.py --size 5.0            # 画面表示あり（q で終了）
    python3 track_marker.py --size 5.0 --no-gui   # 端末に数値だけ出す
    python3 track_marker.py --size 5.0 --csv log.csv

--size は印刷したマーカーの黒い正方形の一辺 [cm]。実測値を入れること。

座標系（カメラ基準）:
    X = 右が正, Y = 下が正, Z = カメラから前方が正  [m]
向き:
    yaw = マーカーの向き [deg]。マーカーの X 軸をカメラの XZ 平面に投影した角度。
深度との比較:
    depth はステレオ深度から読んだ距離。マーカーから求めた Z と近ければ両方とも信頼できる。
"""

import argparse
import csv
import sys
import time

import cv2
import depthai as dai
import numpy as np

RESOLUTIONS = {"720p": (1280, 720), "1080p": (1920, 1080)}
DICT = cv2.aruco.DICT_4X4_50


def build_pipeline(fps: int, width: int, height: int):
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setPreviewSize(width, height)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setFps(fps)

    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    for mono, socket in ((left, dai.CameraBoardSocket.CAM_B), (right, dai.CameraBoardSocket.CAM_C)):
        mono.setBoardSocket(socket)
        mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono.setFps(fps)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(width, height)
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam.preview.link(xout_rgb.input)

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    return pipeline


def marker_object_points(size_m: float) -> np.ndarray:
    """マーカー中心を原点とした、四隅の3次元座標（左上から時計回り）"""
    h = size_m / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


def depth_at(depth_frame: np.ndarray, corners: np.ndarray) -> float:
    """マーカー領域の深度の中央値 [m]。取れなければ nan"""
    mask = np.zeros(depth_frame.shape, np.uint8)
    cv2.fillConvexPoly(mask, corners.astype(np.int32), 255)
    values = depth_frame[(mask > 0) & (depth_frame > 0)]
    return float(np.median(values)) / 1000.0 if values.size else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=float, required=True, help="マーカー一辺の実測値 [cm]")
    ap.add_argument("--id", type=int, default=None, help="この id のみ追跡（省略時は全部）")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--res", choices=sorted(RESOLUTIONS), default="1080p",
                    help="カラーの解像度。遠くのマーカーを見るときは 1080p")
    ap.add_argument("--no-gui", action="store_true", help="ウィンドウを出さない")
    ap.add_argument("--csv", default=None, help="CSV 保存先")
    ap.add_argument("--seconds", type=float, default=None, help="この秒数で自動終了")
    args = ap.parse_args()

    size_m = args.size / 100.0
    obj_points = marker_object_points(size_m)
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT),
                                       cv2.aruco.DetectorParameters())

    writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["t_s", "id", "x_m", "y_m", "z_m", "dist_m", "yaw_deg", "depth_m"])

    width, height = RESOLUTIONS[args.res]
    with dai.Device(build_pipeline(args.fps, width, height)) as device:
        calib = device.readCalibration()
        K = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height), dtype=np.float64)
        dist = np.array(calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A), dtype=np.float64)[:8]
        px_per_m = K[0, 0]
        print(f"USB {device.getUsbSpeed()}  marker {args.size:.1f} cm  {width}x{height}@{args.fps}")
        print(f"目安: 距離 2m でマーカーは約 {px_per_m * size_m / 2.0:.0f} px "
              f"(20〜25 px 以上で検出できる)")
        print("t[s]     id    X[m]    Y[m]    Z[m]  dist[m]  yaw[deg]  depth[m]")

        q_rgb = device.getOutputQueue("rgb", 4, blocking=False)
        q_depth = device.getOutputQueue("depth", 4, blocking=False)

        t0 = time.time()
        last_print = 0.0
        try:
            while True:
                frame = q_rgb.get().getCvFrame()
                depth_msg = q_depth.tryGet()
                depth_frame = depth_msg.getFrame() if depth_msg is not None else None

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray)
                now = time.time() - t0

                found = []
                if ids is not None:
                    for c, marker_id in zip(corners, ids.ravel()):
                        if args.id is not None and marker_id != args.id:
                            continue
                        ok, rvec, tvec = cv2.solvePnP(obj_points, c[0], K, dist,
                                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
                        if not ok:
                            continue
                        x, y, z = tvec.ravel()
                        R, _ = cv2.Rodrigues(rvec)
                        # マーカーの X 軸をカメラの XZ 平面に投影した角度
                        yaw = np.degrees(np.arctan2(R[0, 0], R[2, 0]))
                        d = depth_at(depth_frame, c[0]) if depth_frame is not None else float("nan")
                        found.append((int(marker_id), x, y, z, float(np.linalg.norm(tvec)), yaw, d))

                        if not args.no_gui:
                            cv2.aruco.drawDetectedMarkers(frame, [c], np.array([[marker_id]]))
                            cv2.drawFrameAxes(frame, K, dist, rvec, tvec, size_m * 0.7)

                if now - last_print > 0.2:  # 端末が流れすぎないように 5Hz で表示
                    last_print = now
                    if found:
                        for f in found:
                            print(f"{now:6.2f}  {f[0]:3d}  {f[1]:7.3f} {f[2]:7.3f} {f[3]:7.3f} "
                                  f"{f[4]:7.3f}  {f[5]:8.1f}  {f[6]:7.3f}")
                    else:
                        print(f"{now:6.2f}  マーカーが見つかりません")

                if writer:
                    for f in found:
                        writer.writerow([f"{now:.3f}", f[0]] + [f"{v:.4f}" for v in f[1:]])

                if not args.no_gui:
                    for i, f in enumerate(found):
                        cv2.putText(frame, f"id{f[0]} X{f[1]:+.2f} Y{f[2]:+.2f} Z{f[3]:.2f}m yaw{f[5]:+.0f}",
                                    (10, 30 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.imshow("OAK-D marker tracking (q で終了)", frame)
                    if cv2.waitKey(1) == ord("q"):
                        break

                if args.seconds is not None and now > args.seconds:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            if csv_file:
                csv_file.close()
            if not args.no_gui:
                cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
