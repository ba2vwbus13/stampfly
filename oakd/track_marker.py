#!/usr/bin/env python3
"""OAK-D で ArUco マーカーを追跡し、3次元位置と向きをリアルタイム表示する

    python3 track_marker.py --size 2.8 --id 2              # 画面表示あり（q で終了）
    python3 track_marker.py --size 2.8 --id 2 --no-gui     # 数値だけ
    python3 track_marker.py --size 2.8 --id 2 --depth      # ステレオ深度も併記
    python3 track_marker.py --size 2.8 --id 2 --csv log.csv

--size は印刷したマーカーの黒い正方形の一辺 [cm]。実測値を入れること。

座標系（カメラ基準）:
    X = 右が正, Y = 下が正, Z = カメラから前方が正  [m]
向き:
    yaw = マーカーの向き [deg]。マーカーの X 軸をカメラの XZ 平面に投影した角度。
lat:
    カメラが撮った時刻から表示までの遅れ [ms]。100ms 以下が目安。
    大きいときは USB 2 で繋がっている（起動時の表示が HIGH）か、解像度が高すぎる。

飛んでいる機体を追うので、露出は既定で手動（短時間）にしている。暗い場所では
--exposure を伸ばすか --iso を上げる。伸ばすとぶれて検出できなくなる。
"""

import argparse
import csv
import sys
import time

import cv2
import depthai as dai
import numpy as np

RESOLUTIONS = {"720p": (1280, 720), "1080p": (1920, 1080)}
DEPTH_SIZE = (640, 400)  # 深度は小さくする（1080p だと 4MB/frame になり帯域を食う）
DICT = cv2.aruco.DICT_4X4_50


def build_pipeline(args, width, height):
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setVideoSize(width, height)
    cam.setInterleaved(False)
    cam.setFps(args.fps)
    if args.exposure > 0:
        # 自動露出だと屋内でシャッターが遅くなり、動いている機体がぶれる
        cam.initialControl.setManualExposure(int(args.exposure * 1000), args.iso)
    cam.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)

    xout_video = pipeline.create(dai.node.XLinkOut)
    xout_video.setStreamName("video")
    xout_video.input.setBlocking(False)
    xout_video.input.setQueueSize(1)
    cam.video.link(xout_video.input)  # NV12。輝度面だけ使うので変換不要

    if args.depth:
        left = pipeline.create(dai.node.MonoCamera)
        right = pipeline.create(dai.node.MonoCamera)
        for mono, socket in ((left, dai.CameraBoardSocket.CAM_B), (right, dai.CameraBoardSocket.CAM_C)):
            mono.setBoardSocket(socket)
            mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono.setFps(args.fps)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(*DEPTH_SIZE)
        left.out.link(stereo.left)
        right.out.link(stereo.right)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        xout_depth.input.setBlocking(False)
        xout_depth.input.setQueueSize(1)
        stereo.depth.link(xout_depth.input)

    return pipeline


def marker_object_points(size_m: float) -> np.ndarray:
    """マーカー中心を原点とした四隅の3次元座標（左上から時計回り）"""
    h = size_m / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


def solve_pose(obj_points, corners, K, dist):
    """平面マーカーの姿勢には2つの解がある。再投影誤差が小さい方を選ぶ。

    戻り値: (rvec, tvec, 誤差比) 誤差比が 1 に近いほど、どちらの解か決めにくい
    """
    ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        obj_points, corners, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or len(rvecs) == 0:
        return None, None, None
    errs = [float(e) for e in np.array(errors).ravel()]
    best = int(np.argmin(errs))
    ratio = (sorted(errs)[0] / sorted(errs)[1]) if len(errs) > 1 and sorted(errs)[1] > 0 else 0.0
    return rvecs[best], tvecs[best], ratio


def depth_at(depth_frame, corners, frame_size) -> float:
    """マーカー領域の深度の中央値 [m]。取れなければ nan"""
    if depth_frame is None:
        return float("nan")
    sx = depth_frame.shape[1] / frame_size[0]
    sy = depth_frame.shape[0] / frame_size[1]
    pts = (corners * np.array([sx, sy])).astype(np.int32)
    mask = np.zeros(depth_frame.shape, np.uint8)
    cv2.fillConvexPoly(mask, pts, 255)
    values = depth_frame[(mask > 0) & (depth_frame > 0)]
    return float(np.median(values)) / 1000.0 if values.size else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=float, required=True, help="マーカー一辺の実測値 [cm]")
    ap.add_argument("--id", type=int, default=None, help="この id のみ追跡（省略時は全部）")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--res", choices=sorted(RESOLUTIONS), default="1080p")
    ap.add_argument("--exposure", type=float, default=6.0,
                    help="手動露出 [ms]。0 で自動。動く機体では 4〜8ms")
    ap.add_argument("--iso", type=int, default=800, help="手動露出のときの ISO 感度 (100〜1600)")
    ap.add_argument("--depth", action="store_true", help="ステレオ深度も取る（帯域を使う）")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--csv", default=None)
    ap.add_argument("--seconds", type=float, default=None)
    args = ap.parse_args()

    size_m = args.size / 100.0
    obj_points = marker_object_points(size_m)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # 角を精密化して姿勢を安定させる
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT), params)

    writer = csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["t_s", "id", "x_m", "y_m", "z_m", "dist_m", "yaw_deg", "depth_m", "lat_ms"])

    width, height = RESOLUTIONS[args.res]
    with dai.Device(build_pipeline(args, width, height)) as device:
        calib = device.readCalibration()
        K = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height), dtype=np.float64)
        dist = np.array(calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A), dtype=np.float64)[:8]

        usb = device.getUsbSpeed()
        print(f"USB {usb}  marker {args.size:.1f} cm  {width}x{height}@{args.fps}  "
              f"exposure {'auto' if args.exposure <= 0 else f'{args.exposure:.1f}ms/ISO{args.iso}'}"
              f"{'  depth on' if args.depth else ''}")
        if str(usb).endswith("HIGH"):
            print("警告: USB 2 で接続されている。USB 3 のポートとケーブルに変えると遅延が大きく減る")
        print(f"目安: 距離 2m でマーカーは約 {K[0, 0] * size_m / 2.0:.0f} px (20〜25 px 以上で検出できる)")
        print("t[s]     id    X[m]    Y[m]    Z[m]  dist[m]  yaw[deg]" +
              ("  depth[m]" if args.depth else "") + "  lat[ms]")

        q_video = device.getOutputQueue("video", 1, blocking=False)
        q_depth = device.getOutputQueue("depth", 1, blocking=False) if args.depth else None

        t0 = time.time()
        last_print = 0.0
        frames = 0
        try:
            while True:
                pkt = q_video.get()
                # 溜まっていたら最新だけ使う（遅延を溜めない）
                while True:
                    newer = q_video.tryGet()
                    if newer is None:
                        break
                    pkt = newer
                frames += 1
                latency_ms = (dai.Clock.now() - pkt.getTimestamp()).total_seconds() * 1000.0

                nv12 = pkt.getFrame()
                gray = nv12[:height, :]  # NV12 の先頭が輝度面。そのままグレースケールとして使える

                depth_frame = None
                if q_depth is not None:
                    msg = q_depth.tryGet()
                    if msg is not None:
                        depth_frame = msg.getFrame()

                corners, ids, _ = detector.detectMarkers(gray)
                now = time.time() - t0

                found = []
                for c, marker_id in (zip(corners, ids.ravel()) if ids is not None else []):
                    if args.id is not None and marker_id != args.id:
                        continue
                    rvec, tvec, ratio = solve_pose(obj_points, c[0], K, dist)
                    if rvec is None:
                        continue
                    x, y, z = tvec.ravel()
                    R, _ = cv2.Rodrigues(rvec)
                    yaw = np.degrees(np.arctan2(R[0, 0], R[2, 0]))
                    d = depth_at(depth_frame, c[0], (width, height)) if args.depth else float("nan")
                    found.append((int(marker_id), x, y, z, float(np.linalg.norm(tvec)), yaw, d, ratio, rvec, tvec, c))

                if now - last_print > 0.2:
                    last_print = now
                    if found:
                        for f in found:
                            line = (f"{now:6.2f}  {f[0]:3d}  {f[1]:7.3f} {f[2]:7.3f} {f[3]:7.3f} "
                                    f"{f[4]:7.3f}  {f[5]:8.1f}")
                            if args.depth:
                                line += f"  {f[6]:7.3f}"
                            line += f"  {latency_ms:6.0f}"
                            if f[7] > 0.9:  # 2つの解の誤差が近い = 向きが定まらない
                                line += "  (向き不確か)"
                            print(line)
                    else:
                        print(f"{now:6.2f}  マーカーが見つかりません                       "
                              f"{'         ' if args.depth else ''}{latency_ms:6.0f}")

                if writer:
                    for f in found:
                        writer.writerow([f"{now:.3f}", f[0]] + [f"{v:.4f}" for v in f[1:7]] +
                                        [f"{latency_ms:.1f}"])

                if not args.no_gui:
                    view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                    for i, f in enumerate(found):
                        cv2.aruco.drawDetectedMarkers(view, [f[10]], np.array([[f[0]]]))
                        cv2.drawFrameAxes(view, K, dist, f[8], f[9], size_m * 0.7)
                        cv2.putText(view, f"id{f[0]} X{f[1]:+.2f} Y{f[2]:+.2f} Z{f[3]:.2f}m yaw{f[5]:+.0f}",
                                    (10, 40 + 34 * i), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    cv2.putText(view, f"latency {latency_ms:.0f} ms   {frames / max(now, 1e-3):.0f} fps",
                                (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
                    cv2.imshow("OAK-D marker tracking (q で終了)", view)
                    if cv2.waitKey(1) == ord("q"):
                        break

                if args.seconds is not None and now > args.seconds:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            print(f"平均 {frames / max(time.time() - t0, 1e-3):.1f} fps")
            if csv_file:
                csv_file.close()
            if not args.no_gui:
                cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
