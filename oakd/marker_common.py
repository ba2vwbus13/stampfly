"""OAK-D でマーカーを追うための共通部品

track_marker.py（カメラ基準）と track_world.py（床基準）の両方から使う。
"""

import cv2
import depthai as dai
import numpy as np

RESOLUTIONS = {"720p": (1280, 720), "1080p": (1920, 1080)}
DICT = cv2.aruco.DICT_4X4_50


def build_pipeline(width, height, fps, exposure_ms, iso):
    """カラー映像だけを NV12 で受け取るパイプライン

    NV12 の先頭が輝度面なので、そのままグレースケールとして使える（変換不要）。
    """
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setVideoSize(width, height)
    cam.setInterleaved(False)
    cam.setFps(fps)
    if exposure_ms > 0:
        # 自動露出だと屋内でシャッターが遅くなり、動いている機体がぶれる
        cam.initialControl.setManualExposure(int(exposure_ms * 1000), iso)
    cam.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("video")
    xout.input.setBlocking(False)
    xout.input.setQueueSize(1)
    cam.video.link(xout.input)
    return pipeline


def make_detector():
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # 角を精密化して姿勢を安定させる
    # 小さく写ったマーカーを拾えるようにする（人工画像での比較で効果を確認）
    params.minMarkerPerimeterRate = 0.01      # 既定 0.03
    # useAruco3Detection は試したが、この条件では検出率が大きく下がったため使わない
    # （24通りの条件で 12/24 → 4/24）
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT), params)


def marker_object_points(size_m: float) -> np.ndarray:
    """マーカー中心を原点とした四隅の3次元座標（左上から時計回り）

    マーカー座標系: X = 右, Y = 上, Z = 面から手前（床に置けば Z が鉛直上向き）
    """
    h = size_m / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


def rotation_diff_deg(R1, R2) -> float:
    """2つの姿勢の差 [deg]"""
    cos = (np.trace(R1.T @ R2) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def solve_pose(obj_points, corners, K, dist, prev_R=None):
    """平面マーカーの姿勢には2つの解がある。どちらかを選ぶ。

    再投影誤差に差があればその小さい方。差が小さくて決めきれないときは、
    1つ前のフレームの姿勢に近い方を選ぶ（解が毎フレーム反転するのを防ぐ）。

    戻り値: (rvec, tvec, 誤差比) 誤差比が 1 に近いほど判別が難しい
    """
    ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        obj_points, corners, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or len(rvecs) == 0:
        return None, None, None
    errs = [float(e) for e in np.array(errors).ravel()]
    order = int(np.argmin(errs))
    ratio = (sorted(errs)[0] / sorted(errs)[1]) if len(errs) > 1 and sorted(errs)[1] > 0 else 0.0

    if prev_R is not None and len(rvecs) > 1 and ratio > 0.6:
        diffs = [rotation_diff_deg(cv2.Rodrigues(r)[0], prev_R) for r in rvecs]
        order = int(np.argmin(diffs))
    return rvecs[order], tvecs[order], ratio


def average_rotation(matrices) -> np.ndarray:
    """回転行列の平均。単純に足すと回転行列でなくなるので、SVD で直交化する"""
    U, _, Vt = np.linalg.svd(np.sum(matrices, axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:  # 鏡像になったら直す
        U[:, -1] *= -1
        R = U @ Vt
    return R


def wrap_deg(a: float) -> float:
    """角度を -180〜180 に収める"""
    return (a + 180.0) % 360.0 - 180.0


def latest(queue):
    """溜まっているフレームを捨てて最新だけ返す（遅延を溜めない）"""
    pkt = queue.get()
    while True:
        newer = queue.tryGet()
        if newer is None:
            return pkt
        pkt = newer
