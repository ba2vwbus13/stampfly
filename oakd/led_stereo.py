#!/usr/bin/env python3
"""機体の LED を左右の白黒カメラで捉え、三角測量で3次元位置を求める

マーカー方式の弱点（機体が傾くと読めない）を避けるための追跡方法。
LED は点なので、機体がどれだけ傾いても見え方が変わらない。

    python3 led_stereo.py            # 位置を表示（ArUco の測定値と比較）
    python3 led_stereo.py --no-check # 比較せず LED だけで測る

しくみ:
  1. 長い露出で床の基準マーカー(id 0)を撮り、左カメラ基準の座標系を作る
  2. 露出を 0.5ms 程度まで短くすると、周囲は暗くなり LED だけが白く写る
  3. 左右の画像で光点の位置を求め、三角測量して3次元位置を出す
  4. 基準マーカーの座標系へ変換する

白黒カメラはグローバルシャッターなので、動いている機体でもぶれない。

露出の目安（部屋の明るさで変わるので、飛ばす前に必ず確認する）:
    このプログラムを10秒動かし、ばらつきが 10mm 以下になる露出を選ぶ。
    左右の光点が1個ずつになるのが理想。10個も出るときは露出が長すぎる。
      暗くした室内: 700us 付近
      明るい室内  : 400us 付近（LEDが周囲の光に埋もれやすく、不安定）
    明るい部屋ではカーテンを閉めるなどして暗くした方が確実。
"""

import argparse
import sys
import time

import cv2
import depthai as dai
import numpy as np

from marker_common import average_rotation, make_detector, marker_object_points, solve_pose

WIDTH, HEIGHT = 1280, 800
LEFT, RIGHT = dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C


class LedStereo:
    """左右カメラで LED を追う"""

    def __init__(self, fps=60, led_exposure_us=700, led_iso=400,
                 marker_exposure_us=1000, marker_iso=400, threshold=200, min_area=3, max_area=300,
                 z_min=0.3, z_max=4.0, max_reproj_px=2.0,
                 world_z_min=-0.05, world_z_max=1.5, gate_m=0.25):
        self.led_exp, self.led_iso = led_exposure_us, led_iso
        self.marker_exp, self.marker_iso = marker_exposure_us, marker_iso
        self.threshold, self.min_area = threshold, min_area
        # 窓や白い紙など、大きな明るい面は LED ではないので候補から外す
        self.max_area = max_area
        self.z_min, self.z_max = z_min, z_max          # カメラから見た距離の範囲 [m]
        self.max_reproj_px = max_reproj_px             # 再投影のずれの上限 [px]
        # 世界座標での高さの範囲。床より下は反射の鏡像なので捨てる
        self.world_z_min, self.world_z_max = world_z_min, world_z_max
        # 一度見つけたら、その近く(gate_m 以内)の候補だけを見る。
        # 近くに何も無ければ全体から探し直す
        self.gate_m = gate_m

        pipeline = dai.Pipeline()
        self.ctrl_names = {}
        for sock, name in ((LEFT, "left"), (RIGHT, "right")):
            mono = pipeline.create(dai.node.MonoCamera)
            mono.setBoardSocket(sock)
            mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono.setFps(fps)
            mono.initialControl.setManualExposure(marker_exposure_us, marker_iso)
            xout = pipeline.create(dai.node.XLinkOut)
            xout.setStreamName(name)
            xout.input.setBlocking(False)
            xout.input.setQueueSize(1)
            mono.out.link(xout.input)
            xin = pipeline.create(dai.node.XLinkIn)
            xin.setStreamName(name + "_ctrl")
            xin.out.link(mono.inputControl)
            self.ctrl_names[name] = name + "_ctrl"

        self.device = dai.Device(pipeline)
        cal = self.device.readCalibration()
        self.KL = np.array(cal.getCameraIntrinsics(LEFT, WIDTH, HEIGHT))
        self.KR = np.array(cal.getCameraIntrinsics(RIGHT, WIDTH, HEIGHT))
        self.DL = np.array(cal.getDistortionCoefficients(LEFT))[:8]
        self.DR = np.array(cal.getDistortionCoefficients(RIGHT))[:8]
        # 左カメラから見た右カメラの位置と向き
        ext = np.array(cal.getCameraExtrinsics(LEFT, RIGHT))
        self.R_lr = ext[:3, :3]
        self.T_lr = ext[:3, 3].reshape(3, 1) / 100.0  # cm → m
        self.P1 = np.hstack([np.eye(3), np.zeros((3, 1))])
        self.P2 = np.hstack([self.R_lr, self.T_lr])

        self.q_left = self.device.getOutputQueue("left", 1, blocking=False)
        self.q_right = self.device.getOutputQueue("right", 1, blocking=False)
        self.q_ctrl = {n: self.device.getInputQueue(c) for n, c in self.ctrl_names.items()}
        self.R_world = None
        self.t_ref = None
        self.usb = str(self.device.getUsbSpeed())

    # ---- 露出の切り替え ----
    def set_exposure(self, us, iso):
        ctrl = dai.CameraControl()
        ctrl.setManualExposure(us, iso)
        for q in self.q_ctrl.values():
            q.send(ctrl)
        for _ in range(25):  # センサーに反映されるまでフレームを捨てる
            self.frames()

    def marker_mode(self):
        self.set_exposure(self.marker_exp, self.marker_iso)

    def led_mode(self):
        self.set_exposure(self.led_exp, self.led_iso)

    def frames(self):
        """左右で同じ瞬間の画像を返す（連番で照合する）

        別々に取り出すと時刻がずれ、動いている機体では三角測量が狂う。
        """
        left = self.q_left.get()
        right = self.q_right.get()
        for _ in range(10):
            dn = left.getSequenceNum() - right.getSequenceNum()
            if dn == 0:
                break
            if dn > 0:
                right = self.q_right.get()
            else:
                left = self.q_left.get()
        return left.getCvFrame(), right.getCvFrame()

    # ---- 光点の検出 ----
    def find_blobs(self, img, limit=8):
        """明るいかたまりを大きい順に返す [(x, y, 面積), ...]

        床に映った反射も光点として写るため、1つに決め打ちせず候補を集める。
        """
        _, th = cv2.threshold(img, self.threshold, 255, cv2.THRESH_BINARY)
        n, _, stats, cent = cv2.connectedComponentsWithStats(th)
        blobs = [(float(cent[i][0]), float(cent[i][1]), int(stats[i, cv2.CC_STAT_AREA]))
                 for i in range(1, n)
                 if self.min_area <= stats[i, cv2.CC_STAT_AREA] <= self.max_area]
        blobs.sort(key=lambda b: -b[2])
        return blobs[:limit]

    def triangulate(self, bl, br):
        """左右の光点1組から3次元位置と再投影のずれを返す"""
        pl = cv2.undistortPoints(np.array([[[bl[0], bl[1]]]], np.float64), self.KL, self.DL)
        pr = cv2.undistortPoints(np.array([[[br[0], br[1]]]], np.float64), self.KR, self.DR)
        X = cv2.triangulatePoints(self.P1, self.P2, pl, pr)
        if abs(X[3]) < 1e-9:
            return None, None
        X = (X[:3] / X[3]).ravel()
        if not (self.z_min < X[2] < self.z_max):
            return None, None
        rep_l, _ = cv2.projectPoints(X.reshape(1, 3), np.zeros(3), np.zeros(3), self.KL, self.DL)
        rvec, _ = cv2.Rodrigues(self.R_lr)
        rep_r, _ = cv2.projectPoints(X.reshape(1, 3), rvec, self.T_lr, self.KR, self.DR)
        err = max(np.linalg.norm(rep_l.ravel() - np.array(bl[:2])),
                  np.linalg.norm(rep_r.ravel() - np.array(br[:2])))
        return X, float(err)

    def led_position(self, predict=None, expect_z=None, z_tol=0.2):
        """LED の3次元位置（左カメラ基準）[m]。見つからなければ None

        左右の光点のすべての組み合わせを調べ、
          * 再投影のずれが小さい
          * 世界座標で床より上にある（反射の鏡像を除く）
          * 機体が報告している高度に近い（expect_z を渡した場合）
          * 直前の位置に近い
        ものを選ぶ。

        expect_z は機体の下向き ToF が測った高度。これと照合することで、
        床に置かれた別の光へロックオンしたまま機体を見失う事故を防ぐ。
        """
        left, right = self.frames()
        bls, brs = self.find_blobs(left), self.find_blobs(right)
        if not bls or not brs:
            return None, (bls, brs)

        cand = []
        for bl in bls:
            for br in brs:
                X, err = self.triangulate(bl, br)
                if X is None or err > self.max_reproj_px:
                    continue
                if self.R_world is not None:
                    w = self.R_world @ (X - self.t_ref)
                    if w[2] < self.world_z_min or w[2] > self.world_z_max:
                        continue   # 床より下（反射）や高すぎるものは捨てる
                    if expect_z is not None and abs(w[2] - expect_z) > z_tol:
                        continue   # 機体が報告している高度と合わない光は機体ではない
                dist = float(np.linalg.norm(X - predict)) if predict is not None else 0.0
                score = err + 50.0 * dist       # 直前の位置に近い方を優先
                cand.append((score, dist, X))

        if not cand:
            return None, (bls, brs)
        # まず直前の位置の近くだけで選ぶ。無ければ全体から選ぶ
        near = [c for c in cand if c[1] <= self.gate_m] if predict is not None else []
        best = min(near or cand, key=lambda c: c[0])[2]
        return best, (bls, brs)
        pl = cv2.undistortPoints(np.array([[[bl[0], bl[1]]]], np.float64), self.KL, self.DL)
        pr = cv2.undistortPoints(np.array([[[br[0], br[1]]]], np.float64), self.KR, self.DR)
        X = cv2.triangulatePoints(self.P1, self.P2, pl, pr)
        if abs(X[3]) < 1e-9:
            return None, (bl, br)
        X = (X[:3] / X[3]).ravel()

        # 左右で別の光を拾っていないか検算する。求めた3次元点を両方の画像へ
        # 投影し直し、元の光点と大きく離れていたら捨てる
        if not (self.z_min < X[2] < self.z_max):
            return None, (bl, br)
        rep_l, _ = cv2.projectPoints(X.reshape(1, 3), np.zeros(3), np.zeros(3), self.KL, self.DL)
        rvec, _ = cv2.Rodrigues(self.R_lr)
        rep_r, _ = cv2.projectPoints(X.reshape(1, 3), rvec, self.T_lr, self.KR, self.DR)
        err = max(np.linalg.norm(rep_l.ravel() - np.array(bl[:2])),
                  np.linalg.norm(rep_r.ravel() - np.array(br[:2])))
        if err > self.max_reproj_px:
            return None, (bl, br)
        return X, (bl, br)

    # ---- 座標系 ----
    def calibrate_world(self, ref_id=0, ref_size_cm=5.0, frames=40, timeout=30):
        """床の基準マーカーを見て、左カメラ基準 → 世界座標 の変換を作る"""
        self.marker_mode()
        detector = make_detector()
        obj = marker_object_points(ref_size_cm / 100.0)
        rots, trans, prev_R = [], [], None
        t0 = time.time()
        while len(rots) < frames:
            if time.time() - t0 > timeout:
                return False
            left, _ = self.frames()
            corners, ids, _ = detector.detectMarkers(left)
            if ids is None:
                continue
            for c, mid in zip(corners, ids.ravel()):
                if int(mid) != ref_id:
                    continue
                rvec, tvec, _ = solve_pose(obj, c[0], self.KL, self.DL, prev_R)
                if rvec is None:
                    continue
                R, _ = cv2.Rodrigues(rvec)
                prev_R = R
                rots.append(R)
                trans.append(tvec.reshape(3))
        R_ref = average_rotation(rots)
        self.t_ref = np.mean(trans, axis=0)
        self.R_world = R_ref.T
        return True

    def marker_pose(self, marker_id, size_cm):
        """マーカーの世界座標での位置と向き（確認用）。露出はマーカー用にしておくこと"""
        detector = make_detector()
        obj = marker_object_points(size_cm / 100.0)
        left, _ = self.frames()
        corners, ids, _ = detector.detectMarkers(left)
        if ids is None:
            return None, None
        for c, mid in zip(corners, ids.ravel()):
            if int(mid) != marker_id:
                continue
            rvec, tvec, _ = solve_pose(obj, c[0], self.KL, self.DL)
            if rvec is None:
                continue
            R, _ = cv2.Rodrigues(rvec)
            pos = self.R_world @ (tvec.reshape(3) - self.t_ref)
            Rw = self.R_world @ R
            return pos, float(np.degrees(np.arctan2(Rw[1, 0], Rw[0, 0])))
        return None, None

    def world_position(self, prev_world=None, expect_z=None, z_tol=0.2):
        """LED の世界座標 [m]。見つからなければ None

        prev_world を渡すと、その位置に近い候補を優先する（反射との取り違え防止）。
        expect_z を渡すと、その高さから離れた候補を除外する（別の光へのロックオン防止）。
        """
        predict = None
        if prev_world is not None:
            # 直前の世界座標をカメラ座標へ戻して予測とする
            predict = self.R_world.T @ np.asarray(prev_world) + self.t_ref
        X, blobs = self.led_position(predict, expect_z, z_tol)
        if X is None:
            return None, blobs
        return self.R_world @ (X - self.t_ref), blobs

    def close(self):
        self.device.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exposure", type=int, default=700, help="LED を撮るときの露出 [us]")
    ap.add_argument("--iso", type=int, default=400)
    ap.add_argument("--threshold", type=int, default=200, help="光点とみなす明るさ")
    ap.add_argument("--max-area", type=int, default=300, help="これより大きい光は LED でないとみなす")
    ap.add_argument("--seconds", type=float, default=10)
    ap.add_argument("--no-check", action="store_true", help="ArUco との比較をしない")
    args = ap.parse_args()

    tracker = LedStereo(led_exposure_us=args.exposure, led_iso=args.iso,
                        threshold=args.threshold, max_area=args.max_area)
    print(f"USB {tracker.usb}  基線長 {np.linalg.norm(tracker.T_lr)*100:.1f} cm")
    print("床の基準マーカー(id 0)で座標系を作ります…")
    if not tracker.calibrate_world():
        sys.exit("基準マーカーが見つかりません")
    print("座標系ができました")

    ref_pos = ref_head = None
    if not args.no_check:
        ref_pos, ref_head = tracker.marker_pose(2, 2.8)  # 白黒カメラでは小さすぎて写らないことが多い
        if ref_pos is not None:
            print(f"ArUco で測った機体の位置: x{ref_pos[0]:+.3f} y{ref_pos[1]:+.3f} z{ref_pos[2]:+.3f} m "
                  f"（向き {ref_head:+.1f}度）")
        else:
            print("機体のマーカーは見えません（比較なしで続けます）")

    tracker.led_mode()
    print("\nLED で追跡します（Ctrl-C で終了）")
    t0 = time.time()
    samples = []
    prev = None
    try:
        while time.time() - t0 < args.seconds:
            pos, blobs = tracker.world_position(prev)
            if pos is None:
                sys.stdout.write(f"\r光点なし（左{len(blobs[0])}個 右{len(blobs[1])}個）      ")
            else:
                prev = pos
                samples.append(pos)
                sys.stdout.write(f"\rx{pos[0]:+.3f} y{pos[1]:+.3f} z{pos[2]:+.3f} m  "
                                 f"（候補 左{len(blobs[0])}個 右{len(blobs[1])}個）      ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    print()
    if samples:
        a = np.array(samples)
        print(f"平均 x{a[:,0].mean():+.3f} y{a[:,1].mean():+.3f} z{a[:,2].mean():+.3f} m")
        print(f"ばらつき(標準偏差) {a[:,0].std()*1000:.1f} / {a[:,1].std()*1000:.1f} / "
              f"{a[:,2].std()*1000:.1f} mm  ({len(a)}回)")
        if ref_pos is not None:
            d = a.mean(axis=0) - ref_pos
            print(f"ArUco との差 x{d[0]*100:+.1f} y{d[1]*100:+.1f} z{d[2]*100:+.1f} cm")
            print("※ LED は機体の上面より少し上にあるので、z の差は数cm出るのが正常")
    tracker.close()


if __name__ == "__main__":
    main()
