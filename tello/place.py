#!/usr/bin/env python3
"""飛ばす前に、Tello をどこに置けばよいかをカメラで測って求める

    python3 tello/place.py --waypoints "0,0 0.3,0 0,0" --probe-dist 0.15

hover_oakd.py は、経路の各点と向き測定で動く分が、浮いたときに映像の
30〜70% に入っていないと離陸しない。その条件を満たす置き場所を総当たりで探し、
いまの位置からどちらへ何cm動かせばよいかを表示する。

どこに置いても収まらない場合は、収まる経路の大きさと測定距離の上限を示す。
その場合はカメラを後ろへ下げる（座標系の作り直しが必要）。
"""

import argparse
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tello"))
from hover_oakd import Tracker, average_position, image_ratio, wait_sample  # noqa: E402

LO, HI = 0.3, 0.7


def points(offsets, probe, ground):
    """置き場所 ground に対して、画面に入っている必要がある点すべて"""
    extra = [(probe, 0), (-probe, 0), (0, probe), (0, -probe)]
    return [ground + np.array(d) for d in list(offsets) + extra]


def fits(tracker, pts, z):
    """すべての点が 30〜70% に入るか。入らない数と、いちばん外れた量を返す"""
    worst, n_bad = 0.0, 0
    for p in pts:
        for r in image_ratio(tracker, [p[0], p[1], z]):
            over = max(LO - r, r - HI, 0.0)
            if over > 0:
                worst = max(worst, over)
        if any(not (LO <= r <= HI) for r in image_ratio(tracker, [p[0], p[1], z])):
            n_bad += 1
    return n_bad, worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--waypoints", default="0,0")
    ap.add_argument("--probe-dist", type=float, default=0.20)
    ap.add_argument("--height", type=int, default=40)
    ap.add_argument("--marker-id", type=int, default=1)
    ap.add_argument("--marker-cm", type=float, default=4.0)
    ap.add_argument("--exposure", type=float, default=6.0)
    ap.add_argument("--iso", type=int, default=800)
    ap.add_argument("--calib", default=str(ROOT / "oakd" / "world_calib.json"))
    args = ap.parse_args()

    offsets = [np.array([float(a), float(b)]) for a, b in
               (w.split(",") for w in args.waypoints.split())]

    tracker = Tracker(args.calib, args.marker_id, args.marker_cm, args.exposure, args.iso)
    tracker.start()
    tracker.ready.wait(20)
    try:
        if wait_sample(tracker, 0.3, 5) is None:
            sys.exit(f"マーカー id {args.marker_id} が見えません")
        here, _ = average_position(tracker, 1.0)
        z = here[2] + args.height / 100.0
        print(f"いまの位置 x{here[0]:+.3f} y{here[1]:+.3f} z{here[2]:+.3f} m"
              f"  ホバリング高さ z{z:.3f} m")

        n_bad, _ = fits(tracker, points(offsets, args.probe_dist, here[:2]), z)
        if n_bad == 0:
            print("\nいまの場所で条件を満たしています。そのまま飛ばせます")
            return

        # 置き場所を 1cm きざみで総当たりし、収まる中でいまから最も近い場所を選ぶ
        best = None
        for dx in np.arange(-1.0, 1.0001, 0.01):
            for dy in np.arange(-1.0, 1.0001, 0.01):
                g = here[:2] + np.array([dx, dy])
                if fits(tracker, points(offsets, args.probe_dist, g), z)[0] == 0:
                    d = float(np.hypot(dx, dy))
                    if best is None or d < best[0]:
                        best = (d, dx, dy)
        if best:
            _, dx, dy = best
            print(f"\n置き場所を x 方向に {dx * 100:+.0f}cm、y 方向に {dy * 100:+.0f}cm "
                  f"動かせば収まります（基準マーカーの矢印の向きが +）")
            return

        # どこに置いても無理。何なら収まるかを示す
        print("\nどこに置いても収まりません。カメラを後ろへ下げてください"
              "（動かしたら python3 oakd/track_world.py --calib-only）")
        print("\nいまの画角で収まる大きさ:")
        for probe in (0.20, 0.15, 0.10):
            ok = []
            for size in np.arange(0.40, 0.04, -0.01):
                offs = [o / max(np.abs(np.concatenate(offsets)).max(), 1e-9) * size
                        if np.any(o) else o for o in offsets]
                found = any(
                    fits(tracker, points(offs, probe, here[:2] + np.array([dx, dy])), z)[0] == 0
                    for dx in np.arange(-0.6, 0.601, 0.02)
                    for dy in np.arange(-0.6, 0.601, 0.02))
                if found:
                    ok.append(size)
                    break
            print(f"  測定距離 {probe * 100:.0f}cm → 経路の大きさ "
                  + (f"{ok[0] * 100:.0f}cm まで" if ok else "収まらない"))
    finally:
        tracker.stop = True
        tracker.join(timeout=3)


if __name__ == "__main__":
    main()
