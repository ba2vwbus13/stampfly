#!/usr/bin/env python3
"""印刷用の ArUco マーカーを PDF で作る

    python3 make_markers.py            # markers.pdf を作る
    python3 make_markers.py --sizes 4 5 --ids 0 1

紙に印刷するときは「実際のサイズ」「拡大縮小なし（100%）」で印刷すること。
印刷後に定規で一辺を測り、実測値を追跡プログラムに渡す。
"""

import argparse

import cv2
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt

DICT = cv2.aruco.DICT_4X4_50
PX_PER_MODULE = 40  # 画像の解像度（印刷時のにじみを防ぐため大きめ）


def marker_image(marker_id: int) -> np.ndarray:
    d = cv2.aruco.getPredefinedDictionary(DICT)
    side_modules = d.markerSize + 2  # 黒枠を含めたマス数
    img = cv2.aruco.generateImageMarker(d, marker_id, side_modules * PX_PER_MODULE)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="markers.pdf")
    ap.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--sizes", type=float, nargs="+", default=[5.0, 4.0, 3.0],
                    help="マーカーの一辺 [cm]（黒い部分の外周）")
    args = ap.parse_args()

    if len(args.ids) != len(args.sizes):
        ap.error("--ids と --sizes は同じ個数にする")

    with PdfPages(args.out) as pdf:
        fig = plt.figure(figsize=(8.27, 11.69))  # A4 縦 [inch]
        fig.suptitle("ArUco markers (DICT_4X4_50)  -- print at 100% scale", fontsize=11)

        y = 0.80
        for marker_id, size_cm in zip(args.ids, args.sizes):
            img = marker_image(marker_id)
            size_in = size_cm / 2.54
            w = size_in / 8.27   # 図全体に対する比率
            h = size_in / 11.69
            ax = fig.add_axes([0.15, y - h, w, h])
            ax.imshow(img, cmap="gray", interpolation="nearest")
            ax.set_xticks([])
            ax.set_yticks([])
            for spine in ax.spines.values():
                spine.set_visible(False)
            fig.text(0.15 + w + 0.04, y - h / 2,
                     f"id = {marker_id}\nsize = {size_cm:.1f} cm",
                     va="center", fontsize=11, family="monospace")
            y -= h + 0.06

        fig.text(0.15, 0.06,
                 "白い余白（マーカーの1マス分以上）を残して切り取ること。\n"
                 "印刷後に黒い正方形の一辺を実測し、その値を tracker に渡す。",
                 fontsize=9)
        pdf.savefig(fig)
        plt.close(fig)

    print("作成:", args.out)
    print("ids  :", args.ids)
    print("sizes:", args.sizes, "[cm]")


if __name__ == "__main__":
    main()
