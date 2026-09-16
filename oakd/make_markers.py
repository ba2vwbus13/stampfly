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
from matplotlib import font_manager
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt

# 日本語が豆腐にならないよう、入っている和文フォントを使う
plt.rcParams["pdf.fonttype"] = 42  # TrueType で埋め込む（和文フォントの埋め込みに必要）
for _name in ("Arial Unicode MS", "Hiragino Sans", "Noto Sans CJK JP"):
    if _name in {f.name for f in font_manager.fontManager.ttflist}:
        plt.rcParams["font.family"] = _name
        break

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
            # 座標の向きを紙に描く（床に置いたときの x, y の向き）
            ax.annotate("", xy=(1.28, 0.5), xytext=(1.02, 0.5), xycoords="axes fraction",
                        arrowprops=dict(arrowstyle="-|>", color="red", lw=1.5))
            ax.text(1.31, 0.5, "+x", color="red", transform=ax.transAxes, va="center", fontsize=10)
            ax.annotate("", xy=(0.5, 1.28), xytext=(0.5, 1.02), xycoords="axes fraction",
                        arrowprops=dict(arrowstyle="-|>", color="blue", lw=1.5))
            ax.text(0.5, 1.31, "+y", color="blue", transform=ax.transAxes, ha="center", fontsize=10)

            fig.text(0.15 + w + 0.12, y - h / 2,
                     f"id = {marker_id}\nsize = {size_cm:.1f} cm",
                     va="center", fontsize=11, family="monospace")
            y -= h + 0.09

        fig.text(0.15, 0.04,
                 "白い余白（マーカーの1マス分以上）を残して切り取ること。\n"
                 "印刷後に黒い正方形の一辺を実測し、その値を tracker に渡す。\n\n"
                 "床に置く基準マーカー（id 0）は、文字が読める向きで置く。そのとき\n"
                 "  +x = 紙の右方向 / +y = 紙の奥方向（上側）/ +z = 真上（高さ）\n"
                 "矢印はこの向きを示す。紙を回すと座標の向きも一緒に回る。",
                 fontsize=9)
        pdf.savefig(fig)
        plt.close(fig)

    print("作成:", args.out)
    print("ids  :", args.ids)
    print("sizes:", args.sizes, "[cm]")


if __name__ == "__main__":
    main()
