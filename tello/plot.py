#!/usr/bin/env python3
"""飛行のログを図にする

    python3 tello/plot.py ~/Desktop/tello_rect3.csv

上から見た軌跡（どこを飛んだか）、目標からのずれ（時間の推移）、
積分の大きさ（外乱をどれだけ打ち消していたか）を1枚にまとめる。
数字の表だけでは、行き過ぎているのか流されているのかが分かりにくいので。
"""

import argparse
import csv
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402

plt.rcParams["font.family"] = "Hiragino Sans"
plt.rcParams["axes.unicode_minus"] = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("-o", "--out", default=None, help="出力する画像。既定は入力と同じ名前の .png")
    args = ap.parse_args()

    path = pathlib.Path(args.csv).expanduser()
    rows = [r for r in csv.DictReader(open(path)) if r["phase"] == "ctrl"]
    t = np.array([float(r["t_s"]) for r in rows]); t -= t[0]
    wp = np.array([int(r["wp"]) for r in rows])
    x = np.array([float(r["x_m"]) for r in rows]) * 100
    y = np.array([float(r["y_m"]) for r in rows]) * 100
    tx = np.array([float(r["tgt_x_m"]) for r in rows]) * 100
    ty = np.array([float(r["tgt_y_m"]) for r in rows]) * 100
    ic = (np.array([float(r["i_cmd"]) for r in rows]) if "i_cmd" in rows[0]
          else np.zeros(len(rows)))
    d = np.hypot(tx - x, ty - y)

    # 離陸地点を原点にする
    x0, y0 = tx[0], ty[0]
    x, y, tx, ty = x - x0, y - y0, tx - x0, ty - y0

    fig = plt.figure(figsize=(13, 6.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1], hspace=0.35, wspace=0.25)

    # --- 上から見た軌跡 ---
    ax = fig.add_subplot(gs[:, 0])
    ax.plot(x, y, lw=0.8, color="0.7", zorder=1)
    sc = ax.scatter(x, y, c=t, cmap="viridis", s=6, zorder=2)
    fig.colorbar(sc, ax=ax, label="時刻 [s]", pad=0.02)
    # 各点の目標
    goals = []
    for w in sorted(set(wp)):
        m = wp == w
        goals.append((tx[m][-1], ty[m][-1]))
    gx, gy = zip(*goals)
    ax.plot(gx, gy, "--", color="crimson", lw=1, zorder=3)
    ax.scatter(gx, gy, marker="s", s=90, facecolor="none", edgecolor="crimson",
               lw=2, zorder=4, label="目標の点")
    # 経路が出発点へ戻るとき、最初と最後の点は同じ座標になる。
    # そのままだとラベルが重なって読めないので、同じ場所の番号はまとめる
    labels = {}
    for i, (a, b) in enumerate(goals):
        labels.setdefault((round(a, 1), round(b, 1)), []).append(str(i + 1))
    for (a, b), names in labels.items():
        ax.annotate("・".join(names) + " 点目", (a, b), textcoords="offset points",
                    xytext=(9, 7), color="crimson", fontsize=11, fontweight="bold")
    ax.scatter([0], [0], marker="*", s=180, color="tab:green", zorder=5, label="離陸地点")
    ax.set_aspect("equal")
    ax.margins(0.16)          # ラベルが枠からはみ出さないように余白を取る
    ax.set_xlabel("x [cm]（基準マーカーの矢印の向き）")
    ax.set_ylabel("y [cm]")
    ax.set_title(f"上から見た軌跡　{path.name}")
    ax.grid(alpha=0.3)
    ax.legend(loc="best")

    # --- 目標からのずれ ---
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, d, lw=1, color="tab:blue")
    for w in sorted(set(wp))[1:]:
        ax2.axvline(t[wp == w][0], color="0.8", lw=1)
    ax2.axhline(5, color="tab:green", ls=":", lw=1)
    ax2.set_ylabel("目標からのずれ [cm]")
    ax2.set_title(f"ずれ　平均 {d.mean():.1f}cm　最大 {d.max():.1f}cm")
    ax2.grid(alpha=0.3)

    # --- 積分（外乱を打ち消している量）---
    ax3 = fig.add_subplot(gs[1, 1], sharex=ax2)
    ax3.plot(t, ic, lw=1, color="tab:orange")
    for w in sorted(set(wp))[1:]:
        ax3.axvline(t[wp == w][0], color="0.8", lw=1)
    ax3.set_xlabel("時刻 [s]")
    ax3.set_ylabel("積分の大きさ [rc]")
    ax3.set_title("外乱を打ち消している量")
    ax3.grid(alpha=0.3)

    out = pathlib.Path(args.out).expanduser() if args.out else path.with_suffix(".png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"{out} に保存しました")


if __name__ == "__main__":
    main()
