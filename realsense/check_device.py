#!/usr/bin/env python3
"""RealSense が pyrealsense2 から使えるか確認する

    python3 check_device.py

macOS では初回にカメラ利用の許可が必要。ターミナルから実行すること
（許可のダイアログはターミナルアプリに対して出る）。
"""

import sys

import pyrealsense2 as rs


def main():
    ctx = rs.context()
    try:
        devices = list(ctx.query_devices())
    except RuntimeError as e:
        print(f"デバイス一覧の取得に失敗: {e}")
        print("macOS ではカメラ利用の許可（システム設定 → プライバシーとセキュリティ → カメラ）と、")
        print("USB 3 での直結が必要。sudo で実行すると通る場合もある。")
        return 1

    if not devices:
        print("RealSense が見つからない。USB 3 ポートに直結し、ケーブルがデータ通信対応か確認する。")
        return 1

    for d in devices:
        for key in ("name", "serial_number", "firmware_version", "usb_type_descriptor"):
            try:
                print(f"{key:20s}: {d.get_info(getattr(rs.camera_info, key))}")
            except RuntimeError:
                print(f"{key:20s}: (取得できず)")
        for s in d.query_sensors():
            print(f"  sensor: {s.get_info(rs.camera_info.name)}")

    # 実際にストリームを開いて1フレーム取れるか確認する
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    try:
        pipeline.start(cfg)
    except RuntimeError as e:
        print(f"\nストリーム開始に失敗: {e}")
        return 1

    try:
        frames = pipeline.wait_for_frames(5000)
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        w, h = depth.get_width(), depth.get_height()
        center = depth.get_distance(w // 2, h // 2)
        print(f"\n深度 {w}x{h}, カラー {color.get_width()}x{color.get_height()}")
        print(f"画面中央までの距離: {center:.3f} m  (0.000 は測定不能)")
        print("OK: フレーム取得に成功")
    finally:
        pipeline.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
