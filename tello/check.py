#!/usr/bin/env python3
"""Tello EDU の状態を確認する（飛ばさない）

    python3 check.py

事前に Mac の Wi-Fi を Tello 本体（TELLO-XXXXXX）に接続しておくこと。
接続中は学内ネットワークから切れるので、git や pip は使えなくなる。

表示する内容:
    バッテリー残量、高度、機体の温度、姿勢（ロール/ピッチ/ヨー）、
    速度、下向きToFの距離、Wi-Fi の信号強度
"""

import sys
import time

from djitellopy import Tello


def main():
    tello = Tello()
    print("Tello に接続しています…（Wi-Fi が TELLO-XXXXXX になっているか確認）")
    try:
        tello.connect()
    except Exception as e:
        sys.exit(f"接続できません: {e}\n"
                 f"Mac の Wi-Fi を Tello 本体に接続してから、もう一度実行してください")

    print(f"接続しました。SDK {tello.query_sdk_version()}  "
          f"シリアル {tello.query_serial_number()}")
    battery = tello.get_battery()
    print(f"バッテリー残量: {battery}%" + ("  ← 飛ばすには少ないです" if battery < 30 else ""))

    print("\n状態を5秒間表示します（Ctrl-C で終了）")
    print(" 残量  高度  ToF  温度  ロール ピッチ  ヨー   速度(x,y,z)")
    t0 = time.time()
    try:
        while time.time() - t0 < 5:
            s = tello.get_current_state()
            print(f"{s.get('bat', 0):4d}% {s.get('h', 0):4d}cm {s.get('tof', 0):4d}cm "
                  f"{s.get('templ', 0):3d}C  {s.get('roll', 0):+5d} {s.get('pitch', 0):+5d} "
                  f"{s.get('yaw', 0):+5d}  ({s.get('vgx', 0):+3d},{s.get('vgy', 0):+3d},"
                  f"{s.get('vgz', 0):+3d})")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    print("\n確認のポイント:")
    print("  * バッテリー残量が 30% 以上あるか")
    print("  * 機体を水平に置いた状態で、ロールとピッチが 0 付近か")
    print("  * ToF（下向きの距離）が床までの距離として妥当か")
    tello.end()


if __name__ == "__main__":
    main()
