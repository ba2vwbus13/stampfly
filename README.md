# StampFly 自律飛行の実験

M5Stack StampFly（37g の小型ドローン）を、外部カメラで測った位置にもとづいて
PC から自動制御する実験の記録とプログラム一式。

**現状**: 位置計測・通信・指令の経路は完成。自動ホバリングは未達成のまま中断。
詳しい経緯と再開方法は [HANDOFF.md](HANDOFF.md)、原因調査の記録は
[docs/findings.md](docs/findings.md) を参照。

## 全体の構成

```
OAK-D（白黒2台でLEDを三角測量）
    │ 位置 x,y,z（ばらつき 1mm、30〜60Hz）
    ▼
  Mac（Python）位置と高度を PID で制御
    │ USBシリアル
    ▼
  M5GO（中継機）
    │ ESP-NOW（2.4GHz, ch3, 50Hz, 25バイト）
    ▼
  StampFly（姿勢の安定化は機体が 400Hz で担当）
```

役割を分けたのが要点。**速い制御（姿勢）は機体、遅い制御（位置）は PC** が担当する。

## フォルダ

| パス | 内容 |
|---|---|
| `sensor_monitor/` | 全センサーの値を USB シリアルへ流すファームと、PC 側の表示ツール |
| `bridge/` | **M5GO を PC と機体の中継機にするファーム**（ESP-NOW 送信、テレメトリ転送） |
| `oakd/` | OAK-D による位置計測（LED三角測量、ArUcoマーカー、カメラ設置の確認） |
| `pc/` | PC 側の制御プログラム（手動操作、自動ホバリング） |
| `realsense/` | RealSense の確認用（macOS では動作せず、未使用） |
| `backup/` | 工場出荷ファームのバックアップ（**Git には含めていない**） |
| `docs/` | 調査の記録、参照用の写真 |

## 主なプログラム

### 位置計測（`oakd/`）

| ファイル | 用途 |
|---|---|
| `led_stereo.py` | **本命**。機体の LED を左右の白黒カメラで捉え、三角測量で3次元位置を出す |
| `view.py` | カメラの設置場所を決めるためのビューア |
| `make_markers.py` / `markers.pdf` | 印刷用 ArUco マーカー（床の基準用 id 0 など） |
| `track_world.py` | 床基準の座標系でマーカーを追跡（旧方式。座標系の作成にも使う） |
| `marker_common.py` | カメラのパイプライン、姿勢計算などの共通部品 |

### 制御（`pc/`）

| ファイル | 用途 |
|---|---|
| `hover_led.py` | **本命**。LED の位置で自動ホバリングする |
| `teleop.py` | キーボードで手動操作する（動作確認用） |
| `hover.py` | マーカー方式の自動ホバリング（旧方式。PID と座標変換はここに実装） |

### センサー確認（`sensor_monitor/`）

機体の全センサー（IMU、気圧、地磁気、ToF×2、電圧）を 50Hz で読み、
PC でグラフ表示する。飛行制御とは独立して使える。

## 使い方

準備と手順は [HANDOFF.md](HANDOFF.md) の「自動ホバリングの現状」にまとめてある。
要点だけ:

```bash
# 1. カメラの設置を確認
python3 oakd/view.py

# 2. 追跡の精度を確認（ばらつき10mm以下が目安）
python3 oakd/led_stereo.py --seconds 10 --no-check

# 3. 自動ホバリング
python3 pc/hover_led.py --target-z 0.25 --fence-xy 0.5 --fence-z 0.7 --log ~/Desktop/led.csv
```

**守るべきこと**

- 機体の**上面には何も貼らない**（LED を覆うと追跡できない）
- 飛ばす直前に**リセットボタン**を押し、初期化中（LEDが紫）は動かさない
- **Atom JoyStick の電源は切る**（入っていると PC からの離陸指令が打ち消される）
- 部屋を暗くする（明るいと LED が周囲の光に埋もれる）

## 実測値

| 項目 | 値 |
|---|---|
| LED 追跡のばらつき | x 0.9mm / y 1.7mm / z 4.5mm（静止時、露出1200us） |
| カメラの遅延 | 37ms（USB3接続時。USB2では120ms） |
| 指令の換算 | スティック 1.0 = 傾き 36度 |
| ホバリングに必要なスロットル | 4.1V→0.30、3.8V→0.41、3.6V→0.64 |
| 飛行時間 | 約4分 |

## 工場出荷ファームへ戻す

```bash
esptool --port /dev/cu.usbmodem* --baud 115200 write_flash 0 backup/stampfly_factory_8MB.bin
```

`--baud 921600` では読み書きが途中で切れる。115200 を使う。

## 参考

- 機体: [M5Stack StampFly](https://docs.m5stack.com/en/app/Stamp%20Fly)（81.5×81.5×31mm、27.7g）
- ファーム流用元: [m5stack/M5StampFly](https://github.com/m5stack/M5StampFly)（MIT、commit `fa7b6a5`）
- コントローラーのプロトコル解析: [Atom-JoyStick](https://github.com/m5stack/Atom-JoyStick) の StampFlyController
