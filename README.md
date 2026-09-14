# StampFly 開発

機体: M5Stack StampFly（ESP32-S3）

## フォルダ構成

| パス | 内容 |
|---|---|
| `backup/stampfly_factory_8MB.bin` | 工場出荷時フラッシュ全体（8MB、verify済み）。**機体固有データを含むためGitには入れていない** |
| `backup/SHA256.txt` | 上記のハッシュ |
| `sensor_monitor/firmware/` | センサー値をUSBシリアルに流すファーム（PlatformIO、**モーターは駆動しない**） |
| `sensor_monitor/viewer/stampfly_viewer.py` | PC側リアルタイムグラフ |

## センサーモニターの使い方

### 1. ファームの書き込み（学内ネットワークではプロキシ必須）

```bash
cd sensor_monitor/firmware && HTTPS_PROXY=http://proxy.okinawa-ct.ac.jp:8080 HTTP_PROXY=http://proxy.okinawa-ct.ac.jp:8080 ~/.platformio/penv/bin/pio run -t upload
```

起動直後の2秒でジャイロのバイアスを測るので、**電源投入・リセット時は机に置いて動かさない**。

### 2. 表示

```bash
python3 sensor_monitor/viewer/stampfly_viewer.py
```

CSVにも保存する場合:

```bash
python3 sensor_monitor/viewer/stampfly_viewer.py --log run1.csv
```

シリアルを直接見る場合（`#`はログ、`D,`はデータ行）:

```bash
~/.platformio/penv/bin/pio device monitor -b 115200
```

### 出力データ（50Hz）

| 列 | 内容 |
|---|---|
| ax/ay/az_g | 加速度 [G]、機体座標（X前・Y右・Z下、静止時 az≈-1） |
| gx/gy/gz_dps | 角速度 [deg/s]（起動時バイアス補正済み） |
| roll/pitch_deg | 相補フィルタによる姿勢 |
| mx/my/mz_uT, heading_deg | 地磁気（**未キャリブレーション**。モーター磁石の影響で方位は不正確） |
| press_hPa, temp_C, baro_alt_m | 気圧・温度・起動地点基準の気圧高度 |
| tof_bottom_mm, tof_front_mm | ToF距離。有効な対象なしは -1 |
| vbat_V | バッテリー電圧（INA3221 CH2） |

## 工場出荷ファームへ戻す

```bash
esptool --port /dev/cu.usbmodem83101 --baud 115200 write_flash 0 backup/stampfly_factory_8MB.bin
```

※ `--baud 921600` だと読み書きが途中で切れた。115200で使う。

## 既知の問題・メモ

- **前方ToFは「バッテリーを付けないと」動かない（解決済み、2026-09-14）**
  - USB給電だけ（電圧1.7〜2.5V）だと、前方ToFは2回目の測定開始時にリセットされた。
  - バッテリーを付けると（3.83V）、前方ToFも約30Hzで正常に測定できた。
  - **センサーを使うときは必ずバッテリーを接続する。**
  - リセットするとI²Cアドレスが0x2Aから既定の0x29に戻り、下向きToFと衝突する。そのためファームには、0x2Aが応答しなくなったら前方ToFをXSHUTで停止する保護を残してある。ログに `#WARN front ToF reset` が出たら電源不足を疑う。
- ToFドライバ（`lib/vl53l3c/vl53lx_platform.c`）のI²Cタイムアウトが1msで、大きな設定書き込みが打ち切られうるため50msに延長した。
- BMI270/VL53L3CXのドライバは公式 [m5stack/M5StampFly](https://github.com/m5stack/M5StampFly)（MIT）から流用。BMP280・BMM150・INA3221は `main.cpp` に自前で最小実装した。
