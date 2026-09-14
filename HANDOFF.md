# 引き継ぎメモ（2026-09-14 時点）

マシンを変える前に、作業状態と次にやることをまとめたメモ。使い方の詳細は [README.md](README.md) を参照。

## 1. 新しいマシンでの準備

1. リポジトリを取得する。

   ```bash
   git clone https://github.com/ba2vwbus13/stampfly.git
   ```

2. **工場ファームのバックアップを手で移す。** `backup/stampfly_factory_8MB.bin` はGitに入れていない（機体固有データを含むため）。
   - 旧マシンの場所: `~/Documents/latest/研究/20260914StampFly/backup/`
   - コピー後に `shasum -a 256` を実行し、`backup/SHA256.txt` の値と一致することを確認する。
3. 必要なツールを入れる。

   ```bash
   pip install platformio pyserial matplotlib esptool
   ```

   - PlatformIOは初回ビルド時に espressif32@6.9.0 一式（数百MB）をダウンロードする。
   - **学内ネットワークではプロキシの指定が必要**（指定しないと `InternetConnectionError` になる）。

     ```bash
     export HTTPS_PROXY=http://proxy.okinawa-ct.ac.jp:8080 HTTP_PROXY=http://proxy.okinawa-ct.ac.jp:8080
     ```

4. シリアルポートの名前はマシンによって変わる。StampFlyは `/dev/cu.usbmodem*` になる。旧マシンでは `/dev/cu.usbmodem83101` だった。

## 2. 現在の状態

| 項目 | 状態 |
|---|---|
| StampFly本体に入っているファーム | **センサーモニター**（このリポジトリ）。モーターは回らず、**このままでは飛ばない** |
| センサー | 全部動作確認済み: IMU, 気圧, 地磁気, 電圧, ToF×2 |
| 表示ツール | 動作確認済み（約48Hz、CSV保存可） |
| GitHub | https://github.com/ba2vwbus13/stampfly （public） |

## 3. これまでに分かったこと

- **ToFを使うときはバッテリー必須。**
  - USB給電だけだと、前方ToFが測定のたびにリセットされる。
  - バッテリーを付けると（3.83V）正常に測れる。
- **VL53L3CXはリセットするとI²Cアドレスが0x29に戻る。**
  - 戻ると下向きToFとアドレスが衝突する。
  - そのためファームは0x2Aの応答を監視し、応答が消えたら前方ToFを止める。
- ToFドライバのI²Cタイムアウト1msは短すぎるので、50msに変更済み（`lib/vl53l3c/vl53lx_platform.c`）。
- esptoolは **115200 baud** で使う。921600では途中で切れた。
- 流用元: 公式 [m5stack/M5StampFly](https://github.com/m5stack/M5StampFly)（commit `fa7b6a5`、2026-09-07）。

## 4. 未完了・次にやること

### A. Mini JoyC で StampFly を飛ばせるか（調査途中）

**手元の機材**
- M5StickC系（ESP32-PICO-D4）に Mini JoyC HAT を付けたもの。USBでは `/dev/cu.usbserial-*` として見える。
- 中には既に何らかのESP-NOWファームが入っていた（内容は未確認）。

**StampFly公式ファームの受信仕様**（`src/rc.cpp` と `src/flight_control.cpp` を読んで判明）
- 通信方式は ESP-NOW、**チャンネル3**。
- 起動時にStampFlyが自分のMACアドレスをブロードキャストし、最初に受信した送信元をテレメトリの相手として登録する。
- 受信パケットは25バイト。

  | バイト | 内容 |
  |---|---|
  | [0..2] | StampFly の MAC アドレス下位3バイト（一致しないと無視される） |
  | [3..6] | rudder（float、リトルエンディアン） |
  | [7..10] | throttle（float） |
  | [11..14] | aileron（float） |
  | [15..18] | elevator（float） |
  | [19] | ARM ボタン |
  | [20] | FLIP ボタン |
  | [21] | 制御モード（0=角度、1=角速度） |
  | [22] | 高度モード（4=自動高度、5=手動） |
  | [23] | AHRS リセット |
  | [24] | [0..23] のチェックサム（単純加算） |

- スティック値の範囲は、throttle 0〜1（手動）、各軸 ±1 程度。±0.2 は不感帯。
- ARM は `Stick[BUTTON_ARM]==1` が約10回続くと成立する。
- 受信が約40周期途切れると自動着陸する。
- `rc.hpp` に `// #define MINIJOYC` があり、以前はMini JoyCに対応していた名残と思われる。

**次の手順（案）**
1. Mini JoyC の I²C 仕様（スティックとボタンの読み方）を確認する。
2. StickC用に、上のパケットを送る送信ファームを作る。
   - Mini JoyC のスティックは1本（2軸）しかない。残りの2軸は、StickC本体の傾き（IMU）かボタンで補う。
   - 高度は「自動高度モード（4）」で飛ばすのが安全そう。
3. StampFlyを工場ファームに戻す（`backup/` のbinを書き込む）か、公式ソースをビルドして書き込む。
4. **プロペラを外した状態で**、ARM・スロットル・停止の動作を確認してから飛行テストする。

### B. カメラ映像（保留）

- カメラモジュールを用意するまでは保留（ユーザー判断）。
- 有力案: XIAO ESP32S3 Sense や Unit CamS3 を背中に載せ、Wi-FiのMJPEG映像をPCで受ける。
- 積める重さは数g〜10g程度を目安に、実際に測って決める。

### C. 電源スイッチの有無（未確認）

- 公式ソースからは分からなかった。実機で確認し、README に書き足す。
- スイッチがなければバッテリーコネクタを抜いて電源を切る。USB接続中はマイコンが動き続ける。
