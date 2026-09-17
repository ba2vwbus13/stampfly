# 引き継ぎメモ（2026-09-15 更新）

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

### D. 外部の深度カメラで位置を測って飛ばす（方針決定、未着手）← 次回ここから

**やりたいこと**
- 部屋に置いた深度カメラで StampFly の3次元位置を測る。
- PC が位置制御を計算し、ESP-NOW で操縦指令を送って自動飛行させる。

**決まったこと（2026-09-16 更新: カメラは OAK-D に変更）**
- **カメラは OAK-D（Luxonis / DepthAI）を使う。** macOS で動作確認済み。
  - `pip install depthai==2.33.0.0`（Apple Silicon 用の配布あり）だけで導入できた。
  - USB 3（SUPER）で接続。カラー IMX378（12MP）、ステレオ OV9282 ×2、IMU BNO086。
  - 深度計算をカメラ内部で行うため Mac 側が軽い。ニューラルネットも内部で動き、検出した物体の XYZ を直接返せる。
  - 確認スクリプト: `oakd/check_device.py`（1フレーム取得、中央までの距離表示、PNG 保存）。実測で中央1.53m、有効画素58%。
- **RealSense D455 / D435i は macOS では使えない（保留）。** 下の「RealSense」節を参照。Linux 機を使うときに再開する。
- 以下は RealSense を前提に検討した内容だが、置き方と全体構成は OAK-D でもそのまま通用する。
- （旧）カメラは D455 を使う。手元には D455 と D435i がある。
  - D455 は左右カメラの間隔が95mm（D435iは50mm）。2m先の奥行き誤差がD435iの約半分になる。
  - カラーカメラがグローバルシャッターで、動く機体でも像が歪みにくい。
  - カラーと深度の視野がほぼ同じなので、画面の端まで対応が取れる。
  - D435i は0.5m以内の近距離で試すときか、2台目のカメラとして使う。
- **横から撮ってよい**（深度カメラは1台で奥行きまで測れるため）。

**置き方**
- 飛行範囲の中心から1.5〜2m離す。
- ホバリング高さより少し上に置き、10〜20°見下ろす。
- 背景は無地の壁やカーテンにする。
- 最初の飛行範囲は1m四方に限り、出たら着陸させる。

**全体の構成**

```
D455 ─USB3─▶ PC（Python）─USBシリアル─▶ 送信用ESP32（StickC）─ESP-NOW─▶ StampFly
```

- PC は位置制御だけを担当する（30〜60Hz）。姿勢の安定化は機体側が行う。
- 高さは StampFly の自動高度モード（ALTCONTROLMODE=4、下向きToF）に任せる。PC は前後・左右・向きを制御する。
- 送信用 ESP32 のファームは、タスクAの Mini JoyC 送信機とほぼ共通にできる。

**検出の流れ（案）**
1. 飛ばす前に、背景の深度を数秒記録する。
2. 毎フレーム、背景との差分から機体の候補を見つける。
3. カラー画像で位置を確かめ、周辺の深度の中央値から3次元位置を出す。
4. 最初に床の平面を1回推定し、カメラ座標を床基準の座標に変換する。
5. カルマンフィルタなどで平滑化してから制御に使う。

**注意点**
- 奥行き方向（カメラの向き）の値は揺れやすい。その方向の制御ゲインは弱めにする。
- 機体は小さく細いので、深度が欠けやすい。深度だけに頼らずカラー画像と組み合わせる。
- 向き（ヨー角）は横から見えるマーカーで取る。小さな ArUco を縦に立てるか、前後に色違いのシールを貼る。
- RealSense の赤外線投光器（850nm）が、StampFly の ToF（940nm）と干渉するかもしれない。値が乱れたら投光器を切るか弱める。
- マーカーを見失ったら「ホバリング → 着陸」にする。緊急停止は PC と送信機の両方に用意する。
- 最初はプロペラを外して、指令の向きと符号を確認する。

**進捗（2026-09-16）: 位置計測はほぼ完成。次は PC からの指令送信（タスクA）**

できたもの（`oakd/`）:
| ファイル | 内容 |
|---|---|
| `check_device.py` | OAK-D の動作確認（カラー/深度の1フレーム取得、中央距離、PNG保存） |
| `make_markers.py` / `markers.pdf` | 印刷用 ArUco マーカー（id 0=5cm, 1=4cm, 2=3cm）。+x/+y の矢印つき |
| `marker_common.py` | パイプライン、姿勢計算などの共通部品 |
| `track_marker.py` | カメラ基準でマーカーを追跡（開発・確認用） |
| `track_world.py` | **床の基準マーカーを原点とした位置・向きの追跡（本命）** |

実測した性能:
- 遅延 **37ms**、30fps、USB3(SUPER)接続時。USB2 だと 120ms 以上になるので必ず USB3 に挿す。
- 位置は静止時に mm 単位で安定。自己テスト（同一マーカーを基準と機体に指定）で位置 ±8mm、向き ±0.5度。
- 距離 1.2〜1.5m で、2.8cm のマーカーを安定検出。

機体へのマーカーの貼り方（実機を撮影して決定）:
- StampFly の上面で平らなのは Stamp-S3 モジュールの上面（約 20×17mm）、中央の空きは約 25〜30mm 四方。
- **3cm のマーカー（id 2）を薄い厚紙に貼り、Stamp-S3 の上に両面テープで固定する。**
- 4cm 以上はプロペラの吸い込み口にかかるため不可。気圧センサーの穴、前方/下向き ToF、USB 端子、バッテリーコネクタ、右の拡張基板を塞がないこと。
- カメラは 1.2〜1.5m 離し、斜め上から 40〜60 度見下ろす（真横だと平らなマーカーが見えない）。

つまずいた点と対策（同じ問題が出たら参照）:
- 遅延 0.3 秒 → カラーを NV12 の輝度面で受け取り、深度を既定で切り、キューを1にして古いフレームを捨てる。
- yaw が飛ぶ → 平面マーカーの二重解。再投影誤差＋直前フレームとの連続性で選ぶ。それでも稀に反転するので、外れ値除去（回転速度の上限）を入れてある。
- 動かすと見失う → 自動露出だとぶれる。手動露出（既定 6ms / ISO800）にしてある。暗い場所では `--iso 1600`。

**実機で確認済み（2026-09-16）**
- PC → M5GO → 機体の指令経路が動作。離陸/着陸、姿勢指令とも正常。
- **指令の向きは正しい。`--flip-roll` も `--flip-pitch` も不要。**
  （機体の前を向こう向きにして後ろに立ち、d=右へ、w=前へ動くことを確認）
- **機体の「前」は USB 端子の反対側**。向きを取り違えやすいので注意。
- **Atom JoyStick の電源は必ず切る。** 入っていると 50Hz で arm=0 を送り続け、
  PC からの離陸指令が打ち消されて離陸しない。
- **飛ばす前に必ず機体のリセットボタンを押し、初期化中(LEDが紫の約10秒)は動かさない。**
  これを怠ると、ジャイロのゼロ点が狂ったまま飛び、離陸直後に斜めへ突進する。
  2026-09-16 の飛行はこれが主因だった（Atom JoyStick で操作しても同じ症状が出て、
  リセットボタンを押したら正常に飛んだ）。
  - リセットボタン（再起動）= ジャイロのゼロ点を測り直す。**これが効く**
  - `c` キー / M5GO の B ボタン長押し（AHRSリセット）= 姿勢推定だけを初期化。
    飛行中に少しずつずれてきたときの補正用で、ゼロ点ずれには効かない
- 手動では左へ流れる（重心やモーター個体差による。小型機では普通）。
  位置制御の積分項が補正するので、自動飛行では問題にならない。

**次回の手順**
1. タスクA の送信機を作り、PC から StampFly へ ESP-NOW で指令を送れるようにする（完了）。
2. プロペラを外した状態で、PC からの指令が正しい向き・符号で効くか確認する。
3. `track_world.py` の位置と組み合わせ、高度は機体の自動高度モードに任せて、1点ホバリングの位置制御を試す。

**macOS への pyrealsense2 導入（2026-09-16 実施済み・再現手順）**

macOS (Apple Silicon) には `pyrealsense2` の配布が無いため、ソースからビルドする。Homebrew の `librealsense` は C++ ライブラリのみで Python バインディングが入らない。

```bash
brew install cmake libusb pkg-config
git clone --depth 1 --branch v2.56.5 https://github.com/IntelRealSense/librealsense.git
cd librealsense
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE=$(which python3) \
  -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF \
  -DFORCE_RSUSB_BACKEND=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build build -j $(sysctl -n hw.ncpu)      # 10分程度
```

ビルドで `build/Release/` に `pyrealsense2*.so` と `librealsense2*.dylib` ができる。恒久的な場所へ置き、site-packages からパスを通す。

```bash
mkdir -p ~/.local/lib/realsense
cp -a build/Release/pyrealsense2*.so build/Release/librealsense2*.dylib ~/.local/lib/realsense/
python3 -c "import site;print(site.getsitepackages()[0])"   # 出力先に realsense.pth を作る
echo ~/.local/lib/realsense > <site-packages>/realsense.pth
```

補足:
- `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` は CMake 4 系で古い記述をビルドするために必要。
- `-DFORCE_RSUSB_BACKEND=ON` は macOS で必須。
- 学内ネットワークではプロキシ指定が必要。

**macOS で RealSense は動かなかった（結論・2026-09-16）**

- D455 は macOS からは UVC カメラとして見えるが、`pyrealsense2` から開くと `RuntimeError: failed to set power state`。
- `sudo` で実行すると segmentation fault（即死）。ユーザーのターミナルから実行してもカメラ許可のダイアログは出ず、USB を開く段階で失敗している。
- macOS + Apple Silicon + 最新 macOS の組み合わせは実質未対応と判断し、**OAK-D に切り替えた**。
- Linux 機で使う場合は `pip install pyrealsense2` だけで済むので、そのとき再開する。

**（参考）macOS で試したときの詳細**

- D455 は macOS 側からは UVC カメラとして見えている（`system_profiler SPCameraDataType` に "Intel(R) RealSense(TM) Depth Camera 455 Depth" が出る）。
- しかし `pyrealsense2` から開こうとすると `RuntimeError: failed to set power state` になる。
- 原因の候補: プロセスにカメラ利用の許可（TCC）が無い、macOS の UVC ドライバがデバイスを掴んでいる、USB 2 接続。
- 試すこと:
  1. **ユーザー自身のターミナルから** `python3 realsense/check_device.py` を実行する（許可ダイアログはターミナルアプリに対して出る）。
  2. システム設定 → プライバシーとセキュリティ → カメラ で、ターミナルを許可する。
  3. `sudo python3 realsense/check_device.py` を試す。
  4. それでも駄目なら Linux 機に移る（`pip install pyrealsense2` だけで済む）。
- 確認用スクリプトは `realsense/check_device.py`（デバイス情報の表示と、1フレーム取得、画面中央までの距離表示）。

**優先順位の目安**
- D（位置の表示）と A（送信機）は並行して進められる。
- 飛行には両方が必要。

## 5. 関連する機材

| 機材 | 状態 |
|---|---|
| StampFly | センサーモニターのファーム入り。飛ばすには工場ファームか公式ソースを書き込む |
| M5StickC系 + Mini JoyC HAT | 送信機に使う予定（タスクA） |
| RealSense D455 | 外部位置計測に使う（タスクD） |
| RealSense D435i | 予備。近距離テストか2台目に使う |
| カメラモジュール（機体に載せる用） | 未入手。用意するまでタスクBは保留 |
