// M5GO を PC と StampFly の中継機にする
//
//   PC ──USBシリアル(115200)──> M5GO ──ESP-NOW(ch3)──> StampFly
//
// StampFly の工場ファームは Atom JoyStick と同じ 25 バイトのパケットを待っている。
// PC から届いた指令をそのパケットに変換して 50Hz で送る。
//
// 安全のしくみ:
//   * PC からの指令が 300ms 途切れたら送信を止める。StampFly は電波が切れると
//     自動着陸するので、PC が固まっても墜落しない。
//   * ボタンC = 送信停止（自動着陸）。長押し = 即時停止（モーターを止める。落ちる）
//   * Atom JoyStick は手動操縦用にそのまま使える（こちらを止めれば手動に戻せる）
//
// PC から送る行（改行区切り、値は float）:
//   C,<throttle>,<aileron>,<elevator>,<rudder>,<arm>,<flip>,<mode>,<altmode>
//     throttle : 自動高度モードでは高さの増減、手動では 0〜1
//     aileron  : 右が正 (-1〜1)
//     elevator : 前が正 (-1〜1)
//     rudder   : 右回りが正 (-1〜1)
//     arm      : 1 で離陸/着陸のトグルを1回送る（押しっぱなしにはしない）
//     flip     : 1 で宙返り
//     mode     : 0=角度制御, 1=角速度制御
//     altmode  : 4=自動高度, 5=手動高度
//   P              : 状態を1行返す
//
// M5GO が返す行:
//   S,<pc_ok>,<sent>,<recv>,<state>
//   T,<time>,<roll>,<pitch>,<yaw>,<voltage>,<altitude>,<mode>,<alt_flag>,<front_mm>,<thrust>,<duty_FL>,<duty_RR>
//   R,<roll_ref>,<pitch_ref>  機体が受け取った指令から作った目標角度[deg]。指令の向きの確認用
//     duty_FL は前左、duty_RR は後右のモーター出力（0〜1）。対角の2つだけ機体が送ってくる。
//     右へ倒す指令なら FL が増えて RR が減る。前へ倒す指令なら FL が減って RR が増える。
//     機体からのテレメトリ（約5Hz）。mode は 0=INIT 1=AVERAGE 2=FLIGHT 3=PARKING
//                                       4=LOG 5=AUTO_LANDING 6=FLIP

#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ESPNOW_CHANNEL 3
#define SEND_HZ        50
#define PC_TIMEOUT_MS  300   // これだけ指令が来なければ送信を止める
#define ARM_PULSE_MS   150   // 離陸/着陸ボタンを押している時間

// StampFly の MAC アドレス。起動時の一斉送信を受け取って自動で埋める
static uint8_t drone_mac[6] = {0};
static bool drone_known     = false;

static float f_throttle = 0, f_aileron = 0, f_elevator = 0, f_rudder = 0;
static uint8_t f_mode = 0;      // 0 = 角度制御
static uint8_t f_altmode = 4;   // 4 = 自動高度
static uint8_t f_flip = 0;

static uint32_t arm_until   = 0;  // この時刻まで arm を 1 にする
static uint32_t last_pc_ms  = 0;
static uint32_t sent_count  = 0;
static uint32_t recv_count  = 0;
static uint32_t last_recv_ms = 0;
static bool transmitting    = true;   // ボタンC で false
static bool manual_mode     = false;  // M5GO のボタンだけで操作している間は true
static String line;

// 機体からのテレメトリ（受信コールバックからはコピーだけして、loop で処理する）
static volatile bool telem_ready = false;
static uint8_t telem_buf[120];
struct Telem {
    float t, roll, pitch, yaw, voltage, altitude, thrust, duty_fl, duty_rr, roll_ref, pitch_ref;
    uint8_t alt_flag, mode;
    uint16_t front_mm;
    bool valid;
} telem = {};

static const char *mode_name(uint8_t m) {
    switch (m) {
        case 0: return "INIT";
        case 1: return "CALIB";
        case 2: return "FLIGHT";
        case 3: return "PARKING";
        case 4: return "LOG";
        case 5: return "LANDING";
        case 6: return "FLIP";
        default: return "?";
    }
}

static float telem_float(const uint8_t *d, int n) {  // n 番目の float（0 始まり）
    float v;
    memcpy(&v, d + 2 + 4 * n, 4);
    return v;
}

static void add_peer(const uint8_t *mac) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_now_del_peer(mac);
    esp_now_add_peer(&peer);
}

// StampFly からの受信。起動時の一斉送信で MAC を、飛行中はテレメトリを受け取る
static void on_recv(const uint8_t *mac, const uint8_t *data, int len) {
    recv_count++;
    last_recv_ms = millis();
    if (!drone_known) {
        memcpy(drone_mac, mac, 6);
        drone_known = true;
        add_peer(drone_mac);
    }
    // 88,88 で始まるのが飛行データ。コピーだけして loop で解釈する
    if (len >= 114 && data[0] == 88 && data[1] == 88 && !telem_ready) {
        memcpy(telem_buf, data, 114);
        telem_ready = true;
    }
}

static void parse_telemetry() {
    telem.t        = telem_float(telem_buf, 0);
    telem.roll     = telem_float(telem_buf, 2);
    telem.pitch    = telem_float(telem_buf, 3);
    telem.yaw      = telem_float(telem_buf, 4);
    telem.voltage  = telem_float(telem_buf, 14);
    telem.altitude = telem_float(telem_buf, 24);
    telem.thrust   = telem_float(telem_buf, 13);
    telem.duty_fl  = telem_float(telem_buf, 20);
    telem.duty_rr  = telem_float(telem_buf, 21);
    telem.roll_ref  = telem_float(telem_buf, 8);
    telem.pitch_ref = telem_float(telem_buf, 9);
    telem.alt_flag = telem_buf[110];
    telem.mode     = telem_buf[111];
    memcpy(&telem.front_mm, telem_buf + 112, 2);
    telem.valid    = true;
    Serial.printf("T,%.2f,%.1f,%.1f,%.1f,%.2f,%.3f,%d,%d,%d,%.3f,%.3f,%.3f\n", telem.t, telem.roll,
                  telem.pitch, telem.yaw, telem.voltage, telem.altitude, telem.mode, telem.alt_flag,
                  telem.front_mm, telem.thrust, telem.duty_fl, telem.duty_rr);
    Serial.printf("R,%.2f,%.2f\n", telem.roll_ref, telem.pitch_ref);
}

static void send_packet() {
    if (!drone_known) return;
    uint8_t d[25] = {0};
    d[0] = drone_mac[3];
    d[1] = drone_mac[4];
    d[2] = drone_mac[5];
    memcpy(&d[3], &f_rudder, 4);
    memcpy(&d[7], &f_throttle, 4);
    memcpy(&d[11], &f_aileron, 4);
    memcpy(&d[15], &f_elevator, 4);
    d[19] = (millis() < arm_until) ? 1 : 0;  // 離陸/着陸のトグル
    d[20] = f_flip;
    d[21] = f_mode;
    d[22] = f_altmode;
    d[23] = 0;  // AHRS リセット
    uint8_t sum = 0;
    for (int i = 0; i < 24; i++) sum += d[i];
    d[24] = sum;

    if (esp_now_send(drone_mac, d, sizeof(d)) == ESP_OK) sent_count++;
    f_flip = 0;  // 宙返りは1回だけ
}

static void stop_now() {
    // 最終手段。モーターを止める（飛行中なら落ちる）
    transmitting = false;
    arm_until    = 0;
    f_throttle = f_aileron = f_elevator = f_rudder = 0;
}

static void handle_line(const String &s) {
    if (s.startsWith("C,")) {
        float v[9] = {0};
        int idx = 0;
        int from = 2;
        while (idx < 9 && from < (int)s.length()) {
            int comma = s.indexOf(',', from);
            String tok = (comma < 0) ? s.substring(from) : s.substring(from, comma);
            v[idx++] = tok.toFloat();
            if (comma < 0) break;
            from = comma + 1;
        }
        manual_mode = false;   // PC が指令を送ってきたら PC 優先に戻す
        f_throttle = v[0];
        f_aileron  = v[1];
        f_elevator = v[2];
        f_rudder   = v[3];
        if (v[4] > 0.5f) arm_until = millis() + ARM_PULSE_MS;
        if (v[5] > 0.5f) f_flip = 1;
        f_mode     = (uint8_t)v[6];
        f_altmode  = (uint8_t)(v[7] > 0 ? v[7] : 4);
        last_pc_ms = millis();
        transmitting = true;
    } else if (s.startsWith("P")) {
        Serial.printf("S,%d,%lu,%lu,%s\n", (millis() - last_pc_ms < PC_TIMEOUT_MS) ? 1 : 0,
                      sent_count, recv_count, transmitting ? "TX" : "STOP");
    }
}

static void draw() {
    static uint32_t last = 0;
    if (millis() - last < 200) return;
    last = millis();

    bool pc_ok    = (millis() - last_pc_ms) < PC_TIMEOUT_MS;
    bool drone_ok = drone_known && (millis() - last_recv_ms) < 1000;

    M5.Display.fillRect(0, 40, 320, 200, TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 45);
    M5.Display.setTextColor(drone_ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
    M5.Display.printf("DRONE %s", drone_ok ? "OK  " : "----");
    M5.Display.setCursor(170, 45);
    M5.Display.setTextColor(pc_ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
    M5.Display.printf("PC %s", pc_ok ? "OK  " : "----");

    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(10, 75);
    if (drone_known) {
        M5.Display.printf("%02X:%02X:%02X:%02X:%02X:%02X", drone_mac[0], drone_mac[1], drone_mac[2],
                          drone_mac[3], drone_mac[4], drone_mac[5]);
    } else {
        M5.Display.print("waiting for drone...");
    }

    M5.Display.setTextSize(1);
    M5.Display.setCursor(180, 190);
    M5.Display.printf("thr%+.2f ail%+.2f\n", f_throttle, f_aileron);
    M5.Display.setCursor(180, 202);
    M5.Display.printf("ele%+.2f rud%+.2f", f_elevator, f_rudder);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 155);
    M5.Display.printf("alt %s  sent %lu", f_altmode == 4 ? "AUTO" : "MAN ", sent_count);

    if (telem.valid) {
        M5.Display.setCursor(10, 105);
        M5.Display.printf("%-7s %.2fV  h%.2fm", mode_name(telem.mode), telem.voltage, telem.altitude);
        M5.Display.setCursor(10, 130);
        M5.Display.printf("R%+5.1f P%+5.1f Y%+6.1f", telem.roll, telem.pitch, telem.yaw);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(180, 214);
        M5.Display.printf("FL%.2f RR%.2f", telem.duty_fl, telem.duty_rr);
        M5.Display.setTextSize(2);
    }

    M5.Display.setCursor(10, 185);
    M5.Display.setTextColor(transmitting ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
    M5.Display.printf("%s", transmitting ? "SENDING " : "STOPPED ");

    M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 220);
    M5.Display.print("A:takeoff/land   B:hover   C:stop(long=cut)");
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(1);
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 10);
    M5.Display.print("StampFly bridge");

    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) {
        M5.Display.setCursor(10, 60);
        M5.Display.setTextColor(TFT_RED);
        M5.Display.print("ESP-NOW init failed");
        while (true) delay(100);
    }
    esp_now_register_recv_cb(on_recv);

    // StampFly は起動直後に一斉送信で自分の MAC を知らせる。それを受け取るための登録
    uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    add_peer(broadcast);

    Serial.println("# M5GO StampFly bridge ready");
    Serial.printf("# my MAC %s  channel %d\n", WiFi.macAddress().c_str(), ESPNOW_CHANNEL);
}

void loop() {
    M5.update();

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            handle_line(line);
            line = "";
        } else if (c != '\r' && line.length() < 120) {
            line += c;
        }
    }

    if (M5.BtnA.wasPressed()) {          // 離陸 / 着陸
        arm_until    = millis() + ARM_PULSE_MS;
        transmitting = true;
        manual_mode  = true;   // PC が黙っていても送り続ける（ボタンだけで飛ばせる）
    }
    if (M5.BtnB.wasPressed()) {          // その場で止まる（スティックを中立に）
        f_throttle = f_aileron = f_elevator = f_rudder = 0;
    }
    if (M5.BtnC.wasPressed()) {          // 送信停止 → 機体は自動着陸
        transmitting = false;
        manual_mode  = false;
    }
    if (M5.BtnC.pressedFor(1000)) {      // 長押し = 即時停止
        stop_now();
        M5.Speaker.tone(2000, 200);
    }

    // PC からの指令が途切れたら送信を止める（機体は自動着陸に入る）。
    // ただし M5GO のボタンで操作しているときは止めない
    if (transmitting && !manual_mode && last_pc_ms != 0 && (millis() - last_pc_ms) > PC_TIMEOUT_MS) {
        transmitting = false;
    }

    if (telem_ready) {
        parse_telemetry();
        telem_ready = false;
    }

    static uint32_t last_send = 0;
    if (transmitting && millis() - last_send >= 1000 / SEND_HZ) {
        last_send = millis();
        send_packet();
    }

    draw();
}
