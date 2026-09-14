// StampFly センサーモニター
//
// 全センサーを読み、USBシリアルへ 50Hz で CSV を送る。モーターは常に停止。
// 出力行:
//   "#..."  コメント（起動ログ・ヘッダ）
//   "D,..." データ行（列は HEADER を参照）
//
// 機体座標系（公式ファームと同じ変換）: X=前, Y=右, Z=下

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "common.h"
#include "bmi2.h"
#include <bmi270.h>
#include <vl53lx_api.h>
#include <vl53lx_platform.h>

// ---- ピン ----
#define SDA_PIN      3
#define SCL_PIN      4
#define XSHUT_BOTTOM 7
#define XSHUT_FRONT  9
#define INT_BOTTOM   6
#define INT_FRONT    8
static const int MOTOR_PINS[4] = {5, 42, 10, 41};

// ---- I2C アドレス ----
#define ADDR_BMM150  0x10
#define ADDR_INA3221 0x40
#define ADDR_BMP280  0x76

#define DPS20002RAD 34.90658504  // ±2000 dps を rad/s で表した値（公式 imu.hpp）

#define SAMPLE_HZ 200
#define OUTPUT_HZ 50

static const char *HEADER =
    "#HEADER,t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,"
    "mx_uT,my_uT,mz_uT,heading_deg,press_hPa,temp_C,baro_alt_m,"
    "tof_bottom_mm,tof_front_mm,vbat_V";

// ============================================================
// I2C ヘルパー
// ============================================================
static bool i2c_write8(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    Wire1.write(val);
    return Wire1.endTransmission() == 0;
}

static bool i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom(addr, (uint8_t)len) != len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire1.read();
    return true;
}

// ============================================================
// BMP280 気圧・温度
// ============================================================
static struct {
    uint16_t T1;
    int16_t T2, T3;
    uint16_t P1;
    int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} bmp_cal;
static bool bmp_ok = false;

static bool bmp280_init() {
    uint8_t id;
    if (!i2c_read(ADDR_BMP280, 0xD0, &id, 1) || id != 0x58) return false;
    uint8_t c[24];
    if (!i2c_read(ADDR_BMP280, 0x88, c, 24)) return false;
    auto u16 = [&](int i) { return (uint16_t)(c[i] | (c[i + 1] << 8)); };
    bmp_cal.T1 = u16(0);
    bmp_cal.T2 = (int16_t)u16(2);
    bmp_cal.T3 = (int16_t)u16(4);
    bmp_cal.P1 = u16(6);
    bmp_cal.P2 = (int16_t)u16(8);
    bmp_cal.P3 = (int16_t)u16(10);
    bmp_cal.P4 = (int16_t)u16(12);
    bmp_cal.P5 = (int16_t)u16(14);
    bmp_cal.P6 = (int16_t)u16(16);
    bmp_cal.P7 = (int16_t)u16(18);
    bmp_cal.P8 = (int16_t)u16(20);
    bmp_cal.P9 = (int16_t)u16(22);
    i2c_write8(ADDR_BMP280, 0xF5, 0x10);  // standby 0.5ms, IIR filter x16
    i2c_write8(ADDR_BMP280, 0xF4, 0x57);  // temp x2, press x16, normal mode
    return true;
}

// データシート記載の浮動小数点補償式
static bool bmp280_read(float &press_hPa, float &temp_C) {
    uint8_t d[6];
    if (!i2c_read(ADDR_BMP280, 0xF7, d, 6)) return false;
    int32_t adc_P = ((int32_t)d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
    int32_t adc_T = ((int32_t)d[3] << 12) | (d[4] << 4) | (d[5] >> 4);

    double v1 = (adc_T / 16384.0 - bmp_cal.T1 / 1024.0) * bmp_cal.T2;
    double v2 = (adc_T / 131072.0 - bmp_cal.T1 / 8192.0);
    v2 = v2 * v2 * bmp_cal.T3;
    double t_fine = v1 + v2;
    temp_C = t_fine / 5120.0;

    v1 = t_fine / 2.0 - 64000.0;
    v2 = v1 * v1 * bmp_cal.P6 / 32768.0;
    v2 = v2 + v1 * bmp_cal.P5 * 2.0;
    v2 = v2 / 4.0 + bmp_cal.P4 * 65536.0;
    v1 = (bmp_cal.P3 * v1 * v1 / 524288.0 + bmp_cal.P2 * v1) / 524288.0;
    v1 = (1.0 + v1 / 32768.0) * bmp_cal.P1;
    if (v1 == 0.0) return false;
    double p = 1048576.0 - adc_P;
    p = (p - v2 / 4096.0) * 6250.0 / v1;
    v1 = bmp_cal.P9 * p * p / 2147483648.0;
    v2 = p * bmp_cal.P8 / 32768.0;
    p = p + (v1 + v2 + bmp_cal.P7) / 16.0;
    press_hPa = p / 100.0;
    return true;
}

// ============================================================
// BMM150 地磁気
// ============================================================
static struct {
    int8_t x1, y1, x2, y2, xy2;
    uint8_t xy1;
    int16_t z2, z3, z4;
    uint16_t z1, xyz1;
} bmm_trim;
static bool bmm_ok = false;

static bool bmm150_init() {
    i2c_write8(ADDR_BMM150, 0x4B, 0x01);  // power on (sleep mode)
    delay(5);
    uint8_t id;
    if (!i2c_read(ADDR_BMM150, 0x40, &id, 1) || id != 0x32) return false;

    uint8_t a[2], b[4], c[10];
    if (!i2c_read(ADDR_BMM150, 0x5D, a, 2)) return false;   // x1, y1
    if (!i2c_read(ADDR_BMM150, 0x62, b, 4)) return false;   // z4, x2, y2
    if (!i2c_read(ADDR_BMM150, 0x68, c, 10)) return false;  // z2, z1, xyz1, z3, xy2, xy1
    bmm_trim.x1   = (int8_t)a[0];
    bmm_trim.y1   = (int8_t)a[1];
    bmm_trim.z4   = (int16_t)(b[0] | (b[1] << 8));
    bmm_trim.x2   = (int8_t)b[2];
    bmm_trim.y2   = (int8_t)b[3];
    bmm_trim.z2   = (int16_t)(c[0] | (c[1] << 8));
    bmm_trim.z1   = (uint16_t)(c[2] | (c[3] << 8));
    bmm_trim.xyz1 = (uint16_t)(c[4] | ((c[5] & 0x7F) << 8));
    bmm_trim.z3   = (int16_t)(c[6] | (c[7] << 8));
    bmm_trim.xy2  = (int8_t)c[8];
    bmm_trim.xy1  = c[9];

    i2c_write8(ADDR_BMM150, 0x51, 4);     // XY repetitions = 9 (regular preset)
    i2c_write8(ADDR_BMM150, 0x52, 14);    // Z repetitions = 15
    i2c_write8(ADDR_BMM150, 0x4C, 0x38);  // normal mode, 30Hz
    return true;
}

static float bmm_comp_xy(int16_t raw, uint16_t rhall, int8_t t1, int8_t t2) {
    double c0 = bmm_trim.xyz1 * 16384.0 / rhall;
    double c1 = c0 - 16384.0;
    double c2 = bmm_trim.xy2 * (c1 * c1 / 268435456.0);
    double c3 = c2 + c1 * (bmm_trim.xy1 / 16384.0);
    double c4 = t2 + 160.0;
    double c5 = raw * ((c3 + 256.0) * c4);
    return (c5 / 8192.0 + t1 * 8.0) / 16.0;
}

// Bosch 公式ドライバの浮動小数点補償式（単位 uT）
static bool bmm150_read(float &mx, float &my, float &mz) {
    uint8_t d[8];
    if (!i2c_read(ADDR_BMM150, 0x42, d, 8)) return false;
    int16_t rx    = (int16_t)((d[1] << 8) | d[0]) >> 3;
    int16_t ry    = (int16_t)((d[3] << 8) | d[2]) >> 3;
    int16_t rz    = (int16_t)((d[5] << 8) | d[4]) >> 1;
    uint16_t rhall = (uint16_t)((d[7] << 8) | d[6]) >> 2;
    if (rhall == 0 || bmm_trim.xyz1 == 0 || rx == -4096 || ry == -4096 || rz == -16384) return false;

    mx = bmm_comp_xy(rx, rhall, bmm_trim.x1, bmm_trim.x2);
    my = bmm_comp_xy(ry, rhall, bmm_trim.y1, bmm_trim.y2);

    double z0 = rz - bmm_trim.z4;
    double z1 = (double)rhall - bmm_trim.xyz1;
    double z2 = bmm_trim.z3 * z1;
    double z3 = bmm_trim.z1 * rhall / 32768.0;
    double z4 = bmm_trim.z2 + z3;
    mz = ((z0 * 131072.0 - z2) / (z4 * 4.0)) / 16.0;
    return true;
}

// ============================================================
// INA3221 電圧（公式ファームと同じ CH2 = バッテリー）
// ============================================================
static bool ina3221_read_vbat(float &v) {
    uint8_t d[2];
    if (!i2c_read(ADDR_INA3221, 0x04, d, 2)) return false;  // CH2 bus voltage
    int16_t raw = (int16_t)((d[0] << 8) | d[1]);
    v = (raw >> 3) * 0.008f;
    return true;
}

// ============================================================
// VL53L3CX ToF ×2
// ============================================================
static VL53LX_Dev_t tof_bottom_dev, tof_front_dev;
static VL53LX_DEV ToF_bottom = &tof_bottom_dev;
static VL53LX_DEV ToF_front  = &tof_front_dev;

// 0x29/0x2A への応答確認
static bool i2c_ack(uint8_t addr) {
    Wire1.beginTransmission(addr);
    return Wire1.endTransmission() == 0;
}

static bool tof_setup(const char *name, VL53LX_DEV dev, uint8_t mode) {
    int st = VL53LX_WaitDeviceBooted(dev);
    if (!st) st = VL53LX_DataInit(dev);
    if (!st) st = VL53LX_SetDistanceMode(dev, mode);
    if (!st) st = VL53LX_SetMeasurementTimingBudgetMicroSeconds(dev, 33000);
    if (!st) st = VL53LX_StartMeasurement(dev);
    USBSerial.printf("#ToF %s (0x%02X): %s\n", name, dev->i2c_slave_address, st ? "FAIL" : "OK");
    return st == 0;
}

static bool tof_front_enabled = true;

// ST のサンプルと同じ順序: 前方だけ起こしてアドレス 0x2A へ → 下向きを起こす → 両方初期化
static void tof_init() {
    tof_bottom_dev.comms_speed_khz   = 400;
    tof_bottom_dev.i2c_slave_address = 0x29;
    tof_front_dev.comms_speed_khz    = 400;
    tof_front_dev.i2c_slave_address  = 0x29;

    pinMode(XSHUT_BOTTOM, OUTPUT);
    pinMode(XSHUT_FRONT, OUTPUT);
    pinMode(INT_BOTTOM, INPUT);
    pinMode(INT_FRONT, INPUT);
    digitalWrite(XSHUT_BOTTOM, LOW);
    digitalWrite(XSHUT_FRONT, LOW);
    delay(50);

    digitalWrite(XSHUT_FRONT, HIGH);
    delay(100);
    VL53LX_SetDeviceAddress(ToF_front, 0x2A * 2);  // API は 8bit 表記
    tof_front_dev.i2c_slave_address = 0x2A;
    delay(10);
    if (!i2c_ack(0x2A)) {
        USBSerial.println("#WARN front ToF did not move to 0x2A; disabled");
        digitalWrite(XSHUT_FRONT, LOW);
        tof_front_enabled = false;
    }

    digitalWrite(XSHUT_BOTTOM, HIGH);
    delay(100);
    tof_setup("bottom", ToF_bottom, VL53LX_DISTANCEMODE_MEDIUM);
    if (tof_front_enabled) tof_front_enabled = tof_setup("front", ToF_front, VL53LX_DISTANCEMODE_LONG);
}

// 前方 ToF がリセットすると既定の 0x29 に戻り、下向き ToF とアドレスが衝突する。
// 0x2A が応答しなくなったら前方を XSHUT で止めて下向きを守る
static void tof_front_watchdog() {
    if (!tof_front_enabled || i2c_ack(0x2A)) return;
    digitalWrite(XSHUT_FRONT, LOW);
    tof_front_enabled = false;
    USBSerial.println("#WARN front ToF reset (address reverted to 0x29); disabled");
}

// 新しい測定値があれば out に入れて true。有効な対象なしは -1
static bool tof_poll(VL53LX_DEV dev, int16_t &out) {
    uint8_t ready = 0;
    if (VL53LX_GetMeasurementDataReady(dev, &ready) != 0 || !ready) return false;
    VL53LX_MultiRangingData_t data;
    VL53LX_GetMultiRangingData(dev, &data);
    int16_t best = -1;
    for (uint8_t i = 0; i < data.NumberOfObjectsFound; i++) {
        if (data.RangeData[i].RangeStatus == VL53LX_RANGESTATUS_RANGE_VALID &&
            data.RangeData[i].RangeMilliMeter > best) {
            best = data.RangeData[i].RangeMilliMeter;
        }
    }
    VL53LX_ClearInterruptAndStartMeasurement(dev);
    out = best;
    return true;
}

// ============================================================
// BMI270 IMU
// ============================================================
static bool imu_init() {
    pinMode(46, OUTPUT);
    digitalWrite(46, HIGH);
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);
    delay(5);
    if (spi_init() != ESP_OK) return false;
    bmi270_dev_init();
    if (bmi270_init(pBmi270) != 0) return false;
    if (set_accel_gyro_config(pBmi270) != 0) return false;
    uint8_t sensors[2] = {BMI2_ACCEL, BMI2_GYRO};
    return bmi2_sensor_enable(sensors, 2, pBmi270) == 0;
}

// ============================================================
// 状態
// ============================================================
static float ax, ay, az;          // [G] 機体座標
static float gx, gy, gz;          // [deg/s]
static float gbx, gby, gbz;       // ジャイロのバイアス
static float roll, pitch;         // [deg] 相補フィルタ
static float mx, my, mz, heading;
static float press = NAN, temp = NAN, press0 = NAN, baro_alt = NAN;
static float vbat = NAN;
static int16_t tof_b = -1, tof_f = -1;

static void imu_sample(bool apply_bias) {
    struct bmi2_sens_data d;
    bmi2_get_sensor_data(&d, pBmi270);
    float sax = lsb_to_mps2(d.acc.x, 8.0, 16) / GRAVITY_EARTH;
    float say = lsb_to_mps2(d.acc.y, 8.0, 16) / GRAVITY_EARTH;
    float saz = lsb_to_mps2(d.acc.z, 8.0, 16) / GRAVITY_EARTH;
    float sgx = lsb_to_rps(d.gyr.x, DPS20002RAD, 16) * RAD_TO_DEG;
    float sgy = lsb_to_rps(d.gyr.y, DPS20002RAD, 16) * RAD_TO_DEG;
    float sgz = lsb_to_rps(d.gyr.z, DPS20002RAD, 16) * RAD_TO_DEG;
    // センサー座標 → 機体座標（公式 sensor.cpp と同じ）
    ax = say;
    ay = sax;
    az = -saz;
    gx = sgy;
    gy = sgx;
    gz = -sgz;
    if (apply_bias) {
        gx -= gbx;
        gy -= gby;
        gz -= gbz;
    }
}

static void calibrate_gyro() {
    const int N = 400;
    double sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < N; i++) {
        imu_sample(false);
        sx += gx;
        sy += gy;
        sz += gz;
        delay(1000 / SAMPLE_HZ);
    }
    gbx = sx / N;
    gby = sy / N;
    gbz = sz / N;
}

static void update_attitude(float dt) {
    // 静止時 Z下向き軸では az ≈ -1G
    float acc_roll  = atan2(-ay, -az) * RAD_TO_DEG;
    float acc_pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    const float k   = 0.98f;
    roll  = k * (roll + gx * dt) + (1 - k) * acc_roll;
    pitch = k * (pitch + gy * dt) + (1 - k) * acc_pitch;
}

// 傾き補償つき方位（未キャリブレーション・センサー座標のまま）
static void update_heading() {
    float r = roll * DEG_TO_RAD, p = pitch * DEG_TO_RAD;
    float xh = mx * cos(p) + my * sin(r) * sin(p) + mz * cos(r) * sin(p);
    float yh = my * cos(r) - mz * sin(r);
    heading = atan2(-yh, xh) * RAD_TO_DEG;
    if (heading < 0) heading += 360;
}

// ============================================================
void setup() {
    // 安全のため最初にモーターを確実に停止
    for (int pin : MOTOR_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    USBSerial.begin(115200);
    delay(1500);
    USBSerial.println("#StampFly sensor monitor (motors disabled)");

    Wire1.begin(SDA_PIN, SCL_PIN, 400000UL);

    tof_init();

    bool imu_ok = imu_init();
    USBSerial.printf("#IMU BMI270: %s\n", imu_ok ? "OK" : "FAIL");
    bmp_ok = bmp280_init();
    USBSerial.printf("#BARO BMP280: %s\n", bmp_ok ? "OK" : "FAIL");
    bmm_ok = bmm150_init();
    USBSerial.printf("#MAG BMM150: %s\n", bmm_ok ? "OK" : "FAIL");
    float v;
    USBSerial.printf("#POWER INA3221: %s\n", ina3221_read_vbat(v) ? "OK" : "FAIL");

    if (imu_ok) {
        USBSerial.println("#Calibrating gyro... keep still (2 s)");
        calibrate_gyro();
        USBSerial.printf("#Gyro bias [dps]: %.3f %.3f %.3f\n", gbx, gby, gbz);
        imu_sample(true);
        roll  = atan2(-ay, -az) * RAD_TO_DEG;
        pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    }

    // 気圧高度の基準（起動地点 = 0 m）
    if (bmp_ok) {
        delay(200);
        double sum = 0;
        int n = 0;
        for (int i = 0; i < 20; i++) {
            float p, t;
            if (bmp280_read(p, t)) {
                sum += p;
                n++;
            }
            delay(40);
        }
        if (n) press0 = sum / n;
    }

    USBSerial.println(HEADER);
}

void loop() {
    static uint32_t last_sample = micros();
    static uint32_t last_out    = 0;
    static uint32_t last_slow   = 0;
    static uint32_t tick        = 0;

    uint32_t now = micros();
    if (now - last_sample < 1000000UL / SAMPLE_HZ) return;
    float dt    = (now - last_sample) * 1e-6f;
    last_sample = now;

    imu_sample(true);
    update_attitude(dt);

    // ToF は2台を交互にポーリング（I2C 占有時間を分散）
    if ((tick++ & 1) && tof_front_enabled) {
        tof_poll(ToF_front, tof_f);
    } else {
        tof_poll(ToF_bottom, tof_b);
    }

    uint32_t ms = millis();
    if (ms - last_slow >= 40) {  // 25Hz
        last_slow = ms;
        if (bmp_ok && bmp280_read(press, temp) && !isnan(press0)) {
            baro_alt = 44330.0f * (1.0f - pow(press / press0, 0.1903f));
        }
        if (bmm_ok && bmm150_read(mx, my, mz)) update_heading();
        ina3221_read_vbat(vbat);
    }

    static uint32_t last_check = 0;
    if (ms - last_check >= 500) {
        last_check = ms;
        tof_front_watchdog();
    }

    if (ms - last_out >= 1000 / OUTPUT_HZ) {
        last_out = ms;
        USBSerial.printf("D,%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.3f,%d,%d,%.3f\n",
                         ms, ax, ay, az, gx, gy, gz, roll, pitch, mx, my, mz, heading, press, temp, baro_alt,
                         tof_b, tof_f, vbat);
    }
}
