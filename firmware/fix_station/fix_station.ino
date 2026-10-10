/*
 * fix_station.ino - 고정형 악취 관측소 (Arduino Mega2560 PRO)
 *
 * 1초마다 센서를 읽고, REPORT_S 초 동안 평균해서 LoRa(E220)로 보낸다.
 * USB 시리얼로는 1초마다 CSV 한 줄을 내보내 책상에서 바로 확인할 수 있다.
 *
 * 센서
 *   ME3-NH3, 110-303 : 자작 증폭 회로 -> ADS1115 (A0-A3, A1-A3 차동, A3 = 2.5V 기준)
 *   SGP41            : VOC Index, NOx Index (Sensirion Gas Index Algorithm)
 *   SHT31            : 온습도 (SGP41 보상 입력으로도 씀)
 *   PMS7003          : PM1.0, PM2.5, PM10 (Serial1)
 *   DS3231           : UTC 시계
 *
 * 필요한 라이브러리 (Arduino 라이브러리 매니저)
 *   Adafruit ADS1X15 (2.x), Adafruit SHT31 Library, RTClib, Sensirion Gas Index Algorithm
 *
 * USB 명령 (115200, 줄 끝 \n)
 *   help | show | zero | set nh3_sens <nA/ppm> | set h2s_sens <nA/ppm>
 *   time <UTC epoch 초> | save | report
 *
 * 데이터 규격: moving-ver/docs/DATA_SPEC.md, 패킷: stable-ver/protocol/LORA_PACKET.md
 */
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_SHT31.h>
#include <RTClib.h>
#include <VOCGasIndexAlgorithm.h>
#include <NOxGasIndexAlgorithm.h>

#include "lora_packet.h"
#include "station_logic.h"
#include "config.h"

// ---------------------------------------------------------------- 장치
Adafruit_ADS1115 ads;
Adafruit_SHT31 sht;
RTC_DS3231 rtc;
VOCGasIndexAlgorithm voc_algo;
NOxGasIndexAlgorithm nox_algo;
logic::Pms7003Parser pms;
logic::Chamber chamber({SAMPLING_MODE == lora::MODE_CHAMBER, ZERO_INTERVAL_S, ZERO_DURATION_S,
                        ZERO_AVERAGE_S, PURGE_S});

const uint8_t SGP41_ADDR = 0x59;

// ---------------------------------------------------------------- 보정값 (EEPROM)
struct Calib {
  uint32_t magic;
  float nh3_zero_uv, h2s_zero_uv;
  float nh3_sens, h2s_sens;
  uint16_t crc;
};
const uint32_t CALIB_MAGIC = 0x4F444F32UL;   // "ODO2"
Calib calib;

logic::EcChannel ch_nh3 = {RF_OHM, NH3_SENS_NA_PPM, NH3_POLARITY, 0.0f, NH3_MAX_PPM};
logic::EcChannel ch_h2s = {RF_OHM, H2S_SENS_NA_PPM, H2S_POLARITY, 0.0f, H2S_MAX_PPM};

// ---------------------------------------------------------------- 상태
uint16_t ok_mask = 0;
uint16_t error_count = 0;
uint16_t seq = 0;
bool time_valid = false;
uint32_t uptime_s = 0;
unsigned long last_tick_ms = 0;
uint32_t last_report_uptime = 0xFFFFFFFFUL;
uint32_t last_health_uptime = (uint32_t)(10 - HEALTH_S);   // 켜고 10초 뒤 첫 상태 보고
uint8_t sgp_conditioning_left = 10;          // 켠 뒤 10초 컨디셔닝

// 평균 창
logic::Mean m_nh3, m_h2s, m_nh3_uv, m_h2s_uv, m_voc, m_nox, m_t, m_rh, m_pm1, m_pm25, m_pm10;
uint16_t window_flags = 0;
uint16_t window_seconds = 0;
logic::Mean z_nh3_uv, z_h2s_uv;              // 영점 평균 (챔버 자동 / USB zero 명령)
uint16_t manual_zero_left = 0;

// 최근 1초 값 (USB 출력용)
struct Now {
  float nh3_uv, h2s_uv, nh3, h2s, voc, nox, t, rh;
  uint16_t flags;
} now_v;

// ================================================================ 유틸
int free_ram() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

uint16_t calib_crc(const Calib& c) {
  return lora::crc16_ccitt((const uint8_t*)&c, sizeof(Calib) - sizeof(uint16_t));
}

void load_calib() {
  EEPROM.get(0, calib);
  if (calib.magic != CALIB_MAGIC || calib.crc != calib_crc(calib)) {
    calib = {CALIB_MAGIC, 0.0f, 0.0f, NH3_SENS_NA_PPM, H2S_SENS_NA_PPM, 0};
    Serial.println(F("# EEPROM 보정값 없음 -> config.h 기본값 사용"));
  } else {
    ok_mask |= lora::OK_EEPROM;
  }
  ch_nh3.zero_uv = calib.nh3_zero_uv;  ch_nh3.sens_na_per_ppm = calib.nh3_sens;
  ch_h2s.zero_uv = calib.h2s_zero_uv;  ch_h2s.sens_na_per_ppm = calib.h2s_sens;
}

void save_calib() {
  calib.magic = CALIB_MAGIC;
  calib.nh3_zero_uv = ch_nh3.zero_uv;  calib.nh3_sens = ch_nh3.sens_na_per_ppm;
  calib.h2s_zero_uv = ch_h2s.zero_uv;  calib.h2s_sens = ch_h2s.sens_na_per_ppm;
  calib.crc = calib_crc(calib);
  EEPROM.put(0, calib);
  ok_mask |= lora::OK_EEPROM;
  Serial.println(F("# 보정값 저장"));
}

uint32_t epoch_now() {
  if (!(ok_mask & lora::OK_DS3231) || !time_valid) return 0;
  return rtc.now().unixtime();
}

// ================================================================ SGP41 (I2C 직접)
// 명령 + 습도틱 + 온도틱(각각 CRC) 을 보내고 out_words 개의 16비트 값을 읽는다
bool sgp41_cmd(uint16_t cmd, uint16_t rh_t, uint16_t t_t, uint16_t* out, uint8_t out_words) {
  uint8_t b[8] = {(uint8_t)(cmd >> 8), (uint8_t)cmd, (uint8_t)(rh_t >> 8), (uint8_t)rh_t, 0,
                  (uint8_t)(t_t >> 8), (uint8_t)t_t, 0};
  b[4] = logic::sensirion_crc8(b + 2, 2);
  b[7] = logic::sensirion_crc8(b + 5, 2);
  Wire.beginTransmission(SGP41_ADDR);
  Wire.write(b, 8);
  if (Wire.endTransmission() != 0) return false;
  delay(50);
  uint8_t n = out_words * 3;
  if (Wire.requestFrom(SGP41_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < out_words; i++) {
    uint8_t w[2] = {(uint8_t)Wire.read(), (uint8_t)Wire.read()};
    if (Wire.read() != logic::sensirion_crc8(w, 2)) return false;
    out[i] = (uint16_t)(w[0] << 8 | w[1]);
  }
  return true;
}

// ================================================================ 1초 측정
void sample_once(logic::Phase phase) {
  uint16_t f = 0;

  // 온습도
  float t = NAN, rh = NAN;
  if (ok_mask & lora::OK_SHT31) {
    t = sht.readTemperature();
    rh = sht.readHumidity();
  }
  if (isnan(t) || isnan(rh)) { f |= lora::F_TH_INVALID; t = rh = NAN; }

  // SGP41: 처음 10초는 컨디셔닝, 그다음부터 원신호 -> Gas Index
  float voc = NAN, nox = NAN;
  if (ok_mask & lora::OK_SGP41) {
    uint16_t raw[2];
    if (sgp_conditioning_left > 0) {
      sgp41_cmd(0x2612, 0x8000, 0x6666, raw, 1);
      sgp_conditioning_left--;
    } else if (sgp41_cmd(0x2619, logic::sgp41_rh_ticks(rh), logic::sgp41_t_ticks(t), raw, 2)) {
      int32_t vi = voc_algo.process(raw[0]);
      int32_t ni = nox_algo.process(raw[1]);
      voc = vi > 0 ? (float)vi : NAN;    // 알고리즘 초기 학습 구간에는 0 을 준다
      nox = ni > 0 ? (float)ni : NAN;
    } else {
      error_count++;
    }
  }
  if (isnan(voc) || isnan(nox)) f |= lora::F_VOC_INVALID;

  // 전기화학 2종
  float nh3_uv = NAN, h2s_uv = NAN;
  if (ok_mask & lora::OK_ADS1115) {
    nh3_uv = ads.computeVolts(ads.readADC_Differential_0_3()) * 1e6f;
    h2s_uv = ads.computeVolts(ads.readADC_Differential_1_3()) * 1e6f;
  }
  float nh3 = logic::ec_ppm(ch_nh3, nh3_uv);
  float h2s = logic::ec_ppm(ch_h2s, h2s_uv);
  if (isnan(nh3)) f |= lora::F_NH3_INVALID;
  if (isnan(h2s)) f |= lora::F_H2S_INVALID;

  // 먼지: 최근 프레임이 3초 안에 들어왔을 때만
  static uint32_t last_pms_frames = 0;
  static uint8_t pms_stale = 0;
  if (pms.frames != last_pms_frames) { last_pms_frames = pms.frames; pms_stale = 0; ok_mask |= lora::OK_PMS7003; }
  else if (pms_stale < 255) pms_stale++;
  bool pm_ok = pms_stale <= 3 && pms.frames > 0;
  if (!pm_ok) { f |= lora::F_PM_INVALID; ok_mask &= ~lora::OK_PMS7003; }

  if (uptime_s < WARMUP_S) f |= lora::F_WARMUP;
  if (!time_valid) f |= lora::F_TIME_UNSYNCED;
  if (chamber.pump_on()) f |= lora::F_PUMP_ON;

  // 영점 평균 (챔버 자동 영점 또는 USB zero 명령)
  if (chamber.in_zero_window(uptime_s) || manual_zero_left > 0) {
    z_nh3_uv.add(nh3_uv);
    z_h2s_uv.add(h2s_uv);
    if (manual_zero_left > 0 && --manual_zero_left == 0) {
      ch_nh3.zero_uv = z_nh3_uv.get();  ch_h2s.zero_uv = z_h2s_uv.get();
      Serial.print(F("# 영점 갱신 NH3 µV=")); Serial.print(ch_nh3.zero_uv, 0);
      Serial.print(F(" H2S µV=")); Serial.println(ch_h2s.zero_uv, 0);
      z_nh3_uv.reset(); z_h2s_uv.reset();
    }
  }

  if (phase == logic::PH_ZERO) f |= lora::F_ZERO_CAL;
  if (phase == logic::PH_PURGE) f |= lora::F_PURGE;

  // 외기 측정 구간만 평균에 넣는다
  if (phase == logic::PH_SAMPLE) {
    m_nh3.add(nh3); m_h2s.add(h2s); m_nh3_uv.add(nh3_uv); m_h2s_uv.add(h2s_uv);
    m_voc.add(voc); m_nox.add(nox); m_t.add(t); m_rh.add(rh);
    if (pm_ok) { m_pm1.add(pms.pm1_0); m_pm25.add(pms.pm2_5); m_pm10.add(pms.pm10); }
  }
  window_flags |= f;
  window_seconds++;

  now_v = {nh3_uv, h2s_uv, nh3, h2s, voc, nox, t, rh, f};
}

// ================================================================ 보고
void send_frame(const uint8_t* buf, uint8_t n) {
  Serial2.write(buf, n);
  Serial2.flush();
}

void report() {
  lora::Sample s;
  s.nh3_ppm = m_nh3.get();
  s.h2s_ppm = m_h2s.get();
  s.tvoc_ppb = NAN;                // SGP41 은 ppb 를 못 낸다 (SGP30 을 달면 채움)
  s.eco2_ppm = NAN;
  s.voc_index = m_voc.get();
  s.nox_index = m_nox.get();
  s.temperature_c = m_t.get();
  s.humidity_pct = m_rh.get();
  s.pressure_hpa = NAN;
  s.pm1_0 = m_pm1.get(); s.pm2_5 = m_pm25.get(); s.pm10 = m_pm10.get();
  float nuv = m_nh3_uv.get(), huv = m_h2s_uv.get();
  s.nh3_raw_uv = isnan(nuv) ? lora::I32_NA : (int32_t)lora::round_l(nuv);
  s.h2s_raw_uv = isnan(huv) ? lora::I32_NA : (int32_t)lora::round_l(huv);
  // 평균에 들어간 값이 하나도 없으면 그 항목은 무효
  uint16_t f = window_flags;
  if (m_nh3.n) f &= ~lora::F_NH3_INVALID;
  if (m_h2s.n) f &= ~lora::F_H2S_INVALID;
  if (m_voc.n && m_nox.n) f &= ~lora::F_VOC_INVALID;
  if (m_t.n && m_rh.n) f &= ~lora::F_TH_INVALID;
  if (m_pm25.n) f &= ~lora::F_PM_INVALID;
  s.flags = f;
  s.sampling_mode = SAMPLING_MODE;
  s.pump_duty = chamber.pump_on() ? PUMP_DUTY : 0;
  s.avg_window_s = window_seconds;

  uint8_t buf[lora::MAX_FRAME];
  uint8_t n = lora::encode_sample(buf, STATION_NO, seq++, epoch_now(), s);
  send_frame(buf, n);

  m_nh3.reset(); m_h2s.reset(); m_nh3_uv.reset(); m_h2s_uv.reset(); m_voc.reset(); m_nox.reset();
  m_t.reset(); m_rh.reset(); m_pm1.reset(); m_pm25.reset(); m_pm10.reset();
  window_flags = 0;
  window_seconds = 0;
  Serial.print(F("# LoRa 보고 seq=")); Serial.println(seq - 1);
}

void report_health() {
  lora::Health h;
  h.uptime_s = uptime_s;
  h.fw_major = FW_MAJOR; h.fw_minor = FW_MINOR; h.fw_patch = FW_PATCH;
  uint16_t needed = lora::OK_ADS1115 | lora::OK_SGP41 | lora::OK_SHT31 | lora::OK_PMS7003 | lora::OK_DS3231;
  if (uptime_s < WARMUP_S) h.state = lora::STATE_WARMUP;
  else if ((ok_mask & needed) == needed) h.state = lora::STATE_OK;
  else if (ok_mask & (lora::OK_ADS1115 | lora::OK_SGP41 | lora::OK_PMS7003)) h.state = lora::STATE_DEGRADED;
  else h.state = lora::STATE_ERROR;
  h.sensor_ok_mask = ok_mask;
  h.error_count = error_count;
#if USE_VIN_SENSE
  h.vin_mv = (uint16_t)(analogRead(PIN_VIN_SENSE) * (5.0f / 1023.0f) * VIN_DIVIDER * 1000.0f);
#else
  h.vin_mv = lora::U16_NA;
#endif
  h.free_ram = (uint16_t)free_ram();
  uint8_t buf[lora::MAX_FRAME];
  uint8_t n = lora::encode_health(buf, STATION_NO, seq++, epoch_now(), h);
  send_frame(buf, n);
}

// 이 관측소의 송신 시각인가 (관측소마다 SLOT_SPACING_S 초씩 어긋나게)
bool is_report_slot() {
  uint32_t slot = ((uint32_t)STATION_NO * SLOT_SPACING_S) % REPORT_S;
  uint32_t e = epoch_now();
  uint32_t clock = e ? e : uptime_s;
  return (clock % REPORT_S) == slot;
}

// ================================================================ USB
void print_csv_header() {
  Serial.println(F("uptime_s,epoch,nh3_ppm,h2s_ppm,voc_index,nox_index,temp_c,hum_pct,pm1_0,pm2_5,pm10,nh3_uv,h2s_uv,flags,phase"));
}

void print_float(float v, uint8_t d) { if (isnan(v)) Serial.print(""); else Serial.print(v, d); Serial.print(','); }

void print_csv() {
  Serial.print(uptime_s); Serial.print(',');
  Serial.print(epoch_now()); Serial.print(',');
  print_float(now_v.nh3, 3); print_float(now_v.h2s, 4);
  print_float(now_v.voc, 0); print_float(now_v.nox, 0);
  print_float(now_v.t, 2); print_float(now_v.rh, 2);
  Serial.print(pms.pm1_0); Serial.print(','); Serial.print(pms.pm2_5); Serial.print(','); Serial.print(pms.pm10); Serial.print(',');
  print_float(now_v.nh3_uv, 0); print_float(now_v.h2s_uv, 0);
  Serial.print(now_v.flags); Serial.print(',');
  Serial.println((int)chamber.phase());
}

void show() {
  Serial.print(F("# station fix_0")); Serial.print(STATION_NO);
  Serial.print(F(" fw ")); Serial.print(FW_MAJOR); Serial.print('.'); Serial.print(FW_MINOR); Serial.print('.'); Serial.println(FW_PATCH);
  Serial.print(F("# mode ")); Serial.println(SAMPLING_MODE == lora::MODE_CHAMBER ? F("chamber") : F("diffusion"));
  Serial.print(F("# sensors mask 0x")); Serial.println(ok_mask, HEX);
  Serial.print(F("# NH3 zero µV ")); Serial.print(ch_nh3.zero_uv, 0); Serial.print(F(" sens ")); Serial.println(ch_nh3.sens_na_per_ppm, 2);
  Serial.print(F("# H2S zero µV ")); Serial.print(ch_h2s.zero_uv, 0); Serial.print(F(" sens ")); Serial.println(ch_h2s.sens_na_per_ppm, 2);
  Serial.print(F("# time ")); Serial.print(epoch_now()); Serial.println(time_valid ? F(" (UTC)") : F(" (설정 안 됨: time <epoch>)"));
  Serial.print(F("# free RAM ")); Serial.println(free_ram());
}

void handle_command(char* line) {
  char* cmd = strtok(line, " ");
  if (!cmd) return;
  if (!strcmp(cmd, "help")) {
    Serial.println(F("# help | show | zero | set nh3_sens <v> | set h2s_sens <v> | time <epoch> | save | report"));
  } else if (!strcmp(cmd, "show")) {
    show();
  } else if (!strcmp(cmd, "zero")) {
    z_nh3_uv.reset(); z_h2s_uv.reset();
    manual_zero_left = 30;
    Serial.println(F("# 30초 동안 깨끗한 공기 평균을 영점으로 잡습니다. 끝나면 save"));
  } else if (!strcmp(cmd, "set")) {
    char* key = strtok(NULL, " ");
    char* val = strtok(NULL, " ");
    if (!key || !val) { Serial.println(F("# 사용법: set nh3_sens 120.5")); return; }
    float v = atof(val);
    if (!strcmp(key, "nh3_sens")) ch_nh3.sens_na_per_ppm = v;
    else if (!strcmp(key, "h2s_sens")) ch_h2s.sens_na_per_ppm = v;
    else { Serial.println(F("# 모르는 항목")); return; }
    Serial.println(F("# 적용됨. 유지하려면 save"));
  } else if (!strcmp(cmd, "time")) {
    char* val = strtok(NULL, " ");
    if (!val || !(ok_mask & lora::OK_DS3231)) { Serial.println(F("# DS3231 없음 또는 값 없음")); return; }
    rtc.adjust(DateTime((uint32_t)strtoul(val, NULL, 10)));
    time_valid = true;
    Serial.println(F("# 시계 설정 (UTC)"));
  } else if (!strcmp(cmd, "save")) {
    save_calib();
  } else if (!strcmp(cmd, "report")) {
    report();
  } else {
    Serial.println(F("# 모르는 명령. help"));
  }
}

void poll_usb() {
  static char line[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { line[n] = 0; handle_command(line); n = 0; }
    else if (n < sizeof(line) - 1) line[n++] = c;
  }
}

// ================================================================ setup / loop
void setup() {
  Serial.begin(USB_BAUD);
  Serial1.begin(PMS_BAUD);
  Serial2.begin(LORA_BAUD);
  pinMode(PIN_PUMP_PWM, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  analogWrite(PIN_PUMP_PWM, 0);
  digitalWrite(PIN_VALVE, LOW);

  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);   // I2C 가 멈춰도 루프가 서지 않게
#endif

  if (ads.begin(0x48)) { ads.setGain(ADS_GAIN); ok_mask |= lora::OK_ADS1115; }
  if (sht.begin(0x44)) ok_mask |= lora::OK_SHT31;
  Wire.beginTransmission(SGP41_ADDR);
  if (Wire.endTransmission() == 0) ok_mask |= lora::OK_SGP41;
  if (rtc.begin()) {
    ok_mask |= lora::OK_DS3231;
    time_valid = !rtc.lostPower();
  }
  load_calib();
  if (SAMPLING_MODE == lora::MODE_CHAMBER) chamber.schedule_first_zero(WARMUP_S);

  Serial.println(F("# fix_station 시작. help 로 명령 보기"));
  show();
  print_csv_header();
  last_tick_ms = millis();
}

void loop() {
  while (Serial1.available()) pms.feed((uint8_t)Serial1.read());
  poll_usb();

  if (millis() - last_tick_ms < 1000) return;
  last_tick_ms += 1000;
  uptime_s++;

  logic::Phase ph = chamber.tick(uptime_s);
  analogWrite(PIN_PUMP_PWM, chamber.pump_on() ? PUMP_DUTY : 0);
  digitalWrite(PIN_VALVE, chamber.valve_zero() ? HIGH : LOW);

  sample_once(ph);

  if (chamber.take_zero_done()) {      // 챔버 자동 영점 결과 적용
    if (z_nh3_uv.n) ch_nh3.zero_uv = z_nh3_uv.get();
    if (z_h2s_uv.n) ch_h2s.zero_uv = z_h2s_uv.get();
    z_nh3_uv.reset(); z_h2s_uv.reset();
    save_calib();
  }

  print_csv();
  digitalWrite(PIN_STATUS_LED, uptime_s & 1);

  if (is_report_slot() && uptime_s != last_report_uptime && window_seconds >= REPORT_S / 2) {
    last_report_uptime = uptime_s;
    report();
  } else if (uptime_s - last_health_uptime >= HEALTH_S && !is_report_slot()) {
    last_health_uptime = uptime_s;
    report_health();
  }
}
