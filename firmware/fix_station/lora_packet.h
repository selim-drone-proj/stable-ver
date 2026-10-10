// lora_packet.h - 고정형 관측소 -> 게이트웨이 LoRa 패킷 v1 (헤더만 있는 라이브러리)
//
// Mega2560 과 PC(g++) 양쪽에서 그대로 컴파일된다. 아두이노 의존성 없음.
// 규격 문서: stable-ver/protocol/LORA_PACKET.md, moving-ver/docs/DATA_SPEC.md 8장
// 파이썬 짝: stable-ver/gateway/odor_gateway/lora_codec.py  (둘을 같이 고칠 것)
//
// 프레임 (리틀 엔디언)
//   [0]  0xA5  [1] 0x5A        동기 바이트
//   [2]  version (=1)
//   [3]  type   0x01 SAMPLE, 0x02 HEALTH
//   [4]  station 번호 1~99   (fix_01 = 1)
//   [5]  seq     uint16      보낼 때마다 +1
//   [7]  time    uint32      UTC epoch 초. 0 = 시계 모름
//   [11] len     uint8       페이로드 길이
//   [12] payload
//   [12+len] crc uint16      CRC-16/CCITT-FALSE, [2] 부터 페이로드 끝까지
#pragma once
#include <stdint.h>
#include <math.h>

namespace lora {

const uint8_t SYNC0 = 0xA5;
const uint8_t SYNC1 = 0x5A;
const uint8_t VERSION = 1;
const uint8_t TYPE_SAMPLE = 0x01;
const uint8_t TYPE_HEALTH = 0x02;

const uint8_t HEADER_LEN = 12;
const uint8_t CRC_LEN = 2;
const uint8_t SAMPLE_LEN = 38;
const uint8_t HEALTH_LEN = 16;
const uint8_t MAX_FRAME = HEADER_LEN + SAMPLE_LEN + CRC_LEN;   // 52

// 값 없음 표시 (센서 없음, 무효, 범위 밖이 아니라 "모름")
const uint16_t U16_NA = 0xFFFF;
const int16_t  I16_NA = -32768;
const int32_t  I32_NA = (int32_t)0x80000000UL;

// 품질 비트: GasReading.flags 와 같은 뜻 (DATA_SPEC.md 4.2)
const uint16_t F_WARMUP        = 1u << 0;
const uint16_t F_NH3_INVALID   = 1u << 1;
const uint16_t F_H2S_INVALID   = 1u << 2;
const uint16_t F_VOC_INVALID   = 1u << 3;
const uint16_t F_TH_INVALID    = 1u << 4;
const uint16_t F_PM_INVALID    = 1u << 5;
const uint16_t F_TIME_UNSYNCED = 1u << 6;
const uint16_t F_ZERO_CAL      = 1u << 7;
const uint16_t F_PURGE         = 1u << 8;
const uint16_t F_PUMP_ON       = 1u << 9;
const uint16_t F_TEMP_COMP     = 1u << 10;
const uint16_t F_SIMULATED     = 1u << 11;

const uint8_t MODE_DIFFUSION = 0;
const uint8_t MODE_CHAMBER = 1;

const uint8_t STATE_BOOT = 0, STATE_WARMUP = 1, STATE_OK = 2, STATE_DEGRADED = 3, STATE_ERROR = 4;

// 고정형 sensor_ok_mask 비트 (DATA_SPEC.md 4.6)
const uint16_t OK_ADS1115 = 1u << 0;
const uint16_t OK_SGP41   = 1u << 1;
const uint16_t OK_SHT31   = 1u << 2;
const uint16_t OK_PMS7003 = 1u << 3;
const uint16_t OK_DS3231  = 1u << 4;
const uint16_t OK_EEPROM  = 1u << 5;

struct Sample {
  float nh3_ppm;        // 0.01 ppm 단위로 실음
  float h2s_ppm;        // 0.001 ppm
  float tvoc_ppb;       // 1 ppb
  float eco2_ppm;       // 1 ppm
  float voc_index;      // 1
  float nox_index;      // 1
  float temperature_c;  // 0.01 ℃ (부호 있음)
  float humidity_pct;   // 0.01 %
  float pressure_hpa;   // 0.1 hPa
  float pm1_0;          // 1 µg/m³
  float pm2_5;
  float pm10;
  int32_t nh3_raw_uv;   // 증폭 회로 출력 µV. I32_NA = 없음
  int32_t h2s_raw_uv;
  uint16_t flags;
  uint8_t sampling_mode;
  uint8_t pump_duty;    // 0~255
  uint16_t avg_window_s;
};

struct Health {
  uint32_t uptime_s;
  uint8_t fw_major, fw_minor, fw_patch;
  uint8_t state;
  uint16_t sensor_ok_mask;
  uint16_t error_count;
  uint16_t vin_mv;      // U16_NA = 측정 안 함
  uint16_t free_ram;
};

// NaN 을 넣을 때 쓴다 (아두이노와 PC 모두 같은 값)
inline float na() { return NAN; }

inline uint16_t crc16_ccitt(const uint8_t* d, uint16_t n) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

inline void put_u8(uint8_t* p, uint8_t v) { p[0] = v; }
inline void put_u16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
inline void put_i16(uint8_t* p, int16_t v) { put_u16(p, (uint16_t)v); }
inline void put_u32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF); p[3] = (uint8_t)((v >> 24) & 0xFF);
}
inline void put_i32(uint8_t* p, int32_t v) { put_u32(p, (uint32_t)v); }

// 반올림 (AVR 에 lroundf 가 없을 수 있어 직접)
inline long round_l(float x) { return (long)(x >= 0 ? x + 0.5f : x - 0.5f); }

// float -> 부호 없는 16비트. NaN 이면 U16_NA, 음수는 0, 넘치면 65534 로 자른다
inline uint16_t enc_u16(float v, float lsb) {
  if (isnan(v)) return U16_NA;
  float x = v / lsb;
  if (x < 0) x = 0;
  if (x > 65534.0f) x = 65534.0f;
  return (uint16_t)round_l(x);
}

// float -> 부호 있는 16비트. NaN 이면 I16_NA, 넘치면 ±32767
inline int16_t enc_i16(float v, float lsb) {
  if (isnan(v)) return I16_NA;
  float x = v / lsb;
  if (x < -32767.0f) x = -32767.0f;
  if (x > 32767.0f) x = 32767.0f;
  return (int16_t)round_l(x);
}

inline uint8_t write_header(uint8_t* buf, uint8_t type, uint8_t station, uint16_t seq,
                            uint32_t epoch_s, uint8_t len) {
  buf[0] = SYNC0;
  buf[1] = SYNC1;
  put_u8(buf + 2, VERSION);
  put_u8(buf + 3, type);
  put_u8(buf + 4, station);
  put_u16(buf + 5, seq);
  put_u32(buf + 7, epoch_s);
  put_u8(buf + 11, len);
  return HEADER_LEN;
}

inline uint8_t finish(uint8_t* buf, uint8_t payload_len) {
  uint16_t crc = crc16_ccitt(buf + 2, (uint16_t)(HEADER_LEN - 2 + payload_len));
  put_u16(buf + HEADER_LEN + payload_len, crc);
  return (uint8_t)(HEADER_LEN + payload_len + CRC_LEN);
}

// buf 는 MAX_FRAME 바이트 이상. 프레임 길이를 돌려준다
inline uint8_t encode_sample(uint8_t* buf, uint8_t station, uint16_t seq, uint32_t epoch_s,
                             const Sample& s) {
  write_header(buf, TYPE_SAMPLE, station, seq, epoch_s, SAMPLE_LEN);
  uint8_t* p = buf + HEADER_LEN;
  put_u16(p + 0,  enc_u16(s.nh3_ppm, 0.01f));
  put_u16(p + 2,  enc_u16(s.h2s_ppm, 0.001f));
  put_u16(p + 4,  enc_u16(s.tvoc_ppb, 1.0f));
  put_u16(p + 6,  enc_u16(s.eco2_ppm, 1.0f));
  put_u16(p + 8,  enc_u16(s.voc_index, 1.0f));
  put_u16(p + 10, enc_u16(s.nox_index, 1.0f));
  put_i16(p + 12, enc_i16(s.temperature_c, 0.01f));
  put_u16(p + 14, enc_u16(s.humidity_pct, 0.01f));
  put_u16(p + 16, enc_u16(s.pressure_hpa, 0.1f));
  put_u16(p + 18, enc_u16(s.pm1_0, 1.0f));
  put_u16(p + 20, enc_u16(s.pm2_5, 1.0f));
  put_u16(p + 22, enc_u16(s.pm10, 1.0f));
  put_i32(p + 24, s.nh3_raw_uv);
  put_i32(p + 28, s.h2s_raw_uv);
  put_u16(p + 32, s.flags);
  put_u8(p + 34, s.sampling_mode);
  put_u8(p + 35, s.pump_duty);
  put_u16(p + 36, s.avg_window_s);
  return finish(buf, SAMPLE_LEN);
}

inline uint8_t encode_health(uint8_t* buf, uint8_t station, uint16_t seq, uint32_t epoch_s,
                             const Health& h) {
  write_header(buf, TYPE_HEALTH, station, seq, epoch_s, HEALTH_LEN);
  uint8_t* p = buf + HEADER_LEN;
  put_u32(p + 0, h.uptime_s);
  put_u8(p + 4, h.fw_major);
  put_u8(p + 5, h.fw_minor);
  put_u8(p + 6, h.fw_patch);
  put_u8(p + 7, h.state);
  put_u16(p + 8, h.sensor_ok_mask);
  put_u16(p + 10, h.error_count);
  put_u16(p + 12, h.vin_mv);
  put_u16(p + 14, h.free_ram);
  return finish(buf, HEALTH_LEN);
}

}  // namespace lora
