// station_logic.h - 하드웨어와 무관한 계산들. PC 에서도 컴파일되어 단위 테스트한다.
//   - PMS7003 프레임 파서
//   - 전기화학 센서 µV -> ppm 환산
//   - 1초 값 평균기
//   - 챔버 모드 상태 기계 (샘플 / 영점 / 퍼지)
//   - SGP41 CRC-8
#pragma once
#include <stdint.h>
#include <math.h>

namespace logic {

// ================================================================
// PMS7003 (능동 모드, 32바이트 프레임)
//   42 4D | 길이 00 1C | data1..13 (각 2바이트, 빅 엔디언) | 체크섬 2바이트
//   data4..6 = 대기 환산 PM1.0, PM2.5, PM10 (µg/m³)
// ================================================================
class Pms7003Parser {
 public:
  uint16_t pm1_0 = 0, pm2_5 = 0, pm10 = 0;
  uint32_t frames = 0, bad = 0;

  // 한 바이트씩 넣는다. 완성된 정상 프레임이면 true
  bool feed(uint8_t b) {
    if (idx_ == 0 && b != 0x42) return false;
    if (idx_ == 1 && b != 0x4D) { idx_ = (b == 0x42) ? 1 : 0; return false; }
    buf_[idx_++] = b;
    if (idx_ < 32) return false;
    idx_ = 0;
    uint16_t len = (uint16_t)(buf_[2] << 8 | buf_[3]);
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 30; i++) sum += buf_[i];
    uint16_t chk = (uint16_t)(buf_[30] << 8 | buf_[31]);
    if (len != 28 || sum != chk) { bad++; return false; }
    pm1_0 = word(10);
    pm2_5 = word(12);
    pm10 = word(14);
    frames++;
    return true;
  }

 private:
  uint8_t buf_[32] = {0};
  uint8_t idx_ = 0;
  uint16_t word(uint8_t i) const { return (uint16_t)(buf_[i] << 8 | buf_[i + 1]); }
};

// ================================================================
// 전기화학 센서: 증폭 회로 출력 전압 -> 전류 -> ppm
//   전류(nA) = 극성 * (출력 µV - 영점 µV) * 1000 / Rf(Ω)
//   ppm      = 전류(nA) / 감도(nA/ppm)
// ================================================================
struct EcChannel {
  float rf_ohm;
  float sens_na_per_ppm;   // NaN 이면 아직 보정 안 됨 -> ppm 도 NaN
  int8_t polarity;
  float zero_uv;           // 깨끗한 공기에서의 출력 µV
  float max_ppm;
};

inline float ec_current_na(const EcChannel& c, float uv) {
  return (float)c.polarity * (uv - c.zero_uv) * 1000.0f / c.rf_ohm;
}

// 보정 안 됐거나 포화면 NaN
inline float ec_ppm(const EcChannel& c, float uv) {
  if (isnan(uv) || isnan(c.sens_na_per_ppm) || c.sens_na_per_ppm <= 0.0f) return NAN;
  float ppm = ec_current_na(c, uv) / c.sens_na_per_ppm;
  if (ppm > c.max_ppm * 1.2f) return NAN;
  return ppm;
}

// ================================================================
// 평균기: 유효한 값만 더하고, 하나도 없으면 NaN
// ================================================================
struct Mean {
  double sum = 0;
  uint16_t n = 0;
  void add(float v) { if (!isnan(v)) { sum += v; n++; } }
  float get() const { return n ? (float)(sum / n) : NAN; }
  void reset() { sum = 0; n = 0; }
};

// ================================================================
// 챔버 상태 기계
//   SAMPLE : 외기를 펌프로 흡입해 측정
//   ZERO   : 밸브를 활성탄 경로로 돌려 깨끗한 공기로 영점을 잡음
//   PURGE  : 영점 뒤 외기로 다시 채우는 중. 값은 버림
// 자연 확산 모드에서는 항상 SAMPLE, 펌프와 밸브는 꺼 둔다.
// ================================================================
enum Phase : uint8_t { PH_SAMPLE = 0, PH_ZERO = 1, PH_PURGE = 2 };

struct ChamberConfig {
  bool chamber;
  uint32_t zero_interval_s;
  uint16_t zero_duration_s;
  uint16_t zero_average_s;
  uint16_t purge_s;
};

class Chamber {
 public:
  explicit Chamber(const ChamberConfig& c) : cfg_(c) {}

  // 1초마다 호출. 켠 뒤 경과 초를 넣는다
  Phase tick(uint32_t uptime_s) {
    if (!cfg_.chamber) { phase_ = PH_SAMPLE; return phase_; }
    uint32_t in_phase = uptime_s - since_;
    switch (phase_) {
      case PH_SAMPLE:
        if (uptime_s >= next_zero_) { phase_ = PH_ZERO; since_ = uptime_s; zero_done_ = false; }
        break;
      case PH_ZERO:
        if (in_phase >= cfg_.zero_duration_s) { phase_ = PH_PURGE; since_ = uptime_s; zero_done_ = true; }
        break;
      case PH_PURGE:
        if (in_phase >= cfg_.purge_s) {
          phase_ = PH_SAMPLE; since_ = uptime_s;
          next_zero_ = uptime_s + cfg_.zero_interval_s;
        }
        break;
    }
    return phase_;
  }

  // 지금이 영점 평균에 넣을 구간인가 (영점 단계의 마지막 zero_average_s 초)
  bool in_zero_window(uint32_t uptime_s) const {
    return phase_ == PH_ZERO && (uptime_s - since_) + cfg_.zero_average_s >= cfg_.zero_duration_s;
  }

  // 영점 단계가 방금 끝났으면 한 번만 true
  bool take_zero_done() { bool d = zero_done_; zero_done_ = false; return d; }

  bool pump_on() const { return cfg_.chamber; }
  bool valve_zero() const { return cfg_.chamber && phase_ == PH_ZERO; }
  Phase phase() const { return phase_; }

  // 첫 영점 시각을 정한다 (예: 켜고 예열이 끝난 뒤)
  void schedule_first_zero(uint32_t at_uptime_s) { next_zero_ = at_uptime_s; }

 private:
  ChamberConfig cfg_;
  Phase phase_ = PH_SAMPLE;
  uint32_t since_ = 0;
  uint32_t next_zero_ = 0xFFFFFFFFUL;
  bool zero_done_ = false;
};

// ================================================================
// Sensirion CRC-8 (SGP41, SHT3x 공통): 다항식 0x31, 초기값 0xFF
// ================================================================
inline uint8_t sensirion_crc8(const uint8_t* d, uint8_t n) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

// SGP41 보상 입력: 상대습도 %, 온도 ℃ -> 센서 틱
inline uint16_t sgp41_rh_ticks(float rh) {
  if (isnan(rh)) return 0x8000;               // 기본값 50 %
  if (rh < 0) rh = 0;
  if (rh > 100) rh = 100;
  return (uint16_t)(rh * 65535.0f / 100.0f + 0.5f);
}
inline uint16_t sgp41_t_ticks(float t) {
  if (isnan(t)) return 0x6666;                // 기본값 25 ℃
  if (t < -45) t = -45;
  if (t > 130) t = 130;
  return (uint16_t)((t + 45.0f) * 65535.0f / 175.0f + 0.5f);
}

}  // namespace logic
