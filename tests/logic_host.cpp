// station_logic.h 단위 테스트 (PC). 실패하면 0 이 아닌 값으로 끝난다.
#include <cstdio>
#include <cmath>
#include "../firmware/fix_station/station_logic.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)
static bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

static void make_pms(uint8_t* f, uint16_t pm1, uint16_t pm25, uint16_t pm10) {
  for (int i = 0; i < 32; i++) f[i] = 0;
  f[0] = 0x42; f[1] = 0x4D; f[2] = 0x00; f[3] = 28;
  uint16_t vals[13] = {pm1, pm25, pm10, pm1, pm25, pm10, 1200, 400, 90, 12, 3, 1, 0};
  for (int i = 0; i < 13; i++) { f[4 + 2 * i] = vals[i] >> 8; f[5 + 2 * i] = vals[i] & 0xFF; }
  uint16_t sum = 0;
  for (int i = 0; i < 30; i++) sum += f[i];
  f[30] = sum >> 8; f[31] = sum & 0xFF;
}

int main() {
  // ---- Sensirion CRC-8: 데이터시트 예시 0xBEEF -> 0x92, 기본 보상값 CRC
  uint8_t be[2] = {0xBE, 0xEF};
  CHECK(logic::sensirion_crc8(be, 2) == 0x92);
  uint8_t rh[2] = {0x80, 0x00}, tt[2] = {0x66, 0x66};
  CHECK(logic::sensirion_crc8(rh, 2) == 0xA2);
  CHECK(logic::sensirion_crc8(tt, 2) == 0x93);
  CHECK(logic::sgp41_rh_ticks(50.0f) == 0x8000);
  CHECK(logic::sgp41_t_ticks(25.0f) == 0x6666);
  CHECK(logic::sgp41_rh_ticks(NAN) == 0x8000 && logic::sgp41_t_ticks(NAN) == 0x6666);

  // ---- PMS7003: 정상 프레임, 앞에 쓰레기, 체크섬 깨짐
  uint8_t f[32];
  logic::Pms7003Parser p;
  make_pms(f, 7, 12, 19);
  uint8_t junk[5] = {0x00, 0x42, 0x13, 0x42, 0x42};
  for (uint8_t b : junk) CHECK(!p.feed(b));
  bool got = false;
  for (int i = 0; i < 32; i++) got = p.feed(f[i]);
  CHECK(got && p.pm1_0 == 7 && p.pm2_5 == 12 && p.pm10 == 19 && p.frames == 1);
  make_pms(f, 50, 60, 70);
  f[20] ^= 0x01;
  for (int i = 0; i < 32; i++) got = p.feed(f[i]);
  CHECK(!got && p.bad == 1 && p.pm2_5 == 12);   // 깨진 프레임은 값을 안 바꾼다

  // ---- 전기화학 환산: Rf 100k, 200 nA/ppm. 10 ppm -> 2000 nA -> 0.2 V = 200000 µV
  logic::EcChannel h2s = {100000.0f, 200.0f, 1, 0.0f, 50.0f};
  CHECK(near(logic::ec_current_na(h2s, 200000.0f), 2000.0f, 0.01f));
  CHECK(near(logic::ec_ppm(h2s, 200000.0f), 10.0f, 1e-4f));
  h2s.zero_uv = 1500.0f;                                   // 영점 1.5 mV 빼기
  CHECK(near(logic::ec_ppm(h2s, 201500.0f), 10.0f, 1e-4f));
  h2s.polarity = -1;
  CHECK(near(logic::ec_ppm(h2s, 1500.0f - 200000.0f), 10.0f, 1e-4f));
  CHECK(std::isnan(logic::ec_ppm(h2s, NAN)));
  logic::EcChannel nh3 = {100000.0f, NAN, 1, 0.0f, 100.0f};   // 감도 미입력
  CHECK(std::isnan(logic::ec_ppm(nh3, 123456.0f)));
  logic::EcChannel sat = {100000.0f, 200.0f, 1, 0.0f, 50.0f};
  CHECK(std::isnan(logic::ec_ppm(sat, 2000000.0f)));          // 100 ppm 상당 -> 포화로 무효
  // ADS1115 ±2.048V 한 LSB(62.5 µV)가 몇 ppm 인가: 0.003125 ppm
  CHECK(near(logic::ec_ppm(logic::EcChannel{100000.0f, 200.0f, 1, 0.0f, 50.0f}, 62.5f), 0.003125f, 1e-6f));

  // ---- 평균기
  logic::Mean m;
  CHECK(std::isnan(m.get()));
  m.add(1.0f); m.add(NAN); m.add(3.0f);
  CHECK(m.n == 2 && near(m.get(), 2.0f, 1e-6f));

  // ---- 챔버: 자연 확산 모드는 늘 SAMPLE
  logic::Chamber d({false, 3600, 300, 60, 60});
  for (uint32_t t = 1; t < 5000; t++) CHECK(d.tick(t) == logic::PH_SAMPLE);
  CHECK(!d.pump_on() && !d.valve_zero());

  // ---- 챔버 모드: 100초에 첫 영점, 300초 영점 (마지막 60초 평균), 60초 퍼지, 3600초 뒤 다음 영점
  logic::Chamber c({true, 3600, 300, 60, 60});
  c.schedule_first_zero(100);
  int zero_secs = 0, window_secs = 0, purge_secs = 0, done = 0;
  uint32_t first_purge = 0, back_to_sample = 0, second_zero = 0;
  for (uint32_t t = 1; t <= 5000; t++) {
    logic::Phase ph = c.tick(t);
    if (ph == logic::PH_ZERO) { zero_secs++; if (c.in_zero_window(t)) window_secs++; CHECK(c.valve_zero()); }
    if (ph == logic::PH_PURGE) { purge_secs++; if (!first_purge) first_purge = t; CHECK(!c.valve_zero()); }
    if (ph == logic::PH_SAMPLE && first_purge && !back_to_sample) back_to_sample = t;
    if (ph == logic::PH_ZERO && back_to_sample && !second_zero) second_zero = t;
    if (c.take_zero_done()) done++;
  }
  CHECK(c.pump_on());
  CHECK(first_purge == 400);              // 100 에 영점 시작 + 300
  CHECK(back_to_sample == 460);           // 퍼지 60 초
  CHECK(second_zero == 460 + 3600);
  CHECK(window_secs == 60 * 2);           // 영점 두 번, 각각 마지막 60초
  CHECK(done == 2);
  CHECK(purge_secs == 120);               // 퍼지 60초 x 2회 (두 번째는 4360~4420)

  if (fails == 0) std::printf("station_logic: 모든 검사 통과\n");
  return fails ? 1 : 0;
}
