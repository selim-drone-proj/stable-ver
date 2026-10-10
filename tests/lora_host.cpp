// PC 에서 lora_packet.h 를 그대로 컴파일해 프레임을 16진수로 출력한다.
// test_lora_codec.py 가 이 출력을 파이썬 디코더/인코더와 바이트 단위로 비교한다.
#include <cstdio>
#include "../firmware/fix_station/lora_packet.h"

static void dump(const uint8_t* b, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) std::printf("%02X", b[i]);
  std::printf("\n");
}

int main() {
  uint8_t buf[lora::MAX_FRAME];
  lora::Sample s;
  s.nh3_ppm = 1.234f; s.h2s_ppm = 0.0123f; s.tvoc_ppb = lora::na(); s.eco2_ppm = lora::na();
  s.voc_index = 112.0f; s.nox_index = 1.0f; s.temperature_c = -3.21f; s.humidity_pct = 61.37f;
  s.pressure_hpa = lora::na(); s.pm1_0 = 8.0f; s.pm2_5 = 13.0f; s.pm10 = 21.0f;
  s.nh3_raw_uv = 12345; s.h2s_raw_uv = -2468; s.flags = lora::F_PUMP_ON | lora::F_TEMP_COMP;
  s.sampling_mode = lora::MODE_CHAMBER; s.pump_duty = 180; s.avg_window_s = 60;
  uint8_t n = lora::encode_sample(buf, 3, 0xBEEF, 1791600000UL, s);
  dump(buf, n);

  lora::Health h;
  h.uptime_s = 86400; h.fw_major = 0; h.fw_minor = 2; h.fw_patch = 0; h.state = lora::STATE_OK;
  h.sensor_ok_mask = lora::OK_ADS1115 | lora::OK_SHT31 | lora::OK_PMS7003; h.error_count = 7;
  h.vin_mv = 12050; h.free_ram = 3120;
  n = lora::encode_health(buf, 3, 0xBEF0, 1791600060UL, h);
  dump(buf, n);

  // 범위 밖, 경계값
  lora::Sample e = s;
  e.nh3_ppm = 1000.0f; e.h2s_ppm = -1.0f; e.temperature_c = 500.0f; e.humidity_pct = lora::na();
  e.nh3_raw_uv = lora::I32_NA; e.flags = 0; e.sampling_mode = lora::MODE_DIFFUSION; e.pump_duty = 0;
  n = lora::encode_sample(buf, 1, 0, 0, e);
  dump(buf, n);
  return 0;
}
