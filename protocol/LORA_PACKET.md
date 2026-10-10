# LoRa 패킷 v1 (고정형 관측소 -> 지상국 게이트웨이)

데이터의 뜻은 `moving-ver/docs/DATA_SPEC.md` 가 정하고, 이 문서는 무선으로 보낼 때의 바이트 배치만 정합니다.
펌웨어 `firmware/fix_station/lora_packet.h` 와 게이트웨이 `gateway/odor_gateway/lora_codec.py` 가 이 문서의 구현이며,
둘이 같은 바이트를 만드는지는 `tests/test_lora_codec.py` 가 검사합니다.

## 1. 무선 설정

| 항목 | 값 |
|---|---|
| 모듈 | EBYTE E220-900T22D (LLCC68), 관측소와 게이트웨이 같은 설정 |
| 모드 | 투명 전송 (M0 = M1 = 0) |
| UART | 9600 bps 8N1 |
| 주파수 | 920~923 MHz 대역 (국내 LPWA). 채널, 주소, 공중 속도는 모든 모듈을 같게 |
| RSSI 덧붙이기 | 선택. 게이트웨이 모듈의 REG3 bit7 을 켜면 각 프레임 뒤에 1 B 가 붙고 `--rssi-byte` 로 읽음. dBm = -(256 - 값) |

보내는 때: 관측소 번호 N 은 매분 `(N x 7) mod 60` 초에 SAMPLE 을 보냅니다 (fix_01 07 초, fix_02 14 초, fix_03 21 초, fix_04 28 초).
HEALTH 는 켠 뒤 10 초, 그다음 10 분마다.

## 2. 프레임

모든 정수는 **리틀 엔디언**입니다.

| 오프셋 | 크기 | 이름 | 내용 |
|---|---|---|---|
| 0 | 2 | sync | `A5 5A` |
| 2 | 1 | version | 1 |
| 3 | 1 | type | `0x01` SAMPLE, `0x02` HEALTH, `0x03` 예약 (이동형 요약, 미정) |
| 4 | 1 | station | 관측소 번호 1~99 (`fix_01` = 1) |
| 5 | 2 | seq | u16, 보낼 때마다 +1 (SAMPLE, HEALTH 공통). 65535 다음 0 |
| 7 | 4 | time | u32 UTC epoch 초. 0 = 관측소 시계 모름 |
| 11 | 1 | len | 페이로드 길이 (SAMPLE 38, HEALTH 16) |
| 12 | len | payload | 아래 |
| 12+len | 2 | crc | u16 CRC-16/CCITT-FALSE, 오프셋 2 (version) 부터 페이로드 끝까지 |

CRC-16/CCITT-FALSE: 다항식 0x1021, 초기값 0xFFFF, 반사 없음, 최종 XOR 없음. 확인값 `"123456789"` -> `0x29B1`.

받는 쪽은 `A5 5A` 를 찾고, 헤더의 len 으로 프레임 끝을 정하고, CRC 가 맞을 때만 받아들입니다.
CRC 가 틀리면 sync 다음 바이트부터 다시 찾습니다.

## 3. SAMPLE 페이로드 (38 B)

60 초 평균값입니다. 값이 없으면 NA 값을 넣습니다 (u16 `0xFFFF`, i16 `0x8000` = -32768, i32 `0x80000000`).

| 오프셋 | 형식 | 필드 | 1 LSB | 범위 |
|---|---|---|---|---|
| 0 | u16 | nh3_ppm | 0.01 ppm | 0~655.34 |
| 2 | u16 | h2s_ppm | 0.001 ppm | 0~65.534 |
| 4 | u16 | tvoc_ppb | 1 ppb | |
| 6 | u16 | eco2_ppm | 1 ppm | |
| 8 | u16 | voc_index | 1 | 1~500 |
| 10 | u16 | nox_index | 1 | 1~500 |
| 12 | i16 | temperature_c | 0.01 ℃ | -327.67~327.67 |
| 14 | u16 | humidity_pct | 0.01 % | 0~100 |
| 16 | u16 | pressure_hpa | 0.1 hPa | |
| 18 | u16 | pm1_0_ugm3 | 1 µg/m³ | |
| 20 | u16 | pm2_5_ugm3 | 1 µg/m³ | |
| 22 | u16 | pm10_ugm3 | 1 µg/m³ | |
| 24 | i32 | nh3_raw | 1 µV | 증폭 회로 출력, 영점 보정 전 |
| 28 | i32 | h2s_raw | 1 µV | |
| 32 | u16 | flags | | GasReading.flags 와 같은 비트 |
| 34 | u8 | sampling_mode | | 0 확산, 1 챔버 |
| 35 | u8 | pump_duty | | 0~255 (PWM). 0 이면 펌프 꺼짐 |
| 36 | u16 | avg_window_s | 1 s | 평균에 쓴 시간 |

부호 없는 값이 음수면 0, 넘치면 65534 로 자릅니다 (65535 는 NA). 반올림은 0.5 에서 멀어지는 쪽.
`pump_on` 은 따로 싣지 않고 flags 의 `PUMP_ON`(bit9) 으로 정합니다.

## 4. HEALTH 페이로드 (16 B)

| 오프셋 | 형식 | 필드 | 내용 |
|---|---|---|---|
| 0 | u32 | uptime_s | 켜진 뒤 초 |
| 4 | u8 x 3 | fw_major, fw_minor, fw_patch | 펌웨어 버전 -> `firmware: "fix_station 0.2.0"` |
| 7 | u8 | state | DeviceHealth.STATE_* |
| 8 | u16 | sensor_ok_mask | bit0 ADS1115, bit1 SGP41, bit2 SHT31, bit3 PMS7003, bit4 DS3231, bit5 EEPROM |
| 10 | u16 | error_count | 켜진 뒤 누적 오류 |
| 12 | u16 | vin_mv | 전원 전압 mV (`supply_v`). NA = 측정 안 함 |
| 14 | u16 | free_ram | Mega2560 남은 RAM 바이트 (진단용) |

## 5. 게이트웨이가 하는 일

1. 프레임을 풀어 `station_sample`, `device_health` 기록(DATA_SPEC.md 4.4, 4.6)을 만든다.
2. 위치는 `stations.yaml` 의 설치 좌표를 붙인다.
3. time 이 0 이거나 수신 시각과 300 초 넘게 다르면 `TIME_UNSYNCED` 를 켠다 (0 이면 수신 시각으로 바꿈).
4. seq 가 건너뛴 만큼을 손실로 센다 (관측소 재부팅으로 seq 가 0 부터 다시 시작하면 손실로 치지 않음).
5. JSONL, CSV 로 남기고 `--ros` 면 `/station/<id>/sample`, `/health/<id>` 로 발행한다.

## 6. 시험 벡터

`tests/test_lora_codec.py` 가 아래 두 줄을 그대로 읽어 풀어 보고, 펌웨어 인코더가 같은 바이트를 만드는지도 확인합니다.

SAMPLE: fix_01, seq 1342, time 1791609120 (2026-10-10T05:12:00Z), NH3 1.84 ppm, H2S 0.031 ppm, TVOC 없음,
eCO2 없음, VOC Index 112, NOx Index 1, -3.25 ℃, 61.5 %, 기압 없음, PM 8/12/18, NH3 원신호 없음, H2S 원신호 6213 µV,
flags 0, 확산, 펌프 0, 60 초 평균

```vector-sample
a5 5a 01 01 01 3e 05 20 c9 c9 6a 26 b8 00 1f 00 ff ff ff ff 70 00 01 00 bb fe 06 18 ff ff 08 00 0c 00 12 00 00 00 00 80 45 18 00 00 00 00 00 00 3c 00 74 0e
```

HEALTH: fix_03, seq 77, time 0 (시계 모름), uptime 86400 s, 펌웨어 0.2.0, 정상(2), 센서 6 개 모두 정상(0x3F),
오류 3, 12.31 V, 남은 RAM 5120 B

```vector-health
a5 5a 01 02 03 4d 00 00 00 00 00 10 80 51 01 00 00 02 00 02 3f 00 03 00 16 30 00 14 cd 4d
```
