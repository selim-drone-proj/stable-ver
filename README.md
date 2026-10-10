# stable-ver: 고정형 악취 관측소 (fix_01 ~ fix_04) + 지상국 LoRa 게이트웨이

관측소(Arduino Mega2560 PRO)가 1 초마다 재서 60 초 평균을 LoRa(E220)로 보내고,
지상국 게이트웨이(PC 또는 Pi + E220)가 받아 규격 v0.2 기록(JSONL, CSV, ROS 2)으로 바꿉니다.

데이터의 뜻은 [moving-ver/docs/DATA_SPEC.md](../moving-ver/docs/DATA_SPEC.md) (StationSample, DeviceHealth),
무선 바이트 배치는 [protocol/LORA_PACKET.md](protocol/LORA_PACKET.md).

```
stable-ver/
├── firmware/fix_station/      관측소 펌웨어 (config.h 의 STATION_NO 만 바꿔 4 대에 올림)
│   ├── fix_station.ino
│   ├── config.h               번호, 주기, 핀, 샘플링 방식, 센서 감도
│   ├── lora_packet.h          패킷 인코더 (PC 에서도 컴파일되어 시험에 쓰임)
│   └── station_logic.h        PMS7003 파서, 전기화학 환산, 챔버 상태기계 (PC 시험 가능)
├── gateway/                   지상국 게이트웨이 (파이썬 패키지 odor-gateway)
│   ├── odor_gateway/          lora_codec, records, gateway, simulate, ros_bridge, odor_json
│   └── stations.yaml          관측소 설치 좌표 (지금은 임시값)
├── protocol/LORA_PACKET.md
└── tests/                     C++ 와 파이썬이 같은 바이트를 만드는지, 게이트웨이, 펌웨어 로직
```

## 1. 관측소 결선 (fix_0N)

결선 도면(2026-10-05) 기준이고, **한 줄이 추가**됩니다: 증폭 회로의 2.5 V 기준 전압을 ADS1115 **A3** 에 연결.
펌웨어는 A0-A3 (NH3), A1-A3 (H2S) 를 차동으로 읽어 기준 전압의 흔들림을 지웁니다.

| 장치 | 연결 | 주소, 포트 |
|---|---|---|
| ADS1115 | I2C (SDA 20, SCL 21) | 0x48. A0 NH3 증폭 출력, A1 H2S 증폭 출력, **A3 Vref 2.5 V** |
| SHT31 | I2C | 0x44 |
| SGP41 | I2C | 0x59 |
| DS3231 | I2C | 0x68 (UTC 로 맞출 것) |
| PMS7003 | Serial1 (18 TX, 19 RX), 레벨 컨버터 경유 | 9600 |
| E220-900T22D | Serial2 (16 TX, 17 RX), 레벨 컨버터 경유, M0 = M1 = GND | 9600 |
| 펌프 (챔버 방식만) | D9 -> IRF520 모듈 SIG | PWM |
| 3방향 밸브 (챔버 방식만) | D8 -> Crowtail MOSFET SIG | HIGH = 활성탄(영점) 경로 |
| 전원 전압 측정 (선택) | 12 V -> 20 kΩ -> A15 -> 10 kΩ -> GND | `USE_VIN_SENSE 1` |

증폭 회로: Rf 100 kΩ, ADS1115 이득 ±2.048 V (62.5 µV/LSB). 센서 전류 nA = (µV - 영점) x 1000 / Rf,
ppm = nA / 감도. H2S 110-303 명목 감도 200 nA/ppm, ME3-NH3 는 라벨 값을 넣기 전까지 ppm 을 보내지 않습니다(null).

## 2. 펌웨어 올리기

Arduino IDE 또는 arduino-cli. 라이브러리: Adafruit ADS1X15 (2.x), Adafruit SHT31 Library, RTClib, Sensirion Gas Index Algorithm.

```bash
arduino-cli core install arduino:avr
arduino-cli lib install "Adafruit ADS1X15" "Adafruit SHT31 Library" "RTClib" "Sensirion Gas Index Algorithm"
# config.h 에서 STATION_NO 를 1~4 로 바꾼 뒤
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 firmware/fix_station
arduino-cli upload  --fqbn arduino:avr:mega:cpu=atmega2560 -p /dev/ttyUSB0 firmware/fix_station
```

현재 크기: 플래시 26.6 KB (10 %), RAM 1.2 KB (14 %).

**USB 시리얼 명령** (115200, 줄 끝 `\n`). 1 초마다 CSV 한 줄도 나옵니다.

| 명령 | 하는 일 |
|---|---|
| `show` | 현재 보정값, 시각, 센서 상태 |
| `zero` | 지금 공기를 영점으로 (깨끗한 공기에서 30 초 평균) |
| `set nh3_sens 30.5` | ME3-NH3 감도 nA/ppm (센서 라벨 값) |
| `set h2s_sens 187` | 110-303 감도 nA/ppm |
| `time 1791609120` | RTC 를 UTC epoch 초로 맞춤 (PC 에서 `date +%s`) |
| `save` | 보정값을 EEPROM 에 저장 |
| `report` | LoRa 보고를 지금 한 번 |

설치 순서: RTC 시각 맞추기 -> 깨끗한 공기에서 30 분 예열 후 `zero` -> 감도 입력 -> `save`.

## 3. 샘플링 방식: 자연 확산 / 챔버

`config.h` 의 `SAMPLING_MODE` 하나로 바뀝니다. 데이터 규격은 같고 `sampling_mode` 필드와 flags 로 구분됩니다.

| | 자연 확산 (`MODE_DIFFUSION`, 기본) | 챔버 (`MODE_CHAMBER`) |
|---|---|---|
| 흡입 | 함체 통기구 | 3D 프린트 챔버 + 펌프 |
| 영점 | 수동 (`zero`) | 6 시간마다 자동: 밸브를 활성탄 경로로 5 분 -> 마지막 60 초 평균을 영점 -> 60 초 퍼지 |
| 그동안의 값 | | `ZERO_CAL`, `PURGE` 비트가 켜지고 평균에서 빠짐 |
| 추가 부품 | 없음 | 펌프, 밸브, 튜브 (재고), **활성탄 필터 (구매 필요)** |

## 4. 게이트웨이

```bash
cd gateway && pip install .
odor-gateway --port /dev/ttyUSB0 --stations stations.yaml --out ~/odor_logs        # 실제 수신
odor-gateway --simulate --speed 60 --stations stations.yaml --out /tmp/gw          # 하드웨어 없이 4 개소
odor-gateway ... --ros                                                             # ROS 2 로도 발행
odor-gateway ... --rssi-byte                                                       # E220 RSSI 덧붙이기를 켰을 때
```

남기는 파일: `gateway_YYYYmmdd.jsonl` (모든 기록), `station_fix_0N_YYYYmmdd.csv` (관측소별).
ROS 2: `/station/fix_0N/sample`, `/health/fix_0N`, `/health/gw_01`. `--ros` 는 ROS 2 Jazzy 와 odor_msgs
(moving-ver/ros2_ws) 가 빌드되어 있어야 합니다.

`stations.yaml` 의 좌표는 임시값입니다. 설치 후 실측 좌표로 바꿉니다.

## 5. 시험

```bash
python3 -m pytest tests      # g++ 가 있으면 펌웨어 헤더를 PC 에서 컴파일해 파이썬 디코더와 바이트 단위로 대조
```

- `test_lora_codec.py`: C++ 인코더 = 파이썬 인코더, CRC 확인값, NA 값, LORA_PACKET.md 의 시험 벡터
- `test_firmware_logic.py`: PMS7003 파서, 전기화학 환산, 챔버 상태기계 (영점, 퍼지 시간)
- `test_gateway.py`: 프레임 -> 기록 -> 스키마 검사 (moving-ver 가 옆 폴더에 있으면), 손실 집계, 시계 어긋남 처리
