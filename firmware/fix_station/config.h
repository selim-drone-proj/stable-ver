// config.h - 고정형 관측소 설정. 관측소마다 STATION_NO 만 바꿔서 올린다.
#pragma once

// ---------------------------------------------------------------
// 관측소 번호와 보고 주기
// ---------------------------------------------------------------
#define STATION_NO        1        // fix_01 = 1, fix_02 = 2 ...
#define FW_MAJOR          0
#define FW_MINOR          2
#define FW_PATCH          0
#define REPORT_S          60       // LoRa 보고 주기 (초). 이 동안 1초마다 잰 값을 평균
#define HEALTH_S          600      // 상태 보고 주기 (초)
#define WARMUP_S          600      // 켠 뒤 이 시간 동안 WARMUP 플래그 (전기화학 셀 안정화)
#define SLOT_SPACING_S    7        // 관측소끼리 송신 시각을 이만큼 벌려 충돌을 줄인다

// ---------------------------------------------------------------
// 샘플링 방식
//   MODE_DIFFUSION : 통기구 자연 확산. 펌프, 밸브 안 씀
//   MODE_CHAMBER   : 3D 프린트 챔버 + 펌프 흡입. 정해진 주기로 활성탄 경로 영점 보정
// ---------------------------------------------------------------
#define SAMPLING_MODE     lora::MODE_DIFFUSION

#define PUMP_DUTY         180      // 0~255. 챔버 유량이 너무 세면 전기화학 값이 흔들린다
#define ZERO_INTERVAL_S   (6UL * 3600UL)   // 영점 보정 간격 (챔버 모드)
#define ZERO_DURATION_S   300      // 활성탄 경로로 돌리는 시간
#define ZERO_AVERAGE_S    60       // 그중 마지막 몇 초를 영점으로 쓸지
#define PURGE_S           60       // 영점 후 외기로 다시 채우는 시간 (이 동안 값은 버림)

// ---------------------------------------------------------------
// 핀 (Mega2560 PRO)
//   I2C: SDA 20, SCL 21  (ADS1115 0x48, SHT31 0x44, SGP41 0x59, DS3231 0x68)
//   Serial1: PMS7003 (레벨 컨버터 경유) 9600
//   Serial2: E220-900T22D (레벨 컨버터 경유) 9600, M0, M1 은 GND 에 고정(일반 모드)
// ---------------------------------------------------------------
#define PIN_PUMP_PWM      9        // IRF520 모듈 SIG
#define PIN_VALVE         8        // Crowtail MOSFET SIG. HIGH = 활성탄(영점) 경로
#define USE_VIN_SENSE     1        // 12V 를 20k/10k 로 나눠 A15 에 넣었으면 1, 아니면 0
#define PIN_VIN_SENSE     A15
#define VIN_DIVIDER       3.0f
#define PIN_STATUS_LED    13

#define PMS_BAUD          9600
#define LORA_BAUD         9600
#define USB_BAUD          115200

// ---------------------------------------------------------------
// 전기화학 센서 (증폭 회로 + ADS1115)
//   기준 전압(2.5V 버퍼 출력)을 A3 에 연결하고 A0-A3, A1-A3 를 차동으로 읽는다.
//   결선도의 "A0 NH3, A1 H2S" 에 A3 = Vref 한 줄이 추가된다.
// ---------------------------------------------------------------
#define ADS_GAIN          GAIN_TWO // ±2.048 V, 62.5 µV/LSB
#define RF_OHM            100000.0f
// 감도는 센서 포장 라벨 값으로 바꿀 것 (USB 명령 set nh3_sens ... 로도 가능, EEPROM 저장)
#define NH3_SENS_NA_PPM   NAN      // ME3-NH3: 라벨, 데이터시트 값 입력 전까지 ppm 은 NaN 으로 보냄
#define H2S_SENS_NA_PPM   200.0f   // 110-303 명목값 200 nA/ppm. 개체 편차 ±37% 이니 라벨 값으로 교체
#define NH3_POLARITY      1        // 가스를 댔을 때 값이 내려가면 -1 로
#define H2S_POLARITY      1
#define NH3_MAX_PPM       100.0f   // 이보다 크면 회로 포화로 보고 무효 처리
#define H2S_MAX_PPM       50.0f
