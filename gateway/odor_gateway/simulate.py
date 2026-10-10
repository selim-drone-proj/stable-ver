"""
simulate.py - 고정형 관측소 4대를 흉내 내 LoRa 프레임(바이트)을 만든다.

하드웨어 없이 게이트웨이와 소프트웨어(확산 모델, 대시보드)를 개발하기 위한 것.
실제 패킷 인코더(lora_codec.encode_frame)를 거치므로 디코딩 경로까지 그대로 시험된다.
모든 샘플에 SIMULATED 플래그가 켜져 있어 실제 데이터와 섞일 일이 없다.

악취 덩어리(plume)가 fix_01 -> fix_02 -> fix_03 -> fix_04 순서로 시간차를 두고 지나간다.
확산 모델을 시험할 때 "어느 관측소에 먼저 왔나" 를 볼 수 있게 한 것이다.
"""
from __future__ import annotations

import math
import random

from .lora_codec import FLAGS, TYPE_HEALTH, TYPE_SAMPLE, encode_frame

PLUME_PERIOD_S = 1800        # 30분마다 악취 덩어리 하나
PLUME_WIDTH_S = 300          # 한 관측소를 지나가는 데 걸리는 시간
STATION_LAG_S = 240          # 다음 관측소까지 걸리는 시간


class StationSim:
    def __init__(self, station_no: int, report_s: int = 60, health_s: int = 600, seed: int | None = None):
        self.no = station_no
        self.report_s = report_s
        self.health_s = health_s
        self.seq = 0
        self.uptime = 0
        self.rng = random.Random(seed if seed is not None else station_no)

    def plume(self, t: float) -> float:
        """0~1. 이 관측소에 악취 덩어리가 얼마나 걸려 있나"""
        phase = (t - (self.no - 1) * STATION_LAG_S) % PLUME_PERIOD_S
        if phase > PLUME_WIDTH_S:
            return 0.0
        return math.sin(math.pi * phase / PLUME_WIDTH_S) ** 2

    def sample_fields(self, t: float) -> dict:
        g = self.rng.gauss
        p = self.plume(t)
        hour = (t / 3600.0) % 24
        temp = 18.0 + 6.0 * math.sin((hour - 9) / 24 * 2 * math.pi) + g(0, 0.1)
        nh3_ppm = max(0.0, 0.3 + 5.0 * p + g(0, 0.05))
        h2s_ppm = max(0.0, 0.002 + 0.3 * p + g(0, 0.002))
        return {
            "nh3_ppm": nh3_ppm, "h2s_ppm": h2s_ppm, "tvoc_ppb": None, "eco2_ppm": None,
            "voc_index": round(100 + 200 * p + g(0, 3)), "nox_index": max(1, round(1 + 20 * p + g(0, 0.5))),
            "temperature_c": temp, "humidity_pct": min(100.0, max(0.0, 65 - 1.5 * (temp - 18) + g(0, 0.5))),
            "pressure_hpa": None, "pm1_0_ugm3": max(0, round(8 + g(0, 1))),
            "pm2_5_ugm3": max(0, round(14 + 4 * p + g(0, 1.5))), "pm10_ugm3": max(0, round(22 + 6 * p + g(0, 2))),
            # 원신호도 그럴듯하게: Rf 100k, H2S 200 nA/ppm -> µV = ppm * 200 * 100
            "nh3_raw": round(nh3_ppm * 120 * 100), "h2s_raw": round(h2s_ppm * 200 * 100),
            "flags": FLAGS["SIMULATED"] | (FLAGS["WARMUP"] if self.uptime < 600 else 0),
            "sampling_mode": 0, "pump_duty": 0, "avg_window_s": self.report_s,
        }

    def step(self, t: float) -> list[bytes]:
        """t(UTC epoch 초)에서 1초 진행. 보낼 프레임이 있으면 돌려준다"""
        self.uptime += 1
        out = []
        slot = (self.no * 7) % self.report_s
        if int(t) % self.report_s == slot:
            out.append(encode_frame(TYPE_SAMPLE, self.no, self.seq, int(t), self.sample_fields(t)))
            self.seq += 1
        elif self.uptime % self.health_s == 10:
            out.append(encode_frame(TYPE_HEALTH, self.no, self.seq, int(t), {
                "uptime_s": self.uptime, "fw_version": "0.2.0", "state": 2 if self.uptime > 600 else 1,
                "sensor_ok_mask": 0b11111, "error_count": 0, "supply_v": 12.0 + self.rng.gauss(0, 0.05),
                "free_ram": 6900,
            }))
            self.seq += 1
        return out


class FleetSim:
    """관측소 여러 대를 한꺼번에 돌린다"""

    def __init__(self, n: int = 4, report_s: int = 60, drop_rate: float = 0.02, seed: int = 0):
        self.stations = [StationSim(i + 1, report_s) for i in range(n)]
        self.drop_rate = drop_rate
        self.rng = random.Random(seed)

    def step(self, t: float) -> bytes:
        data = b""
        for s in self.stations:
            for fr in s.step(t):
                if self.rng.random() < self.drop_rate:      # 무선 손실 흉내
                    continue
                data += fr
        return data
