"""
lora_codec.py - 고정형 관측소 LoRa 패킷 v1 디코더/인코더 (표준 라이브러리만 사용)

C++ 짝: stable-ver/firmware/fix_station/lora_packet.h  (둘을 같이 고칠 것)
규격:  stable-ver/protocol/LORA_PACKET.md

    parser = FrameParser(rssi_byte=False)
    for frame in parser.feed(serial_bytes):
        print(frame.type, frame.station, frame.fields)
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

SYNC = b"\xA5\x5A"
VERSION = 1
TYPE_SAMPLE = 0x01
TYPE_HEALTH = 0x02

HEADER = struct.Struct("<BBBHIB")          # version, type, station, seq, time, len  (동기 바이트 뒤 10바이트)
SAMPLE = struct.Struct("<6Hh5H2iH2BH")     # 38 바이트
HEALTH = struct.Struct("<I3BBHHHH")        # 16 바이트
HEADER_LEN = 2 + HEADER.size               # 12
CRC_LEN = 2
PAYLOAD_LEN = {TYPE_SAMPLE: SAMPLE.size, TYPE_HEALTH: HEALTH.size}

U16_NA = 0xFFFF
I16_NA = -32768
I32_NA = -(2**31)

# 품질 비트 (GasReading.flags 와 같음)
FLAGS = {
    "WARMUP": 1 << 0, "NH3_INVALID": 1 << 1, "H2S_INVALID": 1 << 2, "VOC_INVALID": 1 << 3,
    "TH_INVALID": 1 << 4, "PM_INVALID": 1 << 5, "TIME_UNSYNCED": 1 << 6, "ZERO_CAL": 1 << 7,
    "PURGE": 1 << 8, "PUMP_ON": 1 << 9, "TEMP_COMP": 1 << 10, "SIMULATED": 1 << 11,
}

# (필드 이름, LSB, 부호 여부) - SAMPLE 구조 순서와 같다
_SAMPLE_SCALED = [
    ("nh3_ppm", 0.01), ("h2s_ppm", 0.001), ("tvoc_ppb", 1.0), ("eco2_ppm", 1.0),
    ("voc_index", 1.0), ("nox_index", 1.0),
]
_SAMPLE_SCALED_TAIL = [
    ("humidity_pct", 0.01), ("pressure_hpa", 0.1), ("pm1_0_ugm3", 1.0), ("pm2_5_ugm3", 1.0), ("pm10_ugm3", 1.0),
]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _dec_u16(raw: int, lsb: float):
    return None if raw == U16_NA else round(raw * lsb, 6)


def _enc_u16(v, lsb: float) -> int:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return U16_NA
    x = max(0.0, min(65534.0, v / lsb))
    return int(math.floor(x + 0.5))


def _enc_i16(v, lsb: float) -> int:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return I16_NA
    x = max(-32767.0, min(32767.0, v / lsb))
    return int(math.floor(x + 0.5)) if x >= 0 else int(math.ceil(x - 0.5))


@dataclass
class Frame:
    version: int
    type: int
    station: int
    seq: int
    time: int                   # UTC epoch 초, 0 = 관측소 시계 모름
    fields: dict = field(default_factory=dict)
    rssi_dbm: int | None = None

    @property
    def station_id(self) -> str:
        return f"fix_{self.station:02d}"


def decode_payload(ftype: int, payload: bytes) -> dict:
    if ftype == TYPE_SAMPLE:
        v = SAMPLE.unpack(payload)
        out = {}
        for i, (name, lsb) in enumerate(_SAMPLE_SCALED):
            out[name] = _dec_u16(v[i], lsb)
        out["temperature_c"] = None if v[6] == I16_NA else round(v[6] * 0.01, 4)
        for j, (name, lsb) in enumerate(_SAMPLE_SCALED_TAIL):
            out[name] = _dec_u16(v[7 + j], lsb)
        out["nh3_raw"] = None if v[12] == I32_NA else float(v[12])
        out["h2s_raw"] = None if v[13] == I32_NA else float(v[13])
        out["flags"] = v[14]
        out["sampling_mode"] = v[15]
        out["pump_duty"] = v[16]
        out["avg_window_s"] = v[17]
        out["pump_on"] = bool(v[14] & FLAGS["PUMP_ON"])
        return out
    if ftype == TYPE_HEALTH:
        v = HEALTH.unpack(payload)
        return {
            "uptime_s": v[0],
            "firmware": f"fix_station {v[1]}.{v[2]}.{v[3]}",
            "state": v[4],
            "sensor_ok_mask": v[5],
            "error_count": v[6],
            "supply_v": None if v[7] == U16_NA else v[7] / 1000.0,
            "free_ram": v[8],
        }
    raise ValueError(f"모르는 패킷 종류 0x{ftype:02X}")


def encode_frame(ftype: int, station: int, seq: int, epoch_s: int, fields: dict) -> bytes:
    """시뮬레이터, 테스트용 인코더. 펌웨어(lora_packet.h)와 바이트 단위로 같다."""
    if ftype == TYPE_SAMPLE:
        f = fields
        vals = [_enc_u16(f.get(n), lsb) for n, lsb in _SAMPLE_SCALED]
        vals.append(_enc_i16(f.get("temperature_c"), 0.01))
        vals += [_enc_u16(f.get(n), lsb) for n, lsb in _SAMPLE_SCALED_TAIL]
        for raw_name in ("nh3_raw", "h2s_raw"):
            r = f.get(raw_name)
            vals.append(I32_NA if r is None or (isinstance(r, float) and math.isnan(r)) else int(r))
        vals += [f.get("flags", 0), f.get("sampling_mode", 0), f.get("pump_duty", 0), f.get("avg_window_s", 60)]
        payload = SAMPLE.pack(*vals)
    elif ftype == TYPE_HEALTH:
        f = fields
        major, minor, patch = (int(x) for x in str(f.get("fw_version", "0.0.0")).split("."))
        vin = f.get("supply_v")
        payload = HEALTH.pack(
            f.get("uptime_s", 0), major, minor, patch, f.get("state", 2), f.get("sensor_ok_mask", 0),
            f.get("error_count", 0), U16_NA if vin is None else int(round(vin * 1000)), f.get("free_ram", 0),
        )
    else:
        raise ValueError(ftype)
    body = HEADER.pack(VERSION, ftype, station, seq & 0xFFFF, epoch_s & 0xFFFFFFFF, len(payload)) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt(body))


class FrameParser:
    """시리얼로 들어오는 바이트를 쌓아 두었다가 완성된 프레임만 꺼낸다.

    rssi_byte=True 면 E220 의 'RSSI 바이트 덧붙이기' 설정(REG3 bit7)을 켠 것으로 보고
    프레임 뒤 1바이트를 수신 세기로 읽는다: dBm = -(256 - 값)
    """

    def __init__(self, rssi_byte: bool = False, max_buffer: int = 4096):
        self.buf = bytearray()
        self.rssi_byte = rssi_byte
        self.max_buffer = max_buffer
        self.crc_errors = 0
        self.dropped_bytes = 0

    def feed(self, data: bytes) -> list[Frame]:
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                keep = 1 if self.buf.endswith(SYNC[:1]) else 0
                self.dropped_bytes += len(self.buf) - keep
                del self.buf[: len(self.buf) - keep]
                break
            if i > 0:
                self.dropped_bytes += i
                del self.buf[:i]
            if len(self.buf) < HEADER_LEN:
                break
            ver, ftype, station, seq, t, ln = HEADER.unpack_from(self.buf, 2)
            if ver != VERSION or PAYLOAD_LEN.get(ftype) != ln:
                del self.buf[:1]           # 가짜 동기 바이트. 한 바이트 밀고 다시 찾는다
                self.dropped_bytes += 1
                continue
            total = HEADER_LEN + ln + CRC_LEN + (1 if self.rssi_byte else 0)
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2: HEADER_LEN + ln])
            (crc,) = struct.unpack_from("<H", self.buf, HEADER_LEN + ln)
            if crc != crc16_ccitt(body):
                self.crc_errors += 1
                del self.buf[:1]
                continue
            payload = bytes(self.buf[HEADER_LEN: HEADER_LEN + ln])
            frame = Frame(ver, ftype, station, seq, t, decode_payload(ftype, payload))
            if self.rssi_byte:
                frame.rssi_dbm = -(256 - self.buf[HEADER_LEN + ln + CRC_LEN])
            out.append(frame)
            del self.buf[:total]
        if len(self.buf) > self.max_buffer:
            self.dropped_bytes += len(self.buf)
            self.buf.clear()
        return out
