"""
odor_json.py - 규격 v0.2 의 JSON 기록과 ROS 2 메시지를 서로 바꾸는 공통 도구.

세 저장소(moving-ver, stable-ver, fire_detect)에 같은 파일이 들어 있다.
원본은 moving-ver/interfaces/python/odor_json.py. 고치면 나머지 둘에도 복사할 것.

규칙 (DATA_SPEC.md 6장)
  - Header        -> "stamp"(ISO 8601 UTC, 밀리초, Z) + "device_id"
  - Time          -> ISO 8601 UTC 문자열
  - float NaN     -> null
  - 최상위 기록   -> "type", "version" 추가
rclpy 없이도 import 된다 (ROS 변환 함수는 메시지 객체를 받을 때만 ROS 를 쓴다).
"""
from __future__ import annotations

import math
import struct
from datetime import datetime, timezone

SPEC_VERSION = "0.2"


# ------------------------------------------------------------------ 시간
def iso_utc(t: float) -> str:
    """epoch 초 -> '2026-10-10T05:12:33.120Z'"""
    return datetime.fromtimestamp(t, timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def parse_iso(s: str) -> float:
    """'...Z' -> epoch 초"""
    return datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp()


def clean(v):
    """NaN, inf -> None (리스트 안도). 나머지는 그대로"""
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return None
    if isinstance(v, (list, tuple)):
        return [clean(x) for x in v]
    return v


def _as_f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def f32(x: float) -> float:
    """float32 필드 값을 가장 짧은 십진수로. ROS 로 한 번 오가면 24.01 이 24.010000228881836 이 되는 것을 되돌린다"""
    if math.isnan(x) or math.isinf(x) or abs(x) > 3.4e38:
        return x
    target = _as_f32(x)
    for digits in range(6, 10):
        y = float(f"{x:.{digits}g}")
        if _as_f32(y) == target:
            return y
    return x


def record(type_name: str, stamp: float, device_id: str, **fields) -> dict:
    """최상위 JSON 기록 만들기"""
    out = {"type": type_name, "version": SPEC_VERSION, "stamp": iso_utc(stamp), "device_id": device_id}
    out.update({k: clean(v) for k, v in fields.items()})
    return out


def body(stamp: float, device_id: str, **fields) -> dict:
    """메시지 안에 들어가는 메시지(예: StationSample.gas) 용. type, version 없음"""
    out = {"stamp": iso_utc(stamp), "device_id": device_id}
    out.update({k: clean(v) for k, v in fields.items()})
    return out


# ------------------------------------------------------------------ ROS 2 변환
def _is_msg(obj) -> bool:
    return hasattr(type(obj), "get_fields_and_field_types")


def _time_to_epoch(t) -> float:
    return t.sec + t.nanosec * 1e-9


def _epoch_to_time(t_obj, epoch: float):
    sec = int(math.floor(epoch))
    t_obj.sec = sec
    t_obj.nanosec = int(round((epoch - sec) * 1e9)) % 1_000_000_000
    return t_obj


def fill_msg(msg, rec: dict):
    """JSON 기록(dict) -> ROS 메시지 객체에 값을 채운다. 중첩 메시지도 같은 규칙.

    msg 는 rclpy 가 만든 메시지 인스턴스. 필드 타입은 메시지 클래스에서 읽는다.
    """
    types = type(msg).get_fields_and_field_types()
    for key, val in rec.items():
        if key in ("type", "version"):
            continue
        if key == "stamp":
            _epoch_to_time(msg.header.stamp, parse_iso(val))
            continue
        if key == "device_id":
            msg.header.frame_id = val
            continue
        if key not in types:
            raise KeyError(f"{type(msg).__name__} 에 {key} 필드가 없음")
        ftype = types[key]
        cur = getattr(msg, key)
        base, is_array = _base_type(ftype)
        if ftype == "builtin_interfaces/Time":
            _epoch_to_time(cur, parse_iso(val))
        elif _is_msg(cur):
            fill_msg(cur, val)
        elif is_array:
            conv = float if base in FLOAT_TYPES else int
            setattr(msg, key, [float("nan") if x is None else conv(x) for x in val])
        elif base in FLOAT_TYPES:
            setattr(msg, key, float("nan") if val is None else float(val))
        elif base == "boolean":
            setattr(msg, key, bool(val))
        elif base == "string":
            setattr(msg, key, str(val))
        else:  # 정수
            setattr(msg, key, int(val))
    return msg


# rclpy 의 get_fields_and_field_types() 는 IDL 이름을 쓴다:
#   float32 -> 'float', float64 -> 'double', bool -> 'boolean', float32[64] -> 'float[64]'
FLOAT_TYPES = ("float", "double")


def _base_type(ftype: str) -> tuple[str, bool]:
    if ftype.startswith("sequence<"):
        return ftype[len("sequence<"):].split(",")[0].rstrip(">"), True
    if "[" in ftype:
        return ftype.split("[")[0], True
    return ftype, False


def msg_to_dict(msg, type_name: str | None = None) -> dict:
    """ROS 메시지 -> JSON 기록. type_name 을 주면 최상위 기록(type, version 포함)"""
    types = type(msg).get_fields_and_field_types()
    out = {"type": type_name, "version": SPEC_VERSION} if type_name else {}
    for key, ftype in types.items():
        val = getattr(msg, key)
        base, is_array = _base_type(ftype)
        short = f32 if base == "float" else float         # float32 는 짧은 십진수로
        if key == "header":
            out["stamp"] = iso_utc(_time_to_epoch(val.stamp))
            out["device_id"] = val.frame_id
        elif ftype == "builtin_interfaces/Time":
            out[key] = iso_utc(_time_to_epoch(val))
        elif _is_msg(val):
            out[key] = msg_to_dict(val)
        elif is_array and base in FLOAT_TYPES:
            out[key] = [clean(short(float(x))) for x in val]       # rclpy 는 numpy 배열로 준다
        elif is_array:
            out[key] = [int(x) for x in val]
        elif isinstance(val, float):
            out[key] = clean(short(val))
        else:
            out[key] = val
    return out
