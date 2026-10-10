"""
records.py - LoRa 프레임을 규격 v0.2 JSON 기록(station_sample, device_health)과 CSV 줄로 바꾼다.
ROS 없이 동작한다. ROS 발행은 ros_bridge.py 가 이 기록을 받아서 한다.
"""
from __future__ import annotations

from dataclasses import dataclass

from .lora_codec import FLAGS, TYPE_HEALTH, TYPE_SAMPLE, Frame
from .odor_json import body, iso_utc, record

CLOCK_TOLERANCE_S = 300      # 관측소 시계와 수신 시각이 이보다 벌어지면 TIME_UNSYNCED


@dataclass
class Station:
    id: str
    name: str = ""
    latitude: float | None = None
    longitude: float | None = None
    altitude_msl: float | None = None


def load_stations(path) -> dict[str, Station]:
    import yaml  # 지연 import: 테스트, 시뮬레이터에서 YAML 없이도 돌게

    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    return {sid: Station(id=sid, **(v or {})) for sid, v in raw.get("stations", {}).items()}


def _stamp(frame: Frame, received: float) -> tuple[float, int]:
    """측정 시각과 추가할 플래그. 관측소 시계가 없거나 크게 어긋나면 수신 시각을 쓰고 표시한다."""
    if frame.time == 0:
        return received, FLAGS["TIME_UNSYNCED"]
    if abs(frame.time - received) > CLOCK_TOLERANCE_S:
        return float(frame.time), FLAGS["TIME_UNSYNCED"]
    return float(frame.time), 0


GAS_FIELDS = [
    "h2s_ppm", "nh3_ppm", "tvoc_ppb", "temperature_c", "humidity_pct", "pump_on", "payload_voltage",
    "eco2_ppm", "voc_index", "nox_index", "pressure_hpa", "gas_resistance_ohm",
    "pm1_0_ugm3", "pm2_5_ugm3", "pm10_ugm3", "nh3_raw", "h2s_raw", "flags",
]


def sample_record(frame: Frame, received: float, station: Station | None) -> dict:
    d = frame.fields
    sid = frame.station_id
    st = station or Station(id=sid)
    stamp, extra = _stamp(frame, received)
    gas_vals = {k: d.get(k) for k in GAS_FIELDS}
    gas_vals["payload_voltage"] = None           # 고정형은 상태 보고(supply_v)로 보낸다
    gas_vals["gas_resistance_ohm"] = None
    gas_vals["flags"] = d["flags"] | extra
    return record(
        "station_sample", stamp, sid,
        latitude=st.latitude, longitude=st.longitude, altitude_msl=st.altitude_msl,
        sampling_mode=d["sampling_mode"], avg_window_s=d["avg_window_s"],
        gas=body(stamp, sid, **gas_vals),
        seq=frame.seq, rssi_dbm=frame.rssi_dbm or 0, received=iso_utc(received),
    )


def health_record(frame: Frame, received: float) -> dict:
    d = frame.fields
    stamp, _ = _stamp(frame, received)
    return record(
        "device_health", stamp, frame.station_id,
        state=d["state"], firmware=d["firmware"], uptime_s=d["uptime_s"],
        sensor_ok_mask=d["sensor_ok_mask"], error_count=d["error_count"],
        supply_v=d["supply_v"], cpu_temp_c=None, rssi_dbm=frame.rssi_dbm or 0,
        message="" if d["state"] == 2 else f"free_ram={d['free_ram']}",
    )


def frame_to_record(frame: Frame, received: float, stations: dict[str, Station]) -> dict:
    if frame.type == TYPE_SAMPLE:
        return sample_record(frame, received, stations.get(frame.station_id))
    if frame.type == TYPE_HEALTH:
        return health_record(frame, received)
    raise ValueError(frame.type)


# ------------------------------------------------------------------ CSV
STATION_CSV_COLUMNS = [
    "time_unix", "time_iso", "station_id", "seq", "rssi_dbm", "received_iso", "sampling_mode", "avg_window_s",
    "lat", "lon", "nh3_ppm", "h2s_ppm", "tvoc_ppb", "eco2_ppm", "voc_index", "nox_index",
    "temp_c", "hum_pct", "pressure_hpa", "pm1_0", "pm2_5", "pm10", "nh3_raw", "h2s_raw", "pump_on", "flags",
]


def station_csv_row(rec: dict, stamp_unix: float) -> list:
    g = rec["gas"]

    def f(v, nd):
        return "" if v is None else (f"{v:.{nd}f}" if isinstance(v, float) else str(v))

    return [
        f"{stamp_unix:.3f}", rec["stamp"], rec["device_id"], rec["seq"], rec["rssi_dbm"], rec["received"],
        rec["sampling_mode"], rec["avg_window_s"], f(rec["latitude"], 7), f(rec["longitude"], 7),
        f(g["nh3_ppm"], 3), f(g["h2s_ppm"], 4), f(g["tvoc_ppb"], 0), f(g["eco2_ppm"], 0),
        f(g["voc_index"], 0), f(g["nox_index"], 0), f(g["temperature_c"], 2), f(g["humidity_pct"], 2),
        f(g["pressure_hpa"], 1), f(g["pm1_0_ugm3"], 0), f(g["pm2_5_ugm3"], 0), f(g["pm10_ugm3"], 0),
        f(g["nh3_raw"], 0), f(g["h2s_raw"], 0), int(g["pump_on"]), g["flags"],
    ]
