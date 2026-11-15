"""
records.py - LoRa 프레임을 규격 v0.2 JSON 기록(station_sample, device_health)과 CSV 줄로 바꾼다.
ROS 없이 동작한다. ROS 발행은 ros_bridge.py 가 이 기록을 받아서 한다.
"""
from __future__ import annotations

from dataclasses import dataclass

from .lora_codec import FLAGS, TYPE_HEALTH, TYPE_MOBILE, TYPE_SAMPLE, Frame
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


def mobile_record(frame: Frame, received: float) -> dict:
    """이동형 요약 패킷 -> odor_sample 기록. 패킷에 없는 값(자세, 비행 모드, PM, 원신호 등)은 null.
    드론 ID 는 octo_NN, 센서 묶음 ID 는 mob_NN (NN = 패킷의 station 번호)"""
    d = frame.fields
    gas_id = frame.station_id
    drone_id = f"octo_{frame.station:02d}"
    stamp, extra = _stamp(frame, received)
    age = d["drone_age_s"]
    drone = body(
        stamp - age if age is not None else stamp, drone_id,
        armed=d["armed"], mode="", in_air=d["in_air"],
        latitude=d["latitude"], longitude=d["longitude"],
        altitude_msl=d["altitude_msl"], altitude_rel=d["altitude_rel"], heading_deg=d["heading_deg"],
        gps_fix_type=d["gps_fix_type"], gps_satellites=d["gps_satellites"],
        roll_deg=None, pitch_deg=None, yaw_deg=None,
        ground_speed_mps=d["ground_speed_mps"], climb_rate_mps=None,
        battery_voltage=d["battery_voltage"], battery_current=None, battery_percent=-1.0,
    )
    gas_vals = {k: d.get(k) for k in GAS_FIELDS}
    gas_vals["pump_on"] = False
    gas_vals["flags"] = d["flags"] | extra
    return record("odor_sample", stamp, drone_id, drone=drone, gas=body(stamp, gas_id, **gas_vals), drone_age_s=age)


def frame_to_record(frame: Frame, received: float, stations: dict[str, Station]) -> dict:
    if frame.type == TYPE_MOBILE:
        return mobile_record(frame, received)
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


# 이동형 링크 시험용 CSV: 패킷마다 위치와 수신 세기를 남겨 거리별 손실, RSSI 를 본다
MOBILE_CSV_COLUMNS = [
    "time_unix", "time_iso", "device_id", "seq", "rssi_dbm", "received_iso", "lat", "lon", "alt_msl_m", "alt_rel_m",
    "heading_deg", "ground_speed_mps", "gps_fix", "gps_sats", "armed", "in_air", "batt_v", "drone_age_s",
    "nh3_ppm", "h2s_ppm", "tvoc_ppb", "eco2_ppm", "temp_c", "hum_pct", "pressure_hpa", "gas_res_ohm", "flags",
]


def mobile_csv_row(rec: dict, stamp_unix: float, seq: int, rssi_dbm: int | None, received: float) -> list:
    d, g = rec["drone"], rec["gas"]

    def f(v, nd):
        return "" if v is None else (f"{v:.{nd}f}" if isinstance(v, float) else str(v))

    return [
        f"{stamp_unix:.3f}", rec["stamp"], g["device_id"], seq, rssi_dbm or 0, iso_utc(received),
        f(d["latitude"], 7), f(d["longitude"], 7), f(d["altitude_msl"], 1), f(d["altitude_rel"], 1),
        f(d["heading_deg"], 2), f(d["ground_speed_mps"], 2), d["gps_fix_type"], d["gps_satellites"],
        int(d["armed"]), int(d["in_air"]), f(d["battery_voltage"], 2), f(rec["drone_age_s"], 1),
        f(g["nh3_ppm"], 2), f(g["h2s_ppm"], 3), f(g["tvoc_ppb"], 0), f(g["eco2_ppm"], 0),
        f(g["temperature_c"], 2), f(g["humidity_pct"], 2), f(g["pressure_hpa"], 1), f(g["gas_resistance_ohm"], 0),
        g["flags"],
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
