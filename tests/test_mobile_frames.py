"""이동형 LoRa 요약 패킷(type 0x03): 디코드, odor_sample 기록, 스키마, 고정형 패킷과 섞여 들어올 때"""
import csv
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gateway"))
from odor_gateway import lora_codec as lc  # noqa: E402
from odor_gateway.gateway import Logger  # noqa: E402
from odor_gateway.records import frame_to_record  # noqa: E402

SCHEMA_DIR = Path(os.environ.get("ODOR_SCHEMAS", ROOT.parent / "moving-ver" / "interfaces" / "schemas"))
TOOLS = SCHEMA_DIR.parents[1] / "tools"

# moving-ver lora_mobile 시험, LORA_PACKET.md 6절과 같은 벡터
VECTOR = bytes.fromhex(
    "a5 5a 01 03 01 07 05 41 c9 c9 6a 27 83 5d 5a 15 45 56 c9 4b 5a 02 c9 00 2e 22 36 01 53 00 15 00 "
    "f0 00 08 02 06 09 d0 16 64 27 0b 02 00 04 0f 0f 03 24 09 d2 84")
FIELDS = dict(
    latitude=35.8243715, longitude=127.1486021, altitude_msl=60.2, altitude_rel=20.1, heading_deg=87.5,
    ground_speed_mps=3.1, armed=True, in_air=True, gps_fix_type=3, gps_satellites=15, battery_voltage=23.4,
    nh3_ppm=0.83, h2s_ppm=0.021, tvoc_ppb=240.0, eco2_ppm=520.0, temperature_c=23.1, humidity_pct=58.4,
    pressure_hpa=1008.4, gas_resistance_ohm=52300.0, flags=1024, drone_age_s=0.3,
)
T0 = 1791609153            # 2026-10-10T05:12:33Z


@pytest.fixture(scope="module")
def validate():
    if not (SCHEMA_DIR.exists() and (TOOLS / "validate_jsonl.py").exists()):
        pytest.skip("moving-ver 스키마가 옆에 없음 (ODOR_SCHEMAS 지정)")
    sys.path.insert(0, str(TOOLS))
    import validate_jsonl
    return lambda rec: validate_jsonl.errors(rec, SCHEMA_DIR)


def test_encoder_matches_vector():
    assert lc.encode_frame(lc.TYPE_MOBILE, 1, 1287, T0, FIELDS) == VECTOR


def test_decode_vector():
    (fr,) = lc.FrameParser().feed(VECTOR)
    assert (fr.type, fr.station, fr.seq, fr.time, fr.station_id) == (lc.TYPE_MOBILE, 1, 1287, T0, "mob_01")
    for k, v in FIELDS.items():
        assert fr.fields[k] == pytest.approx(v), k


def test_record_is_valid_odor_sample(validate):
    (fr,) = lc.FrameParser().feed(VECTOR)
    rec = frame_to_record(fr, T0 + 0.8, {})
    assert validate(rec) == []
    assert rec["type"] == "odor_sample" and rec["device_id"] == "octo_01"
    assert rec["gas"]["device_id"] == "mob_01" and rec["drone"]["device_id"] == "octo_01"
    assert rec["drone"]["stamp"] == "2026-10-10T05:12:32.700Z"          # 가스 시각 - drone_age_s
    assert rec["gas"]["nh3_ppm"] == pytest.approx(0.83) and rec["gas"]["voc_index"] is None
    assert rec["drone"]["roll_deg"] is None and rec["drone"]["battery_percent"] == -1.0


def test_missing_values_record(validate):
    f = {k: None for k in FIELDS}
    f.update(flags=8, armed=False, in_air=False, gps_fix_type=0, gps_satellites=0)
    (fr,) = lc.FrameParser().feed(lc.encode_frame(lc.TYPE_MOBILE, 2, 0, 0, f))
    rec = frame_to_record(fr, T0, {})
    assert validate(rec) == []
    assert rec["drone"]["latitude"] is None and rec["drone_age_s"] is None
    assert rec["gas"]["flags"] == 8 | lc.FLAGS["TIME_UNSYNCED"]           # time 0 -> 수신 시각, 미동기 표시


def test_mixed_with_fixed_station_frames():
    fix = lc.encode_frame(lc.TYPE_SAMPLE, 1, 10, T0, {"nh3_ppm": 1.0, "flags": 0})
    stream = fix + b"\x00\x11" + VECTOR + fix[:20]
    p = lc.FrameParser()
    frames = p.feed(stream[:30]) + p.feed(stream[30:])
    assert [f.station_id for f in frames] == ["fix_01", "mob_01"]
    assert p.crc_errors == 0


def test_corrupted_mobile_frame_rejected():
    bad = bytearray(VECTOR)
    bad[20] ^= 0x01
    p = lc.FrameParser()
    assert p.feed(bytes(bad)) == [] and p.crc_errors == 1


def test_logger_writes_mobile_csv(tmp_path):
    (fr,) = lc.FrameParser(rssi_byte=True).feed(VECTOR + bytes([256 - 97]))
    rec = frame_to_record(fr, T0 + 1, {})
    log = Logger(tmp_path)
    log.write(rec, T0, fr, T0 + 1)
    log.close()
    rows = list(csv.DictReader(open(tmp_path / "mobile_mob_01_20261010.csv", encoding="utf-8")))
    assert len(rows) == 1
    assert rows[0]["seq"] == "1287" and rows[0]["rssi_dbm"] == "-97" and rows[0]["lat"] == "35.8243715"
    assert (tmp_path / "gateway_20261010.jsonl").exists()


def test_document_vector_matches():
    import re
    text = (ROOT / "protocol" / "LORA_PACKET.md").read_text(encoding="utf-8")
    m = re.search(r"```vector-mobile\n(.*?)\n```", text, re.S)
    assert m and bytes.fromhex(m.group(1)) == VECTOR
