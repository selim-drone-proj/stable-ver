"""게이트웨이: LoRa 프레임 -> 규격 v0.2 기록 -> 로그 파일. 스키마 검증까지."""
import csv
import json
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gateway"))
from odor_gateway import lora_codec as lc  # noqa: E402
from odor_gateway.gateway import Logger, LossTracker, main  # noqa: E402
from odor_gateway.records import CLOCK_TOLERANCE_S, Station, frame_to_record, load_stations  # noqa: E402
from odor_gateway.simulate import FleetSim, StationSim  # noqa: E402

# 스키마는 moving-ver 에 있다. 같은 폴더에 받아 두었거나 ODOR_SCHEMAS 로 지정하면 검증한다
SCHEMA_DIR = Path(os.environ.get("ODOR_SCHEMAS", ROOT.parent / "moving-ver" / "interfaces" / "schemas"))
TOOLS = SCHEMA_DIR.parents[1] / "tools"


@pytest.fixture(scope="module")
def validate():
    if not (SCHEMA_DIR.exists() and (TOOLS / "validate_jsonl.py").exists()):
        pytest.skip("moving-ver 스키마가 옆에 없음 (ODOR_SCHEMAS 지정)")
    sys.path.insert(0, str(TOOLS))
    import validate_jsonl
    return lambda rec: validate_jsonl.errors(rec, SCHEMA_DIR)


def frames_from_sim(seconds=900, t0=1791609120):
    fleet = FleetSim(n=4, drop_rate=0.0)
    parser = lc.FrameParser()
    out = []
    for i in range(seconds):
        out += [(f, t0 + i + 1.2) for f in parser.feed(fleet.step(t0 + i))]
    return out


def test_simulated_records_match_schema(validate):
    stations = load_stations(ROOT / "gateway" / "stations.yaml")
    recs = [frame_to_record(f, rx, stations) for f, rx in frames_from_sim()]
    kinds = {r["type"] for r in recs}
    assert kinds == {"station_sample", "device_health"}
    for r in recs:
        assert validate(r) == [], r
    s = next(r for r in recs if r["type"] == "station_sample" and r["device_id"] == "fix_03")
    assert s["latitude"] == pytest.approx(35.8236) and s["gas"]["device_id"] == "fix_03"
    assert s["gas"]["flags"] & lc.FLAGS["SIMULATED"]


def test_plume_reaches_stations_in_order():
    # 시작 시각을 냄새 주기(PLUME_PERIOD_S=1800 s)의 배수로 맞춰야 fix_01 부터 차례로 보인다
    t0 = 1791608400
    assert t0 % 1800 == 0
    recs = [frame_to_record(f, rx, {}) for f, rx in frames_from_sim(seconds=1800, t0=t0)]
    first_peak = {}
    for r in recs:
        if r["type"] == "station_sample" and r["gas"]["nh3_ppm"] > 3.0:
            first_peak.setdefault(r["device_id"], r["stamp"])
    assert list(first_peak) == ["fix_01", "fix_02", "fix_03", "fix_04"]


def test_unknown_station_clock_uses_receive_time():
    fr = lc.FrameParser().feed(lc.encode_frame(lc.TYPE_SAMPLE, 2, 7, 0, StationSim(2).sample_fields(0)))[0]
    rec = frame_to_record(fr, 1791609153.0, {})
    assert rec["stamp"] == "2026-10-10T05:12:33.000Z"
    assert rec["gas"]["flags"] & lc.FLAGS["TIME_UNSYNCED"]


def test_drifted_station_clock_is_flagged():
    t = 1791609153
    fr = lc.FrameParser().feed(lc.encode_frame(lc.TYPE_SAMPLE, 1, 1, t - CLOCK_TOLERANCE_S - 60, {}))[0]
    rec = frame_to_record(fr, float(t), {})
    assert rec["gas"]["flags"] & lc.FLAGS["TIME_UNSYNCED"]
    assert rec["latitude"] is None                         # stations.yaml 에 없는 관측소


def test_loss_tracker():
    lt = LossTracker()
    assert [lt.update("fix_01", s) for s in (10, 11, 14, 15)] == [0, 0, 2, 0]
    assert lt.update("fix_01", 0) == 0                     # 재부팅
    assert lt.update("fix_02", 65535) == 0 and lt.update("fix_02", 1) == 1   # 16비트 넘어감


def test_logger_writes_jsonl_and_csv(tmp_path):
    stations = {"fix_01": Station("fix_01", latitude=35.0, longitude=127.0, altitude_msl=40.0)}
    log = Logger(tmp_path)
    for f, rx in frames_from_sim(seconds=200):
        log.write(frame_to_record(f, rx, stations), f.time or rx)
    log.close()
    lines = list(tmp_path.glob("gateway_*.jsonl"))[0].read_text().splitlines()
    assert all(json.loads(x)["version"] == "0.2" for x in lines)
    rows = list(csv.reader(open(list(tmp_path.glob("station_fix_01_*.csv"))[0])))
    assert rows[0][:3] == ["time_unix", "time_iso", "station_id"] and len(rows) > 2
    assert rows[1][8] == "35.0000000"


def test_cli_simulate_end_to_end(tmp_path, validate):
    rc = main(["--simulate", "--speed", "100000", "--out", str(tmp_path), "--max-frames", "10", "--quiet",
               "--stations", str(ROOT / "gateway" / "stations.yaml")])
    assert rc == 0
    recs = [json.loads(x) for p in tmp_path.glob("*.jsonl") for x in p.read_text().splitlines()]
    assert any(r["device_id"] == "gw_01" for r in recs)
    for r in recs:
        assert validate(r) == []
