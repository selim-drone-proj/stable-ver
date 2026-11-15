"""관제 서버 전송(forward.py): 변환 규칙, spool, 서버 응답별 처리, 게이트웨이와 함께 끝까지"""
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gateway"))
from odor_gateway import lora_codec as lc  # noqa: E402
from odor_gateway.forward import Forwarder, to_server_records  # noqa: E402
from odor_gateway.gateway import main  # noqa: E402
from odor_gateway.records import Station, frame_to_record  # noqa: E402

T0 = 1791609153
REGISTERED = {"odor": {"FX-01", "FX-02", "FX-03", "DX-01"}, "battery": {"DRN-01"}}
MOBILE = dict(latitude=35.8243715, longitude=127.1486021, altitude_msl=60.2, altitude_rel=20.1, heading_deg=87.5,
              ground_speed_mps=3.1, armed=True, in_air=True, gps_fix_type=3, gps_satellites=15, battery_voltage=23.4,
              nh3_ppm=0.83, h2s_ppm=0.021, tvoc_ppb=240.0, eco2_ppm=520.0, temperature_c=23.1, humidity_pct=58.4,
              pressure_hpa=1008.4, gas_resistance_ohm=52300.0, flags=1024, drone_age_s=0.3)


def rec_from(ftype, station, fields, stations=None):
    (fr,) = lc.FrameParser().feed(lc.encode_frame(ftype, station, 1, T0, fields))
    return frame_to_record(fr, T0 + 1, stations or {})


# ------------------------------------------------------------------ 가짜 서버 (odor_system /api/ingest 의 응답 형식)
class FakeIngest:
    def __init__(self):
        self.status = 200
        self.batches = []
        outer = self

        class H(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                outer.batches.append((self.path, self.headers.get("X-Device-Token"), body["records"]))
                if outer.status != 200:
                    out = {"error": "fake"}
                else:
                    rej = [{"index": i, "error": "등록되지 않은 장비"} for i, r in enumerate(body["records"])
                           if (r.get("sensor_id") or r.get("drone_id")) not in REGISTERED[r["kind"]]]
                    out = {"received": len(body["records"]), "accepted": len(body["records"]) - len(rej),
                           "duplicates": 0, "rejected": rej, "stored": False}
                data = json.dumps(out).encode()
                self.send_response(outer.status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *a):
                pass

        self.httpd = HTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}/api/ingest"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()


@pytest.fixture
def server():
    s = FakeIngest()
    yield s
    s.close()


# ------------------------------------------------------------------ 변환
def test_mobile_to_odor_and_battery():
    odor, batt = to_server_records(rec_from(lc.TYPE_MOBILE, 1, MOBILE))
    assert odor == {"kind": "odor", "sensor_id": "DX-01", "ts": "2026-10-10T05:12:33.000Z",
                    "nh3_ppb": 830.0, "h2s_ppb": 21.0, "so2_ppb": None,
                    "lat": 35.8243715, "lon": 127.1486021, "alt_m": 20.1}
    assert batt["kind"] == "battery" and batt["drone_id"] == "DRN-01" and batt["voltage_v"] == 23.4
    assert batt["coulomb_ah"] is None and batt["lat"] == 35.8243715


def test_station_uses_install_coordinates():
    st = {"fix_02": Station("fix_02", latitude=35.8251, longitude=127.1492, altitude_msl=41.0)}
    (r,) = to_server_records(rec_from(lc.TYPE_SAMPLE, 2, {"nh3_ppm": 1.84, "h2s_ppm": 0.031, "flags": 0}, st))
    assert r == {"kind": "odor", "sensor_id": "FX-02", "ts": "2026-10-10T05:12:33.000Z",
                 "nh3_ppb": 1840.0, "h2s_ppb": 31.0, "so2_ppb": None, "lat": 35.8251, "lon": 127.1492}


@pytest.mark.parametrize("flag", ["WARMUP", "ZERO_CAL", "PURGE"])
def test_not_outdoor_gas_is_not_sent_but_battery_is(flag):
    rows = to_server_records(rec_from(lc.TYPE_MOBILE, 1, {**MOBILE, "flags": lc.FLAGS[flag]}))
    assert [r["kind"] for r in rows] == ["battery"]


def test_simulated_and_warmup_only_in_dry_run():
    rec = rec_from(lc.TYPE_MOBILE, 1, {**MOBILE, "flags": lc.FLAGS["SIMULATED"] | lc.FLAGS["WARMUP"]})
    assert to_server_records(rec) == []
    assert len(to_server_records(rec, send_all=True)) == 2


def test_no_position_and_no_gas():
    f = {**MOBILE, "latitude": None, "longitude": None, "nh3_ppm": None, "h2s_ppm": None, "battery_voltage": None}
    assert to_server_records(rec_from(lc.TYPE_MOBILE, 1, f)) == []
    f = {**MOBILE, "latitude": None, "longitude": None}
    odor = to_server_records(rec_from(lc.TYPE_MOBILE, 1, f))[0]
    assert "lat" not in odor and "alt_m" not in odor


def test_health_is_not_sent():
    assert to_server_records({"type": "device_health"}) == []


# ------------------------------------------------------------------ 전송, spool
def test_send_and_log_rejected(server, tmp_path):
    fw = Forwarder(server.url, "tok", tmp_path)
    fw.submit(rec_from(lc.TYPE_MOBILE, 1, MOBILE))
    fw.submit(rec_from(lc.TYPE_SAMPLE, 4, {"nh3_ppm": 1.0, "flags": 0}))      # FX-04 는 서버에 미등록
    assert fw.pending() == 3
    assert fw.flush_once()
    path, token, records = server.batches[0]
    assert path == "/api/ingest" and token == "tok" and len(records) == 3
    assert (fw.accepted, fw.rejected, fw.pending()) == (2, 1, 0)
    (rej,) = [json.loads(x) for x in open(tmp_path / "forward_rejected.jsonl", encoding="utf-8")]
    assert rej["record"]["sensor_id"] == "FX-04" and "등록" in rej["error"]


def test_server_down_keeps_spool_then_recovers(server, tmp_path):
    fw = Forwarder("http://127.0.0.1:9/api/ingest", "tok", tmp_path, timeout_s=1)
    fw.submit(rec_from(lc.TYPE_MOBILE, 1, MOBILE))
    assert not fw.flush_once() and fw.pending() == 2 and "연결 실패" in fw.last_error
    fw2 = Forwarder(server.url, "tok", tmp_path)                 # 재시작해도 spool 이 남아 있다
    assert fw2.pending() == 2 and fw2.flush_once() and fw2.pending() == 0


@pytest.mark.parametrize("status,kept", [(401, True), (403, True), (500, True), (400, False), (413, False)])
def test_http_errors(server, tmp_path, status, kept):
    server.status = status
    fw = Forwarder(server.url, "bad", tmp_path)
    fw.submit(rec_from(lc.TYPE_MOBILE, 1, MOBILE))
    fw.flush_once()
    assert (fw.pending() == 2) is kept
    assert (tmp_path / "forward_rejected.jsonl").exists() is (not kept)


def test_batches_and_dry_run_url(server, tmp_path):
    fw = Forwarder(server.url, "tok", tmp_path, dry_run=True, batch=3)
    for _ in range(4):
        fw.submit(rec_from(lc.TYPE_MOBILE, 1, MOBILE))
    assert fw.flush_once() and fw.pending() == 5
    assert server.batches[0][0] == "/api/ingest?dry_run=1" and len(server.batches[0][2]) == 3


def test_background_thread_drains(server, tmp_path):
    fw = Forwarder(server.url, "tok", tmp_path, period_s=0.05)
    fw.start()
    for _ in range(3):
        fw.submit(rec_from(lc.TYPE_MOBILE, 1, MOBILE))
    for _ in range(100):
        if fw.pending() == 0 and fw.accepted == 6:
            break
        threading.Event().wait(0.05)
    fw.close()
    assert fw.pending() == 0 and fw.accepted == 6


def test_gateway_simulate_forward_dry_run(server, tmp_path, monkeypatch):
    monkeypatch.setenv("ODOR_INGEST_TOKEN", "tok")
    rc = main(["--simulate", "--speed", "1000", "--max-frames", "6", "--quiet", "--out", str(tmp_path),
               "--forward-url", server.url, "--forward-dry-run"])
    assert rc == 0
    sent = [r for _, _, recs in server.batches for r in recs]
    assert server.batches and all(p.endswith("dry_run=1") for p, _, _ in server.batches)
    assert sent and all(r["kind"] == "odor" and r["sensor_id"].startswith("FX-") for r in sent)
