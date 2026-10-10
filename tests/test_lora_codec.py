"""LoRa 패킷 규격 검증: 펌웨어(C++)와 게이트웨이(파이썬)가 같은 바이트를 같은 값으로 읽는가."""
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gateway"))
from odor_gateway import lora_codec as lc  # noqa: E402

SAMPLE_FIELDS = dict(
    nh3_ppm=1.234, h2s_ppm=0.0123, tvoc_ppb=None, eco2_ppm=None, voc_index=112, nox_index=1,
    temperature_c=-3.21, humidity_pct=61.37, pressure_hpa=None, pm1_0_ugm3=8, pm2_5_ugm3=13, pm10_ugm3=21,
    nh3_raw=12345, h2s_raw=-2468, flags=lc.FLAGS["PUMP_ON"] | lc.FLAGS["TEMP_COMP"],
    sampling_mode=1, pump_duty=180, avg_window_s=60,
)
HEALTH_FIELDS = dict(uptime_s=86400, fw_version="0.2.0", state=2, sensor_ok_mask=0b1101,
                     error_count=7, supply_v=12.05, free_ram=3120)


@pytest.fixture(scope="module")
def cpp_frames():
    if not shutil.which("g++"):
        pytest.skip("g++ 없음")
    exe = Path(tempfile.mkdtemp()) / "lora_host"
    subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-o", str(exe),
                    str(ROOT / "tests" / "lora_host.cpp")], check=True)
    out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.split()
    return [bytes.fromhex(x) for x in out]


def test_python_encoder_matches_firmware_bytes(cpp_frames):
    assert lc.encode_frame(lc.TYPE_SAMPLE, 3, 0xBEEF, 1791600000, SAMPLE_FIELDS) == cpp_frames[0]
    assert lc.encode_frame(lc.TYPE_HEALTH, 3, 0xBEF0, 1791600060, HEALTH_FIELDS) == cpp_frames[1]


def test_decode_firmware_sample(cpp_frames):
    (f,) = lc.FrameParser().feed(cpp_frames[0])
    assert (f.type, f.station, f.station_id, f.seq, f.time) == (lc.TYPE_SAMPLE, 3, "fix_03", 0xBEEF, 1791600000)
    d = f.fields
    assert d["nh3_ppm"] == pytest.approx(1.23)          # 0.01 ppm 단위로 잘림
    assert d["h2s_ppm"] == pytest.approx(0.012)         # 0.001 ppm 단위
    assert d["tvoc_ppb"] is None and d["eco2_ppm"] is None and d["pressure_hpa"] is None
    assert d["voc_index"] == 112 and d["nox_index"] == 1
    assert d["temperature_c"] == pytest.approx(-3.21)
    assert d["humidity_pct"] == pytest.approx(61.37)
    assert (d["pm1_0_ugm3"], d["pm2_5_ugm3"], d["pm10_ugm3"]) == (8, 13, 21)
    assert (d["nh3_raw"], d["h2s_raw"]) == (12345.0, -2468.0)
    assert d["pump_on"] is True and d["sampling_mode"] == 1 and d["avg_window_s"] == 60


def test_decode_firmware_health(cpp_frames):
    (f,) = lc.FrameParser().feed(cpp_frames[1])
    assert f.type == lc.TYPE_HEALTH
    assert f.fields["firmware"] == "fix_station 0.2.0"
    assert f.fields["supply_v"] == pytest.approx(12.05)
    assert f.fields["sensor_ok_mask"] == 0b1101


def test_clamping_and_missing(cpp_frames):
    (f,) = lc.FrameParser().feed(cpp_frames[2])
    d = f.fields
    assert d["nh3_ppm"] == pytest.approx(655.34)        # 넘치면 최대값으로 잘림
    assert d["h2s_ppm"] == 0.0                          # 음수는 0
    assert d["temperature_c"] == pytest.approx(327.67)
    assert d["humidity_pct"] is None and d["nh3_raw"] is None
    assert f.time == 0                                  # 시계 모름


def test_stream_resync_and_crc():
    good = lc.encode_frame(lc.TYPE_SAMPLE, 1, 5, 1791600000, SAMPLE_FIELDS)
    bad = bytearray(good)
    bad[20] ^= 0xFF                                      # 페이로드 깨짐
    junk = b"\x00\xA5\x13\xA5\x5A\x09"                   # 가짜 동기 바이트 포함
    p = lc.FrameParser()
    frames = []
    stream = junk + bytes(bad) + good + good
    for i in range(0, len(stream), 7):                    # 시리얼처럼 조각조각 들어옴
        frames += p.feed(stream[i:i + 7])
    assert [f.seq for f in frames] == [5, 5]
    assert p.crc_errors == 1


def test_rssi_byte():
    frame = lc.encode_frame(lc.TYPE_SAMPLE, 2, 9, 1791600000, SAMPLE_FIELDS)
    (f,) = lc.FrameParser(rssi_byte=True).feed(frame + bytes([256 - 87]))
    assert f.rssi_dbm == -87


def test_crc_reference_value():
    # CRC-16/CCITT-FALSE 표준 확인값: "123456789" -> 0x29B1
    assert lc.crc16_ccitt(b"123456789") == 0x29B1


def test_frame_lengths():
    assert lc.HEADER_LEN + lc.SAMPLE.size + lc.CRC_LEN == 52
    assert lc.HEADER_LEN + lc.HEALTH.size + lc.CRC_LEN == 30


@pytest.mark.parametrize("v", [0.0, 0.004, 12.345, 65.5])
def test_h2s_resolution(v):
    f = dict(SAMPLE_FIELDS, h2s_ppm=v)
    (fr,) = lc.FrameParser().feed(lc.encode_frame(lc.TYPE_SAMPLE, 1, 1, 1, f))
    assert math.isclose(fr.fields["h2s_ppm"], min(v, 65.534), abs_tol=0.0005 + 1e-9)


def _doc_vector(kind: str) -> bytes:
    import re
    text = (ROOT / "protocol" / "LORA_PACKET.md").read_text(encoding="utf-8")
    m = re.search(rf"```vector-{kind}\n(.*?)\n```", text, re.S)
    return bytes.fromhex(m.group(1))


def test_document_vectors_decode_and_reencode():
    """LORA_PACKET.md 6장의 시험 벡터가 코드와 맞는가 (문서가 틀리면 여기서 걸린다)"""
    raw = _doc_vector("sample")
    (f,) = lc.FrameParser().feed(raw)
    d = f.fields
    assert (f.station_id, f.seq, f.time, len(raw)) == ("fix_01", 1342, 1791609120, 52)
    assert d["nh3_ppm"] == pytest.approx(1.84) and d["h2s_ppm"] == pytest.approx(0.031)
    assert d["tvoc_ppb"] is None and d["eco2_ppm"] is None and d["pressure_hpa"] is None and d["nh3_raw"] is None
    assert d["temperature_c"] == pytest.approx(-3.25) and d["h2s_raw"] == 6213.0
    assert lc.encode_frame(lc.TYPE_SAMPLE, 1, 1342, 1791609120, d) == raw

    raw = _doc_vector("health")
    (h,) = lc.FrameParser().feed(raw)
    assert (h.station_id, h.seq, h.time, len(raw)) == ("fix_03", 77, 0, 30)
    assert h.fields["sensor_ok_mask"] == 0x3F and h.fields["supply_v"] == pytest.approx(12.31)
    assert h.fields["firmware"] == "fix_station 0.2.0" and h.fields["free_ram"] == 5120
