"""
forward.py - 게이트웨이 기록을 관제 서버(odor_system)의 적재 API 로 보낸다. 표준 라이브러리만 사용.

    POST <url>  헤더 X-Device-Token: <토큰>  본문 {"records": [...]}
    (odor_system web/server.py /api/ingest, docs/HARDWARE_DATA_SPEC.md 5-1-a)

서버 형식은 평면 레코드 (kind, sensor_id, ts, 값, lat, lon, alt_m) 라서 v0.2 기록을 아래처럼 바꾼다.

    odor_sample   -> odor    (DX-01, nh3/h2s ppm x 1000 -> ppb, so2 null, 드론 위치, alt_m = altitude_rel)
                  -> battery (DRN-01, voltage_v = drone.battery_voltage. 전압이 없으면 보내지 않음)
    station_sample-> odor    (FX-0N, 설치 좌표 lat/lon, alt_m 없음)
    device_health -> 보내지 않음 (서버에 받을 곳이 없음)

서버에 받을 칸이 없는 값(TVOC, eCO2, BME688, PM, flags)은 버린다. 원본은 gateway_*.jsonl 에 그대로 남는다.
예열(WARMUP), 영점 보정(ZERO_CAL), 퍼지(PURGE) 중인 가스 값과 가짜 데이터(SIMULATED)는 보내지 않는다
(서버가 flags 를 저장하지 못하므로). 단 dry_run (서버가 검증만 하고 저장 안 함) 일 때는 형식 확인용으로 모두 보낸다.

끊김 대비: 보낼 레코드는 먼저 spool 파일에 적고, 서버가 받으면 지운다. 서버는 (장비, 시각) 중복을 건너뛰므로
같은 레코드를 다시 보내도 안전하다. 네트워크 오류, 5xx, 401/403(토큰 문제)은 남겨 두고 나중에 다시 보낸다.
서버가 거절한 레코드(미등록 장비 등)와 400 응답 묶음은 forward_rejected.jsonl 에 이유와 함께 옮기고 다시 보내지 않는다.
"""
from __future__ import annotations

import json
import os
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

from .lora_codec import FLAGS

# 게이트웨이 장비 ID -> 서버 등록 ID (odor_system config.py). fix_04 는 서버에 등록되어야 받아진다
SERVER_IDS = {
    "fix_01": "FX-01", "fix_02": "FX-02", "fix_03": "FX-03", "fix_04": "FX-04",
    "mob_01": "DX-01",      # 드론 악취 센서 묶음
    "octo_01": "DRN-01",    # 드론 기체 (배터리)
}
NOT_OUTDOOR = FLAGS["WARMUP"] | FLAGS["ZERO_CAL"] | FLAGS["PURGE"]


def _ppb(ppm):
    return None if ppm is None else round(ppm * 1000.0, 3)


def to_server_records(rec: dict, send_all: bool = False) -> list[dict]:
    """v0.2 기록 1건 -> 서버 레코드 0~2건. send_all 이면 flags 로 거르지 않는다 (dry_run 형식 확인용)"""
    t = rec.get("type")
    if t not in ("odor_sample", "station_sample"):
        return []
    g = rec["gas"]
    flags = 0 if send_all else g.get("flags", 0)
    if flags & FLAGS["SIMULATED"]:
        return []
    out = []
    if t == "odor_sample":
        d = rec["drone"]
        pos = {"lat": d["latitude"], "lon": d["longitude"], "alt_m": d["altitude_rel"]}
        if pos["lat"] is None or pos["lon"] is None:
            pos = {}                                  # 위치를 모르면 위치 없이 (서버가 null 로 저장)
        sid = SERVER_IDS.get(g["device_id"], g["device_id"])
        gas = {"nh3_ppb": _ppb(g["nh3_ppm"]), "h2s_ppb": _ppb(g["h2s_ppm"]), "so2_ppb": None}
        if not flags & NOT_OUTDOOR and any(v is not None for v in gas.values()):
            out.append({"kind": "odor", "sensor_id": sid, "ts": rec["stamp"], **gas, **pos})
        if d["battery_voltage"] is not None:
            out.append({"kind": "battery", "drone_id": SERVER_IDS.get(rec["device_id"], rec["device_id"]),
                        "ts": rec["stamp"], "voltage_v": d["battery_voltage"], "coulomb_ah": None,
                        "current_a": None, "cell_temp_c": None, **pos})
    else:
        gas = {"nh3_ppb": _ppb(g["nh3_ppm"]), "h2s_ppb": _ppb(g["h2s_ppm"]), "so2_ppb": None}
        if not flags & NOT_OUTDOOR and any(v is not None for v in gas.values()):
            r = {"kind": "odor", "sensor_id": SERVER_IDS.get(rec["device_id"], rec["device_id"]),
                 "ts": rec["stamp"], **gas}
            if rec["latitude"] is not None and rec["longitude"] is not None:
                r.update(lat=rec["latitude"], lon=rec["longitude"])
            out.append(r)
    return out


class Forwarder:
    """spool 파일을 큐로 쓰고, 별도 스레드가 period_s 마다 묶어서 보낸다 (시리얼 수신을 막지 않게)"""

    def __init__(self, url: str, token: str, out_dir: Path, dry_run: bool = False,
                 batch: int = 500, period_s: float = 5.0, timeout_s: float = 10.0, max_backoff_s: float = 60.0):
        self.url = (url + ("&" if "?" in url else "?") + "dry_run=1") if dry_run else url
        self.token = token
        self.dry_run = dry_run
        self.spool = Path(out_dir) / "forward_spool.jsonl"
        self.rejected_path = Path(out_dir) / "forward_rejected.jsonl"
        self.batch = batch
        self.period_s = period_s
        self.timeout_s = timeout_s
        self.max_backoff_s = max_backoff_s
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.sent = self.accepted = self.duplicates = self.rejected = self.failures = 0
        self.last_error = ""
        self.thread = None

    # ---------------------------------------------------------------- 큐
    def submit(self, rec: dict):
        rows = to_server_records(rec, send_all=self.dry_run)
        if not rows:
            return
        with self.lock, open(self.spool, "a", encoding="utf-8") as f:
            for r in rows:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")

    def pending(self) -> int:
        with self.lock:
            return len(self._read_spool_unlocked())

    def _read_spool_unlocked(self) -> list[str]:
        if not self.spool.exists():
            return []
        with open(self.spool, encoding="utf-8") as f:
            return [ln for ln in f.read().splitlines() if ln.strip()]

    def _drop_first(self, n: int):
        with self.lock:
            lines = self._read_spool_unlocked()[n:]
            tmp = self.spool.with_suffix(".tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                f.writelines(ln + "\n" for ln in lines)
            os.replace(tmp, self.spool)

    def _log_rejected(self, items: list[tuple[str, str]]):
        with open(self.rejected_path, "a", encoding="utf-8") as f:
            for line, err in items:
                f.write(json.dumps({"error": err, "record": json.loads(line)}, ensure_ascii=False) + "\n")

    # ---------------------------------------------------------------- 전송
    def _post(self, records: list[dict]) -> tuple[int, dict]:
        req = urllib.request.Request(
            self.url, data=json.dumps({"records": records}).encode("utf-8"), method="POST",
            headers={"Content-Type": "application/json", "X-Device-Token": self.token})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                return resp.status, json.loads(resp.read().decode("utf-8") or "{}")
        except urllib.error.HTTPError as e:
            try:
                body = json.loads(e.read().decode("utf-8") or "{}")
            except ValueError:
                body = {}
            return e.code, body

    def flush_once(self) -> bool:
        """spool 맨 앞 batch 건을 한 번 보낸다. 다 비웠거나 진행했으면 True, 다시 시도해야 하면 False"""
        with self.lock:
            lines = self._read_spool_unlocked()[: self.batch]
        if not lines:
            return True
        try:
            status, body = self._post([json.loads(ln) for ln in lines])
        except (urllib.error.URLError, OSError, ValueError) as e:
            self.failures += 1
            self.last_error = f"연결 실패: {e}"
            return False
        if status == 200:
            self.sent += len(lines)
            self.accepted += body.get("accepted", 0)
            self.duplicates += body.get("duplicates", 0)
            rej = [(lines[r["index"]], r.get("error", "")) for r in body.get("rejected", [])
                   if isinstance(r.get("index"), int) and 0 <= r["index"] < len(lines)]
            if rej:
                self.rejected += len(rej)
                self._log_rejected(rej)
            self._drop_first(len(lines))
            self.last_error = ""
            return True
        if status in (400, 413):                    # 다시 보내도 같은 결과. 옮겨 두고 넘어간다
            err = f"HTTP {status}: {body.get('error', '')}"
            self.rejected += len(lines)
            self._log_rejected([(ln, err) for ln in lines])
            self._drop_first(len(lines))
            self.last_error = err
            return True
        self.failures += 1                          # 401, 403 (토큰), 5xx: 남겨 두고 다시
        self.last_error = f"HTTP {status}: {body.get('error', '')}"
        return False

    def _loop(self):
        wait = self.period_s
        while not self.stop_event.wait(wait):
            ok = True
            while ok and not self.stop_event.is_set():
                ok = self.flush_once()
                with self.lock:
                    if not self._read_spool_unlocked():
                        break
            wait = self.period_s if ok else min(self.max_backoff_s, wait * 2)

    def start(self):
        self.thread = threading.Thread(target=self._loop, name="forwarder", daemon=True)
        self.thread.start()

    def close(self):
        """스레드를 멈추고 남은 것을 한 번 더 보내 본다. 못 보낸 것은 spool 에 남아 다음 실행 때 보낸다"""
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=self.timeout_s + 1)
        for _ in range(100):
            if self.pending() == 0 or not self.flush_once():
                break

    def status(self) -> str:
        s = (f"fwd_sent={self.sent} fwd_accepted={self.accepted} fwd_dup={self.duplicates} "
             f"fwd_rejected={self.rejected} fwd_fail={self.failures} fwd_pending={self.pending()}")
        return s + (f" fwd_err={self.last_error}" if self.last_error else "")
