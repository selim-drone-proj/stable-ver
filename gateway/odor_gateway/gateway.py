"""
gateway.py - 지상국 LoRa 게이트웨이 (gw_01)

E220 수신 모듈(CP2102 USB-TTL 로 PC 에 연결)이 받은 고정형 관측소 패킷을 풀어서
  1) JSON Lines 로그   <out>/gateway_YYYYmmdd.jsonl      (모든 기록, 규격 v0.2)
  2) 관측소별 CSV       <out>/station_<id>_YYYYmmdd.csv
  3) (선택) ROS 2 토픽  /station/<id>/sample, /health/<id>, /health/gw_01
로 내보낸다.

    # 실제 수신
    python -m odor_gateway.gateway --port /dev/ttyUSB0 --stations stations.yaml --out ~/odor_logs
    # 하드웨어 없이 (관측소 4대 시뮬레이터, 60배속)
    python -m odor_gateway.gateway --simulate --speed 60 --stations stations.yaml
    # ROS 2 로도 발행
    ... --ros
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

from .lora_codec import FrameParser
from .odor_json import record
from .records import STATION_CSV_COLUMNS, frame_to_record, load_stations, station_csv_row

GW_ID = "gw_01"
GW_VERSION = "lora_gateway 0.2.0"


class Logger:
    """날짜별 파일에 JSONL 과 관측소별 CSV 를 덧붙인다. 매 줄 flush (전원이 꺼져도 남게)."""

    def __init__(self, out_dir: Path):
        self.out = out_dir
        self.out.mkdir(parents=True, exist_ok=True)
        self._files = {}

    def _open(self, name: str, header: list[str] | None):
        f = self._files.get(name)
        if f is None:
            path = self.out / name
            new = not path.exists()
            f = open(path, "a", encoding="utf-8", newline="")
            if new and header:
                csv.writer(f, lineterminator="\n").writerow(header)
            self._files[name] = f
        return f

    def write(self, rec: dict, stamp_unix: float):
        day = time.strftime("%Y%m%d", time.gmtime(stamp_unix))
        f = self._open(f"gateway_{day}.jsonl", None)
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        f.flush()
        if rec["type"] == "station_sample":
            f = self._open(f"station_{rec['device_id']}_{day}.csv", STATION_CSV_COLUMNS)
            csv.writer(f, lineterminator="\n").writerow(station_csv_row(rec, stamp_unix))
            f.flush()

    def close(self):
        for f in self._files.values():
            f.close()


class LossTracker:
    """관측소별 seq 를 보고 빠진 패킷 수를 센다"""

    def __init__(self):
        self.last = {}
        self.lost = {}

    def update(self, sid: str, seq: int) -> int:
        prev = self.last.get(sid)
        self.last[sid] = seq
        if prev is None:
            return 0
        gap = (seq - prev - 1) & 0xFFFF
        if gap > 1000:            # 관측소 재부팅으로 seq 가 0 부터 다시 시작
            return 0
        self.lost[sid] = self.lost.get(sid, 0) + gap
        return gap


def summary(rec: dict) -> str:
    if rec["type"] == "station_sample":
        g = rec["gas"]

        def v(x, nd):
            return "-" if x is None else f"{x:.{nd}f}"

        return (f"{rec['stamp']} {rec['device_id']} seq={rec['seq']:5d} "
                f"NH3={v(g['nh3_ppm'], 2)} H2S={v(g['h2s_ppm'], 3)} VOC={v(g['voc_index'], 0)} "
                f"T={v(g['temperature_c'], 1)} RH={v(g['humidity_pct'], 0)} PM2.5={v(g['pm2_5_ugm3'], 0)} "
                f"flags=0x{g['flags']:04X} rssi={rec['rssi_dbm']}")
    return f"{rec['stamp']} {rec['device_id']} HEALTH state={rec['state']} up={rec['uptime_s']}s {rec['firmware']}"


def run(args) -> int:
    stations = load_stations(args.stations) if args.stations else {}
    log = Logger(Path(os.path.expanduser(args.out)))
    parser = FrameParser(rssi_byte=args.rssi_byte)
    loss = LossTracker()
    bridge = None
    if args.ros:
        from .ros_bridge import RosBridge     # ROS 가 있을 때만 import
        bridge = RosBridge()

    if args.simulate:
        from .simulate import FleetSim
        sim = FleetSim(n=max(4, len(stations)) if stations else 4)
        sim_t = time.time()
        source = None
    else:
        import serial  # pyserial
        source = serial.Serial(args.port, args.baud, timeout=0.2)

    started = time.time()
    last_gw_health = 0.0
    count = 0
    try:
        while True:
            now = time.time()
            if args.simulate:
                data = sim.step(sim_t)
                sim_t += 1
                received = sim_t
                time.sleep(1.0 / args.speed)
            else:
                data = source.read(256)
                received = now
            for fr in parser.feed(data):
                rec = frame_to_record(fr, received, stations)
                stamp_unix = fr.time or received
                gap = loss.update(fr.station_id, fr.seq)
                log.write(rec, stamp_unix)
                if bridge:
                    bridge.publish(rec)
                count += 1
                if not args.quiet:
                    print(summary(rec) + (f"  (패킷 {gap}개 손실)" if gap else ""), flush=True)
            if now - last_gw_health >= 10:
                last_gw_health = now
                h = record("device_health", received, GW_ID, state=2, firmware=GW_VERSION,
                           uptime_s=int(now - started), sensor_ok_mask=1, error_count=parser.crc_errors,
                           supply_v=None, cpu_temp_c=None, rssi_dbm=0,
                           message=f"frames={count} lost={sum(loss.lost.values())} junk_bytes={parser.dropped_bytes}")
                log.write(h, received)
                if bridge:
                    bridge.publish(h)
            if bridge:
                bridge.spin_once()
            if args.max_frames and count >= args.max_frames:
                return 0
    except KeyboardInterrupt:
        return 0
    finally:
        log.close()
        if bridge:
            bridge.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="고정형 관측소 LoRa 게이트웨이 (규격 v0.2)")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="E220 이 붙은 시리얼 포트 (CP2102)")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--rssi-byte", action="store_true", help="E220 REG3 bit7(RSSI 덧붙이기)을 켰을 때")
    ap.add_argument("--stations", help="관측소 좌표 파일 (stations.yaml)")
    ap.add_argument("--out", default="~/odor_logs", help="로그 폴더")
    ap.add_argument("--ros", action="store_true", help="ROS 2 토픽으로도 발행")
    ap.add_argument("--simulate", action="store_true", help="하드웨어 없이 관측소 4대 시뮬레이터로")
    ap.add_argument("--speed", type=float, default=1.0, help="시뮬레이터 배속")
    ap.add_argument("--max-frames", type=int, default=0, help="이만큼 받으면 끝 (시험용)")
    ap.add_argument("--quiet", action="store_true")
    return run(ap.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
