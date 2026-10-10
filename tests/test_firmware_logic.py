"""펌웨어 계산부(station_logic.h)를 PC 에서 컴파일해 돌린다."""
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent


def test_station_logic_host():
    if not shutil.which("g++"):
        pytest.skip("g++ 없음")
    exe = Path(tempfile.mkdtemp()) / "logic_host"
    subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-o", str(exe),
                    str(HERE / "logic_host.cpp")], check=True)
    r = subprocess.run([str(exe)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
