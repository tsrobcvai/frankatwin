"""Compile the native torque math and compare it to the simulator's implementation."""
import shutil
import subprocess

import numpy as np
import pytest

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from frankatwin.force_gap.joint_plan import joint_torque, TAU_LIMIT


def test_native_and_sim_joint_efforts_match_with_saturation(tmp_path):
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("g++ is required for native/sim parity")
    source = tmp_path / "parity.cpp"
    source.write_text(
        """
#include "impedance_math.h"
#include <iostream>
#include <iomanip>
int main() {
  double q,dq,qd,dqd,kp,kd,c,prev,limit,dt;
  std::cout << std::setprecision(17);
  while (std::cin >> q >> dq >> qd >> dqd >> kp >> kd >> c >> prev >> limit >> dt) {
    const double raw = frankatwin::joint_pd(q,dq,qd,dqd,kp,kd,c);
    std::cout << frankatwin::limit_effort(raw,prev,limit,1000.,dt) << "\\n";
  }
}
"""
    )
    executable = tmp_path / "parity"
    compiled = subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-O2",
            "-I",
            str(Path(__file__).resolve().parents[1] / "src"),
            str(source),
            "-o",
            str(executable),
        ],
        capture_output=True,
        text=True,
    )
    assert compiled.returncode == 0, compiled.stderr
    rng = np.random.default_rng(203)
    records, expected = [], []
    for scale in (0.001, 0.01, 1.0, 10.0):
        for _ in range(100):
            q, dq = rng.normal(size=(2, 7))
            qd, dqd = q + rng.normal(size=7) * scale, rng.normal(size=7) * scale
            kp, kd = rng.uniform(50, 600, 7), rng.uniform(5, 40, 7)
            c, previous = rng.normal(size=7) * 3, rng.uniform(-1, 1, 7) * TAU_LIMIT
            dt = rng.choice([0.0005, 0.001, 0.003])
            records.extend(
                np.column_stack(
                    (q, dq, qd, dqd, kp, kd, c, previous, TAU_LIMIT, np.full(7, dt))
                )
            )
            expected.extend(joint_torque(q, dq, qd, dqd, kp, kd, c, previous, dt))
    payload = "\n".join(" ".join(format(v, ".17g") for v in row) for row in records)
    result = subprocess.run(
        [str(executable)], input=payload, capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr
    np.testing.assert_allclose(
        np.fromstring(result.stdout, sep="\n"), expected, atol=1e-12, rtol=1e-12
    )
