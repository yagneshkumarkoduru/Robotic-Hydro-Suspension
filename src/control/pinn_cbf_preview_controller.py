#!/usr/bin/env python3
"""
HOCBF-filtered active suspension - presentation layer.

The high-order control barrier function (HOCBF) stroke filter is evaluated on
the shared harness plant (``suspension_benchmark.py``) with the true plant
dynamics. The earlier version of this file also added an untrained random
"PINN residual" to both the controller and the plant; that network is removed:
it had no trained parameters and its only effect was to make the controller's
internal model match the plant by construction, which is not evidence of
anything. The barrier filter is the repository's real contribution and it is
measured here against the passive baseline and the unfiltered LQR.

Run ``python suspension_benchmark.py`` for the full comparison (bump and ISO
class-C road, Monte Carlo) and the machine-readable results JSON.
"""

import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from suspension_benchmark import (  # noqa: E402
    HOCBFLQR,
    LQR,
    Passive,
    QuarterCarParams,
    Scenario,
    metrics,
    road_profile,
    simulate,
    tune_all,
)


class HOCBFPreviewController:
    """Thin alias over the harness HOCBF-filtered LQR (kept for compatibility)."""

    def __init__(self, ms=15.0, mus=2.5, ks=950.0, cs=45.0, kt=6500.0, ct=5.0, delta_max=0.038):
        self.params = QuarterCarParams(ms=ms, mus=mus, ks=ks, cs=cs, kt=kt, ct=ct, stroke_max=delta_max)
        gains = tune_all(self.params)
        self.controller = HOCBFLQR(self.params, gains["hocbf"]["g1"], gains["hocbf"]["g2"])
        self.delta_max = delta_max

    def filter_control(self, u_des, x, delta_p=0.0, u_min=None, u_max=None):
        """Project a desired force onto the barrier constraint (closed form)."""
        return self.controller.filter(
            u_des,
            x,
            road_now=0.0,
            params=self.params,
        )


def run_pinn_cbf_benchmark():
    params = QuarterCarParams()
    sc = Scenario()
    gains = tune_all(params)

    passive = Passive()
    lqr = LQR(params)
    hocbf = HOCBFLQR(params, gains["hocbf"]["g1"], gains["hocbf"]["g2"])

    traj_p = simulate(passive, params, sc)
    traj_lqr = simulate(lqr, params, sc)
    traj_h = simulate(hocbf, params, sc)
    m_p = metrics(traj_p, params)
    m_lqr = metrics(traj_lqr, params)
    m_h = metrics(traj_h, params)

    print("=" * 78)
    print("HOCBF STROKE FILTER BENCHMARK (shared harness, true plant dynamics)")
    print("=" * 78)
    for name, m in (("passive", m_p), ("lqr (no filter)", m_lqr), ("hocbf-filtered lqr", m_h)):
        print(
            f"[+] {name:<20} rms accel {m['rms_accel_m_s2']:6.3f} m/s^2 | "
            f"peak accel {m['peak_accel_m_s2']:6.3f} | peak stroke {m['peak_stroke_mm']:6.2f} mm | "
            f"violations {m['stroke_violation_samples']}"
        )
    print(f"[+] comfort gain vs passive (filtered): {(1 - m_h['rms_accel_m_s2'] / m_p['rms_accel_m_s2']) * 100:.1f}%")
    print(f"[+] unfiltered LQR comfort gain: {(1 - m_lqr['rms_accel_m_s2'] / m_p['rms_accel_m_s2']) * 100:.1f}% "
          f"at {m_lqr['stroke_violation_samples']} stroke violations")
    print(f"[+] stroke limit {params.stroke_max * 1e3:.0f} mm: passive {m_p['peak_stroke_mm']:.1f} mm, "
          f"filtered {m_h['peak_stroke_mm']:.1f} mm")

    fig_dir = _ROOT / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    t, zr, _ = road_profile(sc)
    fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axs[0].plot(t, zr * 1000, "k--", label="road bump (45 mm)")
    axs[0].plot(t, (traj_p["x"][:, 0] - traj_p["x"][:, 2]) * 1000, "r-",
                label=f"passive stroke (peak {m_p['peak_stroke_mm']:.1f} mm)")
    axs[0].plot(t, (traj_h["x"][:, 0] - traj_h["x"][:, 2]) * 1000, "b-",
                label=f"HOCBF-filtered stroke (peak {m_h['peak_stroke_mm']:.1f} mm)")
    axs[0].axhline(params.stroke_max * 1e3, color="r", linestyle=":")
    axs[0].axhline(-params.stroke_max * 1e3, color="r", linestyle=":")
    axs[0].set_ylabel("stroke (mm)")
    axs[0].set_title("HOCBF stroke filter on the shared harness (measured)")
    axs[0].legend(loc="upper right")
    axs[1].plot(t, traj_h["F"], "g-", label="filtered actuator force (N)")
    axs[1].set_ylabel("force (N)")
    axs[1].set_xlabel("time (s)")
    axs[1].legend(loc="upper right")
    fig.tight_layout()
    out_png = fig_dir / "fig_pinn_hocbf_safety_verification.png"
    fig.savefig(out_png, dpi=200)
    plt.close(fig)
    print(f"[+] figure: {out_png}")
    return {"passive": m_p, "lqr": m_lqr, "hocbf_lqr": m_h}


if __name__ == "__main__":
    run_pinn_cbf_benchmark()
