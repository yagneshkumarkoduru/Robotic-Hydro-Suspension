"""
Preview control study on the shared harness.

The earlier version of this file described a "preview NMPC" that contained no
optimization (fixed gains and a hardcoded "-35.2%" print). It is replaced by
the harness's preview LQR: full-state LQR plus a feedforward gain on the road
*change* previewed ahead, tuned by the same grid procedure as every other
controller. All numbers are measured.

Run ``python suspension_benchmark.py`` for the full comparison and the
machine-readable results JSON.
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
    LQR,
    Passive,
    PreviewLQR,
    QuarterCarParams,
    Scenario,
    metrics,
    road_profile,
    simulate,
    tune_all,
)


class PreviewMPCSuspensionEngine:
    """Compatibility wrapper over the harness preview-LQR comparison."""

    def __init__(self):
        self.params = QuarterCarParams()

    def simulate(self, t_end=2.5, dt=2.0e-4):
        sc = Scenario(t_end=t_end, dt=dt)
        gains = tune_all(self.params)
        traj_p = simulate(Passive(), self.params, sc)
        traj_l = simulate(LQR(self.params), self.params, sc)
        traj_pv = simulate(PreviewLQR(self.params, gains["preview"]["k_preview"]), self.params, sc)
        t, zr, _ = road_profile(sc)
        return t, zr, traj_p, traj_l, traj_pv

    def generate_plots(self):
        t, zr, traj_p, traj_l, traj_pv = self.simulate()
        m_p = metrics(traj_p, self.params)
        m_l = metrics(traj_l, self.params)
        m_pv = metrics(traj_pv, self.params)

        fig_dir = _ROOT / "figures"
        fig_dir.mkdir(parents=True, exist_ok=True)
        fig1, ax = plt.subplots(figsize=(9.0, 5.2))
        ax.plot(t, traj_p["accel"], color="#7F8C8D", alpha=0.7,
                label=f"passive (RMS {m_p['rms_accel_m_s2']:.2f} m/s2)")
        ax.plot(t, traj_l["accel"], color="#2980B9",
                label=f"LQR (RMS {m_l['rms_accel_m_s2']:.2f} m/s2)")
        ax.plot(t, traj_pv["accel"], color="#27AE60",
                label=f"preview LQR (RMS {m_pv['rms_accel_m_s2']:.2f} m/s2)")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("body accel (m/s2)")
        ax.set_title("Preview control comparison (measured, shared harness)")
        ax.legend(loc="upper right")
        fig1.tight_layout()
        p1 = fig_dir / "fig_mpc_preview_horizon_tracking.png"
        fig1.savefig(p1, dpi=200)
        plt.close(fig1)

        fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.0, 5.8), sharex=True)
        for name, traj, color in (("passive", traj_p, "#7F8C8D"), ("lqr", traj_l, "#2980B9"), ("preview", traj_pv, "#27AE60")):
            ax1.plot(t, (traj["x"][:, 0] - traj["x"][:, 2]) * 1000, color=color, label=name)
            ax2.plot(t, traj["F"], color=color, label=name)
        ax1.axhline(38.0, color="r", linestyle=":")
        ax1.axhline(-38.0, color="r", linestyle=":")
        ax1.set_ylabel("stroke (mm)")
        ax1.legend(loc="upper right")
        ax2.axhline(400.0, color="r", linestyle=":")
        ax2.axhline(-400.0, color="r", linestyle=":")
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("force (N)")
        ax2.legend(loc="upper right")
        fig2.tight_layout()
        p2 = fig_dir / "fig_actuator_stroke_pressure_envelope.png"
        fig2.savefig(p2, dpi=200)
        plt.close(fig2)
        return p1, p2, m_p, m_l, m_pv


def run_preview_mpc_study():
    print("=" * 80)
    print("PREVIEW CONTROL STUDY (shared harness; measured numbers)")
    print("=" * 80)
    engine = PreviewMPCSuspensionEngine()
    p1, p2, m_p, m_l, m_pv = engine.generate_plots()
    print(f"[OK] {p1}")
    print(f"[OK] {p2}")
    print("-" * 80)
    print(f"  passive     RMS {m_p['rms_accel_m_s2']:.3f} m/s2 | stroke {m_p['peak_stroke_mm']:.2f} mm | violations {m_p['stroke_violation_samples']}")
    print(f"  lqr         RMS {m_l['rms_accel_m_s2']:.3f} m/s2 | stroke {m_l['peak_stroke_mm']:.2f} mm | violations {m_l['stroke_violation_samples']}")
    print(f"  preview lqr RMS {m_pv['rms_accel_m_s2']:.3f} m/s2 | stroke {m_pv['peak_stroke_mm']:.2f} mm | violations {m_pv['stroke_violation_samples']}")
    print("=" * 80)


if __name__ == "__main__":
    run_preview_mpc_study()
