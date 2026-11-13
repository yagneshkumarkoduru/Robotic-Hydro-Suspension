"""
Active Hydro-Pneumatic Suspension Dynamics & Optimal Control - presentation layer.

All physics, tuning, and numbers come from the shared harness
(``suspension_benchmark.py``): one 2-DOF quarter-car plant, one metric set,
one tuning procedure per controller class. Nothing here is hardcoded.

Run ``python suspension_benchmark.py`` for the full comparison (bump and ISO
class-C road, Monte Carlo) and the machine-readable results JSON.
"""

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from suspension_benchmark import QuarterCarParams, controller_suite, run_full_benchmark, tune_all


def run_simulation():
    print("=" * 75)
    print("ACTIVE HYDRO-PNEUMATIC SUSPENSION CONTROL BENCHMARK (shared harness)")
    print("=" * 75)

    params = QuarterCarParams()
    payload, trajectories = run_full_benchmark(params, mc_samples=10)
    bump = payload["scenarios"]["bump"]["controllers"]

    print(f"\n{'Control Strategy':<16} | {'RMS Accel (m/s2)':<17} | {'Peak Accel':<12} | {'Peak Stroke (mm)':<16} | {'Violations'}")
    print("-" * 90)
    for name, m in bump.items():
        print(
            f"{name:<16} | {m['rms_accel_m_s2']:<17.3f} | {m['peak_accel_m_s2']:<12.3f} | "
            f"{m['peak_stroke_mm']:<16.2f} | {m['stroke_violation_samples']}"
        )
    print("-" * 90)

    p = bump["passive"]["rms_accel_m_s2"]
    for name, m in bump.items():
        if name != "passive":
            print(f"{name} ride comfort gain vs passive: {(1.0 - m['rms_accel_m_s2'] / p) * 100:.1f}% "
                  f"(violations: {m['stroke_violation_samples']})")

    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures")
    os.makedirs(output_dir, exist_ok=True)

    colors = {"passive": "#C0392B", "skyhook": "#2980B9", "lqr": "#8E44AD",
              "preview_lqr": "#E67E22", "hocbf_lqr": "#27AE60"}

    fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(8.5, 6.0), sharex=True)
    ref = trajectories["passive"]
    ax1.plot(ref["t"], ref["zr"] * 1000, "k--", label="road profile", alpha=0.6)
    for name, traj in trajectories.items():
        ax1.plot(traj["t"], traj["x"][:, 0] * 1000, color=colors.get(name, "gray"), alpha=0.85, label=name)
    ax1.set_ylabel("chassis height (mm)")
    ax1.set_title("45 mm bump response (measured, shared harness)")
    ax1.legend(loc="upper right")
    for name, traj in trajectories.items():
        ax2.plot(traj["t"], traj["accel"], color=colors.get(name, "gray"), alpha=0.8, label=name)
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("body accel (m/s2)")
    ax2.legend(loc="upper right")
    fig1.tight_layout()
    fig1.savefig(os.path.join(output_dir, "fig1_bump_response_comparison.png"), dpi=200)
    plt.close(fig1)

    fig2, (ax3, ax4) = plt.subplots(2, 1, figsize=(8.5, 6.0), sharex=True)
    for name, traj in trajectories.items():
        z_rel = (traj["x"][:, 0] - traj["x"][:, 2]) * 1000
        z_tire = (traj["x"][:, 2] - traj["zr"]) * 1000
        ax3.plot(traj["t"], z_rel, color=colors.get(name, "gray"), alpha=0.85, label=name)
        ax4.plot(traj["t"], z_tire, color=colors.get(name, "gray"), alpha=0.85, label=name)
    ax3.axhline(38.0, color="r", linestyle=":")
    ax3.axhline(-38.0, color="r", linestyle=":")
    ax3.set_ylabel("suspension stroke (mm)")
    ax3.legend(loc="upper right")
    ax4.set_xlabel("time (s)")
    ax4.set_ylabel("tire deflection (mm)")
    ax4.legend(loc="upper right")
    fig2.tight_layout()
    fig2.savefig(os.path.join(output_dir, "fig2_suspension_deflection_tradeoff.png"), dpi=200)
    plt.close(fig2)

    print(f"\nfigures written to {output_dir}")


if __name__ == "__main__":
    run_simulation()
