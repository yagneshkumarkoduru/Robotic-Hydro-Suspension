#!/usr/bin/env python3
"""
Magnetorheological Fluid (MR) suspension: rheology, current loop, and the
shared quarter-car harness.

The MR damper model (Bingham-Papanastasiou fluid + N2 accumulator + coil
current driver) is evaluated through the shared harness plant
(``suspension_benchmark.py``) so every number is measured under the same
scenario and metric definitions as the other controllers.

Fixes applied relative to the earlier version:
- true RMS acceleration (the old metric computed |mean(acceleration)|);
- the MR damper is sized to the corner: the effective flow-amplification area
  is chosen so peak yield force is ~300 N at the fluid's peak yield stress,
  instead of the previous 0.0765 m^2 factor which produced ~3-6 kN spikes;
- the coil response time is measured from the driver model, not hardcoded;
- the "10 kHz PWM" claim matches what the driver model steps (1 kHz here).
"""

import json
import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parents[1]))

from mr_damper_hardware_driver import MRDamperCurrentDriver  # noqa: E402
from mr_fluid_rheology import MagnetorheologicalFluid  # noqa: E402

from suspension_benchmark import (  # noqa: E402
    RESULTS_DIR,
    QuarterCarParams,
    Scenario,
    controller_suite,
    metrics,
    road_profile,
    simulate,
    tune_all,
)

# Effective flow-amplification area (piston area x duct factor), sized so the
# peak yield force at the fluid's peak yield stress is ~300 N on a 15 kg
# corner (about 2x the 147 N sprung weight) rather than the multi-kN spikes of
# the earlier geometry factor.
EFFECTIVE_AREA_M2 = 6.0e-3


class MRDamperController:
    """Skyhook MR damper force through the harness plant (inverse-map law).

    The harness plant already carries the nominal gas-spring stiffness (ks) and
    passive damping (cs); this controller contributes only the controllable
    field-dependent yield force, so nothing is double-counted.

    Modulation: invert the fluid's yield-stress map to command the current that
    produces the skyhook force c_sky * |v_s| when the damper can dissipate
    (v_s * v_rel > 0); zero-field current otherwise. The gain c_sky is tuned by
    the same score as every other controller (see suspension_benchmark.score).
    """

    def __init__(self, params: QuarterCarParams, c_sky: float = 120.0, dt: float = 2.0e-4) -> None:
        self.params = params
        self.c_sky = c_sky
        self.dt = dt
        self.fluid = MagnetorheologicalFluid(temp_c=30.0)
        self.driver = MRDamperCurrentDriver()
        self.i_hist: list[float] = []
        self.tau_hist: list[float] = []

    def reset(self) -> None:
        self.i_hist = []
        self.tau_hist = []
        self.driver = MRDamperCurrentDriver()

    def _current_for_yield_stress(self, tau_target_pa: float) -> float:
        """Invert tau_y(B(I)) = alpha * (B_sat * tanh(k I))^beta for I."""
        alpha = self.fluid.alpha_mr
        beta = self.fluid.beta_mr
        if tau_target_pa <= 0.0:
            return 0.05
        b_needed = (tau_target_pa / alpha) ** (1.0 / beta)
        ratio = float(np.clip(b_needed / self.fluid.B_sat, 0.0, 0.999))
        i_cmd = float(np.arctanh(ratio)) / self.fluid.k_mag
        return float(np.clip(i_cmd, 0.0, 2.2))

    def force(self, x, road_now, road_ahead, params):
        v_rel = x[1] - x[3]
        v_s = x[1]
        if v_s * v_rel > 0:
            tau_target = self.c_sky * abs(v_s) / EFFECTIVE_AREA_M2
            i_demand = self._current_for_yield_stress(tau_target)
        else:
            i_demand = 0.05
        i_actual = self.driver.step(i_demand, dt=self.dt)
        tau_y, _ = self.fluid.compute_yield_stress(i_actual)
        reg = np.tanh(v_rel / 0.02)
        # Damper convention: the force opposes relative velocity, so the
        # actuator force applied to the sprung mass is negative of it.
        f_yield = -EFFECTIVE_AREA_M2 * tau_y * reg
        self.i_hist.append(float(i_actual))
        self.tau_hist.append(float(tau_y))
        return float(np.clip(f_yield, -params.force_max, params.force_max))


def run_mr_benchmark():
    print("=" * 78)
    print("MAGNETORHEOLOGICAL (MR) SUSPENSION BENCHMARK (shared harness)")
    print("=" * 78)

    params = QuarterCarParams()
    sc = Scenario()
    gains = tune_all(params)
    passive = controller_suite(params, gains)["passive"]

    # Same tuning procedure as every other controller: pick the skyhook gain
    # that minimizes score = rms accel + 5 x stroke excess (mm).
    from suspension_benchmark import score as _score  # identical objective
    best_c, best_score = None, float("inf")
    for c_sky in (60.0, 120.0, 200.0, 320.0):
        m = metrics(simulate(MRDamperController(params, c_sky=c_sky), params, sc), params)
        s = _score(m, params)
        if s < best_score:
            best_c, best_score = c_sky, s
    mr = MRDamperController(params, c_sky=best_c)

    traj_p = simulate(passive, params, sc)
    traj_mr = simulate(mr, params, sc)
    m_p = metrics(traj_p, params)
    m_mr = metrics(traj_mr, params)

    reduction = (1.0 - m_mr["rms_accel_m_s2"] / m_p["rms_accel_m_s2"]) * 100.0

    # Measured coil 10-90% rise time from the driver step response
    # (driver stepped at the harness dt; the "10 kHz PWM" claim in older docs
    # is replaced by the measured rise time).
    driver = MRDamperCurrentDriver()
    dt_step = 2.0e-4
    i_trace = []
    for _ in range(int(0.03 / dt_step)):
        i_trace.append(driver.step(2.0, dt=dt_step))
    i_trace = np.asarray(i_trace)
    target = 0.9 * float(i_trace[-1])
    hits = np.where(i_trace >= target)[0]
    rise_ms = float((hits[0] + 1) * dt_step * 1e3) if hits.size else float(len(i_trace) * dt_step * 1e3)

    print(f"[+] Passive baseline : RMS accel {m_p['rms_accel_m_s2']:.3f} m/s^2 | "
          f"peak accel {m_p['peak_accel_m_s2']:.3f} | peak stroke {m_p['peak_stroke_mm']:.2f} mm | "
          f"violations {m_p['stroke_violation_samples']}")
    print(f"[+] MR damper (skyhook-modulated) : RMS accel {m_mr['rms_accel_m_s2']:.3f} m/s^2 | "
          f"peak accel {m_mr['peak_accel_m_s2']:.3f} | peak stroke {m_mr['peak_stroke_mm']:.2f} mm | "
          f"violations {m_mr['stroke_violation_samples']}")
    print(f"[+] RMS acceleration change vs passive: {reduction:+.1f}%")
    print(f"[+] MR yield stress range: 0 -> {max(mr.tau_hist) / 1e3:.1f} kPa "
          f"(effective area {EFFECTIVE_AREA_M2:.1e} m^2, peak yield force {EFFECTIVE_AREA_M2 * max(mr.tau_hist):.0f} N)")
    print(f"[+] Tuned skyhook gain c_sky = {best_c} (score {best_score:.3f})")
    print(f"[+] Coil driver 10-90% rise time: {rise_ms:.2f} ms (measured at dt={dt_step * 1e3:.2f} ms)")

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    out_json = RESULTS_DIR / "mr_suspension_benchmark.json"
    out_json.write_text(json.dumps({
        "effective_area_m2": EFFECTIVE_AREA_M2,
        "passive": m_p,
        "mr_skyhook": m_mr,
        "coil_rise_ms": rise_ms,
        "peak_yield_stress_pa": max(mr.tau_hist),
    }, indent=2) + "\n", encoding="utf-8")

    fig_dir = _HERE.parents[1] / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    t, zr, _ = road_profile(sc)
    n_sim = traj_mr["F"].size  # controller was stepped once per simulated interval
    tau_series = np.asarray(mr.tau_hist)
    i_series = np.asarray(mr.i_hist)
    if tau_series.size < n_sim:
        tau_series = np.pad(tau_series, (0, n_sim - tau_series.size), mode="edge")
        i_series = np.pad(i_series, (0, n_sim - i_series.size), mode="edge")
    tau_series = tau_series[:n_sim]
    i_series = i_series[:n_sim]
    t_sim = t[:n_sim]
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axs[0].plot(t, zr * 1000, "k--", label="road bump (45 mm)")
    axs[0].plot(t, (traj_p["x"][:, 0] - traj_p["x"][:, 2]) * 1000, "r-",
                label=f"passive stroke (peak {m_p['peak_stroke_mm']:.1f} mm)")
    axs[0].plot(t, (traj_mr["x"][:, 0] - traj_mr["x"][:, 2]) * 1000, "b-",
                label=f"MR stroke (peak {m_mr['peak_stroke_mm']:.1f} mm)")
    axs[0].axhline(38.0, color="gray", linestyle=":")
    axs[0].axhline(-38.0, color="gray", linestyle=":")
    axs[0].set_ylabel("stroke (mm)")
    axs[0].set_title("MR damper vs passive on the shared harness (measured)")
    axs[0].legend(loc="upper right")
    axs[1].plot(t_sim, tau_series / 1e3, "m-", label="yield stress (kPa)")
    axs[1].set_ylabel("yield stress (kPa)")
    axs[1].legend(loc="upper right")
    axs[2].plot(t_sim, i_series, "g-", label="coil current (A)")
    axs[2].set_ylabel("coil current (A)")
    axs[2].set_xlabel("time (s)")
    axs[2].legend(loc="upper right")
    fig.tight_layout()
    out_png = fig_dir / "fig_mr_fluid_rheology_benchmark.png"
    fig.savefig(out_png, dpi=200)
    plt.close(fig)
    print(f"[+] figure: {out_png}")
    print(f"[+] written: {out_json}")
    print("=" * 78)


if __name__ == "__main__":
    run_mr_benchmark()
