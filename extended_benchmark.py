#!/usr/bin/env python3
"""Extended benchmark matrix for the Robotic-Hydro-Suspension project.

Runs every controller (passive, skyhook, LQR, preview LQR, HOCBF-LQR,
MR skyhook) over six deterministic road profiles: the 45 mm cosine bump and
ISO 8608 classes A (smooth), B (good rural), C (urban), D (poor urban) and
E (off-road), records time-domain metrics plus acceleration PSD band
energies, and writes results/extended_benchmark.json.

All numbers are digital simulation (SIMULATION class). The harness is
suspension_benchmark.py; see EVIDENCE.md for the evidence ledger.
"""

from __future__ import annotations

import json
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "implementations" / "v2_magnetorheological_fluid"))

from suspension_benchmark import (  # noqa: E402
    RESULTS_DIR,
    QuarterCarParams,
    Scenario,
    controller_suite,
    metrics,
    simulate,
    tune_all,
)
from implementations.v2_magnetorheological_fluid.mr_suspension_benchmark import (  # noqa: E402
    MRDamperController,
)

# ISO 8608 displacement PSD levels S0 at n0 = 0.1 cycle/m, with the common
# descriptive labels used throughout the project documentation:
#   A (smooth highway) 16e-6, B (good rural) 64e-6, C (urban) 256e-6,
#   D (poor urban) 1024e-6, E (off-road) 4096e-6 m^3/cycle.
ISO_CLASSES = {
    "A": 16.0e-6,
    "B": 64.0e-6,
    "C": 256.0e-6,
    "D": 1024.0e-6,
    "E": 4096.0e-6,
}

ROAD_LABELS = {
    "A": "smooth",
    "B": "good-rural",
    "C": "urban",
    "D": "poor-urban",
    "E": "off-road",
}


def iso_road_psd(n: int, dt: float, speed: float, seed: int, s0: float) -> tuple[np.ndarray, np.ndarray]:
    """ISO 8608 sum-of-sinusoids road with configurable PSD level S0."""
    rng = np.random.default_rng(seed)
    n_min, n_max, dn = 0.011, 2.83, 0.01
    n_wave = np.arange(n_min, n_max, dn)
    S = s0 * (n_wave / 0.1) ** (-2.0)
    amp = np.sqrt(2.0 * S * dn)
    phase = rng.uniform(0.0, 2.0 * np.pi, n_wave.size)
    t = np.arange(n) * dt
    x = speed * t
    arg = 2.0 * np.pi * np.outer(x, n_wave) + phase[None, :]
    z = (np.sin(arg) * amp[None, :]).sum(axis=1)
    dzdx = (np.cos(arg) * (2.0 * np.pi * n_wave)[None, :] * amp[None, :]).sum(axis=1)
    return z, dzdx * speed


def psd_bands(accel: np.ndarray, dt: float) -> dict[str, float]:
    """Band-limited RMS of body acceleration (ISO 2631 relevant bands)."""
    a = accel - float(np.mean(accel))
    n = a.size
    A = np.fft.rfft(a) / n
    f = np.fft.rfftfreq(n, d=dt)
    psd = 2.0 * (np.abs(A) ** 2)
    bands = {"0.5-2Hz": (0.5, 2.0), "2-8Hz": (2.0, 8.0), "8-80Hz": (8.0, 80.0)}
    out: dict[str, float] = {}
    for name, (f0, f1) in bands.items():
        m = (f >= f0) & (f < f1)
        out[name] = float(np.sqrt(np.trapezoid(psd[m], f[m]))) if np.any(m) else 0.0
    return out


def simulate_on_road(controller, params: QuarterCarParams, sc: Scenario, zr, dzr) -> dict[str, np.ndarray]:
    """Same RK4 loop as suspension_benchmark.simulate but on a given road."""
    n = zr.size
    t = np.arange(n) * sc.dt
    x = np.zeros((n, 4))
    F = np.zeros(n)
    controller.reset()

    def deriv(xk, zrk, dzrk, uk):
        zs, vs, zus, vus = xk
        f_s = params.ks * (zs - zus) + params.cs * (vs - vus)
        f_t = params.kt * (zus - zrk) + params.ct * (vus - dzrk)
        return np.array([vs, (-f_s + uk) / params.ms, vus, (f_s - f_t - uk) / params.mus])

    ip = int(round(sc.preview_s / sc.dt))
    for k in range(n - 1):
        ahead = min(k + ip, n - 1)
        u = controller.force(x[k], zr[k], zr[ahead], params)
        k1 = deriv(x[k], zr[k], dzr[k], u)
        k2 = deriv(x[k] + 0.5 * sc.dt * k1, zr[k], dzr[k], u)
        k3 = deriv(x[k] + 0.5 * sc.dt * k2, zr[k], dzr[k], u)
        k4 = deriv(x[k] + sc.dt * k3, zr[k], dzr[k], u)
        x[k + 1] = x[k] + (sc.dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        F[k] = u
    F[-1] = F[-2]
    accel = np.gradient(x[:, 1], sc.dt)
    return {"t": t, "zr": zr, "x": x, "F": F, "accel": accel}


def run_scenario(params, gains, sname: str) -> dict:
    suite = controller_suite(params, gains)
    controllers = dict(suite)
    controllers["mr_skyhook"] = MRDamperController(params, c_sky=gains["skyhook"]["c_sky"])

    if sname == "bump45mm":
        sc = Scenario(road_kind="bump")
        results = {c: _eval(controllers[c], params, sc, None) for c in controllers}
        return {"scenario": asdict(sc), "controllers": results}

    cls = sname.split("_")[1].upper()
    speed = 5.0 if cls in ("A", "B", "C") else 10.0
    sc = Scenario(road_kind="iso", iso_speed_m_s=speed)
    n = int(round(sc.t_end / sc.dt)) + 1
    zr, dzr = iso_road_psd(n, sc.dt, speed, sc.iso_seed, ISO_CLASSES[cls])
    results = {c: _eval(controllers[c], params, sc, (zr, dzr)) for c in controllers}
    return {
        "scenario": asdict(sc),
        "iso_class": cls,
        "road_label": ROAD_LABELS[cls],
        "iso_psd_s0": ISO_CLASSES[cls],
        "controllers": results,
    }


def _eval(ctrl, params, sc, road) -> dict:
    if road is None:
        traj = simulate(ctrl, params, sc)
    else:
        traj = simulate_on_road(ctrl, params, sc, *road)
    m = metrics(traj, params)
    m["accel_psd_band_rms"] = psd_bands(traj["accel"], sc.dt)
    return m


def main() -> int:
    params = QuarterCarParams()
    gains = tune_all(params)

    payload: dict = {
        "plant": asdict(params),
        "gains": gains,
        "iso_psd_s0": ISO_CLASSES,
        "road_labels": ROAD_LABELS,
        "scenarios": {},
    }
    for sname in ("bump45mm", "iso_a", "iso_b", "iso_c", "iso_d", "iso_e"):
        payload["scenarios"][sname] = run_scenario(params, gains, sname)

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    out = RESULTS_DIR / "extended_benchmark.json"
    out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"written: {out}")
    for sname, block in payload["scenarios"].items():
        print(f"\n[{sname}]")
        for cname, m in block["controllers"].items():
            print(
                f"  {cname:<14} rms {m['rms_accel_m_s2']:.3f} m/s^2  peak stroke {m['peak_stroke_mm']:.2f} mm  "
                f"viol {m['stroke_violation_samples']}  psd 2-8Hz {m['accel_psd_band_rms']['2-8Hz']:.3f}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
