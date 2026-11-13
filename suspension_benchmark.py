#!/usr/bin/env python3
"""Shared benchmark harness for the quarter-car suspension study.

One plant, one metric definition set, one tuning procedure per controller
class, used by every script in this repository. All reported numbers come
from this module and are written to results/suspension_benchmark.json.

Plant: 2-DOF quarter-car at robotic-platform scale (documented parameters):

    ms = 15.0 kg    sprung mass (chassis quarter)
    mus = 2.5 kg    unsprung mass (wheel/linkage)
    ks = 950 N/m    suspension spring
    cs = 45 N s/m   passive damping
    kt = 6500 N/m   tire stiffness
    ct = 5 N s/m    tire damping
    stroke limit  +/- 38 mm (rattlespace)
    actuator force limit +/- 400 N (active force / semi-active clip)

State x = [zs, vs, zus, vus] (absolute displacements, m / m/s):

    dvs/dt  = (-ks (zs-zus) - cs (vs-vus) + F_act) / ms
    dvus/dt = ( ks (zs-zus) + cs (vs-vus) - kt (zus-zr) - ct (vus-zr') - F_act) / mus

Roads: a 45 mm cosine bump (the repository's reference disturbance) and an
ISO 8608 class-C random profile generated in-house from the standard
displacement PSD (fixed seed).

Controllers: passive; skyhook semi-active; LQR (CARE design on the linear
plant, force-limited); preview LQR (state feedback plus a gain on the road
height previewed 120 ms ahead, as a look-ahead sensor would provide); HOCBF
filter (the repository's high-order control barrier function on the stroke
constraint, now evaluated with the true plant dynamics and no learned
residual). Every controller is tuned by the same grid procedure with the same
objective: RMS body acceleration plus a heavy penalty on stroke violations.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from scipy.linalg import solve_continuous_are

ROOT = Path(__file__).resolve().parent
RESULTS_DIR = ROOT / "results"


# ---------------------------------------------------------------------------
# Plant and scenarios
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class QuarterCarParams:
    ms: float = 15.0
    mus: float = 2.5
    ks: float = 950.0
    cs: float = 45.0
    kt: float = 6500.0
    ct: float = 5.0
    stroke_max: float = 0.038
    force_max: float = 400.0

    def lqr_gain(self) -> np.ndarray:
        """CARE design on the state [zs-zus, vs, zus-zr, vus] (repo weights)."""
        A = np.array([
            [0.0, 1.0, 0.0, -1.0],
            [-self.ks / self.ms, -self.cs / self.ms, 0.0, self.cs / self.ms],
            [0.0, 0.0, 0.0, 1.0],
            [self.ks / self.mus, self.cs / self.mus, -self.kt / self.mus, -(self.cs + self.ct) / self.mus],
        ])
        B = np.array([[0.0], [1.0 / self.ms], [0.0], [-1.0 / self.mus]])
        Q = np.diag([10.0, 1000.0, 10.0, 1.0])
        R = np.array([[0.001]])
        P = solve_continuous_are(A, B, Q, R)
        return np.linalg.inv(R) @ (B.T @ P)


@dataclass(frozen=True)
class Scenario:
    t_end: float = 2.5
    dt: float = 2.0e-4
    bump_height: float = 0.045
    bump_at: float = 0.4
    bump_duration: float = 0.25
    road_kind: str = "bump"  # "bump" or "iso"
    iso_speed_m_s: float = 5.0
    iso_seed: int = 20261011
    preview_s: float = 0.12


def iso_road(n: int, dt: float, speed: float, seed: int) -> tuple[np.ndarray, np.ndarray]:
    """ISO 8608 class-C random road displacement and slope over time.

    Sum-of-sinusoids from the standard displacement PSD
    S(n) = S0 (n/n0)^-2 with S0 = 256e-6 m^3/cycle at n0 = 0.1 cycle/m
    (class C). Fixed seed; n in [0.011, 2.83] cycle/m.
    """
    rng = np.random.default_rng(seed)
    n_min, n_max, dn = 0.011, 2.83, 0.01
    n_wave = np.arange(n_min, n_max, dn)
    S0 = 256.0e-6
    S = S0 * (n_wave / 0.1) ** (-2.0)
    amp = np.sqrt(2.0 * S * dn)
    phase = rng.uniform(0.0, 2.0 * np.pi, n_wave.size)
    t = np.arange(n) * dt
    x = speed * t
    arg = 2.0 * np.pi * np.outer(x, n_wave) + phase[None, :]
    z = (np.sin(arg) * amp[None, :]).sum(axis=1)
    dzdx = (np.cos(arg) * (2.0 * np.pi * n_wave)[None, :] * amp[None, :]).sum(axis=1)
    return z, dzdx * speed


def road_profile(sc: Scenario) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (t, zr, dzr) for the scenario."""
    n = int(round(sc.t_end / sc.dt)) + 1
    t = np.arange(n) * sc.dt
    if sc.road_kind == "iso":
        zr, dzr = iso_road(n, sc.dt, sc.iso_speed_m_s, sc.iso_seed)
        return t, zr, dzr
    zr = np.zeros(n)
    dzr = np.zeros(n)
    i0 = int(round(sc.bump_at / sc.dt))
    i1 = int(round((sc.bump_at + sc.bump_duration) / sc.dt))
    m = i1 - i0
    phase = np.pi * np.arange(m) / m
    zr[i0:i1] = sc.bump_height * np.sin(phase)
    dzr[i0:i1] = sc.bump_height * np.pi * np.cos(phase) / (m * sc.dt)
    return t, zr, dzr


# ---------------------------------------------------------------------------
# Controllers (each returns an actuator force in N)
# ---------------------------------------------------------------------------

class Passive:
    def reset(self) -> None:
        pass

    def force(self, x: np.ndarray, road_now: float, road_ahead: float, params: QuarterCarParams) -> float:
        return 0.0


class Skyhook:
    """Semi-active skyhook: force opposes sprung-mass velocity when dissipative."""

    def __init__(self, c_sky: float) -> None:
        self.c_sky = c_sky

    def reset(self) -> None:
        pass

    def force(self, x: np.ndarray, road_now: float, road_ahead: float, params: QuarterCarParams) -> float:
        v_s, v_us = x[1], x[3]
        if v_s * (v_s - v_us) >= 0.0:
            return float(np.clip(-self.c_sky * v_s, -params.force_max, params.force_max))
        return 0.0


class LQR:
    """Full-state LQR from the CARE design, force-limited."""

    def __init__(self, params: QuarterCarParams) -> None:
        self.K = params.lqr_gain().reshape(-1)  # 1-D (4,) gain row

    def reset(self) -> None:
        pass

    def force(self, x: np.ndarray, road_now: float, road_ahead: float, params: QuarterCarParams) -> float:
        zs, vs, zus, vus = x
        state = np.array([zs - zus, vs, zus - road_now, vus])
        u = -float(self.K @ state)
        return float(np.clip(u, -params.force_max, params.force_max))


class PreviewLQR(LQR):
    """LQR plus a feedforward gain on the road *change* previewed ahead.

    The preview signal is (zr(t+T) - zr(t)): what the look-ahead sensor sees
    about to arrive, independent of the absolute road height.
    """

    def __init__(self, params: QuarterCarParams, k_preview: float) -> None:
        super().__init__(params)
        self.k_preview = k_preview

    def force(self, x: np.ndarray, road_now: float, road_ahead: float, params: QuarterCarParams) -> float:
        u = super().force(x, road_now, road_ahead, params)
        u_ff = -self.k_preview * (road_ahead - road_now)
        return float(np.clip(u + u_ff, -params.force_max, params.force_max))


class HOCBFLQR(LQR):
    """LQR with the repository's HOCBF stroke filter (true plant dynamics).

    h(x) = stroke_max^2 - (zs - zus)^2 >= 0, enforced to second order with
    class-K gains (g1, g2). The filter is the closed-form projection of the
    desired force onto the linear barrier constraint, intersected with the
    actuator force box.
    """

    def __init__(self, params: QuarterCarParams, g1: float, g2: float) -> None:
        super().__init__(params)
        self.g1, self.g2 = g1, g2

    def filter(self, u_des: float, x: np.ndarray, road_now: float, params: QuarterCarParams) -> float:
        zs, vs, zus, vus = x
        z_rel = zs - zus
        v_rel = vs - vus
        z_tire = zus - road_now
        h = params.stroke_max ** 2 - z_rel ** 2
        Lf_h = -2.0 * z_rel * v_rel
        # Drift accelerations use the true plant (no learned residual)
        dd_zs = (-params.ks * z_rel - params.cs * v_rel) / params.ms
        dd_zus = (params.ks * z_rel + params.cs * v_rel - params.kt * z_tire - params.ct * vus) / params.mus
        Lf2_h = -2.0 * (v_rel ** 2 + z_rel * (dd_zs - dd_zus))
        Lg_Lf_h = -2.0 * z_rel * (1.0 / params.ms + 1.0 / params.mus)
        bound = -(Lf2_h + (self.g1 + self.g2) * Lf_h + self.g1 * self.g2 * h)
        u = float(np.clip(u_des, -params.force_max, params.force_max))
        if abs(Lg_Lf_h) > 1e-9:
            u_req = bound / Lg_Lf_h
            if Lg_Lf_h > 0.0 and u < u_req:
                u = u_req
            elif Lg_Lf_h < 0.0 and u > u_req:
                u = u_req
        return float(np.clip(u, -params.force_max, params.force_max))

    def force(self, x: np.ndarray, road_now: float, road_ahead: float, params: QuarterCarParams) -> float:
        u_des = super().force(x, road_now, road_ahead, params)
        return self.filter(u_des, x, road_now, params)


# ---------------------------------------------------------------------------
# Simulation and metrics
# ---------------------------------------------------------------------------

def simulate(controller, params: QuarterCarParams, sc: Scenario) -> dict[str, np.ndarray]:
    t, zr, dzr = road_profile(sc)
    n = t.size
    x = np.zeros((n, 4))
    F = np.zeros(n)
    controller.reset()

    def deriv(xk: np.ndarray, zrk: float, dzrk: float, uk: float) -> np.ndarray:
        zs, vs, zus, vus = xk
        f_s = params.ks * (zs - zus) + params.cs * (vs - vus)
        f_t = params.kt * (zus - zrk) + params.ct * (vus - dzrk)
        return np.array([
            vs,
            (-f_s + uk) / params.ms,
            vus,
            (f_s - f_t - uk) / params.mus,
        ])

    ip = int(round(sc.preview_s / sc.dt))
    for k in range(n - 1):
        ahead_idx = min(k + ip, n - 1)
        u = controller.force(x[k], zr[k], zr[ahead_idx], params)
        k1 = deriv(x[k], zr[k], dzr[k], u)
        k2 = deriv(x[k] + 0.5 * sc.dt * k1, zr[k], dzr[k], u)
        k3 = deriv(x[k] + 0.5 * sc.dt * k2, zr[k], dzr[k], u)
        k4 = deriv(x[k] + sc.dt * k3, zr[k], dzr[k], u)
        x[k + 1] = x[k] + (sc.dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        F[k] = u
    F[-1] = F[-2]

    accel = np.gradient(x[:, 1], sc.dt)
    return {"t": t, "zr": zr, "x": x, "F": F, "accel": accel}


def metrics(traj: dict[str, np.ndarray], params: QuarterCarParams) -> dict[str, float]:
    x, F = traj["x"], traj["F"]
    z_rel = x[:, 0] - x[:, 2]
    z_tire = x[:, 2] - traj["zr"]
    accel = traj["accel"]
    peak_stroke_mm = float(np.max(np.abs(z_rel)) * 1e3)
    violations = int(np.sum(np.abs(z_rel) > params.stroke_max))
    return {
        "rms_accel_m_s2": float(np.sqrt(np.mean(accel ** 2))),
        "peak_accel_m_s2": float(np.max(np.abs(accel))),
        "peak_stroke_mm": peak_stroke_mm,
        "stroke_violation_samples": violations,
        "rms_tire_deflection_mm": float(np.sqrt(np.mean(z_tire ** 2)) * 1e3),
        "peak_force_n": float(np.max(np.abs(F))),
        "rms_force_n": float(np.sqrt(np.mean(F ** 2))),
    }


# ---------------------------------------------------------------------------
# Tuning (same procedure and objective for every controller class)
# ---------------------------------------------------------------------------

TUNE_SCENARIO = Scenario(t_end=2.0, dt=4.0e-4)


def score(m: dict[str, float], params: QuarterCarParams) -> float:
    """Comfort plus a heavy penalty on stroke violations (documented).

    Public so every controller script (including the MR damper) tunes by the
    identical objective.
    """
    excess = max(0.0, m["peak_stroke_mm"] - params.stroke_max * 1e3)
    return m["rms_accel_m_s2"] + 5.0 * excess


def _tune(params: QuarterCarParams, sc: Scenario, factory, grid: list) -> object:
    best_score, best = float("inf"), grid[0]
    for g in grid:
        m = metrics(simulate(factory(g), params, sc), params)
        s = score(m, params)
        if s < best_score:
            best_score, best = s, g
    return best


def tune_all(params: QuarterCarParams, sc: Scenario | None = None, force: bool = False) -> dict:
    sc = sc or TUNE_SCENARIO
    cache = RESULTS_DIR / "tuned_gains.json"
    if not force and cache.exists():
        try:
            blob = json.loads(cache.read_text(encoding="utf-8"))
            if blob.get("plant") == asdict(params) and blob.get("scenario") == asdict(sc):
                return blob["gains"]
        except (json.JSONDecodeError, KeyError):
            pass

    c_sky = _tune(params, sc, lambda g: Skyhook(g), [60.0, 120.0, 200.0, 320.0])
    k_prev = _tune(
        params, sc,
        lambda g: PreviewLQR(params, g),
        [-6000.0, -3000.0, -1500.0, 0.0, 1500.0, 3000.0, 6000.0],
    )
    g_pair = _tune(
        params, sc,
        lambda g: HOCBFLQR(params, g[0], g[1]),
        [(5.0, 10.0), (15.0, 30.0), (30.0, 60.0), (60.0, 120.0)],
    )
    gains = {
        "skyhook": {"c_sky": c_sky},
        "preview": {"k_preview": k_prev},
        "hocbf": {"g1": g_pair[0], "g2": g_pair[1]},
    }
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    cache.write_text(
        json.dumps({"plant": asdict(params), "scenario": asdict(sc), "gains": gains}, indent=2) + "\n",
        encoding="utf-8",
    )
    return gains


def controller_suite(params: QuarterCarParams, gains: dict) -> dict:
    return {
        "passive": Passive(),
        "skyhook": Skyhook(gains["skyhook"]["c_sky"]),
        "lqr": LQR(params),
        "preview_lqr": PreviewLQR(params, gains["preview"]["k_preview"]),
        "hocbf_lqr": HOCBFLQR(params, gains["hocbf"]["g1"], gains["hocbf"]["g2"]),
    }


def run_full_benchmark(
    params: QuarterCarParams | None = None,
    mc_samples: int = 20,
    mc_seed: int = 20261011,
) -> tuple[dict, dict[str, dict[str, np.ndarray]]]:
    params = params or QuarterCarParams()
    gains = tune_all(params)

    payload: dict = {"plant": asdict(params), "gains": gains, "scenarios": {}}
    trajectories: dict[str, dict[str, np.ndarray]] = {}
    for kind in ("bump", "iso"):
        sc = Scenario(road_kind=kind)
        results = {}
        for name, ctrl in controller_suite(params, gains).items():
            traj = simulate(ctrl, params, sc)
            results[name] = metrics(traj, params)
            if kind == "bump":
                trajectories[name] = traj
        payload["scenarios"][kind] = {"scenario": asdict(sc), "controllers": results}

    # Monte Carlo on the bump scenario: +-15% on ms, ks, cs, kt
    rng = np.random.default_rng(mc_seed)
    sc = Scenario(road_kind="bump")
    mc: dict[str, dict[str, list[float]]] = {
        name: {"rms_accel_m_s2": [], "peak_stroke_mm": []} for name in controller_suite(params, gains)
    }
    for _ in range(mc_samples):
        pert = QuarterCarParams(
            ms=float(params.ms * rng.uniform(0.85, 1.15)),
            mus=params.mus,
            ks=float(params.ks * rng.uniform(0.85, 1.15)),
            cs=float(params.cs * rng.uniform(0.85, 1.15)),
            kt=float(params.kt * rng.uniform(0.85, 1.15)),
            ct=params.ct,
            stroke_max=params.stroke_max,
            force_max=params.force_max,
        )
        for name, ctrl in controller_suite(pert, gains).items():
            m = metrics(simulate(ctrl, pert, sc), pert)
            mc[name]["rms_accel_m_s2"].append(m["rms_accel_m_s2"])
            mc[name]["peak_stroke_mm"].append(m["peak_stroke_mm"])
    payload["monte_carlo"] = {
        "samples": mc_samples,
        "seed": mc_seed,
        "summary": {
            name: {
                "rms_accel_mean": float(np.mean(v["rms_accel_m_s2"])),
                "rms_accel_std": float(np.std(v["rms_accel_m_s2"])),
                "peak_stroke_mean_mm": float(np.mean(v["peak_stroke_mm"])),
            }
            for name, v in mc.items()
        },
    }
    return payload, trajectories


def main() -> int:
    payload, _ = run_full_benchmark()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    out = RESULTS_DIR / "suspension_benchmark.json"
    out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    print("=" * 84)
    print("QUARTER-CAR SUSPENSION: SHARED HARNESS (one plant, two road scenarios)")
    print("=" * 84)
    for kind, block in payload["scenarios"].items():
        print(f"\n[{kind}] road scenario")
        header = f"{'controller':<14} {'rms accel':>10} {'peak accel':>11} {'peak stroke':>12} {'violations':>11} {'rms force':>10}"
        print(header)
        print("-" * len(header))
        for name, m in block["controllers"].items():
            print(
                f"{name:<14} {m['rms_accel_m_s2']:>10.3f} {m['peak_accel_m_s2']:>11.3f} "
                f"{m['peak_stroke_mm']:>10.2f}mm {m['stroke_violation_samples']:>11d} {m['rms_force_n']:>10.1f}"
            )
    print("\nMonte Carlo (+-15% ms, ks, cs, kt; bump):")
    for name, s in payload["monte_carlo"]["summary"].items():
        print(f"  {name:<14} rms accel {s['rms_accel_mean']:.3f} +/- {s['rms_accel_std']:.3f} m/s^2   peak stroke {s['peak_stroke_mean_mm']:.2f} mm")
    print(f"\nwritten: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
