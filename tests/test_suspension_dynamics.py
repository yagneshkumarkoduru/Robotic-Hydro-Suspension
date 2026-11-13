"""Quarter-car suspension harness invariants (no plotting, no hardware).

The shared harness in suspension_benchmark.py produces the repository's
controller-comparison numbers; these tests pin the mathematical invariants and
the safety properties that comparison relies on.
"""

import numpy as np
import pytest

pytest.importorskip("scipy")

from suspension_benchmark import (
    HOCBFLQR,
    LQR,
    Passive,
    QuarterCarParams,
    Scenario,
    Skyhook,
    controller_suite,
    iso_road,
    metrics,
    road_profile,
    simulate,
    tune_all,
)

PARAMS = QuarterCarParams()
SCENARIO = Scenario()


def test_equilibrium_state_has_zero_acceleration():
    sc = Scenario(bump_height=0.0)
    traj = simulate(Passive(), PARAMS, sc)
    assert np.max(np.abs(traj["accel"])) < 1e-9


def test_road_bump_is_zero_before_start_and_deterministic():
    t, zr, dzr = road_profile(SCENARIO)
    assert zr[0] == 0.0
    assert np.all(zr[t < SCENARIO.bump_at] == 0.0)
    _, zr2, _ = road_profile(SCENARIO)
    assert np.array_equal(zr, zr2)


def test_iso_road_is_deterministic_and_zero_mean():
    z1, _ = iso_road(2000, 2e-4, 5.0, 20261011)
    z2, _ = iso_road(2000, 2e-4, 5.0, 20261011)
    assert np.array_equal(z1, z2)
    assert abs(float(np.mean(z1))) < 0.01


def test_lqr_changes_sprung_mass_response():
    traj_p = simulate(Passive(), PARAMS, SCENARIO)
    traj_l = simulate(LQR(PARAMS), PARAMS, SCENARIO)
    assert not np.allclose(traj_p["accel"], traj_l["accel"])
    m_p = metrics(traj_p, PARAMS)
    m_l = metrics(traj_l, PARAMS)
    assert m_l["rms_accel_m_s2"] < m_p["rms_accel_m_s2"]


def test_metrics_are_true_rms_and_finite():
    traj = simulate(Passive(), PARAMS, SCENARIO)
    m = metrics(traj, PARAMS)
    manual_rms = float(np.sqrt(np.mean(traj["accel"] ** 2)))
    assert m["rms_accel_m_s2"] == pytest.approx(manual_rms, rel=1e-9)
    for key, value in m.items():
        assert np.isfinite(value), key
        assert value >= 0.0, key


def test_all_controllers_respect_force_limit():
    gains = tune_all(PARAMS)
    for name, ctrl in controller_suite(PARAMS, gains).items():
        traj = simulate(ctrl, PARAMS, SCENARIO)
        assert np.max(np.abs(traj["F"])) <= PARAMS.force_max + 1e-9, name


def test_hocbf_filter_reduces_stroke_violations_vs_plain_lqr():
    gains = tune_all(PARAMS)
    traj_l = simulate(LQR(PARAMS), PARAMS, SCENARIO)
    traj_h = simulate(HOCBFLQR(PARAMS, gains["hocbf"]["g1"], gains["hocbf"]["g2"]), PARAMS, SCENARIO)
    m_l = metrics(traj_l, PARAMS)
    m_h = metrics(traj_h, PARAMS)
    assert m_l["stroke_violation_samples"] > 0
    assert m_h["stroke_violation_samples"] == 0
    assert m_h["peak_stroke_mm"] <= PARAMS.stroke_max * 1e3 + 1e-6


def test_hocbf_filter_projects_force_onto_barrier():
    ctrl = HOCBFLQR(PARAMS, 15.0, 30.0)
    x_at_limit = np.array([PARAMS.stroke_max, 1.0, 0.0, 0.0])  # stroke at the limit, moving apart
    u = ctrl.filter(-1000.0, x_at_limit, road_now=0.0, params=PARAMS)
    assert abs(u) <= PARAMS.force_max
    x_far = np.array([0.0, 0.0, 0.0, 0.0])
    assert ctrl.filter(100.0, x_far, road_now=0.0, params=PARAMS) == pytest.approx(100.0)


def test_skyhook_is_dissipative_only():
    ctrl = Skyhook(120.0)
    x_closing = np.array([0.0, 1.0, 0.0, 0.5])  # v_s * v_rel > 0 -> active
    x_opening = np.array([0.0, 1.0, 0.0, 2.0])  # v_s * v_rel < 0 -> passive
    assert ctrl.force(x_closing, 0.0, 0.0, PARAMS) != 0.0
    assert ctrl.force(x_opening, 0.0, 0.0, PARAMS) == 0.0


def test_tuning_cache_is_stable():
    a = tune_all(PARAMS)
    b = tune_all(PARAMS)
    assert a == b
