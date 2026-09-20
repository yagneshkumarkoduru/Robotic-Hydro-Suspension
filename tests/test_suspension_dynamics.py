"""Quarter-car suspension dynamics invariants (no plotting, no hardware).

The simulation in src/simulation/suspension_dynamics.py produces the
repository's controller-comparison numbers; these tests pin the
mathematical invariants that comparison relies on.
"""

import pytest

pytest.importorskip("scipy")

from src.simulation.suspension_dynamics import SuspensionSystem


def _zero_road(_t):
    return 0.0, 0.0


def test_equilibrium_state_has_zero_acceleration():
    system = SuspensionSystem()
    dz = system.dynamics(1.0, [0.0, 0.0, 0.0, 0.0], control_type="passive", road_func=_zero_road)
    assert dz == pytest.approx([0.0, 0.0, 0.0, 0.0], abs=1e-12)


def test_road_bump_is_zero_before_start_and_deterministic():
    system = SuspensionSystem()
    assert system.road_bump(0.0) == (0.0, 0.0)
    assert system.road_bump(0.25) == system.road_bump(0.25)


def test_lqr_control_changes_sprung_mass_acceleration():
    system = SuspensionSystem()
    state = [0.02, 0.0, -0.01, 0.0]
    passive = system.dynamics(1.0, state, control_type="passive", road_func=_zero_road)
    active = system.dynamics(1.0, state, control_type="lqr", road_func=_zero_road)
    assert active != passive
    assert abs(active[1] - passive[1]) > 0.0


def test_unknown_control_type_is_rejected():
    system = SuspensionSystem()
    with pytest.raises(ValueError):
        system.dynamics(0.0, [0.0, 0.0, 0.0, 0.0], control_type="magic", road_func=_zero_road)
