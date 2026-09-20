"""Pure-physics actuator tests (no hardware, run in CI).

Covers the two actuator models the suspension work depends on: the
proportional electro-hydraulic servo valve and the magnetorheological
damper. Both are numpy-only and deterministic.
"""

import numpy as np

from src.actuators.mr_damper_actuator import MRDamperActuator
from src.actuators.proportional_servo_valve import ProportionalServoValve


def test_valve_spool_command_is_clipped_to_normalized_range():
    valve = ProportionalServoValve(max_displacement_mm=4.0)
    assert valve.set_spool_position(2.0) == 4.0
    assert valve.set_spool_position(-2.0) == -4.0
    assert valve.set_spool_position(0.5) == 2.0


def test_valve_flow_is_zero_at_zero_displacement_and_sign_follows_pressure():
    valve = ProportionalServoValve()
    valve.set_spool_position(0.0)
    assert valve.compute_flow_rate(1.0e5) == 0.0

    valve.set_spool_position(1.0)
    positive = valve.compute_flow_rate(1.0e5)
    negative = valve.compute_flow_rate(-1.0e5)
    assert positive > 0.0
    assert negative < 0.0


def test_valve_flow_grows_with_displacement():
    valve = ProportionalServoValve()
    valve.set_spool_position(0.25)
    small = abs(valve.compute_flow_rate(1.0e5))
    valve.set_spool_position(1.0)
    large = abs(valve.compute_flow_rate(1.0e5))
    assert large > small


def test_valve_flow_scales_with_pressure_difference():
    valve = ProportionalServoValve()
    valve.set_spool_position(1.0)
    low = valve.compute_flow_rate(0.25e5)
    high = valve.compute_flow_rate(1.0e5)
    assert high > low > 0.0


def test_mr_damper_current_clipping_and_shutdown():
    damper = MRDamperActuator(max_current=2.5)
    assert damper.set_current(3.0) == 2.5
    assert damper.set_current(-1.0) == 0.0
    assert damper.set_current(1.4) == 1.4
    assert damper.get_current() == 1.4

    damper.shutdown()
    assert damper.get_current() == 0.0
    assert damper.is_active is False


def test_mr_damper_default_range_matches_datasheet():
    damper = MRDamperActuator()
    assert damper.max_current == 2.5
    assert np.isclose(damper.set_current(2.5), 2.5)
