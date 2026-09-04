"""
Proportional Electro-Hydraulic Servo Valve (EHSV) Interface.
Modulates synthetic hydraulic oil discharge volume and chamber pressure.
"""

import numpy as np

class ProportionalServoValve:
    """
    Industrial proportional servo valve interface (Moog / Parker style).
    Flow equation: Q = C_d * A_valve(u) * sqrt(2 * delta_P / rho).
    """
    def __init__(self, max_displacement_mm=4.0, max_pressure_bar=120.0):
        self.max_disp = max_displacement_mm
        self.max_p = max_pressure_bar
        self.valve_spool_position = 0.0

    def set_spool_position(self, normalized_command):
        """normalized_command in [-1.0, 1.0]: negative for rebound, positive for compression."""
        self.valve_spool_position = float(np.clip(normalized_command, -1.0, 1.0)) * self.max_disp
        return self.valve_spool_position

    def compute_flow_rate(self, delta_p_pa, fluid_density=870.0, cd=0.62):
        """Computes instantaneous fluid flow rate in m^3/s."""
        a_orifice = (abs(self.valve_spool_position) * 1e-3) * 0.005 # Area = disp * width
        sign = np.sign(delta_p_pa)
        q = cd * a_orifice * np.sqrt(2.0 * abs(delta_p_pa) / fluid_density) * sign
        return float(q)
