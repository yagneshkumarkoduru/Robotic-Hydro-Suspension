#!/usr/bin/env python3
"""
Hardware Driver & Current Control Loop for Magnetorheological (MR) Damper.
Implements 10 kHz PWM current regulation with coil RL dynamics and back-EMF mitigation.
"""

import numpy as np

class MRDamperCurrentDriver:
    """
    Simulates a high-bandwidth PWM H-Bridge current amplifier driving the MR damper coil.
    Coil electrical parameters: L = 12.5 mH, R = 3.2 Ohms, V_supply = 24.0 V.
    Current feedback: 10 kHz closed-loop PI regulator.
    """
    def __init__(self, L_coil=0.0125, R_coil=3.2, V_bus=24.0, max_current=2.5):
        self.L = L_coil
        self.R = R_coil
        self.V_bus = V_bus
        self.max_current = max_current
        
        self.i_actual = 0.0
        self.integral_error = 0.0
        
        # PI current controller gains
        self.Kp = 45.0
        self.Ki = 1200.0

    def step(self, i_target, dt=0.0001):
        """
        Executes one discrete step of the 10 kHz internal current regulation loop.
        Returns the actual physical current achieved in the coil.
        """
        i_target_clamped = np.clip(i_target, 0.0, self.max_current)
        error = i_target_clamped - self.i_actual
        self.integral_error += error * dt
        
        v_demand = self.Kp * error + self.Ki * self.integral_error
        v_applied = np.clip(v_demand, 0.0, self.V_bus)
        
        # Electrical ODE: di/dt = (V_applied - R*i) / L
        di_dt = (v_applied - self.R * self.i_actual) / self.L
        self.i_actual += di_dt * dt
        self.i_actual = float(np.clip(self.i_actual, 0.0, self.max_current))
        return self.i_actual
