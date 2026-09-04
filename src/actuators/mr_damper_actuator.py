"""
Magnetorheological (MR) Damper Actuator Interface.
Controls electromagnetic coil current to dynamically modulate fluid yield shear stress.
"""

import numpy as np

class MRDamperActuator:
    """
    Actuator interface for LORD MRF-132DG Magnetorheological Damper.
    Modulates current I in [0, 2.5] A to control damping force in real time.
    """
    def __init__(self, max_current=2.5):
        self.max_current = max_current
        self.current_cmd = 0.0
        self.is_active = True

    def set_current(self, current_amps):
        """Sets the target coil current in Amperes."""
        self.current_cmd = float(np.clip(current_amps, 0.0, self.max_current))
        return self.current_cmd

    def get_current(self):
        """Returns the current commanded current."""
        return self.current_cmd

    def shutdown(self):
        """De-energizes the coil to baseline minimum damping."""
        self.current_cmd = 0.0
        self.is_active = False
