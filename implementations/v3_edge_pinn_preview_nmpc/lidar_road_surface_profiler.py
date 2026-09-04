#!/usr/bin/env python3
"""
Forward LiDAR Road Profile Synthesizer.
Extracts road elevation profiles over a 120 ms preview lookahead horizon
from simulated 3D point clouds and planar ground segmentation.
"""

import numpy as np

class LiDARRoadProfiler:
    """
    Simulates forward-facing LiDAR elevation extraction:
      Vehicle speed v_x (m/s) * lookahead time T_preview (s) = lookahead distance d_prev (m).
    """
    def __init__(self, vehicle_speed_mps=10.0, preview_horizon_s=0.120, sample_rate_hz=1000):
        self.v_x = vehicle_speed_mps
        self.T_prev = preview_horizon_s
        self.dt = 1.0 / sample_rate_hz
        self.num_preview_steps = int(self.T_prev / self.dt) # 120 steps

    def extract_preview_vector(self, current_time, road_function):
        """
        Returns an array of future road heights [z_r(t), z_r(t + dt), ..., z_r(t + T_prev)].
        """
        future_times = current_time + np.arange(self.num_preview_steps) * self.dt
        return road_function(future_times)
