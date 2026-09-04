#!/usr/bin/env python3
"""
Top-level entry point for Active Hydro-Pneumatic Suspension Simulation.
"""
import os
import sys

# Ensure src is in sys.path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from simulation.suspension_dynamics import run_simulation

if __name__ == '__main__':
    run_simulation()
