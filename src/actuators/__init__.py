"""
Actuators Package Initialization

This module provides a centralized import and initialization 
for all actuator interfaces in the robotic vehicle project.
"""

from .servo_motor import ServoMotor
from .mr_damper_actuator import MRDamperActuator
from .proportional_servo_valve import ProportionalServoValve
from .motor_controller import DCMotor

# Define which actuators will be exposed when using 'from actuators import *'
__all__ = [
    'ServoMotor', 
    'MRDamperActuator',
    'ProportionalServoValve',
    'DCMotor'
]

def initialize_all_actuators():
    """
    Initialize all actuator modules with default configurations.
    
    Returns:
        dict: A dictionary of initialized actuator instances
    """
    actuators = {
        'steering_servo': ServoMotor(pin=18),
        'mr_damper': MRDamperActuator(max_current=2.5),
        'servo_valve': ProportionalServoValve(max_displacement_mm=4.0),
        'drive_motor': DCMotor(enable_pin=24, forward_pin=25, reverse_pin=26)
    }
    
    # Perform initialization for each actuator
    for name, actuator in actuators.items():
        try:
            actuator.initialize()
        except Exception as e:
            print(f"Error initializing {name} actuator: {e}")
    
    return actuators

def emergency_stop_all_actuators(actuators=None):
    """
    Perform emergency stop on all actuators.
    
    Args:
        actuators (dict, optional): Dictionary of actuator instances. 
                                    If None, reinitializes all actuators.
    """
    if actuators is None:
        actuators = initialize_all_actuators()
    
    for name, actuator in actuators.items():
        try:
            actuator.stop()
        except Exception as e:
            print(f"Error stopping {name} actuator: {e}")