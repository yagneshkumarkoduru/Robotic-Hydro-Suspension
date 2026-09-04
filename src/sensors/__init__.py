"""
Sensors Package Initialization

This module provides a centralized import and initialization 
for all sensor interfaces in the robotic vehicle project.
"""

from .mpu6050 import MPU6050Sensor
from .vl53l0x_lidar import VL53L0XLidar
from .ir_speed_sensor import IRSpeedSensor
from .proximity_sensor import ProximitySensor
from .microwave_radar import MicrowaveRadarSensor

# Define which sensors will be exposed when using 'from sensors import *'
__all__ = [
    'MPU6050Sensor',
    'VL53L0XLidar', 
    'IRSpeedSensor', 
    'ProximitySensor', 
    'MicrowaveRadarSensor'
]

def initialize_all_sensors():
    """
    Initialize all sensor modules with default configurations.
    
    Returns:
        dict: A dictionary of initialized sensor instances
    """
    sensors = {
        'imu': MPU6050Sensor(),
        'lidar': VL53L0XLidar(),
        'speed_sensor': IRSpeedSensor(),
        'proximity': ProximitySensor(),
        'radar': MicrowaveRadarSensor()
    }
    
    # Perform initialization for each sensor
    for name, sensor in sensors.items():
        try:
            sensor.initialize()
        except Exception as e:
            print(f"Error initializing {name} sensor: {e}")
    
    return sensors