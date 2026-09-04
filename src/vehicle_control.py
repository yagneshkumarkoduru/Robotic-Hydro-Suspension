# import RPi.GPIO as GPIO
import gpiozero as GPIO
import time
import logging
from .sensors.mpu6050 import MPU6050Sensor
from .actuators.motor_controller import MotorController
from .communication.bluetooth_controller import BluetoothController

class VehicleController:
    def __init__(self, config):
        """
        Initialize vehicle control system
        
        Args:
            config (dict): System configuration
        """
        self.config = config
        self.logger = logging.getLogger('VehicleController')
        
        # Initialize sensors
        self.imu_sensor = MPU6050Sensor()
        
        # Initialize motor controller
        self.motor_controller = MotorController(
            config['gpio']['dc_motor_pins']
        )
        
        # Initialize communication
        self.bluetooth_controller = BluetoothController(
            config['communication']['bluetooth']
        )
        
        # Calibrate sensors
        self._calibrate_sensors()
    
    def _calibrate_sensors(self):
        """
        Calibrate vehicle sensors
        """
        try:
            self.imu_sensor.calibrate()
            self.logger.info("Sensors calibrated successfully")
        except Exception as e:
            self.logger.error(f"Sensor calibration failed: {e}")
    
    def drive(self, speed, direction):
        """
        Control vehicle movement
        
        Args:
            speed (float): Movement speed (-1 to 1)
            direction (str): Movement direction
        """
        try:
            # Read IMU for stability
            imu_data = self.imu_sensor.read()
            
            # Adjust motor control based on IMU data
            if self._is_stable(imu_data):
                self.motor_controller.set_speed(speed)
                self.motor_controller.set_direction(direction)
                self.logger.info(f"Driving: speed={speed}, direction={direction}")
            else:
                self.logger.warning("Vehicle stability compromised. Stopping motors.")
                self.stop()
        except Exception as e:
            self.logger.error(f"Driving error: {e}")
            self.stop()
    
    def stop(self):
        """
        Stop vehicle movement
        """
        self.motor_controller.stop()
        self.logger.info("Vehicle stopped")
    
    def _is_stable(self, imu_data, threshold=0.5):
        """
        Check vehicle stability based on IMU data
        
        Args:
            imu_data (dict): IMU sensor readings
            threshold (float): Stability threshold
        
        Returns:
            bool: Vehicle stability status
        """
        if not imu_data:
            return False
        
        accel = imu_data['acceleration']
        stability = all(
            abs(axis) < threshold 
            for axis in [accel['x'], accel['y'], accel['z']]
        )
        
        return stability
    
    def emergency_stop(self):
        """
        Immediate emergency stop procedure
        """
        self.stop()
        self.logger.critical("EMERGENCY STOP ACTIVATED")