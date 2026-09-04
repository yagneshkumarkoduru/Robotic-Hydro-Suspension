import board
import busio
import adafruit_vl53l0x
import logging
from . import BaseSensor

class VL53L0XLidar(BaseSensor):
    def __init__(self, i2c_bus=1, address=0x29):
        """
        Initialize VL53L0X LIDAR sensor
        
        Args:
            i2c_bus (int): I2C bus number
            address (int): I2C device address
        """
        super().__init__("VL53L0X LIDAR")
        
        try:
            # Create I2C bus
            i2c = busio.I2C(board.SCL, board.SDA)
            
            # Initialize VL53L0X sensor
            self.sensor = adafruit_vl53l0x.VL53L0X(i2c)
            
            # Configure sensor (optional advanced settings)
            self.sensor.measurement_timing_budget = 200000  # 200ms
            
            self.log_info("VL53L0X LIDAR initialized successfully")
        except Exception as e:
            self.log_error(f"Sensor initialization failed: {e}")
            self.sensor = None
    
    def read(self):
        """
        Read distance measurement
        
        Returns:
            dict: Distance measurement in millimeters
        """
        if not self.sensor:
            self.log_error("Sensor not initialized")
            return None
        
        try:
            # Read distance
            distance = self.sensor.range
            
            return {
                'distance_mm': distance,
                'distance_cm': distance / 10,
                'valid_measurement': distance < 8190  # VL53L0X max range
            }
        except Exception as e:
            self.log_error(f"Distance measurement failed: {e}")
            return None
    
    def calibrate(self):
        """
        Calibration method (LIDAR typically doesn't require extensive calibration)
        
        Returns:
            dict: Calibration information (if any)
        """
        try:
            # Perform basic sensor self-test
            self.log_info("Performing LIDAR sensor self-test")
            
            # Multiple readings to check consistency
            readings = [self.read()['distance_mm'] for _ in range(10)]
            
            calibration_data = {
                'min_distance': min(readings),
                'max_distance': max(readings),
                'average_distance': sum(readings) / len(readings)
            }
            
            self.log_info(f"Calibration complete: {calibration_data}")
            return calibration_data
        except Exception as e:
            self.log_error(f"Calibration failed: {e}")
            return None