# import RPi.GPIO as GPIO
import gpiozero as GPIO
import time
from . import BaseSensor

class ProximitySensor(BaseSensor):
    def __init__(self, pin, detection_range=10):
        """
        Initialize SN04-N Proximity Sensor
        
        Args:
            pin (int): GPIO pin number
            detection_range (float): Maximum detection range in cm
        """
        super().__init__("SN04-N Proximity Sensor")
        
        self.pin = pin
        self.detection_range = detection_range
        
        # Setup GPIO
        GPIO.setup(pin, GPIO.IN)
    
    def read(self):
        """
        Read proximity sensor state
        
        Returns:
            dict: Proximity detection information
        """
        try:
            # Read digital state
            is_detected = GPIO.input(self.pin) == GPIO.HIGH
            
            result = {
                'detected': is_detected,
                'range_cm': self.detection_range if is_detected else None,
                'timestamp': time.time()
            }
            
            if is_detected:
                self.log_info(f"Object detected within {self.detection_range} cm")
            
            return result
        except Exception as e:
            self.log_error(f"Proximity sensing failed: {e}")
            return None
    
    def calibrate(self):
        """
        Calibrate sensor by taking multiple readings
        
        Returns:
            dict: Calibration data
        """
        try:
            # Take multiple readings
            readings = []
            for _ in range(10):
                reading = self.read()
                readings.append(reading['detected'])
                time.sleep(0.1)
            
            # Calculate detection reliability
            detection_rate = sum(readings) / len(readings)
            
            calibration_data = {
                'detection_reliability': detection_rate,
                'false_positive_rate': 1 - detection_rate
            }
            
            self.log_info(f"Proximity sensor calibration: {calibration_data}")
            return calibration_data
        except Exception as e:
            self.log_error(f"Calibration failed: {e}")
            return None