# import RPi.GPIO as GPIO
import gpiozero as GPIO
import time
from . import BaseSensor

class MicrowaveRadarSensor(BaseSensor):
    def __init__(self, pin, sensitivity=1.0):
        """
        Initialize RCWL-0516 Microwave Radar Sensor
        
        Args:
            pin (int): GPIO pin number
            sensitivity (float): Sensor sensitivity adjustment
        """
        super().__init__("RCWL-0516 Microwave Radar")
        
        self.pin = pin
        self.sensitivity = sensitivity
        
        # Setup GPIO
        GPIO.setup(pin, GPIO.IN)
        
        # Tracking variables
        self.motion_events = []
    
    def read(self):
        """
        Read motion detection state
        
        Returns:
            dict: Motion detection information
        """
        try:
            # Check motion state
            is_motion_detected = GPIO.input(self.pin) == GPIO.HIGH
            
            current_time = time.time()
            
            if is_motion_detected:
                self.motion_events.append(current_time)
            
            # Clean up old events (last 60 seconds)
            self.motion_events = [
                event for event in self.motion_events 
                if current_time - event < 60
            ]
            
            result = {
                'motion_detected': is_motion_detected,
                'motion_frequency': len(self.motion_events),
                'timestamp': current_time
            }
            
            if is_motion_detected:
                self.log_info("Motion detected")
            
            return result
        except Exception as e:
            self.log_error(f"Motion sensing failed: {e}")
            return None
    
    def calibrate(self):
        """
        Calibrate sensor sensitivity
        
        Returns:
            dict: Calibration data
        """
        try:
            # Take multiple readings to establish baseline
            motion_readings = []
            
            for _ in range(50):  # 5 seconds of sampling
                reading = self.read()
                motion_readings.append(reading['motion_detected'])
                time.sleep(0.1)
            
            # Calculate motion detection characteristics
            detection_rate = sum(motion_readings) / len(motion_readings)
            
            calibration_data = {
                'detection_rate': detection_rate,
                'false_positive_rate': 1 - detection_rate,
                'recommended_sensitivity': self.sensitivity
            }
            
            self.log_info(f"Microwave radar calibration: {calibration_data}")
            return calibration_data
        except Exception as e:
            self.log_error(f"Calibration failed: {e}")
            return None