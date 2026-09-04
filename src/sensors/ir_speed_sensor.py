# import RPi.GPIO as GPIO
import gpiozero as GPIO
import time
from . import BaseSensor

class IRSpeedSensor(BaseSensor):
    def __init__(self, pin, wheel_circumference=0.5):
        """
        Initialize IR Speed Sensor
        
        Args:
            pin (int): GPIO pin number
            wheel_circumference (float): Wheel circumference in meters
        """
        super().__init__("IR Speed Sensor")
        
        self.pin = pin
        self.wheel_circumference = wheel_circumference
        
        # Setup GPIO for interrupt
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Tracking variables
        self.pulse_count = 0
        self.last_time = time.time()
    
    def _pulse_callback(self, channel):
        """
        Interrupt callback for pulse counting
        
        Args:
            channel (int): GPIO channel that triggered the interrupt
        """
        current_time = time.time()
        self.pulse_count += 1
    
    def start_monitoring(self):
        """
        Start monitoring speed using GPIO interrupts
        """
        try:
            GPIO.add_event_detect(
                self.pin, 
                GPIO.FALLING, 
                callback=self._pulse_callback, 
                bouncetime=50
            )
            self.log_info("Speed sensor monitoring started")
        except Exception as e:
            self.log_error(f"Error starting speed monitoring: {e}")
    
    def read(self):
        """
        Calculate speed based on pulse count
        
        Returns:
            dict: Speed and distance information
        """
        try:
            current_time = time.time()
            time_elapsed = current_time - self.last_time
            
            # Calculate rotations
            rotations = self.pulse_count
            
            # Calculate speed
            speed = (rotations * self.wheel_circumference) / time_elapsed
            
            # Prepare result
            result = {
                'speed_mps': speed,  # meters per second
                'speed_kph': speed * 3.6,  # kilometers per hour
                'rotations': rotations
            }
            
            # Reset for next measurement
            self.pulse_count = 0
            self.last_time = current_time
            
            return result
        except Exception as e:
            self.log_error(f"Speed calculation failed: {e}")
            return None
    
    def stop_monitoring(self):
        """
        Stop GPIO interrupt monitoring
        """
        try:
            GPIO.remove_event_detect(self.pin)
            self.log_info("Speed sensor monitoring stopped")
        except Exception as e:
            self.log_error(f"Error stopping speed monitoring: {e}")
    
    def calibrate(self):
        """
        Calibrate sensor by taking multiple readings
        
        Returns:
            dict: Calibration data
        """
        try:
            # Take multiple speed readings
            readings = []
            for _ in range(5):
                reading = self.read()
                if reading:
                    readings.append(reading['speed_mps'])
                time.sleep(0.5)
            
            calibration_data = {
                'avg_speed': sum(readings) / len(readings) if readings else 0,
                'min_speed': min(readings) if readings else 0,
                'max_speed': max(readings) if readings else 0
            }
            
            self.log_info(f"Sensor calibration complete: {calibration_data}")
            return calibration_data
        except Exception as e:
            self.log_error(f"Calibration failed: {e}")
            return None