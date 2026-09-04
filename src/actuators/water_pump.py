# import RPi.GPIO as GPIO
import gpiozero as GPIO
import time

class WaterPump:
    def __init__(self, pin, pwm_frequency=100):
        """
        Initialize Water Pump
        
        :param pin: GPIO pin number for pump control
        :param pwm_frequency: PWM frequency
        """
        self.pin = pin
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        
        # Create PWM instance
        self.pwm = GPIO.PWM(self.pin, pwm_frequency)
        self.pwm.start(0)

    def turn_on(self, duration=None, intensity=100):
        """
        Turn on water pump
        
        :param duration: Optional duration to run pump (seconds)
        :param intensity: Pump speed (0-100%)
        """
        if intensity < 0 or intensity > 100:
            raise ValueError("Intensity must be between 0 and 100")
        
        self.pwm.ChangeDutyCycle(intensity)
        
        if duration:
            time.sleep(duration)
            self.turn_off()

    def turn_off(self):
        """
        Turn off water pump
        """
        self.pwm.ChangeDutyCycle(0)

    def pulse(self, on_time=0.5, off_time=0.5, cycles=3):
        """
        Pulse water pump on and off
        
        :param on_time: Duration pump is on (seconds)
        :param off_time: Duration pump is off (seconds)
        :param cycles: Number of on/off cycles
        """
        for _ in range(cycles):
            self.turn_on(duration=on_time)
            time.sleep(off_time)

    def cleanup(self):
        """
        Cleanup GPIO resources
        """
        self.pwm.stop()
        GPIO.cleanup(self.pin)

def main():
    """
    Example usage of Water Pump
    """
    try:
        pump = WaterPump(pin=23)  # Example GPIO pin
        
        # Demonstrate pump functionality
        pump.turn_on(duration=2)      # Run at full speed for 2 seconds
        time.sleep(1)
        
        pump.turn_on(intensity=50, duration=3)  # Run at half speed for 3 seconds
        time.sleep(1)
        
        pump.pulse()  # Pulse pump 3 times
    
    except Exception as e:
        print(f"Error: {e}")
    finally:
        pump.cleanup()

if __name__ == "__main__":
    main()