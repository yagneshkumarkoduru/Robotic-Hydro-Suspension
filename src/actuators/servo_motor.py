# import RPi.GPIO as GPIO
import gpiozero as GPIO
import time

class ServoMotor:
    def __init__(self, pin, min_pulse=0.5, max_pulse=2.5, frequency=50):
        """
        Initialize Servo Motor
        
        :param pin: GPIO pin number
        :param min_pulse: Minimum pulse width (ms)
        :param max_pulse: Maximum pulse width (ms)
        :param frequency: PWM frequency
        """
        self.pin = pin
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        
        # Create PWM instance
        self.pwm = GPIO.PWM(self.pin, frequency)
        self.pwm.start(0)

    def set_angle(self, angle):
        """
        Set servo motor to a specific angle
        
        :param angle: Desired angle (0-180 degrees)
        """
        if angle < 0 or angle > 180:
            raise ValueError("Angle must be between 0 and 180 degrees")
        
        # Convert angle to duty cycle
        duty = self.min_pulse + (angle / 180) * (self.max_pulse - self.min_pulse)
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.3)  # Allow time for servo to reach position

    def sweep(self, start=0, end=180, step=10, delay=0.1):
        """
        Sweep servo through a range of angles
        
        :param start: Starting angle
        :param end: Ending angle
        :param step: Angle increment
        :param delay: Delay between angle changes
        """
        for angle in range(start, end + 1, step):
            self.set_angle(angle)
            time.sleep(delay)
        
        for angle in range(end, start - 1, -step):
            self.set_angle(angle)
            time.sleep(delay)

    def cleanup(self):
        """
        Cleanup GPIO resources
        """
        self.pwm.stop()
        GPIO.cleanup(self.pin)

def main():
    """
    Example usage of Servo Motor
    """
    try:
        servo = ServoMotor(pin=18)  # Example GPIO pin
        
        # Demonstrate servo functionality
        servo.set_angle(0)   # Move to 0 degrees
        time.sleep(1)
        servo.set_angle(90)  # Move to 90 degrees
        time.sleep(1)
        servo.set_angle(180) # Move to 180 degrees
        
        # Sweep demonstration
        servo.sweep()
    
    except Exception as e:
        print(f"Error: {e}")
    finally:
        servo.cleanup()

if __name__ == "__main__":
    main()