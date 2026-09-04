# import RPi.GPIO as GPIO
import gpiozero as GPIO
import time

class DCMotor:
    def __init__(self, pwm_pin, dir_pin1, dir_pin2):
        """
        Initialize DC Motor
        
        :param pwm_pin: PWM control pin
        :param dir_pin1: First direction control pin
        :param dir_pin2: Second direction control pin
        """
        self.pwm_pin = pwm_pin
        self.dir_pin1 = dir_pin1
        self.dir_pin2 = dir_pin2
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin1, GPIO.OUT)
        GPIO.setup(self.dir_pin2, GPIO.OUT)
        
        # Create PWM instance
        self.pwm = GPIO.PWM(self.pwm_pin, 1000)  # 1000 Hz PWM frequency
        self.pwm.start(0)

    def set_speed(self, speed, direction=1):
        """
        Set motor speed and direction
        
        :param speed: Motor speed (0-100%)
        :param direction: Motor direction (1: forward, -1: reverse)
        """
        if speed < 0 or speed > 100:
            raise ValueError("Speed must be between 0 and 100")
        
        # Set motor direction
        if direction == 1:
            GPIO.output(self.dir_pin1, GPIO.HIGH)
            GPIO.output(self.dir_pin2, GPIO.LOW)
        else:
            GPIO.output(self.dir_pin1, GPIO.LOW)
            GPIO.output(self.dir_pin2, GPIO.HIGH)
        
        # Set motor speed
        self.pwm.ChangeDutyCycle(speed)

    def stop(self):
        """
        Stop the motor
        """
        self.pwm.ChangeDutyCycle(0)
        GPIO.output(self.dir_pin1, GPIO.LOW)
        GPIO.output(self.dir_pin2, GPIO.LOW)

    def brake(self, braking_time=0.5):
        """
        Apply brake to motor
        
        :param braking_time: Duration of braking
        """
        GPIO.output(self.dir_pin1, GPIO.HIGH)
        GPIO.output(self.dir_pin2, GPIO.HIGH)
        time.sleep(braking_time)
        self.stop()

    def ramp_up(self, max_speed=100, step=10, delay=0.1):
        """
        Gradually increase motor speed
        
        :param max_speed: Maximum speed to reach
        :param step: Speed increment
        :param delay: Delay between speed changes
        """
        for speed in range(0, max_speed + 1, step):
            self.set_speed(speed)
            time.sleep(delay)

    def ramp_down(self, max_speed=100, step=10, delay=0.1):
        """
        Gradually decrease motor speed
        
        :param max_speed: Starting speed
        :param step: Speed decrement
        :param delay: Delay between speed changes
        """
        for speed in range(max_speed, -1, -step):
            self.set_speed(speed)
            time.sleep(delay)
        
        self.stop()

    def cleanup(self):
        """
        Cleanup GPIO resources
        """
        self.pwm.stop()
        GPIO.cleanup(self.pwm_pin)
        GPIO.cleanup(self.dir_pin1)
        GPIO.cleanup(self.dir_pin2)

def main():
    """
    Example usage of DC Motor
    """
    try:
        # Example GPIO pins
        motor = DCMotor(pwm_pin=18, dir_pin1=23, dir_pin2=24)
        
        # Demonstrate motor functionality
        motor.set_speed(50)  # Run at 50% speed forward
        time.sleep(2)
        
        motor.ramp_up(max_speed=80)  # Ramp up to 80% speed
        time.sleep(2)
        
        motor.set_speed(50, direction=-1)  # Run at 50% speed reverse
        time.sleep(2)
        
        motor.brake()  # Apply brake
    
    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor.cleanup()

if __name__ == "__main__":
    main()