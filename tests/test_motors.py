import unittest
import sys
import os
import time

# Add project root to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.actuators.servo_motor import ServoMotor
from src.actuators.motor_controller import DCMotor
from src.actuators.water_pump import WaterPump

class TestMotorControls(unittest.TestCase):
    def setUp(self):
        """
        Set up motor instances for testing
        """
        try:
            # Test Servo Motor
            self.servo = ServoMotor(pin=18)
            
            # Test DC Motor
            self.dc_motor = DCMotor(pwm_pin=23, dir_pin1=24, dir_pin2=25)
            
            # Test Water Pump
            self.water_pump = WaterPump(pin=21)
        except Exception as e:
            self.skipTest(f"Motor initialization failed: {e}")

    def test_servo_angle_control(self):
        """
        Test servo motor angle control
        """
        test_angles = [0, 45, 90, 135, 180]
        
        for angle in test_angles:
            try:
                self.servo.set_angle(angle)
                time.sleep(0.5)  # Allow time for servo to move
            except Exception as e:
                self.fail(f"Failed to set servo to {angle} degrees: {e}")

    def test_servo_sweep(self):
        """
        Test servo motor sweeping functionality
        """
        try:
            self.servo.sweep(start=0, end=180, step=30, delay=0.1)
        except Exception as e:
            self.fail(f"Servo sweep failed: {e}")

    def test_dc_motor_speed_control(self):
        """
        Test DC motor speed and direction control
        """
        speed_tests = [
            (50, 1),   # 50% forward
            (75, -1),  # 75% reverse
            (100, 1),  # Full speed forward
            (0, 1)     # Stop
        ]
        
        for speed, direction in speed_tests:
            try:
                self.dc_motor.set_speed(speed, direction)
                time.sleep(1)  # Run at each setting
            except Exception as e:
                self.fail(f"Failed DC motor test with speed {speed}, direction {direction}: {e}")

    def test_dc_motor_ramp(self):
        """
        Test DC motor ramping functionality
        """
        try:
            self.dc_motor.ramp_up(max_speed=80, step=20, delay=0.2)
            self.dc_motor.ramp_down(max_speed=80, step=20, delay=0.2)
        except Exception as e:
            self.fail(f"DC motor ramping failed: {e}")

    def test_water_pump_operation(self):
        """
        Test water pump functionality
        """
        pump_tests = [
            (50, 2),   # Half intensity for 2 seconds
            (100, 1),  # Full intensity for 1 second
            (25, 3)    # Quarter intensity for 3 seconds
        ]
        
        for intensity, duration in pump_tests:
            try:
                self.water_pump.turn_on(intensity=intensity, duration=duration)
                time.sleep(0.5)  # Small pause between tests
            except Exception as e:
                self.fail(f"Water pump test failed with intensity {intensity}, duration {duration}: {e}")

    def test_water_pump_pulse(self):
        """
        Test water pump pulse functionality
        """
        try:
            self.water_pump.pulse(on_time=0.5, off_time=0.5, cycles=3)
        except Exception as e:
            self.fail(f"Water pump pulse test failed: {e}")

    def tearDown(self):
        """
        Cleanup motor resources
        """
        try:
            if hasattr(self, 'servo'):
                self.servo.cleanup()
            if hasattr(self, 'dc_motor'):
                self.dc_motor.cleanup()
            if hasattr(self, 'water_pump'):
                self.water_pump.cleanup()
        except Exception as e:
            print(f"Cleanup error: {e}")

if __name__ == '__main__':
    unittest.main()