"""Tests for motor controllers with mocked GPIO hardware."""

import sys
import pytest
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.actuators.servo_motor import ServoMotor
from src.actuators.motor_controller import DCMotor


class TestServoMotor:
    """Tests for ServoMotor class."""

    def test_servo_initialization(self):
        """Test servo motor initializes correctly."""
        servo = ServoMotor(pin=18)
        assert servo.pin == 18
        assert servo.pwm is not None
        servo.cleanup()

    def test_servo_set_angle_boundaries(self):
        """Test servo sets boundary angles."""
        servo = ServoMotor(pin=18)
        servo.set_angle(0)
        servo.set_angle(90)
        servo.set_angle(180)
        servo.cleanup()

    def test_servo_set_angle_invalid_low(self):
        """Test servo rejects angle below 0."""
        servo = ServoMotor(pin=18)
        with pytest.raises(ValueError):
            servo.set_angle(-10)
        servo.cleanup()

    def test_servo_set_angle_invalid_high(self):
        """Test servo rejects angle above 180."""
        servo = ServoMotor(pin=18)
        with pytest.raises(ValueError):
            servo.set_angle(200)
        servo.cleanup()

    def test_servo_sweep(self):
        """Test servo sweep function."""
        servo = ServoMotor(pin=18)
        servo.sweep(start=0, end=60, step=30, delay=0)
        servo.cleanup()


class TestDCMotor:
    """Tests for DCMotor class."""

    def test_dc_motor_initialization(self):
        """Test DC motor initializes correctly."""
        motor = DCMotor(pwm_pin=23, dir_pin1=24, dir_pin2=25)
        assert motor.pwm_pin == 23
        assert motor.dir_pin1 == 24
        assert motor.dir_pin2 == 25
        motor.cleanup()

    def test_dc_motor_set_speed_forward(self):
        """Test DC motor forward speed."""
        motor = DCMotor(pwm_pin=23, dir_pin1=24, dir_pin2=25)
        motor.set_speed(50, 1)
        assert motor.pwm.duty_cycle == 50
        motor.cleanup()

    def test_dc_motor_set_speed_reverse(self):
        """Test DC motor reverse speed."""
        motor = DCMotor(pwm_pin=23, dir_pin1=24, dir_pin2=25)
        motor.set_speed(75, -1)
        assert motor.pwm.duty_cycle == 75
        motor.cleanup()

    def test_dc_motor_set_speed_invalid_low(self):
        """Test DC motor rejects negative speed."""
        motor = DCMotor(pwm_pin=23, dir_pin1=24, dir_pin2=25)
        with pytest.raises(ValueError):
            motor.set_speed(-10, 1)
        motor.cleanup()

    def test_dc_motor_set_speed_invalid_high(self):
        """Test DC motor rejects speed over 100."""
        motor = DCMotor(pwm_pin=23, dir_pin1=24, dir_pin2=25)
        with pytest.raises(ValueError):
            motor.set_speed(110, 1)
        motor.cleanup()

    def test_dc_motor_stop(self):
        """Test DC motor stop function."""
        motor = DCMotor(pwm_pin=23, dir_pin1=24, dir_pin2=25)
        motor.set_speed(50, 1)
        motor.stop()
        assert motor.pwm.duty_cycle == 0
        motor.cleanup()

    def test_dc_motor_ramp_up(self):
        """Test DC motor ramp up function."""
        motor = DCMotor(pwm_pin=23, dir_pin1=24, dir_pin2=25)
        motor.ramp_up(max_speed=50, step=25, delay=0)
        assert motor.pwm.duty_cycle == 50
        motor.cleanup()

    def test_dc_motor_ramp_down(self):
        """Test DC motor ramp down function."""
        motor = DCMotor(pwm_pin=23, dir_pin1=24, dir_pin2=25)
        motor.ramp_down(max_speed=50, step=25, delay=0)
        assert motor.pwm.duty_cycle == 0
        motor.cleanup()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
