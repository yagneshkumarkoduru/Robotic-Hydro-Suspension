import unittest
from unittest.mock import Mock, patch
import sys
import os

# Add the project root to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.vehicle_control import VehicleControl

class TestVehicleControl(unittest.TestCase):
    def setUp(self):
        """
        Set up test fixtures before each test method.
        Initialize VehicleControl with mock components.
        """
        # Mock sensor and actuator dependencies
        self.mock_gps = Mock()
        self.mock_imu = Mock()
        self.mock_lidar = Mock()
        self.mock_dc_motor = Mock()
        self.mock_servo = Mock()

        # Create VehicleControl instance with mocked components
        self.vehicle = VehicleControl(
            gps=self.mock_gps,
            imu=self.mock_imu,
            lidar=self.mock_lidar,
            drive_motor=self.mock_dc_motor,
            steering_servo=self.mock_servo
        )

    def test_initialize_vehicle(self):
        """
        Test vehicle initialization process.
        Verify that all components are properly set up.
        """
        # Check that initialization doesn't raise any exceptions
        try:
            self.vehicle.initialize()
        except Exception as e:
            self.fail(f"Vehicle initialization failed: {e}")

        # Verify mock components were initialized
        self.mock_gps.initialize.assert_called_once()
        self.mock_imu.calibrate.assert_called_once()
        self.mock_lidar.start.assert_called_once()

    def test_basic_movement(self):
        """
        Test basic vehicle movement functionality.
        Check forward, reverse, and stop commands.
        """
        # Test forward movement
        self.vehicle.move_forward(speed=0.5)
        self.mock_dc_motor.set_speed.assert_called_with(0.5)
        self.mock_dc_motor.forward.assert_called_once()

        # Test reverse movement
        self.vehicle.move_reverse(speed=0.3)
        self.mock_dc_motor.set_speed.assert_called_with(0.3)
        self.mock_dc_motor.reverse.assert_called_once()

        # Test stopping
        self.vehicle.stop()
        self.mock_dc_motor.stop.assert_called_once()

    def test_steering(self):
        """
        Test steering functionality.
        Verify servo angle adjustments.
        """
        # Test left turn
        self.vehicle.turn_left(angle=45)
        self.mock_servo.set_angle.assert_called_with(45)

        # Test right turn
        self.vehicle.turn_right(angle=45)
        self.mock_servo.set_angle.assert_called_with(-45)

        # Test center reset
        self.vehicle.center_steering()
        self.mock_servo.set_angle.assert_called_with(0)

    def test_obstacle_detection(self):
        """
        Test obstacle detection and avoidance logic.
        """
        # Simulate obstacle detection
        self.mock_lidar.get_distance.return_value = 0.5  # 50 cm obstacle
        
        # Check if obstacle detection triggers appropriate response
        self.vehicle.check_obstacles()
        
        # Verify stop method called when obstacle is too close
        self.mock_dc_motor.stop.assert_called_once()

    def test_emergency_stop(self):
        """
        Test emergency stop functionality.
        """
        self.vehicle.emergency_stop()
        
        # Verify all critical systems are halted
        self.mock_dc_motor.stop.assert_called_once()
        self.mock_servo.center_steering.assert_called_once()

if __name__ == '__main__':
    unittest.main()