"""Vehicle controller logic tests (hardware dependencies stubbed).

The GPIO, serial, and I2C stacks are replaced with stubs before the module
under test is imported, so the controller logic runs on any machine; the
sensor, motor, and bluetooth objects handed to the controller are mocks.
The controller's real API is drive(speed, direction), stop(), and
emergency_stop().
"""

import sys
from contextlib import contextmanager
from unittest.mock import MagicMock, patch

for module_name in ("gpiozero", "serial", "can", "board", "busio", "adafruit_vl53l0x", "smbus2"):
    sys.modules.setdefault(module_name, MagicMock())

import src.vehicle_control as vehicle_control  # noqa: E402

CONFIG = {
    "gpio": {"dc_motor_pins": [23, 24, 25]},
    "communication": {"bluetooth": {"port": "/dev/rfcomm0", "baudrate": 9600}},
}
STABLE_IMU = {"acceleration": {"x": 0.1, "y": 0.1, "z": 0.2}}
UNSTABLE_IMU = {"acceleration": {"x": 1.2, "y": 0.0, "z": 0.0}}


@contextmanager
def patched_controller(imu_read=None, imu_side_effect=None):
    imu = MagicMock()
    imu.read.return_value = STABLE_IMU if imu_read is None else imu_read
    if imu_side_effect is not None:
        imu.read.side_effect = imu_side_effect
    motor = MagicMock()
    bluetooth = MagicMock()
    with patch.object(vehicle_control, "MPU6050", return_value=imu) as imu_cls, patch.object(
        vehicle_control, "DCMotor", return_value=motor
    ) as motor_cls, patch.object(
        vehicle_control, "BluetoothController", return_value=bluetooth
    ) as bt_cls:
        controller = vehicle_control.VehicleController(CONFIG)
        yield controller, imu, motor, bluetooth, imu_cls, motor_cls, bt_cls


def test_initialization_wires_components_and_calibrates():
    with patched_controller() as (controller, imu, motor, bluetooth, _imu_cls, motor_cls, bt_cls):
        imu.calibrate.assert_called_once()
        motor_cls.assert_called_once_with(23, 24, 25)
        bt_cls.assert_called_once_with(CONFIG["communication"]["bluetooth"])
        assert controller.motor_controller is motor
        assert controller.imu_sensor is imu


def test_drive_forward_passes_positive_polarity():
    with patched_controller() as (controller, _imu, motor, _bluetooth, *_classes):
        controller.drive(speed=0.5, direction="forward")
        motor.set_speed.assert_called_once_with(0.5, 1)


def test_drive_reverse_passes_negative_polarity():
    with patched_controller() as (controller, _imu, motor, _bluetooth, *_classes):
        controller.drive(speed=0.3, direction="reverse")
        motor.set_speed.assert_called_once_with(0.3, -1)


def test_unstable_attitude_stops_instead_of_driving():
    with patched_controller(imu_read=UNSTABLE_IMU) as (controller, _imu, motor, _bluetooth, *_classes):
        controller.drive(speed=0.5, direction="forward")
        motor.set_speed.assert_not_called()
        motor.stop.assert_called_once()


def test_sensor_failure_stops_the_motors():
    with patched_controller(imu_side_effect=RuntimeError("I2C bus error")) as (
        controller,
        _imu,
        motor,
        _bluetooth,
        *_classes,
    ):
        controller.drive(speed=0.5, direction="forward")
        motor.set_speed.assert_not_called()
        motor.stop.assert_called_once()


def test_is_stable_threshold_behaviour():
    with patched_controller() as (controller, *_rest):
        assert controller._is_stable(STABLE_IMU) is True
        assert controller._is_stable({"acceleration": {"x": 0.6, "y": 0.0, "z": 0.0}}) is False
        assert controller._is_stable({}) is False


def test_emergency_stop_calls_motor_stop():
    with patched_controller() as (controller, _imu, motor, _bluetooth, *_classes):
        controller.emergency_stop()
        motor.stop.assert_called_once()
