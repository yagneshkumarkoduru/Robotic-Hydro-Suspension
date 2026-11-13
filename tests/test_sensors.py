"""Tests for sensors with mocked I2C hardware."""

import sys
from pathlib import Path

import pytest

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.sensors.mpu6050 import MPU6050


class TestMPU6050:
    """Tests for MPU6050 accelerometer/gyroscope sensor."""

    def test_initialization(self):
        """Test MPU6050 initializes correctly."""
        mpu = MPU6050(bus=1)
        assert mpu.bus is not None

    def test_accel_data_structure(self):
        """Test accelerometer data returns correct structure."""
        mpu = MPU6050(bus=1)
        data = mpu.get_accel_data()
        assert "x" in data
        assert "y" in data
        assert "z" in data

    def test_gyro_data_structure(self):
        """Test gyroscope data returns correct structure."""
        mpu = MPU6050(bus=1)
        data = mpu.get_gyro_data()
        assert "x" in data
        assert "y" in data
        assert "z" in data

    def test_read_raw_data(self):
        """Test raw data reading."""
        mpu = MPU6050(bus=1)
        value = mpu.read_raw_data(MPU6050.ACCEL_XOUT_H)
        assert isinstance(value, int)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
