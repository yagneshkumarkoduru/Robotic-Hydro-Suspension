"""Tests for Bluetooth controller with mocked serial hardware."""

import sys
from pathlib import Path

import pytest

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.communication.bluetooth_controller import BluetoothController


class TestBluetoothController:
    """Tests for BluetoothController class."""

    def test_initialization(self):
        """Test Bluetooth controller initializes correctly."""
        bt = BluetoothController(port="/dev/ttyS0", baudrate=9600)
        assert bt.serial_conn is not None
        assert bt.is_connected is False

    def test_send_command(self):
        """Test sending AT command."""
        bt = BluetoothController()
        response = bt.send_command("AT", wait_time=0)
        assert response is not None

    def test_send_data(self):
        """Test sending data."""
        bt = BluetoothController()
        result = bt.send_data({"test": "data"})
        # Should not raise

    def test_receive_data(self):
        """Test receiving data."""
        bt = BluetoothController()
        data = bt.receive_data()
        # Returns None or parsed data
        assert data is None or isinstance(data, (dict, str))


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
