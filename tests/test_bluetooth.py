import unittest
import sys
import os
import time

# Add project root to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.communication.bluetooth_controller import BluetoothController

class TestBluetoothController(unittest.TestCase):
    def setUp(self):
        """
        Set up Bluetooth controller for testing
        """
        try:
            self.bt = BluetoothController()
        except Exception as e:
            self.skipTest(f"Could not initialize Bluetooth: {e}")

    def test_module_initialization(self):
        """
        Test Bluetooth module initialization
        """
        self.assertIsNotNone(self.bt, "Bluetooth controller should be initialized")
        self.assertTrue(hasattr(self.bt, 'serial_conn'), "Bluetooth controller should have serial connection")

    def test_send_at_command(self):
        """
        Test sending AT commands
        """
        response = self.bt.send_command('AT')
        self.assertIsNotNone(response, "AT command should return a response")
        self.assertTrue(response.startswith('OK'), "AT command should return OK")

    def test_device_configuration(self):
        """
        Test module configuration
        """
        try:
            self.bt.configure_module()
        except Exception as e:
            self.fail(f"Module configuration failed: {e}")

    def test_data_transmission(self):
        """
        Test data sending and receiving
        """
        test_data = {
            'command': 'test_transmission',
            'timestamp': time.time()
        }
        
        try:
            # Send data
            self.bt.send_data(test_data)
            
            # Wait and check receive
            time.sleep(0.5)
            received = self.bt.receive_data()
            
            self.assertIsNotNone(received, "Should receive data after transmission")
        except Exception as e:
            self.fail(f"Data transmission test failed: {e}")

    def test_device_scanning(self):
        """
        Test device scanning functionality
        """
        try:
            devices = self.bt.scan_devices()
            self.assertIsInstance(devices, list, "Scan should return a list")
        except Exception as e:
            self.fail(f"Device scanning failed: {e}")

    def tearDown(self):
        """
        Close Bluetooth connection after tests
        """
        if hasattr(self, 'bt'):
            self.bt.close()

if __name__ == '__main__':
    unittest.main()