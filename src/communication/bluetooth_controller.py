import serial
import time
import json

class BluetoothController:
    def __init__(self, port='/dev/ttyS0', baudrate=9600, timeout=1):
        """
        Initialize Bluetooth Module
        
        :param port: Serial port for Bluetooth module
        :param baudrate: Communication speed
        :param timeout: Serial communication timeout
        """
        try:
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout
            )
            self.is_connected = False
        except serial.SerialException as e:
            print(f"Bluetooth connection error: {e}")
            raise

    def send_command(self, command, wait_time=0.5):
        """
        Send AT command to Bluetooth module
        
        :param command: AT command to send
        :param wait_time: Time to wait for response
        :return: Module response
        """
        try:
            # Ensure command ends with \r\n
            full_command = command.strip() + '\r\n'
            self.serial_conn.write(full_command.encode())
            time.sleep(wait_time)
            
            response = self.serial_conn.readline().decode().strip()
            return response
        except Exception as e:
            print(f"Command sending error: {e}")
            return None

    def configure_module(self):
        """
        Configure Bluetooth module with basic settings
        """
        configurations = [
            'AT',           # Test communication
            'AT+ROLE0',     # Set as slave mode
            'AT+RESET',     # Reset module
            'AT+NAME=RoboVehicle'  # Set device name
        ]
        
        for config in configurations:
            response = self.send_command(config)
            print(f"Config {config}: {response}")

    def send_data(self, data):
        """
        Send data via Bluetooth
        
        :param data: Data to send (dict or string)
        """
        try:
            if isinstance(data, dict):
                data = json.dumps(data)
            
            self.serial_conn.write(data.encode())
        except Exception as e:
            print(f"Data sending error: {e}")

    def receive_data(self, buffer_size=1024):
        """
        Receive data from Bluetooth
        
        :param buffer_size: Maximum data size to read
        :return: Received data
        """
        try:
            if self.serial_conn.in_waiting:
                data = self.serial_conn.read(buffer_size).decode().strip()
                return data
            return None
        except Exception as e:
            print(f"Data receiving error: {e}")
            return None

    def scan_devices(self):
        """
        Scan for nearby Bluetooth devices
        
        :return: List of discovered devices
        """
        try:
            devices = []
            # Send scan command
            self.send_command('AT+SCAN')
            
            # Wait and collect device information
            time.sleep(5)  # Scan duration
            while self.serial_conn.in_waiting:
                device = self.serial_conn.readline().decode().strip()
                if device:
                    devices.append(device)
            
            return devices
        except Exception as e:
            print(f"Device scanning error: {e}")
            return []

    def close(self):
        """
        Close Bluetooth connection
        """
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

def main():
    """
    Example usage of Bluetooth Controller
    """
    try:
        bt = BluetoothController()
        
        # Configure module
        bt.configure_module()
        
        # Scan for devices
        print("Scanning for devices...")
        devices = bt.scan_devices()
        print("Discovered Devices:", devices)
        
        # Send sample data
        sample_data = {
            'command': 'move',
            'speed': 50,
            'direction': 'forward'
        }
        bt.send_data(sample_data)
        
        # Receive data
        while True:
            received = bt.receive_data()
            if received:
                print("Received:", received)
            time.sleep(1)
    
    except Exception as e:
        print(f"Bluetooth controller error: {e}")
    finally:
        bt.close()

if __name__ == "__main__":
    main()