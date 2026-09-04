import serial
import time
import json
import threading

class ArduinoInterface:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=1):
        """
        Initialize Arduino Communication Interface
        
        :param port: Serial port for Arduino
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
            self._stop_event = threading.Event()
            self.receive_thread = None
        except serial.SerialException as e:
            print(f"Arduino connection error: {e}")
            raise

    def send_command(self, command):
        """
        Send command to Arduino
        
        :param command: Command to send
        :return: Arduino response
        """
        try:
            # Convert command to JSON for structured communication
            json_command = json.dumps(command) + '\n'
            self.serial_conn.write(json_command.encode())
            
            # Read response
            response = self.serial_conn.readline().decode().strip()
            return json.loads(response) if response else None
        except Exception as e:
            print(f"Command sending error: {e}")
            return None

    def start_continuous_read(self, callback):
        """
        Start continuous reading of sensor data
        
        :param callback: Function to process received data
        """
        def _read_thread():
            while not self._stop_event.is_set():
                try:
                    if self.serial_conn.in_waiting:
                        data = self.serial_conn.readline().decode().strip()
                        if data:
                            try:
                                parsed_data = json.loads(data)
                                callback(parsed_data)
                            except json.JSONDecodeError:
                                print(f"Invalid JSON: {data}")
                except Exception as e:
                    print(f"Read thread error: {e}")
                time.sleep(0.1)

        # Start reading thread
        self.receive_thread = threading.Thread(target=_read_thread, daemon=True)
        self.receive_thread.start()

    def stop_continuous_read(self):
        """
        Stop continuous reading
        """
        self._stop_event.set()
        if self.receive_thread:
            self.receive_thread.join()

    def calibrate_sensors(self):
        """
        Perform sensor calibration
        
        :return: Calibration results
        """
        calibration_command = {
            'type': 'calibration',
            'sensors': ['mpu6050', 'proximity', 'speed']
        }
        return self.send_command(calibration_command)

    def system_diagnostic(self):
        """
        Run system diagnostic check
        
        :return: Diagnostic results
        """
        diagnostic_command = {
            'type': 'diagnostic',
            'check_components': [
                'power_system', 
                'communication_interfaces', 
                'sensor_health'
            ]
        }
        return self.send_command(diagnostic_command)

    def close(self):
        """
        Close Arduino connection
        """
        self.stop_continuous_read()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

def data_processor(data):
    """
    Example data processing callback
    
    :param data: Received sensor data
    """
    print("Received Sensor Data:", data)
    # Process sensor data as needed
    if 'temperature' in data:
        print(f"Current Temperature: {data['temperature']}°C")
    if 'acceleration' in data:
        print(f"Acceleration: {data['acceleration']}")

def main():
    """
    Example usage of Arduino Interface
    """
    try:
        arduino = ArduinoInterface()
        
        # Calibrate sensors
        print("Sensor Calibration:")
        calibration_result = arduino.calibrate_sensors()
        print(calibration_result)
        
        # Run system diagnostic
        print("\nSystem Diagnostic:")
        diagnostic_result = arduino.system_diagnostic()
        print(diagnostic_result)
        
        # Start continuous sensor reading
        arduino.start_continuous_read(callback=data_processor)
        
        # Keep main thread running
        while True:
            time.sleep(1)
    
    except Exception as e:
        print(f"Arduino interface error: {e}")
    finally:
        arduino.close()

if __name__ == "__main__":
    main()