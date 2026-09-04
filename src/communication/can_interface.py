import can
import threading
import time
import json
import logging

class CANInterface:
    def __init__(self, channel='can0', bitrate=500000):
        """
        Initialize CAN Bus Interface
        
        :param channel: CAN bus channel
        :param bitrate: Communication speed
        """
        try:
            # Configure logging
            logging.basicConfig(level=logging.INFO)
            self.logger = logging.getLogger('CAN_Interface')
            
            # Create CAN bus interface
            self.bus = can.interface.Bus(channel=channel, bustype='socketcan', bitrate=bitrate)
            
            # Message queues and threads
            self.receive_queue = []
            self._stop_event = threading.Event()
            self.receive_thread = None
        except Exception as e:
            self.logger.error(f"CAN bus initialization error: {e}")
            raise

    def send_message(self, arbitration_id, data):
        """
        Send message on CAN bus
        
        :param arbitration_id: CAN message ID
        :param data: Message data (list of bytes or dictionary)
        """
        try:
            # Convert dictionary to bytes if needed
            if isinstance(data, dict):
                data = self._dict_to_bytes(data)
            
            # Create and send CAN message
            message = can.Message(arbitration_id=arbitration_id, data=data)
            self.bus.send(message)
            self.logger.info(f"Sent CAN message: {message}")
        except Exception as e:
            self.logger.error(f"CAN message sending error: {e}")

    def _dict_to_bytes(self, data):
        """
        Convert dictionary to byte representation
        
        :param data: Dictionary to convert
        :return: Byte representation
        """
        json_data = json.dumps(data).encode()
        return list(json_data)[:8]  # Limit to 8 bytes

    def start_listening(self, callback=None, filter_ids=None):
        """
        Start listening to CAN bus messages
        
        :param callback: Optional callback function
        :param filter_ids: Optional list of arbitration IDs to filter
        """
        def _listener_thread():
            # Set up optional filters
            if filter_ids:
                bus_filters = [{"can_id": id, "can_mask": 0x7FF, "extended": False} for id in filter_ids]
                self.bus.set_filters(bus_filters)

            while not self._stop_event.is_set():
                try:
                    message = self.bus.recv(timeout=1.0)
                    if message:
                        # Process message
                        processed_msg = self._process_message(message)
                        
                        # Store in queue
                        self.receive_queue.append(processed_msg)
                        
                        # Call callback if provided
                        if callback:
                            callback(processed_msg)
                except Exception as e:
                    self.logger.error(f"CAN listening error: {e}")

        # Start listener thread
        self.receive_thread = threading.Thread(target=_listener_thread, daemon=True)
        self.receive_thread.start()

    def _process_message(self, message):
        """
        Process received CAN message
        
        :param message: Received CAN message
        :return: Processed message dictionary
        """
        try:
            # Try to decode bytes to JSON
            decoded_data = bytes(message.data).decode('utf-8').rstrip('\x00')
            json_data = json.loads(decoded_data) if decoded_data else {}
        except (UnicodeDecodeError, json.JSONDecodeError):
            # Fallback to raw data if JSON decoding fails
            json_data = {'raw_data': message.data}

        return {
            'arbitration_id': message.arbitration_id,
            'timestamp': message.timestamp,
            'data': json_data
        }

    def get_messages(self, clear=True):
        """
        Retrieve received messages
        
        :param clear: Clear queue after retrieval
        :return: List of received messages
        """
        messages = self.receive_queue.copy()
        if clear:
            self.receive_queue.clear()
        return messages

    def diagnostic_check(self):
        """
        Perform CAN bus diagnostic
        
        :return: Diagnostic results
        """
        try:
            diagnostic_msg = {
                'type': 'diagnostic',
                'components': ['bus_health', 'transmission_rate']
            }
            self.send_message(arbitration_id=0x700, data=diagnostic_msg)
            return True
        except Exception as e:
            self.logger.error(f"CAN diagnostic error: {e}")
            return False

    def close(self):
        """
        Close CAN bus interface
        """
        self._stop_event.set()
        if self.receive_thread:
            self.receive_thread.join()
        
        try:
            self.bus.shutdown()
        except Exception as e:
            self.logger.error(f"CAN bus shutdown error: {e}")

def message_callback(message):
    """
    Example callback for processing CAN messages
    
    :param message: Received CAN message
    """
    print("Received CAN Message:")
    print(f"ID: {message['arbitration_id']}")
    print(f"Timestamp: {message['timestamp']}")
    print(f"Data: {message['data']}")
    print("---")

def main():
    """
    Example usage of CAN Interface
    """
    try:
        # Initialize CAN interface
        can_interface = CANInterface()
        
        # Start listening with callback
        can_interface.start_listening(callback=message_callback)
        
        # Send diagnostic message
        can_interface.diagnostic_check()
        
        # Send sample messages
        vehicle_status = {
            'speed': 50,
            'battery_level': 85,
            'mode': 'autonomous'
        }
        can_interface.send_message(arbitration_id=0x123, data=vehicle_status)
        
        # Keep main thread running
        time.sleep(10)
    
    except Exception as e:
        print(f"CAN interface error: {e}")
    finally:
        can_interface.close()

if __name__ == "__main__":
    main()