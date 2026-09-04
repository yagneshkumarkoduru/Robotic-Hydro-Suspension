"""
Communication Package Initialization

This module provides a centralized import and initialization 
for all communication interfaces in the robotic vehicle project.
"""

from .bluetooth_controller import BluetoothController
from .arduino_interface import ArduinoInterface
from .can_interface import CANInterface

# Define which communication modules will be exposed
__all__ = [
    'BluetoothController', 
    'ArduinoInterface', 
    'CANInterface'
]

class CommunicationManager:
    """
    Centralized manager for handling multiple communication interfaces.
    """
    def __init__(self):
        """
        Initialize communication interfaces.
        """
        self.bluetooth = BluetoothController()
        self.arduino = ArduinoInterface()
        self.can_bus = CANInterface()
        
        # Store all interfaces in a dictionary for easy access
        self.interfaces = {
            'bluetooth': self.bluetooth,
            'arduino': self.arduino,
            'can': self.can_bus
        }
    
    def initialize_all(self):
        """
        Initialize all communication interfaces.
        
        Returns:
            dict: Status of initialization for each interface
        """
        initialization_status = {}
        
        for name, interface in self.interfaces.items():
            try:
                interface.connect()
                initialization_status[name] = True
            except Exception as e:
                print(f"Error initializing {name} interface: {e}")
                initialization_status[name] = False
        
        return initialization_status
    
    def broadcast_message(self, message, interfaces=None):
        """
        Broadcast a message across specified or all interfaces.
        
        Args:
            message (str): Message to broadcast
            interfaces (list, optional): List of interfaces to use. 
                                        If None, uses all interfaces.
        """
        if interfaces is None:
            interfaces = list(self.interfaces.keys())
        
        for name in interfaces:
            if name in self.interfaces:
                try:
                    self.interfaces[name].send(message)
                except Exception as e:
                    print(f"Error broadcasting on {name} interface: {e}")
    
    def close_all_connections(self):
        """
        Close all communication interface connections.
        """
        for interface in self.interfaces.values():
            try:
                interface.disconnect()
            except Exception as e:
                print(f"Error closing interface: {e}")

# Create a singleton communication manager
communication_manager = CommunicationManager()