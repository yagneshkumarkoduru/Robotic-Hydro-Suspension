"""Pytest configuration for Robotic-Hydro-Suspension tests.

Mocks hardware-dependent modules for non-RPi environments.
"""

import sys
from unittest.mock import MagicMock


# --- Mock gpiozero (RPi.GPIO style) ---

class MockPWM:
    """Mock PWM class."""
    def __init__(self, pin, frequency):
        self.pin = pin
        self.frequency = frequency
        self.duty_cycle = 0

    def start(self, duty_cycle):
        self.duty_cycle = duty_cycle

    def ChangeDutyCycle(self, duty_cycle):
        self.duty_cycle = duty_cycle

    def stop(self):
        pass


class MockGPIO:
    """Mock GPIO module mimicking RPi.GPIO interface."""
    BCM = "BCM"
    OUT = "OUT"
    HIGH = 1
    LOW = 0
    _pins = {}

    @classmethod
    def setmode(cls, mode):
        pass

    @classmethod
    def setup(cls, pin, direction):
        cls._pins[pin] = {"direction": direction, "value": cls.LOW}

    @classmethod
    def output(cls, pin, value):
        if pin in cls._pins:
            cls._pins[pin]["value"] = value

    @classmethod
    def PWM(cls, pin, frequency):
        return MockPWM(pin, frequency)

    @classmethod
    def cleanup(cls, pin=None):
        if pin is None:
            cls._pins.clear()
        elif pin in cls._pins:
            del cls._pins[pin]


# --- Mock smbus2 (I2C) ---

class MockSMBus:
    """Mock SMBus for I2C sensor testing."""
    def __init__(self, bus=1):
        self.bus = bus
        self._registers = {}

    def write_byte_data(self, addr, reg, value):
        self._registers[(addr, reg)] = value

    def read_byte_data(self, addr, reg):
        # Return fake sensor data
        return self._registers.get((addr, reg), 0x00)


class MockSMBus2:
    """Mock smbus2 module."""
    SMBus = MockSMBus


# --- Mock serial (pyserial) ---

class MockSerial:
    """Mock serial port for Bluetooth testing."""
    def __init__(self, port=None, baudrate=9600, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._buffer = b""
        self._response = b"OK\r\n"

    def write(self, data):
        self._buffer = data
        return len(data)

    def readline(self):
        return self._response

    def read(self, size=1):
        return self._response[:size]

    def close(self):
        pass

    @property
    def in_waiting(self):
        return len(self._response)


class MockSerialModule:
    """Mock serial module."""
    Serial = MockSerial
    SerialException = Exception


# --- Mock board (CircuitPython) ---

class MockBoard:
    """Mock board module for CircuitPython I2C."""
    SCL = "SCL"
    SDA = "SDA"


# --- Mock busio (CircuitPython I2C bus) ---

class MockI2C:
    """Mock I2C bus."""
    def __init__(self, scl, sda):
        self.scl = scl
        self.sda = sda

    def try_lock(self):
        return True

    def unlock(self):
        pass

    def writeto(self, addr, buf):
        pass

    def readfrom_into(self, addr, buf):
        pass


class MockBusio:
    """Mock busio module."""
    I2C = MockI2C


# --- Mock adafruit_vl53l0x ---

class MockVL53L0X:
    """Mock VL53L0X sensor."""
    def __init__(self, i2c):
        self.i2c = i2c

    @property
    def range(self):
        return 100  # mm


class MockAdafruitVL53L0X:
    """Mock adafruit_vl53l0x module."""
    VL53L0X = MockVL53L0X


# --- Mock python-can ---

class MockCANMessage:
    """Mock CAN message."""
    def __init__(self, arbitration_id=0, data=None, is_extended_id=False):
        self.arbitration_id = arbitration_id
        self.data = data or []
        self.is_extended_id = is_extended_id


class MockCANBus:
    """Mock CAN bus."""
    def __init__(self, channel=None, bustype=None, bitrate=None):
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate

    def send(self, msg):
        pass

    def recv(self, timeout=None):
        return None

    def shutdown(self):
        pass


class MockCANInterface:
    """Mock can.interface module."""
    Bus = MockCANBus


class MockCAN:
    """Mock can module."""
    Message = MockCANMessage
    Bus = MockCANBus
    interface = MockCANInterface


# Patch modules before any test imports
sys.modules["gpiozero"] = MockGPIO
sys.modules["smbus2"] = MockSMBus2
sys.modules["serial"] = MockSerialModule
sys.modules["board"] = MockBoard
sys.modules["busio"] = MockBusio
sys.modules["adafruit_vl53l0x"] = MockAdafruitVL53L0X
sys.modules["can"] = MockCAN
