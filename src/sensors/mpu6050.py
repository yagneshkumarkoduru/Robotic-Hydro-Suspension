import smbus2
import time
import math

class MPU6050:
    # MPU6050 device address
    DEVICE_ADDRESS = 0x68

    # Register addresses
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    # Configuration constants
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    GYRO_SCALE_MODIFIER_250DEG = 131.0

    def __init__(self, bus=1):
        """
        Initialize MPU6050 sensor
        
        :param bus: I2C bus number (default 1 for Raspberry Pi)
        """
        self.bus = smbus2.SMBus(bus)
        
        # Wake up the MPU6050 by writing 0 to power management register
        self.bus.write_byte_data(self.DEVICE_ADDRESS, self.PWR_MGMT_1, 0)

    def read_raw_data(self, register):
        """
        Read raw data from a register
        
        :param register: Starting register address
        :return: Raw sensor data
        """
        # Read high and low bytes and combine
        high = self.bus.read_byte_data(self.DEVICE_ADDRESS, register)
        low = self.bus.read_byte_data(self.DEVICE_ADDRESS, register + 1)
        
        # Combine high and low bytes (two's complement)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        
        return value

    def get_accel_data(self):
        """
        Get accelerometer data
        
        :return: Dictionary of x, y, z accelerometer readings in g
        """
        x = self.read_raw_data(self.ACCEL_XOUT_H) / self.ACCEL_SCALE_MODIFIER_2G
        y = self.read_raw_data(self.ACCEL_XOUT_H + 2) / self.ACCEL_SCALE_MODIFIER_2G
        z = self.read_raw_data(self.ACCEL_XOUT_H + 4) / self.ACCEL_SCALE_MODIFIER_2G
        
        return {
            'x': round(x, 2),
            'y': round(y, 2),
            'z': round(z, 2)
        }

    def get_gyro_data(self):
        """
        Get gyroscope data
        
        :return: Dictionary of x, y, z gyroscope readings in degrees/sec
        """
        x = self.read_raw_data(self.GYRO_XOUT_H) / self.GYRO_SCALE_MODIFIER_250DEG
        y = self.read_raw_data(self.GYRO_XOUT_H + 2) / self.GYRO_SCALE_MODIFIER_250DEG
        z = self.read_raw_data(self.GYRO_XOUT_H + 4) / self.GYRO_SCALE_MODIFIER_250DEG
        
        return {
            'x': round(x, 2),
            'y': round(y, 2),
            'z': round(z, 2)
        }

    def calculate_angle(self):
        """
        Calculate tilt angles using accelerometer data
        
        :return: Dictionary of roll and pitch angles
        """
        accel = self.get_accel_data()
        
        # Calculate roll (rotation around X-axis)
        roll = math.atan2(accel['y'], accel['z']) * 180 / math.pi
        
        # Calculate pitch (rotation around Y-axis)
        pitch = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)) * 180 / math.pi
        
        return {
            'roll': round(roll, 2),
            'pitch': round(pitch, 2)
        }

def main():
    """
    Example usage of MPU6050 sensor
    """
    try:
        mpu = MPU6050()
        
        while True:
            print("Accelerometer Data:", mpu.get_accel_data())
            print("Gyroscope Data:", mpu.get_gyro_data())
            print("Tilt Angles:", mpu.calculate_angle())
            
            time.sleep(1)
    
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()