import unittest
import sys
import os
import time
import statistics

# Add project root to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.sensors.mpu6050 import MPU6050
from src.sensors.vl53l0x_lidar import VL53L0XSensor
from src.sensors.ir_speed_sensor import IRSpeedSensor
from src.sensors.proximity_sensor import ProximitySensor
from src.sensors.microwave_radar import MicrowaveRadarSensor

class TestSensors(unittest.TestCase):
    def setUp(self):
        """
        Initialize sensors for testing
        """
        try:
            # MPU6050 Accelerometer/Gyroscope
            self.mpu = MPU6050()
            
            # LIDAR Sensor
            self.lidar = VL53L0XSensor()
            
            # IR Speed Sensor
            self.ir_speed = IRSpeedSensor(gpio_pin=17)
            
            # Proximity Sensor
            self.proximity = ProximitySensor(trigger_pin=22, echo_pin=27)
            
            # Microwave Radar
            self.radar = MicrowaveRadarSensor(gpio_pin=23)
        except Exception as e:
            self.skipTest(f"Sensor initialization failed: {e}")

    def test_mpu6050_data_consistency(self):
        """
        Test MPU6050 sensor data consistency and range
        """
        # Collect multiple readings
        accel_readings = []
        gyro_readings = []
        
        for _ in range(10):
            accel_data = self.mpu.get_accel_data()
            gyro_data = self.mpu.get_gyro_data()
            
            accel_readings.append(max(abs(x) for x in accel_data.values()))
            gyro_readings.append(max(abs(x) for x in gyro_data.values()))
            
            time.sleep(0.1)
        
        # Check standard deviation (indicator of noise)
        self.assertLess(statistics.stdev(accel_readings), 0.5, 
                        "Accelerometer readings show high variance")
        self.assertLess(statistics.stdev(gyro_readings), 10, 
                        "Gyroscope readings show high variance")
        
        # Check angle calculation
        angles = self.mpu.calculate_angle()
        self.assertIn('roll', angles, "Angle calculation should include roll")
        self.assertIn('pitch', angles, "Angle calculation should include pitch")

    def test_lidar_distance_measurement(self):
        """
        Test LIDAR sensor distance measurement
        """
        distances = []
        for _ in range(5):
            distance = self.lidar.get_distance()
            self.assertIsNotNone(distance, "LIDAR should return a distance")
            self.assertGreater(distance, 0, "Distance must be positive")
            self.assertLess(distance, 2000, "Distance cannot exceed max range")
            
            distances.append(distance)
            time.sleep(0.2)
        
        # Check measurement consistency
        self.assertLess(statistics.stdev(distances), 50, 
                        "LIDAR measurements show high variance")

    def test_ir_speed_sensor(self):
        """
        Test IR speed sensor functionality
        """
        # Simulate rotation events
        speed_readings = []
        for _ in range(5):
            speed = self.ir_speed.get_speed()
            self.assertIsNotNone(speed, "IR speed sensor should return a speed")
            
            speed_readings.append(speed)
            time.sleep(0.2)
        
        # Optional: Check speed range or variance
        self.assertTrue(all(0 <= s <= 1000 for s in speed_readings), 
                        "Speed readings out of expected range")

    def test_proximity_sensor(self):
        """
        Test proximity sensor distance measurement
        """
        distances = []
        for _ in range(5):
            distance = self.proximity.get_distance()
            self.assertIsNotNone(distance, "Proximity sensor should return a distance")
            self.assertGreater(distance, 0, "Distance must be positive")
            self.assertLess(distance, 400, "Distance cannot exceed max range")
            
            distances.append(distance)
            time.sleep(0.2)
        
        # Check measurement consistency
        self.assertLess(statistics.stdev(distances), 20, 
                        "Proximity sensor measurements show high variance")

    def test_microwave_radar_detection(self):
        """
        Test microwave radar motion detection
        """
        # Collect multiple detection states
        detection_states = []
        for _ in range(10):
            is_detected = self.radar.detect_motion()
            self.assertIsInstance(is_detected, bool, 
                                  "Radar detection should return boolean")
            detection_states.append(is_detected)
            time.sleep(0.2)
        
        # Optional: Check detection logic
        self.assertTrue(len(set(detection_states)) > 1, 
                        "Radar should show variation in detection")

    def tearDown(self):
        """
        Cleanup sensor resources
        """
        try:
            if hasattr(self, 'mpu'):
                del self.mpu
            if hasattr(self, 'lidar'):
                self.lidar.close()
            if hasattr(self, 'ir_speed'):
                self.ir_speed.cleanup()
            if hasattr(self, 'proximity'):
                self.proximity.cleanup()
            if hasattr(self, 'radar'):
                self.radar.cleanup()
        except Exception as e:
            print(f"Sensor cleanup error: {e}")

if __name__ == '__main__':
    unittest.main()