# import RPi.GPIO as GPIO
import gpiozero as GPIO
import logging

class GPIOManager:
    def __init__(self, config):
        """
        Initialize GPIO setup with configuration
        
        Args:
            config (dict): GPIO configuration dictionary
        """
        self.config = config
        self.logger = logging.getLogger('GPIOManager')
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO, 
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        # Set GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
    
    def setup_output_pins(self, pins):
        """
        Setup multiple output pins
        
        Args:
            pins (list or dict): Pins to setup as outputs
        """
        try:
            for pin_name, pin_number in pins.items():
                GPIO.setup(pin_number, GPIO.OUT)
                self.logger.info(f"Configured {pin_name} as output on pin {pin_number}")
        except Exception as e:
            self.logger.error(f"Error setting up output pins: {e}")
    
    def setup_input_pins(self, pins):
        """
        Setup multiple input pins
        
        Args:
            pins (list or dict): Pins to setup as inputs
        """
        try:
            for pin_name, pin_number in pins.items():
                GPIO.setup(pin_number, GPIO.IN)
                self.logger.info(f"Configured {pin_name} as input on pin {pin_number}")
        except Exception as e:
            self.logger.error(f"Error setting up input pins: {e}")
    
    def cleanup(self):
        """
        Cleanup GPIO pins on program exit
        """
        try:
            GPIO.cleanup()
            self.logger.info("GPIO pins cleaned up successfully")
        except Exception as e:
            self.logger.error(f"Error during GPIO cleanup: {e}")