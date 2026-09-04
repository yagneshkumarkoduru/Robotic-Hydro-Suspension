import sys
import logging
import signal
from utils import ConfigManager, LoggingManager
from gpio_setup import GPIOManager
from vehicle_control import VehicleController

class RoboticVehicleApp:
    def __init__(self):
        """
        Initialize the Robotic Vehicle Application
        """
        # Setup logging
        LoggingManager.setup_logging()
        self.logger = logging.getLogger('RoboticVehicleApp')
        
        # Load configuration
        self.config = ConfigManager.load_config()
        if not self.config:
            self.logger.critical("Failed to load configuration. Exiting.")
            sys.exit(1)
        
        # Initialize GPIO
        self.gpio_manager = GPIOManager(self.config)
        self.setup_gpio()
        
        # Initialize Vehicle Controller
        self.vehicle_controller = VehicleController(self.config)
        
        # Setup signal handlers for graceful shutdown
        self.setup_signal_handlers()
    
    def setup_gpio(self):
        """
        Setup GPIO pins based on configuration
        """
        try:
            # Setup output pins from configuration
            output_pins = self.config.get('gpio', {})
            self.gpio_manager.setup_output_pins(output_pins)
            
            # Setup input pins from configuration
            input_pins = self.config.get('sensors', {})
            self.gpio_manager.setup_input_pins(input_pins)
        except Exception as e:
            self.logger.error(f"GPIO setup failed: {e}")
    
    def setup_signal_handlers(self):
        """
        Setup signal handlers for graceful shutdown
        """
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """
        Handle termination signals
        
        Args:
            signum (int): Signal number
            frame (frame): Current stack frame
        """
        self.logger.info(f"Received signal {signum}. Performing cleanup.")
        self.vehicle_controller.emergency_stop()
        self.gpio_manager.cleanup()
        sys.exit(0)
    
    def run(self):
        """
        Main application run method
        """
        try:
            self.logger.info("Robotic Vehicle Application Started")
            
            # Example driving sequence
            self.vehicle_controller.drive(speed=0.5, direction='forward')
            
            # Add more application logic here
            
        except Exception as e:
            self.logger.critical(f"Application error: {e}")
            self.vehicle_controller.emergency_stop()
        finally:
            self.cleanup()
    
    def cleanup(self):
        """
        Perform final cleanup
        """
        self.gpio_manager.cleanup()
        self.logger.info("Application shutdown complete")

def main():
    """
    Entry point for the application
    """
    try:
        app = RoboticVehicleApp()
        app.run()
    except Exception as e:
        logging.critical(f"Unhandled exception: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()