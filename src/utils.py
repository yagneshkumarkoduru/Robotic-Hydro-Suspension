import os
import json
import logging
from datetime import datetime
import psutil

class SystemUtils:
    @staticmethod
    def get_system_info():
        """
        Retrieve comprehensive system information
        
        Returns:
            dict: System performance and resource metrics
        """
        try:
            return {
                'cpu_usage': psutil.cpu_percent(),
                'memory_usage': psutil.virtual_memory().percent,
                'disk_usage': psutil.disk_usage('/').percent,
                'network_stats': SystemUtils.get_network_stats(),
                'timestamp': datetime.now().isoformat()
            }
        except Exception as e:
            logging.error(f"Error retrieving system info: {e}")
            return {}
    
    @staticmethod
    def get_network_stats():
        """
        Get network interface statistics
        
        Returns:
            dict: Network interface stats
        """
        try:
            net_stats = psutil.net_io_counters()
            return {
                'bytes_sent': net_stats.bytes_sent,
                'bytes_recv': net_stats.bytes_recv,
                'packets_sent': net_stats.packets_sent,
                'packets_recv': net_stats.packets_recv
            }
        except Exception as e:
            logging.error(f"Error retrieving network stats: {e}")
            return {}

class ConfigManager:
    @staticmethod
    def load_config(config_path='config/config.json'):
        """
        Load configuration from JSON file
        
        Args:
            config_path (str): Path to configuration file
        
        Returns:
            dict: Loaded configuration
        """
        try:
            with open(config_path, 'r') as config_file:
                return json.load(config_file)
        except FileNotFoundError:
            logging.error(f"Configuration file not found: {config_path}")
            return {}
        except json.JSONDecodeError:
            logging.error(f"Invalid JSON in configuration file: {config_path}")
            return {}
    
    @staticmethod
    def save_config(config, config_path='config/config.json'):
        """
        Save configuration to JSON file
        
        Args:
            config (dict): Configuration to save
            config_path (str): Path to save configuration
        """
        try:
            with open(config_path, 'w') as config_file:
                json.dump(config, config_file, indent=4)
            logging.info(f"Configuration saved to {config_path}")
        except Exception as e:
            logging.error(f"Error saving configuration: {e}")

class LoggingManager:
    @staticmethod
    def setup_logging(log_dir='logs', log_level=logging.INFO):
        """
        Setup logging configuration
        
        Args:
            log_dir (str): Directory to store log files
            log_level (int): Logging level
        """
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, f"robotic_vehicle_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler()
            ]
        )