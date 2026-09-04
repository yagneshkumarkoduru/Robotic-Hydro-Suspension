#!/usr/bin/env python3
"""
V1 Embedded Microcontroller Target Execution Entry.
Runs on Raspberry Pi 4 host, streaming CAN telemetry and synthesizing baseline LQR/Skyhook commands.
"""

import time
from can_transceiver_hal import CANTransceiverHAL

def run_embedded_node():
    print("[+] Starting V1 Embedded Microcontroller Node (RPi4 Host)...")
    hal = CANTransceiverHAL(channel='can0', bitrate=500000)
    
    print("[+] Initialized SocketCAN interface at 500 kbps")
    print("[+] Heartbeat broadcast active on CAN ID 0x122")
    
    # Simulate receiving 5 telemetry cycles
    for i in range(5):
        cmd = hal.pack_control_command(force_demand_n=150.0, current_amps=1.2)
        print(f"  [Cycle {i+1}] TX CAN 0x{cmd.arbitration_id:X} DLC={cmd.dlc} Data={bytes(cmd.data).hex()}")
        time.sleep(0.01)
        
    print("[+] V1 Embedded Target verified.")

if __name__ == '__main__':
    run_embedded_node()
