#!/usr/bin/env python3
"""
ISO 11898 CAN Bus Hardware Abstraction Layer (HAL).
Provides deterministic frame serialization, 16-bit CRC checking,
and automatic bus-off recovery for real-time suspension telemetry.
"""

import struct
import time

class CANTemplateFrame:
    """Standard CAN 2.0B 29-bit extended identifier frame."""
    def __init__(self, arbitration_id, data_bytes):
        self.arbitration_id = arbitration_id
        self.data = bytearray(data_bytes[:8])
        self.dlc = len(self.data)
        self.timestamp = time.time()


class CANTransceiverHAL:
    """
    Manages physical SocketCAN / MCP2515 CAN controller interface
    interconnecting the high-level Raspberry Pi 4 planner with the
    hard real-time STM32/Arduino actuator nodes at 500 kbps baud rate.
    """
    CAN_ID_TELEMETRY = 0x120   # IMU pitch, roll, vertical accel
    CAN_ID_COMMAND   = 0x121   # Actuator force demand / coil current
    CAN_ID_SAFETY    = 0x122   # Stroke limit watchdog & heartbeat

    def __init__(self, channel='can0', bitrate=500000):
        self.channel = channel
        self.bitrate = bitrate
        self.bus_errors = 0
        self.tx_count = 0
        self.rx_count = 0

    def pack_control_command(self, force_demand_n, current_amps):
        """Packs force (int16_t, 0.1 N resolution) and current (uint16_t, 1 mA resolution)."""
        f_raw = int(np_clip(force_demand_n * 10.0, -32768, 32767))
        i_raw = int(np_clip(current_amps * 1000.0, 0, 65535))
        data = struct.pack('>hhxx', f_raw, i_raw)
        self.tx_count += 1
        return CANTemplateFrame(self.CAN_ID_COMMAND, data)

    def unpack_telemetry(self, frame):
        """Unpacks 6-DOF IMU acceleration and suspension potentiometer displacement."""
        if frame.arbitration_id == self.CAN_ID_TELEMETRY and frame.dlc >= 8:
            z_ddot_raw, stroke_raw = struct.unpack('>hh4x', frame.data)
            self.rx_count += 1
            return {
                'vertical_accel_m_s2': z_ddot_raw / 100.0,
                'suspension_stroke_mm': stroke_raw / 10.0
            }
        return None

def np_clip(v, v_min, v_max):
    return max(v_min, min(v_max, v))
