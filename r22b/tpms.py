"""
BLE TPMS sensors: advertisement decoding and a Bleak scanner loop.

Sensors advertise as TPMS1_xxxxxx .. TPMS4_xxxxxx with manufacturer data under
company id 256. Split out of bumble_service.py; logic unchanged.
"""

import asyncio
import logging
import struct
from typing import Dict

logger = logging.getLogger(__name__)


class TPMSData:
    """Container for TPMS sensor data."""
    def __init__(self):
        self.data: Dict[str, Dict[str, float]] = {
            'FL': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0},
            'FR': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0},
            'RL': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0},
            'RR': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0}
        }

    def update(self, position: str, pressure: float, temp: float, battery: int, leaking: int):
        """Update TPMS data for a position."""
        if position in self.data:
            self.data[position]['pressure'] = pressure
            self.data[position]['temp'] = temp
            self.data[position]['battery'] = battery
            self.data[position]['leaking'] = leaking

    def get(self, position: str) -> Dict[str, float]:
        """Get TPMS data for a position."""
        return self.data.get(position, {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0})


def decode_tpms(raw_bytes):
    """Decodes TPMS data from BLE advertisement."""
    try:
        pressure_raw = struct.unpack('<I', raw_bytes[6:10])[0]
        pressure_bar = pressure_raw / 100000
        temp_raw = struct.unpack('<I', raw_bytes[10:14])[0]
        temp_c = temp_raw / 100
        battery_raw = raw_bytes[14]
        battery_low = 1 if battery_raw < 20 else 0
        leaking = raw_bytes[15]
        return pressure_bar, temp_c, battery_low, leaking
    except:
        return 0.0, 0.0, 0, 0


async def tpms_scanner_loop(tpms_data: TPMSData):
    """Async loop for TPMS BLE scanning."""
    from bleak import BleakScanner

    def callback(device, advertisement_data):
        """Callback for TPMS sensor detection."""
        if device.name and "TPMS" in device.name:
            try:
                p, t, b, l = decode_tpms(advertisement_data.manufacturer_data[256])
                if "TPMS1" in device.name:
                    pos = 'FL'
                elif "TPMS2" in device.name:
                    pos = 'FR'
                elif "TPMS3" in device.name:
                    pos = 'RL'
                elif "TPMS4" in device.name:
                    pos = 'RR'
                else:
                    pos = 'FL'
                tpms_data.update(pos, p, t, b, l)
            except:
                pass

    try:
        scanner = BleakScanner(callback)
        await scanner.start()
        logger.info("TPMS Scanner started...")
        while True:
            await asyncio.sleep(1)
    except Exception as e:
        logger.warning(f"TPMS Scanner unavailable (Bumble using HCI): {e}")
        logger.info("TPMS scanning disabled - Bumble has exclusive HCI access")
        # Keep running but do nothing
        while True:
            await asyncio.sleep(10)
