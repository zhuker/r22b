"""
BLE GATT Server using Bumble
Provides RealDash CAN data streaming and H.264 video streaming

Rewritten from hello.py using Google Bumble library
"""

import traceback
import ADS1263
import time
import struct
import asyncio
import os
import sys
import glob
from typing import List, Iterator
from struct import pack
import logging

# Bumble imports
from bumble.device import Device, Connection, AdvertisingType
from bumble.host import Host
from bumble.gatt import (
    Service,
    Characteristic,
    CharacteristicValue,
)
from bumble.att import ATT_Error
from bumble.transport import open_transport
from bumble.core import UUID, AdvertisingData
from bumble import data_types
from bumble.hci import HCI_LE_1M_PHY

# Bumble GATT property constants
PROPERTY_READ = Characteristic.Properties.READ
PROPERTY_WRITE = Characteristic.Properties.WRITE
PROPERTY_WRITE_WITHOUT_RESPONSE = Characteristic.Properties.WRITE_WITHOUT_RESPONSE
PROPERTY_NOTIFY = Characteristic.Properties.NOTIFY
PROPERTY_INDICATE = Characteristic.Properties.INDICATE

# Bleak for TPMS scanning
from bleak import BleakScanner

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# --- Configuration ---
V_SOURCE = 3.3
R_PULLUP = 2200.0
REF = 5.08

# BLE UUIDs
NUS_SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
NUS_RX_CHAR_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
NUS_TX_CHAR_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
H264_STREAM_CHAR_UUID = '6E400012-B5A3-F393-E0A9-E50E24DCCA9E'
H264_CONTROL_CHAR_UUID = '6E400013-B5A3-F393-E0A9-E50E24DCCA9E'

H264_PACKET_SIZE = 500

# Calibration Table
CALIBRATION_TABLE = {
    28136: -20, 15813: -10, 9319: 0, 5589: 10, 3476: 20,
    2230: 30, 1466: 40, 984: 50, 671: 60, 468: 70,
    332: 80, 239: 90, 175: 100, 129: 110, 97: 120,
    73: 130, 57: 140, 43: 150
}

# TPMS Data Storage
tpms_data = {
    'FL': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0},
    'FR': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0},
    'RL': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0},
    'RR': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0}
}

# H.264 Streaming State
h264_streaming_active = False
h264_current_frame = 0
h264_frame_files = []

# H.264 Constants
NAL_TYPE_FU_A = 28
NAL_TYPE_STAP_A = 24
NAL_HEADER_SIZE = 1
FU_A_HEADER_SIZE = 2
STAP_A_HEADER_SIZE = 1
NAL_LENGTH_SIZE = 2

# ADC
ADC = ADS1263.ADS1263()
if ADC.ADS1263_init_ADC1('ADS1263_400SPS') != -1:
    ADC.ADS1263_SetMode(0)

# --- H.264 Packetization Functions ---
def find_nal_units(buf: bytes) -> Iterator[bytes]:
    """Find NAL units in H.264 bitstream."""
    i = 0
    while True:
        i = buf.find(b"\x00\x00\x01", i)
        if i == -1:
            return
        i += 3
        nal_start = i
        i = buf.find(b"\x00\x00\x01", i)
        if i == -1:
            yield buf[nal_start:len(buf)]
            return
        elif buf[i - 1] == 0:
            yield buf[nal_start:i - 1]
        else:
            yield buf[nal_start:i]

def packetize_nal(nal_unit: bytes, max_size: int = H264_PACKET_SIZE) -> List[bytes]:
    """Packetize single NAL unit."""
    packets = []
    nal_size = len(nal_unit)
    if nal_size == 0:
        return packets
    nal_type = nal_unit[0] & 0x1F
    f_nri = nal_unit[0] & 0xE0
    if nal_size <= max_size:
        packets.append(nal_unit)
    else:
        fu_indicator = f_nri | NAL_TYPE_FU_A
        payload_size = max_size - FU_A_HEADER_SIZE
        nal_payload = nal_unit[NAL_HEADER_SIZE:]
        num_fragments = (len(nal_payload) + payload_size - 1) // payload_size
        for i in range(num_fragments):
            start = i * payload_size
            end = min(start + payload_size, len(nal_payload))
            fu_header = nal_type
            if i == 0:
                fu_header |= 0x80
            if i == num_fragments - 1:
                fu_header |= 0x40
            packet = bytes([fu_indicator, fu_header]) + nal_payload[start:end]
            packets.append(packet)
    return packets

def create_stap_a_packet(nal_units: List[bytes], max_size: int = H264_PACKET_SIZE) -> bytes:
    """Create STAP-A packet from multiple NAL units."""
    if not nal_units:
        return b''
    f_nri = nal_units[0][0] & 0xE0
    stap_header = f_nri | NAL_TYPE_STAP_A
    packet = bytes([stap_header])
    for nal_unit in nal_units:
        nal_size = len(nal_unit)
        packet += pack('>H', nal_size)
        packet += nal_unit
    return packet

def packetize_frame(frame_data: bytes, max_size: int = H264_PACKET_SIZE) -> List[bytes]:
    """Packetize complete H.264 frame."""
    all_packets = []
    nal_units = list(find_nal_units(frame_data))
    aggregation_buffer = []
    aggregation_size = STAP_A_HEADER_SIZE
    for nal_unit in nal_units:
        nal_size = len(nal_unit)
        needed_size = NAL_LENGTH_SIZE + nal_size
        needs_fragmentation = nal_size > max_size
        if needs_fragmentation:
            if aggregation_buffer:
                if len(aggregation_buffer) > 1:
                    stap_packet = create_stap_a_packet(aggregation_buffer, max_size)
                    all_packets.append(stap_packet)
                elif len(aggregation_buffer) == 1:
                    all_packets.append(aggregation_buffer[0])
                aggregation_buffer = []
                aggregation_size = STAP_A_HEADER_SIZE
            packets = packetize_nal(nal_unit, max_size)
            all_packets.extend(packets)
        elif aggregation_size + needed_size <= max_size:
            aggregation_buffer.append(nal_unit)
            aggregation_size += needed_size
        else:
            if len(aggregation_buffer) > 1:
                stap_packet = create_stap_a_packet(aggregation_buffer, max_size)
                all_packets.append(stap_packet)
            elif len(aggregation_buffer) == 1:
                all_packets.append(aggregation_buffer[0])
            aggregation_buffer = [nal_unit]
            aggregation_size = STAP_A_HEADER_SIZE + needed_size
    if aggregation_buffer:
        if len(aggregation_buffer) > 1:
            stap_packet = create_stap_a_packet(aggregation_buffer, max_size)
            all_packets.append(stap_packet)
        elif len(aggregation_buffer) == 1:
            all_packets.append(aggregation_buffer[0])
    return all_packets

def read_h264_frame(filepath: str) -> bytes:
    """Read H.264 frame from file."""
    with open(filepath, 'rb') as f:
        return f.read()

# --- Temperature/TPMS Functions ---
def get_temp_from_resistance(r_measured):
    """Interpolates temperature from resistance value."""
    sorted_ohms = sorted(CALIBRATION_TABLE.keys(), reverse=True)
    if r_measured >= sorted_ohms[0]:
        return CALIBRATION_TABLE[sorted_ohms[0]]
    if r_measured <= sorted_ohms[-1]:
        return CALIBRATION_TABLE[sorted_ohms[-1]]
    for i in range(len(sorted_ohms) - 1):
        r_high = sorted_ohms[i]
        r_low = sorted_ohms[i+1]
        if r_low <= r_measured <= r_high:
            t_low_r = CALIBRATION_TABLE[r_high]
            t_high_r = CALIBRATION_TABLE[r_low]
            ratio = (r_measured - r_low) / (r_high - r_low)
            temp = t_high_r - (ratio * (t_high_r - t_low_r))
            return temp
    return None

def read_current_temp():
    """Reads sensor and returns temp in Float."""
    try:
        ADC_Value = ADC.ADS1263_GetChannalValue(0)
        V_out = ADC_Value * (REF / 0x7fffffff)
        if V_out >= (REF - 0.05):
            return -999.0
        r_sensor = (R_PULLUP * V_out) / (REF - V_out)
        return get_temp_from_resistance(r_sensor)
    except:
        return -999.0

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

def tpms_detection_callback(device, advertisement_data):
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
            tpms_data[pos]['pressure'] = p
            tpms_data[pos]['temp'] = t
            tpms_data[pos]['battery'] = b
            tpms_data[pos]['leaking'] = l
        except:
            pass

async def tpms_scanner_loop():
    """Async loop for TPMS BLE scanning."""
    try:
        scanner = BleakScanner(tpms_detection_callback)
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

# --- RealDash CAN Frame Builder ---
def build_realdash_frame(can_id, value):
    """Packs data into RealDash CAN Protocol."""
    frame = bytearray([0x44, 0x33, 0x22, 0x11])
    frame.extend(can_id.to_bytes(4, byteorder='little'))
    temp_int = int(value * 10)
    frame.extend(temp_int.to_bytes(2, byteorder='little', signed=True))
    frame.extend(bytearray([0,0,0,0,0,0]))
    return bytes(frame)

def build_tpms_frame(can_id, pressure_bar, temp_c, leaking, battery_low):
    """Builds TPMS CAN frame."""
    frame = bytearray([0x44, 0x33, 0x22, 0x11])
    frame.extend(can_id.to_bytes(4, byteorder='little'))
    pressure_int = int(pressure_bar * 100)
    frame.extend(pressure_int.to_bytes(2, byteorder='little', signed=False))
    temp_int = int(temp_c * 10)
    frame.extend(temp_int.to_bytes(2, byteorder='little', signed=True))
    frame.append(leaking & 0xFF)
    frame.append(battery_low & 0xFF)
    frame.extend(bytearray([0, 0]))
    return bytes(frame)

# --- GATT Server Implementation ---
class BumbleGATTServer:
    def __init__(self, device: Device):
        self.device = device
        self.nus_tx_char = None
        self.h264_stream_char = None
        self.nus_tx_subscribed = False
        self.h264_stream_subscribed = False
        self.counter = 0
        
        # Setup services
        self.setup_services()
        
    def setup_services(self):
        """Setup GATT services and characteristics."""
        # NUS Service
        nus_service = Service(
            NUS_SERVICE_UUID,
            [
                # TX Characteristic (Notify) - RealDash CAN data
                Characteristic(
                    NUS_TX_CHAR_UUID,
                    PROPERTY_NOTIFY,
                    CharacteristicValue(read=self._read_nus_tx),
                ),
                # RX Characteristic (Write)
                Characteristic(
                    NUS_RX_CHAR_UUID,
                    PROPERTY_WRITE | PROPERTY_WRITE_WITHOUT_RESPONSE,
                    CharacteristicValue(write=self._write_nus_rx),
                ),
                # H.264 Stream Characteristic (Notify)
                Characteristic(
                    H264_STREAM_CHAR_UUID,
                    PROPERTY_NOTIFY,
                    CharacteristicValue(read=self._read_h264_stream),
                ),
                # H.264 Control Characteristic (Write)
                Characteristic(
                    H264_CONTROL_CHAR_UUID,
                    PROPERTY_WRITE | PROPERTY_WRITE_WITHOUT_RESPONSE,
                    CharacteristicValue(write=self._write_h264_control),
                ),
            ]
        )
        
        self.device.add_service(nus_service)
        
        # Store characteristic references for notifications
        self.nus_tx_char = nus_service.characteristics[0]
        self.h264_stream_char = nus_service.characteristics[2]
        
        # Subscribe to characteristic subscription events
        self.nus_tx_char.on('subscription', self._on_nus_tx_subscription)
        self.h264_stream_char.on('subscription', self._on_h264_subscription)
        
    def _read_nus_tx(self, connection):
        """Read handler for NUS TX characteristic."""
        return b''
    
    def _write_nus_rx(self, connection, value):
        """Write handler for NUS RX characteristic."""
        pass
    
    def _read_h264_stream(self, connection):
        """Read handler for H.264 stream characteristic."""
        return b''
    
    def _write_h264_control(self, connection, value):
        """Write handler for H.264 control characteristic."""
        global h264_streaming_active, h264_current_frame, h264_frame_files
        
        cmd = value.decode('utf-8', errors='ignore').strip()
        logger.info(f"H.264 Control: {cmd}")
        
        if cmd == "START":
            h264_streaming_active = True
            h264_current_frame = 0
            h264_frame_files = sorted(glob.glob("h264SampleFrames/frame-*.h264"))
            logger.info(f"H.264: Starting stream ({len(h264_frame_files)} frames)")
        elif cmd == "STOP":
            h264_streaming_active = False
            logger.info("H.264: Stopping stream")
        elif cmd == "RESET":
            h264_current_frame = 0
            logger.info("H.264: Reset to frame 0")
    
    def _on_nus_tx_subscription(self, connection, notify_enabled, indicate_enabled):
        """Handle subscription changes for NUS TX."""
        self.nus_tx_subscribed = notify_enabled
        if notify_enabled:
            logger.info("RealDash Connected! Streaming Data...")
        else:
            logger.info("RealDash Disconnected.")
    
    def _on_h264_subscription(self, connection, notify_enabled, indicate_enabled):
        global h264_streaming_active, h264_frame_files
        """Handle subscription changes for H.264 stream."""
        self.h264_stream_subscribed = notify_enabled
        if notify_enabled:
            logger.info("H.264 Client Connected! Ready to stream.")
            logger.info("Send 'START' to begin streaming")
            h264_streaming_active = True
            h264_frame_files = sorted(glob.glob("h264SampleFrames/frame-*.h264"))
        else:
            logger.info("H.264 Client Disconnected.")
            h264_streaming_active = False
    
    async def send_realdash_data(self):
        """Send RealDash CAN frames."""
        if not self.nus_tx_subscribed:
            return
        
        # Read sensor data
        temp = read_current_temp()
        if temp == -999.0:
            temp = 0.0
        
        # Build frames
        frames = []
        frames.append(build_realdash_frame(3200, temp))
        
        for pos, can_id in [('FL', 3201), ('FR', 3202), ('RL', 3203), ('RR', 3204)]:
            tpms = tpms_data[pos]
            frames.append(build_tpms_frame(can_id, tpms['pressure'], tpms['temp'], 
                                          tpms['leaking'], tpms['battery']))
        
        # Send all frames
        for frame in frames:
            await self.device.notify_subscribers(self.nus_tx_char, frame)
            await asyncio.sleep(0.01)
        
        fl = tpms_data['FL']
        fr = tpms_data['FR']
        rl = tpms_data['RL']
        rr = tpms_data['RR']
        print(f"\rFrame {self.counter}: Diff={temp:.1f}C FL={fl['pressure']:.2f}bar "
              f"FR={fr['pressure']:.2f}bar RL={rl['pressure']:.2f}bar RR={rr['pressure']:.2f}bar  ", end="")
        self.counter += 1
    
    sent_h264_packets = 0
    async def send_h264_frame(self):
        """Send H.264 video frame."""
        global h264_streaming_active, h264_current_frame, h264_frame_files
        
        if not self.h264_stream_subscribed or not h264_streaming_active or not h264_frame_files:
            return
        
        # Get current frame file
        if h264_current_frame >= len(h264_frame_files):
            h264_current_frame = 0
        
        frame_file = h264_frame_files[h264_current_frame]
        
        try:
            # Read and packetize frame
            frame_data = read_h264_frame(frame_file)
            packets = packetize_frame(frame_data, H264_PACKET_SIZE)
            
            # Send each packet as notification
            for packet in packets:
                await self.device.notify_subscribers(self.h264_stream_char, packet)
                # await asyncio.sleep(0.001)
                self.sent_h264_packets += 1
            
            print(f"\rH.264: Frame {h264_current_frame+1}/{len(h264_frame_files)} "
                  f"({len(packets)} packets) Sent={self.sent_h264_packets} ", end="")
            h264_current_frame += 1
            
        except Exception as e:
            logger.error(f"H.264 Error: {e}")

TRANSPORT="hci-socket:0"
DEVICE_NAME="22B_Sensors"

async def main():
    # Open transport
    async with await open_transport(TRANSPORT) as hci_transport:
        logger.info("Transport opened.")
        
        # Create device with HCI transport (similar to Bumble example)
        device = Device.with_hci(
            DEVICE_NAME,
            'F0:F1:F2:F3:F4:F5',
            hci_transport.source,
            hci_transport.sink
        )
        
        # Create GATT server and add services
        gatt_server = BumbleGATTServer(device)
        
        # Debug print GATT attributes
        logger.info("GATT Server Attributes:")
        for attribute in device.gatt_server.attributes:
            logger.info(f"  {attribute}")
        
        # Prepare legacy advertising payloads per Bumble examples
        device.advertising_data = bytes(
            AdvertisingData([
                data_types.Flags(0x02),  # General Discoverable Mode
                data_types.CompleteListOf128BitServiceUUIDs([UUID(NUS_SERVICE_UUID)]),
                data_types.Appearance(
                        data_types.Appearance.Category.WEARABLE_AUDIO_DEVICE,
                        data_types.Appearance.WearableAudioDeviceSubcategory.EARBUD,
                    )
            ])
        )
        device.scan_response_data = bytes(
            AdvertisingData([
                data_types.CompleteLocalName(DEVICE_NAME),
                data_types.Appearance(
                        data_types.Appearance.Category.WEARABLE_AUDIO_DEVICE,
                        data_types.Appearance.WearableAudioDeviceSubcategory.EARBUD,
                    )
            ])
        )

        # Power on the device
        logger.info("Powering on device...")
        await device.power_on()
        logger.info("Device powered on.")
        
        # Start legacy connectable + scannable advertising (shows Flags & services)
        logger.info("Starting legacy advertising (connectable + scannable)...")
        await device.start_advertising(
            advertising_type=AdvertisingType.UNDIRECTED_CONNECTABLE_SCANNABLE,
            auto_restart=True
        )
        logger.info(f"Advertising '{DEVICE_NAME}' with Flags + NUS UUID; name/appearance in scan response.")
        
        # Start TPMS scanner
        asyncio.create_task(tpms_scanner_loop())
        
        # Main loop
        try:
            while True:
                # Send RealDash data every 1 second
                await gatt_server.send_realdash_data()
                # await asyncio.sleep(1)
                
                # Send H.264 frames at ~30 FPS (33ms per frame)
                await gatt_server.send_h264_frame()
                await asyncio.sleep(0.040)
                
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        
        # Wait for termination
        await hci_transport.source.wait_for_termination()

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(main())
