import traceback
from r22b.adc import ads1263 as ADS1263
import time
import struct
import asyncio
import threading
import os
import glob
from typing import List, Iterator
from struct import pack
# --- BLE Imports ---
from bluezero import adapter
from bluezero import peripheral
from bluezero import async_tools
import logging
from bleak import BleakScanner

# --- Configuration ---
V_SOURCE = 3.3          # We use 3.3V for Pi safety (Datasheet uses 5V, so we recalc)
R_PULLUP = 2200.0       # 2.2k Ohm resistor as per Datasheet 
REF = 5.08          # Modify according to actual voltage
                        # external AVDD and AVSS(Default), or internal 2.5V
# BLE UUIDs (Standard Environmental Sensing)
# Service: Environmental Sensing (0x181A)
# Characteristic: Temperature (0x2A6E)
SERVICE_UUID = '181A' 
TEMP_CHAR_UUID = '2A6E'
NUS_SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
NUS_RX_CHAR_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E' # Write (App to Pi)
NUS_TX_CHAR_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E' # Notify (Pi to App)

# H.264 Video Stream Service UUIDs
H264_STREAM_CHAR_UUID = '6E400012-B5A3-F393-E0A9-E50E24DCCA9E' # Notify (Pi to App - Video packets)
H264_CONTROL_CHAR_UUID = '6E400013-B5A3-F393-E0A9-E50E24DCCA9E' # Write (App to Pi - Control)

ENABLE_MTU_PROBE = True  # Set True to run a one-time notification length probe
NOTIF_PAYLOAD_LIMIT = 20  # Default safe data bytes per notification
H264_PACKET_SIZE = 500  # Maximum H.264 packet size

# --- Calibration Table from AEM 30-2012 Datasheet  ---
# Format: {Resistance_Ohms: Temp_Celsius}
# Selected points covering the full useful range for a rear diff
OLD_CALIBRATION_TABLE = {
    402000: -40,  # [cite: 50, 52]
    114000: -20,  # [cite: 82, 84]
     37500:   0,  # [cite: 114, 116]
     14000:  20,  # [cite: 146, 148]
     11100:  25,  # [cite: 154, 156]
      5800:  40,  # [cite: 178, 180]
      2700:  60,  # [cite: 54, 56]
      1300:  80,  # [cite: 86, 88]
       965:  90,  # [cite: 102, 104]
       710: 100,  # [cite: 118, 120]
       403: 120,  # [cite: 150, 152]
       241: 140,  # [cite: 182, 184]
       189: 150   # [cite: 198, 200]
}
CALIBRATION_TABLE = {
    28136: -20,  # [cite: 82, 84]
    15813: -10,  # [cite: 82, 84]
     9319:   0,  # [cite: 114, 116]
     5589:  10,  # [cite: 146, 148]
     3476:  20,  # [cite: 154, 156]
      2230:  30,  # [cite: 178, 180]
      1466:  40,  # [cite: 54, 56]
      984:  50,  # [cite: 86, 88]
       671:  60,  # [cite: 102, 104]
       468: 70,  # [cite: 118, 120]
       332: 80,  # [cite: 150, 152]
       239: 90,  # [cite: 182, 184]
       175: 100,   # [cite: 198, 200]
       129: 110,   # [cite: 198, 200]
       97: 120,   # [cite: 198, 200]
       73: 130,   # [cite: 198, 200]
       57: 140,   # [cite: 198, 200]
       43: 150   # [cite: 198, 200]
}

# --- TPMS Data Storage ---
tpms_data = {
    'FL': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0},
    'FR': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0},
    'RL': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0},
    'RR': {'pressure': 0.0, 'temp': 0.0, 'battery': 0, 'leaking': 0}
}

# --- H.264 Video Streaming State ---
h264_streaming_active = True
h264_current_frame = 0
# h264_frame_files = []
h264_frame_files = sorted(glob.glob("data/samples/h264/frame-*.h264"))

# --- H.264 Packetization (from h264_packetize.py) ---
NAL_TYPE_FU_A = 28
NAL_TYPE_STAP_A = 24
NAL_HEADER_SIZE = 1
FU_A_HEADER_SIZE = 2
STAP_A_HEADER_SIZE = 1
NAL_LENGTH_SIZE = 2

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

def decode_tpms(raw_bytes):
    """
    Decodes TPMS data from BLE advertisement
    Returns: (pressure_bar, temp_c, battery_low)
    """
    try:
        # Pressure is Bytes 6-9 (Little Endian uint32)
        pressure_raw = struct.unpack('<I', raw_bytes[6:10])[0]
        # Pressure is already in kPa, convert directly to bar
        pressure_bar = pressure_raw / 100000

        # Temp is Bytes 10-13 (Little Endian uint32)
        temp_raw = struct.unpack('<I', raw_bytes[10:14])[0]
        temp_c = temp_raw / 100

        # Battery is typically at index 14
        battery_raw = raw_bytes[14]
        
        # If battery is low (< 20%), set flag
        battery_low = 1 if battery_raw < 20 else 0

        leaking = raw_bytes[15]  

        return pressure_bar, temp_c, battery_low, leaking
    except:
        return 0.0, 0.0, 0, 0

def tpms_detection_callback(device, advertisement_data):
    """Callback for TPMS sensor detection"""
    if device.name and "TPMS" in device.name:
        try:
            p, t, b, l = decode_tpms(advertisement_data.manufacturer_data[256])
            
            # Map device name to tire position
            # Adjust these names to match your actual TPMS sensor names
            if "TPMS1" in device.name:
                pos = 'FL'
            elif "TPMS2" in device.name:
                pos = 'FR'
            elif "TPMS3" in device.name:
                pos = 'RL'
            elif "TPMS4" in device.name:
                pos = 'RR'
            else:
                pos = 'FL'  # Default
            
            tpms_data[pos]['pressure'] = p
            tpms_data[pos]['temp'] = t
            tpms_data[pos]['battery'] = b
            tpms_data[pos]['leaking'] = l
            
            print(f"{device.name} -> {pos}: {p:.2f} bar | {t:.1f} °C | Battery Low: {b}")
        except Exception as e:
            pass

async def tpms_scanner_loop():
    """Async loop for TPMS BLE scanning"""
    scanner = BleakScanner(tpms_detection_callback)
    await scanner.start()
    print("TPMS Scanner started...")
    while True:
        await asyncio.sleep(1)

def start_tpms_scanner():
    """Start TPMS scanner in separate thread"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(tpms_scanner_loop())

def get_temp_from_resistance(r_measured):
    """
    Interpolates temperature from the resistance value using the AEM datasheet points.
    """
    # Sort resistance keys (descending because NTC: higher resistance = lower temp)
    sorted_ohms = sorted(CALIBRATION_TABLE.keys(), reverse=True)
    
    # Check bounds
    if r_measured >= sorted_ohms[0]:
        return CALIBRATION_TABLE[sorted_ohms[0]]
    if r_measured <= sorted_ohms[-1]:
        return CALIBRATION_TABLE[sorted_ohms[-1]]

    # Linear Interpolation
    for i in range(len(sorted_ohms) - 1):
        r_high = sorted_ohms[i]
        r_low = sorted_ohms[i+1]
        
        if r_low <= r_measured <= r_high:
            t_low_r = CALIBRATION_TABLE[r_high] # Temp at high resistance (colder)
            t_high_r = CALIBRATION_TABLE[r_low] # Temp at low resistance (hotter)
            
            # Interpolate
            ratio = (r_measured - r_low) / (r_high - r_low)
            temp = t_high_r - (ratio * (t_high_r - t_low_r))
            return temp
    return None

def read_diff_temp(ads):
    try:
        # 1. Read Raw Voltage from ADS1263 (Channel 0)
        # Note: ADS1263 library usually returns a value 0-1, or raw bits. 
        # Adjust 'ADC.ADS1263_GetCh0Value()' to your specific library method.
        # Assuming we get a normalized voltage (V_out):
        
        ADC_Value = ads.ADS1263_GetChannalValue(0) # Pseudo-call
        # Convert bits to Voltage if necessary (e.g. value * 3.3 / 0x7FFFFFFF)
        V_out = ADC_Value * (V_SOURCE / 0x7fffffff) 

        # 2. Safety Check (Open Circuit / Unplugged)
        if V_out >= (V_SOURCE - 0.05):
            return "Sensor Unplugged"

        # 3. Calculate Resistance using Voltage Divider Law
        # R_sensor = (R_pullup * V_out) / (V_source - V_out)
        r_sensor = (R_PULLUP * V_out) / (V_SOURCE - V_out)

        # 4. Get Temperature
        temp_c = get_temp_from_resistance(r_sensor)
        return temp_c

    except Exception as e:
        traceback.print_exc()
        return f"Error: {e}"

def main0():
    # Test Loop
    ADC = ADS1263.ADS1263()

    # The faster the rate, the worse the stability
    # and the need to choose a suitable digital filter(REG_MODE1)
    if (ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1):
        exit()
    ADC.ADS1263_SetMode(0) # 0 is singleChannel, 1 is diffChannel

    # ADC.ADS1263_DAC_Test(1, 1)      # Open IN6
    # ADC.ADS1263_DAC_Test(0, 1)      # Open IN7
   
    i=0
    while True:
        ADC_Value = ADC.ADS1263_GetChannalValue(0)    # get ADC1 value
            # Convert bits to Voltage if necessary (e.g. value * 3.3 / 0x7FFFFFFF)
        V_out = ADC_Value * (V_SOURCE / 0x7fffffff) 
        V_out5 = ADC_Value * (REF / 0x7fffffff) 
        #print(f"V_out {V_out} {V_out5}")

        # 2. Safety Check (Open Circuit / Unplugged)
        # if V_out >= (V_SOURCE - 0.05):
            # return "Sensor Unplugged"

        # 3. Calculate Resistance using Voltage Divider Law
        # R_sensor = (R_pullup * V_out) / (V_source - V_out)
        new_var = (V_SOURCE - V_out)
        if new_var <= 0:
            continue
        r_sensor = (R_PULLUP * V_out) / new_var
        r_sensor5 = (R_PULLUP * V_out5) / (REF - V_out5)
        #print(f"r_sensor {r_sensor} {r_sensor5}")


        # 4. Get Temperature
        temp_c = get_temp_from_resistance(r_sensor)
        temp_c5 = get_temp_from_resistance(r_sensor5)

        print("1 ADC1 IN%d = %.3f  %.3f %d %d %.3f C %.3f C" %(i, V_out, V_out5, int(r_sensor), int(r_sensor5), temp_c, temp_c5))   # 32bit
        print("\33[2A")

# Test Loop
ADC = ADS1263.ADS1263()

# The faster the rate, the worse the stability
# and the need to choose a suitable digital filter(REG_MODE1)
if (ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1):
    exit()
ADC.ADS1263_SetMode(0) # 0 is singleChannel, 1 is diffChannel

def read_current_temp():
    """Reads sensor and returns temp in Float (e.g. 25.55)"""
    try:
        ADC_Value = ADC.ADS1263_GetChannalValue(0)
        # print(f"ADC_Value {ADC_Value}")
        V_out = ADC_Value * (REF / 0x7fffffff)
        # print(f"V_out {V_out}")
        
        if V_out >= (REF - 0.05):
            return -999.0 # Error Code for Open Circuit
            
        r_sensor = (R_PULLUP * V_out) / (REF - V_out)
        # print(f"r_sensor {r_sensor}")
        return get_temp_from_resistance(r_sensor)
    except:
        return -999.0
    
# --- RealDash CAN Frame Builder ---
def build_realdash_frame(can_id, value):
    """
    Packs data into RealDash CAN Protocol (Frame 44)
    Header: 0x44, 0x33, 0x22, 0x11
    ID: 4 bytes (Little Endian)
    Data: 8 bytes (Little Endian)
    """
    # 1. Header (0x44332211)
    frame = bytearray([0x44, 0x33, 0x22, 0x11])
    
    # 2. CAN ID (3200 = 0x0C80) -> Little Endian
    frame.extend(can_id.to_bytes(4, byteorder='little'))
    
    # 3. Data (8 bytes)
    # We will put Temp (Celsius) in the first 2 bytes (int16), multiplied by 10
    # e.g., 25.5 C -> 255
    temp_int = int(value * 10)
    
    # Pack into 8 bytes (2 bytes for data, 6 bytes padding)
    frame.extend(temp_int.to_bytes(2, byteorder='little', signed=True))
    frame.extend(bytearray([0,0,0,0,0,0]))
    return list(frame)

def build_tpms_frame(can_id, pressure_bar, temp_c, leaking, battery_low):
    """
    Builds TPMS CAN frame
    Data layout:
    - Offset 0-1: Pressure in bar * 100 (uint16)
    - Offset 2-3: Temperature in C * 10 (int16)
    - Offset 4: Leaking flag (uint8)
    - Offset 5: Battery Low flag (uint8)
    - Offset 6-7: Reserved (0)
    """
    frame = bytearray([0x44, 0x33, 0x22, 0x11])
    frame.extend(can_id.to_bytes(4, byteorder='little'))
    
    # Pressure (bar * 100)
    pressure_int = int(pressure_bar * 100)
    frame.extend(pressure_int.to_bytes(2, byteorder='little', signed=False))
    
    # Temperature (C * 10)
    temp_int = int(temp_c * 10)
    frame.extend(temp_int.to_bytes(2, byteorder='little', signed=True))
    
    # Leaking flag
    frame.append(leaking & 0xFF)
    
    # Battery Low flag
    frame.append(battery_low & 0xFF)
    
    # Reserved bytes
    frame.extend(bytearray([0, 0]))
    
    return list(frame)

# --- D-Bus MTU Logger ---
try:
    from pydbus import SystemBus
    _PYDBUS_AVAILABLE = True
except Exception:
    _PYDBUS_AVAILABLE = False

def log_negotiated_mtu(target_uuid=NUS_TX_CHAR_UUID):
    """Logs negotiated ATT MTU via BlueZ D-Bus.

    Strategy:
    1) Prefer org.bluez.Device1 'MTU' on any Connected device (server side).
    2) Fallback: try to locate our characteristic UUID and read its 'MTU'.
    """
    if not _PYDBUS_AVAILABLE:
        print("MTU: pydbus not installed; skipping MTU query.")
        return
    try:
        bus = SystemBus()
        om = bus.get('org.bluez', '/')
        managed = om.GetManagedObjects()

        # 1) Check Device1 objects for a connected central and read MTU
        connected_devices = []
        for path, ifaces in managed.items():
            dev = ifaces.get('org.bluez.Device1')
            if not dev:
                continue
            if dev.get('Connected'):
                connected_devices.append((path, dev))
        if connected_devices:
            for path, dev in connected_devices:
                mtu = dev.get('MTU')
                addr = dev.get('Address')
                name = dev.get('Name')
                if mtu is not None:
                    print(f"Negotiated MTU: {mtu} (device {name or ''} {addr or ''} at {path})")
                    return
            # If connected but MTU missing, continue to characteristic fallback

        # 2) Fallback: locate our characteristic by UUID and attempt reading MTU
        target_uuid_lower = (target_uuid or '').lower()
        for path, ifaces in managed.items():
            props = ifaces.get('org.bluez.GattCharacteristic1')
            if not props:
                continue
            uuid = props.get('UUID', '').lower()
            if target_uuid_lower and uuid == target_uuid_lower:
                mtu = props.get('MTU')
                if mtu is not None:
                    print(f"Negotiated MTU: {mtu} (characteristic at {path})")
                    return
                try:
                    char = bus.get('org.bluez', path)
                    mtu = char.Get('org.bluez.GattCharacteristic1', 'MTU')
                    print(f"Negotiated MTU: {mtu} (characteristic at {path})")
                except Exception:
                    print(f"MTU property not available on characteristic {path}.")
                return

        # At this point, likely running as GATT server. BlueZ typically exposes
        # 'MTU' on Device1 only when acting as a GATT client. For server role,
        # MTU may not be available via D-Bus.
        print("MTU: no Device1 with MTU and matching characteristic not found.")
        # Diagnostic: list connected Device1 entries and all GATT characteristics
        try:
            connected_paths = []
            for path, ifaces in managed.items():
                dev = ifaces.get('org.bluez.Device1')
                if dev and dev.get('Connected'):
                    connected_paths.append(path)
            if connected_paths:
                print("Connected Device1 paths:")
                for p in connected_paths:
                    dev = managed[p]['org.bluez.Device1']
                    print(f" - {p} name={dev.get('Name')} addr={dev.get('Address')} hasMTU={'MTU' in dev}")
            else:
                print("No connected Device1 entries found (server role likely).")

            print("Available GattCharacteristic1 UUIDs:")
            for path, ifaces in managed.items():
                props = ifaces.get('org.bluez.GattCharacteristic1')
                if not props:
                    continue
                print(f" - {path} uuid={props.get('UUID')} service={props.get('Service')}")
        except Exception:
            pass
    except Exception as e:
        print(f"MTU query failed: {e}")

def probe_notification_limit(characteristic):
    """
    Attempts to find the maximum notification payload length accepted
    by the current connection. Starts from a safe upper bound and
    decreases until a notification is accepted without error.

    Note: This sends a dummy RealDash CAN-like frame padded to target length.
    RealDash should ignore unknown CAN IDs. Use sparingly.
    """
    # Start from a typical negotiated max (e.g., 100 bytes) and go down.
    # Default BLE 4.0 MTU (23) allows 20 data bytes.
    global NOTIF_PAYLOAD_LIMIT
    for length in [100, 64, 40, 32, 24, 20, 16]:
        try:
            # Build header + ID + data with padding
            header = bytearray([0x44, 0x33, 0x22, 0x11])
            can_id = (0xD000).to_bytes(4, byteorder='little')  # dummy CAN id
            data_len = max(0, length - len(header) - len(can_id))
            payload = header + can_id + bytearray([0]*data_len)
            characteristic.set_value(list(payload))
            print(f"MTU probe: notification accepted at {length} bytes.")
            NOTIF_PAYLOAD_LIMIT = length
            return length
        except Exception as e:
            # Try next smaller length
            continue
    print("MTU probe: unable to send even minimal payload; using 16 bytes.")
    NOTIF_PAYLOAD_LIMIT = 16
    return 16

# --- H.264 Streaming Functions ---
def h264_control_callback(value, options, characteristic):
    """Handle control commands for H.264 streaming."""
    global h264_streaming_active, h264_current_frame, h264_frame_files
    
    if not value:
        return
    
    cmd = bytes(value).decode('utf-8', errors='ignore').strip()
    print(f"\nH.264 Control: {cmd}")
    
    if cmd == "START":
        h264_streaming_active = True
        h264_current_frame = 0
        # Load frame files
        h264_frame_files = sorted(glob.glob("data/samples/h264/frame-*.h264"))
        print(f"H.264: Starting stream ({len(h264_frame_files)} frames)")
    elif cmd == "STOP":
        h264_streaming_active = False
        print("H.264: Stopping stream")
    elif cmd == "RESET":
        h264_current_frame = 0
        print("H.264: Reset to frame 0")

packets_sent = 0
def stream_h264_frame(characteristic):
    """Stream one H.264 frame as BLE notifications."""
    global h264_streaming_active, h264_current_frame, h264_frame_files
    global packets_sent
    
    if not h264_streaming_active or not h264_frame_files:
        return True
    
    # Get current frame file
    if h264_current_frame >= len(h264_frame_files):
        h264_current_frame = 0  # Loop back
    
    frame_file = h264_frame_files[h264_current_frame]
    
    try:
        # Read and packetize frame
        frame_data = read_h264_frame(frame_file)
        packets = packetize_frame(frame_data, H264_PACKET_SIZE)
        
        # Send each packet as notification
        for packet in packets:
            characteristic.set_value(list(packet))
            time.sleep(0.001)  # Small delay between packets
            packets_sent += 1
        
        print(f"\rH.264: Frame {h264_current_frame+1}/{len(h264_frame_files)} ({len(packets)} packets, {packets_sent} total packets sent)", end="")
        h264_current_frame += 1
        
    except Exception as e:
        print(f"\nH.264 Error: {e}")
    
    return True

def h264_notify_callback(notifying, characteristic):
    """Called when H.264 stream notifications are enabled/disabled."""
    if notifying:
        print("\nH.264 Client Connected! Ready to stream.")
        print("Send 'START' to begin streaming")
        # Start streaming loop (30 FPS = ~33ms per frame)
        async_tools.add_timer_ms(80, stream_h264_frame, characteristic)
    else:
        print("\nH.264 Client Disconnected.")
        global h264_streaming_active
        h264_streaming_active = False

# --- BLE Loop ---
counter=0
def send_can_frame(characteristic):
    global counter
    # 1. Read Diff Temp
    temp = read_current_temp()
    
    if temp == -999.0:
        temp = 0.0  # Default value if error
    
    # 2. Build all frames
    frames = []
    
    # Frame 1: Diff Temp (CAN ID 0xC80 = 3200)
    frames.append(build_realdash_frame(3200, temp))
    
    # Frame 2: Front Left TPMS (CAN ID 0xC81 = 3201)
    fl = tpms_data['FL']
    frames.append(build_tpms_frame(3201, fl['pressure'], fl['temp'], fl['leaking'], fl['battery']))
    
    # Frame 3: Front Right TPMS (CAN ID 0xC82 = 3202)
    fr = tpms_data['FR']
    frames.append(build_tpms_frame(3202, fr['pressure'], fr['temp'], fr['leaking'], fr['battery']))
    
    # Frame 4: Rear Left TPMS (CAN ID 0xC83 = 3203)
    rl = tpms_data['RL']
    frames.append(build_tpms_frame(3203, rl['pressure'], rl['temp'], rl['leaking'], rl['battery']))
    
    # Frame 5: Rear Right TPMS (CAN ID 0xC84 = 3204)
    rr = tpms_data['RR']
    frames.append(build_tpms_frame(3204, rr['pressure'], rr['temp'], rr['leaking'], rr['battery']))
    
    # 3. Send frames: bundle if within NOTIF_PAYLOAD_LIMIT, else individually
    combined = []
    for frame in frames:
        combined.extend(frame)
    if len(combined) <= NOTIF_PAYLOAD_LIMIT:
        characteristic.set_value(combined)
    else:
        for frame in frames:
            characteristic.set_value(frame)
            time.sleep(0.01)
    
    print(f"\rFrame {counter}: Diff={temp:.1f}C FL={fl['pressure']:.2f}bar FR={fr['pressure']:.2f}bar RL={rl['pressure']:.2f}bar RR={rr['pressure']:.2f}bar  ", end="")
    counter += 1
    return True

def notify_callback(notifying, characteristic):
    if notifying:
        print("\nRealDash Connected! Streaming Data...")
        # Log negotiated MTU once a central connects
        log_negotiated_mtu()
        if ENABLE_MTU_PROBE:
            probe_notification_limit(characteristic)
        # Update fast (100ms) for smooth gauge movement
        async_tools.add_timer_seconds(1, send_can_frame, characteristic)
    else:
        print("\nRealDash Disconnected.")

def main(adapter_address):
    # Create Peripheral
    tx_monitor = peripheral.Peripheral(adapter_address, 
                                       local_name='STI_Diff_Sensor', 
                                       appearance=1344)

    # Add Nordic UART Service (Service ID 1)
    tx_monitor.add_service(srv_id=1, uuid=NUS_SERVICE_UUID, primary=True)

    # Add TX Characteristic (Notify) - RealDash CAN data
    tx_monitor.add_characteristic(srv_id=1, chr_id=1, uuid=NUS_TX_CHAR_UUID,
                                  value=[], notifying=False,
                                  flags=['notify'],
                                  notify_callback=notify_callback)
                                  
    # Add RX Characteristic (Write - Required for NUS but we won't use it)
    tx_monitor.add_characteristic(srv_id=1, chr_id=2, uuid=NUS_RX_CHAR_UUID,
                                  value=[], notifying=False,
                                  flags=['write', 'write-without-response'],
                                  write_callback=None)

    # Add H.264 Stream Characteristic (Notify) - Video packets
    tx_monitor.add_characteristic(srv_id=1, chr_id=3, uuid=H264_STREAM_CHAR_UUID,
                                  value=[], notifying=False,
                                  flags=['notify'],
                                  notify_callback=h264_notify_callback)
    
    # Add H.264 Control Characteristic (Write) - Stream control
    tx_monitor.add_characteristic(srv_id=1, chr_id=4, uuid=H264_CONTROL_CHAR_UUID,
                                  value=[], notifying=False,
                                  flags=['write', 'write-without-response'],
                                  write_callback=h264_control_callback)

    print("Advertising 'STI_Diff_Sensor' with RealDash + H.264 Video Stream...")
    
    # Start TPMS scanner in background thread
    tpms_thread = threading.Thread(target=start_tpms_scanner, daemon=True)
    tpms_thread.start()
    
    tx_monitor.publish()

if __name__ == '__main__':
    adapter_list = list(adapter.Adapter.available())
    if len(adapter_list) > 0:
        main(adapter_list[0].address)
    else:
        print("No Bluetooth Adapter found.")
