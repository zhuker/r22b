import traceback
import ADS1263
import time
import struct
import asyncio
import threading
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
    
    # 3. Send all frames individually
    # Send each frame separately to avoid exceeding BLE MTU limits
    for frame in frames:
        characteristic.set_value(frame)
        time.sleep(0.01)  # Small delay between frames
    
    print(f"\rFrame {counter}: Diff={temp:.1f}C FL={fl['pressure']:.2f}bar FR={fr['pressure']:.2f}bar RL={rl['pressure']:.2f}bar RR={rr['pressure']:.2f}bar  ", end="")
    counter += 1
    return True

def notify_callback(notifying, characteristic):
    if notifying:
        print("\nRealDash Connected! Streaming Data...")
        # Update fast (100ms) for smooth gauge movement
        async_tools.add_timer_seconds(1, send_can_frame, characteristic)
    else:
        print("\nRealDash Disconnected.")

def main(adapter_address):
    # Create Peripheral
    tx_monitor = peripheral.Peripheral(adapter_address, 
                                       local_name='STI_Diff_Sensor', 
                                       appearance=1344)

    # Add Nordic UART Service
    tx_monitor.add_service(srv_id=1, uuid=NUS_SERVICE_UUID, primary=True)

    # Add TX Characteristic (Notify)
    tx_monitor.add_characteristic(srv_id=1, chr_id=1, uuid=NUS_TX_CHAR_UUID,
                                  value=[], notifying=False,
                                  flags=['notify'],
                                  notify_callback=notify_callback)
                                  
    # Add RX Characteristic (Write - Required for NUS but we won't use it)
    tx_monitor.add_characteristic(srv_id=1, chr_id=2, uuid=NUS_RX_CHAR_UUID,
                                  value=[], notifying=False,
                                  flags=['write', 'write-without-response'],
                                  write_callback=None)

    print("Advertising 'STI_Diff_Sensor' for RealDash...")
    
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
