import traceback
import ADS1263
import time
# --- BLE Imports ---
from bluezero import adapter
from bluezero import peripheral
from bluezero import async_tools
import logging

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

# --- Calibration Table from AEM 30-2012 Datasheet  ---
# Format: {Resistance_Ohms: Temp_Celsius}
# Selected points covering the full useful range for a rear diff
CALIBRATION_TABLE = {
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
        print(f"ADC_Value {ADC_Value}")
        V_out = ADC_Value * (REF / 0x7fffffff)
        print(f"V_out {V_out}")
        
        if V_out >= (REF - 0.05):
            return -999.0 # Error Code for Open Circuit
            
        r_sensor = (R_PULLUP * V_out) / (REF - V_out)
        print(f"r_sensor {r_sensor}")
        return get_temp_from_resistance(r_sensor)
    except:
        return -999.0
    
# --- BLE Callbacks ---
def read_value():
    """
    Called when a device connects and requests a READ.
    Must return a list of bytes. 
    Standard Temp format (0x2A6E) is sint16 (hundredths of a degree).
    Example: 24.50 C -> 2450
    """
    temp_c = read_current_temp()
    print(f"temp_c {temp_c}")
    
    # Format for BLE (sint16 little endian)
    # Multiply by 100 to keep 2 decimals
    temp_ble_int = int(temp_c * 100) 
    
    # Safety clamp for sint16 (-32768 to 32767)
    temp_ble_int = max(min(temp_ble_int, 32767), -32768)
    
    return list(temp_ble_int.to_bytes(2, byteorder='little', signed=True))

def update_value(characteristic):
    """
    Called by the timer every X seconds to notify connected devices.
    """
    # 1. Read new data
    new_bytes = read_value()
    
    # 2. Update the characteristic
    characteristic.set_value(new_bytes)
    
    # 3. Print to console for debugging
    temp_c = int.from_bytes(new_bytes, byteorder='little', signed=True) / 100.0
    print(f"Sent over BLE: {temp_c:.2f} °C")
    
    # Return True to keep the timer running
    return True

def notify_callback(notifying, characteristic):
    """
    Called when a phone subscribes to notifications.
    Starts the timer loop.
    """
    if notifying:
        print("Device Connected! Starting Notifications...")
        async_tools.add_timer_seconds(1, update_value, characteristic)
    else:
        print("Device Disconnected. Stopping Notifications.")

# --- Main Application ---
def main(adapter_address):
    logger = logging.getLogger('localGATT')
    logger.setLevel(logging.DEBUG)
    
    # 1. Create Peripheral
    # "appearance=1344" tells the phone this is a "Generic Sensor"
    diff_monitor = peripheral.Peripheral(adapter_address, 
                                         local_name='STI Rear Diff', 
                                         appearance=1344)

    # 2. Add Service (Environmental Sensing)
    diff_monitor.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)

    # 3. Add Characteristic (Temperature)
    diff_monitor.add_characteristic(srv_id=1, chr_id=1, uuid=TEMP_CHAR_UUID,
                                    value=[], notifying=False,
                                    flags=['read', 'notify'],
                                    read_callback=read_value,
                                    write_callback=None,
                                    notify_callback=notify_callback)

    # 4. Publish and Run
    print("BLE Advertising started: 'STI Rear Diff'")
    diff_monitor.publish()

if __name__ == '__main__':
    # Automatically find the first Bluetooth adapter (usually hci0)
    adapter_list = list(adapter.Adapter.available())
    if len(adapter_list) > 0:
        main(adapter_list[0].address)
    else:
        print("No Bluetooth Adapter found on this Raspberry Pi.")