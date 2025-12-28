#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
import ADS1263
import RPi.GPIO as GPIO

REF = 5.08          # Modify according to actual voltage
                    # external AVDD and AVSS(Default), or internal 2.5V

# ADC1 test part
TEST_ADC1       = True

# --- Configuration ---
V_SOURCE = 3.3          # We use 3.3V for Pi safety (Datasheet uses 5V, so we recalc)
R_PULLUP = 2200.0       # 2.2k Ohm resistor as per Datasheet 

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


try:
    ADC = ADS1263.ADS1263()
    
    # The faster the rate, the worse the stability
    # and the need to choose a suitable digital filter(REG_MODE1)
    if (ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1):
        exit()
    ADC.ADS1263_SetMode(0) # 0 is singleChannel, 1 is diffChannel

    # ADC.ADS1263_DAC_Test(1, 1)      # Open IN6
    # ADC.ADS1263_DAC_Test(0, 1)      # Open IN7
    
    if(TEST_ADC1):       # ADC1 Test
        channelList = [0]  # The channel must be less than 10
        while(1):
            #print("before ADC_Value")
            ADC_Value = ADC.ADS1263_GetChannalValue(0)    # get ADC1 value
            #print(f"ADC_Value {ADC_Value}")
            for i in channelList:
                if(ADC_Value>>31 ==1):
                    print("0 ADC1 IN%d = -%lf" %(i, (REF*2 - ADC_Value[i] * REF / 0x80000000)))  
                else:
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
                        print(new_var)
                        continue
                    r_sensor = (R_PULLUP * V_out) / new_var
                    r_sensor5 = (R_PULLUP * V_out5) / (REF - V_out5)
                    #print(f"r_sensor {r_sensor} {r_sensor5}")


                    # 4. Get Temperature
                    temp_c = get_temp_from_resistance(r_sensor)
                    temp_c5 = get_temp_from_resistance(r_sensor5)

                    print("1 ADC1 IN%d = %.3f  %.3f %d %d %.3f C %.3f C" %(i, V_out, V_out5, int(r_sensor), int(r_sensor5), temp_c, temp_c5))   # 32bit
            # for i in channelList:
                # print("\33[2A")
        
    elif(TEST_ADC2):
        if (ADC.ADS1263_init_ADC2('ADS1263_ADC2_400SPS') == -1):
            exit()
        while(1):
            ADC_Value = ADC.ADS1263_GetAll_ADC2()   # get ADC2 value
            for i in range(0, 10):
                if(ADC_Value[i]>>23 ==1):
                    print("ADC2 IN%d = -%lf"%(i, (REF*2 - ADC_Value[i] * REF / 0x800000)))
                else:
                    print("ADC2 IN%d = %lf"%(i, (ADC_Value[i] * REF / 0x7fffff)))     # 24bit
            print("\33[11A")

    elif(TEST_ADC1_RATE):    # rate test
        time_start = time.time()
        ADC_Value = []
        isSingleChannel = True
        if isSingleChannel:
            while(1):
                ADC_Value.append(ADC.ADS1263_GetChannalValue(0))
                if len(ADC_Value) == 5000:
                    time_end = time.time()
                    print(time_start, time_end)
                    print(time_end - time_start)
                    print('frequency = ', 5000 / (time_end - time_start))
                    break
        else:
            while(1):
                ADC_Value.append(ADC.ADS1263_GetChannalValue(0))
                if len(ADC_Value) == 5000:
                    time_end = time.time()
                    print(time_start, time_end)
                    print(time_end - time_start)
                    print('frequency = ', 5000 / (time_end - time_start))
                    break

    elif(TEST_RTD):     # RTD Test
        while(1):
            ADC_Value = ADC.ADS1263_RTD_Test()
            RES = ADC_Value / 2147483647.0 * 2.0 *2000.0       #2000.0 -- 2000R, 2.0 -- 2*i
            print("RES is %lf"%RES)
            TEMP = (RES/100.0 - 1.0) / 0.00385      #0.00385 -- pt100
            print("TEMP is %lf"%TEMP)
            print("\33[3A")
        
    ADC.ADS1263_Exit()

except IOError as e:
    print(e)
   
except KeyboardInterrupt:
    print("ctrl + c:")
    print("Program end")
    ADC.ADS1263_Exit()
    exit()
   
