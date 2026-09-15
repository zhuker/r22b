"""
RealDash Target ID to Parameter ID Mapping

Maps RealDash target identifiers to supported SSM/ECU parameter IDs.
Based on: https://realdash.net/manuals/targetid.php
"""

# RealDash Target ID -> Parameter ID mapping
REALDASH_TARGET_MAP = {
    # Engine/ECU Inputs
    0: None,    # AFR 1 - Not directly mapped
    1: None,    # AFR 2 - Not directly mapped
    2: None,    # AFR Target 1 - Not directly mapped
    3: None,    # AFR Target 2 - Not directly mapped
    12: 'P17',  # Battery Voltage
    14: 'P2',   # Coolant Temperature
    27: 'P11',  # Intake Air Temperature
    30: 'P12',  # MAF (g/s)
    31: 'P7',   # Manifold Absolute Pressure (kPa)
    35: 'P21',  # Pulse Width 1 (ms)
    37: 'P8',   # RPM
    38: 'P10',  # Spark Advance (deg)
    42: 'P13',  # Throttle Position (%)
    64: 'P9',   # Vehicle Speed (KPH)
    65: None,   # Check Engine Light
    100: 'P200', # Engine Load (%)
    139: 'E59',  # Gear (Transmission)
    151: None,  # Engine Oil Pressure (Bar)
    152: None,  # Engine Oil Temperature (Celsius)
    202: None,  # Fuel Pressure (Bar)
    
    # Extended Parameters (E-series)
    # These map to enhanced ECU parameters
    # Note: RealDash may not have direct equivalents for all E-parameters
    
    # Using custom target IDs for E-parameters (not standard RealDash)
    # These would need to be defined in custom RealDash XML
    1000: 'E31',  # IAM (Ignition Advance Multiplier)
    1001: 'E32',  # Engine Load (4-Byte)
    1002: 'E33',  # CL/OL Fueling
    1003: 'E34',  # Turbo Dynamics Integral
    1004: 'E35',  # Boost Error
    1005: 'E36',  # Target Boost
    1006: 'E37',  # Turbo Dynamics Proportional
    1007: 'E38',  # Throttle Plate Opening Angle
    1008: 'E39',  # Feedback Knock Correction
    1009: 'E40',  # Knock Correction Advance (IAM only)
    1010: 'E41',  # Fine Learning Knock Correction
    1011: 'E42',  # Map Ratio (Primary)
    1012: 'E44',  # A/F Learning #1 A
    1013: 'E45',  # A/F Learning #1 B
    1014: 'E46',  # A/F Learning #1 C
    1015: 'E47',  # A/F Learning #1 D
    1016: 'E48',  # A/F Learning #1 (4-byte)
    1017: 'E49',  # Idle Speed Map Selection
    1018: 'E50',  # Fuel Injector #1 Latency
    1019: 'E51',  # Manifold Absolute Pressure (4-byte)
    1020: 'E52',  # Manifold Relative Sea Level Pressure
    1021: 'E53',  # Ignition Base Timing
    1022: 'E54',  # Tip-in Throttle
    1023: 'E55',  # Tip-in Enrichment
    1024: 'E56',  # Requested Torque
    1025: 'E57',  # Target Throttle Plate Position
    1026: 'E58',  # Fine Learning Table Offset
    1027: 'E59',  # Gear (Calculated)
    1028: 'E60',  # Fuel Injector #1 Pulse Width (4-byte)
    1029: 'E61',  # A/F Learning Airflow Range
    1030: 'E70',  # Primary Wastegate Duty Maximum
    1031: 'E77',  # Primary Wastegate Duty Maximum (duplicate?)
    1032: 'E81',  # A/F Correction #1 (4-byte)
    1033: 'E84',  # Primary Open Loop Map Enrichment
    1034: 'E91',  # A/F Sensor #1 (4-byte)
    1035: 'E113', # Manifold Relative Pressure (4-byte)
    1036: 'E115', # Primary Enrichment Final
    1037: 'E116', # Knock Correction Advance Primary Map Ratio
    1038: 'E117', # Map Ratio (Alternate)
    1039: 'E118', # Knock Correction Advance Max Primary
    1040: 'E119', # Knock Correction Advance Additive
    1041: 'E121', # Closed Loop Fueling Target
    1042: 'E123', # Final Fueling Base
    
    # P-series Parameters
    1100: 'P3',   # A/F Correction #1
    1101: 'P4',   # A/F Learning #1
    1102: 'P15',  # Rear O2 Sensor
    1103: 'P18',  # Mass Airflow Sensor Voltage
    1104: 'P23',  # Knock Correction Advance
    1105: 'P24',  # Atmospheric Pressure
    1106: 'P25',  # Manifold Relative Pressure
    1107: 'P27',  # Fuel Tank Pressure
    1108: 'P30',  # Accelerator Pedal Angle
    1109: 'P31',  # Fuel Temperature
    1110: 'P35',  # Fuel Level
    1111: 'P36',  # Primary Wastegate Duty Cycle
    1112: 'P38',  # CPC Valve Duty Ratio
    1113: 'P39',  # Tumble Valve Position Sensor Right
    1114: 'P40',  # Tumble Valve Position Sensor Left
    1115: 'P47',  # Fuel Pump Duty
    1116: 'P48',  # Intake VVT Advance Angle Right
    1117: 'P49',  # Intake VVT Advance Angle Left
    1118: 'P50',  # Intake OCV Duty Right
    1119: 'P51',  # Intake OCV Duty Left
    1120: 'P52',  # Intake OCV Current Right
    1121: 'P53',  # Intake OCV Current Left
    1122: 'P54',  # A/F Sensor #1 Current
    1123: 'P56',  # A/F Sensor #1 Resistance
    1124: 'P58',  # A/F Sensor #1
    1125: 'P63',  # Roughness Monitor Cylinder #1
    1126: 'P64',  # Roughness Monitor Cylinder #2
    1127: 'P65',  # A/F Correction #3 (16-bit ECU)
    1128: 'P69',  # Roughness Monitor Cylinder #3
    1129: 'P70',  # Roughness Monitor Cylinder #4
    1130: 'P71',  # Throttle Motor Duty
    1131: 'P72',  # Throttle Motor Voltage
    1132: 'P73',  # Sub Throttle Sensor
    1133: 'P74',  # Main Throttle Sensor
    1134: 'P75',  # Sub Accelerator Sensor
    1135: 'P76',  # Main Accelerator Sensor
    1136: 'P82',  # Memorised Cruise Speed
    1137: 'P89',  # A/F Correction #3 (32-bit ECU)
    1138: 'P120', # Estimated odometer
    1139: 'P201', # Injector Duty Cycle
    1140: 'P202', # Manifold Relative Pressure (Corrected)
    1141: 'P203', # Fuel Consumption (Est.)
    1142: 'P239', # Global Timing User Adjustment Value
    1143: 'P240', # Engine Idle Speed User Adjustment (A/C off)
    1144: 'P241', # Engine Idle Speed User Adjustment (A/C on)
    
    # S-series Parameters (Switches/Status)
    1200: 'S1',   # AT Vehicle ID
    1201: 'S2',   # Test Mode Signal
    1202: 'S4',   # Neutral Position Switch
    1203: 'S5',   # Idle Switch
    1204: 'S7',   # Ignition Switch
    1205: 'S8',   # Power Steering Switch
    1206: 'S9',   # Air Conditioning Switch
    1207: 'S11',  # Starter Switch
    1208: 'S13',  # Rear O2 Sensor Rich Signal
    1209: 'S15',  # Knocking Signal #1
    1210: 'S18',  # Crankshaft Position Signal
    1211: 'S19',  # Camshaft Position Signal
    1212: 'S20',  # Rear Defogger Switch
    1213: 'S21',  # Blower Fan Switch
    1214: 'S22',  # Light Switch
    1215: 'S26',  # Air Conditioning Compressor Signal
    1216: 'S28',  # Radiator Fan Relay #1
    1217: 'S29',  # Radiator Fan Relay #2
    1218: 'S33',  # Blow-By Leak Connector
    1219: 'S34',  # Positive Crankcase Ventilation (PCV) Solenoid Valve
    1220: 'S35',  # Tumble Generator Valve (TGV) Output
    1221: 'S36',  # Tumble Generator Valve (TGV) Drive
    1222: 'S39',  # Ventilation Solenoid Valve
    1223: 'S42',  # Tank Sensor Control Valve
    1224: 'S62',  # Electronic Throttle Control (ETC) Motor Relay
    1225: 'S63',  # Clutch Switch
    1226: 'S64',  # Stop Light Switch
    1227: 'S65',  # Cruise Control Set/Coast Switch
    1228: 'S66',  # Cruise Control Resume/Accelerate Switch
    1229: 'S67',  # Brake Switch
    1230: 'S68',  # Cruise Control Main Toggle Switch
    1231: 'S133', # Cruise Control System Status
    1232: 'S157', # Oil Level Switch
}

# Reverse mapping: Parameter ID -> RealDash Target ID
PARAM_TO_REALDASH = {v: k for k, v in REALDASH_TARGET_MAP.items() if v is not None}

# Standard RealDash mappings (using official target IDs)
STANDARD_REALDASH_MAP = {
    12: 'P17',   # Battery Voltage
    14: 'P2',    # Coolant Temperature
    27: 'P11',   # Intake Air Temperature
    30: 'P12',   # MAF
    31: 'P7',    # Manifold Absolute Pressure
    35: 'P21',   # Pulse Width 1
    37: 'P8',    # RPM
    38: 'P10',   # Spark Advance
    42: 'P13',   # Throttle Position
    64: 'P9',    # Vehicle Speed
    100: 'P200', # Engine Load
    139: 'E59',  # Gear (Transmission)
}


def get_param_for_target(target_id):
    """Get parameter ID for a given RealDash target ID"""
    return REALDASH_TARGET_MAP.get(target_id)


def get_target_for_param(param_id):
    """Get RealDash target ID for a given parameter ID"""
    return PARAM_TO_REALDASH.get(param_id)


def is_standard_target(target_id):
    """Check if target ID is a standard RealDash target"""
    return target_id in STANDARD_REALDASH_MAP


if __name__ == '__main__':
    print("Standard RealDash Mappings:")
    print("-" * 60)
    for target_id in sorted(STANDARD_REALDASH_MAP.keys()):
        param_id = STANDARD_REALDASH_MAP[target_id]
        print(f"Target ID {target_id:3d} -> {param_id}")
    
    print("\n\nAll Parameter to Target Mappings:")
    print("-" * 60)
    for param_id in sorted(PARAM_TO_REALDASH.keys()):
        target_id = PARAM_TO_REALDASH[param_id]
        print(f"{param_id:5s} -> Target ID {target_id}")
