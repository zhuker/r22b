
# Mapping from supported_parameters.csv IDs to RealDash IDs
# Format: 'SUPPORTED_PARAM_ID': REALDASH_ID

REALDASH_MAPPING = {
    'E31': 93,  # E31: IAM (4-byte)* -> No direct match found, using Dummy 01
    'E32': 100,  # E32: Engine Load (4-Byte)* -> Engine Load (100)
    'E33': 395,  # E33: CL/OL Fueling* -> Enginebit: Closed Loop Active (395) - partial match
    'E34': 94,  # E34: Turbo Dynamics Integral (4-byte)* -> No direct match, using Dummy 02
    'E35': 95,  # E35: Boost Error* -> No direct match, using Dummy 03
    'E36': 270,  # E36: Target Boost (4-byte)* -> Boost Target (kPa) (270)
    'E37': 96,  # E37: Turbo Dynamics Proportional (4-byte)* -> No direct match, using Dummy 04
    'E38': 42,  # E38: Throttle Plate Opening Angle (4-byte)* -> Throttle Position (42)
    'E39': 28,  # E39: Feedback Knock Correction (4-byte)* -> Knock Advance Retard (28) - best guess
    'E40': 28,  # E40: Knock Correction Advance (IAM only)* -> Knock Advance Retard (28) - best guess
    'E41': 29,  # E41: Fine Learning Knock Correction (4-byte)* -> Knock Percentage (29) - best guess
    'E42': 97,  # E42: Map Ratio (Primary)* -> No direct match, using Dummy 05
    'E44': 102,  # E44: A/F Learning #1 A (Stored)* -> Long Term Fuel Trim 1 (102) - best guess
    'E45': 102,  # E45: A/F Learning #1 B (Stored)* -> Long Term Fuel Trim 1 (102) - best guess
    'E46': 102,  # E46: A/F Learning #1 C (Stored)* -> Long Term Fuel Trim 1 (102) - best guess
    'E47': 102,  # E47: A/F Learning #1 D (Stored)* -> Long Term Fuel Trim 1 (102) - best guess
    'E48': 102,  # E48: A/F Learning #1 (4-byte)* -> Long Term Fuel Trim 1 (102)
    'E49': 121,  # E49: Idle Speed Map Selection* -> No direct match, using Dummy 06
    'E50': 122,  # E50: Fuel Injector #1 Latency (4-byte)* -> No direct match, using Dummy 07
    'E51': 31,  # E51: Manifold Absolute Pressure (4-byte)* -> Manifold Absolute Pressure (31)
    'E52': 83,  # E52: Manifold Relative Sea Level Pressure (4-byte)* -> Boost (Bar/Psi) (83) - best guess
    'E53': 38,  # E53: Ignition Base Timing* -> Spark Advance (38) - best guess
    'E54': 41,  # E54: Tip-in Throttle* -> TPS dot (41) - best guess
    'E55': 4,  # E55: Tip-in Enrichment (Last Calculated)* -> Accel Enrich (4)
    'E56': 207,  # E56: Requested Torque* -> Torque (207) - best guess
    'E57': 42,  # E57: Target Throttle Plate Position* -> Throttle Position (42) - best guess
    'E58': 123,  # E58: Fine Learning Table Offset* -> No direct match, using Dummy 08
    'E59': 200,  # E59: Gear (Calculated)* -> Gear (200)
    'E60': 35,  # E60: Fuel Injector #1 Pulse Width (4-byte)* -> Pulse Width 1 (35)
    'E61': 124,  # E61: A/F Learning Airflow Range (Current)* -> No direct match, using Dummy 09
    'E70': 125,  # E70: Primary Wastegate Duty Maximum* (4-byte)* -> No direct match, using Dummy 10
    'E77': 126,  # E77: Primary Wastegate Duty Maximum* (4-byte)* -> No direct match, using Dummy 11
    'E81': 17,  # E81: A/F Correction #1 (4-byte)* -> Fuel Trim 1 (17)
    'E84': 127,  # E84: Primary Open Loop Map Enrichment (4-byte)* -> No direct match, using Dummy 12
    'E91': 0,  # E91: A/F Sensor #1 (4-byte)* -> AFR 1 (0)
    'E113': 83,  # E113: Manifold Relative Pressure (4-byte)* -> Boost (Bar/Psi) (83)
    'E115': 128,  # E115: Primary Enrichment Final (4-byte)* -> No direct match, using Dummy 13
    'E116': 129,  # E116: Knock Correction Advance Primary Map Ratio* -> No direct match, using Dummy 14
    'E117': 130,  # E117: Map Ratio (Alternate)* -> No direct match, using Dummy 15
    'E118': 131,  # E118: Knock Correction Advance Max Primary* -> No direct match, using Dummy 16
    'E119': 132,  # E119: Knock Correction Advance Additive* -> No direct match, using Dummy 17
    'E121': 2,  # E121: Closed Loop Fueling Target (4-byte)* -> AFR Target 1 (2)
    'E123': 43,  # E123: Final Fueling Base (4-byte)* -> Total Fuel Correction (43) - best guess
    'P2': 14,  # P2: Coolant Temperature -> Coolant Temperature (14)
    'P3': 17,  # P3: A/F Correction #1 -> Fuel Trim 1 (17)
    'P4': 102,  # P4: A/F Learning #1 -> Long Term Fuel Trim 1 (102)
    'P7': 31,  # P7: Manifold Absolute Pressure -> Manifold Absolute Pressure (31)
    'P8': 37,  # P8: Engine Speed -> RPM (37)
    'P9': 64,  # P9: Vehicle Speed -> Vehicle Speed (64)
    'P10': 38,  # P10: Ignition Total Timing -> Spark Advance (38)
    'P11': 27,  # P11: Intake Air Temperature -> Intake Air Temperature (27)
    'P12': 30,  # P12: Mass Airflow -> MAF g/s (30)
    'P13': 42,  # P13: Throttle Opening Angle -> Throttle Position (42)
    'P15': 255,  # P15: Rear O2 Sensor -> Lambda 2 (255) - best guess
    'P17': 12,  # P17: Battery Voltage -> Battery Voltage (12)
    'P18': 133,  # P18: Mass Airflow Sensor Voltage -> No direct match, using Dummy 18
    'P21': 35,  # P21: Fuel Injector #1 Pulse Width -> Pulse Width 1 (35)
    'P23': 28,  # P23: Knock Correction Advance -> Knock Advance Retard (28)
    'P24': 11,  # P24: Atmospheric Pressure -> Barometric Pressure (11)
    'P25': 83,  # P25: Manifold Relative Pressure -> Boost (Bar/Psi) (83)
    'P27': 202,  # P27: Fuel Tank Pressure -> Fuel Pressure (202) - best guess
    'P30': 42,  # P30: Accelerator Pedal Angle -> Throttle Position (42) - best guess
    'P31': 499,  # P31: Fuel Temperature -> Fuel Temperature (499)
    'P35': 170,  # P35: Fuel Level -> Fuel Level (170)
    'P36': 134,  # P36: Primary Wastegate Duty Cycle -> No direct match, using Dummy 19
    'P38': 135,  # P38: CPC Valve Duty Ratio -> No direct match, using Dummy 20
    'P39': 281,  # P39: Tumble Valve Position Sensor Right -> No direct match, using Dummy 21
    'P40': 282,  # P40: Tumble Valve Position Sensor Left -> No direct match, using Dummy 22
    'P47': 283,  # P47: Fuel Pump Duty -> No direct match, using Dummy 23
    'P48': 492,  # P48: Intake VVT Advance Angle Right -> VVT: Intake Cam Position 1 (492)
    'P49': 494,  # P49: Intake VVT Advance Angle Left -> VVT: Intake Cam Position 2 (494)
    'P50': 284,  # P50: Intake OCV Duty Right -> No direct match, using Dummy 24
    'P51': 285,  # P51: Intake OCV Duty Left -> No direct match, using Dummy 25
    'P52': 286,  # P52: Intake OCV Current Right -> No direct match, using Dummy 26
    'P53': 287,  # P53: Intake OCV Current Left -> No direct match, using Dummy 27
    'P54': 288,  # P54: A/F Sensor #1 Current -> No direct match, using Dummy 28
    'P56': 289,  # P56: A/F Sensor #1 Resistance -> No direct match, using Dummy 29
    'P58': 0,  # P58: A/F Sensor #1 -> AFR 1 (0)
    'P63': 290,  # P63: Roughness Monitor Cylinder #1 -> No direct match, using Dummy 30
    'P64': 291,  # P64: Roughness Monitor Cylinder #2 -> No direct match, using Dummy 31
    'P65': 18,  # P65: A/F Correction #3 (16-bit ECU) -> Fuel Trim 2 (18) - best guess
    'P69': 292,  # P69: Roughness Monitor Cylinder #3 -> No direct match, using Dummy 32
    'P70': 293,  # P70: Roughness Monitor Cylinder #4 -> No direct match, using Dummy 33
    'P71': 294,  # P71: Throttle Motor Duty -> No direct match, using Dummy 34
    'P72': 295,  # P72: Throttle Motor Voltage -> No direct match, using Dummy 35
    'P73': 296,  # P73: Sub Throttle Sensor -> No direct match, using Dummy 36
    'P74': 297,  # P74: Main Throttle Sensor -> No direct match, using Dummy 37
    'P75': 298,  # P75: Sub Accelerator Sensor -> No direct match, using Dummy 38
    'P76': 299,  # P76: Main Accelerator Sensor -> No direct match, using Dummy 39
    'P82': 171,  # P82: Memorised Cruise Speed -> Cruise Control Set Value (171)
    'P89': 18,  # P89: A/F Correction #3 (32-bit ECU) -> Fuel Trim 2 (18) - best guess
    'P120': 310,  # P120: Estimated odometer -> Odometer (310)
    'P200': 100,  # P200: Engine Load (Calculated) -> Engine Load (100)
    'P201': 119,  # P201: Injector Duty Cycle -> Duty Cycle 1 (119)
    'P202': 83,  # P202: Manifold Relative Pressure (Corrected) -> Boost (Bar/Psi) (83)
    'P203': 203,  # P203: Fuel Consumption (Est.) -> Fuel Consumption (Instant) (203)
    'P239': 301,  # P239: Global Timing User Adjustment Value -> No direct match, using Dummy 40
    'P240': 777,  # P240: Engine Idle Speed User Adjustment (A/C off) -> No direct match, using Dummy 41
    'P241': 778,  # P241: Engine Idle Speed User Adjustment (A/C on) -> No direct match, using Dummy 42
    'S1': 779,  # S1: AT Vehicle ID -> No direct match, using Dummy 43
    'S2': 781,  # S2: Test Mode Signal -> No direct match, using Dummy 44
    'S4': 489,  # S4: Neutral Position Switch -> Neutral Safety Switch (489)
    'S5': 396,  # S5: Idle Switch -> Enginebit: Idle (396)
    'S7': 167,  # S7: Ignition Switch -> Ignition On (167)
    'S8': 782,  # S8: Power Steering Switch -> No direct match, using Dummy 45
    'S9': 335,  # S9: Air Conditioning Switch -> AC Active (335)
    'S11': 154,  # S11: Starter Switch -> Starter Motor (154)
    'S13': 783,  # S13: Rear O2 Sensor Rich Signal -> No direct match, using Dummy 46
    'S15': 784,  # S15: Knocking Signal #1 -> No direct match, using Dummy 47
    'S18': 785,  # S18: Crankshaft Position Signal -> No direct match, using Dummy 48
    'S19': 786,  # S19: Camshaft Position Signal -> No direct match, using Dummy 49
    'S20': 808,  # S20: Rear Defogger Switch -> Rear Defrost Active (808)
    'S21': 172,  # S21: Blower Fan Switch -> Heater Motor Level (172) - best guess
    'S22': 155,  # S22: Light Switch -> Parking Lights (155) - best guess
    'S26': 335,  # S26: Air Conditioning Compressor Signal -> AC Active (335)
    'S28': 153,  # S28: Radiator Fan Relay #1 -> Engine Cooling Fan (153)
    'S29': 153,  # S29: Radiator Fan Relay #2 -> Engine Cooling Fan (153)
    'S33': 787,  # S33: Blow-By Leak Connector -> No direct match, using Dummy 50
    'S34': 788,  # S34: Positive Crankcase Ventilation (PCV) Solenoid Valve -> No direct match, using Dash Data 01
    'S35': 789,  # S35: Tumble Generator Valve (TGV) Output -> No direct match, using Dash Data 02
    'S36': 790,  # S36: Tumble Generator Valve (TGV) Drive -> No direct match, using Dash Data 03
    'S39': 791,  # S39: Ventilation Solenoid Valve -> No direct match, using Dash Data 04
    'S42': 792,  # S42: Tank Sensor Control Valve -> No direct match, using Dash Data 05
    'S62': 793,  # S62: Electronic Throttle Control (ETC) Motor Relay -> No direct match, using Dash Data 06
    'S63': 231,  # S63: Clutch Switch -> Clutch pedal position (231) - best guess
    'S64': 230,  # S64: Stop Light Switch -> Brake pedal position (230) - best guess
    'S65': 169,  # S65: Cruise Control Set/Coast Switch -> Cruise Control Active (169)
    'S66': 735,  # S66: Cruise Control Resume/Accelerate Switch -> Cruise Control Accelerate (735)
    'S67': 230,  # S67: Brake Switch -> Brake pedal position (230)
    'S68': 169,  # S68: Cruise Control Main Toggle Switch -> Cruise Control Active (169)
    'S133': 169,  # S133: Cruise Control System Status -> Cruise Control Active (169)
    'S157': 180,  # S157: Oil Level Switch -> Engine Oil Level (180) - best guess
}
