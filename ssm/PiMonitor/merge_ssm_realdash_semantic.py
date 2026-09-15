#!/usr/bin/env python3
"""
Semantic SSM to RealDash Parameter Matcher

Matches 2005 Subaru STi SSM parameters to RealDash target IDs using
domain knowledge and semantic understanding of automotive ECU concepts.

Based on Cobb Tuning's official Subaru Monitor List documentation.
"""

import csv
import re
from dataclasses import dataclass, field
from typing import Optional
from pathlib import Path

# =============================================================================
# CONCEPT ONTOLOGY - Based on Cobb Tuning Monitor Descriptions
# =============================================================================

# Each concept defines:
# - keywords: terms that identify this concept in parameter names
# - unit_patterns: expected units for this concept
# - sub_concepts: more specific variants (e.g., "target" vs "actual")

CONCEPT_ONTOLOGY = {
    # ----- ENGINE SPEED -----
    "engine_speed": {
        "keywords": ["engine speed", "rpm", "revolutions"],
        "units": ["rpm", "RPM"],
        "description": "Engine crankshaft rotational speed"
    },
    
    # ----- BOOST / MANIFOLD PRESSURE -----
    "manifold_absolute_pressure": {
        "keywords": ["manifold absolute pressure", "map", "manifold abs"],
        "units": ["kPa", "kPa absolute", "psi", "bar"],
        "description": "Absolute pressure in intake manifold"
    },
    "manifold_relative_pressure": {
        "keywords": ["manifold relative pressure", "boost", "manifold rel"],
        "units": ["kPa", "kPa relative", "psi", "bar", "psi relative"],
        "description": "Relative pressure (MAP - barometric) = boost/vacuum"
    },
    "boost_target": {
        "keywords": ["boost target", "target boost"],
        "units": ["kPa", "psi", "bar"],
        "description": "ECU's target boost pressure"
    },
    "boost_error": {
        "keywords": ["boost error"],
        "units": ["kPa", "psi"],
        "description": "Difference between target and actual boost"
    },
    "barometric_pressure": {
        "keywords": ["atmospheric pressure", "barometric pressure", "baro"],
        "units": ["kPa", "psi", "bar"],
        "description": "Ambient atmospheric pressure"
    },
    
    # ----- AIR-FUEL RATIO -----
    "afr_measured": {
        "keywords": ["a/f sensor", "af sensor", "afr", "air/fuel ratio"],
        "units": ["AFR", "Lambda", "lambda", ""],
        "description": "Measured air-fuel ratio from O2 sensor"
    },
    "afr_target": {
        "keywords": ["afr target", "a/f target", "fuel target", "cl fuel target", 
                     "commanded fuel", "open loop fueling", "primary enrichment"],
        "units": ["AFR", "Lambda", "lambda", "estimated AFR"],
        "description": "Target air-fuel ratio commanded by ECU"
    },
    "lambda_measured": {
        "keywords": ["lambda"],
        "units": ["Lambda", "lambda", ""],
        "description": "Lambda value (AFR / stoich)"
    },
    "fuel_trim_short": {
        "keywords": ["a/f correction", "af correction", "fuel trim", "fuel correction"],
        "exclude": ["learning", "learned", "long term"],
        "units": ["%", "percent"],
        "description": "Short-term fuel trim based on O2 feedback"
    },
    "fuel_trim_long": {
        "keywords": ["a/f learning", "af learning", "long term fuel trim", "fuel learning"],
        "units": ["%", "percent"],
        "description": "Long-term learned fuel correction"
    },
    
    # ----- KNOCK / IGNITION CORRECTIONS -----
    "knock_feedback": {
        "keywords": ["feedback knock", "knock correction advance", "knock advance retard"],
        "units": ["deg", "degrees", "°"],
        "description": "Immediate timing retard from detected knock"
    },
    "knock_learned": {
        "keywords": ["fine learning knock", "fine knock learn", "learned knock"],
        "units": ["deg", "degrees"],
        "description": "Learned timing correction stored per load/RPM cell"
    },
    "iam_dam": {
        "keywords": ["iam", "dam", "dynamic advance multiplier", "ignition advance multiplier"],
        "units": ["multiplier", ""],
        "description": "Global knock learning multiplier (0-1 or 0-16)"
    },
    
    # ----- IGNITION TIMING -----
    "ignition_timing_total": {
        "keywords": ["ignition total timing", "ignition timing", "total timing", "spark advance"],
        "units": ["deg", "degrees", "°"],
        "description": "Total ignition timing for cylinder 1"
    },
    "ignition_timing_base": {
        "keywords": ["ignition base timing", "base timing", "primary ignition"],
        "units": ["deg", "degrees"],
        "description": "Base timing from ignition tables before corrections"
    },
    
    # ----- THROTTLE -----
    "throttle_position": {
        "keywords": ["throttle opening angle", "throttle position", "throttle plate", "tps"],
        "exclude": ["target", "pedal", "accelerator"],
        "units": ["%", "percent"],
        "description": "Actual throttle plate opening percentage"
    },
    "throttle_target": {
        "keywords": ["target throttle", "throttle target"],
        "units": ["%", "percent"],
        "description": "Target throttle plate position"
    },
    "accelerator_position": {
        "keywords": ["accelerator pedal", "accel pedal", "accelerator position"],
        "units": ["%", "percent"],
        "description": "Accelerator pedal position percentage"
    },
    
    # ----- TEMPERATURES -----
    "coolant_temp": {
        "keywords": ["coolant temperature", "coolant temp", "ect", "engine coolant"],
        "units": ["C", "Celsius", "°C", "F"],
        "description": "Engine coolant temperature"
    },
    "intake_air_temp": {
        "keywords": ["intake air temperature", "intake temp", "iat", "intake air temp"],
        "units": ["C", "Celsius", "°C"],
        "description": "Intake air temperature"
    },
    "fuel_temp": {
        "keywords": ["fuel temperature", "fuel temp"],
        "units": ["C", "Celsius", "°C"],
        "description": "Fuel temperature"
    },
    "oil_temp": {
        "keywords": ["oil temperature", "oil temp"],
        "exclude": ["transmission"],
        "units": ["C", "Celsius", "°C"],
        "description": "Engine oil temperature"
    },
    "exhaust_gas_temp": {
        "keywords": ["exhaust gas temperature", "egt", "exhaust temp"],
        "units": ["C", "Celsius", "°C"],
        "description": "Exhaust gas temperature"
    },
    
    # ----- AIRFLOW -----
    "mass_airflow": {
        "keywords": ["mass airflow", "maf", "mass air flow"],
        "exclude": ["voltage", "sensor voltage"],
        "units": ["g/s", "g/sec", "grams/sec"],
        "description": "Mass airflow in grams per second"
    },
    "maf_voltage": {
        "keywords": ["mass airflow sensor voltage", "maf voltage", "maf volts"],
        "units": ["V", "v", "volts"],
        "description": "MAF sensor output voltage"
    },
    
    # ----- ENGINE LOAD -----
    "engine_load": {
        "keywords": ["engine load", "calculated load"],
        "units": ["g/rev", "%", "percent"],
        "description": "Engine load (MAF*60/RPM) in grams per revolution"
    },
    
    # ----- FUEL INJECTION -----
    "injector_pulse_width": {
        "keywords": ["injector pulse width", "fuel injector pulse", "inj pulse", "ipw"],
        "units": ["ms", "milliseconds"],
        "description": "Fuel injector on-time per cycle"
    },
    "injector_duty_cycle": {
        "keywords": ["injector duty cycle", "inj duty", "idc"],
        "units": ["%", "percent"],
        "description": "Percentage of cycle time injector is open"
    },
    "injector_latency": {
        "keywords": ["injector latency", "inj latency", "dead time"],
        "units": ["ms", "milliseconds"],
        "description": "Injector opening delay compensation"
    },
    
    # ----- VEHICLE SPEED -----
    "vehicle_speed": {
        "keywords": ["vehicle speed", "vss"],
        "exclude": ["transmission"],
        "units": ["km/h", "KPH", "mph", "MPH"],
        "description": "Vehicle speed from speed sensor"
    },
    
    # ----- GEAR -----
    "gear_position": {
        "keywords": ["gear", "gear position", "gear calculated"],
        "exclude": ["transmission", "ratio"],
        "units": ["gear", "position", ""],
        "description": "Current gear position (estimated from RPM/speed)"
    },
    
    # ----- VVT / AVCS -----
    "vvt_intake_position": {
        "keywords": ["intake vvt", "avcs intake", "intake cam", "vvt advance angle"],
        "exclude": ["target", "exhaust"],
        "units": ["deg", "degrees"],
        "description": "Intake cam timing position"
    },
    "vvt_exhaust_position": {
        "keywords": ["exhaust vvt", "avcs exhaust", "exhaust cam"],
        "exclude": ["target"],
        "units": ["deg", "degrees"],
        "description": "Exhaust cam timing position"
    },
    "vvt_target": {
        "keywords": ["vvt target", "avcs target", "cam target"],
        "units": ["deg", "degrees"],
        "description": "Target cam timing"
    },
    "ocv_duty": {
        "keywords": ["ocv duty", "oil control valve duty", "intake ocv"],
        "units": ["%", "percent"],
        "description": "Oil control valve duty for VVT"
    },
    
    # ----- WASTEGATE / BOOST CONTROL -----
    "wastegate_duty": {
        "keywords": ["wastegate duty", "wg duty", "turbo control valve"],
        "units": ["%", "percent"],
        "description": "Wastegate solenoid duty cycle"
    },
    "turbo_dynamics": {
        "keywords": ["turbo dynamics", "td integral", "td proportional"],
        "units": ["%", "absolute %"],
        "description": "Turbo dynamics boost control correction"
    },
    
    # ----- BATTERY / ELECTRICAL -----
    "battery_voltage": {
        "keywords": ["battery voltage", "battery volts"],
        "units": ["V", "v", "volts"],
        "description": "Battery/charging system voltage"
    },
    
    # ----- O2 SENSORS -----
    "rear_o2_voltage": {
        "keywords": ["rear o2", "rear oxygen", "o2 sensor"],
        "units": ["V", "v", "volts"],
        "description": "Rear oxygen sensor voltage"
    },
    
    # ----- FUEL SYSTEM -----
    "fuel_pump_duty": {
        "keywords": ["fuel pump duty", "fuel pump"],
        "units": ["%", "percent"],
        "description": "Fuel pump duty cycle"
    },
    "fuel_pressure": {
        "keywords": ["fuel pressure", "fuel rail pressure"],
        "exclude": ["tank"],
        "units": ["bar", "psi", "kPa", "Bar"],
        "description": "Fuel rail pressure"
    },
    "fuel_tank_pressure": {
        "keywords": ["fuel tank pressure"],
        "units": ["kPa", "psi"],
        "description": "Fuel tank evaporative pressure"
    },
    "fuel_level": {
        "keywords": ["fuel level"],
        "units": ["V", "%", "volts"],
        "description": "Fuel level sensor reading"
    },
    
    # ----- CLOSED/OPEN LOOP -----
    "cl_ol_status": {
        "keywords": ["cl/ol", "closed loop", "open loop", "fuel system status"],
        "units": ["status", ""],
        "description": "Closed loop or open loop fueling mode"
    },
    
    # ----- SWITCHES / BINARY STATES -----
    "idle_switch": {
        "keywords": ["idle switch", "idle mode"],
        "units": ["", "On/Off"],
        "description": "Idle mode active state"
    },
    "neutral_switch": {
        "keywords": ["neutral position", "neutral switch"],
        "units": ["", "On/Off"],
        "description": "Transmission in neutral"
    },
    "clutch_switch": {
        "keywords": ["clutch switch"],
        "units": ["", "On/Off"],
        "description": "Clutch pedal pressed"
    },
    "brake_switch": {
        "keywords": ["brake switch", "stop light switch"],
        "units": ["", "On/Off"],
        "description": "Brake pedal pressed"
    },
    "ac_switch": {
        "keywords": ["air conditioning switch", "a/c switch", "ac switch", "ac compressor"],
        "units": ["", "On/Off"],
        "description": "A/C system state"
    },
    "ignition_switch": {
        "keywords": ["ignition switch"],
        "units": ["", "On/Off"],
        "description": "Ignition key position"
    },
    "knock_signal": {
        "keywords": ["knocking signal", "knock signal", "knock activity"],
        "units": ["", "On/Off"],
        "description": "Knock detected binary signal"
    },
    "cruise_control": {
        "keywords": ["cruise control", "cruise speed", "memorised cruise"],
        "units": ["km/h", "KPH", "", "On/Off"],
        "description": "Cruise control state or set speed"
    },
    
    # ----- MISFIRE / ROUGHNESS -----
    "misfire_count": {
        "keywords": ["roughness monitor", "misfire", "roughness cyl"],
        "units": ["count", "misfire count", ""],
        "description": "Cylinder misfire counter"
    },
    
    # ----- TGV -----
    "tgv_position": {
        "keywords": ["tumble valve", "tgv", "tumble generator"],
        "units": ["V", "volts", "", "On/Off"],
        "description": "Tumble generator valve position/state"
    },
    
    # ----- CPC / EVAP -----
    "cpc_duty": {
        "keywords": ["cpc valve", "canister purge", "cpc duty"],
        "units": ["%", "percent"],
        "description": "Canister purge control valve duty"
    },
    
    # ----- ENGINE RUNTIME -----
    "engine_runtime": {
        "keywords": ["engine run time", "engine runtime"],
        "units": ["s", "sec", "seconds", "hrs"],
        "description": "Engine running time counter"
    },
    
    # ----- ODOMETER -----
    "odometer": {
        "keywords": ["odometer", "estimated odometer"],
        "units": ["km", "Kilometers", "miles"],
        "description": "Vehicle odometer reading"
    },
    
    # ----- TIP-IN ENRICHMENT -----
    "tip_in_enrichment": {
        "keywords": ["tip-in", "tip in", "throttle tip"],
        "units": ["", "raw ecu value", "%"],
        "description": "Throttle tip-in fuel enrichment"
    },
    
    # ----- MAP RATIO -----
    "map_ratio": {
        "keywords": ["map ratio"],
        "exclude": ["manifold"],
        "units": ["multiplier", ""],
        "description": "ECU table blending ratio"
    },
}

# =============================================================================
# MANUAL SEMANTIC MAPPINGS - High confidence direct matches
# Based on Cobb documentation and RealDash parameter definitions
# =============================================================================

# Format: SSM_keyword_pattern -> (RealDash_Target_ID, confidence, reasoning)
MANUAL_MATCHES = {
    # Engine fundamentals
    ("P8", "Engine Speed"): (37, "high", "Both measure engine RPM from crankshaft sensor"),
    ("P9", "Vehicle Speed"): (64, "high", "Both measure vehicle speed from VSS"),
    ("P2", "Coolant Temperature"): (14, "high", "Both measure engine coolant temperature"),
    ("P11", "Intake Air Temperature"): (27, "high", "Both measure intake air temperature"),
    ("P12", "Mass Airflow"): (30, "high", "Both measure MAF in g/s"),
    ("P17", "Battery Voltage"): (12, "high", "Both measure battery/charging voltage"),
    
    # Pressure
    ("P7", "Manifold Absolute Pressure"): (31, "high", "Both measure MAP in kPa absolute"),
    ("E51", "Manifold Absolute Pressure"): (31, "high", "4-byte higher precision MAP"),
    ("P25", "Manifold Relative Pressure"): (83, "high", "Boost pressure = MAP - barometric"),
    ("E113", "Manifold Relative Pressure"): (83, "high", "4-byte higher precision boost"),
    ("P24", "Atmospheric Pressure"): (11, "high", "Both measure barometric pressure"),
    ("E36", "Target Boost"): (270, "high", "Both represent ECU boost target"),
    
    # Air-Fuel
    ("P58", "A/F Sensor #1"): (254, "high", "SSM outputs lambda, RealDash Lambda 1"),
    ("E91", "A/F Sensor #1"): (254, "high", "4-byte lambda from front O2"),
    ("P3", "A/F Correction #1"): (17, "high", "Short-term fuel trim percentage"),
    ("E81", "A/F Correction #1"): (17, "high", "4-byte short-term fuel trim"),
    ("P4", "A/F Learning #1"): (102, "high", "Long-term fuel trim stored correction"),
    ("E48", "A/F Learning #1"): (102, "high", "4-byte long-term fuel trim"),
    
    # Ignition/Knock
    ("P10", "Ignition Total Timing"): (38, "high", "Total ignition timing = Spark Advance"),
    ("E39", "Feedback Knock Correction"): (28, "high", "Immediate knock timing retard"),
    ("P23", "Knock Correction Advance"): (28, "medium", "Partial learned knock timing"),
    
    # Throttle
    ("P13", "Throttle Opening Angle"): (42, "high", "Throttle plate position percentage"),
    ("E38", "Throttle Plate Opening Angle"): (42, "high", "4-byte throttle position"),
    ("P30", "Accelerator Pedal Angle"): (42, "medium", "Pedal vs plate - use throttle slot"),
    
    # Load
    ("E32", "Engine Load"): (100, "high", "Both measure load in g/rev"),
    ("P200", "Engine Load (Calculated)"): (100, "high", "Calculated load = MAF*60/RPM"),
    
    # Injection
    ("P21", "Fuel Injector #1 Pulse Width"): (35, "high", "Injector on-time in ms"),
    ("E60", "Fuel Injector #1 Pulse Width"): (35, "high", "4-byte injector pulse width"),
    ("P201", "Injector Duty Cycle"): (119, "high", "IDC percentage"),
    
    # Gear
    ("E59", "Gear (Calculated)"): (200, "high", "Both estimate gear from RPM/speed"),
    
    # VVT/AVCS
    ("P48", "Intake VVT Advance Angle Right"): (492, "high", "Intake cam position bank 1"),
    ("P49", "Intake VVT Advance Angle Left"): (494, "high", "Intake cam position bank 2"),
    
    # Wastegate
    ("P36", "Primary Wastegate Duty Cycle"): (93, "medium", "No direct RealDash match - use Dummy 01"),
    
    # Fuel system
    ("P31", "Fuel Temperature"): (499, "high", "Both measure fuel temperature"),
    ("P47", "Fuel Pump Duty"): (93, "low", "No direct match - use Dummy"),
    
    # Switches
    ("S5", "Idle Switch"): (396, "high", "Enginebit: Idle"),
    ("S63", "Clutch Switch"): (231, "medium", "Clutch pedal position (binary as %)"),
    ("S67", "Brake Switch"): (230, "medium", "Brake pedal position (binary as %)"),
    ("S9", "Air Conditioning Switch"): (335, "high", "AC Active"),
    
    # Runtime
    ("P120", "Estimated odometer"): (310, "high", "Both are odometer readings"),
}

# =============================================================================
# REALDASH CONCEPT MAPPING - Maps RealDash params to concepts
# =============================================================================

REALDASH_CONCEPTS = {
    # Engine Speed
    37: "engine_speed",  # RPM
    
    # Pressure
    31: "manifold_absolute_pressure",  # Manifold Absolute Pressure
    11: "barometric_pressure",  # Barometric Pressure
    83: "manifold_relative_pressure",  # Boost (Bar/Psi) - calculated
    84: "manifold_relative_pressure",  # Boost (Psi)
    270: "boost_target",  # Boost Target (kPa)
    
    # Air-Fuel
    0: "afr_measured",  # AFR 1
    1: "afr_measured",  # AFR 2
    2: "afr_target",  # AFR Target 1
    3: "afr_target",  # AFR Target 2
    254: "lambda_measured",  # Lambda 1
    255: "lambda_measured",  # Lambda 2
    256: "afr_target",  # Lambda Target 1
    257: "afr_target",  # Lambda Target 2
    17: "fuel_trim_short",  # Fuel Trim 1
    18: "fuel_trim_short",  # Fuel Trim 2
    102: "fuel_trim_long",  # Long Term Fuel Trim 1
    104: "fuel_trim_long",  # Long Term Fuel Trim 2
    
    # Knock
    28: "knock_feedback",  # Knock Advance Retard
    29: "knock_feedback",  # Knock Percentage
    
    # Ignition
    38: "ignition_timing_total",  # Spark Advance
    
    # Throttle
    42: "throttle_position",  # Throttle Position
    
    # Temperatures
    14: "coolant_temp",  # Coolant Temperature
    27: "intake_air_temp",  # Intake Air Temperature
    499: "fuel_temp",  # Fuel Temperature
    152: "oil_temp",  # Engine Oil Temperature
    106: "exhaust_gas_temp",  # EGT1
    
    # Airflow
    30: "mass_airflow",  # MAF g/s
    
    # Load
    100: "engine_load",  # Engine Load
    
    # Injection
    35: "injector_pulse_width",  # Pulse Width 1
    36: "injector_pulse_width",  # Pulse Width 2
    119: "injector_duty_cycle",  # Duty Cycle 1
    120: "injector_duty_cycle",  # Duty Cycle 2
    
    # Speed
    64: "vehicle_speed",  # Vehicle Speed
    81: "vehicle_speed",  # VSS1
    
    # Gear
    200: "gear_position",  # Gear
    25: "gear_position",  # Gear (Master Speed RPM Gear Ratio)
    
    # VVT
    492: "vvt_intake_position",  # VVT: Intake Cam Position 1
    494: "vvt_intake_position",  # VVT: Intake Cam Position 2
    493: "vvt_exhaust_position",  # VVT: Exhaust Cam Position 1
    495: "vvt_exhaust_position",  # VVT: Exhaust Cam Position 2
    496: "vvt_target",  # VVT: Intake Cam Target Position
    497: "vvt_target",  # VVT: Exhaust Cam Target Position
    
    # Battery
    12: "battery_voltage",  # Battery Voltage
    
    # Fuel System
    202: "fuel_pressure",  # Fuel Pressure
    170: "fuel_level",  # Fuel Level
    
    # Switches/States
    396: "idle_switch",  # Enginebit: Idle
    335: "ac_switch",  # AC Active
    56: "cl_ol_status",  # Enginebit: Running
    57: "cl_ol_status",  # Enginebit: Cranking
    169: "cruise_control",  # Cruise Control Active
    171: "cruise_control",  # Cruise Control Set Value
    
    # Runtime
    33: "engine_runtime",  # Engine Runtime (sec)
    310: "odometer",  # Odometer
}


@dataclass
class SSMParameter:
    """Represents an SSM parameter from the 2005 STi."""
    id: str
    type: str
    unit: str
    name: str
    desc: str
    idle_expected: str
    cruise_expected: str
    wot_expected: str
    accessport_monitor: str
    
    # Matching results
    concept: Optional[str] = None
    sub_type: Optional[str] = None  # e.g., "left", "right", "#1", "#2"
    
    def normalized_name(self) -> str:
        """Normalize name for matching."""
        name = self.name.lower()
        # Remove common suffixes
        name = re.sub(r'\s*\([^)]*\)\s*', ' ', name)  # Remove parenthetical
        name = re.sub(r'\s*\*+\s*', '', name)  # Remove asterisks
        name = re.sub(r'\s*(4-byte|direct|high|low)\s*', ' ', name)
        name = name.strip()
        return name


@dataclass 
class RealDashParameter:
    """Represents a RealDash parameter."""
    category: str
    name: str
    units: str
    target_id: int
    
    # Matching
    concept: Optional[str] = None
    
    def normalized_name(self) -> str:
        """Normalize name for matching."""
        name = self.name.lower()
        name = re.sub(r'\s*\([^)]*\)\s*', ' ', name)
        name = name.strip()
        return name


@dataclass
class MatchResult:
    """Result of matching an SSM parameter to RealDash."""
    ssm: SSMParameter
    realdash: Optional[RealDashParameter]
    target_id: Optional[int]
    confidence: str  # "high", "medium", "low", "none"
    reasoning: str
    match_method: str  # "manual", "concept", "keyword", "unmatched"


def load_ssm_parameters(filepath: Path) -> list[SSMParameter]:
    """Load SSM parameters from CSV."""
    params = []
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            params.append(SSMParameter(
                id=row['id'],
                type=row['type'],
                unit=row['unit'],
                name=row['name'],
                desc=row['desc'],
                idle_expected=row['Idle Expected'],
                cruise_expected=row['Cruise Expected'],
                wot_expected=row['WOT Expected'],
                accessport_monitor=row['Accessport Monitor'],
            ))
    return params


def load_realdash_parameters(filepath: Path) -> list[RealDashParameter]:
    """Load RealDash parameters from CSV."""
    params = []
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            params.append(RealDashParameter(
                category=row['Category'],
                name=row['Realdash Name'],
                units=row['Units'],
                target_id=int(row['Target ID']),
            ))
    return params


def identify_concept(param_name: str, param_desc: str, param_unit: str) -> tuple[Optional[str], float]:
    """
    Identify the semantic concept for a parameter.
    Returns (concept_name, confidence_score).
    """
    text = f"{param_name} {param_desc}".lower()
    unit_lower = param_unit.lower() if param_unit else ""
    
    best_concept = None
    best_score = 0.0
    
    for concept_name, concept_def in CONCEPT_ONTOLOGY.items():
        score = 0.0
        
        # Check for exclusions first
        if 'exclude' in concept_def:
            if any(excl.lower() in text for excl in concept_def['exclude']):
                continue
        
        # Check keyword matches
        for keyword in concept_def['keywords']:
            if keyword.lower() in text:
                # Longer keywords are more specific = higher confidence
                keyword_score = len(keyword) / 20.0  # Normalize
                score = max(score, 0.5 + keyword_score)
        
        # Boost score if units match
        if score > 0 and 'units' in concept_def:
            for unit in concept_def['units']:
                if unit.lower() == unit_lower or unit.lower() in unit_lower:
                    score += 0.2
                    break
        
        if score > best_score:
            best_score = score
            best_concept = concept_name
    
    return best_concept, best_score


def identify_sub_type(param_name: str) -> Optional[str]:
    """Extract sub-type like #1, #2, Left, Right, A, B, C, D."""
    name = param_name.lower()
    
    # Check for numbered variants
    match = re.search(r'#(\d+)', param_name)
    if match:
        return f"#{match.group(1)}"
    
    # Check for left/right
    if 'left' in name:
        return 'left'
    if 'right' in name:
        return 'right'
    
    # Check for range letters (A, B, C, D for fuel learning)
    match = re.search(r'\s([A-D])\s*(\(|$)', param_name)
    if match:
        return f"range_{match.group(1)}"
    
    return None


def match_by_manual_mapping(ssm: SSMParameter, realdash_by_id: dict) -> Optional[MatchResult]:
    """Try to match using manual semantic mappings."""
    for (ssm_id_pattern, ssm_name_pattern), (target_id, confidence, reasoning) in MANUAL_MATCHES.items():
        # Check if SSM ID matches
        if ssm.id == ssm_id_pattern:
            rd = realdash_by_id.get(target_id)
            return MatchResult(
                ssm=ssm,
                realdash=rd,
                target_id=target_id,
                confidence=confidence,
                reasoning=reasoning,
                match_method="manual"
            )
        # Or if name pattern matches
        if ssm_name_pattern.lower() in ssm.name.lower() and ssm_id_pattern.startswith(ssm.id[0]):
            rd = realdash_by_id.get(target_id)
            return MatchResult(
                ssm=ssm,
                realdash=rd,
                target_id=target_id,
                confidence=confidence,
                reasoning=reasoning,
                match_method="manual"
            )
    return None


def match_by_concept(ssm: SSMParameter, realdash_list: list[RealDashParameter], 
                     realdash_by_id: dict) -> Optional[MatchResult]:
    """Match by semantic concept."""
    # Identify SSM concept
    concept, score = identify_concept(ssm.name, ssm.desc, ssm.unit)
    if not concept or score < 0.5:
        return None
    
    ssm.concept = concept
    sub_type = identify_sub_type(ssm.name)
    ssm.sub_type = sub_type
    
    # Find RealDash params with same concept
    candidates = []
    for target_id, rd_concept in REALDASH_CONCEPTS.items():
        if rd_concept == concept:
            rd = realdash_by_id.get(target_id)
            if rd:
                candidates.append(rd)
    
    if not candidates:
        return None
    
    # Pick best candidate based on sub-type matching
    best = candidates[0]
    if sub_type and len(candidates) > 1:
        for rd in candidates:
            rd_sub = identify_sub_type(rd.name)
            if rd_sub == sub_type:
                best = rd
                break
            # Also check for 1/2 vs left/right mapping
            if sub_type == 'right' and rd_sub == '#1':
                best = rd
            elif sub_type == 'left' and rd_sub == '#2':
                best = rd
    
    confidence = "high" if score > 0.7 else "medium"
    concept_desc = CONCEPT_ONTOLOGY[concept].get('description', concept)
    
    return MatchResult(
        ssm=ssm,
        realdash=best,
        target_id=best.target_id,
        confidence=confidence,
        reasoning=f"Concept match: {concept_desc}",
        match_method="concept"
    )


def match_by_keyword(ssm: SSMParameter, realdash_list: list[RealDashParameter]) -> Optional[MatchResult]:
    """Fallback keyword matching for remaining parameters."""
    ssm_name = ssm.normalized_name()
    ssm_words = set(ssm_name.split())
    
    best_match = None
    best_score = 0
    
    for rd in realdash_list:
        rd_name = rd.normalized_name()
        rd_words = set(rd_name.split())
        
        # Calculate word overlap
        common = ssm_words & rd_words
        if not common:
            continue
        
        # Score based on overlap ratio
        score = len(common) / max(len(ssm_words), len(rd_words))
        
        # Boost for significant words
        significant = {'temperature', 'pressure', 'voltage', 'speed', 'position', 
                      'duty', 'timing', 'fuel', 'intake', 'exhaust', 'boost'}
        if common & significant:
            score += 0.2
        
        if score > best_score and score > 0.4:
            best_score = score
            best_match = rd
    
    if best_match:
        confidence = "high" if best_score > 0.7 else "medium" if best_score > 0.5 else "low"
        return MatchResult(
            ssm=ssm,
            realdash=best_match,
            target_id=best_match.target_id,
            confidence=confidence,
            reasoning=f"Keyword overlap match (score: {best_score:.2f})",
            match_method="keyword"
        )
    
    return None


def assign_dummy_slot(ssm: SSMParameter, used_dummies: set[int]) -> int:
    """Assign an unused Dummy slot for unmatched SSM parameters."""
    # Dummy slots: 93-97 (Dummy 01-05), 121-135 (Dummy 06-20), 281-301 (Dummy 21-40)
    dummy_ids = list(range(93, 98)) + list(range(121, 136)) + list(range(281, 302))
    
    for dummy_id in dummy_ids:
        if dummy_id not in used_dummies:
            used_dummies.add(dummy_id)
            return dummy_id
    
    # Extended dummies
    extended = list(range(777, 788))  # Dummy 41-50
    for dummy_id in extended:
        if dummy_id not in used_dummies:
            used_dummies.add(dummy_id)
            return dummy_id
    
    return -1  # No more slots


def perform_matching(ssm_params: list[SSMParameter], 
                     realdash_params: list[RealDashParameter]) -> list[MatchResult]:
    """Perform semantic matching of all SSM parameters to RealDash."""
    
    # Build lookup
    realdash_by_id = {rd.target_id: rd for rd in realdash_params}
    
    results = []
    used_dummies = set()
    matched_target_ids = set()
    
    for ssm in ssm_params:
        # Try manual mapping first (highest confidence)
        result = match_by_manual_mapping(ssm, realdash_by_id)
        
        # Try concept matching
        if not result:
            result = match_by_concept(ssm, realdash_params, realdash_by_id)
        
        # Try keyword matching
        if not result:
            result = match_by_keyword(ssm, realdash_params)
        
        # No match - suggest dummy slot
        if not result:
            dummy_id = assign_dummy_slot(ssm, used_dummies)
            rd = realdash_by_id.get(dummy_id)
            result = MatchResult(
                ssm=ssm,
                realdash=rd,
                target_id=dummy_id if dummy_id > 0 else None,
                confidence="none",
                reasoning=f"No semantic match found - assigned Dummy slot",
                match_method="unmatched"
            )
        
        # Track used target IDs
        if result.target_id:
            if result.target_id in matched_target_ids and result.confidence != "none":
                result.reasoning += f" (WARNING: Target ID {result.target_id} already used)"
            matched_target_ids.add(result.target_id)
        
        results.append(result)
    
    return results


def write_merged_csv(results: list[MatchResult], realdash_params: list[RealDashParameter], 
                     output_path: Path):
    """Write merged CSV with all columns from both sources."""
    
    # Build lookup for unmatched RealDash params
    matched_rd_ids = {r.target_id for r in results if r.target_id}
    unmatched_rd = [rd for rd in realdash_params if rd.target_id not in matched_rd_ids]
    
    fieldnames = [
        # SSM columns
        'ssm_id', 'ssm_type', 'ssm_unit', 'ssm_name', 'ssm_desc',
        'idle_expected', 'cruise_expected', 'wot_expected', 'accessport_monitor',
        # RealDash columns  
        'realdash_category', 'realdash_name', 'realdash_units', 'target_id',
        # Match metadata
        'match_confidence', 'match_method', 'match_reasoning'
    ]
    
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        
        # Write matched SSM parameters (sorted by confidence)
        confidence_order = {'high': 0, 'medium': 1, 'low': 2, 'none': 3}
        sorted_results = sorted(results, key=lambda r: (confidence_order.get(r.confidence, 4), r.ssm.id))
        
        for result in sorted_results:
            row = {
                'ssm_id': result.ssm.id,
                'ssm_type': result.ssm.type,
                'ssm_unit': result.ssm.unit,
                'ssm_name': result.ssm.name,
                'ssm_desc': result.ssm.desc,
                'idle_expected': result.ssm.idle_expected,
                'cruise_expected': result.ssm.cruise_expected,
                'wot_expected': result.ssm.wot_expected,
                'accessport_monitor': result.ssm.accessport_monitor,
                'realdash_category': result.realdash.category if result.realdash else '',
                'realdash_name': result.realdash.name if result.realdash else '',
                'realdash_units': result.realdash.units if result.realdash else '',
                'target_id': result.target_id if result.target_id else '',
                'match_confidence': result.confidence,
                'match_method': result.match_method,
                'match_reasoning': result.reasoning,
            }
            writer.writerow(row)
        
        # Write unmatched RealDash parameters
        for rd in unmatched_rd:
            row = {
                'ssm_id': '',
                'ssm_type': '',
                'ssm_unit': '',
                'ssm_name': '',
                'ssm_desc': '',
                'idle_expected': '',
                'cruise_expected': '',
                'wot_expected': '',
                'accessport_monitor': '',
                'realdash_category': rd.category,
                'realdash_name': rd.name,
                'realdash_units': rd.units,
                'target_id': rd.target_id,
                'match_confidence': 'unmatched_realdash',
                'match_method': 'none',
                'match_reasoning': 'RealDash parameter with no SSM equivalent',
            }
            writer.writerow(row)


def print_summary(results: list[MatchResult]):
    """Print matching summary."""
    high = sum(1 for r in results if r.confidence == 'high')
    medium = sum(1 for r in results if r.confidence == 'medium')
    low = sum(1 for r in results if r.confidence == 'low')
    none = sum(1 for r in results if r.confidence == 'none')
    
    print("\n" + "="*60)
    print("SSM to RealDash Matching Summary")
    print("="*60)
    print(f"Total SSM parameters: {len(results)}")
    print(f"  High confidence:    {high} ({100*high/len(results):.1f}%)")
    print(f"  Medium confidence:  {medium} ({100*medium/len(results):.1f}%)")
    print(f"  Low confidence:     {low} ({100*low/len(results):.1f}%)")
    print(f"  No match (Dummy):   {none} ({100*none/len(results):.1f}%)")
    print()
    
    by_method = {}
    for r in results:
        by_method[r.match_method] = by_method.get(r.match_method, 0) + 1
    print("By match method:")
    for method, count in sorted(by_method.items()):
        print(f"  {method}: {count}")
    print()
    
    # Show some examples
    print("Sample high-confidence matches:")
    for r in results[:10]:
        if r.confidence == 'high':
            print(f"  {r.ssm.id} '{r.ssm.name}' -> {r.target_id} '{r.realdash.name if r.realdash else 'N/A'}'")
    
    print("\nUnmatched SSM parameters (assigned to Dummy slots):")
    for r in results:
        if r.confidence == 'none':
            print(f"  {r.ssm.id} '{r.ssm.name}' -> Dummy {r.target_id}")


def main():
    """Main entry point."""
    base_path = Path(__file__).parent
    
    ssm_path = base_path / "2005_STi_SSM_Parameters_Ranges.csv"
    realdash_path = base_path / "data" / "realdash.csv"
    output_path = base_path / "merged_ssm_realdash_2005_sti.csv"
    
    print(f"Loading SSM parameters from: {ssm_path}")
    ssm_params = load_ssm_parameters(ssm_path)
    print(f"  Loaded {len(ssm_params)} SSM parameters")
    
    print(f"Loading RealDash parameters from: {realdash_path}")
    realdash_params = load_realdash_parameters(realdash_path)
    print(f"  Loaded {len(realdash_params)} RealDash parameters")
    
    print("\nPerforming semantic matching...")
    results = perform_matching(ssm_params, realdash_params)
    
    print(f"\nWriting merged CSV to: {output_path}")
    write_merged_csv(results, realdash_params, output_path)
    
    print_summary(results)
    
    print(f"\nDone! Output written to: {output_path}")


if __name__ == "__main__":
    main()
