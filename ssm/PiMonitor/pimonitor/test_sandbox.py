import os
import pickle
import re
import unittest
import csv

from pimonitor.PMPacket import PMPacket
from pimonitor.cu.PMCUContext import PMCUContext
from pimonitor.cu.PMCUParameter import PMCUParameter
from pimonitor.cu.PMCUStandardParameter import PMCUStandardParameter
import pandas


def load_params():
    print(os.path.abspath(os.path.curdir))
    serializedDataFile = open("data/data_v370.pkl", "rb")
    defined_parameters = pickle.load(serializedDataFile)
    serializedDataFile.close()
    return defined_parameters


def cu_type_str(cu_type):
    if cu_type == PMCUParameter.CU_TYPE_STD_PARAMETER():
        return "Standard"
    if cu_type == PMCUParameter.CU_TYPE_FIXED_ADDRESS_PARAMETER():
        return "Fixed Addr"
    if cu_type == PMCUParameter.CU_TYPE_SWITCH_PARAMETER():
        return "Switch"
    if cu_type == PMCUParameter.CU_TYPE_CALCULATED_PARAMETER():
        return "Calculated"
    return f"Unknown({cu_type})"


class MyTestCase(unittest.TestCase):
    def test_merge_csv(self):
        import merge_realdash_ssm
        print("Running merge_realdash_ssm.main()...")
        merge_realdash_ssm.main()
        print("Merge complete.")

    def test_ranges(self):
        import pandas as pd
        import numpy as np

        # Read the file
        df = pd.read_csv('supported_parameters.csv')

        # Initialize new columns
        df['Idle Expected'] = ''
        df['Cruise Expected'] = ''
        df['WOT Expected'] = ''

        def get_ranges(row):
            name = row['name'].lower()
            unit = str(row['unit']).lower()

            idle = "N/A"
            cruise = "N/A"
            wot = "N/A"

            # IAM
            if "iam" in name:
                idle = "1.0 (Ideal)"
                cruise = "1.0"
                wot = "1.0"

            # Engine Load
            elif "engine load" in name:
                idle = "0.15 - 0.40 g/rev"
                cruise = "0.30 - 1.00 g/rev"
                wot = "2.0 - 3.5 g/rev"

            # Fuel System Status / CL/OL
            elif "cl/ol" in name or "fuel system status" in name:
                idle = "8 (Closed Loop)"
                cruise = "8 (Closed Loop)"
                wot = "10 (Open Loop)"

            # Boost / Manifold Pressure
            elif "manifold relative pressure" in name or "boost" in name:
                if "error" in name:
                    idle = "N/A"
                    cruise = "N/A"
                    wot = "~0 psi"  # Ideal
                elif "target" in name:
                    idle = "N/A"
                    cruise = "N/A"
                    wot = "~14.5 psi (Stock)"
                else:  # Actual pressure
                    idle = "-9 to -8 psi (Vacuum)"
                    cruise = "Vacuum (< 0 psi)"
                    wot = "~14.5 psi (Peak)"

            # Manifold Absolute Pressure
            elif "manifold absolute pressure" in name:
                idle = "30 - 50 kPa"
                cruise = "40 - 90 kPa"
                wot = "200 - 230 kPa (Stock)"

            # Throttle
            elif "throttle" in name and "opening" in name:
                idle = "0 - 5 %"
                cruise = "10 - 40 %"
                wot = "100 %"
            elif "accelerator" in name:
                idle = "0 %"
                cruise = "10 - 40 %"
                wot = "100 %"

            # Knock Correction
            elif "knock" in name and ("correction" in name or "learning" in name):
                idle = "0 deg"
                cruise = "0 deg (occasional -1.4 normal)"
                wot = "0 deg (<-2.8 is bad)"

            # A/F Learning / Correction
            elif "a/f learning" in name or "a/f correction" in name:
                idle = "+/- 10%"
                cruise = "+/- 10%"
                wot = "0% (Correction) / +/-5% (Learning)"

            # AFR / A/F Sensor
            elif "a/f sensor" in name and "ratio" in name or ("afr" in unit):
                idle = "~14.7"
                cruise = "~14.7"
                wot = "11.0 - 11.5"  # Richer

            # RPM
            elif "engine speed" in name:
                idle = "700 - 800 rpm"
                cruise = "2000 - 3500 rpm"
                wot = "Redline (~7000 rpm)"

            # Injector Duty Cycle
            elif "injector duty" in name:
                idle = "1 - 2 %"
                cruise = "5 - 15 %"
                wot = "< 90 % (Safe limit)"

            # Vehicle Speed
            elif "vehicle speed" in name:
                idle = "0"
                cruise = "Constant"
                wot = "Increasing"

            # Temperatures
            elif "coolant" in name:
                idle = "80 - 96 C"
                cruise = "80 - 96 C"
                wot = "80 - 96 C"
            elif "intake air" in name:
                idle = "Ambient + Heatsoak"
                cruise = "~Ambient"
                wot = "~Ambient"

            # Battery
            elif "battery" in name:
                idle = "13.5 - 14.5 V"
                cruise = "13.5 - 14.5 V"
                wot = "13.5 - 14.5 V"

            # MAF
            elif "mass airflow" in name:
                if "voltage" in name:
                    idle = "1.0 - 1.4 V"
                    cruise = "1.5 - 2.5 V"
                    wot = "4.0 - 4.7 V"
                else:  # g/s
                    idle = "2 - 6 g/s"
                    cruise = "10 - 60 g/s"
                    wot = "200+ g/s"

            # Roughness (Misfire)
            elif "roughness" in name:
                idle = "0"
                cruise = "0"
                wot = "0"

            # Switches
            elif "switch" in name or "signal" in name:
                if "idle" in name:
                    idle = "1 (On)"
                    cruise = "0 (Off)"
                    wot = "0 (Off)"
                elif "neutral" in name:
                    idle = "1 (On)"
                    cruise = "0 (Off)"
                    wot = "0 (Off)"
                elif "starter" in name:
                    idle = "0"
                    cruise = "0"
                    wot = "0"
                elif "air conditioning" in name:
                    idle = "User Def"
                    cruise = "User Def"
                    wot = "0 (Often cuts at WOT)"
                else:
                    idle = "State Dep."
                    cruise = "State Dep."
                    wot = "State Dep."

            # TGV
            elif "tumble" in name or "tgv" in name:
                # TGV usually opens after warm up
                idle = "Open"
                cruise = "Open"
                wot = "Open"

            # OCV / VVT (AVCS)
            elif "vvt" in name or "ocv" in name or "intake advance" in name:
                idle = "0 deg"
                cruise = "0 - 20 deg (Load dep)"
                wot = "High Advance (RPM dep)"

            # Wastegate
            elif "wastegate" in name:
                idle = "0 %"
                cruise = "0 - 10 %"
                wot = "40 - 70 % (Map dep)"

            # Default for unknown
            else:
                idle = "-"
                cruise = "-"
                wot = "-"

            return pd.Series([idle, cruise, wot])

        # Apply the function
        df[['Idle Expected', 'Cruise Expected', 'WOT Expected']] = df.apply(get_ranges, axis=1)

        # Save to CSV
        output_filename = '2005_STi_SSM_Parameters_Ranges.csv'
        df.to_csv(output_filename, index=False)

        print(df[['name', 'Idle Expected', 'Cruise Expected', 'WOT Expected']].head(20))
        print(f"File saved as {output_filename}")
    def test_something(self):
        ecu_data = [255, 162, 16, 17, 61, 18, 89, 64, 6, 115, 250, 203, 166, 43, 129, 254, 168, 0, 130, 0, 96, 206, 84,
                    248, 177, 228, 128, 0, 0, 0, 0, 0, 0, 0, 220, 0, 0, 117, 30, 48, 192, 240, 34, 0, 0, 67, 251, 0,
                    241, 0, 0, 0, 0, 0, 0, 0, 240]
        ecu_dst = 240
        ecu_src = 16
        ecu_packet = PMPacket(ecu_dst, ecu_src, ecu_data)
        print(ecu_packet)

        defined_parameters = load_params()

        ecu_context = PMCUContext(ecu_packet, [1, 3])
        ecu_parameters = ecu_context.match_parameters(defined_parameters)
        ecu_switch_parameters = ecu_context.match_switch_parameters(defined_parameters)
        ecu_calculated_parameters = ecu_context.match_calculated_parameters(defined_parameters, ecu_parameters)

        # tcu_context = PMCUContext(tcu_packet, [2])
        # tcu_parameters = tcu_context.match_parameters(defined_parameters)
        # tcu_switch_parameters = tcu_context.match_switch_parameters(defined_parameters)
        # tcu_calculated_parameters = tcu_context.match_calculated_parameters(defined_parameters, tcu_parameters)

        print("ECU ROM ID: " + ecu_context.get_rom_id())

        # print("TCU ROM ID: " + tcu_context.get_rom_id())

        def stringSplitByNumbers(x):
            r = re.compile('(\\d+)')
            l = r.split(x.get_id())
            return [int(y) if y.isdigit() else y for y in l]

        supported_parameters = ecu_parameters + ecu_switch_parameters + ecu_calculated_parameters  # + tcu_parameters + tcu_switch_parameters + tcu_calculated_parameters
        supported_parameters = sorted(supported_parameters, key=stringSplitByNumbers)

        records = []
        with open("supported_parameters.csv", "w") as f:
            for p in supported_parameters:
                if isinstance(p, PMCUStandardParameter):
                    record = {"id": p.get_id(), "type": cu_type_str(p.get_cu_type()), "unit": p.get_default_unit(),
                              "name": p.get_name(), "desc": p._desc}
                    records.append(record)
                    print(p.get_id(), cu_type_str(p.get_cu_type()), p.get_default_unit(), p.get_name(), p._desc)
                else:
                    print(p)
        pandas.DataFrame(records).to_csv("supported_parameters.csv", index=False)


if __name__ == '__main__':
    unittest.main()
