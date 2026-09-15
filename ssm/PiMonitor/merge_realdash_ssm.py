import csv
import os

def normalize(s):
    if not s: return ""
    return s.lower().replace(" ", "").replace("_", "").replace("-", "").replace("(", "").replace(")", "").replace("*", "").replace("/", "")

def main():
    # Determine the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = script_dir # Assuming script is in project root

    realdash_path = os.path.join(project_root, 'data', 'realdash.csv')
    ssm_path = os.path.join(project_root, '2005_STi_SSM_Parameters_Ranges.csv')
    output_path = os.path.join(project_root, 'data', 'merged_realdash_ssm.csv')

    print(f"Reading {realdash_path}...")
    with open(realdash_path, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        realdash_data = list(reader)
        realdash_fieldnames = reader.fieldnames

    print(f"Reading {ssm_path}...")
    with open(ssm_path, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        ssm_data = list(reader)
        ssm_fieldnames = reader.fieldnames

    # Manual mapping for common terms (Realdash Name -> SSM Name fragment)
    manual_map = {
        'rpm': 'enginespeed',
        'boosttargetkpa': 'targetboost',
        'engineload': 'engineload4byte', # Prefer E32
        'throttleposition': 'throttleopeningangle',
        'barometricpressure': 'atmosphericpressure',
        'mafg/s': 'massairflow',
        'sparkadvance': 'ignitiontotaltiming',
        'knockadvanceretard': 'feedbackknockcorrection', # E39
        'fueltrim1': 'afcorrection#1',
        'longtermfueltrim1': 'aflearning#1',
        'intakeairtemperature': 'intakeairtemperature',
        'coolanttemperature': 'coolanttemperature',
        'manifoldabsolutepressure': 'manifoldabsolutepressure4byte', # Prefer E51
        'vehiclespeed': 'vehiclespeed',
        'batteryvoltage': 'batteryvoltage',
        'fuellevel': 'fuellevel',
        'acceleratorpedalangle': 'acceleratorpedalangle',
        'fuelpumpduty': 'fuelpumpduty',
        'fueltemperature': 'fueltemperature',
        'wastegateduty': 'primarywastegatedutycycle',
        'injectordutycycle': 'injectordutycycle',
        'roughnessmonitorcylinder#1': 'roughnessmonitorcylinder#1',
        'roughnessmonitorcylinder#2': 'roughnessmonitorcylinder#2',
        'roughnessmonitorcylinder#3': 'roughnessmonitorcylinder#3',
        'roughnessmonitorcylinder#4': 'roughnessmonitorcylinder#4',
        'rear02sensor': 'rear02sensor',
        'massairflowsensorvoltage': 'massairflowsensorvoltage',
        'geartransmission': 'gearcalculated',
        'boosttarget': 'targetboost',
        'boosterror': 'boosterror',
        'feedbackknockcorrection': 'feedbackknockcorrection',
        'finelearningknockcorrection': 'finelearningknockcorrection',
        'iam': 'iam',
        'ignitiondwell': 'ignitiondwell', # No match likely
        'lambdatarget1': 'closedloopfuelingtarget', # E121
        'lambdatarget2': 'closedloopfuelingtarget',
        'fuelpressure': 'fuelpressure', # No match
        'oilpressure': 'oilpressure', # No match
        'oiltemperature': 'oiltemperature', # No match
        'pulsewidth1': 'fuelinjector#1pulsewidth',
        'dutycycle1': 'injectordutycycle',
        'boostbarpsi': 'manifoldrelativepressure',
    }

    # Prepare output fieldnames
    fieldnames = list(realdash_fieldnames)
    for f in ssm_fieldnames:
        if f not in fieldnames:
            fieldnames.append(f)
    
    merged_rows = []
    ssm_matched_indices = set()

    print("Merging...")
    for rd_row in realdash_data:
        rd_name = rd_row.get('Realdash Name', '')
        rd_name_norm = normalize(rd_name)
        
        best_match = None
        best_score = 0
        best_idx = -1

        # Try manual map
        target_ssm_name_norm = manual_map.get(rd_name_norm)

        for idx, ssm_row in enumerate(ssm_data):
            ssm_name = ssm_row.get('name', '')
            ssm_name_norm = normalize(ssm_name)
            ssm_monitor = ssm_row.get('Accessport Monitor', '')
            ssm_monitor_norm = normalize(ssm_monitor)

            score = 0
            
            # Exact match of normalized names
            if rd_name_norm == ssm_name_norm:
                score = 100
            elif rd_name_norm == ssm_monitor_norm:
                score = 95
            
            # Manual map match
            elif target_ssm_name_norm and (target_ssm_name_norm == ssm_name_norm or target_ssm_name_norm == ssm_monitor_norm):
                score = 90
            elif target_ssm_name_norm and target_ssm_name_norm in ssm_name_norm:
                score = 85
            
            # Fuzzy match: containment
            elif rd_name_norm and ssm_name_norm and (rd_name_norm in ssm_name_norm or ssm_name_norm in rd_name_norm):
                l1 = len(rd_name_norm)
                l2 = len(ssm_name_norm)
                if l1 > 0 and l2 > 0:
                    ratio = min(l1, l2) / max(l1, l2)
                    score = 50 + (ratio * 30) # 50 to 80

            if score > best_score:
                best_score = score
                best_match = ssm_row
                best_idx = idx

        new_row = rd_row.copy()
        if best_match and best_score > 60: # Threshold
            ssm_matched_indices.add(best_idx)
            # Merge SSM data
            for key in ssm_fieldnames:
                if key not in new_row or not new_row[key]:
                    new_row[key] = best_match[key]
        
        merged_rows.append(new_row)

    # Add unmatched SSM rows
    for idx, ssm_row in enumerate(ssm_data):
        if idx not in ssm_matched_indices:
            new_row = {k: '' for k in realdash_fieldnames}
            new_row.update(ssm_row)
            merged_rows.append(new_row)

    print(f"Writing to {output_path}...")
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(merged_rows)

    print(f"Done. Created {output_path} with {len(merged_rows)} rows.")

if __name__ == '__main__':
    main()