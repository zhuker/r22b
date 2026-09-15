import csv
import difflib
import sys

def normalize(text):
    if not text:
        return ""
    text = str(text).strip().lower()
    # Common automotive abbreviations
    replacements = {
        "afr": "a/f",
        "temp": "temperature",
        "press": "pressure",
        "sens": "sensor",
        "corr": "correction",
        "deg": "degree",
        "inj": "injector",
        "man": "manifold",
        "abs": "absolute",
        "rel": "relative",
    }
    for old, new in replacements.items():
        # simple replace, might be risky if part of word, but usually okay for these keys
        text = text.replace(old, new)
    
    # Remove special chars that might confuse fuzzy match
    text = text.replace("*", "").replace("(", "").replace(")", "").replace("#", "")
    return text

def load_csv(filepath):
    with open(filepath, 'r', encoding='utf-8-sig') as f: # utf-8-sig to handle BOM if present
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames
    return rows, fieldnames

def main():
    realdash_path = 'data/realdash.csv'
    sti_path = '2005_STi_SSM_Parameters_Ranges.csv'
    output_path = 'merged_realdash_sti.csv'

    print(f"Loading {realdash_path}...")
    rd_rows, rd_headers = load_csv(realdash_path)
    print(f"Loaded {len(rd_rows)} rows from Realdash")

    print(f"Loading {sti_path}...")
    sti_rows, sti_headers = load_csv(sti_path)
    print(f"Loaded {len(sti_rows)} rows from STi Parameters")

    # Prepare headers for output
    unique_sti_headers = [h for h in sti_headers if h not in rd_headers]
    merged_fieldnames = rd_headers + unique_sti_headers + ['match_status', 'match_score', 'match_name', 'candidate_match_name', 'candidate_match_score']

    merged_data = []
    matched_sti_indices = set()

    for rd_row in rd_rows:
        rd_name = rd_row.get('Realdash Name', '')
        rd_name_norm = normalize(rd_name)

        best_match = None
        best_score = 0.0
        best_sti_row = None
        best_sti_index = -1
        match_source = ""

        # Threshold increased to 0.8 to avoid bad matches like Pressure -> Temp
        threshold = 0.80

        for i, sti_row in enumerate(sti_rows):
            sti_name = sti_row.get('name', '')
            ap_monitor = sti_row.get('Accessport Monitor', '')
            
            # Additional logic: id match? No, IDs are different schemas.
            
            sti_name_norm = normalize(sti_name)
            ap_monitor_norm = normalize(ap_monitor)
            
            score = 0.0
            source = ""

            if rd_name_norm == sti_name_norm:
                score = 1.0
                source = "exact_name"
            elif rd_name_norm == ap_monitor_norm:
                score = 1.0
                source = "exact_ap"
            else:
                sim_name = difflib.SequenceMatcher(None, rd_name_norm, sti_name_norm).ratio()
                sim_ap = difflib.SequenceMatcher(None, rd_name_norm, ap_monitor_norm).ratio()
                
                if sim_name > sim_ap:
                    score = sim_name
                    source = "fuzzy_name"
                else:
                    score = sim_ap
                    source = "fuzzy_ap"
            
            if score > best_score:
                best_score = score
                best_sti_row = sti_row
                best_match = sti_name
                best_sti_index = i
                match_source = source

        new_row = rd_row.copy()
        
        # Always record the best candidate
        new_row['candidate_match_name'] = best_match if best_match else ""
        new_row['candidate_match_score'] = f"{best_score:.2f}"

        if best_score >= threshold and best_sti_row:
            matched_sti_indices.add(best_sti_index)
            for h in unique_sti_headers:
                new_row[h] = best_sti_row.get(h, '')
            new_row['match_status'] = "MATCHED"
            new_row['match_score'] = f"{best_score:.2f}"
            new_row['match_name'] = best_match
        else:
            for h in unique_sti_headers:
                new_row[h] = ''
            new_row['match_status'] = "UNMATCHED"
            new_row['match_score'] = ""
            new_row['match_name'] = ""

        merged_data.append(new_row)

    print(f"Matched {len(matched_sti_indices)} STi rows out of {len(sti_rows)}.")
    
    for i, sti_row in enumerate(sti_rows):
        if i not in matched_sti_indices:
            new_row = {k: '' for k in merged_fieldnames}
            # Fill shared columns if they exist in source_headers, but primary key is different?
            # Actually we just fill what we can.
            for k, v in sti_row.items():
                if k in merged_fieldnames:
                    new_row[k] = v
            
            new_row['match_status'] = "UNMATCHED_STI_SOURCE"
            new_row['match_score'] = ""
            new_row['candidate_match_name'] = ""
            new_row['candidate_match_score'] = ""
            merged_data.append(new_row)

    print(f"Writing output to {output_path}...")
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=merged_fieldnames)
        writer.writeheader()
        writer.writerows(merged_data)
    
    print("Done.")

if __name__ == "__main__":
    main()
