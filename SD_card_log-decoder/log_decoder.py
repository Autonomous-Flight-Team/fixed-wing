import struct
import csv
import sys
import os
import pandas as pd

# Binary formats based on your C++ packed structs
FORMATS = {
    b"BARO": "<fffI",             
    b"IMU":  "<ffffffI",          
    b"GPS":  "<ddffI",            
    b"CTRL": "<ffffHHHH?I",       
    b"State_Vector": "<ddddddddddddI", 
    b"Pitot": "<BI",              
    b"Manual_Controller": "<hhhhHBI", 
}

# Column Headers for CSVs and Excel Tabs
HEADERS = {
    b"BARO": ["alt", "pres", "temp", "time"],
    b"IMU":  ["ax", "ay", "az", "gx", "gy", "gz", "time"],
    b"GPS":  ["lat", "lon", "alt", "vs", "time"],
    b"CTRL": ["ail", "ele", "rud", "thr", "ail_pwm", "ele_pwm", "rud_pwm", "thr_pwm", "ok", "time"],
    b"State_Vector": ["x","y","z","u","v","w","phi","theta","psi","p","q","r","time"],
    b"Manual_Controller": ["x", "y", "z", "r", "buttons", "target", "time"],
    b"Pitot": ["dummy", "time"]
}

def process_log(file_path):
    if not os.path.isfile(file_path):
        print(f"Error: The file '{file_path}' does not exist.")
        return

    base_name = os.path.splitext(file_path)[0]
    folder_path = os.path.dirname(file_path) or "."
    excel_output = f"{base_name}_Compiled.xlsx"
    
    with open(file_path, "rb") as f:
        content = f.read()

    extracted_files = []
    print(f"--- Phase 1: Decoding {file_path} to CSV ---")

    for tag, fmt in FORMATS.items():
        size = struct.calcsize(fmt)
        start = 0
        records = []
        
        while True:
            start = content.find(tag, start)
            if start == -1: break
            
            data_start = start + len(tag)
            data_end = data_start + size
            raw_data = content[data_start:data_end]
            
            if len(raw_data) == size:
                try:
                    records.append(struct.unpack(fmt, raw_data))
                except struct.error:
                    pass
            start = data_end 
        
        if records:
            csv_name = f"{base_name}_{tag.decode()}.csv"
            with open(csv_name, "w", newline='') as cf:
                writer = csv.writer(cf)
                writer.writerow(HEADERS[tag])
                writer.writerows(records)
            extracted_files.append((tag.decode(), csv_name))
            print(f" - Created {csv_name} ({len(records)} records)")

    if not extracted_files:
        print("No valid data found in file.")
        return

    print(f"\n--- Phase 2: Compiling into Excel ---")
    try:
        with pd.ExcelWriter(excel_output, engine='openpyxl') as writer:
            for tag_str, csv_path in extracted_files:
                df = pd.read_csv(csv_path)
                # Sheet names limited to 31 chars
                df.to_excel(writer, sheet_name=tag_str[:31], index=False)
        print(f"Success! Final workbook created: {excel_output}")
    except Exception as e:
        print(f"Excel compilation failed: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = input("Enter path to the .dat file: ").strip()

    if not path:
        print("Error: No file path provided. A valid file is required.")
    else:
        path = path.replace('"', '').replace("'", "")
        process_log(path)
