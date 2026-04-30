import struct
import csv

# Configuration: Define the binary formats
# < = little-endian
# I = uint32 (timestamp), f = float, d = double, h = int16, H = uint16, B = uint8, ? = bool

FORMATS = {
    b"BARO": "<fffI",             # BaroData_t: alt, press, temp + timestamp
    b"IMU":  "<ffffffI",          # IMUData_t: ax,ay,az, gx,gy,gz + timestamp
    b"GPS":  "<ddffI",            # GPSData_t: lat, lon, alt, vs + timestamp
    b"CTRL": "<ffffHHHH?I",       # ControlOutput_t: 4 floats, 4 uint16, 1 bool + timestamp
    b"State_Vector": "<ddddddddddddI", # StateVector_t: 12 doubles + timestamp
    b"Pitot": "<BI",              # PitotData_t: 1 dummy byte + timestamp
    # MAVLink Manual Control (Common v1/v2): 4 int16, 1 uint16, 1 uint8 + timestamp
    b"Manual_Controller": "<hhhhHB I", 
}

HEADERS = {
    b"BARO": ["alt", "pres", "temp", "time"],
    b"IMU":  ["ax", "ay", "az", "gx", "gy", "gz", "time"],
    b"GPS":  ["lat", "lon", "alt", "vs", "time"],
    b"CTRL": ["ail", "ele", "rud", "thr", "ail_pwm", "ele_pwm", "rud_pwm", "thr_pwm", "ok", "time"],
    b"State_Vector": ["x","y","z","u","v","w","phi","theta","psi","p","q","r","time"],
    b"Manual_Controller": ["x", "y", "z", "r", "buttons", "target", "time"],
}

def decode_log(input_file, output_prefix):
    with open(input_file, "rb") as f:
        content = f.read()

    files = {tag: open(f"{output_prefix}_{tag.decode()}.csv", "w", newline='') for tag in HEADERS}
    writers = {tag: csv.writer(files[tag]) for tag in HEADERS}
    
    for tag in HEADERS:
        writers[tag].writerow(HEADERS[tag])

    for tag, fmt in FORMATS.items():
        size = struct.calcsize(fmt)
        start = 0
        while True:
            start = content.find(tag, start)
            if start == -1: break
            
            data_start = start + len(tag)
            data_end = data_start + size
            
            raw_data = content[data_start:data_end]
            if len(raw_data) == size:
                try:
                    decoded = struct.unpack(fmt, raw_data)
                    if tag in writers:
                        writers[tag].writerow(decoded)
                except struct.error:
                    pass # Skip corrupted packets
            
            start = data_end 

    for f in files.values(): f.close()
    print("Decoding complete. Separate CSVs created for all sensors.")

if __name__ == "__main__":
    decode_log("LOG_00-00-00.dat", "flight_data")
