""" SICKSense Agripollinate LiDAR Logger Version 0.1 
    Created by:    Josiah Faircloth 
    date:    10/31/2025

    Reads and parses raw LiDAR data from SICK TiM561 LiDAR scanner 
    and saves each scan's distance values to a timestamped text file.
"""

import socket
import json
import time
from datetime import datetime

HOST = "192.168.137.2"   # LiDAR IP
PORT = 2112             # Default data port

CMD_START = b'\x02sEN LMDscandata 1\x03'  # Start streaming
CMD_STOP = b'\x02sEN LMDscandata 0\x03'   # Stop streaming

def parse_scan(telegram):
    """Extract distance measurements from an LMDscandata telegram."""
    parts = telegram.strip().split(' ')
    if 'DIST1' not in parts:
        return None

    try:
        num_index = parts.index('DIST1') + 5
        num_values = int(parts[num_index], 16)
        distances = [int(v, 16) / 1000.0 for v in parts[num_index + 1:num_index + num_values]]
        return distances
    except Exception:
        return None

def main():
    timestamp = datetime.now().strftime("%m-%d-%Y_%H.%M.%S")
    out_file = f"lidar_log_{timestamp}.jsonl"

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(CMD_START)
        print("Connected to LiDAR, capturing data... (Ctrl+C to stop)")
        start_time = time.time()
        scan_count = 0

        with open(out_file, 'w') as f:
            try:
                buffer = ""
                while True:
                    data = s.recv(4096).decode(errors='ignore')
                    
                    buffer += data
                    while '\x03' in buffer:  # end of one telegram
                        telegram, buffer = buffer.split('\x03', 1)
                        telegram = telegram.replace('\x02', '').strip()
                        distances = parse_scan(telegram)
                        if distances:
                            scan = {
                                "timestamp": time.time(),
                                "num_points": len(distances),
                                "ranges": distances
                            }
                            f.write(json.dumps(scan) + "\n")
                            # print(f"Scan with {len(distances)} points logged.")
                            scan_count += 1

            except KeyboardInterrupt:
                print("\nStopping stream...")
                time_elapsed = time.time() - start_time
                s.sendall(CMD_STOP)
                print(f"Data saved to {out_file}")
                print(f"Time elapsed: {time_elapsed:.2f} seconds")
                print(f"Total scans logged: {scan_count}")
            except Exception as e:
                print(f"Error: {e}")
                time_elapsed = time.time() - start_time
                s.sendall(CMD_STOP)
                print(f"Data saved to {out_file}")
                print(f"Time elapsed: {time_elapsed:.2f} seconds")
                print(f"Total scans logged: {scan_count}")


if __name__ == "__main__":
    main()