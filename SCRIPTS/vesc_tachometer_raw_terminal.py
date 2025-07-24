#!/usr/bin/env python3
"""
VESC Raw Tachometer Terminal Monitor
-----------------------------------
Reads CAN STATUS_5 messages from VESCs (IDs 28 and 46) and prints raw tachometer values to the terminal in real time.
Also saves the data to a CSV file.

How to launch:
    python3 vesc_tachometer_raw_terminal.py --interface socketcan --channel can0

What is it for:
    For debugging/calibration: see raw electrical revolutions (ticks) from VESCs in real time and log them.

Requirements:
    python-can
"""
import can
import struct
import argparse
import time
import csv

TACHO_CAN_IDS = {0x1B1C: 28, 0x1B2E: 46}

def parse_tachometer_message(msg):
    if msg.arbitration_id not in TACHO_CAN_IDS or len(msg.data) < 6:
        return None, None
    vesc_id = TACHO_CAN_IDS[msg.arbitration_id]
    tach_raw = struct.unpack('>h', msg.data[2:4])[0]
    return vesc_id, tach_raw

def main():
    parser = argparse.ArgumentParser(description='VESC Raw Tachometer Terminal Monitor')
    parser.add_argument('--interface', '-i', default='socketcan', help='CAN interface type (default: socketcan)')
    parser.add_argument('--channel', '-c', default='can0', help='CAN channel (default: can0)')
    parser.add_argument('--bitrate', '-b', type=int, default=500000, help='CAN bitrate (default: 500000)')
    parser.add_argument('--csv', default='vesc_tachometer_raw_log.csv', help='CSV file to save data (default: vesc_tachometer_raw_log.csv)')
    args = parser.parse_args()

    print("VESC Raw Tachometer Terminal Monitor")
    print(f"Interface: {args.interface} | Channel: {args.channel}")
    print("Format: [timestamp] VESC_ID RAW_TACH")
    print("-" * 40)

    bus = can.interface.Bus(channel=args.channel, interface=args.interface, bitrate=args.bitrate)
    with open(args.csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'vesc_id', 'tachometer_raw'])
        try:
            while True:
                msg = bus.recv(timeout=1.0)
                if msg is not None:
                    vesc_id, tach_raw = parse_tachometer_message(msg)
                    if vesc_id is not None:
                        t = time.time()
                        # Print to terminal
                        timestamp_str = time.strftime('%H:%M:%S', time.localtime(t)) + f'.{int((t%1)*1000):03d}'
                        print(f"[{t:.9f}] VESC_{vesc_id}: {tach_raw:8d}")
                        # Write to CSV
                        writer.writerow([f"{t:.9f}", vesc_id, tach_raw])
                        csvfile.flush()
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            bus.shutdown()

if __name__ == "__main__":
    main()
