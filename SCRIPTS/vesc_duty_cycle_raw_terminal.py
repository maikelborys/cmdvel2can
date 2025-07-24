#!/usr/bin/env python3
"""
VESC Raw Duty Cycle Terminal Monitor (STATUS_1)
----------------------------------------------
Reads CAN STATUS_1 messages from VESCs (IDs 28 and 46) and prints duty cycle to the terminal in real time.
Also saves the data to a CSV file.

How to launch:
    python3 vesc_duty_cycle_raw_terminal.py --interface socketcan --channel can0

What is it for:
    For debugging/calibration: see raw duty cycle values reported by VESCs in real time and log them.

Requirements:
    python-can
"""
import can
import struct
import argparse
import time
import csv

STATUS1_CAN_IDS = {0x091C: 28, 0x092E: 46}

def parse_status1_message(msg):
    if msg.arbitration_id not in STATUS1_CAN_IDS or len(msg.data) < 8:
        return None, None
    vesc_id = STATUS1_CAN_IDS[msg.arbitration_id]
    duty_raw = struct.unpack('>h', msg.data[6:8])[0]
    duty_cycle = duty_raw / 1000.0
    return vesc_id, duty_cycle

def main():
    parser = argparse.ArgumentParser(description='VESC Raw Duty Cycle Terminal Monitor (STATUS_1)')
    parser.add_argument('--interface', '-i', default='socketcan', help='CAN interface type (default: socketcan)')
    parser.add_argument('--channel', '-c', default='can0', help='CAN channel (default: can0)')
    parser.add_argument('--bitrate', '-b', type=int, default=500000, help='CAN bitrate (default: 500000)')
    parser.add_argument('--csv', default='vesc_duty_cycle_raw_log.csv', help='CSV file to save data (default: vesc_duty_cycle_raw_log.csv)')
    args = parser.parse_args()

    print("VESC Raw Duty Cycle Terminal Monitor (STATUS_1)")
    print(f"Interface: {args.interface} | Channel: {args.channel}")
    print("Format: [timestamp] VESC_ID DUTY_CYCLE")
    print("-" * 40)

    bus = can.interface.Bus(channel=args.channel, interface=args.interface, bitrate=args.bitrate)
    with open(args.csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'vesc_id', 'duty_cycle'])
        try:
            while True:
                msg = bus.recv(timeout=1.0)
                if msg is not None:
                    vesc_id, duty_cycle = parse_status1_message(msg)
                    if vesc_id is not None:
                        t = time.time()
                        # Print to terminal
                        print(f"[{t:.9f}] VESC_{vesc_id}: {duty_cycle:8.4f}")
                        # Write to CSV
                        writer.writerow([f"{t:.9f}", vesc_id, f"{duty_cycle:.4f}"])
                        csvfile.flush()
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            bus.shutdown()

if __name__ == "__main__":
    main()
