#!/usr/bin/env python3
"""
VESC Duty Cycle Sweep Logger
---------------------------
Sends a sweep of duty cycle commands to VESCs (IDs 28 and 46) and logs each command with timestamp to a CSV file.

How to launch:
    python3 duty_cycle_tachometer_sweep.py

What is it for:
    For calibration: generates a sequence of duty cycle commands for mapping to velocity/tachometer ticks.

Requirements:
    python-can
"""
import can
import struct
import time
import csv
import threading

# --- CONFIGURATION ---
CAN_CHANNEL = 'can0'
CAN_INTERFACE = 'socketcan'
BITRATE = 500000
VESC_IDS = [28, 46]
DUTY_MIN = -0.20
DUTY_MAX = 0.20
DUTY_STEP = 0.005
STEP_PERIOD = 0.2  # seconds per step
CSV_FILE = 'duty_cycle_tachometer_log.csv'

# CAN ID for duty cycle command (VESC protocol, extended frame)
def duty_cycle_can_id(vesc_id):
    return (vesc_id | 0x80000000)

# CAN IDs for STATUS_5 (tachometer) messages
TACHO_CAN_IDS = {0x1B1C: 28, 0x1B2E: 46}

def build_duty_cycle_frame(vesc_id, duty):
    can_id = duty_cycle_can_id(vesc_id)
    scaled = int(duty * 100000)
    data = [(scaled >> 24) & 0xFF, (scaled >> 16) & 0xFF, (scaled >> 8) & 0xFF, scaled & 0xFF] + [0]*4
    return can.Message(arbitration_id=can_id, data=bytearray(data), is_extended_id=True)

def parse_tachometer_message(msg):
    if msg.arbitration_id not in TACHO_CAN_IDS or len(msg.data) < 6:
        return None
    vesc_id = TACHO_CAN_IDS[msg.arbitration_id]
    tach_raw = struct.unpack('>h', msg.data[2:4])[0]
    voltage_raw = struct.unpack('>h', msg.data[4:6])[0]
    return {
        'vesc_id': vesc_id,
        'tachometer_raw': tach_raw,
        'tachometer_mech': tach_raw // 6,
        'voltage': voltage_raw / 10.0
    }

class DutyLogger:
    def __init__(self, filename):
        self.csv = open(filename, 'w', newline='')
        self.writer = csv.writer(self.csv)
        self.writer.writerow(['timestamp', 'vesc_id', 'duty_cycle'])
        self.lock = threading.Lock()
    def log(self, timestamp, vesc_id, duty):
        with self.lock:
            self.writer.writerow([timestamp, vesc_id, duty])
            self.csv.flush()
    def close(self):
        self.csv.close()

def sweep_and_log(bus, logger, latest_duty):
    duty = 0.0
    phase = 'up'  # 'up', 'down', 'return'
    print("Starting duty cycle sweep...")
    running = True
    while running:
        for vesc_id in VESC_IDS:
            msg = build_duty_cycle_frame(vesc_id, duty)
            bus.send(msg)
            latest_duty[vesc_id] = duty
            now = time.time()
            logger.log(now, vesc_id, duty)
        print(f"[{time.strftime('%H:%M:%S', time.localtime(now))}] Sent duty: {duty:.3f}")
        time.sleep(STEP_PERIOD)
        if phase == 'up':
            duty += DUTY_STEP
            if duty > DUTY_MAX:
                duty = DUTY_MAX
                phase = 'down'
        elif phase == 'down':
            duty -= DUTY_STEP
            if duty < DUTY_MIN:
                duty = DUTY_MIN
                phase = 'return'
        elif phase == 'return':
            duty += DUTY_STEP
            if duty >= 0.0:
                duty = 0.0
                running = False
    # Send zero at end
    for vesc_id in VESC_IDS:
        msg = build_duty_cycle_frame(vesc_id, 0.0)
        bus.send(msg)
        latest_duty[vesc_id] = 0.0
        now = time.time()
        logger.log(now, vesc_id, 0.0)
    print("Sweep complete.")

def tachometer_listener(bus, logger, latest_duty):
    print("Listening for VESC STATUS_5 (tachometer) messages...")
    while True:
        msg = bus.recv(timeout=1.0)
        if msg is None:
            continue
        t = time.time()
        tacho = parse_tachometer_message(msg)
        if tacho:
            vesc_id = tacho['vesc_id']
            duty = latest_duty.get(vesc_id, '')
            logger.log(t, vesc_id, duty, tacho['tachometer_raw'], tacho['tachometer_mech'], tacho['voltage'])

def main():
    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)
    logger = DutyLogger(CSV_FILE)
    latest_duty = {}  # {vesc_id: last_duty_cycle}
    try:
        sweep_and_log(bus, logger, latest_duty)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        logger.close()
        bus.shutdown()
        print("Log saved to", CSV_FILE)

if __name__ == '__main__':
    main()