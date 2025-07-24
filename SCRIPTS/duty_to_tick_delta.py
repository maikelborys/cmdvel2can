#!/usr/bin/env python3
"""
Duty Cycle to Tick Delta Analyzer
--------------------------------
Matches each duty cycle command to the corresponding change in tachometer ticks for each VESC.
Outputs a CSV mapping duty_cycle, vesc_id, tick_delta, and time_delta.

How to launch:
    python3 duty_to_tick_delta.py

What is it for:
    For calibration/analysis: quantifies how many tachometer ticks result from each duty cycle step.

Inputs:
    - duty_cycle_tachometer_log.csv (from sweep script)
    - vesc_tachometer_raw_log.csv (from raw terminal script)
Outputs:
    - duty_to_tick_delta.csv

Requirements:
    Python standard library only (csv)
"""
import csv
from collections import defaultdict

DUTY_CSV = 'duty_cycle_tachometer_log.csv'
TACHO_CSV = 'vesc_tachometer_raw_log.csv'
OUTPUT_CSV = 'duty_to_tick_delta.csv'

def load_csv(filename, fields):
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        result = []
        for row in reader:
            d = {}
            for k, v in row.items():
                if k == 'timestamp':
                    d[k] = float(v)
                elif k == 'vesc_id':
                    d[k] = str(v)
                elif k in ('duty_cycle', 'tachometer_raw'):
                    try:
                        d[k] = float(v)
                    except ValueError:
                        d[k] = v
                else:
                    d[k] = v
            result.append(d)
        return result

def main():
    # Load duty and tacho logs
    duty_log = load_csv(DUTY_CSV, ['timestamp','vesc_id','duty_cycle'])
    tacho_log = load_csv(TACHO_CSV, ['timestamp','vesc_id','tachometer_raw'])

    # Organize tacho log by vesc_id
    tacho_by_vesc = defaultdict(list)
    for row in tacho_log:
        tacho_by_vesc[row['vesc_id']].append(row)

    # For each vesc_id, process duty steps
    results = []
    for vesc_id in set(row['vesc_id'] for row in duty_log):
        # Get all duty steps for this vesc, sorted by time
        steps = [row for row in duty_log if row['vesc_id'] == vesc_id]
        steps.sort(key=lambda r: r['timestamp'])
        tacho = tacho_by_vesc[vesc_id]
        tacho.sort(key=lambda r: r['timestamp'])
        for i in range(len(steps)-1):
            t0, t1 = steps[i]['timestamp'], steps[i+1]['timestamp']
            duty = steps[i]['duty_cycle']
            # Find first tacho at or after t0, and last before t1
            tacho_start = next((r for r in tacho if r['timestamp'] >= t0), None)
            tacho_end = next((r for r in reversed(tacho) if r['timestamp'] < t1), None)
            if tacho_start and tacho_end:
                tick0 = int(tacho_start['tachometer_raw'])
                tick1 = int(tacho_end['tachometer_raw'])
                delta = tick1 - tick0
                # Handle wraparound (16-bit signed)
                if delta > 32767:
                    delta -= 65536
                elif delta < -32768:
                    delta += 65536
                time_delta = tacho_end['timestamp'] - tacho_start['timestamp']
                results.append([duty, vesc_id, delta, time_delta])

    # Write output
    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['duty_cycle','vesc_id','tick_delta','time_delta'])
        for row in results:
            writer.writerow(row)
    print(f"Done. Output written to {OUTPUT_CSV}")

if __name__ == '__main__':
    main()
