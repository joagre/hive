#!/usr/bin/env python3
"""
Analyze PID controller performance from telemetry data.

Computes metrics for altitude and position controllers:
- Overshoot percentage
- Rise time (10% to 90%)
- Settling time (within 5% of target)
- Steady-state error
- RMS tracking error
- Oscillation amplitude

Usage:
    python3 analyze_pid.py telemetry.csv
    python3 analyze_pid.py telemetry.csv --altitude
    python3 analyze_pid.py telemetry.csv --position
    python3 analyze_pid.py telemetry.csv --json
"""

import argparse
import csv
import json
import sys
import numpy as np


def load_telemetry(filename):
    """Load telemetry CSV file into numpy arrays."""
    with open(filename, 'r') as f:
        rows = list(csv.DictReader(f))

    if not rows:
        print(f"Error: No data in {filename}", file=sys.stderr)
        sys.exit(1)

    data = {}
    data['t'] = np.array([float(r['time_ms']) for r in rows]) / 1000
    data['altitude'] = np.array([float(r['altitude']) for r in rows])
    data['target_z'] = np.array([float(r['target_z']) for r in rows])
    data['x'] = np.array([float(r['x']) for r in rows])
    data['y'] = np.array([float(r['y']) for r in rows])
    data['target_x'] = np.array([float(r['target_x']) for r in rows])
    data['target_y'] = np.array([float(r['target_y']) for r in rows])
    data['vz'] = np.array([float(r['vz']) for r in rows])
    data['thrust'] = np.array([float(r['thrust']) for r in rows])

    return data


def analyze_step_response(t, actual, target, settle_pct=0.05):
    """
    Analyze step response characteristics.

    Returns dict with: overshoot, rise_time, settling_time, steady_state_error
    """
    results = {}

    # Find first significant target (ignore initial zeros)
    target_val = None
    for i, tgt in enumerate(target):
        if tgt > 0.1:
            target_val = tgt
            start_idx = i
            break

    if target_val is None:
        return {'error': 'No target found'}

    # Subset data from start of step
    t_sub = t[start_idx:] - t[start_idx]
    actual_sub = actual[start_idx:]
    target_sub = target[start_idx:]

    # Find where target is constant (first step)
    step_mask = target_sub == target_val
    if not np.any(step_mask):
        return {'error': 'No constant target period found'}

    # Analyze only the first step response
    step_end = np.where(~step_mask)[0]
    if len(step_end) > 0:
        end_idx = step_end[0]
    else:
        end_idx = len(t_sub)

    t_step = t_sub[:end_idx]
    actual_step = actual_sub[:end_idx]

    if len(t_step) < 10:
        return {'error': 'Insufficient data for analysis'}

    # Max value and overshoot
    max_val = np.max(actual_step)
    overshoot = (max_val - target_val) / target_val * 100
    results['overshoot_pct'] = round(overshoot, 2)
    results['max_value'] = round(max_val, 4)
    results['target'] = round(target_val, 4)

    # Rise time (10% to 90% of target)
    t10_idx = np.where(actual_step >= 0.1 * target_val)[0]
    t90_idx = np.where(actual_step >= 0.9 * target_val)[0]
    if len(t10_idx) > 0 and len(t90_idx) > 0:
        rise_time = t_step[t90_idx[0]] - t_step[t10_idx[0]]
        results['rise_time_s'] = round(rise_time, 3)
    else:
        results['rise_time_s'] = None

    # Settling time (within settle_pct of target)
    settle_band = target_val * settle_pct
    settled = np.abs(actual_step - target_val) < settle_band

    # Find last time it leaves the settle band
    if np.all(settled[-10:]):  # Check if settled at end
        # Walk backwards to find settling time
        for i in range(len(settled) - 1, -1, -1):
            if not settled[i]:
                results['settling_time_s'] = round(t_step[i + 1], 3)
                break
        else:
            results['settling_time_s'] = round(t_step[0], 3)
    else:
        results['settling_time_s'] = None  # Never settled

    # Steady-state error (average of last 20% of step)
    last_portion = int(len(actual_step) * 0.2)
    if last_portion > 5:
        ss_actual = np.mean(actual_step[-last_portion:])
        ss_error = target_val - ss_actual
        results['steady_state_error'] = round(ss_error, 4)
    else:
        results['steady_state_error'] = None

    # RMS error
    error = actual_step - target_val
    rms_error = np.sqrt(np.mean(error ** 2))
    results['rms_error'] = round(rms_error, 4)

    return results


def analyze_altitude(data, time_window=None):
    """Analyze altitude controller performance."""
    t, alt, target = data['t'], data['altitude'], data['target_z']

    if time_window:
        mask = (t >= time_window[0]) & (t <= time_window[1])
        t, alt, target = t[mask], alt[mask], target[mask]

    results = analyze_step_response(t, alt, target)
    results['controller'] = 'altitude'

    return results


def analyze_position(data, time_window=None):
    """Analyze position controller performance."""
    t = data['t']
    x, y = data['x'], data['y']
    target_x, target_y = data['target_x'], data['target_y']

    if time_window:
        mask = (t >= time_window[0]) & (t <= time_window[1])
        t = t[mask]
        x, y = x[mask], y[mask]
        target_x, target_y = target_x[mask], target_y[mask]

    results = {'controller': 'position'}

    # Analyze X response
    x_results = analyze_step_response(t, x, target_x)
    results['x_overshoot_pct'] = x_results.get('overshoot_pct')
    results['x_rise_time_s'] = x_results.get('rise_time_s')
    results['x_rms_error'] = x_results.get('rms_error')

    # Analyze Y response
    y_results = analyze_step_response(t, y, target_y)
    results['y_overshoot_pct'] = y_results.get('overshoot_pct')
    results['y_rise_time_s'] = y_results.get('rise_time_s')
    results['y_rms_error'] = y_results.get('rms_error')

    # Combined position metrics
    # Find where target_x is significant
    mask = target_x > 0.1
    if np.any(mask):
        pos_error = np.sqrt((x[mask] - target_x[mask])**2 +
                            (y[mask] - target_y[mask])**2)
        results['position_rms_error'] = round(np.sqrt(np.mean(pos_error**2)), 4)
        results['position_max_error'] = round(np.max(pos_error), 4)

        # Oscillation amplitude
        results['x_range'] = round(np.max(x[mask]) - np.min(x[mask]), 4)
        results['y_range'] = round(np.max(y[mask]) - np.min(y[mask]), 4)

    return results


def print_results(results, verbose=False):
    """Print analysis results in human-readable format."""
    ctrl = results.get('controller', 'unknown')
    print(f"\n{'='*50}")
    print(f"{ctrl.upper()} CONTROLLER ANALYSIS")
    print('='*50)

    if 'error' in results:
        print(f"Error: {results['error']}")
        return

    if ctrl == 'altitude':
        print(f"Target altitude:     {results.get('target', 'N/A')} m")
        print(f"Max altitude:        {results.get('max_value', 'N/A')} m")
        print(f"Overshoot:           {results.get('overshoot_pct', 'N/A')}%")
        print(f"Rise time (10-90%):  {results.get('rise_time_s', 'N/A')} s")
        print(f"Settling time (5%):  {results.get('settling_time_s', 'N/A')} s")
        print(f"Steady-state error:  {results.get('steady_state_error', 'N/A')} m")
        print(f"RMS error:           {results.get('rms_error', 'N/A')} m")

    elif ctrl == 'position':
        print("X-axis:")
        print(f"  Overshoot:         {results.get('x_overshoot_pct', 'N/A')}%")
        print(f"  Rise time:         {results.get('x_rise_time_s', 'N/A')} s")
        print(f"  RMS error:         {results.get('x_rms_error', 'N/A')} m")
        print(f"  Range:             {results.get('x_range', 'N/A')} m")
        print("Y-axis:")
        print(f"  Overshoot:         {results.get('y_overshoot_pct', 'N/A')}%")
        print(f"  Rise time:         {results.get('y_rise_time_s', 'N/A')} s")
        print(f"  RMS error:         {results.get('y_rms_error', 'N/A')} m")
        print(f"  Range:             {results.get('y_range', 'N/A')} m")
        print("Combined:")
        print(f"  Position RMS error: {results.get('position_rms_error', 'N/A')} m")
        print(f"  Position max error: {results.get('position_max_error', 'N/A')} m")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze PID controller performance from telemetry',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s telemetry.csv                  # Analyze all controllers
  %(prog)s telemetry.csv --altitude       # Only altitude analysis
  %(prog)s telemetry.csv --position       # Only position analysis
  %(prog)s telemetry.csv --time 0 10      # Analyze first 10 seconds
  %(prog)s telemetry.csv --json           # Output as JSON
""")

    parser.add_argument('csvfile', help='Telemetry CSV file')
    parser.add_argument('--altitude', action='store_true',
                        help='Analyze altitude controller only')
    parser.add_argument('--position', action='store_true',
                        help='Analyze position controller only')
    parser.add_argument('--time', nargs=2, type=float, metavar=('START', 'END'),
                        help='Time window to analyze (seconds)')
    parser.add_argument('--json', action='store_true',
                        help='Output results as JSON')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Verbose output')

    args = parser.parse_args()

    data = load_telemetry(args.csvfile)
    time_window = tuple(args.time) if args.time else None

    all_results = []

    # Determine what to analyze
    analyze_alt = args.altitude or (not args.altitude and not args.position)
    analyze_pos = args.position or (not args.altitude and not args.position)

    if analyze_alt:
        alt_results = analyze_altitude(data, time_window)
        all_results.append(alt_results)

    if analyze_pos:
        pos_results = analyze_position(data, time_window)
        all_results.append(pos_results)

    # Output results
    if args.json:
        print(json.dumps(all_results, indent=2))
    else:
        for results in all_results:
            print_results(results, args.verbose)
        print()


if __name__ == '__main__':
    main()
