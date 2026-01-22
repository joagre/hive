#!/usr/bin/env python3
"""
Debug flight telemetry - show timeline and detect issues.

Displays flight progress at regular intervals and identifies problems:
- Failed takeoff (never reached target altitude)
- Flip/crash (excessive tilt angle)
- Position drift
- Thrust saturation

Usage:
    python3 flight_debug.py /tmp/pilot_telemetry.csv
    python3 flight_debug.py telemetry.csv --interval 1.0
    python3 flight_debug.py telemetry.csv --time 0 10
    python3 flight_debug.py telemetry.csv --verbose
"""

import argparse
import csv
import sys
from typing import List


def load_telemetry(filepath: str) -> List[dict]:
    """Load telemetry CSV file."""
    data = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({
                'time_ms': int(row['time_ms']),
                'altitude': float(row['altitude']),
                'vz': float(row['vz']),
                'x': float(row['x']),
                'y': float(row['y']),
                'roll': float(row['roll']),
                'pitch': float(row['pitch']),
                'thrust': float(row['thrust']),
                'target_z': float(row['target_z']),
                'target_x': float(row.get('target_x', 0)),
                'target_y': float(row.get('target_y', 0)),
            })
    return data


def analyze_flight(data: List[dict]) -> dict:
    """Analyze flight and detect issues."""
    if not data:
        return {'error': 'No data'}

    max_alt = max(d['altitude'] for d in data)
    max_alt_t = next(d['time_ms'] for d in data if d['altitude'] == max_alt)

    max_roll = max(abs(d['roll']) for d in data)
    max_pitch = max(abs(d['pitch']) for d in data)
    max_tilt_deg = max(
        (d['roll']**2 + d['pitch']**2)**0.5 * 57.3 for d in data
    )

    # Find when tilt exceeded 45 degrees (crash)
    crash_time = None
    for d in data:
        tilt = (d['roll']**2 + d['pitch']**2)**0.5 * 57.3
        if tilt > 45:
            crash_time = d['time_ms'] / 1000
            break

    # Check thrust saturation
    saturated = sum(1 for d in data if d['thrust'] >= 0.99 or d['thrust'] <= 0.01)
    saturation_pct = 100 * saturated / len(data)

    # Position at end vs start
    x_drift = data[-1]['x'] - data[0]['x']
    y_drift = data[-1]['y'] - data[0]['y']

    # Target altitude (first non-zero target)
    target_alt = next((d['target_z'] for d in data if d['target_z'] > 0.1), 1.0)

    # Did we reach target?
    reached_target = max_alt >= target_alt * 0.8

    issues = []
    if not reached_target:
        issues.append(f"Failed takeoff: max alt {max_alt:.2f}m < target {target_alt:.1f}m")
    if crash_time:
        issues.append(f"Crash detected at t={crash_time:.1f}s (tilt > 45deg)")
    if saturation_pct > 20:
        issues.append(f"Thrust saturation: {saturation_pct:.0f}% of flight")
    if abs(x_drift) > 2 or abs(y_drift) > 2:
        issues.append(f"Large drift: ({x_drift:.1f}, {y_drift:.1f})m")

    return {
        'duration_s': data[-1]['time_ms'] / 1000,
        'max_altitude': max_alt,
        'max_altitude_t': max_alt_t / 1000,
        'target_altitude': target_alt,
        'max_tilt_deg': max_tilt_deg,
        'crash_time': crash_time,
        'thrust_saturation_pct': saturation_pct,
        'x_drift': x_drift,
        'y_drift': y_drift,
        'issues': issues,
    }


def print_timeline(data: List[dict], interval: float = 2.0):
    """Print flight timeline at regular intervals."""
    print("\nFlight timeline:")
    print(f"{'Time':>6} {'Alt':>6} {'Vz':>6} {'X':>6} {'Y':>6} {'Roll':>6} {'Thrust':>6}")
    print("-" * 50)

    interval_ms = int(interval * 1000)
    tolerance = 50  # ms

    for d in data:
        if d['time_ms'] % interval_ms < tolerance:
            roll_deg = d['roll'] * 57.3
            print(f"{d['time_ms']/1000:5.1f}s {d['altitude']:6.2f} {d['vz']:6.2f} "
                  f"{d['x']:6.2f} {d['y']:6.2f} {roll_deg:6.1f} {d['thrust']:6.2f}")


def print_stable_periods(data: List[dict], target_alt: float = 1.0):
    """Find and print stable hover periods."""
    print(f"\nStable hover periods (alt {target_alt-0.15:.1f}-{target_alt+0.15:.1f}m, tilt < 15deg):")

    stable_start = None
    stable_samples = []

    for d in data:
        tilt_deg = (d['roll']**2 + d['pitch']**2)**0.5 * 57.3
        in_stable = (
            abs(d['altitude'] - target_alt) < 0.15 and
            tilt_deg < 15 and
            d['thrust'] > 0.1
        )

        if in_stable:
            if stable_start is None:
                stable_start = d['time_ms']
            stable_samples.append(d)
        else:
            if stable_start is not None and len(stable_samples) > 25:  # > 1 second
                duration = (stable_samples[-1]['time_ms'] - stable_start) / 1000
                alt_mean = sum(s['altitude'] for s in stable_samples) / len(stable_samples)
                print(f"  t={stable_start/1000:.1f}-{stable_samples[-1]['time_ms']/1000:.1f}s "
                      f"({duration:.1f}s, mean alt={alt_mean:.3f}m)")
            stable_start = None
            stable_samples = []

    # Don't forget last period
    if stable_start is not None and len(stable_samples) > 25:
        duration = (stable_samples[-1]['time_ms'] - stable_start) / 1000
        alt_mean = sum(s['altitude'] for s in stable_samples) / len(stable_samples)
        print(f"  t={stable_start/1000:.1f}-{stable_samples[-1]['time_ms']/1000:.1f}s "
              f"({duration:.1f}s, mean alt={alt_mean:.3f}m)")

    if not any(
        abs(d['altitude'] - target_alt) < 0.15 and
        (d['roll']**2 + d['pitch']**2)**0.5 * 57.3 < 15
        for d in data
    ):
        print("  None found")


def main():
    parser = argparse.ArgumentParser(description='Debug flight telemetry')
    parser.add_argument('file', help='Telemetry CSV file')
    parser.add_argument('--interval', type=float, default=2.0,
                        help='Timeline interval in seconds (default: 2.0)')
    parser.add_argument('--time', nargs=2, type=float, metavar=('START', 'END'),
                        help='Analyze specific time window')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show detailed timeline (0.5s intervals)')
    args = parser.parse_args()

    try:
        data = load_telemetry(args.file)
    except Exception as e:
        print(f"Error loading {args.file}: {e}", file=sys.stderr)
        sys.exit(1)

    if args.time:
        data = [d for d in data
                if args.time[0] * 1000 <= d['time_ms'] <= args.time[1] * 1000]

    if not data:
        print("No data in specified range")
        sys.exit(1)

    # Analyze
    analysis = analyze_flight(data)

    print(f"=== Flight Analysis: {args.file} ===")
    print(f"Duration: {analysis['duration_s']:.1f}s")
    print(f"Max altitude: {analysis['max_altitude']:.3f}m at t={analysis['max_altitude_t']:.1f}s "
          f"(target: {analysis['target_altitude']:.1f}m)")
    print(f"Max tilt: {analysis['max_tilt_deg']:.1f}deg")
    print(f"Thrust saturation: {analysis['thrust_saturation_pct']:.0f}%")
    print(f"XY drift: ({analysis['x_drift']:.2f}, {analysis['y_drift']:.2f})m")

    if analysis['issues']:
        print("\nIssues detected:")
        for issue in analysis['issues']:
            print(f"  - {issue}")
    else:
        print("\nNo major issues detected")

    # Timeline
    interval = 0.5 if args.verbose else args.interval
    print_timeline(data, interval)

    # Stable periods
    print_stable_periods(data, analysis['target_altitude'])


if __name__ == '__main__':
    main()
