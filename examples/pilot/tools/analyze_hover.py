#!/usr/bin/env python3
"""
Analyze hover stability from telemetry CSV files.

Detects hover phases and computes stability metrics:
- Altitude standard deviation
- Vertical velocity noise (stddev)
- Maximum tilt angle
- Position drift

Useful for comparing filter performance (KF vs LPF) and tuning.

Usage:
    python3 analyze_hover.py /tmp/tlog.csv
    python3 analyze_hover.py file1.csv file2.csv --compare
    python3 analyze_hover.py telemetry.csv --json
    python3 analyze_hover.py telemetry.csv --time 5 15
"""

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class HoverMetrics:
    """Hover stability metrics."""
    duration_s: float
    samples: int
    altitude_mean: float
    altitude_stddev: float
    altitude_range: Tuple[float, float]
    velocity_mean: float
    velocity_stddev: float
    velocity_range: Tuple[float, float]
    roll_max_deg: float
    pitch_max_deg: float
    tilt_max_deg: float
    x_drift: float
    y_drift: float
    target_altitude: float


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
            })
    return data


def find_hover_phases(data: List[dict],
                      min_duration_s: float = 1.0,
                      altitude_tolerance: float = 0.2) -> List[List[dict]]:
    """
    Find hover phases in telemetry data.

    A hover phase is detected when:
    - Altitude is within tolerance of target
    - Thrust is non-zero (airborne)
    - Duration exceeds minimum
    """
    phases = []
    current_phase = []

    for d in data:
        in_hover = (
            d['thrust'] > 0.1 and
            abs(d['altitude'] - d['target_z']) < altitude_tolerance and
            d['target_z'] > 0.1  # Not landed
        )

        if in_hover:
            current_phase.append(d)
        else:
            if current_phase:
                duration = (current_phase[-1]['time_ms'] - current_phase[0]['time_ms']) / 1000
                if duration >= min_duration_s:
                    phases.append(current_phase)
            current_phase = []

    # Don't forget last phase
    if current_phase:
        duration = (current_phase[-1]['time_ms'] - current_phase[0]['time_ms']) / 1000
        if duration >= min_duration_s:
            phases.append(current_phase)

    return phases


def compute_stddev(values: List[float]) -> float:
    """Compute standard deviation."""
    if len(values) < 2:
        return 0.0
    mean = sum(values) / len(values)
    variance = sum((x - mean) ** 2 for x in values) / (len(values) - 1)
    return math.sqrt(variance)


def analyze_hover_phase(phase: List[dict]) -> HoverMetrics:
    """Compute hover metrics for a phase."""
    altitudes = [d['altitude'] for d in phase]
    velocities = [d['vz'] for d in phase]
    rolls = [abs(d['roll']) * 57.3 for d in phase]  # Convert to degrees
    pitches = [abs(d['pitch']) * 57.3 for d in phase]

    # Compute tilt angle (combined roll+pitch)
    tilts = [math.sqrt(d['roll']**2 + d['pitch']**2) * 57.3 for d in phase]

    duration = (phase[-1]['time_ms'] - phase[0]['time_ms']) / 1000

    # Position drift
    x_drift = phase[-1]['x'] - phase[0]['x']
    y_drift = phase[-1]['y'] - phase[0]['y']

    return HoverMetrics(
        duration_s=duration,
        samples=len(phase),
        altitude_mean=sum(altitudes) / len(altitudes),
        altitude_stddev=compute_stddev(altitudes),
        altitude_range=(min(altitudes), max(altitudes)),
        velocity_mean=sum(velocities) / len(velocities),
        velocity_stddev=compute_stddev(velocities),
        velocity_range=(min(velocities), max(velocities)),
        roll_max_deg=max(rolls),
        pitch_max_deg=max(pitches),
        tilt_max_deg=max(tilts),
        x_drift=x_drift,
        y_drift=y_drift,
        target_altitude=phase[0]['target_z'],
    )


def filter_by_time(data: List[dict], t_start: float, t_end: float) -> List[dict]:
    """Filter data to time range (seconds)."""
    return [d for d in data
            if t_start * 1000 <= d['time_ms'] <= t_end * 1000]


def print_metrics(metrics: HoverMetrics, label: str = ""):
    """Print hover metrics in human-readable format."""
    if label:
        print(f"\n=== {label} ===")
    print(f"Hover at {metrics.target_altitude:.1f}m for {metrics.duration_s:.1f}s ({metrics.samples} samples)")
    print(f"  Altitude: mean={metrics.altitude_mean:.4f}m, stddev={metrics.altitude_stddev:.4f}m")
    print(f"            range=[{metrics.altitude_range[0]:.4f}, {metrics.altitude_range[1]:.4f}]m")
    print(f"  Velocity: mean={metrics.velocity_mean:.4f}m/s, stddev={metrics.velocity_stddev:.4f}m/s")
    print(f"            range=[{metrics.velocity_range[0]:.4f}, {metrics.velocity_range[1]:.4f}]m/s")
    print(f"  Max tilt: {metrics.tilt_max_deg:.2f}deg (roll={metrics.roll_max_deg:.2f}, pitch={metrics.pitch_max_deg:.2f})")
    print(f"  XY drift: ({metrics.x_drift:.3f}, {metrics.y_drift:.3f})m")


def metrics_to_dict(metrics: HoverMetrics) -> dict:
    """Convert metrics to dictionary for JSON output."""
    return {
        'duration_s': metrics.duration_s,
        'samples': metrics.samples,
        'altitude': {
            'mean': metrics.altitude_mean,
            'stddev': metrics.altitude_stddev,
            'min': metrics.altitude_range[0],
            'max': metrics.altitude_range[1],
        },
        'velocity': {
            'mean': metrics.velocity_mean,
            'stddev': metrics.velocity_stddev,
            'min': metrics.velocity_range[0],
            'max': metrics.velocity_range[1],
        },
        'tilt_max_deg': metrics.tilt_max_deg,
        'roll_max_deg': metrics.roll_max_deg,
        'pitch_max_deg': metrics.pitch_max_deg,
        'x_drift': metrics.x_drift,
        'y_drift': metrics.y_drift,
        'target_altitude': metrics.target_altitude,
    }


def main():
    parser = argparse.ArgumentParser(
        description='Analyze hover stability from telemetry CSV')
    parser.add_argument('files', nargs='+', help='Telemetry CSV file(s)')
    parser.add_argument('--compare', action='store_true',
                        help='Compare two files side-by-side')
    parser.add_argument('--json', action='store_true',
                        help='Output as JSON')
    parser.add_argument('--time', nargs=2, type=float, metavar=('START', 'END'),
                        help='Analyze specific time window (seconds)')
    parser.add_argument('--tolerance', type=float, default=0.2,
                        help='Altitude tolerance for hover detection (default: 0.2m)')
    parser.add_argument('--min-duration', type=float, default=1.0,
                        help='Minimum hover duration (default: 1.0s)')
    parser.add_argument('--summary', action='store_true',
                        help='Show only summary (first hover phase)')
    args = parser.parse_args()

    results = []

    for filepath in args.files:
        try:
            data = load_telemetry(filepath)
        except Exception as e:
            print(f"Error loading {filepath}: {e}", file=sys.stderr)
            sys.exit(1)

        if args.time:
            data = filter_by_time(data, args.time[0], args.time[1])

        phases = find_hover_phases(data, args.min_duration, args.tolerance)

        if not phases:
            # Check if flight failed
            max_alt = max(d['altitude'] for d in data) if data else 0
            if args.json:
                results.append({
                    'file': filepath,
                    'error': 'No hover phases found',
                    'max_altitude': max_alt,
                })
            else:
                print(f"\n{filepath}:")
                print(f"  No hover phases found (max altitude: {max_alt:.3f}m)")
                if max_alt < 0.5:
                    print("  Flight likely failed - drone never climbed above 0.5m")
            continue

        file_metrics = []
        for i, phase in enumerate(phases):
            metrics = analyze_hover_phase(phase)
            file_metrics.append(metrics)

            if args.summary and i > 0:
                continue

            if not args.json:
                label = f"{filepath} - Phase {i+1}" if len(phases) > 1 else filepath
                print_metrics(metrics, label)

        if args.json:
            results.append({
                'file': filepath,
                'phases': [metrics_to_dict(m) for m in file_metrics],
            })

    # Comparison output
    if args.compare and len(args.files) == 2 and len(results) == 2:
        if not args.json:
            print("\n=== Comparison ===")
            if 'error' not in results[0] and 'error' not in results[1]:
                m1 = results[0]['phases'][0] if args.json else file_metrics[0]
                # Need to reload second file metrics
                data2 = load_telemetry(args.files[1])
                if args.time:
                    data2 = filter_by_time(data2, args.time[0], args.time[1])
                phases2 = find_hover_phases(data2, args.min_duration, args.tolerance)
                if phases2:
                    m2 = analyze_hover_phase(phases2[0])
                    print(f"Altitude stddev: {file_metrics[0].altitude_stddev:.4f}m vs {m2.altitude_stddev:.4f}m")
                    print(f"Velocity stddev: {file_metrics[0].velocity_stddev:.4f}m/s vs {m2.velocity_stddev:.4f}m/s")
                    print(f"Max tilt: {file_metrics[0].tilt_max_deg:.2f}deg vs {m2.tilt_max_deg:.2f}deg")

    if args.json:
        print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
