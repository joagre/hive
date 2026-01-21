#!/usr/bin/env python3
"""
Plot full flight summary with 3D trajectory visualization.

Generates a comprehensive flight report including:
- 3D trajectory plot
- Altitude profile
- Horizontal position (XY)
- Attitude angles
- Thrust command
- Velocities
- Flight statistics summary

Usage:
    python3 plot_flight.py telemetry.csv
    python3 plot_flight.py telemetry.csv -o flight_report.png
    python3 plot_flight.py telemetry.csv --stats
"""

import argparse
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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
    data['vx'] = np.array([float(r['vx']) for r in rows])
    data['vy'] = np.array([float(r['vy']) for r in rows])
    data['vz'] = np.array([float(r['vz']) for r in rows])
    data['roll'] = np.array([float(r['roll']) for r in rows])
    data['pitch'] = np.array([float(r['pitch']) for r in rows])
    data['yaw'] = np.array([float(r['yaw']) for r in rows])
    data['thrust'] = np.array([float(r['thrust']) for r in rows])

    return data


def compute_flight_stats(data):
    """Compute flight statistics."""
    stats = {}

    t = data['t']
    alt = data['altitude']
    thrust = data['thrust']
    x, y = data['x'], data['y']
    target_x, target_y = data['target_x'], data['target_y']
    target_z = data['target_z']

    # Basic stats
    stats['duration_s'] = round(t[-1], 1)
    stats['max_altitude_m'] = round(np.max(alt), 3)
    stats['max_x_m'] = round(np.max(x), 3)
    stats['min_x_m'] = round(np.min(x), 3)
    stats['max_y_m'] = round(np.max(y), 3)
    stats['min_y_m'] = round(np.min(y), 3)

    # Landing detection
    if alt[-1] < 0.05 and thrust[-1] < 0.1:
        stats['landing'] = 'SUCCESS'
    elif alt[-1] < 0.1:
        stats['landing'] = 'TOUCHDOWN'
    else:
        stats['landing'] = 'IN FLIGHT'

    # Altitude tracking error (when in flight)
    mask = target_z > 0.3
    if np.any(mask):
        alt_error = np.abs(alt[mask] - target_z[mask])
        stats['altitude_rms_error_m'] = round(np.sqrt(np.mean(alt_error**2)), 3)

    # Position tracking error (when position target is set)
    mask = target_x > 0.1
    if np.any(mask):
        pos_error = np.sqrt((x[mask] - target_x[mask])**2 +
                            (y[mask] - target_y[mask])**2)
        stats['position_rms_error_m'] = round(np.sqrt(np.mean(pos_error**2)), 3)

    # Flight time (motors on)
    motor_on = thrust > 0.1
    if np.any(motor_on):
        motor_on_indices = np.where(motor_on)[0]
        flight_start = t[motor_on_indices[0]]
        flight_end = t[motor_on_indices[-1]]
        stats['flight_time_s'] = round(flight_end - flight_start, 1)

    # Max velocities
    stats['max_vz_m_s'] = round(np.max(np.abs(data['vz'])), 2)
    stats['max_vxy_m_s'] = round(np.max(np.sqrt(data['vx']**2 + data['vy']**2)), 2)

    return stats


def print_stats(stats):
    """Print flight statistics."""
    print("\n" + "="*50)
    print("FLIGHT STATISTICS")
    print("="*50)
    print(f"Duration:            {stats.get('duration_s', 'N/A')} s")
    print(f"Flight time:         {stats.get('flight_time_s', 'N/A')} s")
    print(f"Landing:             {stats.get('landing', 'N/A')}")
    print(f"Max altitude:        {stats.get('max_altitude_m', 'N/A')} m")
    print(f"X range:             {stats.get('min_x_m', 'N/A')} to {stats.get('max_x_m', 'N/A')} m")
    print(f"Y range:             {stats.get('min_y_m', 'N/A')} to {stats.get('max_y_m', 'N/A')} m")
    print(f"Altitude RMS error:  {stats.get('altitude_rms_error_m', 'N/A')} m")
    print(f"Position RMS error:  {stats.get('position_rms_error_m', 'N/A')} m")
    print(f"Max vertical vel:    {stats.get('max_vz_m_s', 'N/A')} m/s")
    print(f"Max horizontal vel:  {stats.get('max_vxy_m_s', 'N/A')} m/s")
    print()


def plot_flight(data, title=None):
    """Generate full flight visualization."""
    t = data['t']
    x, y, alt = data['x'], data['y'], data['altitude']
    target_x, target_y, target_z = data['target_x'], data['target_y'], data['target_z']
    roll, pitch, yaw = data['roll'], data['pitch'], data['yaw']
    thrust = data['thrust']
    vx, vy, vz = data['vx'], data['vy'], data['vz']

    fig = plt.figure(figsize=(14, 10))
    if title:
        fig.suptitle(title, fontsize=14, fontweight='bold')

    # 1. 3D trajectory
    ax = fig.add_subplot(2, 3, 1, projection='3d')
    ax.plot(x, y, alt, 'b-', linewidth=1, label='Actual')
    ax.plot(target_x, target_y, target_z, 'r--', linewidth=2, alpha=0.7, label='Target')
    ax.scatter([x[0]], [y[0]], [alt[0]], c='green', s=100, marker='o', label='Start')
    ax.scatter([x[-1]], [y[-1]], [alt[-1]], c='red', s=100, marker='x', label='End')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('3D Trajectory')
    ax.legend(loc='upper left', fontsize=8)

    # 2. Altitude over time
    ax = fig.add_subplot(2, 3, 2)
    ax.plot(t, alt, 'b-', label='Actual', linewidth=1.5)
    ax.plot(t, target_z, 'r--', label='Target', linewidth=1.5)
    ax.fill_between(t, 0, alt, alpha=0.3)
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Altitude Profile')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # 3. XY position
    ax = fig.add_subplot(2, 3, 3)
    ax.plot(t, x, 'b-', label='X', linewidth=1)
    ax.plot(t, y, 'g-', label='Y', linewidth=1)
    ax.plot(t, target_x, 'b--', alpha=0.5, linewidth=1)
    ax.plot(t, target_y, 'g--', alpha=0.5, linewidth=1)
    ax.set_ylabel('Position (m)')
    ax.set_title('Horizontal Position')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # 4. Attitude angles
    ax = fig.add_subplot(2, 3, 4)
    ax.plot(t, np.degrees(roll), label='Roll', linewidth=1)
    ax.plot(t, np.degrees(pitch), label='Pitch', linewidth=1)
    ax.plot(t, np.degrees(yaw), label='Yaw', linewidth=1)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('Attitude')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # 5. Thrust
    ax = fig.add_subplot(2, 3, 5)
    ax.plot(t, thrust, 'orange', linewidth=1.5)
    ax.axhline(y=0.553, color='r', linestyle='--', alpha=0.5, label='Hover')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Thrust')
    ax.set_title('Thrust Command')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 0.8)

    # 6. Velocities
    ax = fig.add_subplot(2, 3, 6)
    ax.plot(t, vx, label='Vx', linewidth=1)
    ax.plot(t, vy, label='Vy', linewidth=1)
    ax.plot(t, vz, label='Vz', linewidth=1)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Velocities')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(
        description='Plot full flight summary with 3D trajectory',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s telemetry.csv                    # Show plot
  %(prog)s telemetry.csv -o report.png      # Save to file
  %(prog)s telemetry.csv --stats            # Print statistics only
  %(prog)s telemetry.csv --title "Test 1"   # Custom title
""")

    parser.add_argument('csvfile', help='Telemetry CSV file')
    parser.add_argument('-o', '--output', help='Output image file (PNG)')
    parser.add_argument('--title', help='Plot title',
                        default='Full Flight Test')
    parser.add_argument('--stats', action='store_true',
                        help='Print statistics only (no plot)')
    parser.add_argument('--dpi', type=int, default=150,
                        help='Output image DPI (default: 150)')

    args = parser.parse_args()

    data = load_telemetry(args.csvfile)
    stats = compute_flight_stats(data)

    # Always print stats
    print_stats(stats)

    if not args.stats:
        fig = plot_flight(data, args.title)

        if args.output:
            plt.savefig(args.output, dpi=args.dpi)
            print(f"Saved: {args.output}")
        else:
            plt.show()


if __name__ == '__main__':
    main()
