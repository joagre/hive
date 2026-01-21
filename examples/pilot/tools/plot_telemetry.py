#!/usr/bin/env python3
"""
Plot telemetry data from pilot CSV logs.

Usage:
    python3 plot_telemetry.py telemetry.csv
    python3 plot_telemetry.py telemetry.csv -o output.png
    python3 plot_telemetry.py telemetry.csv --altitude --position
"""

import argparse
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt


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
    data['roll_rate'] = np.array([float(r['roll_rate']) for r in rows])
    data['pitch_rate'] = np.array([float(r['pitch_rate']) for r in rows])
    data['yaw_rate'] = np.array([float(r['yaw_rate']) for r in rows])
    data['thrust'] = np.array([float(r['thrust']) for r in rows])
    data['accel_x'] = np.array([float(r['accel_x']) for r in rows])
    data['accel_y'] = np.array([float(r['accel_y']) for r in rows])
    data['accel_z'] = np.array([float(r['accel_z']) for r in rows])
    data['gyro_x'] = np.array([float(r['gyro_x']) for r in rows])
    data['gyro_y'] = np.array([float(r['gyro_y']) for r in rows])
    data['gyro_z'] = np.array([float(r['gyro_z']) for r in rows])

    return data


def plot_altitude(ax, data, xlim=None):
    """Plot altitude response."""
    t, alt, target = data['t'], data['altitude'], data['target_z']
    ax.plot(t, alt, 'b-', label='Actual', linewidth=1.5)
    ax.plot(t, target, 'r--', label='Target', linewidth=1.5)
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Altitude Response')
    ax.legend(loc='lower right')
    ax.grid(True, alpha=0.3)
    if xlim:
        ax.set_xlim(xlim)


def plot_vertical_velocity(ax, data, xlim=None):
    """Plot vertical velocity and thrust."""
    t, vz, thrust = data['t'], data['vz'], data['thrust']
    ax2 = ax.twinx()
    ax.plot(t, vz, 'g-', label='Vertical Vel', linewidth=1)
    ax2.plot(t, thrust, 'orange', label='Thrust', linewidth=1, alpha=0.7)
    ax.set_ylabel('Velocity (m/s)', color='g')
    ax2.set_ylabel('Thrust', color='orange')
    ax.set_title('Vertical Velocity & Thrust')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left')
    ax2.legend(loc='upper right')
    if xlim:
        ax.set_xlim(xlim)


def plot_attitude(ax, data, xlim=None):
    """Plot attitude angles."""
    t = data['t']
    ax.plot(t, np.degrees(data['roll']), label='Roll', linewidth=1)
    ax.plot(t, np.degrees(data['pitch']), label='Pitch', linewidth=1)
    ax.plot(t, np.degrees(data['yaw']), label='Yaw', linewidth=1)
    ax.set_ylabel('Angle (deg)')
    ax.set_title('Attitude Angles')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    if xlim:
        ax.set_xlim(xlim)


def plot_rates(ax, data, xlim=None):
    """Plot angular rates."""
    t = data['t']
    ax.plot(t, np.degrees(data['roll_rate']), label='Roll Rate', linewidth=1)
    ax.plot(t, np.degrees(data['pitch_rate']), label='Pitch Rate', linewidth=1)
    ax.plot(t, np.degrees(data['yaw_rate']), label='Yaw Rate', linewidth=1)
    ax.set_ylabel('Rate (deg/s)')
    ax.set_title('Angular Rates')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    if xlim:
        ax.set_xlim(xlim)


def plot_position(ax, data, xlim=None):
    """Plot horizontal position."""
    t = data['t']
    ax.plot(t, data['x'], label='X', linewidth=1)
    ax.plot(t, data['y'], label='Y', linewidth=1)
    ax.plot(t, data['target_x'], 'r--', label='Target X', linewidth=1, alpha=0.7)
    ax.plot(t, data['target_y'], 'm--', label='Target Y', linewidth=1, alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('Horizontal Position')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    if xlim:
        ax.set_xlim(xlim)


def plot_accelerometer(ax, data, xlim=None):
    """Plot raw accelerometer data."""
    t = data['t']
    ax.plot(t, data['accel_x'], label='Accel X', linewidth=0.5, alpha=0.8)
    ax.plot(t, data['accel_y'], label='Accel Y', linewidth=0.5, alpha=0.8)
    ax.plot(t, data['accel_z'], label='Accel Z', linewidth=0.5, alpha=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.set_title('Raw Accelerometer')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    if xlim:
        ax.set_xlim(xlim)


def plot_all(data, title=None, xlim=None):
    """Generate 6-panel telemetry plot."""
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    if title:
        fig.suptitle(title, fontsize=14, fontweight='bold')

    plot_altitude(axes[0, 0], data, xlim)
    plot_vertical_velocity(axes[0, 1], data, xlim)
    plot_attitude(axes[1, 0], data, xlim)
    plot_rates(axes[1, 1], data, xlim)
    plot_position(axes[2, 0], data, xlim)
    plot_accelerometer(axes[2, 1], data, xlim)

    plt.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(
        description='Plot telemetry data from pilot CSV logs',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s telemetry.csv                    # Show all plots
  %(prog)s telemetry.csv -o output.png      # Save to file
  %(prog)s telemetry.csv --xlim 0 20        # Limit time range
  %(prog)s telemetry.csv --altitude         # Only altitude plot
""")

    parser.add_argument('csvfile', help='Telemetry CSV file')
    parser.add_argument('-o', '--output', help='Output image file (PNG)')
    parser.add_argument('--title', help='Plot title')
    parser.add_argument('--xlim', nargs=2, type=float, metavar=('MIN', 'MAX'),
                        help='Time axis limits')
    parser.add_argument('--altitude', action='store_true',
                        help='Plot only altitude response')
    parser.add_argument('--position', action='store_true',
                        help='Plot only position response')
    parser.add_argument('--attitude', action='store_true',
                        help='Plot only attitude angles')
    parser.add_argument('--dpi', type=int, default=150,
                        help='Output image DPI (default: 150)')

    args = parser.parse_args()

    data = load_telemetry(args.csvfile)
    xlim = tuple(args.xlim) if args.xlim else None

    # Single plot mode
    if args.altitude or args.position or args.attitude:
        fig, ax = plt.subplots(figsize=(10, 5))
        if args.altitude:
            plot_altitude(ax, data, xlim)
        elif args.position:
            plot_position(ax, data, xlim)
        elif args.attitude:
            plot_attitude(ax, data, xlim)
        if args.title:
            ax.set_title(args.title)
    else:
        # Full 6-panel plot
        title = args.title or 'Pilot Telemetry'
        fig = plot_all(data, title, xlim)

    if args.output:
        plt.savefig(args.output, dpi=args.dpi)
        print(f"Saved: {args.output}")
    else:
        plt.show()


if __name__ == '__main__':
    main()
