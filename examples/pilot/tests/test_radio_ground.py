#!/usr/bin/env python3
"""
Radio Communication Test - Ground Station

Tests interrupt-based syslink radio communication with Crazyflie running test_esb.

Requirements:
    pip install cflib

Usage:
    python test_radio_ground.py [--uri URI]

The script will:
    1. Scan for Crazyflie
    2. Connect via Crazyradio PA/2.0
    3. Receive and decode telemetry packets
    4. Display statistics

Packet types (matching test_esb.c):
    0x01: Attitude (gyro, roll/pitch/yaw)
    0x02: Position (altitude, velocities, thrust)
    0xE0: Echo test packet
"""

import argparse
import struct
import sys
import time
from collections import defaultdict

try:
    import cflib.crtp
    from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
    from cflib.drivers.crazyradio import Crazyradio
except ImportError:
    print("Error: cflib not installed")
    print("Install with: pip install cflib")
    sys.exit(1)


# Packet types (must match test_esb.c)
PACKET_TYPE_ATTITUDE = 0x01
PACKET_TYPE_POSITION = 0x02
PACKET_TYPE_ECHO = 0xE0

# Attitude packet format: type(1) + timestamp(4) + gyro_xyz(6) + rpy(6) = 17 bytes
ATTITUDE_FORMAT = "<BIhhhhhh"
ATTITUDE_SIZE = struct.calcsize(ATTITUDE_FORMAT)

# Position packet format: type(1) + timestamp(4) + alt(2) + vz(2) + vx(2) + vy(2) + thrust(2) = 15 bytes
POSITION_FORMAT = "<BIhhhhhH"
POSITION_SIZE = struct.calcsize(POSITION_FORMAT)

# Echo packet format: type(1) + sequence(1) + data(29) = 31 bytes
ECHO_FORMAT = "<BB29s"
ECHO_SIZE = struct.calcsize(ECHO_FORMAT)


class RadioTest:
    def __init__(self, uri=None):
        self.uri = uri
        self.radio = None
        self.running = False

        # Statistics
        self.stats = defaultdict(int)
        self.last_attitude = None
        self.last_position = None
        self.last_echo = None
        self.start_time = None

    def scan(self):
        """Scan for available Crazyflies."""
        print("Scanning for Crazyflie...")
        cflib.crtp.init_drivers()

        available = cflib.crtp.scan_interfaces()
        if not available:
            print("No Crazyflie found!")
            return None

        print(f"Found {len(available)} Crazyflie(s):")
        for i, uri in enumerate(available):
            print(f"  [{i}] {uri[0]}")

        return available[0][0]  # Return first URI

    def connect(self):
        """Connect to Crazyflie."""
        if not self.uri:
            self.uri = self.scan()
            if not self.uri:
                return False

        print(f"Connecting to {self.uri}...")

        # Initialize the low-level radio driver
        cflib.crtp.init_drivers()

        # Get the link driver
        self.link = cflib.crtp.get_link_driver(self.uri)
        if not self.link:
            print("Failed to get link driver")
            return False

        print("Connected!")
        return True

    def decode_attitude(self, data):
        """Decode attitude packet."""
        if len(data) < ATTITUDE_SIZE:
            return None

        try:
            pkt_type, ts, gx, gy, gz, roll, pitch, yaw = struct.unpack(
                ATTITUDE_FORMAT, data[:ATTITUDE_SIZE]
            )
            return {
                'type': 'attitude',
                'timestamp_ms': ts,
                'gyro': (gx / 1000.0, gy / 1000.0, gz / 1000.0),  # millirad/s -> rad/s
                'roll': roll / 1000.0,   # millirad -> rad
                'pitch': pitch / 1000.0,
                'yaw': yaw / 1000.0,
            }
        except struct.error:
            return None

    def decode_position(self, data):
        """Decode position packet."""
        if len(data) < POSITION_SIZE:
            return None

        try:
            pkt_type, ts, alt, vz, vx, vy, thrust = struct.unpack(
                POSITION_FORMAT, data[:POSITION_SIZE]
            )
            return {
                'type': 'position',
                'timestamp_ms': ts,
                'altitude': alt / 1000.0,  # mm -> m
                'vz': vz / 1000.0,         # mm/s -> m/s
                'vx': vx / 1000.0,
                'vy': vy / 1000.0,
                'thrust': thrust / 65535.0,  # 0-65535 -> 0.0-1.0
            }
        except struct.error:
            return None

    def decode_echo(self, data):
        """Decode echo packet."""
        if len(data) < 3:
            return None

        try:
            pkt_type, seq, payload = struct.unpack(ECHO_FORMAT, data[:ECHO_SIZE])
            return {
                'type': 'echo',
                'sequence': seq,
                'data': payload.rstrip(b'\x00').decode('ascii', errors='replace'),
            }
        except struct.error:
            return None

    def decode_packet(self, data):
        """Decode a received packet based on type byte."""
        if not data or len(data) < 1:
            return None

        pkt_type = data[0]

        if pkt_type == PACKET_TYPE_ATTITUDE:
            return self.decode_attitude(data)
        elif pkt_type == PACKET_TYPE_POSITION:
            return self.decode_position(data)
        elif pkt_type == PACKET_TYPE_ECHO:
            return self.decode_echo(data)
        else:
            return {'type': 'unknown', 'raw': data.hex()}

    def print_status(self):
        """Print current status."""
        elapsed = time.time() - self.start_time if self.start_time else 0

        print(f"\r[{elapsed:6.1f}s] ", end='')
        print(f"RX: {self.stats['total']:5d} ", end='')
        print(f"(att:{self.stats['attitude']:4d} pos:{self.stats['position']:4d} echo:{self.stats['echo']:4d}) ", end='')

        if self.last_attitude:
            att = self.last_attitude
            print(f"| R:{att['roll']:+5.2f} P:{att['pitch']:+5.2f} Y:{att['yaw']:+5.2f} ", end='')

        print("", end='', flush=True)

    def run(self, duration=60):
        """Run the test for specified duration."""
        if not self.connect():
            return False

        print(f"\nReceiving packets for {duration} seconds...")
        print("Press Ctrl+C to stop\n")

        self.start_time = time.time()
        self.running = True

        try:
            while self.running and (time.time() - self.start_time) < duration:
                # Receive packet (with timeout)
                pk = self.link.receive_packet(wait=0.1)

                if pk is not None:
                    # Decode the packet
                    decoded = self.decode_packet(pk.data)

                    if decoded:
                        self.stats['total'] += 1
                        self.stats[decoded['type']] += 1

                        if decoded['type'] == 'attitude':
                            self.last_attitude = decoded
                        elif decoded['type'] == 'position':
                            self.last_position = decoded
                        elif decoded['type'] == 'echo':
                            self.last_echo = decoded

                # Update display periodically
                if self.stats['total'] % 10 == 0 or self.stats['total'] == 1:
                    self.print_status()

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")

        self.running = False

        # Final statistics
        elapsed = time.time() - self.start_time
        print("\n")
        print("=" * 60)
        print("  TEST COMPLETE")
        print("=" * 60)
        print(f"  Duration:     {elapsed:.1f} seconds")
        print(f"  Total RX:     {self.stats['total']}")
        print(f"  Attitude:     {self.stats['attitude']}")
        print(f"  Position:     {self.stats['position']}")
        print(f"  Echo:         {self.stats['echo']}")
        print(f"  Unknown:      {self.stats['unknown']}")
        if elapsed > 0:
            print(f"  Rate:         {self.stats['total'] / elapsed:.1f} packets/sec")
        print("=" * 60)

        # Cleanup
        self.link.close()

        return self.stats['total'] > 0


def main():
    parser = argparse.ArgumentParser(
        description='Radio communication test - ground station'
    )
    parser.add_argument(
        '--uri', '-u',
        default=None,
        help='Crazyflie URI (e.g., radio://0/80/2M). Auto-scan if not specified.'
    )
    parser.add_argument(
        '--duration', '-d',
        type=int,
        default=60,
        help='Test duration in seconds (default: 60)'
    )

    args = parser.parse_args()

    print("=" * 60)
    print("  Radio Communication Test - Ground Station")
    print("=" * 60)
    print()

    test = RadioTest(uri=args.uri)
    success = test.run(duration=args.duration)

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
