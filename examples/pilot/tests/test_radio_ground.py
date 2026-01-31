#!/usr/bin/env python3
"""
ESB Radio Communication Test - Ground Station

Tests radio communication with Crazyflie running test_esb firmware.

ESB Protocol:
    - Ground station is PTX (Primary Transmitter) - initiates all communication
    - Drone's nRF51 is PRX (Primary Receiver) - responds via ACK payloads
    - Ground station must poll continuously; telemetry piggybacks on ACK packets

Packet format (must match test_esb.c):
    Byte 0: Type (0x03 = battery)
    Bytes 1-4: Voltage (float, little-endian)

Usage:
    python test_radio_ground.py [--uri URI] [--duration SECONDS] [--rate HZ]
"""

import argparse
import struct
import sys
import time
from collections import defaultdict

try:
    import cflib.crtp
    from cflib.crtp.crtpstack import CRTPPacket
except ImportError:
    print("Error: cflib not installed")
    print("Install with: pip install cflib")
    sys.exit(1)


# Packet type (must match test_esb.c)
PACKET_TYPE_BATTERY = 0x03


def decode_packet(data):
    """Decode a telemetry packet from drone."""
    if not data or len(data) < 1:
        return None

    pkt_type = data[0]

    # Battery packet: type(1) + voltage(4) = 5 bytes
    if pkt_type == PACKET_TYPE_BATTERY and len(data) >= 5:
        try:
            _, voltage = struct.unpack("<Bf", data[:5])
            if 2.0 < voltage < 5.0:
                return {'type': 'battery', 'voltage': voltage}
        except struct.error:
            pass

    # Debug: show unknown packet contents
    hex_dump = ' '.join(f'{b:02x}' for b in data[:min(len(data), 16)])
    print(f"\nUnknown: type={pkt_type} len={len(data)} hex=[{hex_dump}]")
    return {'type': 'unknown', 'first_byte': pkt_type, 'len': len(data)}


class GroundStation:
    def __init__(self, uri=None, poll_rate_hz=100):
        self.uri = uri
        self.poll_rate_hz = poll_rate_hz
        self.poll_interval = 1.0 / poll_rate_hz
        self.link = None
        self.running = False
        self.stats = defaultdict(int)
        self.last_voltage = None
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

        return available[0][0]

    def connect(self):
        """Connect to Crazyflie."""
        if not self.uri:
            self.uri = self.scan()
            if not self.uri:
                return False

        print(f"Connecting to {self.uri}...")
        cflib.crtp.init_drivers()

        self.link = cflib.crtp.get_link_driver(self.uri)
        if not self.link:
            print("Failed to get link driver")
            return False

        print("Connected!")
        return True

    def poll(self):
        """
        Send heartbeat and receive telemetry.

        ESB protocol: we send a packet, drone responds with ACK + telemetry.
        """
        # Send heartbeat
        pk = CRTPPacket()
        pk.port = 0
        pk.data = bytes([0x00])
        self.link.send_packet(pk)

        # Receive ACK payload
        pk = self.link.receive_packet(wait=0.005)
        if pk is not None and pk.data:
            # CRTP uses first byte as header; our type ends up in pk.channel
            pkt_type = pk.channel
            full_data = bytes([pkt_type]) + pk.data
            return decode_packet(full_data)
        return None

    def run(self, duration=60):
        """Run ground station for specified duration."""
        if not self.connect():
            return False

        print(f"\nPolling at {self.poll_rate_hz} Hz for {duration} seconds...")
        print("Press Ctrl+C to stop\n")

        self.start_time = time.time()
        self.running = True
        last_status = 0
        poll_count = 0

        try:
            while self.running and (time.time() - self.start_time) < duration:
                loop_start = time.time()

                telemetry = self.poll()
                poll_count += 1

                if telemetry:
                    self.stats[telemetry['type']] += 1
                    if telemetry['type'] == 'battery':
                        self.last_voltage = telemetry['voltage']

                # Update display every 0.5 seconds
                now = time.time()
                if now - last_status >= 0.5:
                    last_status = now
                    elapsed = now - self.start_time
                    voltage_str = f"{self.last_voltage:.2f}V" if self.last_voltage else "---"
                    print(f"\r[{elapsed:5.1f}s] Battery: {self.stats['battery']:4d}  "
                          f"Unknown: {self.stats['unknown']:4d}  | {voltage_str}", end='', flush=True)

                # Maintain poll rate
                sleep_time = self.poll_interval - (time.time() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\nInterrupted")

        self.running = False
        elapsed = time.time() - self.start_time

        print("\n")
        print("=" * 50)
        print("  TEST COMPLETE")
        print("=" * 50)
        print(f"  Duration:     {elapsed:.1f}s")
        print(f"  Poll rate:    {self.poll_rate_hz} Hz")
        print(f"  Polls sent:   {poll_count}")
        print(f"  Battery pkts: {self.stats['battery']}")
        print(f"  Unknown pkts: {self.stats['unknown']}")
        if elapsed > 0:
            print(f"  Packet rate:  {self.stats['battery'] / elapsed:.1f} pkts/sec")
        if self.last_voltage:
            print(f"  Last voltage: {self.last_voltage:.2f}V")
        print("=" * 50)

        self.link.close()
        return self.stats['battery'] > 0


def main():
    parser = argparse.ArgumentParser(description='ESB radio test ground station')
    parser.add_argument('--uri', '-u', default=None,
                        help='Crazyflie URI (auto-scan if not specified)')
    parser.add_argument('--duration', '-d', type=int, default=30,
                        help='Test duration in seconds (default: 30)')
    parser.add_argument('--rate', '-r', type=int, default=100,
                        help='Poll rate in Hz (default: 100)')

    args = parser.parse_args()

    print("=" * 50)
    print("  ESB Radio Test - Ground Station")
    print("=" * 50)
    print()
    print("Protocol:")
    print("  Ground station polls -> Drone ACKs with telemetry")
    print()

    gs = GroundStation(uri=args.uri, poll_rate_hz=args.rate)
    success = gs.run(duration=args.duration)

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
