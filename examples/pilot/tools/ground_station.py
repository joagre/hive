#!/usr/bin/env python3
"""
Receive telemetry and download logs from Crazyflie via Crazyradio 2.0.

Connects to the Crazyflie using ESB radio protocol and decodes binary
telemetry packets. Displays real-time data and optionally logs to CSV.
Can also download the flight log from flash storage.

ESB Protocol:
  Ground station is PTX (Primary Transmitter) - initiates all communication.
  Drone's nRF51 is PRX (Primary Receiver) - responds via ACK payloads.
  Telemetry from drone piggybacks on ACK packets when ground station polls.

  Uses Crazyradio direct API for raw byte access (no CRTP layer).

Telemetry packet formats (13 bytes each, no timestamp due to nRF51 16-byte limit):
  Type 0x01 - Attitude/Rates (13 bytes):
    type(1) + gyro_xyz(6) + roll/pitch/yaw(6)

  Type 0x02 - Position/Altitude (13 bytes):
    type(1) + altitude(2) + vz(2) + vx(2) + vy(2) + thrust(2) + battery_mv(2)

Log download packet formats (must match comms_actor.c):
  Type 0x10 - CMD_REQUEST_LOG (1 byte): Ground -> Drone to request log
  Type 0x11 - PACKET_LOG_CHUNK (31 bytes): Drone -> Ground log data chunk
    type(1) + sequence(2) + data(28)
  Type 0x12 - PACKET_LOG_DONE (3 bytes): Drone -> Ground download complete
    type(1) + total_chunks(2)

Requirements:
  pip install cflib

Usage:
  sudo ./ground_station.py                         # Display telemetry to stdout
  sudo ./ground_station.py -o flight.csv           # Log telemetry to CSV file
  sudo ./ground_station.py --download-log flight.log  # Download log file
  sudo ./ground_station.py --uri radio://0/80/2M   # Custom radio URI

Default radio URI: radio://0/80/2M (channel 80, 2Mbps)
"""

import argparse
import csv
import struct
import sys
import time
from datetime import datetime

try:
    import cflib.crtp
    from cflib.drivers.crazyradio import Crazyradio
except ImportError:
    print("Error: cflib not installed. Run: pip install cflib", file=sys.stderr)
    sys.exit(1)

# Packet type identifiers - telemetry
PACKET_TYPE_ATTITUDE = 0x01
PACKET_TYPE_POSITION = 0x02

# Packet type identifiers - log download (must match comms_actor.c)
CMD_REQUEST_LOG = 0x10
PACKET_LOG_CHUNK = 0x11
PACKET_LOG_DONE = 0x12

# Log chunk data size
LOG_CHUNK_DATA_SIZE = 28

# Scale factors (inverse of transmitter)
SCALE_ANGLE = 1000.0   # millirad -> rad
SCALE_RATE = 1000.0    # millirad/s -> rad/s
SCALE_POS = 1000.0     # mm -> m
SCALE_VEL = 1000.0     # mm/s -> m/s
SCALE_THRUST = 65535.0 # 0-65535 -> 0.0-1.0


def decode_attitude_packet(data: bytes) -> dict:
    """Decode attitude/rates packet (type 0x01).

    Format matches comms_actor.c telemetry_attitude_t:
      type(1) + gyro_xyz(6) + roll/pitch/yaw(6) = 13 bytes
    Note: No timestamp due to nRF51 16-byte syslink limit.
    """
    if len(data) < 13:
        return None

    _, gx, gy, gz, roll, pitch, yaw = struct.unpack(
        "<Bhhhhhh", data[:13]
    )

    return {
        "type": "attitude",
        "gyro_x": gx / SCALE_RATE,
        "gyro_y": gy / SCALE_RATE,
        "gyro_z": gz / SCALE_RATE,
        "roll": roll / SCALE_ANGLE,
        "pitch": pitch / SCALE_ANGLE,
        "yaw": yaw / SCALE_ANGLE,
    }


def decode_position_packet(data: bytes) -> dict:
    """Decode position/altitude packet (type 0x02).

    Format matches comms_actor.c telemetry_position_t:
      type(1) + alt(2) + vz(2) + vx(2) + vy(2) + thrust(2) + battery_mv(2) = 13 bytes
    Note: No timestamp due to nRF51 16-byte syslink limit.
    """
    if len(data) < 13:
        return None

    _, alt, vz, vx, vy, thrust, battery_mv = struct.unpack(
        "<BhhhhHH", data[:13]
    )

    return {
        "type": "position",
        "altitude": alt / SCALE_POS,
        "vz": vz / SCALE_VEL,
        "vx": vx / SCALE_VEL,
        "vy": vy / SCALE_VEL,
        "thrust": thrust / SCALE_THRUST,
        "battery_v": battery_mv / 1000.0,
    }


def decode_packet(data: bytes) -> dict:
    """Decode a telemetry packet based on type byte."""
    if not data or len(data) < 1:
        return None

    pkt_type = data[0]

    if pkt_type == PACKET_TYPE_ATTITUDE:
        return decode_attitude_packet(data)
    elif pkt_type == PACKET_TYPE_POSITION:
        return decode_position_packet(data)
    else:
        return None


def format_attitude(pkt: dict) -> str:
    """Format attitude packet for display."""
    return (
        f"ATT  "
        f"gyro=({pkt['gyro_x']:+6.2f}, {pkt['gyro_y']:+6.2f}, {pkt['gyro_z']:+6.2f}) rad/s  "
        f"rpy=({pkt['roll']:+5.2f}, {pkt['pitch']:+5.2f}, {pkt['yaw']:+5.2f}) rad"
    )


def format_position(pkt: dict) -> str:
    """Format position packet for display."""
    return (
        f"POS  "
        f"alt={pkt['altitude']:+5.2f}m  vz={pkt['vz']:+5.2f}m/s  "
        f"vxy=({pkt['vx']:+5.2f}, {pkt['vy']:+5.2f})m/s  "
        f"thrust={pkt['thrust']:.1%}  bat={pkt['battery_v']:.2f}V"
    )


def format_packet(pkt: dict) -> str:
    """Format any packet for display."""
    if pkt["type"] == "attitude":
        return format_attitude(pkt)
    elif pkt["type"] == "position":
        return format_position(pkt)
    return str(pkt)


def parse_uri(uri):
    """Parse radio URI to extract parameters.

    Format: radio://dongle/channel/datarate
    Example: radio://0/80/2M
    """
    parts = uri.replace("radio://", "").split("/")
    dongle_id = int(parts[0]) if len(parts) > 0 else 0
    channel = int(parts[1]) if len(parts) > 1 else 80
    datarate_str = parts[2] if len(parts) > 2 else "2M"

    datarate_map = {
        "250K": Crazyradio.DR_250KPS,
        "1M": Crazyradio.DR_1MPS,
        "2M": Crazyradio.DR_2MPS,
    }
    datarate = datarate_map.get(datarate_str.upper(), Crazyradio.DR_2MPS)

    return dongle_id, channel, datarate, datarate_str


class TelemetryReceiver:
    """Receives telemetry from Crazyflie via Crazyradio."""

    def __init__(self, uri: str = None):
        self.uri = uri
        self.radio = None
        self.running = False

        # Statistics
        self.packets_received = 0
        self.attitude_count = 0
        self.position_count = 0
        self.unknown_count = 0
        self.errors = 0
        self.start_time = None

    def scan(self):
        """Scan for available Crazyflies."""
        print("Scanning for Crazyflie...", file=sys.stderr)
        cflib.crtp.init_drivers()

        available = cflib.crtp.scan_interfaces()
        if not available:
            print("No Crazyflie found!", file=sys.stderr)
            return None

        print(f"Found {len(available)} Crazyflie(s):", file=sys.stderr)
        for i, uri in enumerate(available):
            print(f"  [{i}] {uri[0]}", file=sys.stderr)

        return available[0][0]

    def connect(self):
        """Initialize radio and connect to Crazyflie."""
        if not self.uri:
            self.uri = self.scan()
            if not self.uri:
                raise RuntimeError("No Crazyflie found")

        dongle_id, channel, datarate, datarate_str = parse_uri(self.uri)

        print(f"Connecting to channel {channel}, {datarate_str}...", file=sys.stderr)

        self.radio = Crazyradio(devid=dongle_id)
        if self.radio is None:
            raise RuntimeError("Could not find Crazyradio dongle")

        self.radio.set_channel(channel)
        self.radio.set_data_rate(datarate)
        self.radio.set_address((0xE7, 0xE7, 0xE7, 0xE7, 0xE7))

        print("Connected!", file=sys.stderr)

    def disconnect(self):
        """Disconnect from radio."""
        if self.radio:
            self.radio.close()
            self.radio = None

    def poll(self):
        """Send poll packet and receive telemetry.

        ESB protocol: we send a packet, drone responds with ACK + telemetry.
        Using Crazyradio direct: response.data contains raw bytes.
        """
        response = self.radio.send_packet((0xFF,))

        if response and response.ack and response.data:
            return bytes(response.data)
        return None

    def receive_loop(self, callback, csv_writer=None):
        """Main receive loop. Calls callback for each decoded packet."""
        self.running = True
        self.start_time = time.time()

        while self.running:
            try:
                data = self.poll()

                if data:
                    pkt = decode_packet(data)

                    if pkt:
                        self.packets_received += 1
                        if pkt["type"] == "attitude":
                            self.attitude_count += 1
                        elif pkt["type"] == "position":
                            self.position_count += 1

                        callback(pkt)

                        if csv_writer:
                            self._write_csv_row(csv_writer, pkt)
                    else:
                        # Unknown packet - show for debugging
                        self.unknown_count += 1
                        pkt_type = data[0] if data else 0
                        hex_dump = ' '.join(f'{b:02x}' for b in data[:min(len(data), 16)])
                        print(f"\nUnknown: type=0x{pkt_type:02x} len={len(data)} hex=[{hex_dump}]",
                              file=sys.stderr)

            except KeyboardInterrupt:
                self.running = False
            except Exception as e:
                print(f"Error: {e}", file=sys.stderr)
                self.errors += 1

    def _write_csv_row(self, writer, pkt):
        """Write packet to CSV file."""
        row = {
            "receive_time": datetime.now().isoformat(),
            "type": pkt["type"],
        }

        if pkt["type"] == "attitude":
            row.update({
                "gyro_x": pkt["gyro_x"],
                "gyro_y": pkt["gyro_y"],
                "gyro_z": pkt["gyro_z"],
                "roll": pkt["roll"],
                "pitch": pkt["pitch"],
                "yaw": pkt["yaw"],
            })
        elif pkt["type"] == "position":
            row.update({
                "altitude": pkt["altitude"],
                "vz": pkt["vz"],
                "vx": pkt["vx"],
                "vy": pkt["vy"],
                "thrust": pkt["thrust"],
                "battery_v": pkt["battery_v"],
            })

        writer.writerow(row)

    def print_stats(self):
        """Print reception statistics."""
        elapsed = time.time() - self.start_time if self.start_time else 0
        rate = self.packets_received / elapsed if elapsed > 0 else 0

        print("\n--- Statistics ---", file=sys.stderr)
        print(f"Duration: {elapsed:.1f}s", file=sys.stderr)
        print(f"Packets received: {self.packets_received} ({rate:.1f}/s)", file=sys.stderr)
        print(f"  Attitude: {self.attitude_count}", file=sys.stderr)
        print(f"  Position: {self.position_count}", file=sys.stderr)
        print(f"  Unknown: {self.unknown_count}", file=sys.stderr)
        print(f"Errors: {self.errors}", file=sys.stderr)

    def download_log(self, output_path: str) -> bool:
        """Download log file from drone.

        Sends CMD_REQUEST_LOG command and receives LOG_CHUNK packets until
        LOG_COMPLETE is received. Returns True on success.
        """
        print(f"Requesting log download...", file=sys.stderr)

        # Send request command
        response = self.radio.send_packet((CMD_REQUEST_LOG,))

        if not response or not response.ack:
            print("Error: No ACK for log request", file=sys.stderr)
            return False

        # Open output file
        with open(output_path, "wb") as f:
            chunks_received = 0
            expected_seq = 0
            total_bytes = 0

            while True:
                try:
                    # Poll for next chunk
                    data = self.poll()

                    if not data or len(data) < 1:
                        continue

                    pkt_type = data[0]

                    if pkt_type == PACKET_LOG_CHUNK:
                        if len(data) < 3 + LOG_CHUNK_DATA_SIZE:
                            print(f"Warning: Short chunk packet ({len(data)} bytes)",
                                  file=sys.stderr)
                            continue

                        sequence = struct.unpack("<H", data[1:3])[0]
                        chunk_data = data[3:3 + LOG_CHUNK_DATA_SIZE]

                        if sequence != expected_seq:
                            print(f"Warning: Sequence mismatch, expected {expected_seq}, "
                                  f"got {sequence}", file=sys.stderr)

                        f.write(chunk_data)
                        chunks_received += 1
                        total_bytes += len(chunk_data)
                        expected_seq = sequence + 1

                        # Progress indicator
                        if chunks_received % 100 == 0:
                            print(f"  Received {chunks_received} chunks "
                                  f"({total_bytes} bytes)...", file=sys.stderr)

                    elif pkt_type == PACKET_LOG_DONE:
                        if len(data) >= 3:
                            total_chunks = struct.unpack("<H", data[1:3])[0]
                            print(f"Download complete: {total_chunks} chunks, "
                                  f"{total_bytes} bytes", file=sys.stderr)
                        else:
                            print(f"Download complete: {chunks_received} chunks, "
                                  f"{total_bytes} bytes", file=sys.stderr)
                        break

                    elif pkt_type in (PACKET_TYPE_ATTITUDE, PACKET_TYPE_POSITION):
                        # Ignore telemetry packets during download
                        pass

                except KeyboardInterrupt:
                    print("\nDownload interrupted", file=sys.stderr)
                    return False
                except Exception as e:
                    print(f"Error during download: {e}", file=sys.stderr)
                    return False

        print(f"Log saved to {output_path}", file=sys.stderr)
        return True


def main():
    parser = argparse.ArgumentParser(
        description="Receive telemetry and download logs from Crazyflie via Crazyradio 2.0",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        "--uri", "-u", default=None,
        help="Crazyflie radio URI (auto-scan if not specified)"
    )
    parser.add_argument(
        "-o", "--output", type=str,
        help="Output CSV file for telemetry logging"
    )
    parser.add_argument(
        "--quiet", "-q", action="store_true",
        help="Suppress real-time output (only log to file)"
    )
    parser.add_argument(
        "--download-log", type=str, metavar="FILE",
        help="Download log file from drone flash storage"
    )

    args = parser.parse_args()

    # Setup CSV output
    csv_file = None
    csv_writer = None
    if args.output:
        csv_file = open(args.output, "w", newline="")
        fieldnames = [
            "receive_time", "type",
            "gyro_x", "gyro_y", "gyro_z", "roll", "pitch", "yaw",
            "altitude", "vz", "vx", "vy", "thrust", "battery_v"
        ]
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames, extrasaction="ignore")
        csv_writer.writeheader()
        print(f"Logging to {args.output}", file=sys.stderr)

    # Create receiver
    receiver = TelemetryReceiver(args.uri)

    try:
        receiver.connect()

        if args.download_log:
            # Log download mode
            success = receiver.download_log(args.download_log)
            sys.exit(0 if success else 1)
        else:
            # Telemetry receive mode
            def on_packet(pkt):
                if not args.quiet:
                    print(format_packet(pkt))

            print("Waiting for telemetry... (Ctrl+C to stop)", file=sys.stderr)
            receiver.receive_loop(on_packet, csv_writer)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        receiver.disconnect()
        if not args.download_log:
            receiver.print_stats()
        if csv_file:
            csv_file.close()


if __name__ == "__main__":
    main()
