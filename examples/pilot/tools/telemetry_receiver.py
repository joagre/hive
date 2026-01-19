#!/usr/bin/env python3
"""
Receive telemetry and download logs from Crazyflie via Crazyradio PA.

Connects to the Crazyflie using ESB radio protocol and decodes binary
telemetry packets. Displays real-time data and optionally logs to CSV.
Can also download the binary flight log from flash storage.

Telemetry packet formats:
  Type 0x01 - Attitude/Rates (17 bytes):
    type(1) + timestamp_ms(4) + gyro_xyz(6) + roll/pitch/yaw(6)

  Type 0x02 - Position/Altitude (15 bytes):
    type(1) + timestamp_ms(4) + altitude(2) + vz(2) + vx(2) + vy(2) + thrust(2)

Log download packet formats:
  Type 0x10 - CMD_REQUEST_LOG (1 byte): Ground -> Drone to request log
  Type 0x11 - LOG_CHUNK (31 bytes): Drone -> Ground log data chunk
    type(1) + sequence(2) + data(28)
  Type 0x12 - LOG_COMPLETE (3 bytes): Drone -> Ground download complete
    type(1) + total_chunks(2)

Requirements:
  pip install cflib

Usage:
  ./telemetry_receiver.py                         # Display telemetry to stdout
  ./telemetry_receiver.py -o flight.csv           # Log telemetry to CSV file
  ./telemetry_receiver.py --download-log log.bin  # Download binary log file
  ./telemetry_receiver.py --uri radio://0/80/2M   # Custom radio URI

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
    from cflib.crtp.radiodriver import RadioDriver
    from cflib.drivers.crazyradio import Crazyradio
except ImportError:
    print("Error: cflib not installed. Run: pip install cflib", file=sys.stderr)
    sys.exit(1)

# Packet type identifiers - telemetry
PACKET_TYPE_ATTITUDE = 0x01
PACKET_TYPE_POSITION = 0x02

# Packet type identifiers - log download
CMD_REQUEST_LOG = 0x10
PACKET_LOG_CHUNK = 0x11
PACKET_LOG_COMPLETE = 0x12

# Log chunk data size
LOG_CHUNK_DATA_SIZE = 28

# Scale factors (inverse of transmitter)
SCALE_ANGLE = 1000.0   # millirad -> rad
SCALE_RATE = 1000.0    # millirad/s -> rad/s
SCALE_POS = 1000.0     # mm -> m
SCALE_VEL = 1000.0     # mm/s -> m/s
SCALE_THRUST = 65535.0 # 0-65535 -> 0.0-1.0


def decode_attitude_packet(data: bytes) -> dict:
    """Decode attitude/rates packet (type 0x01)."""
    if len(data) < 17:
        return None

    _, timestamp_ms, gx, gy, gz, roll, pitch, yaw = struct.unpack(
        "<BIHHHHHH", data[:17]
    )

    # Convert from unsigned to signed
    gx = gx if gx < 32768 else gx - 65536
    gy = gy if gy < 32768 else gy - 65536
    gz = gz if gz < 32768 else gz - 65536
    roll = roll if roll < 32768 else roll - 65536
    pitch = pitch if pitch < 32768 else pitch - 65536
    yaw = yaw if yaw < 32768 else yaw - 65536

    return {
        "type": "attitude",
        "timestamp_ms": timestamp_ms,
        "gyro_x": gx / SCALE_RATE,
        "gyro_y": gy / SCALE_RATE,
        "gyro_z": gz / SCALE_RATE,
        "roll": roll / SCALE_ANGLE,
        "pitch": pitch / SCALE_ANGLE,
        "yaw": yaw / SCALE_ANGLE,
    }


def decode_position_packet(data: bytes) -> dict:
    """Decode position/altitude packet (type 0x02)."""
    if len(data) < 15:
        return None

    _, timestamp_ms, alt, vz, vx, vy, thrust = struct.unpack(
        "<BIhhhhH", data[:15]
    )

    return {
        "type": "position",
        "timestamp_ms": timestamp_ms,
        "altitude": alt / SCALE_POS,
        "vz": vz / SCALE_VEL,
        "vx": vx / SCALE_VEL,
        "vy": vy / SCALE_VEL,
        "thrust": thrust / SCALE_THRUST,
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
        f"[{pkt['timestamp_ms']:8d}ms] ATT  "
        f"gyro=({pkt['gyro_x']:+6.2f}, {pkt['gyro_y']:+6.2f}, {pkt['gyro_z']:+6.2f}) rad/s  "
        f"rpy=({pkt['roll']:+5.2f}, {pkt['pitch']:+5.2f}, {pkt['yaw']:+5.2f}) rad"
    )


def format_position(pkt: dict) -> str:
    """Format position packet for display."""
    return (
        f"[{pkt['timestamp_ms']:8d}ms] POS  "
        f"alt={pkt['altitude']:+5.2f}m  vz={pkt['vz']:+5.2f}m/s  "
        f"vxy=({pkt['vx']:+5.2f}, {pkt['vy']:+5.2f})m/s  "
        f"thrust={pkt['thrust']:.1%}"
    )


def format_packet(pkt: dict) -> str:
    """Format any packet for display."""
    if pkt["type"] == "attitude":
        return format_attitude(pkt)
    elif pkt["type"] == "position":
        return format_position(pkt)
    return str(pkt)


class TelemetryReceiver:
    """Receives telemetry from Crazyflie via Crazyradio."""

    def __init__(self, uri: str = "radio://0/80/2M"):
        self.uri = uri
        self.radio = None
        self.running = False

        # Statistics
        self.packets_received = 0
        self.attitude_count = 0
        self.position_count = 0
        self.errors = 0
        self.start_time = None

    def connect(self):
        """Initialize radio and connect to Crazyflie."""
        cflib.crtp.init_drivers()

        # Parse URI to extract radio parameters
        # Format: radio://dongle/channel/datarate
        parts = self.uri.replace("radio://", "").split("/")
        dongle_id = int(parts[0]) if len(parts) > 0 else 0
        channel = int(parts[1]) if len(parts) > 1 else 80
        datarate_str = parts[2] if len(parts) > 2 else "2M"

        # Map datarate string to cflib constant
        datarate_map = {
            "250K": Crazyradio.DR_250KPS,
            "1M": Crazyradio.DR_1MPS,
            "2M": Crazyradio.DR_2MPS,
        }
        datarate = datarate_map.get(datarate_str.upper(), Crazyradio.DR_2MPS)

        # Open Crazyradio
        self.radio = Crazyradio(devid=dongle_id)
        if self.radio is None:
            raise RuntimeError("Could not find Crazyradio dongle")

        self.radio.set_channel(channel)
        self.radio.set_data_rate(datarate)
        self.radio.set_address((0xE7, 0xE7, 0xE7, 0xE7, 0xE7))  # Default Crazyflie address

        print(f"Connected to Crazyradio on channel {channel}, {datarate_str}", file=sys.stderr)

    def disconnect(self):
        """Disconnect from radio."""
        if self.radio:
            self.radio.close()
            self.radio = None

    def receive_loop(self, callback, csv_writer=None):
        """Main receive loop. Calls callback for each decoded packet."""
        self.running = True
        self.start_time = time.time()

        # Send empty packet to enable communication
        empty_packet = bytes([0xFF])

        while self.running:
            try:
                # Send empty packet, receive response
                # This is how syslink flow control works - we send to receive
                response = self.radio.send_packet(empty_packet)

                if response and response.ack and response.data:
                    data = bytes(response.data)
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
                        self.errors += 1

                # Small delay to avoid hammering the radio
                time.sleep(0.005)  # 5ms = 200Hz polling

            except KeyboardInterrupt:
                self.running = False
            except Exception as e:
                print(f"Error: {e}", file=sys.stderr)
                self.errors += 1

    def _write_csv_row(self, writer, pkt):
        """Write packet to CSV file."""
        row = {
            "receive_time": datetime.now().isoformat(),
            "timestamp_ms": pkt["timestamp_ms"],
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
        print(f"Errors: {self.errors}", file=sys.stderr)

    def download_log(self, output_path: str) -> bool:
        """Download binary log file from drone.

        Sends CMD_REQUEST_LOG command and receives LOG_CHUNK packets until
        LOG_COMPLETE is received. Returns True on success.
        """
        print(f"Requesting log download...", file=sys.stderr)

        # Send request command
        request = bytes([CMD_REQUEST_LOG])
        response = self.radio.send_packet(request)

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
                    # Send empty packet to receive next chunk
                    empty_packet = bytes([0xFF])
                    response = self.radio.send_packet(empty_packet)

                    if not response or not response.ack or not response.data:
                        time.sleep(0.005)
                        continue

                    data = bytes(response.data)
                    if len(data) < 1:
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

                    elif pkt_type == PACKET_LOG_COMPLETE:
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

                    time.sleep(0.005)

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
        description="Receive telemetry and download logs from Crazyflie via Crazyradio PA",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        "--uri", default="radio://0/80/2M",
        help="Crazyflie radio URI (default: radio://0/80/2M)"
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
        help="Download binary log file from drone flash storage"
    )

    args = parser.parse_args()

    # Setup CSV output
    csv_file = None
    csv_writer = None
    if args.output:
        csv_file = open(args.output, "w", newline="")
        fieldnames = [
            "receive_time", "timestamp_ms", "type",
            "gyro_x", "gyro_y", "gyro_z", "roll", "pitch", "yaw",
            "altitude", "vz", "vx", "vy", "thrust"
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
