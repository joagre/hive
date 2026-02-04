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

  Uses Crazyradio direct API with CRTP header for nRF51 compatibility.
  All packets have a CRTP header (0xA0 = port 10) as first byte.

Telemetry packet formats (max 31 bytes on wire = 30 payload + CRTP):
  The HAL layer prepends the CRTP header, so wire format is crtp(1) + payload.
  ESB max payload is 32 bytes.

  Type 0x01 - Attitude/Rates (17 bytes payload, 18 on wire):
    crtp(1) + type(1) + timestamp_ms(4) + gyro_xyz(6) + roll/pitch/yaw(6)

  Type 0x02 - Position/Altitude (17 bytes payload, 18 on wire):
    crtp(1) + type(1) + timestamp_ms(4) + altitude(2) + vz(2) + vx(2) + vy(2) + thrust(2) + battery_mv(2)

Log download packet formats (must match comms_actor.c):
  Type 0x10 - CMD_REQUEST_LOG (2 bytes): Ground -> Drone to request log
    crtp(1) + type(1)
  Type 0x11 - PACKET_LOG_CHUNK (31 bytes on wire): Drone -> Ground log data chunk
    crtp(1) + type(1) + sequence(2) + data(27)
  Type 0x12 - PACKET_LOG_DONE (4 bytes on wire): Drone -> Ground download complete
    crtp(1) + type(1) + total_chunks(2)

Requirements:
  pip install cflib

Usage:
  sudo ./ground_station.py                         # Display telemetry to stdout
  sudo ./ground_station.py -o flight.csv           # Log telemetry to CSV file
  sudo ./ground_station.py --go                    # Start flight (60s countdown)
  sudo ./ground_station.py --download-log flight.log  # Download log file
  sudo ./ground_station.py --uri radio://0/80/2M   # Custom radio URI
  sudo ./ground_station.py --list-params           # List all tunable parameters
  sudo ./ground_station.py --get-param rate_kp     # Get a parameter value
  sudo ./ground_station.py --set-param rate_kp 0.025  # Set a parameter

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

# CRTP header for custom telemetry port (must match comms_actor.c)
# The nRF51 expects CRTP-formatted packets. First byte is CRTP header.
CRTP_HEADER_TELEMETRY = 0xA0  # Port 10, channel 0

# Packet type identifiers - telemetry (byte after CRTP header)
PACKET_TYPE_ATTITUDE = 0x01
PACKET_TYPE_POSITION = 0x02

# Packet type identifiers - log download (must match comms_actor.c)
CMD_REQUEST_LOG = 0x10
PACKET_LOG_CHUNK = 0x11
PACKET_LOG_DONE = 0x12

# Packet type identifiers - flight go command (must match comms_actor.c)
CMD_GO = 0x20

# Packet type identifiers - parameter tuning (must match comms_actor.c)
CMD_SET_PARAM = 0x30
CMD_GET_PARAM = 0x31
CMD_LIST_PARAMS = 0x32
RESP_PARAM_ACK = 0x33
RESP_PARAM_VALUE = 0x34
RESP_PARAM_LIST = 0x35

# Log chunk data size (31 - 4 byte header = 27 bytes)
LOG_CHUNK_DATA_SIZE = 27

# Parameter IDs (must match tunable_params.h enum)
PARAM_NAMES = {
    # Rate PID (0-6)
    0: "rate_kp",
    1: "rate_ki",
    2: "rate_kd",
    3: "rate_imax",
    4: "rate_omax_roll",
    5: "rate_omax_pitch",
    6: "rate_omax_yaw",
    # Attitude PID (7-11)
    7: "att_kp",
    8: "att_ki",
    9: "att_kd",
    10: "att_imax",
    11: "att_omax",
    # Altitude PID (12-18)
    12: "alt_kp",
    13: "alt_ki",
    14: "alt_kd",
    15: "alt_imax",
    16: "alt_omax",
    17: "hover_thrust",
    18: "vvel_damping",
    # Emergency limits (19-20)
    19: "emergency_tilt_limit",
    20: "emergency_alt_max",
    # Landing (21-22)
    21: "landing_descent_rate",
    22: "landing_velocity_gain",
    # Position control (23-25)
    23: "pos_kp",
    24: "pos_kd",
    25: "max_tilt_angle",
    # Complementary filter (26-30)
    26: "cf_alpha",
    27: "cf_mag_alpha",
    28: "cf_use_mag",
    29: "cf_accel_thresh_lo",
    30: "cf_accel_thresh_hi",
    # Waypoint navigation (31-35)
    31: "wp_tolerance_xy",
    32: "wp_tolerance_z",
    33: "wp_tolerance_yaw",
    34: "wp_tolerance_vel",
    35: "wp_hover_time_s",
    # Flight manager (36)
    36: "thrust_ramp_ms",
}

# Reverse lookup: name to ID
PARAM_IDS = {v: k for k, v in PARAM_NAMES.items()}

# Scale factors (inverse of transmitter)
SCALE_ANGLE = 1000.0   # millirad -> rad
SCALE_RATE = 1000.0    # millirad/s -> rad/s

# Poll rate - matches drone's telemetry update rate (comms_actor.c)
POLL_RATE_HZ = 100
POLL_INTERVAL = 1.0 / POLL_RATE_HZ
SCALE_POS = 1000.0     # mm -> m
SCALE_VEL = 1000.0     # mm/s -> m/s
SCALE_THRUST = 65535.0 # 0-65535 -> 0.0-1.0


def decode_attitude_packet(data: bytes) -> dict:
    """Decode attitude/rates packet (type 0x01).

    Wire format: crtp(1) + type(1) + timestamp(4) + gyro_xyz(6) + roll/pitch/yaw(6) = 18 bytes
    The CRTP header is prepended by HAL, payload is 17 bytes.
    """
    if len(data) < 18:
        return None

    _, _, timestamp_ms, gx, gy, gz, roll, pitch, yaw = struct.unpack(
        "<BBIhhhhhh", data[:18]
    )

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
    """Decode position/altitude packet (type 0x02).

    Wire format: crtp(1) + type(1) + timestamp(4) + alt(2) + vz(2) + vx(2) + vy(2) + thrust(2) + battery_mv(2) = 18 bytes
    The CRTP header is prepended by HAL, payload is 17 bytes.
    """
    if len(data) < 18:
        return None

    _, _, timestamp_ms, alt, vz, vx, vy, thrust, battery_mv = struct.unpack(
        "<BBIhhhhHH", data[:18]
    )

    return {
        "type": "position",
        "timestamp_ms": timestamp_ms,
        "altitude": alt / SCALE_POS,
        "vz": vz / SCALE_VEL,
        "vx": vx / SCALE_VEL,
        "vy": vy / SCALE_VEL,
        "thrust": thrust / SCALE_THRUST,
        "battery_v": battery_mv / 1000.0,
    }


def decode_packet(data: bytes) -> dict:
    """Decode a telemetry packet based on type byte.

    Packet format: CRTP header (1 byte) + type (1 byte) + payload
    We skip the CRTP header and decode based on the type byte.
    """
    if not data or len(data) < 2:
        return None

    # Check CRTP header - should be our custom telemetry port
    crtp_header = data[0]
    if crtp_header != CRTP_HEADER_TELEMETRY:
        # Not our telemetry packet (might be link port response, etc.)
        return None

    pkt_type = data[1]

    if pkt_type == PACKET_TYPE_ATTITUDE:
        return decode_attitude_packet(data)
    elif pkt_type == PACKET_TYPE_POSITION:
        return decode_position_packet(data)
    else:
        return None


def format_attitude(pkt: dict) -> str:
    """Format attitude packet for display."""
    t = pkt['timestamp_ms'] / 1000.0
    return (
        f"ATT  t={t:8.3f}  "
        f"gyro=({pkt['gyro_x']:+6.2f}, {pkt['gyro_y']:+6.2f}, {pkt['gyro_z']:+6.2f}) rad/s  "
        f"rpy=({pkt['roll']:+5.2f}, {pkt['pitch']:+5.2f}, {pkt['yaw']:+5.2f}) rad"
    )


def format_position(pkt: dict) -> str:
    """Format position packet for display."""
    t = pkt['timestamp_ms'] / 1000.0
    return (
        f"POS  t={t:8.3f}  "
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
            loop_start = time.time()
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

            # Rate limit to match drone's telemetry update rate
            elapsed = time.time() - loop_start
            if elapsed < POLL_INTERVAL:
                time.sleep(POLL_INTERVAL - elapsed)

    def _write_csv_row(self, writer, pkt):
        """Write packet to CSV file."""
        row = {
            "receive_time": datetime.now().isoformat(),
            "type": pkt["type"],
            "timestamp_ms": pkt["timestamp_ms"],
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

    def send_go(self) -> bool:
        """Send GO command to drone.

        This command tells the drone to start its flight sequence.
        The drone will then begin a 60-second countdown before flight.
        Returns True if ACK received.
        """
        print("Sending GO command...", file=sys.stderr)

        # Send GO command (with CRTP header)
        response = self.radio.send_packet((CRTP_HEADER_TELEMETRY, CMD_GO))

        if response and response.ack:
            print("GO command sent successfully!", file=sys.stderr)
            print("*** DRONE WILL START 60-SECOND COUNTDOWN ***", file=sys.stderr)
            print("*** STEP BACK TO SAFE DISTANCE NOW ***", file=sys.stderr)
            return True
        else:
            print("Error: No ACK for GO command", file=sys.stderr)
            return False

    def set_param(self, name: str, value: float) -> bool:
        """Set a parameter on the drone.

        Args:
            name: Parameter name (e.g., "rate_kp")
            value: New value for the parameter

        Returns True if the parameter was set successfully.
        """
        if name not in PARAM_IDS:
            print(f"Error: Unknown parameter '{name}'", file=sys.stderr)
            print(f"Available parameters: {', '.join(sorted(PARAM_IDS.keys()))}", file=sys.stderr)
            return False

        param_id = PARAM_IDS[name]
        print(f"Setting {name} (id={param_id}) = {value}", file=sys.stderr)

        # Send SET_PARAM command: CRTP + cmd + id + value(float)
        value_bytes = struct.pack("<f", value)
        response = self.radio.send_packet(
            (CRTP_HEADER_TELEMETRY, CMD_SET_PARAM, param_id) + tuple(value_bytes)
        )

        if not response or not response.ack:
            print("Error: No ACK for SET_PARAM command", file=sys.stderr)
            return False

        # Wait for response packet
        for _ in range(10):
            data = self.poll()
            if data and len(data) >= 3:
                if data[0] == CRTP_HEADER_TELEMETRY and data[1] == RESP_PARAM_ACK:
                    status = data[2]
                    if status == 0:
                        print(f"Success: {name} = {value}", file=sys.stderr)
                        return True
                    else:
                        print(f"Error: Parameter rejected (validation failed)", file=sys.stderr)
                        return False

        print("Error: No response for SET_PARAM", file=sys.stderr)
        return False

    def get_param(self, name: str) -> float:
        """Get a parameter value from the drone.

        Args:
            name: Parameter name (e.g., "rate_kp")

        Returns the parameter value, or None on error.
        """
        if name not in PARAM_IDS:
            print(f"Error: Unknown parameter '{name}'", file=sys.stderr)
            print(f"Available parameters: {', '.join(sorted(PARAM_IDS.keys()))}", file=sys.stderr)
            return None

        param_id = PARAM_IDS[name]

        # Send GET_PARAM command: CRTP + cmd + id
        response = self.radio.send_packet(
            (CRTP_HEADER_TELEMETRY, CMD_GET_PARAM, param_id)
        )

        if not response or not response.ack:
            print("Error: No ACK for GET_PARAM command", file=sys.stderr)
            return None

        # Wait for response packet
        for _ in range(10):
            data = self.poll()
            if data and len(data) >= 7:
                if data[0] == CRTP_HEADER_TELEMETRY and data[1] == RESP_PARAM_VALUE:
                    resp_id = data[2]
                    value = struct.unpack("<f", bytes(data[3:7]))[0]
                    if resp_id == param_id:
                        print(f"{name} = {value:.6f}", file=sys.stderr)
                        return value

        print("Error: No response for GET_PARAM", file=sys.stderr)
        return None

    def list_params(self) -> dict:
        """List all parameters from the drone.

        Returns a dict of {name: value} for all parameters.
        """
        # Send LIST_PARAMS command
        response = self.radio.send_packet(
            (CRTP_HEADER_TELEMETRY, CMD_LIST_PARAMS)
        )

        if not response or not response.ack:
            print("Error: No ACK for LIST_PARAMS command", file=sys.stderr)
            return {}

        params = {}
        received_ids = set()

        # Receive param list packets (may come in multiple batches)
        for _ in range(50):  # Enough polls to get all params
            data = self.poll()
            if not data or len(data) < 7:
                continue

            if data[0] == CRTP_HEADER_TELEMETRY and data[1] == RESP_PARAM_LIST:
                # Parse param list: type(1) + offset(1) + [id(1) + value(4)] * 5
                offset = data[2]
                idx = 3
                while idx + 5 <= len(data):
                    param_id = data[idx]
                    value = struct.unpack("<f", bytes(data[idx+1:idx+5]))[0]
                    if param_id in PARAM_NAMES:
                        params[PARAM_NAMES[param_id]] = value
                        received_ids.add(param_id)
                    idx += 5

                # Check if we have all params
                if len(received_ids) >= len(PARAM_NAMES):
                    break

        # Print all params
        print("\nTunable Parameters:", file=sys.stderr)
        print("-" * 40, file=sys.stderr)
        for name in sorted(params.keys()):
            print(f"  {name:25s} = {params[name]:.6f}", file=sys.stderr)
        print("-" * 40, file=sys.stderr)
        print(f"Total: {len(params)} parameters", file=sys.stderr)

        return params

    def download_log(self, output_path: str) -> bool:
        """Download log file from drone.

        Sends CMD_REQUEST_LOG command and receives LOG_CHUNK packets until
        LOG_COMPLETE is received. Returns True on success.
        """
        print(f"Requesting log download...", file=sys.stderr)

        # Send request command (with CRTP header)
        response = self.radio.send_packet((CRTP_HEADER_TELEMETRY, CMD_REQUEST_LOG))

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

                    if not data or len(data) < 2:
                        continue

                    # Skip CRTP header (byte 0), type is in byte 1
                    crtp_header = data[0]
                    pkt_type = data[1]

                    # Ignore packets that aren't from our telemetry port
                    if crtp_header != CRTP_HEADER_TELEMETRY:
                        continue

                    if pkt_type == PACKET_LOG_CHUNK:
                        if len(data) < 4 + LOG_CHUNK_DATA_SIZE:
                            print(f"Warning: Short chunk packet ({len(data)} bytes)",
                                  file=sys.stderr)
                            continue

                        sequence = struct.unpack("<H", data[2:4])[0]
                        chunk_data = data[4:4 + LOG_CHUNK_DATA_SIZE]

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
                        if len(data) >= 4:
                            total_chunks = struct.unpack("<H", data[2:4])[0]
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
    parser.add_argument(
        "--go", action="store_true",
        help="Send GO command to drone (starts 60-second countdown, then flight)"
    )
    parser.add_argument(
        "--set-param", nargs=2, metavar=("NAME", "VALUE"),
        help="Set a parameter (e.g., --set-param rate_kp 0.025)"
    )
    parser.add_argument(
        "--get-param", metavar="NAME",
        help="Get a parameter value (e.g., --get-param rate_kp)"
    )
    parser.add_argument(
        "--list-params", action="store_true",
        help="List all tunable parameters with current values"
    )

    args = parser.parse_args()

    # Setup CSV output
    csv_file = None
    csv_writer = None
    if args.output:
        csv_file = open(args.output, "w", newline="")
        fieldnames = [
            "receive_time", "type", "timestamp_ms",
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

        if args.go:
            # GO mode - send go command and exit
            success = receiver.send_go()
            sys.exit(0 if success else 1)
        elif args.set_param:
            # Set param mode
            name, value_str = args.set_param
            try:
                value = float(value_str)
            except ValueError:
                print(f"Error: Invalid value '{value_str}'", file=sys.stderr)
                sys.exit(1)
            success = receiver.set_param(name, value)
            sys.exit(0 if success else 1)
        elif args.get_param:
            # Get param mode
            value = receiver.get_param(args.get_param)
            if value is not None:
                print(f"{args.get_param} = {value}")
                sys.exit(0)
            else:
                sys.exit(1)
        elif args.list_params:
            # List params mode
            params = receiver.list_params()
            sys.exit(0 if params else 1)
        elif args.download_log:
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
