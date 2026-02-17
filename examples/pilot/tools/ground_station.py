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

  Type 0x03 - tlog_state (29 bytes payload, 30 on wire):
    crtp(1) + type(1) + time_ms(4) + roll(2) + pitch(2) + yaw(2) +
    roll_rate(2) + pitch_rate(2) + yaw_rate(2) + x(2) + y(2) + altitude(2) +
    vx(2) + vy(2) + vz(2)

  Type 0x04 - tlog_sensors (27 bytes payload, 28 on wire):
    crtp(1) + type(1) + time_ms(4) + thrust(2) + target_x(2) + target_y(2) +
    target_z(2) + target_yaw(2) + gyro_x(2) + gyro_y(2) + gyro_z(2) +
    accel_x(2) + accel_y(2) + accel_z(2)

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
  sudo ./ground_station.py --go                    # Start flight (10s countdown)
  sudo ./ground_station.py --go -o flight.csv      # GO + record telemetry (single connection)
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
# 0x03/0x04 carry all 24 tlog.csv columns; old 0x01/0x02 retired.
PACKET_TYPE_TLOG_STATE = 0x03
PACKET_TYPE_TLOG_SENSORS = 0x04

# Packet type identifiers - log download (must match comms_actor.c)
CMD_REQUEST_LOG = 0x10
PACKET_LOG_CHUNK = 0x11
PACKET_LOG_DONE = 0x12

# Packet type identifiers - flight commands (must match comms_actor.c)
CMD_GO = 0x20
CMD_STATUS = 0x22
RESP_STATUS = 0x23

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
    # Altitude PID (12-17)
    12: "alt_kp",
    13: "alt_ki",
    14: "alt_kd",
    15: "alt_imax",
    16: "alt_omax",
    17: "vvel_damping",
    # Emergency limits (18-19)
    18: "emergency_tilt_limit",
    19: "emergency_alt_max",
    # Landing (20-21)
    20: "landing_descent_rate",
    21: "landing_velocity_gain",
    # Position control (22-24)
    22: "pos_kp",
    23: "pos_kd",
    24: "max_tilt_angle",
    # Complementary filter (25-29)
    25: "cf_alpha",
    26: "cf_mag_alpha",
    27: "cf_use_mag",
    28: "cf_accel_thresh_lo",
    29: "cf_accel_thresh_hi",
    # Waypoint navigation (30-34)
    30: "wp_tolerance_xy",
    31: "wp_tolerance_z",
    32: "wp_tolerance_yaw",
    33: "wp_tolerance_vel",
    34: "wp_hover_time_s",
    # Altitude Kalman filter (35-41)
    35: "kf_q_altitude",
    36: "kf_q_velocity",
    37: "kf_q_bias",
    38: "kf_r_altitude",
    39: "kf_p0_altitude",
    40: "kf_p0_velocity",
    41: "kf_p0_bias",
    # Horizontal velocity filter (42)
    42: "hvel_filter_alpha",
    # Flight manager lifecycle (43-44)
    43: "armed_countdown_s",
    44: "auto_go_delay_s",
    # Yaw rate PID (45-47)
    45: "rate_yaw_kp",
    46: "rate_yaw_ki",
    47: "rate_yaw_kd",
}

# Reverse lookup: name to ID
PARAM_IDS = {v: k for k, v in PARAM_NAMES.items()}

# Scale factors (inverse of transmitter)
SCALE_ANGLE = 1000.0   # millirad -> rad
SCALE_RATE = 1000.0    # millirad/s -> rad/s
SCALE_POS = 1000.0     # mm -> m
SCALE_VEL = 1000.0     # mm/s -> m/s
SCALE_THRUST = 65535.0 # 0-65535 -> 0.0-1.0
SCALE_ACCEL = 100.0    # cm/s^2 -> m/s^2

# Poll rate - matches drone's telemetry update rate (comms_actor.c)
POLL_RATE_HZ = 100
POLL_INTERVAL = 1.0 / POLL_RATE_HZ


def decode_tlog_state_packet(data: bytes) -> dict:
    """Decode tlog_state packet (type 0x03).

    Wire format: crtp(1) + type(1) + time_ms(4) + roll(2) + pitch(2) + yaw(2) +
                 roll_rate(2) + pitch_rate(2) + yaw_rate(2) + x(2) + y(2) +
                 altitude(2) + vx(2) + vy(2) + vz(2) = 30 bytes
    """
    if len(data) < 30:
        return None

    fields = struct.unpack("<BBIhhhhhhhhhhhh", data[:30])
    # fields: crtp, type, time_ms, roll, pitch, yaw, roll_rate, pitch_rate,
    #         yaw_rate, x, y, altitude, vx, vy, vz

    return {
        "type": "tlog_state",
        "time_ms": fields[2],
        "roll": fields[3] / SCALE_ANGLE,
        "pitch": fields[4] / SCALE_ANGLE,
        "yaw": fields[5] / SCALE_ANGLE,
        "roll_rate": fields[6] / SCALE_RATE,
        "pitch_rate": fields[7] / SCALE_RATE,
        "yaw_rate": fields[8] / SCALE_RATE,
        "x": fields[9] / SCALE_POS,
        "y": fields[10] / SCALE_POS,
        "altitude": fields[11] / SCALE_POS,
        "vx": fields[12] / SCALE_VEL,
        "vy": fields[13] / SCALE_VEL,
        "vz": fields[14] / SCALE_VEL,
    }


def decode_tlog_sensors_packet(data: bytes) -> dict:
    """Decode tlog_sensors packet (type 0x04).

    Wire format: crtp(1) + type(1) + time_ms(4) + thrust(2) + target_x(2) +
                 target_y(2) + target_z(2) + target_yaw(2) + gyro_x(2) +
                 gyro_y(2) + gyro_z(2) + accel_x(2) + accel_y(2) + accel_z(2) = 28 bytes
    """
    if len(data) < 28:
        return None

    fields = struct.unpack("<BBIHhhhhhhhhhh", data[:28])
    # fields: crtp, type, time_ms, thrust, target_x, target_y, target_z,
    #         target_yaw, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z

    return {
        "type": "tlog_sensors",
        "time_ms": fields[2],
        "thrust": fields[3] / SCALE_THRUST,
        "target_x": fields[4] / SCALE_POS,
        "target_y": fields[5] / SCALE_POS,
        "target_z": fields[6] / SCALE_POS,
        "target_yaw": fields[7] / SCALE_ANGLE,
        "gyro_x": fields[8] / SCALE_RATE,
        "gyro_y": fields[9] / SCALE_RATE,
        "gyro_z": fields[10] / SCALE_RATE,
        "accel_x": fields[11] / SCALE_ACCEL,
        "accel_y": fields[12] / SCALE_ACCEL,
        "accel_z": fields[13] / SCALE_ACCEL,
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

    if pkt_type == PACKET_TYPE_TLOG_STATE:
        return decode_tlog_state_packet(data)
    elif pkt_type == PACKET_TYPE_TLOG_SENSORS:
        return decode_tlog_sensors_packet(data)
    else:
        return None


def format_tlog(pkt: dict, latest_state: dict, latest_sensors: dict) -> str:
    """Format merged tlog data for display (one-line summary)."""
    t = pkt['time_ms'] / 1000.0

    roll = latest_state.get('roll', 0)
    pitch = latest_state.get('pitch', 0)
    yaw = latest_state.get('yaw', 0)
    alt = latest_state.get('altitude', 0)
    thrust = latest_sensors.get('thrust', 0)

    return (
        f"t={t:8.3f}  "
        f"rpy=({roll:+5.2f}, {pitch:+5.2f}, {yaw:+5.2f})  "
        f"alt={alt:+5.2f}m  "
        f"thrust={thrust:.1%}"
    )


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
        self.tlog_state_count = 0
        self.tlog_sensors_count = 0
        self.unknown_count = 0
        self.errors = 0
        self.start_time = None

        # Latest packet data for merging (packet A + B -> one CSV row)
        self.latest_state = {}
        self.latest_sensors = {}

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
                        if pkt["type"] == "tlog_state":
                            self.tlog_state_count += 1
                            self.latest_state = pkt
                        elif pkt["type"] == "tlog_sensors":
                            self.tlog_sensors_count += 1
                            self.latest_sensors = pkt

                        callback(pkt)

                        # Write CSV row on every sensors packet (B)
                        if csv_writer and pkt["type"] == "tlog_sensors":
                            self._write_csv_row(csv_writer)
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

    def _write_csv_row(self, writer):
        """Write merged tlog row to CSV (called on each sensors packet).

        Uses time_ms from sensors packet (B). State fields come from
        the latest state packet (A), which is at most one poll cycle stale.
        """
        s = self.latest_state
        r = self.latest_sensors

        row = {
            "time_ms": r.get("time_ms", 0),
            "roll": s.get("roll", 0),
            "pitch": s.get("pitch", 0),
            "yaw": s.get("yaw", 0),
            "roll_rate": s.get("roll_rate", 0),
            "pitch_rate": s.get("pitch_rate", 0),
            "yaw_rate": s.get("yaw_rate", 0),
            "x": s.get("x", 0),
            "y": s.get("y", 0),
            "altitude": s.get("altitude", 0),
            "vx": s.get("vx", 0),
            "vy": s.get("vy", 0),
            "vz": s.get("vz", 0),
            "thrust": r.get("thrust", 0),
            "target_x": r.get("target_x", 0),
            "target_y": r.get("target_y", 0),
            "target_z": r.get("target_z", 0),
            "target_yaw": r.get("target_yaw", 0),
            "gyro_x": r.get("gyro_x", 0),
            "gyro_y": r.get("gyro_y", 0),
            "gyro_z": r.get("gyro_z", 0),
            "accel_x": r.get("accel_x", 0),
            "accel_y": r.get("accel_y", 0),
            "accel_z": r.get("accel_z", 0),
        }

        writer.writerow(row)

    def print_stats(self):
        """Print reception statistics."""
        elapsed = time.time() - self.start_time if self.start_time else 0
        rate = self.packets_received / elapsed if elapsed > 0 else 0

        print("\n--- Statistics ---", file=sys.stderr)
        print(f"Duration: {elapsed:.1f}s", file=sys.stderr)
        print(f"Packets received: {self.packets_received} ({rate:.1f}/s)", file=sys.stderr)
        print(f"  tlog_state: {self.tlog_state_count}", file=sys.stderr)
        print(f"  tlog_sensors: {self.tlog_sensors_count}", file=sys.stderr)
        print(f"  Unknown: {self.unknown_count}", file=sys.stderr)
        print(f"Errors: {self.errors}", file=sys.stderr)

    def send_go(self) -> bool:
        """Send GO command to drone.

        This command tells the drone to start its flight sequence.
        The drone will then begin a armed countdown (default 10s) before flight.
        Returns True if ACK received.
        """
        print("Sending GO command...", file=sys.stderr)

        # Send GO command (with CRTP header)
        response = self.radio.send_packet((CRTP_HEADER_TELEMETRY, CMD_GO))

        if response and response.ack:
            print("GO command sent successfully!", file=sys.stderr)
            print("*** DRONE WILL START ARMED COUNTDOWN (default 10s) ***", file=sys.stderr)
            print("*** STEP BACK TO SAFE DISTANCE NOW ***", file=sys.stderr)
            return True
        else:
            print("Error: No ACK for GO command", file=sys.stderr)
            return False

    def request_status(self) -> bool:
        """Request flight manager status from drone.

        Sends CMD_STATUS, waits for RESP_STATUS response with state and
        countdown. Returns True if status received.
        """
        FM_STATES = {
            0: "IDLE", 1: "PREFLIGHT", 2: "ARMED",
            3: "FLYING", 4: "LANDING", 5: "LANDED",
        }
        print("Requesting status...", file=sys.stderr)
        response = self.radio.send_packet((CRTP_HEADER_TELEMETRY, CMD_STATUS))
        if not response or not response.ack:
            print("Error: No ACK for STATUS command", file=sys.stderr)
            return False
        # Poll for response packet
        for _ in range(50):
            response = self.radio.send_packet((CRTP_HEADER_TELEMETRY,))
            if response and response.ack and len(response.data) >= 3:
                if response.data[0] == RESP_STATUS:
                    state_num = response.data[1]
                    countdown = response.data[2]
                    state_name = FM_STATES.get(state_num, f"UNKNOWN({state_num})")
                    print(f"Flight manager state: {state_name}", file=sys.stderr)
                    if countdown > 0:
                        print(f"Countdown: {countdown}s", file=sys.stderr)
                    return True
            time.sleep(0.02)
        print("No status response received", file=sys.stderr)
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

                    elif pkt_type in (PACKET_TYPE_TLOG_STATE, PACKET_TYPE_TLOG_SENSORS):
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
        help="Send GO command to drone (starts armed countdown (default 10s), then flight)"
    )
    parser.add_argument(
        "--status", action="store_true",
        help="Query flight manager state (IDLE, PREFLIGHT, ARMED, FLYING, LANDING, LANDED)"
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
            "time_ms",
            "roll", "pitch", "yaw",
            "roll_rate", "pitch_rate", "yaw_rate",
            "x", "y", "altitude",
            "vx", "vy", "vz",
            "thrust",
            "target_x", "target_y", "target_z", "target_yaw",
            "gyro_x", "gyro_y", "gyro_z",
            "accel_x", "accel_y", "accel_z",
        ]
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        csv_writer.writeheader()
        print(f"Logging to {args.output}", file=sys.stderr)

    # Create receiver
    receiver = TelemetryReceiver(args.uri)

    try:
        receiver.connect()

        if args.go and not args.output:
            # GO-only mode - send go command and exit
            success = receiver.send_go()
            sys.exit(0 if success else 1)
        elif args.go and args.output:
            # GO + listen mode - send GO then record telemetry
            success = receiver.send_go()
            if not success:
                sys.exit(1)

            def on_packet(pkt):
                if not args.quiet:
                    print(format_tlog(pkt, receiver.latest_state,
                                      receiver.latest_sensors))

            print("Recording telemetry... (Ctrl+C to stop)", file=sys.stderr)
            receiver.receive_loop(on_packet, csv_writer)
        elif args.status:
            # Status mode - query flight manager state
            success = receiver.request_status()
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
                    print(format_tlog(pkt, receiver.latest_state,
                                      receiver.latest_sensors))

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
