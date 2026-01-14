#!/usr/bin/env python3
"""
Decode Hive binary log files to human-readable text.

Binary log format (little-endian):
  Offset  Size  Field
  0       2     magic       (0x47, 0x4C = "LG")
  2       2     seq         Monotonic sequence number
  4       4     timestamp   Microseconds since boot
  8       2     len         Payload length
  10      1     level       Log level (0=TRACE, 1=DEBUG, 2=INFO, 3=WARN, 4=ERROR)
  11      1     reserved
  12      len   payload     Log message text

Usage:
  ./decode_log.py flight.bin              # Decode to stdout
  ./decode_log.py flight.bin -o out.txt   # Decode to file
  ./decode_log.py flight.bin --raw        # Include raw hex dump
"""

import argparse
import struct
import sys
from pathlib import Path

MAGIC = 0x4C47  # "LG" in little-endian
HEADER_SIZE = 12

LEVEL_NAMES = ["TRACE", "DEBUG", "INFO", "WARN", "ERROR"]


def format_timestamp(us: int) -> str:
    """Format microseconds as MM:SS.mmm"""
    total_sec = us / 1_000_000
    minutes = int(total_sec // 60)
    seconds = total_sec % 60
    return f"{minutes:02d}:{seconds:06.3f}"


def decode_entry(data: bytes, offset: int) -> tuple:
    """
    Decode a single log entry.
    Returns (next_offset, entry_dict) or (next_offset, None) if invalid.
    """
    if offset + HEADER_SIZE > len(data):
        return len(data), None

    # Parse header: magic(2) + seq(2) + timestamp(4) + len(2) + level(1) + reserved(1) = 12 bytes
    magic, seq, timestamp, payload_len, level, _ = struct.unpack_from(
        "<HHIHBB", data, offset
    )

    if magic != MAGIC:
        # Try to find next valid entry
        return offset + 1, None

    payload_end = offset + HEADER_SIZE + payload_len
    if payload_end > len(data):
        # Truncated entry
        return len(data), None

    payload = data[offset + HEADER_SIZE : payload_end]

    try:
        text = payload.decode("utf-8", errors="replace")
    except Exception:
        text = payload.hex()

    entry = {
        "offset": offset,
        "seq": seq,
        "timestamp": timestamp,
        "level": level,
        "level_name": LEVEL_NAMES[level] if level < len(LEVEL_NAMES) else f"L{level}",
        "text": text,
        "raw": data[offset : payload_end],
    }

    return payload_end, entry


def decode_file(data: bytes, show_raw: bool = False) -> list:
    """Decode all entries from binary data."""
    entries = []
    offset = 0
    skipped = 0

    while offset < len(data):
        # Skip 0xFF bytes (erased flash)
        if data[offset] == 0xFF:
            offset += 1
            skipped += 1
            continue

        next_offset, entry = decode_entry(data, offset)

        if entry:
            entries.append(entry)
            if skipped > 0:
                print(f"[Skipped {skipped} erased bytes]", file=sys.stderr)
                skipped = 0
        else:
            skipped += (next_offset - offset)

        offset = next_offset

    if skipped > 0:
        print(f"[Skipped {skipped} bytes at end]", file=sys.stderr)

    return entries


def format_entry(entry: dict, show_raw: bool = False) -> str:
    """Format a log entry for display."""
    ts = format_timestamp(entry["timestamp"])
    level = entry["level_name"]
    seq = entry["seq"]
    text = entry["text"]

    line = f"[{ts}] {level:5} #{seq:05d}: {text}"

    if show_raw:
        raw_hex = entry["raw"].hex()
        line += f"\n    RAW: {raw_hex}"

    return line


def main():
    parser = argparse.ArgumentParser(
        description="Decode Hive binary log files to text",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("logfile", type=Path, help="Binary log file to decode")
    parser.add_argument("-o", "--output", type=Path, help="Output file (default: stdout)")
    parser.add_argument("--raw", action="store_true", help="Include raw hex dump")
    parser.add_argument("--stats", action="store_true", help="Show statistics")

    args = parser.parse_args()

    if not args.logfile.exists():
        print(f"Error: File not found: {args.logfile}", file=sys.stderr)
        sys.exit(1)

    data = args.logfile.read_bytes()
    print(f"Read {len(data)} bytes from {args.logfile}", file=sys.stderr)

    entries = decode_file(data, args.raw)
    print(f"Decoded {len(entries)} log entries", file=sys.stderr)

    output = sys.stdout if args.output is None else open(args.output, "w")

    try:
        for entry in entries:
            print(format_entry(entry, args.raw), file=output)

        if args.stats and entries:
            print("\n--- Statistics ---", file=output)
            print(f"Total entries: {len(entries)}", file=output)

            # Check for dropped entries (gaps in sequence numbers)
            dropped = 0
            for i in range(1, len(entries)):
                expected_seq = (entries[i-1]["seq"] + 1) & 0xFFFF
                if entries[i]["seq"] != expected_seq:
                    dropped += (entries[i]["seq"] - expected_seq) & 0xFFFF
            if dropped:
                print(f"Dropped entries (seq gaps): {dropped}", file=output)

            # Level distribution
            levels = {}
            for e in entries:
                name = e["level_name"]
                levels[name] = levels.get(name, 0) + 1
            print("Level distribution:", file=output)
            for level, count in sorted(levels.items()):
                print(f"  {level}: {count}", file=output)

            # Time range
            if len(entries) >= 2:
                first_ts = entries[0]["timestamp"]
                last_ts = entries[-1]["timestamp"]
                duration = (last_ts - first_ts) / 1_000_000
                print(f"Duration: {duration:.2f} seconds", file=output)

    finally:
        if args.output:
            output.close()


if __name__ == "__main__":
    main()
