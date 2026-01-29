#!/bin/bash
# SWO trace viewer for Crazyflie bringup
# Uses locally built stlink-tools (system version 1.8.0 has bugs)
#
# Usage:
#   ./st-trace.sh           # Run with 70s timeout (enough for all phases)
#   ./st-trace.sh -t 30     # Run with 30s timeout
#   ./st-trace.sh -t 0      # Run without timeout (Ctrl-C to stop)
#   ./st-trace.sh --help    # Show help

set -e

STLINK_DIR="$(cd "$(dirname "$0")/../../../../.." && pwd)/local/stlink/install/bin"
ST_TRACE="$STLINK_DIR/st-trace"

# Default timeout: 70 seconds covers all bringup phases including:
# - Motor test with 3s initial delay + 4x(1s+1.5s+1s) per motor + 2s all motors
# - Radio test with 5s timeout
# - Plus margin for flash/SD tests
DEFAULT_TIMEOUT=70

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Run SWO trace for Crazyflie bringup tests."
    echo ""
    echo "Options:"
    echo "  -t, --timeout SECS   Timeout in seconds (default: $DEFAULT_TIMEOUT)"
    echo "                       Use 0 for no timeout (Ctrl-C to stop)"
    echo "  -h, --help           Show this help"
    echo ""
    echo "Examples:"
    echo "  $0                   # Run full bringup test"
    echo "  $0 -t 30             # Run with 30s timeout"
    echo "  $0 -t 0              # Run indefinitely"
}

TIMEOUT=$DEFAULT_TIMEOUT

while [[ $# -gt 0 ]]; do
    case $1 in
        -t|--timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

if [ ! -x "$ST_TRACE" ]; then
    echo "Error: st-trace not found at $ST_TRACE"
    echo "Build it with: cd local/stlink && mkdir build && cd build && cmake .. && make"
    exit 1
fi

if [ "$TIMEOUT" -eq 0 ] 2>/dev/null; then
    # No timeout - run directly
    exec "$ST_TRACE" --clock=168m --trace=2m
else
    # Run with timeout
    echo "Running st-trace with ${TIMEOUT}s timeout (use -t 0 for no timeout)"
    timeout "$TIMEOUT" "$ST_TRACE" --clock=168m --trace=2m || {
        exit_code=$?
        if [ $exit_code -eq 124 ]; then
            echo ""
            echo "Trace completed (timeout reached)"
            exit 0
        fi
        exit $exit_code
    }
fi
