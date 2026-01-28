#!/bin/bash
# SWO trace viewer for Crazyflie bringup
# Uses locally built stlink-tools (system version 1.8.0 has bugs)

STLINK_DIR="$(cd "$(dirname "$0")/../../../../.." && pwd)/local/stlink/install/bin"
ST_TRACE="$STLINK_DIR/st-trace"

if [ ! -x "$ST_TRACE" ]; then
    echo "Error: st-trace not found at $ST_TRACE"
    echo "Build it with: cd local/stlink && mkdir build && cd build && cmake .. && make"
    exit 1
fi

exec "$ST_TRACE" --clock=168m --trace=2m "$@"
