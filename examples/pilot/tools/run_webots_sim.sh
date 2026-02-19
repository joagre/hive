#!/usr/bin/env bash
# Run a Webots simulation, wait for the flight to complete, and analyze results.
#
# Builds the pilot, launches Webots in fast/headless mode, waits for the
# controller to finish one flight, kills Webots, then prints the hive
# runtime log and runs flight_summary.py on the telemetry CSV.
#
# Usage:
#   tools/run_webots_sim.sh                          # Full 3D, noise=3
#   tools/run_webots_sim.sh --profile FIRST_TEST     # 6s hover test
#   tools/run_webots_sim.sh --noise 0                # Clean sensors
#   tools/run_webots_sim.sh --no-build               # Skip rebuild
#   tools/run_webots_sim.sh --timeline               # Show flight timeline
#   tools/run_webots_sim.sh --keep                   # Keep Webots running
#
# The script expects to be run from examples/pilot/.

set -euo pipefail

# ---- defaults ---------------------------------------------------------------

PROFILE="FULL_3D"
NOISE=3
MOTOR_LAG=20
DO_BUILD=1
SHOW_TIMELINE=0
KEEP_WEBOTS=0
EXTRA_MAKE_ARGS=""
WORLD="hover_test.wbt"

# ---- paths ------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PILOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLDS_DIR="$PILOT_DIR/worlds"
TLOG="/tmp/tlog.csv"
HIVE_LOG="/var/tmp/hive.log"
WEBOTS="${WEBOTS_HOME:-/usr/local/webots}/webots"

# ---- argument parsing -------------------------------------------------------

usage() {
    cat <<'USAGE'
Usage: tools/run_webots_sim.sh [OPTIONS]

Options:
  --profile PROFILE   Flight profile (default: FULL_3D)
                      Values: FIRST_TEST, ALTITUDE, FULL_3D, GROUND_TEST
  --noise N           Sensor noise level 0-5 (default: 3)
  --motor-lag MS      Motor time constant in ms (default: 20, 0=instant)
  --world FILE        World file in worlds/ (default: hover_test.wbt)
  --no-build          Skip the build step
  --timeline          Show flight timeline in summary
  --keep              Keep Webots running after flight (don't kill)
  --make-args "ARGS"  Extra arguments passed to make
  -h, --help          Show this help
USAGE
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile)   PROFILE="$2"; shift 2 ;;
        --noise)     NOISE="$2"; shift 2 ;;
        --motor-lag) MOTOR_LAG="$2"; shift 2 ;;
        --world)     WORLD="$2"; shift 2 ;;
        --no-build)  DO_BUILD=0; shift ;;
        --timeline)  SHOW_TIMELINE=1; shift ;;
        --keep)      KEEP_WEBOTS=1; shift ;;
        --make-args) EXTRA_MAKE_ARGS="$2"; shift 2 ;;
        -h|--help)   usage ;;
        *)           echo "Unknown option: $1"; usage ;;
    esac
done

FLIGHT_PROFILE="FLIGHT_PROFILE_${PROFILE}"
WORLD_PATH="$WORLDS_DIR/$WORLD"

if [[ ! -f "$WORLD_PATH" ]]; then
    echo "ERROR: World file not found: $WORLD_PATH"
    exit 1
fi

if [[ ! -x "$WEBOTS" ]] && [[ ! -f "$WEBOTS" ]]; then
    echo "ERROR: Webots not found at $WEBOTS"
    echo "Set WEBOTS_HOME to the Webots installation directory."
    exit 1
fi

# ---- build ------------------------------------------------------------------

if [[ "$DO_BUILD" -eq 1 ]]; then
    echo "=== Building pilot (profile=$PROFILE noise=$NOISE motor_lag=$MOTOR_LAG) ==="
    # shellcheck disable=SC2086
    make -C "$PILOT_DIR" clean 2>/dev/null || true
    # shellcheck disable=SC2086
    make -C "$PILOT_DIR" \
        FLIGHT_PROFILE="$FLIGHT_PROFILE" \
        SENSOR_NOISE="$NOISE" \
        MOTOR_LAG="$MOTOR_LAG" \
        $EXTRA_MAKE_ARGS \
        2>&1 | tail -3
    echo ""
fi

# ---- pre-flight cleanup ----------------------------------------------------

rm -f "$TLOG"
rm -f "$HIVE_LOG"

# Kill any lingering pilot or webots-bin processes from previous runs
pkill -f "controllers/pilot/pilot" 2>/dev/null || true
pkill -f "webots-bin" 2>/dev/null || true
sleep 0.5

# ---- launch Webots ----------------------------------------------------------

echo "=== Launching Webots (fast mode, no rendering) ==="
echo "    World: $WORLD"
echo "    Profile: $PROFILE"
echo ""

"$WEBOTS" --mode=fast --no-rendering --batch --stdout --stderr \
    "$WORLD_PATH" 2>&1 &
WEBOTS_PID=$!

# ---- wait for flight to complete -------------------------------------------

# Expected durations per profile (grace period + countdown + flight + landing)
case "$PROFILE" in
    GROUND_TEST) MAX_WAIT=60 ;;
    FIRST_TEST)  MAX_WAIT=60 ;;
    ALTITUDE)    MAX_WAIT=120 ;;
    FULL_3D)     MAX_WAIT=150 ;;
    *)           MAX_WAIT=120 ;;
esac

echo "Waiting for flight to complete (timeout: ${MAX_WAIT}s)..."

LANDED=0
for i in $(seq 1 "$MAX_WAIT"); do
    sleep 1

    # Check if the hive log contains the LANDED marker
    if [[ -f "$HIVE_LOG" ]] && grep -q "Flight complete - returning to IDLE" "$HIVE_LOG" 2>/dev/null; then
        # Give logger a moment to flush the CSV
        sleep 2
        LANDED=1
        echo "Flight complete after ${i}s."
        break
    fi

    # Check if webots died unexpectedly
    if ! kill -0 "$WEBOTS_PID" 2>/dev/null; then
        echo "Webots exited after ${i}s."
        break
    fi

    # Progress indicator every 15s
    if [[ $((i % 15)) -eq 0 ]]; then
        CSV_LINES=0
        if [[ -f "$TLOG" ]]; then
            CSV_LINES=$(wc -l < "$TLOG")
        fi
        echo "  ${i}s elapsed (CSV: ${CSV_LINES} lines)"
    fi
done

# ---- stop Webots ------------------------------------------------------------

if [[ "$KEEP_WEBOTS" -eq 0 ]]; then
    if kill -0 "$WEBOTS_PID" 2>/dev/null; then
        kill "$WEBOTS_PID" 2>/dev/null
        wait "$WEBOTS_PID" 2>/dev/null || true
    fi
fi

# ---- print results ----------------------------------------------------------

echo ""
echo "================================================================"
echo "  HIVE RUNTIME LOG"
echo "================================================================"
if [[ -f "$HIVE_LOG" ]]; then
    cat "$HIVE_LOG"
else
    echo "(no log file at $HIVE_LOG)"
fi

echo ""
echo "================================================================"
echo "  FLIGHT ANALYSIS"
echo "================================================================"
if [[ -f "$TLOG" ]]; then
    CSV_LINES=$(wc -l < "$TLOG")
    echo "Telemetry CSV: $TLOG ($CSV_LINES lines)"
    echo ""

    SUMMARY_ARGS=("$TLOG")
    if [[ "$SHOW_TIMELINE" -eq 1 ]]; then
        SUMMARY_ARGS+=("--timeline")
    fi

    python3 "$SCRIPT_DIR/flight_summary.py" "${SUMMARY_ARGS[@]}"
else
    echo "ERROR: No telemetry CSV at $TLOG"
    echo "The controller may not have started. Check Webots output above."
    exit 1
fi

# ---- exit status based on landing ------------------------------------------

if [[ "$LANDED" -eq 1 ]]; then
    exit 0
else
    echo ""
    echo "WARNING: Flight did not complete within ${MAX_WAIT}s."
    exit 1
fi
