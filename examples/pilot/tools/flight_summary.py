#!/usr/bin/env python3
"""
Flight summary - quick post-flight analysis.

Prints key metrics from a telemetry CSV without needing an LLM to sift
through data. Covers liftoff detection, hover stability, crash analysis,
drift, and post-landing estimator health.

Usage:
    python3 flight_summary.py flight_test23.csv
    python3 flight_summary.py flight_test23.csv --timeline
"""

import argparse
import csv
import math
import sys


def load(path):
    """Load telemetry CSV. Returns list of dicts with numeric values."""
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def analyze(data):
    """Compute all flight metrics."""
    r = {}
    if not data:
        return {'error': 'No data'}

    r['total_samples'] = len(data)
    r['time_span_s'] = (data[-1]['time_ms'] - data[0]['time_ms']) / 1000

    # Find flight window (thrust > 0)
    flight = [d for d in data if d['thrust'] > 0.001]
    if not flight:
        r['flew'] = False
        return r
    r['flew'] = True

    t0 = flight[0]['time_ms']
    t1 = flight[-1]['time_ms']
    r['flight_start_s'] = t0 / 1000
    r['flight_end_s'] = t1 / 1000
    r['flight_duration_s'] = (t1 - t0) / 1000

    # Target altitude (first non-zero target for display)
    targets = [d['target_z'] for d in data if d['target_z'] > 0.01]
    r['target_alt'] = targets[0] if targets else 0.0

    # --- Per-waypoint phase detection ---
    # Detect phases by tracking target changes (target_x, target_y, target_z)
    phases = []
    cur_target = None
    phase_start = 0
    for i, d in enumerate(flight):
        t = (d['target_x'], d['target_y'], d['target_z'])
        if cur_target is None or t != cur_target:
            if cur_target is not None and cur_target[2] > 0.01:
                phases.append({
                    'start': phase_start, 'end': i,
                    'target_x': cur_target[0], 'target_y': cur_target[1],
                    'target_z': cur_target[2],
                })
            cur_target = t
            phase_start = i
    # Close final phase
    if cur_target is not None and cur_target[2] > 0.01:
        phases.append({
            'start': phase_start, 'end': len(flight),
            'target_x': cur_target[0], 'target_y': cur_target[1],
            'target_z': cur_target[2],
        })

    # Analyze each waypoint phase
    wp_metrics = []
    for ph in phases:
        seg = flight[ph['start']:ph['end']]
        if len(seg) < 5:
            continue
        tz = ph['target_z']
        alts = [d['altitude'] for d in seg]
        max_alt = max(alts)
        overshoot_pct = (max_alt - tz) / tz * 100 if tz > 0.01 else 0.0

        # Steady-state: last 40% of phase (after settling)
        ss_start = len(seg) * 6 // 10
        ss_seg = seg[ss_start:]
        if len(ss_seg) > 2:
            ss_alts = [d['altitude'] for d in ss_seg]
            ss_mean = sum(ss_alts) / len(ss_alts)
            ss_err_pct = (ss_mean - tz) / tz * 100 if tz > 0.01 else 0.0
        else:
            ss_mean = 0.0
            ss_err_pct = 0.0

        # Per-waypoint XY tracking error
        tx, ty = ph['target_x'], ph['target_y']
        xy_errs = [math.sqrt((d['x'] - tx)**2 + (d['y'] - ty)**2)
                    for d in seg]
        max_xy_err = max(xy_errs)

        # Steady-state XY error
        ss_xy_errs = [math.sqrt((d['x'] - tx)**2 + (d['y'] - ty)**2)
                       for d in ss_seg] if len(ss_seg) > 2 else [0.0]
        ss_xy_mean = sum(ss_xy_errs) / len(ss_xy_errs)

        wp_metrics.append({
            'target_x': tx, 'target_y': ty, 'target_z': tz,
            'max_alt': max_alt, 'overshoot_pct': overshoot_pct,
            'ss_mean_alt': ss_mean, 'ss_err_pct': ss_err_pct,
            'max_xy_err': max_xy_err, 'ss_xy_mean': ss_xy_mean,
            'duration_s': (seg[-1]['time_ms'] - seg[0]['time_ms']) / 1000,
        })

    r['waypoint_metrics'] = wp_metrics
    if wp_metrics:
        r['worst_overshoot_pct'] = max(m['overshoot_pct'] for m in wp_metrics)
        r['worst_ss_err_pct'] = max(abs(m['ss_err_pct']) for m in wp_metrics)
        r['worst_xy_err'] = max(m['ss_xy_mean'] for m in wp_metrics)
    else:
        r['worst_overshoot_pct'] = 0.0
        r['worst_ss_err_pct'] = 0.0
        r['worst_xy_err'] = 0.0

    # --- Liftoff validation ---
    # Detect KF drift: if altitude was already elevated before thrust started,
    # the altitude data is unreliable (propwash, baro noise, etc.)
    pre_flight = [d for d in data if d['time_ms'] < t0 and d['thrust'] < 0.001]
    if len(pre_flight) > 1:
        initial_alt = pre_flight[0]['altitude']
        pre_thrust_alt = pre_flight[-1]['altitude']
        r['pre_flight_drift'] = pre_thrust_alt - initial_alt
    else:
        initial_alt = flight[0]['altitude']
        r['pre_flight_drift'] = 0.0

    # Altitude gain above pre-thrust baseline during the thrust phase.
    # Real liftoff produces >0.15m gain; KF drift on the ground is smaller
    # or was already present before thrust.
    baseline = pre_flight[-1]['altitude'] if pre_flight else flight[0]['altitude']
    flight_alts = [d['altitude'] for d in flight]
    altitude_gain = max(flight_alts) - baseline
    r['altitude_gain'] = altitude_gain
    r['baseline_alt'] = baseline

    # Cross-check with vertical velocity: real liftoff produces >0.1 m/s upward
    max_vz = max(d['vz'] for d in flight)
    r['max_vz_in_flight'] = max_vz

    # Liftoff validation. If KF drifted >0.1m before thrust even started,
    # altitude and velocity data are unreliable (propwash, baro noise,
    # rangefinder blocked). Cannot trust them for liftoff detection.
    if abs(r.get('pre_flight_drift', 0)) > 0.1:
        r['liftoff_validated'] = False
    else:
        r['liftoff_validated'] = (altitude_gain > 0.15 and max_vz > 0.1)

    # --- Liftoff detection ---
    # Find ramp phase: thrust increasing monotonically from first > 0
    ramp_end_idx = 0
    for i in range(1, len(flight)):
        if flight[i]['thrust'] < flight[i-1]['thrust'] - 0.01:
            ramp_end_idx = i
            break

    if ramp_end_idx > 0:
        ramp = flight[:ramp_end_idx]
        r['ramp_duration_s'] = (ramp[-1]['time_ms'] - ramp[0]['time_ms']) / 1000
        r['discovered_hover_thrust'] = ramp[-1]['thrust']
        r['liftoff_alt'] = ramp[-1]['altitude']
        r['ramp_start_thrust'] = ramp[0]['thrust']
    else:
        r['ramp_duration_s'] = 0
        r['discovered_hover_thrust'] = flight[0]['thrust']
        r['liftoff_alt'] = flight[0]['altitude']

    # --- Altitude metrics ---
    r['max_alt'] = max(d['altitude'] for d in flight)
    r['max_alt_time_s'] = next(
        d['time_ms'] / 1000 for d in flight if d['altitude'] == r['max_alt']
    )
    r['min_alt_in_flight'] = min(d['altitude'] for d in flight)

    # --- Thrust metrics ---
    thrusts = [d['thrust'] for d in flight]
    r['thrust_mean'] = sum(thrusts) / len(thrusts)
    r['thrust_min'] = min(thrusts)
    r['thrust_max'] = max(thrusts)
    r['thrust_std'] = (sum((t - r['thrust_mean'])**2 for t in thrusts) / len(thrusts)) ** 0.5
    r['thrust_saturated_pct'] = 100 * sum(1 for t in thrusts if t >= 0.99) / len(thrusts)

    # Thrust oscillation: count direction changes
    direction_changes = 0
    for i in range(2, len(thrusts)):
        d1 = thrusts[i-1] - thrusts[i-2]
        d2 = thrusts[i] - thrusts[i-1]
        if d1 * d2 < 0 and abs(d1) > 0.02 and abs(d2) > 0.02:
            direction_changes += 1
    r['thrust_oscillations'] = direction_changes
    r['thrust_osc_per_sec'] = direction_changes / max(r['flight_duration_s'], 0.1)

    # --- Attitude metrics ---
    r['max_roll_deg'] = max(abs(d['roll']) for d in flight) * 57.3
    r['max_pitch_deg'] = max(abs(d['pitch']) for d in flight) * 57.3
    r['max_tilt_deg'] = max(
        math.sqrt(d['roll']**2 + d['pitch']**2) * 57.3 for d in flight
    )

    # --- Crash detection ---
    crash_threshold = 0.78  # 45 degrees in radians
    r['crashed'] = False
    for d in flight:
        if abs(d['roll']) > crash_threshold or abs(d['pitch']) > crash_threshold:
            r['crashed'] = True
            r['crash_time_s'] = d['time_ms'] / 1000
            r['crash_roll_deg'] = d['roll'] * 57.3
            r['crash_pitch_deg'] = d['pitch'] * 57.3
            break

    # --- XY tracking error (per-waypoint) ---
    r['x_start'] = flight[0]['x']
    r['y_start'] = flight[0]['y']
    r['x_end'] = flight[-1]['x']
    r['y_end'] = flight[-1]['y']
    # Max instantaneous tracking error from current waypoint target
    r['max_xy_tracking_err'] = max(
        math.sqrt((d['x'] - d['target_x'])**2 + (d['y'] - d['target_y'])**2)
        for d in flight
    )
    # Steady-state XY error (used for verdict)
    if r.get('worst_xy_err', 0) > 0:
        r['ss_xy_tracking_err'] = r['worst_xy_err']
    else:
        r['ss_xy_tracking_err'] = r['max_xy_tracking_err']

    # --- Post-landing estimator health ---
    post = [d for d in data if d['time_ms'] > t1 and d['thrust'] < 0.001]
    if len(post) > 10:
        post_alts = [d['altitude'] for d in post]
        r['post_alt_mean'] = sum(post_alts) / len(post_alts)
        r['post_alt_min'] = min(post_alts)
        r['post_alt_max'] = max(post_alts)
        r['post_alt_drift'] = post_alts[-1] - post_alts[0]
        r['post_duration_s'] = (post[-1]['time_ms'] - post[0]['time_ms']) / 1000

        # Check for upside-down (roll near +/- pi)
        post_rolls = [d['roll'] for d in post[-10:]]
        avg_roll = sum(post_rolls) / len(post_rolls)
        r['post_upside_down'] = abs(abs(avg_roll) - math.pi) < 0.3
    else:
        r['post_alt_mean'] = None

    return r


def print_summary(r, path):
    """Print formatted flight summary."""
    print(f"=== Flight Summary: {path} ===\n")

    if r.get('error'):
        print(f"Error: {r['error']}")
        return

    if not r.get('flew'):
        print("No flight detected (thrust never exceeded 0)")
        return

    # Flight basics
    print(f"Flight:    {r['flight_start_s']:.1f}s - {r['flight_end_s']:.1f}s "
          f"({r['flight_duration_s']:.1f}s)")
    print(f"Target:    {r['target_alt']:.2f}m")

    # Liftoff validation
    if not r.get('liftoff_validated'):
        print(f"\n** NO LIFTOFF - motors ran but drone stayed on ground **")
        print(f"  Thrust commanded but altitude gain: {r['altitude_gain']:.3f}m "
              f"(need >0.15m)")
        print(f"  Max vertical velocity: {r['max_vz_in_flight']:.3f} m/s "
              f"(need >0.1)")
        if abs(r.get('pre_flight_drift', 0)) > 0.05:
            print(f"  KF drift before thrust: {r['pre_flight_drift']:+.3f}m "
                  f"(altitude data unreliable)")
        print(f"\nVerdict: NO LIFTOFF")
        return

    # Liftoff
    print(f"\nLiftoff detection:")
    print(f"  Discovered hover thrust: {r['discovered_hover_thrust']:.3f} "
          f"({r['discovered_hover_thrust']*100:.1f}%)")
    print(f"  Altitude at detection:   {r['liftoff_alt']:.3f}m")
    if r['ramp_duration_s'] > 0:
        print(f"  Ramp duration:           {r['ramp_duration_s']:.2f}s")

    # Altitude (per-waypoint)
    print(f"\nAltitude:")
    print(f"  Max: {r['max_alt']:.3f}m at t={r['max_alt_time_s']:.1f}s")
    print(f"  Min (in flight): {r['min_alt_in_flight']:.3f}m")
    wp_metrics = r.get('waypoint_metrics', [])
    if len(wp_metrics) == 1:
        m = wp_metrics[0]
        print(f"  Overshoot: {m['overshoot_pct']:+.0f}% "
              f"(peak {m['max_alt']:.3f}m vs target {m['target_z']:.2f}m)")
        print(f"  Steady-state error: {m['ss_err_pct']:+.1f}% "
              f"(mean {m['ss_mean_alt']:.3f}m)")
    elif len(wp_metrics) > 1:
        print(f"  Per-waypoint analysis ({len(wp_metrics)} waypoints):")
        for i, m in enumerate(wp_metrics):
            label = f"WP{i+1}"
            pos = f"({m['target_x']:.1f},{m['target_y']:.1f},{m['target_z']:.1f})"
            print(f"    {label} {pos}: overshoot {m['overshoot_pct']:+.0f}%, "
                  f"SS err {m['ss_err_pct']:+.1f}%, "
                  f"XY err {m['ss_xy_mean']:.3f}m")
        print(f"  Worst overshoot: {r['worst_overshoot_pct']:+.0f}%")
        print(f"  Worst SS error: {r['worst_ss_err_pct']:.1f}%")
    elif r['target_alt'] > 0:
        overshoot = (r['max_alt'] - r['target_alt']) / r['target_alt'] * 100
        print(f"  Overshoot: {overshoot:+.0f}%")

    # Thrust
    print(f"\nThrust:")
    print(f"  Mean: {r['thrust_mean']*100:.1f}%  "
          f"Range: [{r['thrust_min']*100:.1f}%, {r['thrust_max']*100:.1f}%]  "
          f"Std: {r['thrust_std']*100:.1f}%")
    print(f"  Saturation (>=99%%): {r['thrust_saturated_pct']:.0f}% of flight")
    print(f"  Oscillations: {r['thrust_oscillations']} "
          f"({r['thrust_osc_per_sec']:.1f}/s)")
    if r['thrust_osc_per_sec'] > 5:
        print(f"  ** WARNING: High oscillation rate - PID may be unstable **")

    # Attitude
    print(f"\nAttitude:")
    print(f"  Max roll: {r['max_roll_deg']:.1f} deg  "
          f"Max pitch: {r['max_pitch_deg']:.1f} deg  "
          f"Max tilt: {r['max_tilt_deg']:.1f} deg")

    # Crash
    if r['crashed']:
        print(f"\n** CRASH at t={r['crash_time_s']:.1f}s **")
        print(f"  Roll: {r['crash_roll_deg']:.1f} deg  "
              f"Pitch: {r['crash_pitch_deg']:.1f} deg")

    # XY tracking error
    print(f"\nPosition tracking:")
    if wp_metrics:
        print(f"  Worst per-WP XY error: {r['worst_xy_err']:.3f}m")
    else:
        print(f"  Max XY error: {r['max_xy_tracking_err']:.3f}m")
    print(f"  End pos: ({r['x_end']:.3f}, {r['y_end']:.3f})m")

    # Post-landing
    if r.get('post_alt_mean') is not None:
        print(f"\nPost-landing ({r['post_duration_s']:.1f}s):")
        print(f"  Altitude: mean={r['post_alt_mean']:.3f}m  "
              f"range=[{r['post_alt_min']:.3f}, {r['post_alt_max']:.3f}]m  "
              f"drift={r['post_alt_drift']:+.3f}m")
        if r.get('post_upside_down'):
            print(f"  ** Drone is UPSIDE DOWN **")

    # Overall verdict
    print(f"\nVerdict: ", end='')
    issues = []
    if r['crashed']:
        issues.append("CRASHED")
    if r.get('post_upside_down'):
        issues.append("FLIPPED")
    if r['thrust_osc_per_sec'] > 5:
        issues.append("UNSTABLE THRUST")
    if r['thrust_saturated_pct'] > 30:
        issues.append("THRUST SATURATED")
    if r['ss_xy_tracking_err'] > 0.5:
        issues.append(f"XY ERR {r['ss_xy_tracking_err']:.1f}m")
    if r.get('post_alt_mean') is not None and abs(r['post_alt_drift']) > 0.5:
        issues.append("KF DRIFT")

    if issues:
        print(", ".join(issues))
    else:
        print("OK")


def print_timeline(data, interval=0.5):
    """Print sampled timeline during flight."""
    flight = [d for d in data if d['thrust'] > 0.001]
    if not flight:
        return

    print(f"\nTimeline (every {interval}s during flight):")
    print(f"{'Time':>7} {'Alt':>6} {'Vz':>6} {'Thrust':>7} "
          f"{'Roll':>6} {'Pitch':>6} {'X':>6} {'Y':>6}")
    print("-" * 62)

    t0 = flight[0]['time_ms']
    next_print = t0

    for d in flight:
        if d['time_ms'] >= next_print:
            elapsed = (d['time_ms'] - t0) / 1000
            print(f"{elapsed:6.2f}s {d['altitude']:6.3f} {d['vz']:+6.2f} "
                  f"{d['thrust']*100:6.1f}% "
                  f"{d['roll']*57.3:+6.1f} {d['pitch']*57.3:+6.1f} "
                  f"{d['x']:+6.3f} {d['y']:+6.3f}")
            next_print = d['time_ms'] + interval * 1000


def main():
    p = argparse.ArgumentParser(description='Flight summary analysis')
    p.add_argument('file', help='Telemetry CSV file')
    p.add_argument('--timeline', '-t', action='store_true',
                   help='Show flight timeline')
    p.add_argument('--interval', type=float, default=0.5,
                   help='Timeline interval in seconds (default: 0.5)')
    args = p.parse_args()

    try:
        data = load(args.file)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    r = analyze(data)
    print_summary(r, args.file)

    if args.timeline:
        print_timeline(data, args.interval)


if __name__ == '__main__':
    main()
