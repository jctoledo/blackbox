#!/usr/bin/env python3
"""
Debug track recorder issues by analyzing CSV telemetry data.
Looks for position jumps, speed/position correlation, and other anomalies.
"""

import sys
import csv
import math
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional, Tuple

@dataclass
class Sample:
    timestamp_ms: int
    gps_lat: float
    gps_lon: float
    gps_valid: bool
    speed_kmh: float
    ax: float
    ay: float
    az: float
    wz: float
    # EKF fields (may be 0 if not present in old CSVs)
    ekf_x: float = 0.0
    ekf_y: float = 0.0
    ekf_yaw: float = 0.0
    has_ekf: bool = False
    # Derived fields (GPS-based local coords for comparison)
    gps_local_x: float = 0.0
    gps_local_y: float = 0.0
    dist_from_prev: float = 0.0
    time_delta_ms: int = 0

def gps_to_local(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Convert GPS to local coordinates (same as firmware)."""
    METERS_PER_DEG = 111320.0
    dlat = lat - ref_lat
    dlon = lon - ref_lon
    north = dlat * METERS_PER_DEG
    east = dlon * METERS_PER_DEG * math.cos(math.radians(ref_lat))
    return (east, north)

def load_csv(filepath: str) -> List[Sample]:
    """Load CSV and parse into samples."""
    samples = []
    has_ekf_columns = False
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        # Check if EKF columns exist
        if reader.fieldnames:
            has_ekf_columns = 'ekf_x' in reader.fieldnames

        for row in reader:
            try:
                # Handle different column naming conventions
                timestamp = int(row.get('time', row.get('timestamp_ms', 0)))
                s = Sample(
                    timestamp_ms=timestamp,
                    gps_lat=float(row['gps_lat']),
                    gps_lon=float(row['gps_lon']),
                    gps_valid=row['gps_valid'] == '1',
                    speed_kmh=float(row['speed']),
                    ax=float(row['ax']),
                    ay=float(row['ay']),
                    az=float(row.get('az', '0')),  # az may not exist
                    wz=float(row['wz']),
                )
                # Parse EKF fields if present
                if has_ekf_columns:
                    s.ekf_x = float(row.get('ekf_x', 0))
                    s.ekf_y = float(row.get('ekf_y', 0))
                    s.ekf_yaw = float(row.get('ekf_yaw', 0))
                    s.has_ekf = True
                samples.append(s)
            except (KeyError, ValueError) as e:
                # Only warn for first few errors
                if len(samples) < 3:
                    print(f"  Warning: skipping row due to {e}")
    return samples, has_ekf_columns

def analyze_file(filepath: str):
    """Comprehensive analysis of a CSV file."""
    print(f"\n{'='*70}")
    print(f"ANALYZING: {filepath}")
    print(f"{'='*70}")

    samples, has_ekf = load_csv(filepath)
    if not samples:
        print("  No data!")
        return

    if has_ekf:
        print("  [EKF data detected - will compare EKF vs GPS coordinates]")

    print(f"\n[1] BASIC STATS")
    print(f"    Total samples: {len(samples)}")
    valid_samples = [s for s in samples if s.gps_valid]
    print(f"    GPS valid samples: {len(valid_samples)} ({100*len(valid_samples)/len(samples):.1f}%)")

    if not valid_samples:
        print("  No valid GPS samples!")
        return

    # Calculate duration
    duration_ms = samples[-1].timestamp_ms - samples[0].timestamp_ms
    duration_s = duration_ms / 1000.0
    print(f"    Duration: {duration_s:.1f}s")
    print(f"    Sample rate: {len(samples)/duration_s:.1f} Hz")

    # Find reference point (first valid GPS)
    ref_lat = valid_samples[0].gps_lat
    ref_lon = valid_samples[0].gps_lon
    print(f"    Reference point: {ref_lat:.6f}, {ref_lon:.6f}")

    # Convert all GPS positions to local coordinates
    for s in samples:
        if s.gps_valid:
            s.gps_local_x, s.gps_local_y = gps_to_local(s.gps_lat, s.gps_lon, ref_lat, ref_lon)

    # Calculate distances and time deltas (using GPS-derived local coords)
    prev = None
    for s in samples:
        if prev and s.gps_valid and prev.gps_valid:
            dx = s.gps_local_x - prev.gps_local_x
            dy = s.gps_local_y - prev.gps_local_y
            s.dist_from_prev = math.sqrt(dx*dx + dy*dy)
            s.time_delta_ms = s.timestamp_ms - prev.timestamp_ms
        prev = s

    print(f"\n[2] POSITION ANALYSIS")

    # Find position jumps
    jump_threshold_05m = 0.5  # Track recorder threshold
    jump_threshold_2m = 2.0   # Suspicious
    jump_threshold_5m = 5.0   # Very suspicious

    jumps_05m = [(i, s) for i, s in enumerate(samples) if s.dist_from_prev >= jump_threshold_05m]
    jumps_2m = [(i, s) for i, s in enumerate(samples) if s.dist_from_prev >= jump_threshold_2m]
    jumps_5m = [(i, s) for i, s in enumerate(samples) if s.dist_from_prev >= jump_threshold_5m]

    print(f"    Position jumps >= 0.5m: {len(jumps_05m)} (track recorder would count these)")
    print(f"    Position jumps >= 2.0m: {len(jumps_2m)} (suspicious)")
    print(f"    Position jumps >= 5.0m: {len(jumps_5m)} (very suspicious)")

    # Calculate total "recorded" distance (jumps >= 0.5m)
    total_recorded_dist = sum(s.dist_from_prev for i, s in jumps_05m)
    print(f"    Total distance (>=0.5m jumps): {total_recorded_dist:.1f}m")

    # Show largest jumps
    if jumps_2m:
        print(f"\n    Largest position jumps:")
        sorted_jumps = sorted(jumps_2m, key=lambda x: x[1].dist_from_prev, reverse=True)[:10]
        for idx, s in sorted_jumps:
            print(f"      Sample {idx}: {s.dist_from_prev:.2f}m jump, speed={s.speed_kmh:.1f}km/h, dt={s.time_delta_ms}ms")

    print(f"\n[3] STATIONARY ANALYSIS (speed < 2 km/h)")

    # Find samples where speed is ~0 but position changes
    stationary_with_movement = []
    for i, s in enumerate(samples):
        if s.gps_valid and s.speed_kmh < 2.0 and s.dist_from_prev >= 0.5:
            stationary_with_movement.append((i, s))

    print(f"    Samples with speed<2km/h but movement>=0.5m: {len(stationary_with_movement)}")
    if stationary_with_movement:
        total_phantom_dist = sum(s.dist_from_prev for i, s in stationary_with_movement)
        print(f"    Total 'phantom' distance while stationary: {total_phantom_dist:.1f}m")
        print(f"    (This is the BUG - distance increasing while not moving)")

        # Show examples
        print(f"\n    Examples of phantom movement:")
        for idx, s in stationary_with_movement[:10]:
            print(f"      Sample {idx}: speed={s.speed_kmh:.1f}km/h, jump={s.dist_from_prev:.2f}m")

    print(f"\n[4] SPEED VS POSITION CORRELATION")

    # Calculate expected distance based on speed
    total_expected_dist = 0
    for s in samples:
        if s.gps_valid and s.time_delta_ms > 0:
            # Distance = speed * time
            speed_ms = s.speed_kmh / 3.6
            dt_s = s.time_delta_ms / 1000.0
            total_expected_dist += speed_ms * dt_s

    print(f"    Expected distance (from GPS speed): {total_expected_dist:.1f}m")
    print(f"    Actual distance (from position changes >= 0.5m): {total_recorded_dist:.1f}m")
    if total_expected_dist > 0:
        ratio = total_recorded_dist / total_expected_dist
        print(f"    Ratio: {ratio:.2f}x")
        if ratio > 1.5:
            print(f"    WARNING: Recording {ratio:.1f}x more distance than expected from speed!")

    print(f"\n[5] TIME GAP ANALYSIS")

    # Find time gaps
    time_gaps = [(i, s) for i, s in enumerate(samples) if s.time_delta_ms > 100]  # > 100ms gap
    if time_gaps:
        print(f"    Samples with >100ms gap: {len(time_gaps)}")
        print(f"    Largest gaps:")
        sorted_gaps = sorted(time_gaps, key=lambda x: x[1].time_delta_ms, reverse=True)[:5]
        for idx, s in sorted_gaps:
            print(f"      Sample {idx}: {s.time_delta_ms}ms gap")
    else:
        print(f"    No significant time gaps (good!)")

    print(f"\n[6] IMU ANALYSIS (when stationary)")

    # Check IMU readings when stationary
    stationary_samples = [s for s in samples if s.gps_valid and s.speed_kmh < 2.0]
    if stationary_samples:
        ax_vals = [s.ax for s in stationary_samples]
        ay_vals = [s.ay for s in stationary_samples]
        az_vals = [s.az for s in stationary_samples]
        wz_vals = [s.wz for s in stationary_samples]

        print(f"    Stationary samples: {len(stationary_samples)}")
        print(f"    Accel X: mean={sum(ax_vals)/len(ax_vals):.3f}, range=[{min(ax_vals):.3f}, {max(ax_vals):.3f}] m/s²")
        print(f"    Accel Y: mean={sum(ay_vals)/len(ay_vals):.3f}, range=[{min(ay_vals):.3f}, {max(ay_vals):.3f}] m/s²")
        print(f"    Accel Z: mean={sum(az_vals)/len(az_vals):.3f}, range=[{min(az_vals):.3f}, {max(az_vals):.3f}] m/s²")
        print(f"    Yaw rate: mean={sum(wz_vals)/len(wz_vals):.4f}, range=[{min(wz_vals):.4f}, {max(wz_vals):.4f}] rad/s")

        # Check if accelerations are suspiciously high when stationary
        high_accel = [s for s in stationary_samples if abs(s.ax) > 1.0 or abs(s.ay) > 1.0]
        if high_accel:
            print(f"    WARNING: {len(high_accel)} stationary samples with |accel| > 1.0 m/s²")

    print(f"\n[7] LOOP CLOSURE CHECK (GPS-derived)")

    # Check start to end distance using GPS-derived local coords
    start = valid_samples[0]
    end = valid_samples[-1]
    start_end_dist = math.sqrt((end.gps_local_x - start.gps_local_x)**2 + (end.gps_local_y - start.gps_local_y)**2)
    print(f"    GPS Start position: ({start.gps_local_x:.2f}, {start.gps_local_y:.2f})")
    print(f"    GPS End position: ({end.gps_local_x:.2f}, {end.gps_local_y:.2f})")
    print(f"    GPS Start-to-end distance: {start_end_dist:.1f}m")
    print(f"    Loop detection threshold: 25m")
    if start_end_dist < 25:
        print(f"    GPS says: SHOULD detect as loop!")
    else:
        print(f"    GPS says: Would NOT detect as loop (need {start_end_dist - 25:.1f}m closer)")

    # If EKF data available, also check EKF-based loop closure
    if has_ekf:
        ekf_start_end_dist = math.sqrt((end.ekf_x - start.ekf_x)**2 + (end.ekf_y - start.ekf_y)**2)
        print(f"\n    EKF Start position: ({start.ekf_x:.2f}, {start.ekf_y:.2f})")
        print(f"    EKF End position: ({end.ekf_x:.2f}, {end.ekf_y:.2f})")
        print(f"    EKF Start-to-end distance: {ekf_start_end_dist:.1f}m")
        if ekf_start_end_dist < 25:
            print(f"    EKF says: SHOULD detect as loop!")
        else:
            print(f"    EKF says: Would NOT detect as loop (need {ekf_start_end_dist - 25:.1f}m closer)")

        # Show difference between GPS and EKF
        print(f"\n    GPS vs EKF difference at end: {abs(start_end_dist - ekf_start_end_dist):.1f}m")

    print(f"\n[8] POSITION BOUNDS (GPS-derived)")

    xs = [s.gps_local_x for s in valid_samples]
    ys = [s.gps_local_y for s in valid_samples]
    print(f"    GPS X range: {min(xs):.1f} to {max(xs):.1f}m (span: {max(xs)-min(xs):.1f}m)")
    print(f"    GPS Y range: {min(ys):.1f} to {max(ys):.1f}m (span: {max(ys)-min(ys):.1f}m)")

    if has_ekf:
        ekf_xs = [s.ekf_x for s in valid_samples]
        ekf_ys = [s.ekf_y for s in valid_samples]
        print(f"    EKF X range: {min(ekf_xs):.1f} to {max(ekf_xs):.1f}m (span: {max(ekf_xs)-min(ekf_xs):.1f}m)")
        print(f"    EKF Y range: {min(ekf_ys):.1f} to {max(ekf_ys):.1f}m (span: {max(ekf_ys)-min(ekf_ys):.1f}m)")

    # EKF vs GPS comparison
    if has_ekf:
        print(f"\n[9] EKF vs GPS COMPARISON (CRITICAL FOR DEBUGGING)")

        # Calculate distance between EKF and GPS positions
        diffs = []
        for s in valid_samples:
            diff = math.sqrt((s.ekf_x - s.gps_local_x)**2 + (s.ekf_y - s.gps_local_y)**2)
            diffs.append((s, diff))

        diff_values = [d[1] for d in diffs]
        print(f"    Mean EKF-GPS offset: {sum(diff_values)/len(diff_values):.2f}m")
        print(f"    Max EKF-GPS offset: {max(diff_values):.2f}m")
        print(f"    Min EKF-GPS offset: {min(diff_values):.2f}m")

        # Count samples where EKF is significantly different from GPS
        large_diffs = [d for d in diffs if d[1] > 5.0]
        if large_diffs:
            print(f"    Samples with >5m EKF-GPS difference: {len(large_diffs)}")
            print(f"    WARNING: EKF and GPS positions are diverging significantly!")
            print(f"    This could cause track recorder to see different positions than expected.")

            # Show worst cases
            print(f"\n    Largest EKF-GPS differences:")
            sorted_diffs = sorted(diffs, key=lambda x: x[1], reverse=True)[:5]
            for s, diff in sorted_diffs:
                print(f"      GPS:({s.gps_local_x:.1f},{s.gps_local_y:.1f}) EKF:({s.ekf_x:.1f},{s.ekf_y:.1f}) diff={diff:.1f}m speed={s.speed_kmh:.1f}km/h")
        else:
            print(f"    EKF-GPS tracking looks good (no large divergences)")

        # Analyze EKF position jumps specifically
        print(f"\n[10] EKF POSITION JUMP ANALYSIS (what track recorder sees)")
        ekf_jumps = []
        prev = None
        for i, s in enumerate(valid_samples):
            if prev:
                ekf_dist = math.sqrt((s.ekf_x - prev.ekf_x)**2 + (s.ekf_y - prev.ekf_y)**2)
                ekf_jumps.append((i, s, ekf_dist, prev))
            prev = s

        ekf_jumps_05m = [(i, s, d, p) for i, s, d, p in ekf_jumps if d >= 0.5]
        ekf_jumps_2m = [(i, s, d, p) for i, s, d, p in ekf_jumps if d >= 2.0]

        print(f"    EKF jumps >= 0.5m: {len(ekf_jumps_05m)} (track recorder counts these)")
        print(f"    EKF jumps >= 2.0m: {len(ekf_jumps_2m)} (suspicious)")

        total_ekf_dist = sum(d for _, _, d, _ in ekf_jumps_05m)
        print(f"    Total EKF distance (>=0.5m jumps): {total_ekf_dist:.1f}m")

        # Stationary EKF movement
        stationary_ekf_movement = [(i, s, d, p) for i, s, d, p in ekf_jumps if d >= 0.5 and s.speed_kmh < 2.0]
        if stationary_ekf_movement:
            total_phantom = sum(d for _, _, d, _ in stationary_ekf_movement)
            print(f"    EKF movement while stationary (speed<2km/h): {len(stationary_ekf_movement)} events, {total_phantom:.1f}m total")
            print(f"    THIS IS THE BUG - EKF position changing when vehicle is stopped")
            print(f"\n    Examples of EKF phantom movement:")
            for i, s, d, p in stationary_ekf_movement[:5]:
                print(f"      Sample {i}: speed={s.speed_kmh:.1f}km/h, EKF jump={d:.2f}m")
                print(f"        from ({p.ekf_x:.2f},{p.ekf_y:.2f}) to ({s.ekf_x:.2f},{s.ekf_y:.2f})")

    # Visualize rough shape
    print(f"\n[11] TRAJECTORY VISUALIZATION (rough ASCII)")
    visualize_trajectory(valid_samples, has_ekf)

    return samples

def visualize_trajectory(samples: List[Sample], has_ekf: bool = False, width: int = 60, height: int = 20):
    """Create rough ASCII visualization of trajectory."""
    if not samples:
        return

    xs = [s.gps_local_x for s in samples]
    ys = [s.gps_local_y for s in samples]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # Avoid division by zero
    range_x = max(max_x - min_x, 1)
    range_y = max(max_y - min_y, 1)

    # Create grid
    grid = [[' ' for _ in range(width)] for _ in range(height)]

    # Plot points
    for i, s in enumerate(samples):
        col = int((s.gps_local_x - min_x) / range_x * (width - 1))
        row = int((max_y - s.gps_local_y) / range_y * (height - 1))  # Flip Y for display
        col = max(0, min(width - 1, col))
        row = max(0, min(height - 1, row))

        if i == 0:
            grid[row][col] = 'S'  # Start
        elif i == len(samples) - 1:
            grid[row][col] = 'E'  # End
        else:
            grid[row][col] = '.'

    # Print grid
    print(f"    GPS trajectory:")
    print(f"    {'─' * width}")
    for row in grid:
        print(f"    │{''.join(row)}│")
    print(f"    {'─' * width}")
    print(f"    S=Start, E=End, .=trajectory")

def main():
    if len(sys.argv) > 1:
        files = sys.argv[1:]
    else:
        # Find all CSV files in project root
        root = Path(__file__).parent.parent.parent
        files = sorted(root.glob("blackbox_*.csv"))

    if not files:
        print("No CSV files found!")
        print("Usage: python debug_track_recorder.py <file1.csv> [file2.csv ...]")
        return

    print("=" * 70)
    print("TRACK RECORDER DEBUG ANALYSIS")
    print("=" * 70)
    print(f"Analyzing {len(files)} file(s)...")

    for f in files:
        analyze_file(str(f))

    print("\n" + "=" * 70)
    print("ANALYSIS COMPLETE")
    print("=" * 70)

if __name__ == "__main__":
    main()
