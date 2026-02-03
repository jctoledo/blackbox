#!/usr/bin/env python3
"""
Simulate the JavaScript track recorder logic to test fixes with CSV data.
This mirrors the exact behavior of the dashboard's TrackRecorder class.
"""

import sys
import csv
import math
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

# Constants matching JavaScript TrackRecorder
MIN_SAMPLE_DISTANCE = 0.5  # meters - minimum distance between samples
MIN_LOOP_DISTANCE = 150.0  # meters - minimum distance before loop can close
CLOSE_PROXIMITY = 25.0     # meters - how close to start to detect loop
HEADING_TOLERANCE = 0.5    # radians (~28 degrees) - heading must match for loop

# Speed gate thresholds (hysteresis)
SPEED_ENTRY_THRESHOLD = 2.0   # km/h - start accepting samples when above this
SPEED_EXIT_THRESHOLD = 0.5    # km/h - stop accepting samples when below this

@dataclass
class Position:
    x: float
    y: float
    speed: float  # km/h
    heading: float  # radians (GPS course preferred, EKF yaw fallback)
    lat: float = 0.0
    lon: float = 0.0
    timestamp: int = 0
    gps_course: float = float('nan')  # GPS course over ground (radians)

@dataclass
class TrackRecorderConfig:
    min_sample_distance: float = MIN_SAMPLE_DISTANCE
    min_loop_distance: float = MIN_LOOP_DISTANCE
    close_proximity: float = CLOSE_PROXIMITY
    heading_tolerance: float = HEADING_TOLERANCE

class TrackRecorder:
    """Python port of JavaScript TrackRecorder class for simulation."""

    def __init__(self, track_type: str = 'loop', config: Optional[TrackRecorderConfig] = None):
        self.track_type = track_type
        self.config = config or TrackRecorderConfig()
        self.reset()

    def reset(self):
        self.recording = False
        self.samples: List[Position] = []
        self.total_distance = 0.0
        self.start_pos: Optional[Position] = None
        self.start_heading = 0.0
        self.last_pos: Optional[Position] = None
        self.loop_detected = False
        self.speed_gate = False  # False = waiting for speed > entry threshold

        # Debug counters
        self.rejected_speed_low = 0
        self.rejected_speed_gate = 0
        self.rejected_distance = 0
        self.samples_stored = 0

    def start(self, pos: Position, capture_heading_when_moving: bool = False):
        self.reset()
        self.recording = True
        self.start_pos = pos
        self.start_heading = pos.heading  # Initial heading (may be updated when moving)
        self.capture_heading_when_moving = capture_heading_when_moving
        self.heading_captured_while_moving = False
        self.last_pos = pos
        self.samples.append(pos)
        print(f"  Recording started at ({pos.x:.1f}, {pos.y:.1f}) heading={math.degrees(pos.heading):.1f}°")
        if capture_heading_when_moving:
            print(f"  (Will capture true heading when car starts moving)")

    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def add_sample_old_logic(self, pos: Position) -> dict:
        """OLD BUGGY LOGIC: Speed gate checked before loop closure."""
        if not self.recording or not self.last_pos:
            return {'stored': False, 'reason': 'not_recording'}

        # OLD BUG: Speed check first - blocks loop closure when stopped at start!
        spd = pos.speed or 0
        if not self.speed_gate:
            if spd < SPEED_ENTRY_THRESHOLD:
                self.rejected_speed_gate += 1
                return {'stored': False, 'reason': 'speed_gate_entry'}
            self.speed_gate = True
        else:
            if spd < SPEED_EXIT_THRESHOLD:
                self.speed_gate = False
                self.rejected_speed_low += 1
                return {'stored': False, 'reason': 'speed_gate_exit'}

        # Loop closure check (too late - already filtered by speed!)
        if self.track_type == 'loop' and self.total_distance >= self.config.min_loop_distance:
            dist_to_start = math.sqrt(
                (pos.x - self.start_pos.x) ** 2 +
                (pos.y - self.start_pos.y) ** 2
            )
            if dist_to_start < self.config.close_proximity:
                heading_diff = abs(self._wrap_angle(pos.heading - self.start_heading))
                if heading_diff < self.config.heading_tolerance:
                    self.loop_detected = True
                    return {'stored': True, 'loop_detected': True}

        # Distance check
        dist = math.sqrt(
            (pos.x - self.last_pos.x) ** 2 +
            (pos.y - self.last_pos.y) ** 2
        )
        if dist < self.config.min_sample_distance:
            self.rejected_distance += 1
            return {'stored': False, 'reason': 'distance_too_small'}

        self.total_distance += dist
        self.last_pos = pos
        self.samples.append(pos)
        self.samples_stored += 1
        return {'stored': True}

    def add_sample_new_logic(self, pos: Position) -> dict:
        """NEW FIXED LOGIC: Loop closure checked before speed gate."""
        if not self.recording or not self.last_pos:
            return {'stored': False, 'reason': 'not_recording'}

        # FIX: Check loop closure FIRST - even when stopped at start position
        if self.track_type == 'loop' and self.total_distance >= self.config.min_loop_distance:
            dist_to_start = math.sqrt(
                (pos.x - self.start_pos.x) ** 2 +
                (pos.y - self.start_pos.y) ** 2
            )
            if dist_to_start < self.config.close_proximity:
                heading_diff = abs(self._wrap_angle(pos.heading - self.start_heading))
                if heading_diff < self.config.heading_tolerance:
                    self.loop_detected = True
                    return {'stored': True, 'loop_detected': True}

        # Speed gate - skip samples while stationary (but loop check already ran above)
        spd = pos.speed or 0
        if not self.speed_gate:
            if spd < SPEED_ENTRY_THRESHOLD:
                self.rejected_speed_gate += 1
                return {'stored': False, 'reason': 'speed_gate_entry'}
            self.speed_gate = True
            # FIX: Reset last_pos when speed gate opens to prevent distance jump
            self.last_pos = Position(x=pos.x, y=pos.y, heading=pos.heading, speed=pos.speed)
            # FIX: Capture true heading when car first starts moving (not when stopped)
            if self.capture_heading_when_moving and not self.heading_captured_while_moving:
                old_heading = self.start_heading
                self.start_heading = pos.heading
                self.heading_captured_while_moving = True
                print(f"  Updated startHeading: {math.degrees(old_heading):.1f}° → {math.degrees(pos.heading):.1f}° (captured when moving)")
        else:
            if spd < SPEED_EXIT_THRESHOLD:
                self.speed_gate = False
                self.rejected_speed_low += 1
                return {'stored': False, 'reason': 'speed_gate_exit'}

        # Distance check
        dist = math.sqrt(
            (pos.x - self.last_pos.x) ** 2 +
            (pos.y - self.last_pos.y) ** 2
        )
        if dist < self.config.min_sample_distance:
            self.rejected_distance += 1
            return {'stored': False, 'reason': 'distance_too_small'}

        self.total_distance += dist
        self.last_pos = pos
        self.samples.append(pos)
        self.samples_stored += 1
        return {'stored': True}


def gps_to_local(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Convert GPS to local x/y meters (same as firmware/JS)."""
    lat_rad = math.radians(ref_lat)
    dx = (lon - ref_lon) * math.cos(lat_rad) * 111320
    dy = (lat - ref_lat) * 111320
    return dx, dy


def compute_heading(x1: float, y1: float, x2: float, y2: float) -> float:
    """Compute heading from position change."""
    return math.atan2(y2 - y1, x2 - x1)


def load_csv(filepath: str) -> Tuple[List[dict], bool, bool]:
    """Load CSV data. Returns (rows, has_ekf_columns, has_gps_course)."""
    rows = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    has_ekf = len(rows) > 0 and 'ekf_x' in rows[0]
    has_gps_course = len(rows) > 0 and 'gps_course' in rows[0]
    return rows, has_ekf, has_gps_course


def integrate_yaw_from_rate(rows: List[dict], start_yaw: float = 0.0) -> List[float]:
    """Integrate yaw rate to estimate yaw angle over time."""
    yaws = [start_yaw]
    prev_t = int(rows[0]['time'])
    for row in rows[1:]:
        t = int(row['time'])
        dt = (t - prev_t) / 1000.0  # ms to seconds
        wz = float(row['wz'])  # rad/s
        new_yaw = yaws[-1] + wz * dt
        # Wrap to [-pi, pi]
        while new_yaw > math.pi:
            new_yaw -= 2 * math.pi
        while new_yaw < -math.pi:
            new_yaw += 2 * math.pi
        yaws.append(new_yaw)
        prev_t = t
    return yaws


def simulate_track_recording(filepath: str):
    """Simulate track recording with CSV data."""
    print(f"\n{'='*70}")
    print(f"TRACK RECORDER SIMULATION")
    print(f"File: {filepath}")
    print(f"{'='*70}")

    rows, has_ekf, has_gps_course = load_csv(filepath)
    if not rows:
        print("No data!")
        return

    print(f"\nTotal samples in CSV: {len(rows)}")
    print(f"Has EKF columns (ekf_x, ekf_y, ekf_yaw): {has_ekf}")
    print(f"Has GPS course column: {has_gps_course}")

    # Find first 5 valid GPS fixes to establish reference (mirrors JS/firmware)
    ref_samples = []
    for row in rows:
        if row['gps_valid'] == '1' and float(row['gps_lat']) != 0:
            ref_samples.append({
                'lat': float(row['gps_lat']),
                'lon': float(row['gps_lon'])
            })
            if len(ref_samples) >= 5:
                break

    if len(ref_samples) < 5:
        print(f"Only {len(ref_samples)} valid GPS samples - cannot establish reference!")
        return

    ref_lat = sum(s['lat'] for s in ref_samples) / 5
    ref_lon = sum(s['lon'] for s in ref_samples) / 5
    print(f"GPS Reference: {ref_lat:.6f}, {ref_lon:.6f}")

    # Option 1: Use EKF columns if available (more accurate)
    # Option 2: Integrate yaw from yaw rate (better than position-derived)
    # Option 3: Compute heading from position change (fallback)

    # Try to integrate yaw from yaw rate for more accurate heading
    integrated_yaws = integrate_yaw_from_rate(rows)

    # Convert all valid samples to positions with local coordinates
    positions = []
    prev_pos = None
    gps_course_used_count = 0
    for i, row in enumerate(rows):
        if row['gps_valid'] != '1':
            continue

        lat = float(row['gps_lat'])
        lon = float(row['gps_lon'])
        if lat == 0 and lon == 0:
            continue

        speed = float(row['speed'])
        timestamp = int(row['time'])

        # Get position: EKF columns preferred, else compute from GPS
        if has_ekf:
            x = float(row['ekf_x'])
            y = float(row['ekf_y'])
            ekf_yaw = float(row['ekf_yaw'])
        else:
            x, y = gps_to_local(lat, lon, ref_lat, ref_lon)
            ekf_yaw = integrated_yaws[i]

        # GPS course (radians): preferred when valid (speed > 2 km/h, not empty)
        gps_course = float('nan')
        if has_gps_course:
            gc_str = row.get('gps_course', '').strip()
            if gc_str:
                gps_course = float(gc_str)

        # Use GPS course for heading when valid (not NaN and speed > 2 km/h)
        if not math.isnan(gps_course) and speed > 2.0:
            heading = gps_course
            gps_course_used_count += 1
        else:
            heading = ekf_yaw

        pos = Position(x=x, y=y, speed=speed, heading=heading, lat=lat, lon=lon,
                       timestamp=timestamp, gps_course=gps_course)
        positions.append(pos)

    if has_gps_course:
        print(f"Using GPS course for heading when valid ({gps_course_used_count}/{len(positions)} samples)")
        print("  (GPS course is direct measurement, more reliable than EKF yaw for loop closure)")
    elif has_ekf:
        print("Using EKF position (ekf_x, ekf_y) and heading (ekf_yaw)")
    else:
        print("Using GPS-derived position and integrated yaw rate for heading")
        print("  (This is an approximation - EKF yaw would be more accurate)")

    print(f"Valid positions: {len(positions)}")

    if len(positions) < 10:
        print("Not enough data!")
        return

    # Analyze the data
    print(f"\n--- DATA ANALYSIS ---")

    # Time analysis
    dt_samples = []
    for i in range(1, len(positions)):
        dt = (positions[i].timestamp - positions[i-1].timestamp) / 1000.0
        if dt > 0:
            dt_samples.append(dt)

    if dt_samples:
        avg_dt = sum(dt_samples) / len(dt_samples)
        avg_rate = 1.0 / avg_dt if avg_dt > 0 else 0
        print(f"Average sample rate: {avg_rate:.1f} Hz (dt={avg_dt*1000:.1f}ms)")

    # Speed analysis
    speeds = [p.speed for p in positions]
    print(f"Speed range: {min(speeds):.1f} - {max(speeds):.1f} km/h")
    stationary_samples = sum(1 for s in speeds if s < SPEED_EXIT_THRESHOLD)
    moving_samples = sum(1 for s in speeds if s >= SPEED_ENTRY_THRESHOLD)
    print(f"Stationary samples (<{SPEED_EXIT_THRESHOLD} km/h): {stationary_samples}")
    print(f"Moving samples (>={SPEED_ENTRY_THRESHOLD} km/h): {moving_samples}")

    # Position analysis
    xs = [p.x for p in positions]
    ys = [p.y for p in positions]
    print(f"X range: {min(xs):.1f} to {max(xs):.1f} m")
    print(f"Y range: {min(ys):.1f} to {max(ys):.1f} m")

    # Start/end proximity (key for loop detection)
    start_pos = positions[0]
    end_pos = positions[-1]
    start_end_dist = math.sqrt((end_pos.x - start_pos.x) ** 2 + (end_pos.y - start_pos.y) ** 2)
    print(f"\nStart position: ({start_pos.x:.1f}, {start_pos.y:.1f})")
    print(f"End position: ({end_pos.x:.1f}, {end_pos.y:.1f})")
    print(f"Start-to-end distance: {start_end_dist:.1f} m (threshold: {CLOSE_PROXIMITY} m)")

    # Check if loop should close
    total_dist_raw = 0
    for i in range(1, len(positions)):
        d = math.sqrt((positions[i].x - positions[i-1].x) ** 2 +
                      (positions[i].y - positions[i-1].y) ** 2)
        total_dist_raw += d
    print(f"Total raw distance: {total_dist_raw:.1f} m (threshold: {MIN_LOOP_DISTANCE} m)")

    # Find when we return near start
    print(f"\n--- PROXIMITY TO START OVER TIME ---")
    for i, pos in enumerate(positions):
        dist_to_start = math.sqrt((pos.x - start_pos.x) ** 2 + (pos.y - start_pos.y) ** 2)
        if dist_to_start < 50:  # Show when close to start
            print(f"  Sample {i}: dist_to_start={dist_to_start:.1f}m speed={pos.speed:.1f}km/h")

    # === SIMULATE OLD LOGIC ===
    print(f"\n{'='*70}")
    print("SIMULATION: OLD BUGGY LOGIC (speed gate before loop check)")
    print(f"{'='*70}")

    recorder_old = TrackRecorder(track_type='loop')
    recorder_old.start(positions[0])

    for i, pos in enumerate(positions[1:], 1):
        result = recorder_old.add_sample_old_logic(pos)
        if result.get('loop_detected'):
            print(f"  ✓ LOOP DETECTED at sample {i}! Distance: {recorder_old.total_distance:.1f}m")
            break

    print(f"\nOLD LOGIC Results:")
    print(f"  Loop detected: {recorder_old.loop_detected}")
    print(f"  Total distance recorded: {recorder_old.total_distance:.1f} m")
    print(f"  Samples stored: {recorder_old.samples_stored}")
    print(f"  Rejected (speed gate entry): {recorder_old.rejected_speed_gate}")
    print(f"  Rejected (speed gate exit): {recorder_old.rejected_speed_low}")
    print(f"  Rejected (distance < {MIN_SAMPLE_DISTANCE}m): {recorder_old.rejected_distance}")

    # === SIMULATE NEW LOGIC (heading from start) ===
    print(f"\n{'='*70}")
    print("SIMULATION: NEW LOGIC (heading captured at start, speed=0)")
    print(f"{'='*70}")

    recorder_new = TrackRecorder(track_type='loop')
    recorder_new.start(positions[0], capture_heading_when_moving=False)

    for i, pos in enumerate(positions[1:], 1):
        result = recorder_new.add_sample_new_logic(pos)
        if result.get('loop_detected'):
            print(f"  ✓ LOOP DETECTED at sample {i}! Distance: {recorder_new.total_distance:.1f}m")
            break

    print(f"\nNEW LOGIC (heading@start) Results:")
    print(f"  Loop detected: {recorder_new.loop_detected}")

    # === SIMULATE NEW LOGIC WITH HEADING FIX ===
    print(f"\n{'='*70}")
    print("SIMULATION: NEW LOGIC + HEADING FIX (heading captured when moving)")
    print(f"{'='*70}")

    recorder_fixed = TrackRecorder(track_type='loop')
    recorder_fixed.start(positions[0], capture_heading_when_moving=True)

    for i, pos in enumerate(positions[1:], 1):
        result = recorder_fixed.add_sample_new_logic(pos)
        if result.get('loop_detected'):
            print(f"  ✓ LOOP DETECTED at sample {i}! Distance: {recorder_fixed.total_distance:.1f}m")
            break

    print(f"\nNEW LOGIC + HEADING FIX Results:")
    print(f"  Loop detected: {recorder_fixed.loop_detected}")
    print(f"  Total distance recorded: {recorder_new.total_distance:.1f} m")
    print(f"  Samples stored: {recorder_new.samples_stored}")
    print(f"  Rejected (speed gate entry): {recorder_new.rejected_speed_gate}")
    print(f"  Rejected (speed gate exit): {recorder_new.rejected_speed_low}")
    print(f"  Rejected (distance < {MIN_SAMPLE_DISTANCE}m): {recorder_new.rejected_distance}")

    # === 14Hz IMPACT ANALYSIS ===
    print(f"\n{'='*70}")
    print("14Hz IMPACT ANALYSIS")
    print(f"{'='*70}")

    # At 14Hz, samples are ~71ms apart
    # Calculate max position jump at various speeds
    print("\nPosition jump per sample at 14Hz:")
    for speed_kmh in [10, 20, 30, 50, 100]:
        speed_ms = speed_kmh / 3.6
        jump_m = speed_ms * (1.0 / 14.0)
        print(f"  {speed_kmh:3d} km/h: {jump_m:.2f} m/sample")

    # Check actual sample-to-sample jumps in data
    print("\nActual position jumps in CSV data:")
    jumps = []
    for i in range(1, len(positions)):
        dist = math.sqrt((positions[i].x - positions[i-1].x) ** 2 +
                        (positions[i].y - positions[i-1].y) ** 2)
        jumps.append((i, dist, positions[i].speed))

    # Sort by jump size
    jumps.sort(key=lambda x: x[1], reverse=True)
    print("  Top 10 largest jumps:")
    for idx, (i, dist, speed) in enumerate(jumps[:10]):
        print(f"    {idx + 1}. Sample {i}: {dist:.2f}m at {speed:.1f}km/h")

    # Impact on loop detection
    print("\n14Hz Impact on Loop Detection:")
    print(f"  Close proximity threshold: {CLOSE_PROXIMITY}m")
    print(f"  At 30 km/h (8.3 m/s): {8.3 / 14:.2f}m between samples")
    print(f"  At 50 km/h (13.9 m/s): {13.9 / 14:.2f}m between samples")
    print(f"  -> Even at highway speeds, samples are <1m apart")
    print(f"  -> With 25m proximity threshold, loop should always detect")

    # Check heading variation at end
    print("\nHeading Analysis at End of Recording:")
    for i in range(-10, 0):
        pos = positions[i]
        dist_to_start = math.sqrt((pos.x - start_pos.x) ** 2 + (pos.y - start_pos.y) ** 2)
        heading_diff = abs(recorder_new._wrap_angle(pos.heading - recorder_new.start_heading))
        within_prox = "YES" if dist_to_start < CLOSE_PROXIMITY else "no"
        within_head = "YES" if heading_diff < HEADING_TOLERANCE else "no"
        print(f"  Sample {i}: dist={dist_to_start:.1f}m ({within_prox}) "
              f"heading_diff={math.degrees(heading_diff):.1f}° ({within_head}) "
              f"speed={pos.speed:.1f}km/h")


def main():
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        # Find most recent CSV
        root = Path(__file__).parent.parent.parent
        csvs = list(root.glob("*blackbox*.csv"))
        if not csvs:
            print("No CSV files found!")
            return
        filepath = max(csvs, key=lambda p: p.stat().st_mtime)
        print(f"Using most recent CSV: {filepath}")

    simulate_track_recording(filepath)


if __name__ == "__main__":
    main()
