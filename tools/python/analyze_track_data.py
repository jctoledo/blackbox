#!/usr/bin/env python3
"""
Analyze CSV telemetry data for track recording debugging.
Converts GPS lat/lon to local x/y coordinates and analyzes movement patterns.
"""

import sys
import csv
import math
from pathlib import Path

def gps_to_local(lat, lon, ref_lat, ref_lon):
    """Convert GPS coordinates to local x/y meters (simplified flat-earth)."""
    # Same approximation as firmware uses
    lat_rad = math.radians(ref_lat)
    dx = (lon - ref_lon) * math.cos(lat_rad) * 111320  # meters east
    dy = (lat - ref_lat) * 111320  # meters north
    return dx, dy

def analyze_csv(filepath):
    """Analyze a single CSV file."""
    print(f"\n{'='*60}")
    print(f"Analyzing: {filepath}")
    print(f"{'='*60}")

    rows = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        print("  No data!")
        return

    print(f"  Total samples: {len(rows)}")

    # Get reference point (first valid GPS position)
    ref_lat, ref_lon = None, None
    for row in rows:
        if row['gps_valid'] == '1':
            ref_lat = float(row['gps_lat'])
            ref_lon = float(row['gps_lon'])
            break

    if ref_lat is None:
        print("  No valid GPS data!")
        return

    print(f"  Reference point: {ref_lat:.6f}, {ref_lon:.6f}")

    # Convert all positions to local coordinates
    positions = []
    speeds = []
    for row in rows:
        if row['gps_valid'] == '1':
            lat = float(row['gps_lat'])
            lon = float(row['gps_lon'])
            x, y = gps_to_local(lat, lon, ref_lat, ref_lon)
            speed = float(row['speed'])
            positions.append((x, y, speed))
            speeds.append(speed)

    print(f"  Valid GPS samples: {len(positions)}")

    if len(positions) < 2:
        print("  Not enough data!")
        return

    # Calculate movement statistics
    total_distance = 0
    max_jump = 0
    jump_count_05m = 0  # Jumps > 0.5m (track recorder threshold)
    jump_count_2m = 0   # Jumps > 2m (suspicious)

    stationary_samples = 0
    moving_samples = 0

    for i in range(1, len(positions)):
        x1, y1, s1 = positions[i-1]
        x2, y2, s2 = positions[i]
        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        total_distance += dist
        max_jump = max(max_jump, dist)

        if dist > 0.5:
            jump_count_05m += 1
        if dist > 2.0:
            jump_count_2m += 1

        # Check if stationary (speed < 2 km/h)
        avg_speed = (s1 + s2) / 2
        if avg_speed < 2:
            if dist > 0.5:
                stationary_samples += 1
        else:
            moving_samples += 1

    # Position bounds
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # Start/end distance (for loop detection)
    start_x, start_y, _ = positions[0]
    end_x, end_y, _ = positions[-1]
    start_end_dist = math.sqrt((end_x-start_x)**2 + (end_y-start_y)**2)

    print(f"\n  Movement Analysis:")
    print(f"    Total distance: {total_distance:.1f} m")
    print(f"    Max single jump: {max_jump:.2f} m")
    print(f"    Jumps > 0.5m (recorder threshold): {jump_count_05m}")
    print(f"    Jumps > 2m (suspicious): {jump_count_2m}")
    print(f"    Position drift while stationary: {stationary_samples} samples")

    print(f"\n  Position Bounds:")
    print(f"    X range: {min_x:.1f} to {max_x:.1f} m (span: {max_x-min_x:.1f} m)")
    print(f"    Y range: {min_y:.1f} to {max_y:.1f} m (span: {max_y-min_y:.1f} m)")

    print(f"\n  Loop Analysis:")
    print(f"    Start to end distance: {start_end_dist:.1f} m")
    print(f"    (Loop closure threshold: 25m)")

    print(f"\n  Speed Statistics:")
    print(f"    Min speed: {min(speeds):.1f} km/h")
    print(f"    Max speed: {max(speeds):.1f} km/h")
    print(f"    Avg speed: {sum(speeds)/len(speeds):.1f} km/h")

    # Show first/last few positions
    print(f"\n  First 5 positions (x, y, speed):")
    for i in range(min(5, len(positions))):
        x, y, s = positions[i]
        print(f"    {i}: ({x:7.2f}, {y:7.2f}) @ {s:.1f} km/h")

    print(f"\n  Last 5 positions (x, y, speed):")
    for i in range(max(0, len(positions)-5), len(positions)):
        x, y, s = positions[i]
        print(f"    {i}: ({x:7.2f}, {y:7.2f}) @ {s:.1f} km/h")

    return positions

def main():
    if len(sys.argv) > 1:
        files = sys.argv[1:]
    else:
        # Default: find all CSV files in project root
        root = Path(__file__).parent.parent.parent
        files = sorted(root.glob("blackbox_*.csv"))

    if not files:
        print("No CSV files found!")
        return

    print("Track Recording Data Analysis")
    print("=" * 60)

    for f in files:
        analyze_csv(f)

if __name__ == "__main__":
    main()
