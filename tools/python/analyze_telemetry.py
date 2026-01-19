#!/usr/bin/env python3
"""
Analyze telemetry CSV files from blackbox device.

Usage: python analyze_telemetry.py <csvfile>

CSV format expected (new format with fusion diagnostics):
time,speed,ax,ay,wz,mode,lat_g,lon_g,gps_lat,gps_lon,gps_valid,lon_imu,lon_gps,gps_weight,pitch_corr,pitch_conf,roll_corr,roll_conf,tilt_x,tilt_y

Also supports legacy format:
time,speed,ax,ay,wz,mode,lat_g,lon_g,gps_lat,gps_lon,gps_valid
"""

import csv
import sys
from collections import Counter
import statistics

def analyze_telemetry(filename):
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print("No data in file")
        return

    # Detect if we have fusion columns (new format)
    has_fusion = 'lon_imu' in rows[0] and 'lon_gps' in rows[0]

    print(f"{'='*60}")
    print(f"TELEMETRY ANALYSIS: {filename}")
    print(f"{'='*60}")
    print(f"Total samples: {len(rows)}")
    print(f"Format: {'Extended (with fusion diagnostics)' if has_fusion else 'Legacy'}")
    print()

    # === TIMING ANALYSIS ===
    print("--- TIMING ---")
    times = [int(r['time']) for r in rows]
    intervals = [times[i] - times[i-1] for i in range(1, len(times))]
    if intervals:
        avg_interval = statistics.mean(intervals)
        std_interval = statistics.stdev(intervals) if len(intervals) > 1 else 0
        print(f"Sample rate: {1000/avg_interval:.1f} Hz (interval: {avg_interval:.0f}ms +/- {std_interval:.0f}ms)")
        print(f"Duration: {(times[-1] - times[0])/1000:.1f} seconds")
    print()

    # === SPEED ANALYSIS ===
    print("--- SPEED ---")
    speeds = [float(r['speed']) for r in rows]
    print(f"Range: {min(speeds):.1f} to {max(speeds):.1f} km/h")
    print(f"Mean: {statistics.mean(speeds):.1f} km/h")

    # Time spent in speed bands
    stopped = sum(1 for s in speeds if s < 2)
    slow = sum(1 for s in speeds if 2 <= s < 20)
    medium = sum(1 for s in speeds if 20 <= s < 50)
    fast = sum(1 for s in speeds if s >= 50)
    print(f"Time distribution: stopped(<2)={100*stopped/len(rows):.0f}%, "
          f"slow(2-20)={100*slow/len(rows):.0f}%, "
          f"medium(20-50)={100*medium/len(rows):.0f}%, "
          f"fast(>50)={100*fast/len(rows):.0f}%")
    print()

    # === ACCELERATION ANALYSIS ===
    print("--- LONGITUDINAL ACCELERATION (lon_g in G) ---")
    lon_g_vals = [float(r['lon_g']) for r in rows]
    print(f"Range: {min(lon_g_vals):.3f}g to {max(lon_g_vals):.3f}g")
    print(f"Mean: {statistics.mean(lon_g_vals):.4f}g (should be ~0 for balanced driving)")
    print(f"Std dev: {statistics.stdev(lon_g_vals):.4f}g")

    # Distribution around zero
    near_zero = sum(1 for v in lon_g_vals if abs(v) < 0.05)
    positive = sum(1 for v in lon_g_vals if v >= 0.05)
    negative = sum(1 for v in lon_g_vals if v <= -0.05)
    print(f"Distribution: negative={100*negative/len(rows):.0f}%, "
          f"near-zero={100*near_zero/len(rows):.0f}%, "
          f"positive={100*positive/len(rows):.0f}%")

    # Check for bias - compare when speed is stable
    stable_lon = []
    for i in range(1, len(rows)):
        speed_diff = abs(float(rows[i]['speed']) - float(rows[i-1]['speed']))
        if speed_diff < 0.3:  # Speed stable within 0.3 km/h
            stable_lon.append(float(rows[i]['lon_g']))
    if stable_lon:
        print(f"lon_g when speed stable: mean={statistics.mean(stable_lon):.4f}g "
              f"(bias indicator - should be ~0)")
    print()

    # === FUSION DIAGNOSTICS (if available) ===
    if has_fusion:
        print("--- FUSION DIAGNOSTICS ---")

        # IMU vs GPS acceleration comparison
        lon_imu_vals = [float(r['lon_imu']) for r in rows]
        lon_gps_vals = [float(r['lon_gps']) for r in rows]
        gps_weights = [float(r['gps_weight']) for r in rows]

        # Filter out samples where GPS accel is 0 (stale/unavailable)
        valid_gps = [(imu, gps, w) for imu, gps, w in zip(lon_imu_vals, lon_gps_vals, gps_weights) if abs(gps) > 0.001]

        if valid_gps:
            imu_valid = [v[0] for v in valid_gps]
            gps_valid = [v[1] for v in valid_gps]

            print(f"lon_imu range: {min(lon_imu_vals):.3f} to {max(lon_imu_vals):.3f} m/s²")
            print(f"lon_gps range: {min(gps_valid):.3f} to {max(gps_valid):.3f} m/s²")

            # IMU vs GPS error
            errors = [i - g for i, g in zip(imu_valid, gps_valid)]
            mean_error = statistics.mean(errors)
            print(f"IMU-GPS error: mean={mean_error:.3f} m/s² ({mean_error/9.81:.3f}g), "
                  f"std={statistics.stdev(errors):.3f} m/s²")

            # Correlation between IMU and GPS
            if len(imu_valid) >= 10:
                mean_imu = statistics.mean(imu_valid)
                mean_gps = statistics.mean(gps_valid)
                numerator = sum((i - mean_imu) * (g - mean_gps) for i, g in zip(imu_valid, gps_valid))
                denom_imu = sum((i - mean_imu)**2 for i in imu_valid) ** 0.5
                denom_gps = sum((g - mean_gps)**2 for g in gps_valid) ** 0.5
                if denom_imu > 0 and denom_gps > 0:
                    corr = numerator / (denom_imu * denom_gps)
                    print(f"IMU-GPS correlation: {corr:.3f} (want >0.7)")

        # GPS weight distribution
        avg_weight = statistics.mean(gps_weights)
        high_gps = sum(1 for w in gps_weights if w > 0.7)
        mid_gps = sum(1 for w in gps_weights if 0.3 <= w <= 0.7)
        low_gps = sum(1 for w in gps_weights if w < 0.3)
        print(f"GPS weight: mean={avg_weight:.2f}, high(>0.7)={100*high_gps/len(rows):.0f}%, "
              f"mid={100*mid_gps/len(rows):.0f}%, low(<0.3)={100*low_gps/len(rows):.0f}%")

        # OrientationCorrector status
        pitch_corrs = [float(r['pitch_corr']) for r in rows]
        pitch_confs = [float(r['pitch_conf']) for r in rows]
        roll_corrs = [float(r['roll_corr']) for r in rows]
        roll_confs = [float(r['roll_conf']) for r in rows]

        print(f"Pitch correction: {statistics.mean(pitch_corrs):.1f}° (range {min(pitch_corrs):.1f}° to {max(pitch_corrs):.1f}°)")
        print(f"Pitch confidence: {statistics.mean(pitch_confs):.0f}% (range {min(pitch_confs):.0f}% to {max(pitch_confs):.0f}%)")
        print(f"Roll correction: {statistics.mean(roll_corrs):.1f}° (range {min(roll_corrs):.1f}° to {max(roll_corrs):.1f}°)")
        print(f"Roll confidence: {statistics.mean(roll_confs):.0f}% (range {min(roll_confs):.0f}% to {max(roll_confs):.0f}%)")

        # Tilt offsets
        tilt_x = [float(r['tilt_x']) for r in rows]
        tilt_y = [float(r['tilt_y']) for r in rows]
        print(f"Tilt offset: X={statistics.mean(tilt_x):.3f}, Y={statistics.mean(tilt_y):.3f} m/s²")
        print()

        # Confidence assessment
        avg_pitch_conf = statistics.mean(pitch_confs)
        if avg_pitch_conf < 30:
            print("⚠️  LOW PITCH CONFIDENCE: OrientationCorrector hasn't learned enough")
            print("    Need more acceleration/braking events for learning")
        elif avg_pitch_conf < 70:
            print("⚠️  MODERATE PITCH CONFIDENCE: OrientationCorrector still learning")
        else:
            print("✓ HIGH PITCH CONFIDENCE: OrientationCorrector well-calibrated")
        print()

    # === RAW ACCELEROMETER ===
    print("--- RAW ACCELEROMETER (ax in m/s²) ---")
    ax_vals = [float(r['ax']) for r in rows]
    print(f"Range: {min(ax_vals):.2f} to {max(ax_vals):.2f} m/s²")
    print(f"Mean: {statistics.mean(ax_vals):.3f} m/s²")
    print()

    # === MODE DISTRIBUTION ===
    print("--- MODE DISTRIBUTION ---")
    modes = Counter(int(float(r['mode'])) for r in rows)
    mode_names = {0: 'IDLE', 1: 'ACCEL', 2: 'BRAKE', 4: 'CORNER',
                  5: 'ACCEL+CORNER', 6: 'BRAKE+CORNER'}
    for mode, count in sorted(modes.items()):
        name = mode_names.get(mode, f'UNKNOWN({mode})')
        print(f"  {name:15} {count:5} ({100*count/len(rows):5.1f}%)")

    total_accel = modes.get(1, 0) + modes.get(5, 0)
    total_brake = modes.get(2, 0) + modes.get(6, 0)
    print(f"\n  Total ACCEL modes: {total_accel} ({100*total_accel/len(rows):.1f}%)")
    print(f"  Total BRAKE modes: {total_brake} ({100*total_brake/len(rows):.1f}%)")
    print(f"  Ratio ACCEL/BRAKE: {total_accel/max(total_brake,1):.1f}x (should be ~1x for balanced driving)")
    print()

    # === GROUND TRUTH FROM SPEED CHANGES ===
    print("--- GROUND TRUTH (from speed changes) ---")

    # Calculate acceleration from speed changes
    true_accels = []
    for i in range(1, len(rows)):
        dt = (int(rows[i]['time']) - int(rows[i-1]['time'])) / 1000.0  # seconds
        if dt > 0:
            dv = (float(rows[i]['speed']) - float(rows[i-1]['speed'])) / 3.6  # m/s
            accel_g = (dv / dt) / 9.80665  # in G
            true_accels.append(accel_g)

    if true_accels:
        print(f"GPS-derived accel range: {min(true_accels):.3f}g to {max(true_accels):.3f}g")

        # Count actual accel/brake events using speed-derived acceleration
        accel_events = sum(1 for a in true_accels if a > 0.05)  # > 0.05g
        brake_events = sum(1 for a in true_accels if a < -0.05)  # < -0.05g
        coast_events = len(true_accels) - accel_events - brake_events
        print(f"True events: accel={accel_events}, brake={brake_events}, coast={coast_events}")
    print()

    # === MODE ACCURACY ANALYSIS ===
    print("--- MODE DETECTION ACCURACY ---")

    # Use lon_g thresholds (City defaults: 0.10g accel, 0.18g brake)
    ACCEL_THRESH = 0.10
    BRAKE_THRESH = 0.18

    tp_accel = fp_accel = fn_accel = 0
    tp_brake = fp_brake = fn_brake = 0

    for i in range(1, len(rows)):
        mode = int(float(rows[i]['mode']))
        lon_g = float(rows[i]['lon_g'])

        # Use speed-derived acceleration as ground truth
        dt = (int(rows[i]['time']) - int(rows[i-1]['time'])) / 1000.0
        if dt > 0:
            dv = (float(rows[i]['speed']) - float(rows[i-1]['speed'])) / 3.6
            true_accel_g = (dv / dt) / 9.80665
        else:
            true_accel_g = 0

        is_accel_mode = mode in [1, 5]
        is_brake_mode = mode in [2, 6]
        truly_accelerating = true_accel_g > 0.05
        truly_braking = true_accel_g < -0.05

        # ACCEL accuracy
        if is_accel_mode and truly_accelerating:
            tp_accel += 1
        elif is_accel_mode and not truly_accelerating:
            fp_accel += 1
        elif not is_accel_mode and truly_accelerating:
            fn_accel += 1

        # BRAKE accuracy
        if is_brake_mode and truly_braking:
            tp_brake += 1
        elif is_brake_mode and not truly_braking:
            fp_brake += 1
        elif not is_brake_mode and truly_braking:
            fn_brake += 1

    print(f"ACCEL detection:")
    print(f"  True Positives:  {tp_accel:5}")
    print(f"  False Positives: {fp_accel:5} (mode=ACCEL but not actually accelerating)")
    print(f"  False Negatives: {fn_accel:5} (accelerating but mode!=ACCEL)")
    if tp_accel + fp_accel > 0:
        precision = tp_accel / (tp_accel + fp_accel)
        print(f"  Precision: {100*precision:.1f}% (want >80%)")
    if tp_accel + fn_accel > 0:
        recall = tp_accel / (tp_accel + fn_accel)
        print(f"  Recall: {100*recall:.1f}%")

    print(f"\nBRAKE detection:")
    print(f"  True Positives:  {tp_brake:5}")
    print(f"  False Positives: {fp_brake:5} (mode=BRAKE but not actually braking)")
    print(f"  False Negatives: {fn_brake:5} (braking but mode!=BRAKE)")
    if tp_brake + fp_brake > 0:
        precision = tp_brake / (tp_brake + fp_brake)
        print(f"  Precision: {100*precision:.1f}% (want >80%)")
    if tp_brake + fn_brake > 0:
        recall = tp_brake / (tp_brake + fn_brake)
        print(f"  Recall: {100*recall:.1f}%")
    print()

    # === lon_g vs TRUE ACCEL CORRELATION ===
    print("--- lon_g vs GROUND TRUTH CORRELATION ---")
    if len(true_accels) >= 10:
        # Compare lon_g with speed-derived acceleration
        lon_g_shifted = lon_g_vals[1:]  # Align with true_accels
        if len(lon_g_shifted) == len(true_accels):
            # Calculate correlation
            mean_lon = statistics.mean(lon_g_shifted)
            mean_true = statistics.mean(true_accels)

            numerator = sum((l - mean_lon) * (t - mean_true)
                          for l, t in zip(lon_g_shifted, true_accels))
            denom_lon = sum((l - mean_lon)**2 for l in lon_g_shifted) ** 0.5
            denom_true = sum((t - mean_true)**2 for t in true_accels) ** 0.5

            if denom_lon > 0 and denom_true > 0:
                correlation = numerator / (denom_lon * denom_true)
                print(f"Correlation coefficient: {correlation:.3f} (want >0.7)")

            # Calculate error
            errors = [l - t for l, t in zip(lon_g_shifted, true_accels)]
            mean_error = statistics.mean(errors)
            rmse = (sum(e**2 for e in errors) / len(errors)) ** 0.5
            print(f"Mean error (lon_g - true): {mean_error:.4f}g (bias)")
            print(f"RMSE: {rmse:.4f}g")
    print()

    # === DIAGNOSTIC SUMMARY ===
    print("--- DIAGNOSTIC SUMMARY ---")
    issues = []

    # Check for lon_g bias
    if stable_lon:
        bias = statistics.mean(stable_lon)
        if abs(bias) > 0.03:
            issues.append(f"lon_g bias detected: {bias:.3f}g (expect ~0)")

    # Check for ACCEL/BRAKE imbalance
    if total_brake > 0 and total_accel / total_brake > 3:
        issues.append(f"ACCEL/BRAKE ratio too high: {total_accel/total_brake:.1f}x (suggests positive bias)")
    if total_accel > 0 and total_brake / total_accel > 3:
        issues.append(f"BRAKE/ACCEL ratio too high: {total_brake/total_accel:.1f}x (suggests negative bias)")

    # Check false positive rates
    if tp_accel + fp_accel > 0 and fp_accel / (tp_accel + fp_accel) > 0.3:
        issues.append(f"High ACCEL false positive rate: {100*fp_accel/(tp_accel+fp_accel):.0f}%")
    if tp_brake + fp_brake > 0 and fp_brake / (tp_brake + fp_brake) > 0.3:
        issues.append(f"High BRAKE false positive rate: {100*fp_brake/(tp_brake+fp_brake):.0f}%")

    # Check OrientationCorrector (if fusion data available)
    if has_fusion:
        avg_pitch_conf = statistics.mean([float(r['pitch_conf']) for r in rows])
        if avg_pitch_conf < 30:
            issues.append(f"Low OrientationCorrector confidence: {avg_pitch_conf:.0f}% (need more driving to learn)")

        avg_gps_weight = statistics.mean([float(r['gps_weight']) for r in rows])
        if avg_gps_weight > 0.8:
            issues.append(f"High GPS weight: {avg_gps_weight:.0f}% (IMU correction not trusted yet)")

    if issues:
        print("ISSUES DETECTED:")
        for issue in issues:
            print(f"  - {issue}")
    else:
        print("No major issues detected")

    # Recommendations
    print()
    print("--- RECOMMENDATIONS ---")
    if has_fusion:
        avg_pitch_conf = statistics.mean([float(r['pitch_conf']) for r in rows])
        if avg_pitch_conf < 50:
            print("• Drive more with varied acceleration/braking to train OrientationCorrector")

        if stable_lon:
            bias = statistics.mean(stable_lon)
            if bias > 0.05:
                print(f"• Consider raising ACCEL threshold by ~{bias:.2f}g to compensate for positive bias")
                print(f"• Consider lowering BRAKE threshold by ~{bias:.2f}g to improve brake detection")
            elif bias < -0.05:
                print(f"• Consider lowering ACCEL threshold by ~{abs(bias):.2f}g to compensate for negative bias")
    else:
        print("• Re-record with latest firmware to get fusion diagnostics for detailed analysis")

    print(f"\n{'='*60}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python analyze_telemetry.py <csvfile>")
        sys.exit(1)
    analyze_telemetry(sys.argv[1])
