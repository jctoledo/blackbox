#!/usr/bin/env python3
"""
Analyze telemetry CSV files from blackbox device.

Usage: python analyze_telemetry.py <csvfile>

CSV format expected (new format with fusion diagnostics):
time,speed,ax,ay,wz,mode,lat_g,lon_g,gps_lat,gps_lon,gps_valid,
lon_imu,lon_gps,gps_weight,pitch_corr,pitch_conf,roll_corr,roll_conf,tilt_x,tilt_y

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
    intervals = [times[i] - times[i - 1] for i in range(1, len(times))]
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
    n = len(rows)
    print(f"Time distribution: stopped(<2)={100 * stopped / n:.0f}%, "
          f"slow(2-20)={100 * slow / n:.0f}%, "
          f"medium(20-50)={100 * medium / n:.0f}%, "
          f"fast(>50)={100 * fast / n:.0f}%")
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
        speed_diff = abs(float(rows[i]['speed']) - float(rows[i - 1]['speed']))
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
        valid_gps = [
            (imu, gps, w)
            for imu, gps, w in zip(lon_imu_vals, lon_gps_vals, gps_weights)
            if abs(gps) > 0.001
        ]

        if valid_gps:
            imu_valid = [v[0] for v in valid_gps]
            gps_valid = [v[1] for v in valid_gps]

            print(f"lon_imu range: {min(lon_imu_vals):.3f} to {max(lon_imu_vals):.3f} m/s²")
            print(f"lon_gps range: {min(gps_valid):.3f} to {max(gps_valid):.3f} m/s²")

            # IMU vs GPS error
            errors = [i - g for i, g in zip(imu_valid, gps_valid)]
            mean_error = statistics.mean(errors)
            std_error = statistics.stdev(errors)
            print(f"IMU-GPS error: mean={mean_error:.3f} m/s² ({mean_error/9.81:.3f}g), "
                  f"std={std_error:.3f} m/s²")

            # Correlation between IMU and GPS
            if len(imu_valid) >= 10:
                mean_imu = statistics.mean(imu_valid)
                mean_gps = statistics.mean(gps_valid)
                numerator = sum(
                    (i - mean_imu) * (g - mean_gps)
                    for i, g in zip(imu_valid, gps_valid)
                )
                denom_imu = sum((i - mean_imu) ** 2 for i in imu_valid) ** 0.5
                denom_gps = sum((g - mean_gps) ** 2 for g in gps_valid) ** 0.5
                if denom_imu > 0 and denom_gps > 0:
                    corr = numerator / (denom_imu * denom_gps)
                    print(f"IMU-GPS correlation: {corr:.3f} (want >0.7)")

            # Sign agreement analysis
            sign_agree = sum(1 for i, g in zip(imu_valid, gps_valid)
                           if (i > 0.1 and g > 0.1) or (i < -0.1 and g < -0.1))
            sign_disagree = sum(1 for i, g in zip(imu_valid, gps_valid)
                              if (i > 0.1 and g < -0.1) or (i < -0.1 and g > 0.1))
            if sign_agree + sign_disagree > 0:
                sign_rate = sign_agree / (sign_agree + sign_disagree)
                print(f"Sign agreement: {100*sign_rate:.0f}% ({sign_agree}/{sign_agree+sign_disagree} "
                      f"samples where both |val|>0.1 m/s²)")

        # GPS weight distribution
        avg_weight = statistics.mean(gps_weights)
        high_gps = sum(1 for w in gps_weights if w > 0.7)
        mid_gps = sum(1 for w in gps_weights if 0.3 <= w <= 0.7)
        low_gps = sum(1 for w in gps_weights if w < 0.3)
        n = len(rows)
        print(f"GPS weight: mean={avg_weight:.2f}, high(>0.7)={100 * high_gps / n:.0f}%, "
              f"mid={100 * mid_gps / n:.0f}%, low(<0.3)={100 * low_gps / n:.0f}%")

        # OrientationCorrector status
        pitch_corrs = [float(r['pitch_corr']) for r in rows]
        pitch_confs = [float(r['pitch_conf']) for r in rows]
        roll_corrs = [float(r['roll_corr']) for r in rows]
        roll_confs = [float(r['roll_conf']) for r in rows]

        pc_mean, pc_min, pc_max = statistics.mean(pitch_corrs), min(pitch_corrs), max(pitch_corrs)
        print(f"Pitch correction: {pc_mean:.1f}° (range {pc_min:.1f}° to {pc_max:.1f}°)")
        pcf_mean, pcf_min, pcf_max = statistics.mean(pitch_confs), min(pitch_confs), max(pitch_confs)
        print(f"Pitch confidence: {pcf_mean:.0f}% (range {pcf_min:.0f}% to {pcf_max:.0f}%)")
        rc_mean, rc_min, rc_max = statistics.mean(roll_corrs), min(roll_corrs), max(roll_corrs)
        print(f"Roll correction: {rc_mean:.1f}° (range {rc_min:.1f}° to {rc_max:.1f}°)")
        rcf_mean, rcf_min, rcf_max = statistics.mean(roll_confs), min(roll_confs), max(roll_confs)
        print(f"Roll confidence: {rcf_mean:.0f}% (range {rcf_min:.0f}% to {rcf_max:.0f}%)")

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
    total = len(rows)
    for mode, count in sorted(modes.items()):
        name = mode_names.get(mode, f'UNKNOWN({mode})')
        print(f"  {name:15} {count:5} ({100 * count / total:5.1f}%)")

    total_accel = modes.get(1, 0) + modes.get(5, 0)
    total_brake = modes.get(2, 0) + modes.get(6, 0)
    print(f"\n  Total ACCEL modes: {total_accel} ({100 * total_accel / total:.1f}%)")
    print(f"  Total BRAKE modes: {total_brake} ({100 * total_brake / total:.1f}%)")
    print(f"  Ratio ACCEL/BRAKE: {total_accel/max(total_brake,1):.1f}x (should be ~1x for balanced driving)")
    print()

    # === GROUND TRUTH FROM SPEED CHANGES ===
    print("--- GROUND TRUTH (from speed changes) ---")

    # Calculate acceleration from speed changes
    true_accels = []
    for i in range(1, len(rows)):
        dt = (int(rows[i]['time']) - int(rows[i - 1]['time'])) / 1000.0  # seconds
        if dt > 0:
            dv = (float(rows[i]['speed']) - float(rows[i - 1]['speed'])) / 3.6  # m/s
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

    # Ground truth thresholds for determining "truly accelerating/braking"
    # (independent of mode detection thresholds)
    TRUE_ACCEL_THRESH = 0.05  # 0.05g from speed changes = real acceleration
    TRUE_BRAKE_THRESH = 0.05  # 0.05g from speed changes = real braking

    tp_accel = fp_accel = fn_accel = 0
    tp_brake = fp_brake = fn_brake = 0

    for i in range(1, len(rows)):
        mode = int(float(rows[i]['mode']))

        # Use speed-derived acceleration as ground truth
        dt = (int(rows[i]['time']) - int(rows[i - 1]['time'])) / 1000.0
        if dt > 0:
            dv = (float(rows[i]['speed']) - float(rows[i - 1]['speed'])) / 3.6
            true_accel_g = (dv / dt) / 9.80665
        else:
            true_accel_g = 0

        is_accel_mode = mode in [1, 5]
        is_brake_mode = mode in [2, 6]
        truly_accelerating = true_accel_g > TRUE_ACCEL_THRESH
        truly_braking = true_accel_g < -TRUE_BRAKE_THRESH

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
        print(f"  Precision: {100 * precision:.1f}% (want >80%)")
    if tp_accel + fn_accel > 0:
        recall = tp_accel / (tp_accel + fn_accel)
        print(f"  Recall: {100 * recall:.1f}%")

    print(f"\nBRAKE detection:")
    print(f"  True Positives:  {tp_brake:5}")
    print(f"  False Positives: {fp_brake:5} (mode=BRAKE but not actually braking)")
    print(f"  False Negatives: {fn_brake:5} (braking but mode!=BRAKE)")
    if tp_brake + fp_brake > 0:
        precision = tp_brake / (tp_brake + fp_brake)
        print(f"  Precision: {100 * precision:.1f}% (want >80%)")
    if tp_brake + fn_brake > 0:
        recall = tp_brake / (tp_brake + fn_brake)
        print(f"  Recall: {100 * recall:.1f}%")
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

            numerator = sum(
                (lon - mean_lon) * (true - mean_true)
                for lon, true in zip(lon_g_shifted, true_accels)
            )
            denom_lon = sum((lon - mean_lon) ** 2 for lon in lon_g_shifted) ** 0.5
            denom_true = sum((t - mean_true) ** 2 for t in true_accels) ** 0.5

            if denom_lon > 0 and denom_true > 0:
                correlation = numerator / (denom_lon * denom_true)
                print(f"Correlation coefficient: {correlation:.3f} (want >0.7)")

            # Calculate error
            errors = [lon - t for lon, t in zip(lon_g_shifted, true_accels)]
            mean_error = statistics.mean(errors)
            rmse = (sum(e ** 2 for e in errors) / len(errors)) ** 0.5
            print(f"Mean error (lon_g - true): {mean_error:.4f}g (bias)")
            print(f"RMSE: {rmse:.4f}g")
    print()

    # === INDIVIDUAL SOURCE CORRELATIONS WITH GROUND TRUTH ===
    if has_fusion and len(true_accels) >= 10:
        print("--- INDIVIDUAL SOURCE vs GROUND TRUTH ---")
        print("(Helps diagnose why IMU-GPS correlation might be low)")
        print()

        # Convert ground truth to m/s² for comparison
        true_accels_ms2 = [t * 9.80665 for t in true_accels]

        # Align arrays (true_accels is 1 shorter due to differencing)
        lon_imu_shifted = lon_imu_vals[1:]
        lon_gps_shifted = lon_gps_vals[1:]

        def calc_correlation(a, b):
            """Calculate Pearson correlation between two lists."""
            if len(a) != len(b) or len(a) < 10:
                return None
            mean_a, mean_b = statistics.mean(a), statistics.mean(b)
            num = sum((x - mean_a) * (y - mean_b) for x, y in zip(a, b))
            den_a = sum((x - mean_a) ** 2 for x in a) ** 0.5
            den_b = sum((y - mean_b) ** 2 for y in b) ** 0.5
            if den_a > 0 and den_b > 0:
                return num / (den_a * den_b)
            return None

        # Correlation of each source with ground truth
        corr_imu_gt = calc_correlation(lon_imu_shifted, true_accels_ms2)
        corr_gps_gt = calc_correlation(lon_gps_shifted, true_accels_ms2)

        if corr_imu_gt is not None:
            print(f"lon_imu vs ground_truth: {corr_imu_gt:.3f}")
        if corr_gps_gt is not None:
            print(f"lon_gps vs ground_truth: {corr_gps_gt:.3f}")

        # Analysis of what this means
        if corr_imu_gt is not None and corr_gps_gt is not None:
            print()
            if corr_imu_gt > 0.5 and corr_gps_gt > 0.5:
                print("Both sources correlate with ground truth - blending should help")
            elif corr_imu_gt > 0.5 > corr_gps_gt:
                print("IMU correlates better - GPS might be noisy or delayed")
            elif corr_gps_gt > 0.5 > corr_imu_gt:
                print("GPS correlates better - IMU orientation correction may need work")
            else:
                print("Neither source correlates well - fundamental issue to investigate")

        # Phase lag analysis using cross-correlation
        print()
        print("--- PHASE LAG ANALYSIS ---")
        print("(Checking if IMU and GPS are time-shifted)")

        # Simple lag detection: try shifting one signal
        best_lag = 0
        best_corr = corr_imu_gt if corr_imu_gt else 0

        for lag in range(-10, 11):  # Check lags from -10 to +10 samples
            if lag == 0:
                continue
            if lag > 0:
                imu_lagged = lon_imu_shifted[lag:]
                gt_lagged = true_accels_ms2[:-lag]
            else:
                imu_lagged = lon_imu_shifted[:lag]
                gt_lagged = true_accels_ms2[-lag:]

            if len(imu_lagged) < 10:
                continue

            corr = calc_correlation(imu_lagged, gt_lagged)
            if corr and corr > best_corr:
                best_corr = corr
                best_lag = lag

        if best_lag != 0:
            # Calculate lag in ms based on sample rate
            if intervals:
                lag_ms = best_lag * statistics.mean(intervals)
                print(f"IMU appears shifted by {best_lag} samples ({lag_ms:.0f}ms)")
                print(f"Correlation improves from {corr_imu_gt:.3f} to {best_corr:.3f} with lag correction")
                if abs(lag_ms) > 50:
                    print("⚠️  Significant timing offset detected - check filtering/buffering")
        else:
            print(f"No significant phase lag detected (IMU-GT correlation: {corr_imu_gt:.3f})")

        # Same for GPS
        best_lag_gps = 0
        best_corr_gps = corr_gps_gt if corr_gps_gt else 0

        for lag in range(-10, 11):
            if lag == 0:
                continue
            if lag > 0:
                gps_lagged = lon_gps_shifted[lag:]
                gt_lagged = true_accels_ms2[:-lag]
            else:
                gps_lagged = lon_gps_shifted[:lag]
                gt_lagged = true_accels_ms2[-lag:]

            if len(gps_lagged) < 10:
                continue

            corr = calc_correlation(gps_lagged, gt_lagged)
            if corr and corr > best_corr_gps:
                best_corr_gps = corr
                best_lag_gps = lag

        if best_lag_gps != 0 and intervals:
            lag_ms = best_lag_gps * statistics.mean(intervals)
            print(f"GPS appears shifted by {best_lag_gps} samples ({lag_ms:.0f}ms)")
            print(f"Correlation improves from {corr_gps_gt:.3f} to {best_corr_gps:.3f} with lag correction")

        # Extended lag search if we hit the boundary
        if abs(best_lag) >= 9 or abs(best_lag_gps) >= 9:
            print()
            print("--- EXTENDED LAG SEARCH (boundary hit) ---")
            for lag in range(-50, 51, 5):
                if lag == 0:
                    continue
                if lag > 0:
                    imu_lagged = lon_imu_shifted[lag:]
                    gt_lagged = true_accels_ms2[:-lag]
                else:
                    imu_lagged = lon_imu_shifted[:lag]
                    gt_lagged = true_accels_ms2[-lag:]
                if len(imu_lagged) < 10:
                    continue
                corr = calc_correlation(imu_lagged, gt_lagged)
                if corr and abs(lag) <= 50:
                    lag_ms = lag * statistics.mean(intervals) if intervals else 0
                    if corr > 0.3:  # Only show promising lags
                        print(f"  lag={lag:+3d} ({lag_ms:+5.0f}ms): IMU corr={corr:.3f}")

        print()

        # === SANITY CHECK: lon_gps vs speed derivative ===
        print("--- SANITY CHECK: lon_gps vs CSV speed derivative ---")
        print("(These SHOULD be nearly identical if derived from same GPS data)")

        # The script's ground truth is dv/dt from CSV speed column
        # lon_gps is the firmware's GPS acceleration
        # If they differ, something is wrong with data flow

        # Direct comparison
        if len(lon_gps_shifted) == len(true_accels_ms2):
            errors = [g - t for g, t in zip(lon_gps_shifted, true_accels_ms2)]
            mean_diff = statistics.mean(errors)
            std_diff = statistics.stdev(errors)
            max_diff = max(abs(e) for e in errors)
            print(f"lon_gps - ground_truth: mean={mean_diff:.3f}, std={std_diff:.3f}, max={max_diff:.3f} m/s²")

            if std_diff > 1.0:
                print("⚠️  Large variance suggests lon_gps and speed column are from different sources/times")
            elif abs(mean_diff) > 0.5:
                print("⚠️  Systematic offset suggests calibration or sign issue")

        print()

        # === ROLLING CORRELATION (does it improve after warmup?) ===
        print("--- ROLLING CORRELATION (IMU vs Ground Truth) ---")
        print("(Checking if correlation improves over time)")

        window_size = len(lon_imu_shifted) // 5  # 5 windows
        if window_size >= 20:
            for i in range(5):
                start = i * window_size
                end = start + window_size
                window_imu = lon_imu_shifted[start:end]
                window_gt = true_accels_ms2[start:end]
                corr = calc_correlation(window_imu, window_gt)
                time_start = (start * statistics.mean(intervals)) / 1000 if intervals else 0
                time_end = (end * statistics.mean(intervals)) / 1000 if intervals else 0
                if corr:
                    print(f"  {time_start:5.0f}s - {time_end:5.0f}s: corr={corr:+.3f}")

        print()

        # === MAGNITUDE ANALYSIS ===
        print("--- MAGNITUDE ANALYSIS ---")
        print("(Are the signals the right scale?)")

        imu_std = statistics.stdev(lon_imu_shifted)
        gps_std = statistics.stdev(lon_gps_shifted)
        gt_std = statistics.stdev(true_accels_ms2)

        print(f"Std dev: lon_imu={imu_std:.3f}, lon_gps={gps_std:.3f}, ground_truth={gt_std:.3f} m/s²")

        imu_rms = (sum(x**2 for x in lon_imu_shifted) / len(lon_imu_shifted)) ** 0.5
        gps_rms = (sum(x**2 for x in lon_gps_shifted) / len(lon_gps_shifted)) ** 0.5
        gt_rms = (sum(x**2 for x in true_accels_ms2) / len(true_accels_ms2)) ** 0.5

        print(f"RMS:     lon_imu={imu_rms:.3f}, lon_gps={gps_rms:.3f}, ground_truth={gt_rms:.3f} m/s²")

        if imu_std > 0 and gt_std > 0:
            scale_ratio = imu_std / gt_std
            print(f"Scale ratio (IMU/GT): {scale_ratio:.2f}x (should be ~1.0)")
            if scale_ratio > 2 or scale_ratio < 0.5:
                print("⚠️  Scale mismatch - IMU may have wrong units or gain")

        print()

        # === CORRELATION vs PITCH CORRECTION ===
        print("--- CORRELATION vs PITCH CORRECTION ---")
        print("(Checking if sign-flipping correlates with OrientationCorrector state)")

        pitch_corrs_shifted = pitch_corrs[1:]  # Align with derivatives
        window_size = len(lon_imu_shifted) // 10  # 10 windows for finer resolution

        if window_size >= 10:
            print(f"{'Window':>12} {'Corr':>8} {'Pitch':>8} {'PitchΔ':>8}")
            print("-" * 40)
            prev_pitch = None
            for i in range(10):
                start = i * window_size
                end = start + window_size
                window_imu = lon_imu_shifted[start:end]
                window_gt = true_accels_ms2[start:end]
                window_pitch = pitch_corrs_shifted[start:end]

                corr = calc_correlation(window_imu, window_gt)
                avg_pitch = statistics.mean(window_pitch)
                pitch_change = avg_pitch - prev_pitch if prev_pitch is not None else 0
                prev_pitch = avg_pitch

                time_start = (start * statistics.mean(intervals)) / 1000 if intervals else 0
                time_end = (end * statistics.mean(intervals)) / 1000 if intervals else 0

                if corr:
                    flag = " <<<" if corr < 0 else ""
                    print(f"{time_start:5.0f}-{time_end:5.0f}s {corr:+.3f}   {avg_pitch:+.1f}°   {pitch_change:+.1f}°{flag}")

        # Check for systematic relationship
        print()
        print("Analysis: If negative correlation windows have different pitch than positive,")
        print("the OrientationCorrector may be overcorrecting or oscillating.")

        print()

        # === SAMPLE-LEVEL COMPARISON ===
        print("--- SAMPLE COMPARISON (first 20 where both active) ---")
        print(f"{'#':>4} {'lon_imu':>8} {'lon_gps':>8} {'GT':>8} {'IMU-GT':>8} {'Sign':>6}")
        print("-" * 50)

        count = 0
        for i, (imu, gps, gt) in enumerate(zip(lon_imu_shifted, lon_gps_shifted, true_accels_ms2)):
            if abs(gt) > 0.5:  # Only look at significant accelerations
                diff = imu - gt
                sign_match = "✓" if (imu > 0) == (gt > 0) else "✗"
                print(f"{i:4d} {imu:+8.2f} {gps:+8.2f} {gt:+8.2f} {diff:+8.2f} {sign_match:>6}")
                count += 1
                if count >= 20:
                    break

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
