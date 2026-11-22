#!/usr/bin/env python3
"""Summarize Session CSV quality without exposing route coordinates or timestamps.

This is an observation report, not a physical accuracy or classifier evaluation.
GPS CSV has a separate receiver-epoch analyzer: analyze_gps_timestamps.py.
"""
import argparse
import csv
import json
import math
from collections import Counter


class Aggregate:
    """Constant-memory finite-value statistics."""

    def __init__(self):
        self.count = 0
        self.mean = 0.0
        self.low = None
        self.high = None

    def add(self, value):
        self.count += 1
        self.mean = self.mean * ((self.count - 1) / self.count) + value / self.count
        self.low = value if self.low is None else min(self.low, value)
        self.high = value if self.high is None else max(self.high, value)

    def result(self):
        if not self.count:
            return None
        return dict(count=self.count, min=self.low, max=self.high, mean=self.mean)


def analyze(rows):
    """Consume Session CSV rows once; quality fields absent in legacy files stay unknown."""
    samples = Counter(total=0, speed_qualified=0, force_qualified=0)
    provenance = Counter(synthetic=0, device_labelled=0, unknown=0)
    excluded = Counter()
    continuity = Counter()
    browser, device = Aggregate(), Aggregate()
    speed, longitudinal, lateral = Aggregate(), Aggregate(), Aggregate()
    previous = None
    for line, row in enumerate(rows, 2):
        def number(key, optional=False, integer=False, low=None, high=None):
            value = row.get(key)
            if optional and value in (None, ''):
                return None
            try:
                result = float(value)
            except (TypeError, ValueError) as error:
                raise ValueError(f'Row {line}: invalid {key}') from error
            if (not math.isfinite(result) or (integer and not result.is_integer())
                    or (low is not None and result < low) or (high is not None and result > high)):
                raise ValueError(f'Row {line}: invalid {key}')
            return int(result) if integer else result

        event = dict(time=number('time', integer=True, low=0, high=2**53 - 1), speed=number('speed'),
                     lng=number('lon_g'), latg=number('lat_g'),
                     valid=number('gps_valid', integer=True, low=0, high=1),
                     gps=number('gps_age_ms', optional=True, low=0),
                     imu=number('imu_age_ms', optional=True, low=0),
                     clock=number('device_time_ms', optional=True, integer=True, low=0, high=2**32 - 1),
                     boot=number('gps_boot_id', optional=True, integer=True, low=0, high=2**32 - 1),
                     synthetic=number('synthetic', optional=True, integer=True, low=0, high=1))
        samples['total'] += 1
        label = 'unknown' if event['synthetic'] is None else 'synthetic' if event['synthetic'] else 'device_labelled'
        provenance[label] += 1
        for key, limit in [('gps', 300), ('imu', 200)]:
            if event[key] is None:
                excluded[f'{key}_age_unavailable'] += 1
            elif event[key] > limit:
                excluded[f'{key}_stale'] += 1
        if not event['valid']:
            excluded['gps_invalid'] += 1
        if event['speed'] < 0:
            excluded['negative_speed'] += 1
        speed_ok = event['valid'] == 1 and event['gps'] is not None and event['gps'] <= 300 and event['speed'] >= 0
        if speed_ok:
            speed.add(event['speed'])  # Session CSV speed is already km/h; GPS CSV is different.
            samples['speed_qualified'] += 1
        if speed_ok and event['imu'] is not None and event['imu'] <= 200 and event['speed'] >= 10:
            longitudinal.add(event['lng'])
            lateral.add(event['latg'])
            samples['force_qualified'] += 1
        elif speed_ok and event['speed'] < 10:
            excluded['below_force_speed_gate'] += 1
        if previous is not None:
            dt = event['time'] - previous['time']
            if dt < 0:
                continuity['browser_reversals'] += 1
            elif dt == 0:
                continuity['browser_duplicates'] += 1
            else:
                browser.add(dt)
                continuity['browser_gaps_over_250ms'] += dt > 250
            if event['boot'] is not None and previous['boot'] is not None and event['boot'] != previous['boot']:
                continuity['boot_changes'] += 1
            elif event['boot'] is None or previous['boot'] is None or event['clock'] is None or previous['clock'] is None:
                continuity['device_clock_pairs_unavailable'] += 1
            else:
                dt = (event['clock'] - previous['clock']) % 2**32
                if dt == 0:
                    continuity['device_duplicates'] += 1
                elif dt >= 2**31:
                    continuity['device_reversals'] += 1
                else:
                    device.add(dt)
                    continuity['device_gaps_over_250ms'] += dt > 250
                    continuity['device_wraps'] += event['clock'] < previous['clock']
        previous = event
    return dict(
        version=1, scope='session-csv-observations', samples=dict(samples), provenance=dict(provenance),
        excluded=dict(excluded), continuity=dict(continuity),
        cadence=dict(browser_interval_ms=browser.result(), device_interval_ms=device.result()),
        speed_kmh=speed.result(),
        force_samples_g=dict(longitudinal=longitudinal.result(), lateral=lateral.result()),
        interpretation=[
            'Counts describe samples, not time shares or independent maneuvers; exclusions can overlap.',
            'Browser intervals use phone wall time and include suspension or clock changes; '
            'no recording duration is inferred.',
            'Device intervals require the same boot and use uint32 wrap handling; '
            'gaps remain in the interval statistics.',
            'Speed uses valid GPS aged at most 300 ms. Forces also require IMU age at most 200 ms '
            'and speed at least 10 km/h.',
            'Force statistics describe individual displayed estimates, not sustained Peaks, '
            'calibrated grip or independent truth.',
            'Device-labelled provenance is a CSV label, not certification. '
            'Keep synthetic and driving reports separate.',
            'Use GPS CSV for receiver cadence; dashboard speed differences do not establish '
            'physical measurement accuracy.',
        ])


def analyze_telemetry(filename):
    """Read a dashboard export and return the privacy-preserving report."""
    with open(filename, newline='', encoding='utf-8-sig') as source:
        reader = csv.DictReader(source)
        required = {'time', 'speed', 'lat_g', 'lon_g', 'gps_valid'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError('Expected Session CSV columns: time, speed, lat_g, lon_g, gps_valid')
        return analyze(reader)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', help='Session CSV from Saved data (not GPS CSV)')
    args = parser.parse_args()
    try:
        report = analyze_telemetry(args.csv)
    except OSError:
        parser.error('Could not read Session CSV')
    except ValueError as error:
        parser.error(str(error))
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
