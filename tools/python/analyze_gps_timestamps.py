#!/usr/bin/env python3
"""Summarize Blackbox GPS CSV timing evidence without printing location traces."""
import argparse
import csv
import json
import math
from collections import Counter


def summary(values):
    if not values:
        return None
    ordered = sorted(values)
    return {
        'count': len(ordered),
        'median': ordered[(len(ordered) - 1) // 2],
        'p95': ordered[math.ceil(len(ordered) * .95) - 1],
        'min': ordered[0], 'max': ordered[-1],
    }


def analyze(rows):
    counts = Counter()
    intervals, arrivals, residuals = [], [], []
    previous = None
    for line, row in enumerate(rows, 2):
        def number(key, optional=False):
            value = row.get(key)
            if optional and value == '':
                return None
            try:
                result = int(value)
            except (TypeError, ValueError) as error:
                raise ValueError(f'Row {line}: invalid {key}') from error
            if result < 0:
                raise ValueError(f'Row {line}: negative {key}')
            return result

        required = ('boot_id', 'rmc_seq', 'host_parse_us', 'rmc_active',
                    'fix_accepted', 'boundary', 'synthetic')
        event = {k: number(k) for k in required}
        for k in ('receiver_utc_us_of_day', 'receiver_date_ddmmyy', 'missing_before'):
            event[k] = number(k, optional=True)
        counts['events'] += 1
        counts['receiver_inactive'] += event['rmc_active'] == 0
        counts['not_accepted'] += event['fix_accepted'] == 0
        counts['synthetic_events'] += event['synthetic'] != 0
        counts['missing_receiver_time'] += event['receiver_utc_us_of_day'] is None
        counts['missing_receiver_date'] += event['receiver_date_ddmmyy'] is None
        counts['missing_log_events'] += event['missing_before'] or 0
        if previous is None or event['boot_id'] != previous['boot_id'] or event['boundary']:
            counts['segments'] += 1
        else:
            seq_delta = event['rmc_seq'] - previous['rmc_seq']
            if seq_delta <= 0:
                counts['nonincreasing_sequence'] += 1
            host_delta = event['host_parse_us'] - previous['host_parse_us']
            if host_delta < 0:
                counts['host_clock_reversals'] += 1
            else:
                arrivals.append(host_delta / 1000)
            utc, old_utc = event['receiver_utc_us_of_day'], previous['receiver_utc_us_of_day']
            date, old_date = event['receiver_date_ddmmyy'], previous['receiver_date_ddmmyy']
            # Do not guess a century, leap-second model, or date during outages.
            # Split on date changes rather than inventing a midnight interval.
            if date != old_date:
                counts['date_transitions'] += 1
            elif utc is not None and old_utc is not None and date is not None:
                delta = utc - old_utc
                if delta == 0:
                    counts['duplicate_receiver_epochs'] += 1
                elif delta < 0:
                    counts['receiver_clock_reversals'] += 1
                elif seq_delta == 1 and host_delta >= 0:
                    intervals.append(delta / 1000)
                    residuals.append((host_delta - delta) / 1000)
                    counts['closely_batched_arrivals'] += host_delta <= 1000 and delta >= 10000
        previous = event
    return {
        'counts': dict(counts),
        'receiver_interval_ms': summary(intervals),
        'host_parse_interval_ms': summary(arrivals),
        'host_minus_receiver_interval_ms': summary(residuals),
        'interpretation': [
            'Receiver intervals describe exported RMC epochs, not absolute positioning or lap accuracy.',
            'Host times mark sentence parsing, including UART buffering and scheduling delay; they are not PPS.',
            'Interval differences cancel constant clock offset but mix clock drift with variable delivery delay.',
            'Missing log events exclude loss before the parser and unknown history before each segment.',
            'The first batch may include up to 64 pre-recording events; boundary 2 identifies a new device boot.',
            'Date transitions and missing dates are excluded from receiver-interval statistics.',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', help='GPS CSV from Saved data (not the ordinary Session CSV)')
    args = parser.parse_args()
    try:
        with open(args.csv, newline='', encoding='utf-8-sig') as source:
            report = analyze(csv.DictReader(source))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
