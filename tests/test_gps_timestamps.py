import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('gps_analysis', Path(__file__).parents[1] / 'tools/python/analyze_gps_timestamps.py')
gps = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gps)


def row(seq, host, utc, **extra):
    data = dict(boot_id=7, rmc_seq=seq, host_parse_us=host, rmc_active=1,
                fix_accepted=1, boundary=0, synthetic=0, receiver_utc_us_of_day=utc,
                receiver_date_ddmmyy=201126, missing_before=0)
    data.update(extra)
    return {k: str(v) for k, v in data.items()}


class TimestampAnalysis(unittest.TestCase):
    def test_batching_duplicates_and_gaps(self):
        report = gps.analyze([row(1, 100000, 40000, boundary=1), row(2, 100100, 80000),
                              row(3, 140000, 80000), row(7, 300000, 240000, missing_before=3)])
        self.assertEqual(report['counts']['closely_batched_arrivals'], 1)
        self.assertEqual(report['counts']['duplicate_receiver_epochs'], 1)
        self.assertEqual(report['counts']['missing_log_events'], 3)
        self.assertEqual(report['receiver_interval_ms']['median'], 40)
        self.assertAlmostEqual(report['host_minus_receiver_interval_ms']['median'], -39.9)

    def test_midnight_reboot_missing_clock_and_synthetic_data(self):
        report = gps.analyze([row(1, 1, 86399960000, boundary=1),
                              row(2, 40001, 0, receiver_date_ddmmyy=211126),
                              row(1, 1, '', boot_id=8, boundary=2, synthetic=1)])
        self.assertEqual(report['counts']['segments'], 2)
        self.assertEqual(report['counts']['date_transitions'], 1)
        self.assertEqual(report['counts']['synthetic_events'], 1)
        self.assertEqual(report['counts']['missing_receiver_time'], 1)
        self.assertIsNone(report['receiver_interval_ms'])

    def test_wrong_csv_and_nonfinite_fields_are_rejected(self):
        for data in [{'time': '1'}, row(1, 'NaN', 0), row(1, -1, 0)]:
            with self.assertRaises(ValueError):
                gps.analyze([data])


if __name__ == '__main__':
    unittest.main()
