import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('session_analysis', Path(__file__).parents[1] / 'tools/python/analyze_telemetry.py')
session = importlib.util.module_from_spec(spec)
spec.loader.exec_module(session)


def row(**changes):
    """Create one current Session CSV observation with explicit quality."""
    values = dict(time=1000, speed=36, lat_g=.1, lon_g=.2, gps_valid=1,
                  gps_age_ms=40, imu_age_ms=10, device_time_ms=100,
                  gps_boot_id=7, synthetic=0)
    values.update(changes)
    return {key: str(value) for key, value in values.items()}


class SessionAnalysis(unittest.TestCase):
    """Exercise observable clock, quality, unit and privacy boundaries."""
    def test_empty_and_single_sample(self):
        """Short captures have unknown intervals without crashing."""
        empty = session.analyze([])
        self.assertEqual(empty['samples']['total'], 0)
        self.assertIsNone(empty['cadence']['browser_interval_ms'])
        report = session.analyze([row()])
        self.assertEqual(report['speed_kmh']['max'], 36)
        self.assertEqual(report['samples']['force_qualified'], 1)
        self.assertIsNone(report['cadence']['device_interval_ms'])

    def test_clock_domains_wrap_gaps_reboot_and_reverse(self):
        """Device and browser discontinuities remain distinct."""
        rows = [row(device_time_ms=4294967280), row(time=1033, device_time_ms=17),
                row(time=1033, device_time_ms=17), row(time=1000, device_time_ms=10),
                row(time=1400, device_time_ms=410), row(time=1433, device_time_ms=5, gps_boot_id=8)]
        report = session.analyze(rows)
        counts = report['continuity']
        self.assertEqual(counts['device_wraps'], 1)
        self.assertEqual(counts['device_duplicates'], 1)
        self.assertEqual(counts['device_reversals'], 1)
        self.assertEqual(counts['device_gaps_over_250ms'], 1)
        self.assertEqual(counts['boot_changes'], 1)
        self.assertEqual(counts['browser_duplicates'], 1)
        self.assertEqual(counts['browser_reversals'], 1)
        self.assertEqual(report['cadence']['device_interval_ms']['count'], 2)
        self.assertEqual(report['cadence']['device_interval_ms']['min'], 33)

    def test_quality_provenance_and_units(self):
        """Only fresh evidence enters the documented km/h and g aggregates."""
        report = session.analyze([row(), row(speed=300, gps_age_ms=301),
                                  row(speed=72, imu_age_ms='', synthetic=1),
                                  row(speed=100, gps_valid=0), row(speed=-1)])
        self.assertEqual(report['speed_kmh']['max'], 72)
        self.assertEqual(report['samples']['speed_qualified'], 2)
        self.assertEqual(report['samples']['force_qualified'], 1)
        self.assertEqual(report['provenance']['synthetic'], 1)
        self.assertEqual(report['excluded']['imu_age_unavailable'], 1)
        self.assertEqual(report['force_samples_g']['longitudinal']['mean'], .2)

    def test_known_reboot_does_not_require_device_clock(self):
        """Known boot changes survive missing device timestamps."""
        report = session.analyze([row(device_time_ms=''), row(device_time_ms='', gps_boot_id=8)])
        self.assertEqual(report['continuity']['boot_changes'], 1)
        self.assertIsNone(report['cadence']['device_interval_ms'])

    def test_legacy_unknown_quality_does_not_certify_forces_or_speed(self):
        """Missing legacy freshness cannot manufacture qualified results."""
        report = session.analyze([dict(time='1', speed='36', lat_g='.1', lon_g='.2', gps_valid='1')])
        self.assertIsNone(report['speed_kmh'])
        self.assertIsNone(report['force_samples_g']['lateral'])
        self.assertEqual(report['provenance']['unknown'], 1)
        self.assertEqual(report['excluded']['gps_age_unavailable'], 1)

    def test_malformed_required_or_optional_values_rejected(self):
        """Invalid numeric fields are errors rather than silent exclusions."""
        for data in [row(speed='NaN'), row(time='inf'), row(gps_valid=2),
                     row(device_time_ms=4294967296), row(gps_age_ms='bad'), {}]:
            with self.subTest(data=data), self.assertRaises(ValueError):
                session.analyze([data])

    def test_report_has_no_trace_or_absolute_timestamps(self):
        """Reduced reports do not echo the source route or wall timestamp."""
        import json
        report = json.dumps(session.analyze([row(gps_lat=37.123456, gps_lon=-122.123456, time=987654321)]))
        for secret in ['37.123456', '-122.123456', '987654321', 'ground truth', 'precision', 'recall']:
            self.assertNotIn(secret, report)

    def test_csv_header_bom_and_large_finite_observations(self):
        """Header-only/BOM files and opposite large values remain well-defined."""
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8-sig') as source:
            source.write('time,speed,lat_g,lon_g,gps_valid\n')
            source.flush()
            self.assertEqual(session.analyze_telemetry(source.name)['samples']['total'], 0)
        with tempfile.NamedTemporaryFile(mode='w') as source:
            source.write('browser_received_ms,boot_id\n')
            source.flush()
            with self.assertRaisesRegex(ValueError, 'Expected Session CSV'):
                session.analyze_telemetry(source.name)
        report = session.analyze([row(lon_g=1e308), row(lon_g=-1e308)])
        self.assertEqual(report['force_samples_g']['longitudinal']['mean'], 0)


if __name__ == '__main__':
    unittest.main()
