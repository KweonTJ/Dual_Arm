#!/usr/bin/env python3
import csv
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from monitor_view import CsvRecorder, MonitorView, JOINTS, demo_records, records_from_message


class MonitorTests(unittest.TestCase):
    def test_waiting_does_not_fabricate_values(self):
        text = MonitorView().render(0)
        self.assertIn('WAITING', text)
        self.assertNotIn('11.1', text)
        self.assertEqual(text.count('WAIT\n'), 12)

    def test_fresh_then_stale_and_recovery(self):
        view = MonitorView(2)
        view.update(demo_records(), 10, '42.0')
        self.assertNotIn('STALE', view.render(11))
        self.assertIn('STALE', view.render(12))
        self.assertIn('과거 값', view.render(12))
        view.update(demo_records(), 13)
        self.assertNotIn('STALE', view.render(13))

    def test_fault_hides_even_supplied_old_numbers_and_off_claim(self):
        records = demo_records()
        records[0]['values']['state'] = 'FAULT'
        records[0]['values']['torque_off_unconfirmed'] = 'ID 1 timeout'
        for record in records:
            record['level'] = 2
            record['values']['sample_valid'] = 'false'
        view = MonitorView()
        view.update(records, 0)
        text = view.render(0)
        self.assertIn('FAULT', text)
        self.assertIn('ID 1 timeout', text)
        self.assertNotIn('11.1', text)
        rows = [line for line in text.splitlines() if line.startswith(('L.', 'R.'))]
        self.assertTrue(all(' OFF ' not in row for row in rows))

    def test_ros_byte_levels_and_topic_filter(self):
        message = SimpleNamespace(status=[
            SimpleNamespace(name='dual_arm/system', level=bytes([2]), message='fault',
                            values=[SimpleNamespace(key='state', value='FAULT')]),
            SimpleNamespace(name='unrelated', level=bytes([0]), message='other', values=[])])
        records = records_from_message(message)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['level'], 2)
        self.assertEqual(records[0]['values']['state'], 'FAULT')

    def test_missing_joint_does_not_reuse_old_reading(self):
        view = MonitorView()
        view.update(demo_records(), 0)
        view.update(demo_records()[:-1], 1)
        row = next(line for line in view.render(1).splitlines() if line.startswith('R.outer_finger'))
        self.assertIn('NO DATA', row)
        self.assertNotIn('11.1', row)

    def test_nonfinite_and_escape_text_not_rendered_as_values_or_commands(self):
        records = demo_records()
        records[0]['message'] = '\x1b[2Jtest\nmessage'
        records[1]['values']['voltage_v'] = 'nan'
        view = MonitorView()
        view.update(records, 0)
        text = view.render(0)
        self.assertNotIn('\x1b', text)
        self.assertNotIn('nan', text)

    def test_csv_preserves_fault_and_blanks_measurements_without_overwriting(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.csv'
            recorder = CsvRecorder(path)
            records = demo_records()
            recorder.write(records, '42.000000001')
            for record in records:
                record['level'] = 2
                record['values']['sample_valid'] = 'false'
            recorder.write(records, '43.000000001')
            recorder.close()
            with path.open(encoding='utf-8-sig') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 26)
            self.assertEqual(rows[1]['voltage_v'], '11.1')
            self.assertEqual(rows[14]['voltage_v'], '')
            self.assertEqual(rows[14]['level'], '2')
            self.assertEqual(rows[14]['ros_stamp'], '43.000000001')
            with self.assertRaises(FileExistsError):
                CsvRecorder(path)


if __name__ == '__main__':
    unittest.main()
