"""Independent R_T19 regression tests; all writes stay in temporary directories."""
import contextlib
import csv
from decimal import Decimal
import copy
import hashlib
import io
import json
from pathlib import Path
import sqlite3
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from tools.stamp_drift import analyze as analyzer

FIXTURES = Path(__file__).with_name('fixtures')


def header(stamp, endian='<'):
    sec, ns = divmod(stamp, 10**9)
    return (b'\x00\x01\x00\x00' if endian == '<' else b'\x00\x00\x00\x00') + struct.pack(endian + 'iII', sec, ns, 2) + b'f\0'


def bag(path, topics, messages):
    with sqlite3.connect(path) as db:
        db.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT);'
                         'CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB);')
        db.executemany('INSERT INTO topics VALUES (?,?,?,?)', topics)
        db.executemany('INSERT INTO messages VALUES (?,?,?,?)', messages)


class ReviewTests(unittest.TestCase):
    def cli(self, *args):
        return subprocess.run([sys.executable, str(ROOT / 'tools/stamp_drift/analyze.py'), *map(str, args)],
                              capture_output=True, text=True, cwd=ROOT)

    def test_residuals_preserve_nanoseconds_at_receive_utc_epoch(self):
        rows = [analyzer.Row('/imu', 1700000000000000000 + i*1000000,
                            1700000001000000000 + i*1000000 + (0, 1, -1)[i % 3], i)
                for i in range(30)]
        residual = analyzer.analyze_rows(rows)['streams'][0]['segments'][0]['residual_abs_ns']
        self.assertGreater(residual['p50'], 0.5)
        self.assertLess(residual['max'], 2)

    def test_fail_precedes_blocked_on_short_reversal(self):
        report = analyzer.analyze_rows([analyzer.Row('/imu', 20, 100, 1), analyzer.Row('/imu', 10, 110, 2)])
        self.assertEqual(report['diagnostic_status'], 'FAIL')

    def test_constant_receive_clock_is_blocked_without_exception(self):
        report = analyzer.analyze_rows([analyzer.Row('/imu', i, 100, i) for i in range(20)])
        self.assertEqual(report['diagnostic_status'], 'BLOCKED')

    def test_empty_rows_blocked(self):
        self.assertEqual(analyzer.analyze_rows([])['diagnostic_status'], 'BLOCKED')

    def test_header_clock_does_not_imply_option_readback(self):
        rows = [analyzer.Row('/imu', i, i+10, i) for i in range(5)]
        for domain in ('SYSTEM_TIME', 'D435I_HARDWARE_TIME', 'D435I_GLOBAL_TIME'):
            self.assertIsNone(analyzer.analyze_rows(rows, header_clock=domain)['clock_domains']['global_time_enabled'])
        self.assertTrue(analyzer.analyze_rows(rows, global_time_enabled=True)['clock_domains']['global_time_enabled'])

    def test_mixed_bag_skips_unstamped_and_tf_and_reports_empty_sensor(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/'bag.db3'
            bag(path, [(1,'/imu','sensor_msgs/msg/Imu','cdr'), (2,'/tf','tf2_msgs/msg/TFMessage','cdr'),
                       (3,'/events','std_msgs/msg/String','cdr'), (4,'/infra','sensor_msgs/msg/Image','cdr')],
                [(i+1,1,100000+i*1000,header(i*1000)) for i in range(5)] +
                [(6,2,100000,b'\x00\x01\x00\x00'+b'\0'*20), (7,3,100000,b'\x00\x01\x00\x00'+b'\0'*20)])
            report = analyzer.analyze_path(path)
        self.assertEqual(len(report['input']['skipped_topics']), 2)
        self.assertEqual(report['diagnostic_status'], 'BLOCKED')
        self.assertEqual({s['topic'] for s in report['streams']}, {'/imu','/infra'})
        self.assertEqual(next(s for s in report['streams'] if s['topic']=='/infra')['sample_count'],0)

    def test_header_big_endian_and_malformed_headers(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'bag.db3'
            bag(path,[(1,'/imu','sensor_msgs/msg/Imu','cdr')],
                [(i+1,1,100000+i*1000,header(i*1000,'>')) for i in range(5)])
            self.assertEqual(analyzer.analyze_path(path)['diagnostic_status'], 'PASS')
        for payload in (header(100)[:12], header(100)[:-1], b'\0\x01\0\0'+struct.pack('<iII',0,10**9,1)+b'\0'):
            with self.subTest(payload=payload), tempfile.TemporaryDirectory() as td:
                path=Path(td)/'bag.db3'
                bag(path,[(1,'/imu','sensor_msgs/msg/Imu','cdr')],[(1,1,100,payload)])
                with self.assertRaises(ValueError):
                    analyzer.analyze_path(path)

    def test_non_cdr_serialization_refused(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'bag.db3'
            bag(path,[(1,'/imu','sensor_msgs/msg/Imu','other')],[(1,1,100,header(100))])
            with self.assertRaisesRegex(ValueError,'serialization'):
                analyzer.analyze_path(path)

    def test_db3_not_read_whole_into_memory(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'bag.db3'
            bag(path,[(1,'/imu','sensor_msgs/msg/Imu','cdr')],[(i+1,1,i*1000+10,header(i*1000)) for i in range(5)])
            original=Path.read_bytes
            def bounded_read(p):
                if p.suffix=='.db3':
                    raise AssertionError('must stream hash and SQLite payloads')
                return original(p)
            with patch.object(Path,'read_bytes',bounded_read):
                self.assertEqual(analyzer.analyze_path(path)['diagnostic_status'],'PASS')

    def test_real_converted_ros_headers_fixture(self):
        fixture=json.loads((FIXTURES/'real_ros_headers.json').read_text())
        self.assertEqual(fixture['evidence_label'],'MEASURED')
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'real-excerpt.db3'
            topics=[]
            messages=[]
            for ident,topic in enumerate(fixture['topics'],1):
                topics.append((ident,topic['name'],topic['type'],'cdr'))
                for row in topic['rows']:
                    messages.append((len(messages)+1,ident,row['bag_receive_stamp_ns'],bytes.fromhex(row['cdr_header_hex'])))
            bag(path,topics,messages)
            report=analyzer.analyze_path(path)
        self.assertEqual(report['input']['row_count'],12)
        self.assertEqual(report['diagnostic_status'],'PASS')
        for stream in report['streams']:
            self.assertAlmostEqual(stream['drift_ppm'],0,places=5)

    def test_adapted_sdk_fixture_preserves_exact_decimal_ns(self):
        originals=[json.loads(line,parse_float=Decimal) for line in
                   (FIXTURES/'real_sdk_source.jsonl').read_text().splitlines()]
        with (FIXTURES/'real_saved_evidence.csv').open() as stream:
            saved=list(csv.DictReader(stream))
        self.assertEqual(len(saved),len(originals))
        for row,original in zip(saved,originals):
            self.assertEqual(int(row['header_stamp_ns']),int(original['sdk_timestamp_ms']*1000000))
            self.assertEqual(int(row['receive_ns']),original['host_steady_ns'])

    def test_missing_numpy_is_blocked_with_result(self):
        with tempfile.TemporaryDirectory() as td:
            out=Path(td)/'out'
            with patch.object(analyzer,'np',None),contextlib.redirect_stdout(io.StringIO()):
                code=analyzer.main([str(Path(td)/'absent.csv'),'--output-dir',str(out)])
            self.assertEqual(code,2)
            self.assertEqual(json.loads((out/'result.json').read_text())['status'],'BLOCKED')
            self.assertIn('NumPy',(out/'diagnostic.txt').read_text())

    def test_config_rejects_nonfinite_short_fit_and_missing_percentiles(self):
        baseline=json.loads(analyzer.CONFIG_PATH.read_text())
        invalid=[]
        for value in (float('nan'),float('inf')):
            config=copy.deepcopy(baseline)
            config['thresholds']['outlier_mad_scale']['value']=value
            invalid.append(config)
        config=copy.deepcopy(baseline)
        config['thresholds']['minimum_fit_samples']['value']=1
        invalid.append(config)
        config=copy.deepcopy(baseline)
        config['percentiles']=[50]
        invalid.extend([config,[]])
        for config in invalid:
            with self.subTest(config=config), tempfile.TemporaryDirectory() as td:
                path=Path(td)/'config.json'
                path.write_text(json.dumps(config))
                with self.assertRaises(ValueError):
                    analyzer._load_config(path)

    def test_cli_contract_all_statuses_roundtrip_hashes_and_missing_input(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            for case,stamps,expected,code in [('pass',range(20),'PASS',0), ('fail',[20,10],'FAIL',1),
                                              ('short',[1,2],'BLOCKED',2), ('empty',[],'BLOCKED',2)]:
                with self.subTest(case=case):
                    source=root/(case+'.csv')
                    source.write_text('topic,header_stamp_ns,receive_ns\n'+''.join(f'/imu,{stamp},{100+i*1000}\n' for i,stamp in enumerate(stamps)))
                    out=root/case
                    run=self.cli(source,'--output-dir',out,'--header-clock','SYSTEM_TIME','--global-time-enabled','true')
                    self.assertEqual(run.returncode,code,run.stderr+run.stdout)
                    result=json.loads((out/'result.json').read_text())
                    self.assertEqual(result['status'],expected)
                    for key in ('check','tool_version','started_utc','ended_utc','host','criteria','raw_files','notes'):
                        self.assertIn(key,result)
                    for item in result['criteria']:
                        self.assertTrue((out/item['evidence']).is_file())
                    for item in result['raw_files']:
                        self.assertEqual(hashlib.sha256((out/item['path']).read_bytes()).hexdigest(),item['sha256'])
                    again=self.cli('--analyze-only',out)
                    self.assertEqual(again.returncode,code,again.stderr)
                    if case=='pass':
                        report=json.loads((out/'report.json').read_text())
                        self.assertEqual(report['clock_domains']['header_stamp'],'SYSTEM_TIME')
                        self.assertTrue(report['clock_domains']['global_time_enabled'])
                        source.unlink()
                        self.assertEqual(self.cli('--analyze-only',out).returncode,0)
                        (out/'input.csv').unlink()
                        self.assertEqual(self.cli('--analyze-only',out).returncode,2)
                        self.assertFalse((out/'report.json').exists())
            out=root/'missing'
            self.assertEqual(self.cli(root/'absent.csv','--output-dir',out).returncode,2)
            self.assertEqual(json.loads((out/'result.json').read_text())['status'],'BLOCKED')

    def test_analyze_only_missing_or_corrupt_metadata_is_blocked(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            source=root/'source.csv'
            source.write_text('topic,header_stamp_ns,receive_ns\n'+''.join(f'/imu,{i},{i+10}\n' for i in range(5)))
            out=root/'out'
            self.assertEqual(self.cli(source,'--output-dir',out).returncode,0)
            saved_config=(out/'config.json').read_text()
            for filename,content in [('run.json','[]'),('run.json','{"header_clock":"typo"}'),('config.json',None),('config.json','{}')]:
                (out/'config.json').write_text(saved_config)
                (out/'run.json').write_text('{}')
                path=out/filename
                path.unlink() if content is None else path.write_text(content)
                self.assertEqual(self.cli('--analyze-only',out).returncode,2)
                self.assertEqual(json.loads((out/'result.json').read_text())['status'],'BLOCKED')

    def test_cli_usage_errors_exit_three_and_do_not_overwrite_input(self):
        self.assertEqual(self.cli().returncode,3)
        self.assertEqual(self.cli('--bogus').returncode,3)
        with tempfile.TemporaryDirectory() as td:
            source=Path(td)/'input.csv'
            raw='topic,header_stamp_ns,receive_ns\n/imu,1,2\n'
            source.write_text(raw)
            self.assertEqual(self.cli(source,'--json-out',source).returncode,3)
            self.assertEqual(source.read_text(),raw)
            config=Path(td)/'bad.json'
            config.write_text('{}')
            self.assertEqual(self.cli(source,'--config',config).returncode,3)

    def test_step_segment_cap_is_enforced(self):
        config=json.loads(analyzer.CONFIG_PATH.read_text())
        config['thresholds']['maximum_step_segments']['value']=3
        rows=[analyzer.Row('/imu',i*100000000,10**9+i*100000000+(i//20)*40000000,i) for i in range(160)]
        stream=analyzer.analyze_rows(rows,config=config)['streams'][0]
        self.assertLessEqual(stream['segment_count'],3)
        self.assertEqual(stream['diagnostic_status'],'FAIL')
        config['thresholds']['maximum_step_segments']['value']=1
        stream=analyzer.analyze_rows(rows,config=config)['streams'][0]
        self.assertEqual(stream['segment_count'],1)
        self.assertEqual(stream['diagnostic_status'],'FAIL')
        self.assertTrue(stream['step_discontinuities'][0]['segment_cap_reached'])


if __name__=='__main__':
    unittest.main()
