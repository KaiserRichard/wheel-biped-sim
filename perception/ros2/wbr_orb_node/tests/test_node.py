"""unittest drives the actual ROS-free C++ core; no ROS, camera or serial API."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(ROOT))
PROBE = PACKAGE / 'build/core_test'


def load(name):
    spec = importlib.util.spec_from_file_location(name, PACKAGE / 'scripts' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CoreTests(unittest.TestCase):
    def scenario(self, name, *arguments):
        if not PROBE.is_file():
            self.fail('build/core_test missing; use make test to compile the core first')
        run = subprocess.run([str(PROBE), name, *map(str, arguments)], capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        return run

    def test_exact_pairing(self):
        self.scenario('pairing')

    def test_imu_coverage(self):
        self.scenario('coverage')

    def test_stereo_image_rejection_not_imu_fault(self):
        self.scenario('image_rejection')

    def test_drop_oldest_and_fault_after_imu_loss(self):
        self.scenario('drops')

    def test_original_stamp_and_camera_imu_composition(self):
        self.scenario('frames')

    def test_clock_faults_and_replay_seek(self):
        self.scenario('faults')

    def test_loss_reset_and_reinitialization(self):
        self.scenario('states')

    def test_executor_ingress_progress_during_slow_tracking(self):
        self.scenario('concurrent')

    def test_engine_exception_keeps_source_image_identity(self):
        self.scenario('engine_fault')

    def test_sender_tracking_enum_compatibility(self):
        run = self.scenario('schema')
        schema = json.loads((PACKAGE / 'tests/fixtures/sender_schema.json').read_text())
        records = [json.loads(line) for line in run.stdout.splitlines() if line.startswith('{')]
        self.assertEqual(len(records), 5)
        for record in records:
            self.assertIn(record['tracking_state'], schema['tracking_states'])
        self.assertEqual([r['tracking_state'] for r in records],
                         ['SYSTEM_NOT_READY', 'NOT_INITIALIZED', 'OK', 'LOST', 'SYSTEM_NOT_READY'])

    def test_real_saved_source_epochs(self):
        self.scenario('real', PACKAGE / 'tests/fixtures/real_epochs.txt')
        provenance = json.loads((PACKAGE / 'tests/fixtures/provenance.json').read_text())
        self.assertTrue(provenance['source_sha256'])

    def test_health_schema_and_runtime_recorder_compatibility(self):
        from tools.runtime_observer.aggregate import read_csv, summarize
        with tempfile.TemporaryDirectory(dir=PACKAGE / 'build') as directory:
            path = Path(directory)
            self.scenario('emit', path)
            event = json.loads((path / 'pose_events.jsonl').read_text())
            self.assertEqual(event['pose_source_d435i_ns'], 15)
            self.assertEqual(event['tracking_state'], 'OK')
            self.assertEqual(event['adapter_tracking_state'], 'TRACKING')
            self.assertFalse(event['mapping']['valid'])
            self.assertIs(event['imu_initialized'], False)
            self.assertEqual(event['tracked_features'], 0)
            self.assertIs(event['degraded'], True)
            self.assertEqual(event['map_reset_counter'], 0)
            events, losses = read_csv((path / 'events.csv').read_text())
            # Existing aggregation validates lifecycle and source identity, rather
            # than a second implementation of observer interpretation.
            report = summarize(events, losses=losses)
            self.assertTrue(report)
            kinds = {e.kind for e in events}
            self.assertTrue({'frame_received', 'queued', 'track_start', 'track_end',
                             'snapshot_published', 'queue_depth'} <= kinds)
            self.assertTrue(all(e.host_ns >= 0 for e in events))
            self.assertTrue(all(value == 0 for value in losses.values()))


class ResultContractTests(unittest.TestCase):
    def command(self, *args):
        return subprocess.run([sys.executable, str(PACKAGE / 'scripts/check_core.py'), *map(str, args)],
                              capture_output=True, text=True)

    def test_pass_and_analyze_saved_raw_files(self):
        with tempfile.TemporaryDirectory(dir=PACKAGE / 'build') as directory:
            outdir = Path(directory) / 'pass'
            run = self.command('--outdir', outdir)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            before = json.loads((outdir / 'result.json').read_text())
            run = self.command('--analyze-only', outdir)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            after = json.loads((outdir / 'result.json').read_text())
            self.assertEqual(after['status'], 'PASS')
            self.assertEqual(before['raw_files'], after['raw_files'])
            self.assertTrue(after['criteria'])

    def test_fail_from_saved_nonzero_run(self):
        with tempfile.TemporaryDirectory(dir=PACKAGE / 'build') as directory:
            outdir = Path(directory) / 'fail'
            self.assertEqual(self.command('--outdir', outdir).returncode, 0)
            metadata = json.loads((outdir / 'execution.json').read_text())
            # Capture a real assertion failure from the same executable.
            failed = subprocess.run([str(PROBE), 'fail'], capture_output=True)
            self.assertEqual(failed.returncode, 1)
            metadata['runs'][0]['returncode'] = failed.returncode
            (outdir / 'all.stdout').write_bytes(failed.stdout)
            (outdir / 'all.stderr').write_bytes(failed.stderr)
            (outdir / 'execution.json').write_text(json.dumps(metadata))
            self.assertEqual(self.command('--analyze-only', outdir).returncode, 1)
            self.assertEqual(json.loads((outdir / 'result.json').read_text())['status'], 'FAIL')

    def test_blocked_missing_executable(self):
        with tempfile.TemporaryDirectory(dir=PACKAGE / 'build') as directory:
            outdir = Path(directory) / 'blocked'
            run = self.command('--outdir', outdir, '--probe', Path(directory) / 'absent')
            self.assertEqual(run.returncode, 2, run.stdout + run.stderr)
            self.assertEqual(json.loads((outdir / 'result.json').read_text())['status'], 'BLOCKED')
            self.assertEqual(self.command('--analyze-only', outdir).returncode, 2)

    def test_malformed_raw_is_internal_error(self):
        with tempfile.TemporaryDirectory(dir=PACKAGE / 'build') as directory:
            (Path(directory) / 'execution.json').write_text('{}')
            self.assertEqual(self.command('--analyze-only', directory).returncode, 3)


class MappingTests(unittest.TestCase):
    def test_absent_mapping_does_not_need_numpy(self):
        # The node's default health trace always explicitly marks mapping invalid.
        with tempfile.TemporaryDirectory(dir=PACKAGE / 'build') as directory:
            CoreTests().scenario('emit', directory)
            self.assertFalse(json.loads((Path(directory) / 'pose_events.jsonl').read_text())['mapping']['valid'])

    def test_prepared_mapping_uses_existing_clockmap(self):
        try:
            from tools.clockmap import ClockMap
        except ImportError:
            self.skipTest('NumPy unavailable; configured mapping preparation is BLOCKED')
        helper = load('prepare_mapping')
        data = {'valid': True, 'camera_domain': 'global_time', 'camera_session': 'camera-a',
                'pi_domain': 'CLOCK_MONOTONIC_RAW', 'pi_session': 'pi-a',
                'exchanges_ns': [[0, 1100, 1100, 200], [1000000, 1001100, 1001100, 1000200],
                                 [2000000, 2001100, 2001100, 2000200]]}
        prepared = helper.prepare(data, 'global_time', 'camera-a', 'pi-a')
        self.assertTrue(prepared['valid'])
        self.assertEqual(prepared['exchanges_ns'], data['exchanges_ns'])
        self.assertEqual(prepared['valid_source_min_ns'], 100)
        self.assertEqual(prepared['valid_source_max_ns'], 2000100)
        extended = dict(data, valid_source_max_ns=3000100)
        with self.assertRaises(ValueError):
            helper.prepare(extended, 'global_time', 'camera-a', 'pi-a')
        extended['validity_rationale'] = 'PROPOSED synthetic finite extrapolation test only'
        self.assertEqual(helper.prepare(extended, 'global_time', 'camera-a', 'pi-a')
                         ['valid_source_max_ns'], 3000100)
        for key, value in [('pi_domain', 'CLOCK_MONOTONIC'), ('camera_session', 'other')]:
            bad = dict(data, **{key: value})
            with self.assertRaises(ValueError):
                helper.prepare(bad, 'global_time', 'camera-a', 'pi-a')

    def test_mapping_cli_refuses_missing_input(self):
        with tempfile.TemporaryDirectory(dir=PACKAGE / 'build') as directory:
            run = subprocess.run([sys.executable, str(PACKAGE / 'scripts/prepare_mapping.py'),
                                  str(Path(directory) / 'absent'), str(Path(directory) / 'output.json'),
                                  '--source-domain', 'global_time', '--source-session', 'camera-a',
                                  '--pi-session', 'pi-a'], capture_output=True)
            self.assertEqual(run.returncode, 2)
