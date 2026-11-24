"""Offline acceptance tests; synthetic inputs do not qualify hardware."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import importlib.util
import yaml

ROOT = Path(__file__).resolve().parents[3]
LAB = ROOT / 'calibration/lab'
HERE = Path(__file__).resolve().parents[1]
FIX = LAB / 'tests/fixtures'
SCRIPT = HERE / 'check_kalibr_result.py'
IMU = LAB / 'config/imu_d435i_provisional.yaml'
FACTORY = LAB / 'config/camchain_factory_serial-alias-6.yaml'
spec = importlib.util.spec_from_file_location('kalibr_gate', SCRIPT)
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


class Acceptance(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='kalibr-gate-', dir=HERE / 'tests')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def inputs(self, stem='base'):
        target = self.root / stem
        target.mkdir()
        paths = {
            'camimu-chain': FIX / 'synthetic-camchain-imucam.yaml',
            'camimu-results': HERE / 'tests/fixtures/synthetic-results-imucam.txt',
            'stereo-chain': FIX / 'synthetic-camchain-imucam.yaml',
            'stereo-results': FIX / 'synthetic-results-cam.txt',
            'run-imu': IMU,
            'provisional-imu': IMU,
            'factory': FACTORY,
        }
        for key, path in paths.items():
            shutil.copyfile(path, target / (key + path.suffix))
        return target

    def run_gate(self, args, expected):
        result = subprocess.run([sys.executable, str(SCRIPT), '--outdir', str(self.root / 'out'), *args],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        data = json.loads((self.root / 'out/result.json').read_text())
        return data

    def cli_args(self, source):
        args = []
        for key in ('camimu-chain', 'camimu-results', 'stereo-chain', 'stereo-results',
                    'run-imu', 'provisional-imu', 'factory'):
            matches = list(source.glob(key + '.*'))
            if matches:
                args.extend(['--' + key, str(matches[0])])
        return args

    def test_synthetic_fixtures_pass_and_saved_inputs_reanalyze(self):
        source = self.inputs()
        data = self.run_gate(self.cli_args(source), 0)
        self.assertEqual(data['status'], 'PASS')
        self.assertTrue(data['raw_files'])
        self.assertIn('49.912553', (self.root / 'out/diff.txt').read_text())
        result = subprocess.run([sys.executable, str(SCRIPT), '--outdir', str(self.root / 'again'),
                                 '--analyze-only', str(self.root / 'out')], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        again = json.loads((self.root / 'again/result.json').read_text())
        self.assertEqual(again['status'], 'PASS')

    def test_perturbations_fail_each_major_criterion(self):
        cases = [
            ('cam0_mean', 'camimu-results.txt', 'mean 0.3, median 0.28', 'mean 2.3, median 0.28', 'cam0_camimu_reprojection_mean'),
            ('residual', 'camimu-results.txt', 'std: 0.4', 'std: 1.4', 'cam0_camimu_reprojection_std'),
            ('cam1_residual', 'camimu-results.txt', 'std: 8.0e-1', 'std: 1.8', 'cam1_camimu_reprojection_std'),
            ('gyro_mean', 'camimu-results.txt', 'Gyroscope error (imu0) [rad/s]:     mean 0.01', 'Gyroscope error (imu0) [rad/s]:     mean 0.05', 'gyro_residual_mean'),
            ('gyro_std', 'camimu-results.txt', 'Gyroscope error (imu0) [rad/s]:     mean 0.01, median 0.01, std: 0.01', 'Gyroscope error (imu0) [rad/s]:     mean 0.01, median 0.01, std: 0.05', 'gyro_residual_std'),
            ('accel_mean', 'camimu-results.txt', 'Accelerometer error (imu0) [m/s^2]: mean 0.02', 'Accelerometer error (imu0) [m/s^2]: mean 0.20', 'accel_residual_mean'),
            ('accel_std', 'camimu-results.txt', 'mean 0.02, median 0.02, std: 0.03', 'mean 0.02, median 0.02, std: 0.30', 'accel_residual_std'),
            ('stereo_reprojection', 'stereo-results.txt', 'reprojection error: [0.300000, -0.400000]', 'reprojection error: [3.300000, -0.400000]', 'cam0_stereo_reprojection_mean_x'),
            ('time', 'camimu-chain.yaml', '0.012345678', '0.040000000', 'cam0_timeshift'),
            ('cam1_time', 'camimu-chain.yaml', 'timeshift_cam_imu: 0.012345678', 'timeshift_cam_imu: 0.040000000', 'cam1_timeshift'),
            ('extrinsic', 'camimu-chain.yaml', '-0.01174', '-0.02174', 'imu_translation_delta'),
            ('rotation', 'camimu-chain.yaml', '', '', 'imu_rotation_delta'),
            ('baseline', 'stereo-chain.yaml', '-0.049912553280592', '-0.052912553280592', 'stereo_baseline_delta'),
            ('intrinsics', 'stereo-chain.yaml', '386.953308105469', '390.953308105469', 'cam0_fx_drift'),
            ('distortion_model', 'stereo-chain.yaml', 'distortion_model: radtan', 'distortion_model: equidistant', 'cam0_distortion_model'),
            ('distortion_delta', 'stereo-chain.yaml', 'distortion_coeffs: [0.0, 0.0, 0.0, 0.0]', 'distortion_coeffs: [0.04, 0.0, 0.0, 0.0]', 'cam0_k1_distortion_delta'),
            ('distortion', 'stereo-chain.yaml', 'distortion_coeffs: [0.0, 0.0, 0.0, 0.0]', 'distortion_coeffs: [0.2, 0.0, 0.0, 0.0]', 'cam0_k1_distortion_sanity'),
        ]
        for name, filename, old, new, criterion in cases:
            with self.subTest(name=name):
                source = self.inputs(name)
                file = next(source.glob(filename))
                content = file.read_text()
                if name == 'rotation':
                    chain = yaml.safe_load(content)
                    rz = [[0.998629534754574, -0.052335956242944, 0.0],
                          [0.052335956242944, 0.998629534754574, 0.0], [0.0, 0.0, 1.0]]
                    for camera in ('cam0', 'cam1'):
                        matrix = chain[camera]['T_cam_imu']
                        for row in range(3):
                            matrix[row][:3] = rz[row]
                    file.write_text(yaml.safe_dump(chain, sort_keys=False))
                elif name == 'cam1_time':
                    chain = yaml.safe_load(content)
                    chain['cam1']['timeshift_cam_imu'] = 0.04
                    file.write_text(yaml.safe_dump(chain, sort_keys=False))
                else:
                    file.write_text(content.replace(old, new, 2 if name == 'extrinsic' else 1))
                out = self.root / ('out-' + name)
                result = subprocess.run([sys.executable, str(SCRIPT), '--outdir', str(out), *self.cli_args(source)],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                data = json.loads((out / 'result.json').read_text())
                self.assertEqual(data['status'], 'FAIL')
                self.assertIn('FAIL', [c['status'] for c in data['criteria'] if c['id'] == criterion])

    def test_noise_mismatch_fails_provenance(self):
        source = self.inputs('noise')
        runimu = next(source.glob('run-imu.yaml'))
        runimu.write_text(runimu.read_text().replace('1.0e-3', '2.0e-3'))
        out = self.root / 'noise-out'
        result = subprocess.run([sys.executable, str(SCRIPT), '--outdir', str(out), *self.cli_args(source)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        data = json.loads((out / 'result.json').read_text())
        row = next(c for c in data['criteria'] if c['id'] == 'imu_noise_provenance')
        self.assertEqual(row['status'], 'FAIL')
        self.assertIn('gyroscope_noise_density', row['measured']['mismatched_fields'])

    def test_missing_required_input_is_blocked(self):
        source = self.inputs('missing')
        path = next(source.glob('stereo-results.txt'))
        path.unlink()
        out = self.root / 'blocked-out'
        result = subprocess.run([sys.executable, str(SCRIPT), '--outdir', str(out), *self.cli_args(source)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        data = json.loads((out / 'result.json').read_text())
        self.assertEqual(data['status'], 'BLOCKED')
        self.assertTrue(any(c['status'] == 'BLOCKED' for c in data['criteria']))

    def test_analyze_only_missing_saved_policy_is_blocked(self):
        source = self.inputs('policy')
        original = self.run_gate(self.cli_args(source), 0)
        self.assertEqual(original['status'], 'PASS')
        (self.root / 'out/input-thresholds.yaml').unlink()

        result = subprocess.run([sys.executable, str(SCRIPT), '--outdir', str(self.root / 'reanalysis'),
                                 '--analyze-only', str(self.root / 'out')], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        data = json.loads((self.root / 'reanalysis/result.json').read_text())
        self.assertEqual(data['status'], 'BLOCKED')
        self.assertTrue(any('threshold policy unavailable' in c['description'] for c in data['criteria']))

    def test_unreadable_threshold_policy_writes_blocked_result(self):
        source = self.inputs('bad-policy')
        bad_policy = self.root / 'thresholds.yaml'
        bad_policy.write_text('thresholds: [not, a, mapping]\n')
        out = self.root / 'bad-policy-out'
        result = subprocess.run([sys.executable, str(SCRIPT), '--outdir', str(out), '--thresholds', str(bad_policy),
                                 *self.cli_args(source)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        data = json.loads((out / 'result.json').read_text())
        self.assertEqual(data['status'], 'BLOCKED')
        self.assertTrue(any('threshold policy could not be evaluated' in c['description'].lower()
                            for c in data['criteria']))

    def test_usage_error_uses_contract_exit_code(self):
        result = subprocess.run([sys.executable, str(SCRIPT), '--outdir', str(self.root / 'usage-out'), '--unknown'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 3)

    def test_real_saved_tumvi_report_is_a_parser_fixture_only(self):
        # Real Kalibr smoke output, unrelated TUM-VI fisheye rig; never an acceptance PASS.
        fixture = HERE / 'tests/fixtures/real-tumvi-smoke'
        real = fixture / 'imu4-results-imucam.txt'
        stats = gate.SUMMARY.parse_reprojection(real.read_text(), 'camimu')
        self.assertAlmostEqual(stats['cam0']['mean'], 8.594411060709534)
        self.assertAlmostEqual(stats['cam1']['std'], 0.7358460195549441)
        chain = gate.SUMMARY.load_chain(fixture / 'imu4-camchain-imucam.yaml')
        self.assertEqual(chain['cam0']['distortion_model'], 'equidistant')
        gate.SUMMARY.transform(chain['cam0']['T_cam_imu'])


if __name__ == '__main__':
    unittest.main()
