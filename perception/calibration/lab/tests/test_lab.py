"""SYNTHETIC offline tests only. No Docker, network, Pi or camera commands."""
import copy
import csv
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import yaml

LAB = Path(__file__).resolve().parents[1]
FIXTURES = LAB / 'tests/fixtures'


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


s = module('summary', LAB / 'mac/summarize_kalibr.py')
f = module('files', LAB / 'mac/lab_files.py')
d = module('diagnostics', LAB / 'pi/take_diagnostics.py')


class Summaries(unittest.TestCase):
    def setUp(self):
        self.chain = s.load_chain(FIXTURES / 'synthetic-camchain-imucam.yaml')
        self.text = (FIXTURES / 'synthetic-results-imucam.txt').read_text()

    def test_nonidentity_inverse_origins_basis_and_composition(self):
        # Rz(90deg) Rx(0.2rad): a full inverse must rotate translation too.
        c, z = np.cos(0.2), np.sin(0.2)
        rotation = np.array([[0, -c, z], [1, 0, 0], [0, z, c]])
        matrix = np.eye(4)
        matrix[:3, :3], matrix[:3, 3] = rotation, [0.012, -0.021, 0.034]
        inverse = s.inverse_transform(matrix)
        np.testing.assert_allclose(inverse, np.linalg.inv(matrix), atol=1e-14)
        np.testing.assert_allclose(inverse @ matrix, np.eye(4), atol=1e-14)
        np.testing.assert_allclose(matrix @ inverse, np.eye(4), atol=1e-14)
        camera_points = np.column_stack(([0, 0, 0, 1], [1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]))
        expected = np.vstack((rotation.T @ (camera_points[:3] - matrix[:3, 3, None]), np.ones(4)))
        np.testing.assert_allclose(inverse @ camera_points, expected, atol=1e-14)
        self.assertGreater(np.linalg.det(inverse[:3, :3]), 0)
        self.assertFalse(np.allclose(inverse[:3, 3], -matrix[:3, 3]))

    def test_factory_inverse_translation_sign(self):
        inverse = s.inverse_transform(self.chain['cam0']['T_cam_imu'])
        np.testing.assert_allclose(inverse[:3, 3], [-0.00552, 0.00510, 0.01174])
        factory = s.load_chain(LAB / 'config/camchain_factory_serial-alias-6.yaml')
        right_to_left = s.inverse_transform(factory['cam1']['T_cn_cnm1'])
        self.assertAlmostEqual(right_to_left[0, 3], 0.049912553280592)

    def test_time_sign_and_integer_precision(self):
        camera = 1_700_000_000_000_000_123
        self.assertEqual(s.camera_to_imu_ns(camera, '0.012345678'), camera + 12_345_678)
        self.assertEqual(s.camera_to_imu_ns(camera, '-0.012345678'), camera - 12_345_678)
        self.assertEqual(s.camera_to_imu_ns(camera, 0), camera)
        self.assertEqual(s.camera_to_imu_ns(camera, '0.0000000005'), camera)
        with self.assertRaises(ValueError):
            s.camera_to_imu_ns(camera, 'NaN')

    def test_raw_pixel_statistics_ignore_normalized(self):
        stats = s.parse_reprojection(self.text, 'camimu')
        self.assertAlmostEqual(stats['cam0']['mean'], 0.3)
        self.assertAlmostEqual(stats['cam0']['std'], 0.4)
        self.assertAlmostEqual(stats['cam0']['rms'], 0.5)
        self.assertAlmostEqual(stats['cam1']['rms'], 1.0)
        summary = s.camimu_summary(self.chain, self.text)
        self.assertIn('norm=0.000000000 m (0.000000 mm)', summary)
        self.assertIn('12.345678000 ms', summary)
        self.assertIn('within ASSUMED threshold (<=0.5 px)', summary)
        self.assertIn('outside ASSUMED threshold (<=0.5 px)', summary)
        self.assertNotIn('PASS', summary)

    def test_missing_no_corners_and_malformed_are_unknown(self):
        text = 'Reprojection error (cam0): mean 0.1, median 0.1, std: 0.1\nReprojection error (cam1) [px]: no corners\n'
        self.assertEqual(s.parse_reprojection(text, 'camimu'), {})
        self.assertIn('UNKNOWN', '\n'.join(s.residual_lines(text, 'camimu')))
        self.assertEqual(s.parse_reprojection('Reprojection error (cam0) [px]: mean nan, median 0, std: 0', 'camimu'), {})

    def test_duplicate_or_negative_pixel_statistics_rejected(self):
        line = 'Reprojection error (cam0) [px]: mean 0.3, median 0.28, std: 0.4\n'
        with self.assertRaises(ValueError):
            s.parse_reprojection(line + line, 'camimu')
        with self.assertRaises(ValueError):
            s.parse_reprojection(line.replace('std: 0.4', 'std: -0.4'), 'camimu')

    def test_stereo_components_and_baseline(self):
        factory = s.load_chain(LAB / 'config/camchain_factory_serial-alias-6.yaml')
        fitted = copy.deepcopy(factory)
        fitted['cam0']['intrinsics'][0] += 1
        fitted['cam0']['intrinsics'][1] -= 1
        fitted['cam0']['distortion_coeffs'] = [0.01, 0.02, -0.001, 0.002]
        fitted['cam1']['T_cn_cnm1'][0][3] -= 0.0005
        text = (FIXTURES / 'synthetic-results-cam.txt').read_text()
        stats = s.parse_reprojection(text, 'stereo')
        self.assertAlmostEqual(stats['cam0']['rms'], 0.5)
        self.assertAlmostEqual(stats['cam1']['rms'], 0.5)
        result = s.stereo_summary(fitted, factory, text)
        self.assertIn('delta=+1.000000000 px', result)
        self.assertIn('absolute delta=0.500000 mm', result)
        self.assertIn('never replace MEASURED factory', result)
        self.assertNotIn('PASS', result)

    def test_invalid_transforms_and_chain(self):
        for matrix in (np.zeros((4, 4)), np.diag([-1, 1, 1, 1]), np.eye(3), np.full((4, 4), np.nan)):
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                s.inverse_transform(matrix)
        self.chain['cam1']['T_cam_imu'][0][3] += 0.01
        with self.assertRaises(ValueError):
            s.camimu_summary(self.chain, self.text)

    def test_cli_synthetic_fixture(self):
        result = subprocess.run([sys.executable, str(LAB / 'mac/summarize_kalibr.py'), 'camimu',
                                 '--chain', str(FIXTURES / 'synthetic-camchain-imucam.yaml'),
                                 '--results', str(FIXTURES / 'synthetic-results-imucam.txt')], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('ORB IMU.T_b_c1', result.stdout)

    def test_duplicate_yaml_rejected(self):
        with self.assertRaises(ValueError):
            yaml.load('cam0: {}\ncam0: {}\n', Loader=s.UniqueLoader)


class Layout(unittest.TestCase):
    def setUp(self):
        # Every file written by these tests stays under calibration/lab/.
        self.temp = tempfile.TemporaryDirectory(prefix='synthetic-', dir=LAB / 'tests')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.euroc = self.root / 'euroc'
        self.output = self.root / 'bagcreater'
        self.imu = self.euroc / 'mav0/imu0/data.csv'
        self.imu.parent.mkdir(parents=True)
        self.rows = [['1700000000000000001', '0.123456789012345', '-2', '3', '4', '5', '9.81'],
                     ['1700000000005000001', '1', '2', '3', '4', '5', '6']]
        self.write_imu(f.EUROC_HEADER, self.rows)
        for camera in ('cam0', 'cam1'):
            base = self.euroc / 'mav0' / camera
            (base / 'data').mkdir(parents=True)
            stamp = '1700000000000000001'
            # Synthetic payload; tests cover links and hashes, never image decoding.
            (base / 'data' / (stamp + '.png')).write_bytes(b'SYNTHETIC PNG payload, not an actual image')
            with (base / 'data.csv').open('w', newline='') as stream:
                csv.writer(stream).writerows([['#timestamp [ns]', 'filename'], [stamp, stamp + '.png']])

    def write_imu(self, header, rows):
        with self.imu.open('w', newline='') as stream:
            csv.writer(stream).writerows([header, *rows])

    def test_layout_units_order_precision_links(self):
        before = f.sha256(self.imu)
        f.prepare_bagcreater(self.euroc, self.output)
        with (self.output / 'imu0.csv').open(newline='') as stream:
            rows = list(csv.reader(stream))
        self.assertEqual(rows[1:], self.rows)
        self.assertEqual(rows[0], ['timestamp [ns]', 'omega_x [rad/s]', 'omega_y [rad/s]', 'omega_z [rad/s]',
                                   'alpha_x [m/s^2]', 'alpha_y [m/s^2]', 'alpha_z [m/s^2]'])
        self.assertEqual(before, f.sha256(self.imu))
        for camera in ('cam0', 'cam1'):
            source = next((self.euroc / 'mav0' / camera / 'data').glob('*.png'))
            link = self.output / camera / source.name
            self.assertEqual(source.stat().st_ino, link.stat().st_ino)
            self.assertFalse(link.is_symlink())

    def test_existing_directory_preserved(self):
        self.output.mkdir()
        marker = self.output / 'user-work.txt'
        marker.write_text('preserve')
        with self.assertRaises(FileExistsError):
            f.prepare_bagcreater(self.euroc, self.output)
        self.assertEqual(marker.read_text(), 'preserve')

    def test_invalid_header_units_empty_nonfinite_and_stamps(self):
        cases = [(f.EUROC_HEADER[::-1], self.rows), (f.EUROC_HEADER, []),
                 (f.EUROC_HEADER, [['1', *self.rows[0][1:]]]),
                 (f.EUROC_HEADER, [self.rows[0], self.rows[0]]),
                 (f.EUROC_HEADER, [[self.rows[0][0], 'nan', *self.rows[0][2:]]])]
        for header, rows in cases:
            with self.subTest(rows=rows):
                self.write_imu(header, rows)
                with self.assertRaises(ValueError):
                    f.prepare_bagcreater(self.euroc, self.output)
                self.assertFalse(self.output.exists())

    def test_receipt_quotes_commands_and_hashes_input(self):
        receipt = self.root / 'README.txt'
        f.receipt(receipt, [self.imu], ['tool', 'path with spaces', '$(literal)', 'line\nbreak'], diagnostic=True)
        content = receipt.read_text()
        self.assertIn(f.sha256(self.imu), content)
        self.assertIn("'$(literal)'", content)
        self.assertIn('Diagnostic override: True', content)

    def test_synthetic_chain_through_pr3_converter(self):
        converter = LAB.parents[1] / 'tools/calib_tools/calib_convert.py'
        result = subprocess.run([sys.executable, str(converter), '--camchain',
                                 str(FIXTURES / 'synthetic-camchain-imucam.yaml'), '--imu',
                                 str(LAB / 'config/imu_d435i_provisional.yaml'), '--output-dir',
                                 str(self.root / 'orbslam3'), '--target', 'orb', '--fps', '30', '--units-si'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        text = (self.root / 'orbslam3/orbslam3.yaml').read_text()
        # Independently load OpenCV matrix payload; assert the physical direction.
        class OpenCVLoader(yaml.SafeLoader):
            pass
        OpenCVLoader.add_constructor('tag:yaml.org,2002:opencv-matrix',
                                     lambda loader, node: loader.construct_mapping(node, deep=True))
        converted = yaml.load('\n'.join(text.splitlines()[1:]), Loader=OpenCVLoader)
        matrix = np.array(converted['IMU.T_b_c1']['data']).reshape(4, 4)
        np.testing.assert_allclose(matrix[:3, 3], [-0.00552, 0.00510, 0.01174], atol=1e-8)
        self.assertEqual(converted['Camera.fps'], 30)
        self.assertTrue((self.root / 'orbslam3/provenance.json').is_file())

    def test_real_bash_receipt_default_and_diagnostic_modes(self):
        # Source-only helper path: never invokes Docker, hardware or the exporter.
        script = 'source "$1"; OUT="$2"; receipt --input "$3" -- tool "path with spaces"; LAB_DIAGNOSTIC=yes; receipt -- tool'
        result = subprocess.run(['/bin/bash', '-c', script, 'synthetic-shell-check',
                                 str(LAB / 'mac/common.sh'), str(self.root), str(self.imu)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        content = (self.root / 'README.txt').read_text()
        self.assertIn('Diagnostic override: False', content)
        self.assertIn('Diagnostic override: True', content)
        self.assertIn("'path with spaces'", content)

    def test_shell_entrypoints_reject_missing_arguments_without_external_work(self):
        scripts = [LAB / 'mac/take_to_kalibr_bag.sh', LAB / 'mac/run_kalibr_camimu.sh',
                   LAB / 'mac/run_kalibr_stereo_check.sh', LAB / 'pi/record_calib_take.sh']
        for script in scripts:
            with self.subTest(script=script):
                result = subprocess.run(['/bin/bash', str(script)], capture_output=True, text=True)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn('Usage:', result.stderr)


class RecorderDiagnostics(unittest.TestCase):
    def setUp(self):
        self.analysis = {stream: {'count': 100, 'max_gap_ms': 1000/hz, 'rate_hz': hz,
                                 'nonpositive_steps': 0, 'domains': ['Global Time'], 'frame_number_skips': 0}
                         for stream, hz in d.RATES.items()}

    def test_ram_formula_and_bounds(self):
        self.assertEqual(d.ram_required(120), 3_974_400_000)
        self.assertEqual(d.ram_required(60), 1_987_200_000)
        for value in (0, 121, 'nan', 'inf', '-1', 'abc'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                d.ram_required(value)

    def test_clean_is_diagnostic_and_faults_never_usable(self):
        text, usable = d.summary(self.analysis, 'throttled=0x0\nthrottled=0x0', 0, 0, True)
        self.assertTrue(usable)
        self.assertIn('take usable for Kalibr: yes (diagnostic)', text)
        self.assertNotIn('PASS', text)
        for power, rc, arc, copied in [('throttled=0x50005', 0, 0, True), ('', 0, 0, True),
                                      ('throttled=0x0 POWER_COMMAND_FAILED', 0, 0, True),
                                      ('throttled=0x0', 1, 0, True), ('throttled=0x0', 0, 1, True),
                                      ('throttled=0x0', 0, 0, False)]:
            with self.subTest(power=power, rc=rc):
                self.assertFalse(d.summary(self.analysis, power, rc, arc, copied)[1])

    def test_gaps_steps_frames_and_domains(self):
        cases = [('max_gap_ms', 100), ('nonpositive_steps', 1), ('frame_number_skips', 1),
                 ('domains', ['System Time']), ('count', 0), ('rate_hz', 1)]
        for field, value in cases:
            analysis = copy.deepcopy(self.analysis)
            analysis['Infrared 1'][field] = value
            with self.subTest(field=field):
                self.assertFalse(d.summary(analysis, 'throttled=0x0', 0, 0, True)[1])
        self.analysis['Accel']['domains'] = ['Hardware Clock']
        self.assertFalse(d.summary(self.analysis, 'throttled=0x0', 0, 0, True)[1])


if __name__ == '__main__':
    unittest.main()
