"""Offline fixtures only; UNTESTED ON HARDWARE."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml

HERE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("n3_params", HERE / "n3_params.py")
n3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(n3)


class ReadbackTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads(n3.PROFILE.read_text())
        self.fixture = HERE / "tests/fixtures/matching_params.yaml"
        self.dump = yaml.safe_load(self.fixture.read_text())

    def test_handwritten_match_and_cli(self):
        self.assertEqual(n3.compare_dump(self.dump, self.profile), [])
        result = subprocess.run([sys.executable, str(HERE / "n3_params.py"),
                                 "--dump", str(self.fixture)], capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout)

    def test_every_required_parameter_mismatch_and_missing(self):
        params = n3.flatten(self.dump['/camera/camera']['ros__parameters'])
        for name in n3.expected_parameters(self.profile):
            with self.subTest(name=name):
                for value in (None, 'WRONG'):
                    bad = copy.deepcopy(params)
                    if value is None:
                        del bad[name]
                    else:
                        bad[name] = value
                    errors = n3.compare_dump({'/camera/camera': {'ros__parameters': bad}}, self.profile)
                    self.assertTrue(any(error.startswith(name + ':') for error in errors))

    def test_profile_emitter_unite_and_wrong_node(self):
        for key, value in [('depth_module.infra_profile', '848x480x30'),
                           ('depth_module.emitter_enabled', 1), ('unite_imu_method', 2),
                           ('gyro_fps', 100), ('accel_fps', 200)]:
            params = n3.flatten(self.dump['/camera/camera']['ros__parameters'])
            params[key] = value
            self.assertTrue(n3.compare_dump({'/camera/camera': {'ros__parameters': params}}, self.profile))
        with self.assertRaises(KeyError):
            n3.compare_dump({'/wrong': self.dump['/camera/camera']}, self.profile)

    def test_profile_value_is_taken_from_config(self):
        self.profile['profile']['width']['value'] = 848
        self.assertTrue(n3.compare_dump(self.dump, self.profile))

    def test_cli_mismatch_and_malformed_are_nonzero(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'params.yaml'
            for content, code in [
                (self.fixture.read_text().replace('emitter_enabled: 0', 'emitter_enabled: 1'), 1),
                ('[malformed', 2),
                ('{}', 2),
            ]:
                path.write_text(content)
                result = subprocess.run([sys.executable, str(HERE / 'n3_params.py'),
                                         '--dump', str(path)], capture_output=True)
                self.assertEqual(result.returncode, code, result.stdout)


if __name__ == '__main__':
    unittest.main()
