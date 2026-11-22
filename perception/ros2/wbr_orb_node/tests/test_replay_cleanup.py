"""POSIX supervisor fault check with sleeping stubs; no ROS, bag or camera."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest


class ReplayCleanupTest(unittest.TestCase):
    def test_sigterm_cleans_owned_process_groups(self):
        harness = Path(__file__).with_name('replay_map_points.py')
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            stub = '#!' + sys.executable + '\nimport time\nprint("Listening for topics; All requested topics are subscribed; viewer disabled", flush=True)\ntime.sleep(60)\n'
            for name in ('ros2', 'node'):
                path = root / name
                path.write_text(stub)
                path.chmod(0o755)
            output = root / 'output'
            command = [sys.executable, str(harness), '--node', str(root / 'node'),
                       '--bag', str(root / 'unused-bag'), '--settings', str(root / 'unused-settings'),
                       '--vocabulary', str(root / 'unused-vocabulary'), '--output', str(output)]
            env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'])
            supervisor = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
            receipt = {}
            try:
                until = time.monotonic() + 5
                while time.monotonic() < until:
                    path = output / 'receipt.json'
                    if path.exists():
                        try:
                            receipt = json.loads(path.read_text())
                        except json.JSONDecodeError:
                            continue
                    if 'play' in receipt.get('pids', {}):
                        break
                    time.sleep(0.02)
                self.assertIn('play', receipt.get('pids', {}))
                supervisor.send_signal(signal.SIGTERM)
                logs, _ = supervisor.communicate(timeout=12)
                self.assertEqual(supervisor.returncode, 1, logs.decode())
                receipt = json.loads((output / 'receipt.json').read_text())
                self.assertIn('signal 15', receipt['error'])
                self.assertTrue(receipt['all_owned_children_exited'])
                self.assertEqual(receipt['remaining_process_groups'], [])
                for pid in receipt['pids'].values():
                    with self.assertRaises(ProcessLookupError):
                        os.killpg(pid, 0)
            finally:
                if supervisor.poll() is None:
                    supervisor.kill()
                    supervisor.wait()
                for pid in receipt.get('pids', {}).values():
                    try:
                        os.killpg(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                if supervisor.stdout:
                    supervisor.stdout.close()
