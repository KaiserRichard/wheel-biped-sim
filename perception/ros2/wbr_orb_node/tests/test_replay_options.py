"""Pure-Python replay option and command construction checks."""
import importlib.util
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).with_name('replay_map_points.py')
SPEC = importlib.util.spec_from_file_location('replay_map_points', SCRIPT)
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)


class ReplayOptionsTest(unittest.TestCase):
    def test_duration_and_rate_defaults_preserve_existing_replay(self):
        args = replay.make_parser().parse_args([
            '--node', 'node', '--bag', 'bag', '--settings', 'settings',
            '--vocabulary', 'vocabulary', '--output', 'output'])
        self.assertEqual(args.seconds, 20)
        self.assertEqual(args.rate, 0.3)

    def test_player_deadline_is_play_time_plus_margin(self):
        self.assertEqual(replay.child_deadline_seconds(20, 0.3), 97)
        self.assertEqual(replay.child_deadline_seconds(300, 0.5, margin=45), 645)

    def test_reads_rosbag_metadata_duration(self):
        metadata = '''rosbag2_bagfile_information:
  version: 5
  storage_identifier: mcap
  duration:
    nanoseconds: 300250000000
  starting_time:
    nanoseconds_since_epoch: 0
'''
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'metadata.yaml'
            path.write_text(metadata)
            self.assertEqual(replay.read_bag_duration(path), 300.25)

    def test_play_command_uses_requested_duration_and_rate(self):
        command = replay.build_play_command('/recording', 137.5, 0.75)
        self.assertEqual(command[0:4], ['ros2', 'bag', 'play', '/recording'])
        self.assertEqual(command[command.index('--rate') + 1], '0.75')
        self.assertEqual(command[command.index('--playback-duration') + 1], '137.5')


if __name__ == '__main__':
    unittest.main()
