import base64
import csv
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from mcap.reader import make_reader
from PIL import Image

from tools.foxglove_twin.to_mcap import convert, SCHEMA_NAMES


class ConverterTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.frames = self.root / "render"
        self.frames.mkdir()
        self.output = self.root / "twin.mcap"
        # Deliberately not representable through a float with ns precision.
        self.stamps = [1792900867686080566, 1792900867786080567, 1792900867886080569]

    def pngs(self, count=3, digits=5):
        paths = []
        for k in range(count):
            path = self.frames / f"frame_{k:0{digits}d}.png"
            Image.new("RGB", (3, 2), (k * 50, 60, 90)).save(path)
            paths.append(path)
        return paths

    def csv(self, name, rows):
        with (self.frames / name).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    def timeline(self):
        # Five gate ticks but only three render frames: positional zip is wrong.
        rows = []
        for t, index, phase in [(0, 0, "STAND"), (.05, 0, "CROUCH"),
                                (.1, 1, "FLIGHT"), (.15, 1, "LAND"), (.2, 2, "SETTLE")]:
            rows.append(dict(t_s=t, frame_stamp_ns=self.stamps[index],
                             raw_advisory="STOP_CANDIDATE", gated_advisory="UNCERTAIN",
                             gate_phase=phase, reason="synthetic_fixture", gated_confidence=0))
        self.csv("timeline.csv", rows)

    def read(self):
        with self.output.open("rb") as stream:
            return list(make_reader(stream).iter_messages(log_time_order=False))

    def check_round_trip(self, expected_stamps, paths):
        records = self.read()
        self.assertEqual(len(records), len(paths) * 3)
        self.assertEqual({channel.topic for _, channel, _ in records}, set(SCHEMA_NAMES))
        by_topic = {}
        for schema, channel, message in records:
            self.assertEqual(schema.name, SCHEMA_NAMES[channel.topic])
            self.assertEqual(schema.encoding, "jsonschema")
            self.assertEqual(json.loads(schema.data)["title"], schema.name)
            self.assertEqual(channel.message_encoding, "json")
            self.assertEqual(message.log_time, message.publish_time)
            by_topic.setdefault(channel.topic, []).append(message)
        for messages in by_topic.values():
            self.assertEqual([m.log_time for m in messages], expected_stamps)
            self.assertEqual([m.sequence for m in messages], list(range(len(paths))))
        for message, stamp, path in zip(by_topic["/twin/image"], expected_stamps, paths):
            image = json.loads(message.data)
            self.assertEqual(image["timestamp"], dict(zip(("sec", "nsec"), divmod(stamp, 10**9))))
            self.assertEqual(image["frame_id"], "twin")
            self.assertEqual(image["format"], "png")
            self.assertEqual(base64.b64decode(image["data"]), path.read_bytes())
        return by_topic

    def test_jump_gate_samples_ticks_at_render_times(self):
        paths = self.pngs()
        self.timeline()
        result = convert(self.frames, self.output)
        self.assertEqual(result["message_counts"], {topic: 3 for topic in SCHEMA_NAMES})
        topics = self.check_round_trip(self.stamps, paths)
        advisories = [json.loads(m.data) for m in topics["/twin/advisory"]]
        self.assertEqual([a["phase"] for a in advisories], ["STAND", "FLIGHT", "SETTLE"])
        self.assertEqual(advisories[1], dict(raw="STOP_CANDIDATE", gated="UNCERTAIN",
                                           phase="FLIGHT", reason="synthetic_fixture", confidence=0))
        self.assertEqual([json.loads(m.data) for m in topics["/twin/phase"]],
                         [{"message": a["phase"]} for a in advisories])

    def test_scene_frames_csv_preserves_held_duplicate_stamps(self):
        paths = self.pngs(digits=4)
        stamps = [self.stamps[0], self.stamps[0], self.stamps[2]]
        self.csv("frames.csv", [dict(png=p.name, playback_t_s=k / 10, frame_stamp_ns=s,
                                     packet_index=k) for k, (p, s) in enumerate(zip(paths, stamps))])
        convert(self.frames, self.output)
        topics = self.check_round_trip(stamps, paths)
        for message in topics["/twin/advisory"]:
            self.assertEqual(json.loads(message.data), dict(raw=None, gated=None, phase="UNKNOWN",
                                                            reason=None, confidence=None))

    def test_custom_jump_render_fps_and_clamped_endpoint(self):
        paths = self.pngs(count=2)
        self.timeline()
        convert(self.frames, self.output, fps=4)
        self.check_round_trip([self.stamps[0], self.stamps[-1]], paths)

    def test_wrong_fps_fails_before_output_creation(self):
        self.pngs()
        self.timeline()
        with self.assertRaisesRegex(ValueError, "same --fps"):
            convert(self.frames, self.output, fps=20)
        self.assertFalse(self.output.exists())

    def test_existing_output_is_preserved(self):
        self.pngs()
        self.timeline()
        self.output.write_bytes(b"existing evidence")
        with self.assertRaises(FileExistsError):
            convert(self.frames, self.output)
        self.assertEqual(self.output.read_bytes(), b"existing evidence")

    def test_output_inside_input_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            convert(self.frames, self.frames / "twin.mcap")

    def test_source_archive_output_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "source archive"):
            convert(self.frames, Path.home() / "wbr-evidence-archive/lab-day-20261025/new.mcap")

    def test_missing_scene_png_is_rejected(self):
        self.pngs(count=2, digits=4)
        self.csv("frames.csv", [dict(png="frame_0000.png", frame_stamp_ns=self.stamps[0])])
        with self.assertRaisesRegex(ValueError, "match"):
            convert(self.frames, self.output)

    def test_ambiguous_csv_is_rejected(self):
        self.pngs()
        self.timeline()
        self.csv("frames.csv", [dict(png="frame_00000.png", frame_stamp_ns=self.stamps[0])])
        with self.assertRaisesRegex(ValueError, "exactly one"):
            convert(self.frames, self.output)

    def test_non_integer_stamp_is_rejected(self):
        self.pngs(count=1, digits=4)
        self.csv("frames.csv", [dict(png="frame_0000.png", frame_stamp_ns="1792900867.5")])
        with self.assertRaisesRegex(ValueError, "frames.csv:2"):
            convert(self.frames, self.output)

    def test_nonfinite_confidence_is_rejected(self):
        self.pngs(count=1)
        self.csv("timeline.csv", [dict(t_s=0, frame_stamp_ns=self.stamps[0], raw_advisory="NONE",
                                       gated_advisory="NONE", gate_phase="STAND", reason="fixture",
                                       gated_confidence="nan")])
        with self.assertRaisesRegex(ValueError, "gated_confidence"):
            convert(self.frames, self.output)
        self.assertFalse(self.output.exists())

    def test_layout_has_two_image_topics_and_phase_expression(self):
        layout_path = Path(__file__).resolve().parents[3] / "docs/foxglove_twin/layout.json"
        configs = json.loads(layout_path.read_text())["configById"]
        self.assertEqual({c["imageMode"]["imageTopic"] for key, c in configs.items()
                          if key.startswith("Image!")},
                         {"/camera/camera/color/image_raw", "/twin/image"})
        self.assertEqual(configs["RawMessages!advisory"]["topicPath"], "/twin/advisory")
        self.assertEqual(configs["StateTransitions!phase"]["paths"][0]["value"], "/twin/phase.message")


if __name__ == "__main__":
    unittest.main()
