import contextlib
import io
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import e2e_pty_test as e2e
from vio_to_pose_events import Converter, load_mapping
from perception_health import TRACKING_STATES, U32_MAX


class ConverterTests(unittest.TestCase):
    def setUp(self):
        self.snapshot = e2e.synthetic_snapshot()

    def test_no_mapping_has_invalid_mapping_without_timestamp_subtraction(self):
        event = Converter("synthetic-camera").convert(self.snapshot)
        self.assertFalse(event["mapping"]["valid"])
        self.assertEqual(event["pose_source_d435i_ns"], self.snapshot["source_ns"])

    def test_all_exact_orb_state_names(self):
        collapsed = {"SYSTEM_NOT_READY": "STARTING", "NO_IMAGES_YET": "STARTING",
                     "NOT_INITIALIZED": "INITIALIZING", "OK": "TRACKING",
                     "OK_KLT": "TRACKING", "RECENTLY_LOST": "LOST", "LOST": "LOST"}
        for raw, state in collapsed.items():
            with self.subTest(raw=raw):
                snapshot = dict(self.snapshot, state=state, orb_tracking_state=raw)
                event = Converter("synthetic-camera").convert(snapshot)
                self.assertEqual(event["tracking_state"], raw)
                self.assertTrue(event["integration"]["tracking_exact"])

    def test_real_evidence_fixture_preserves_unknowns(self):
        converter = Converter("saved-real-replay")
        rows = [json.loads(line) for line in (HERE / "tests/fixtures/real_excerpt.jsonl").read_text().splitlines()]
        events = [converter.convert(row) for row in rows]
        self.assertEqual(events[1]["tracking_state"], "OK")
        self.assertTrue(events[1]["degraded"])
        self.assertFalse(events[1]["imu_initialized"])
        self.assertNotIn("tracked_features", events[1])
        self.assertEqual(events[2]["tracking_state"], "LOST")
        self.assertFalse(events[2]["integration"]["tracking_exact"])

    def test_absent_reset_not_invented(self):
        self.snapshot.pop("reset_counter")
        event = Converter("test").convert(self.snapshot)
        self.assertNotIn("map_reset_counter", event)
        self.assertEqual(event["integration"]["map_reset_source"], "UNAVAILABLE")
        self.assertTrue(event["degraded"])

    def test_reset_epoch_wrap_and_map_change_derivation(self):
        converter = Converter("test")
        self.assertEqual(converter.convert(dict(self.snapshot, reset_counter=65535))["map_reset_counter"], 65535)
        self.assertEqual(converter.convert(dict(self.snapshot, reset_counter=65536))["map_reset_counter"], 0)
        self.snapshot.pop("reset_counter")
        self.assertEqual(Converter("test").convert(dict(self.snapshot, epoch_id=12))["map_reset_counter"], 12)
        converter = Converter("test")
        events = [converter.convert(dict(self.snapshot, map_id=m)) for m in (7, 7, 8, 8, 2)]
        self.assertEqual([e["map_reset_counter"] for e in events], [0, 0, 1, 1, 2])
        self.assertTrue(all(e["degraded"] for e in events))

    def test_regressions_and_domain_switch_are_rejected(self):
        for change in ({"source_ns": 1}, {"domain": "hardware_clock"}, {"reset_counter": 0}):
            converter = Converter("test")
            converter.convert(dict(self.snapshot, reset_counter=1))
            with self.assertRaises(ValueError):
                converter.convert(dict(self.snapshot, **change))

    def test_invalid_fields_rejected_and_feature_count_saturates(self):
        for change in ({"source_ns": 1.5}, {"reset_counter": -1}, {"tracked_features": True},
                       {"orb_tracking_state": "OK_KLT", "state": "LOST"}):
            with self.assertRaises(ValueError):
                Converter("test").convert(dict(self.snapshot, **change))
        self.assertEqual(Converter("test").convert(dict(self.snapshot, tracked_features=100000))["tracked_features"], 65535)

    def test_null_source_and_unknown_state_never_fresh(self):
        event = Converter("test").convert(dict(self.snapshot, source_ns=None, state="UNKNOWN", orb_tracking_state=None))
        self.assertFalse(event["mapping"]["valid"])
        self.assertFalse(event["integration"]["source_available"])
        self.assertTrue(event["degraded"])

    def test_mapping_rejects_wrong_clock_and_insufficient_exchanges(self):
        with tempfile.TemporaryDirectory(dir=HERE / "tests") as temp:
            path = Path(temp) / "mapping.json"
            for mapping in (dict(e2e.synthetic_mapping(), pi_domain="host_steady"),
                            dict(e2e.synthetic_mapping(), pi_domain="Linux_CLOCK_MONOTONIC_RAW"),
                            dict(e2e.synthetic_mapping(), exchanges_ns=[]),
                            dict(e2e.synthetic_mapping(), valid=1),
                            dict(e2e.synthetic_mapping(), valid=False)):
                path.write_text(json.dumps(mapping))
                with self.assertRaises(ValueError):
                    load_mapping(path)

    def test_mapping_explicit_file_and_identity_matching(self):
        with tempfile.TemporaryDirectory(dir=HERE / "tests") as temp:
            path = Path(temp) / "mapping.json"
            path.write_text(json.dumps(e2e.synthetic_mapping()))
            mapping = load_mapping(path)
            self.assertTrue(Converter("synthetic-camera", mapping).convert(self.snapshot)["mapping"]["valid"])
            self.assertFalse(Converter("wrong-session", mapping).convert(self.snapshot)["mapping"]["valid"])
            wrong_domain = dict(self.snapshot, domain="hardware_clock")
            self.assertFalse(Converter("synthetic-camera", mapping).convert(wrong_domain)["mapping"]["valid"])

    def test_cli_streaming_and_usage_error(self):
        command = [sys.executable, "-B", str(HERE / "vio_to_pose_events.py"), "--snapshot-session", "test"]
        good = subprocess.run(command, input=json.dumps(self.snapshot) + "\n", text=True, capture_output=True)
        self.assertEqual(good.returncode, 0, good.stderr)
        self.assertFalse(json.loads(good.stdout)["mapping"]["valid"])
        bad = subprocess.run(command, input="{bad}\n", text=True, capture_output=True)
        self.assertEqual(bad.returncode, 3)
        self.assertIn("line 1", bad.stderr)
        wrong_shape = subprocess.run(command, input="[]\n", text=True, capture_output=True)
        self.assertEqual(wrong_shape.returncode, 3)
        missing_argument = subprocess.run(command[:-2], text=True, capture_output=True)
        self.assertEqual(missing_argument.returncode, 3)


class EndToEndTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(dir=HERE / "tests")
        cls.out = Path(cls.temp.name)
        (cls.out / "policy.json").write_bytes(e2e.POLICY_PATH.read_bytes())
        (cls.out / "integration_config.json").write_bytes((HERE / "config.json").read_bytes())
        (cls.out / "provenance.json").write_text(json.dumps({"captured_utc": e2e.utc(), "inputs": []}))
        for name in e2e.SCENARIOS:
            e2e.capture(cls.out / name, name)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def states(self, name):
        trace, health, integrity, metrics = e2e.replay(self.out / name, silence=name == "sender_stops", primary_fresh=True)
        self.assertTrue(integrity)
        self.assertGreater(metrics["bytes_written_and_received"], 0)
        return [row["state"] for row in trace if row["kind"] == "health"], trace, health

    def test_no_mapping_hold_only(self):
        states, trace, health = self.states("no_mapping")
        self.assertTrue(all(h.pose_age_us == U32_MAX and h.flags & 3 == 0 for h in health))
        self.assertEqual(states, ["HOLD"] * len(health))
        self.assertTrue(all(row["reason"] == "SOURCE_AGE_HOLD" for row in trace if row["kind"] == "health"))

    def test_health_cannot_grant_primary_freshness(self):
        trace, _, integrity, _ = e2e.replay(self.out / "mapped_ok")
        self.assertTrue(integrity)
        self.assertNotIn("FRESH", [row["state"] for row in trace])
        self.assertTrue(all(row["primary_fresh"] is False for row in trace))

    def test_mapped_ok_is_fresh(self):
        states, _, health = self.states("mapped_ok")
        self.assertEqual(states, ["FRESH"] * len(health))
        self.assertTrue(all(h.flags == 3 and h.tracking_state == TRACKING_STATES["OK"] for h in health))

    def test_lost_is_latched_after_recovery(self):
        states, trace, _ = self.states("lost_latched")
        self.assertEqual(states, ["FRESH", "FAILSAFE", "FAILSAFE"])
        self.assertEqual(trace[-1]["reason"], "SOURCE_AGE_TIMEOUT")

    def test_sender_eof_is_receipt_silence_with_isolated_policy(self):
        states, trace, _ = self.states("sender_stops")
        self.assertEqual(states, ["FRESH"] * len(states))
        self.assertEqual([(row["state"], row["reason"]) for row in trace[-2:]],
                         [("HOLD", "RESULT_SILENCE"), ("FAILSAFE", "RESULT_TIMEOUT")])

    def test_entire_saved_real_replay_passes_through_sender(self):
        states, _, health = self.states("real_replay")
        self.assertEqual(len(health), len(e2e.REAL.read_text().splitlines()))
        self.assertNotIn("FRESH", states)
        self.assertTrue(all(h.pose_age_us == U32_MAX and h.flags & 4 for h in health))

    def test_analyzer_pass_contract_with_real_evidence(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(e2e.analyze(self.out), 0)
        result = json.loads((self.out / "result.json").read_text())
        self.assertEqual(result["status"], "PASS")
        self.assertEqual([c["id"] for c in result["criteria"] if c["status"] == "FAIL"], [])
        self.assertTrue(result["raw_files"])

    def test_analyzer_blocked_for_missing_captures(self):
        with tempfile.TemporaryDirectory(dir=HERE / "tests") as temp, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(e2e.analyze(Path(temp)), 2)
            self.assertEqual(json.loads((Path(temp) / "result.json").read_text())["status"], "BLOCKED")

    def test_analyzer_uses_raw_bytes_instead_of_saved_trace(self):
        with tempfile.TemporaryDirectory(dir=HERE / "tests") as temp:
            out = Path(temp) / "saved"
            shutil.copytree(self.out, out)
            wire = out / "mapped_ok/uart.bin"
            damaged = bytearray(wire.read_bytes())
            damaged[-1] ^= 1
            wire.write_bytes(damaged)
            (out / "mapped_ok/trace.json").write_text('[{"state":"FRESH"}]')
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(e2e.analyze(out), 1)
            result = json.loads((out / "result.json").read_text())
            transport = next(c for c in result["criteria"] if c["id"] == "mapped_ok.transport")
            self.assertEqual(transport["status"], "FAIL")

    def test_analyzer_requires_heartbeat_evidence(self):
        with tempfile.TemporaryDirectory(dir=HERE / "tests") as temp:
            out = Path(temp) / "saved"
            shutil.copytree(self.out, out)
            scenario = out / "no_mapping"
            frames = e2e.StreamParser().feed((scenario / "uart.bin").read_bytes())
            health_wire = b"".join(e2e.encode_frame(typ, seq, payload)
                                     for typ, seq, payload in frames
                                     if typ == e2e.TYPE_PERCEPTION_HEALTH)
            capture = json.loads((scenario / "capture.json").read_text())
            capture["ended_host_CLOCK_MONOTONIC_ns"] = capture["started_host_CLOCK_MONOTONIC_ns"] + 1_000_000
            (scenario / "capture.json").write_text(json.dumps(capture))
            (scenario / "uart.bin").write_bytes(health_wire)
            (scenario / "reads.json").write_text(json.dumps([{
                "offset": 0, "length": len(health_wire),
                "host_CLOCK_MONOTONIC_ns": capture["started_host_CLOCK_MONOTONIC_ns"] + 500_000,
            }]))
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(e2e.analyze(out), 1)
            result = json.loads((out / "result.json").read_text())
            transport = next(c for c in result["criteria"] if c["id"] == "no_mapping.transport")
            self.assertEqual(transport["status"], "FAIL")

    def test_analyzer_rejects_read_timestamp_before_capture_start(self):
        with tempfile.TemporaryDirectory(dir=HERE / "tests") as temp:
            out = Path(temp) / "saved"
            shutil.copytree(self.out, out)
            scenario = out / "no_mapping"
            reads = json.loads((scenario / "reads.json").read_text())
            capture = json.loads((scenario / "capture.json").read_text())
            reads[0]["host_CLOCK_MONOTONIC_ns"] = capture["started_host_CLOCK_MONOTONIC_ns"] - 1
            (scenario / "reads.json").write_text(json.dumps(reads))
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(e2e.analyze(out), 1)
            result = json.loads((out / "result.json").read_text())
            criterion = next(c for c in result["criteria"] if c["id"] == "no_mapping")
            self.assertEqual(criterion["status"], "FAIL")

    def test_missing_capture_prerequisite_returns_blocked_result(self):
        with tempfile.TemporaryDirectory(dir=HERE / "tests") as temp:
            out = Path(temp) / "missing"
            missing_real = Path(temp) / "missing-snapshots.jsonl"
            with mock.patch.object(e2e, "REAL", missing_real), \
                    mock.patch.object(e2e.sys, "argv", ["e2e_pty_test.py", "--outdir", str(out)]), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(e2e.main(), 2)
            result = json.loads((out / "result.json").read_text())
            self.assertEqual(result["status"], "BLOCKED")

    def test_summary_retains_full_capture_hashes_within_ticket_budget(self):
        with tempfile.TemporaryDirectory(dir=HERE / "tests") as temp:
            out = Path(temp) / "full"
            shutil.copytree(self.out, out)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(e2e.analyze(out), 0)
            archive = Path(temp) / "summary"
            e2e.summarize(out, archive)
            self.assertLess(sum(p.stat().st_size for p in archive.iterdir()),
                            e2e.CONFIG["evidence_max_bytes"]["value"])
            summary = json.loads((archive / "summary.json").read_text())
            self.assertEqual(summary["result"]["status"], "PASS")
            manifest = json.loads((archive / "sha256.json").read_text())
            events = next(row for row in manifest if row["path"] == "real_replay/events.jsonl")
            self.assertEqual(events["sha256"], e2e.hashlib.sha256((out / events["path"]).read_bytes()).hexdigest())
            self.assertEqual(events["bytes"], (out / events["path"]).stat().st_size)
            self.assertFalse((archive / events["path"]).exists())
            with self.assertRaises(FileExistsError):
                e2e.summarize(out, archive)
