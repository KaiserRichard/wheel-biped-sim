"""SYNTHETIC unit/CLI fixtures plus a separately labelled REAL-DATA regression."""

from __future__ import annotations

import builtins
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import traj_eval as te

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SCRATCH = HERE / "_scratch" / "tests"


def synthetic_trajectory(positions, stamps=None, quaternions=None):
    """SYNTHETIC fixture: default integer-second stamps, identity rotations."""
    positions = np.asarray(positions, dtype=float)
    n = len(positions)
    if stamps is None:
        stamps = np.arange(n, dtype=np.int64) * te.NS_PER_SECOND
    if quaternions is None:
        quaternions = np.tile([0., 0., 0., 1.], (n, 1))
    return te.Trajectory(np.asarray(stamps, dtype=np.int64), positions, quaternions,
                         {"fixture_label": "SYNTHETIC", "time_unit": "ns"})


def synthetic_evaluate(est, ref, **kwargs):
    return te.evaluate(est, ref, clock_assumption="SYNTHETIC shared clock",
                       est_frame="SYNTHETIC body", ref_frame="SYNTHETIC body", **kwargs)


class SyntheticTests(unittest.TestCase):
    """Every input produced by this class is SYNTHETIC."""

    def setUp(self):
        SCRATCH.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix="synthetic-", dir=SCRATCH)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.rng = np.random.default_rng(8123)
        self.points = self.rng.normal(size=(40, 3))
        quat = self.rng.normal(size=4)
        quat /= np.linalg.norm(quat)
        self.rotation = te.quaternion_matrices(quat[None])[0]
        self.translation = self.rng.normal(size=3)

    def write(self, name, text):
        path = self.root / name
        path.write_text(text, encoding="utf-8")
        return path

    def write_tum(self, name, points=None):
        points = self.points if points is None else points
        text = "# SYNTHETIC fixture\n" + "\n".join(
            f"{i} {p[0]:.17g} {p[1]:.17g} {p[2]:.17g} 0 0 0 1"
            for i, p in enumerate(points)) + "\n"
        return self.write(name, text)

    def cli(self, args):
        output, errors = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(errors):
            try:
                code = te.main(args)
            except SystemExit as exc:
                code = exc.code
        return code, output.getvalue(), errors.getvalue()

    def common_cli(self, est, ref, out):
        return ["--est", str(est), "--ref", str(ref), "--out", str(out),
                "--clock-assumption", "SYNTHETIC shared clock", "--est-frame", "SYNTHETIC body",
                "--ref-frame", "SYNTHETIC body", "--no-plots"]

    def test_synthetic_umeyama_se3_and_sim3(self):
        for scale in (1., 2.7):
            with self.subTest(scale=scale):
                target = scale * self.points @ self.rotation.T + self.translation
                rotation, translation, recovered_scale = te.umeyama(self.points, target, scale != 1)
                np.testing.assert_allclose(rotation, self.rotation, atol=1e-9, rtol=0)
                np.testing.assert_allclose(translation, self.translation, atol=1e-9, rtol=0)
                self.assertLess(abs(recovered_scale - scale), 1e-9)

    def test_synthetic_ate_alignment_and_wrong_scale(self):
        ref = synthetic_trajectory(self.points)
        rigid = synthetic_trajectory(self.points @ self.rotation.T + self.translation)
        scaled = synthetic_trajectory(2.7 * self.points @ self.rotation.T + self.translation)
        self.assertLess(synthetic_evaluate(rigid, ref).result["metrics"]["se3"]["ate_translation_m"]["rmse"], 1e-9)
        both = synthetic_evaluate(scaled, ref, align_both=True)
        self.assertLess(both.result["metrics"]["sim3"]["ate_translation_m"]["rmse"], 1e-9)
        self.assertGreater(both.result["metrics"]["se3"]["ate_translation_m"]["rmse"], 0.1)
        self.assertAlmostEqual(both.result["metrics"]["sim3"]["alignment"]["s"], 1/2.7)

    def test_synthetic_reflection_fixed_and_degenerate_rejected(self):
        reflected = self.points * [-1, 1, 1]
        rotation, _, _ = te.umeyama(self.points, reflected, True)
        self.assertAlmostEqual(np.linalg.det(rotation), 1., places=12)
        with self.assertRaisesRegex(ValueError, "rank < 2"):
            te.umeyama(np.zeros((5, 3)), np.zeros((5, 3)))
        with self.assertRaisesRegex(ValueError, "at least three"):
            te.umeyama(self.points[:2], self.points[:2])

    def test_synthetic_n_to_align_fits_prefix_evaluates_all(self):
        est = self.points @ self.rotation.T + self.translation
        est[-1] += [0.8, 0, 0]
        evaluation = synthetic_evaluate(synthetic_trajectory(est), synthetic_trajectory(self.points), n_to_align=10)
        self.assertEqual(evaluation.result["metrics"]["se3"]["alignment"]["n_to_align"], 10)
        self.assertEqual(evaluation.result["metrics"]["se3"]["ate_translation_m"]["n"], 40)
        np.testing.assert_allclose(evaluation.series["se3"]["ate_m"][:-1], 0, atol=1e-9)
        self.assertAlmostEqual(evaluation.series["se3"]["ate_m"][-1], 0.8)

    def test_synthetic_quaternion_orders_and_comments(self):
        q = np.array([0.1, -0.2, 0.3, 0.9])
        q /= np.linalg.norm(q)
        tum = self.write("pose.tum", "# SYNTHETIC\n\n1 2 3 4 " + " ".join(map(str, q)) + " # inline\n")
        euroc = self.write("pose.csv", "# SYNTHETIC timestamp,p,q\n1000000000,2,3,4," +
                           ",".join(map(str, q[[3, 0, 1, 2]])) + ",1,2,3\n")
        a, b = te.read_trajectory(tum), te.read_trajectory(euroc, "euroc_gt")
        np.testing.assert_array_equal(a.t_ns, b.t_ns)
        np.testing.assert_array_equal(a.positions, b.positions)
        np.testing.assert_array_equal(a.quaternions, b.quaternions)
        self.assertEqual(b.metadata["input_quaternion_order"], "wxyz")
        np.testing.assert_allclose(te.quaternion_matrices(a.quaternions), te.quaternion_matrices(b.quaternions))

    def test_synthetic_timestamp_decimal_exactness_and_rounding(self):
        self.assertEqual(te.parse_timestamp("1403636579963555584.000000", "ns"), 1403636579963555584)
        self.assertEqual(te.parse_timestamp("1403636579.963555584", "s"), 1403636579963555584)
        self.assertEqual(te.parse_timestamp("1.403636579963555584e9", "s"), 1403636579963555584)
        self.assertEqual(te.parse_timestamp("0.0000000005", "s"), 0)
        self.assertEqual(te.parse_timestamp("0.0000000015", "s"), 2)
        trajectory = te.read_trajectory(self.write("rounding.tum", "0.0000000015 0 0 0 0 0 0 1\n"))
        self.assertEqual(trajectory.metadata["timestamps_rounded"], 1)
        self.assertEqual(trajectory.metadata["max_rounding_ns"], 0.5)
        for bad in ("nan", "inf", "x", "9223372036854775808", "1e999999999"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                te.parse_timestamp(bad, "ns")

    def test_synthetic_explicit_time_unit_overrides_default(self):
        path = self.write("seconds.orb", "1 0 0 0 0 0 0 1\n")
        self.assertEqual(te.read_trajectory(path, "orbslam3").t_ns[0], 1)
        parsed = te.read_trajectory(path, "orbslam3", "s")
        self.assertEqual(parsed.t_ns[0], 1_000_000_000)
        self.assertEqual(parsed.metadata["time_unit_source"], "explicit")

    def test_synthetic_nonmonotonic_and_quantized_duplicate_rejected(self):
        for stamps, kind in ((["1", "1"], "duplicate"), (["2", "1"], "reversal"),
                             (["1.0000000001", "1.0000000002"], "duplicate")):
            with self.subTest(stamps=stamps):
                path = self.write("bad.tum", "\n".join(f"{t} 0 0 0 0 0 0 1" for t in stamps))
                with self.assertRaisesRegex(ValueError, kind):
                    te.read_trajectory(path)

    def test_synthetic_pose_validation_and_norm_record(self):
        for row in ("0 0 nan 0 0 0 0 1", "0 inf 0 0 0 0 0 1", "0 0 0 0 0 0 0 0.998",
                    "0 0 0 0 0 0 0 0", "0 0 0 0 0 0 0 nan", "0 0 0"):
            with self.subTest(row=row), self.assertRaises(ValueError):
                te.read_trajectory(self.write("bad.tum", row))
        trajectory = te.read_trajectory(self.write("near.tum", "0 0 0 0 0 0 0 1.0005\n"))
        self.assertAlmostEqual(trajectory.metadata["quaternion_max_norm_deviation"], 0.0005)
        self.assertEqual(np.linalg.norm(trajectory.quaternions[0]), 1.)

    def test_synthetic_association_boundary_offset_and_shorter_side(self):
        est = synthetic_trajectory(np.zeros((2, 3)), [10_000_000, 120_000_001])
        ref = synthetic_trajectory(np.zeros((3, 3)), [0, 100_000_000, 300_000_000])
        a = te.associate(est, ref, "0.02")
        np.testing.assert_array_equal(a.est_indices, [0])
        a = te.associate(est, ref, "0.01", "-0.000000001")
        np.testing.assert_array_equal(a.est_indices, [0])
        a = te.associate(est, ref, "0.02", "-0.000000001")
        np.testing.assert_array_equal(a.est_indices, [0, 1])
        self.assertEqual(a.stats["max_abs_dt_ns"], 20_000_000)
        self.assertEqual(a.stats["iteration_side"], "estimate")
        self.assertEqual(a.stats["unmatched_ref"], 1)
        b = te.associate(ref, est, "0.02")
        self.assertEqual(b.stats["iteration_side"], "reference")
        np.testing.assert_array_equal(b.ref_indices, [0])

    def test_synthetic_association_equal_length_iterates_estimate_like_evo(self):
        # evo sync.associate_trajectories iterates the estimate unless it is strictly longer.
        est = synthetic_trajectory(np.zeros((3, 3)), [0, 1_000_000_000, 2_000_000_000])
        ref = synthetic_trajectory(np.zeros((3, 3)), [900_000_000, 1_100_000_000, 1_900_000_000])
        a = te.associate(est, ref, "0.95")
        self.assertEqual(a.stats["iteration_side"], "estimate")
        self.assertEqual(list(zip(a.est_indices.tolist(), a.ref_indices.tolist())), [(0, 0), (1, 0), (2, 2)])

    def test_synthetic_association_inclusive_exact_boundary(self):
        est = synthetic_trajectory(np.zeros((2, 3)), [10_000_000, 110_000_001])
        ref = synthetic_trajectory(np.zeros((3, 3)), [0, 100_000_000, 300_000_000])
        matched = te.associate(est, ref, "0.01")
        np.testing.assert_array_equal(matched.est_indices, [0])
        self.assertEqual(matched.stats["max_abs_dt_s"], 0.01)

    def test_synthetic_association_tie_and_reuse_counts(self):
        est = synthetic_trajectory(np.zeros((2, 3)), [5, 6])
        ref = synthetic_trajectory(np.zeros((3, 3)), [0, 10, 20])
        a = te.associate(est, ref, "0.00000001")
        np.testing.assert_array_equal(a.ref_indices, [0, 1])
        a = te.associate(synthetic_trajectory(np.zeros((2, 3)), [1, 2]), ref, "0.00000001")
        np.testing.assert_array_equal(a.ref_indices, [0, 0])
        self.assertTrue(a.stats["reference_sample_reused"])
        self.assertEqual(a.stats["reference_reuse_count"], 1)
        self.assertEqual(a.stats["unmatched_ref"], 2)
        b = te.associate(ref, synthetic_trajectory(np.zeros((2, 3)), [1, 2]), "0.00000001")
        self.assertTrue(b.stats["estimate_sample_reused"])
        self.assertEqual(b.stats["unmatched_est"], 2)

    def test_synthetic_large_int64_differences_and_offset_range(self):
        est = synthetic_trajectory([[0, 0, 0]], [te.INT64.min])
        ref = synthetic_trajectory([[0, 0, 0]], [te.INT64.max])
        self.assertEqual(te.associate(est, ref).stats["matched_pairs"], 0)
        with self.assertRaisesRegex(ValueError, "int64 range"):
            te.associate(ref, ref, t_offset="0.000000001")
        with self.assertRaisesRegex(ValueError, "whole nanoseconds"):
            te.associate(ref, ref, t_offset="0.0000000001")
        with self.assertRaisesRegex(ValueError, "nonnegative"):
            te.associate(ref, ref, t_max_diff="-1")

    def test_synthetic_statistics_sse_population_std_and_empty(self):
        stats = te.statistics(np.array([1., 2., 3.]), sse=True)
        self.assertEqual(stats["SSE"], 14.)
        self.assertAlmostEqual(stats["rmse"], np.sqrt(14/3))
        self.assertAlmostEqual(stats["std"], np.sqrt(2/3))
        empty = te.statistics(np.array([]))
        self.assertEqual(empty["n"], 0)
        self.assertIsNone(empty["rmse"])

    def test_synthetic_rpe_analytic_constant_velocity_drift(self):
        p = np.column_stack((np.arange(8), np.zeros((8, 2))))
        ref = synthetic_trajectory(p, np.arange(8) * 500_000_000)
        est = synthetic_trajectory(p * 1.1, ref.t_ns)
        for unit, delta, expected, count in (("frames", "1", .1, 7), ("s", "1", .2, 6), ("m", "2", .2, 6)):
            with self.subTest(unit=unit):
                e = synthetic_evaluate(est, ref, align="none", rpe_unit=unit, rpe_delta=delta)
                np.testing.assert_allclose(e.series["none"]["rpe_translation_m"], expected, atol=1e-12)
                np.testing.assert_allclose(e.series["none"]["rpe_rotation_deg"], 0, atol=1e-12)
                self.assertEqual(e.result["metrics"]["none"]["rpe_translation_m"]["n"], count)

    def test_synthetic_rpe_first_endpoint_irregular_time_distance(self):
        stamps = np.array([0, 400_000_000, 1_200_000_000, 2_000_000_000])
        positions = np.array([[0, 0, 0], [0.3, 0, 0], [1.4, 0, 0], [2.2, 0, 0]])
        for unit in ("s", "m"):
            starts, ends = te.rpe_pair_indices(stamps, positions, "1", unit)
            np.testing.assert_array_equal(starts, [0, 1])
            np.testing.assert_array_equal(ends, [2, 2] if unit == "m" else [2, 3])
        for unit, delta in (("frames", "1.5"), ("s", "0"), ("m", "nan"), ("s", "-1")):
            with self.subTest(unit=unit, delta=delta), self.assertRaises(ValueError):
                te.rpe_pair_indices(stamps, positions, delta, unit)

    def test_synthetic_rpe_rigid_invariance_and_sim3_scaling(self):
        ref = synthetic_trajectory(self.points)
        angle = 0.7
        q = np.array([0, 0, np.sin(angle/2), np.cos(angle/2)])
        rotation = te.quaternion_matrices(q[None])[0]
        est = synthetic_trajectory(2. * self.points @ rotation.T + self.translation,
                                   quaternions=np.tile(q, (len(self.points), 1)))
        e = synthetic_evaluate(est, ref, align_both=True, rpe_unit="frames", rotation_ate=True)
        none = synthetic_evaluate(est, ref, align="none", rpe_unit="frames")
        np.testing.assert_allclose(e.series["se3"]["rpe_translation_m"], none.series["none"]["rpe_translation_m"], atol=1e-12)
        np.testing.assert_allclose(e.series["sim3"]["rpe_translation_m"], 0, atol=1e-12)
        np.testing.assert_allclose(e.series["sim3"]["rpe_rotation_deg"], 0, atol=1e-12)
        self.assertLess(e.result["metrics"]["sim3"]["ate_rotation_deg"]["rmse"], 1e-9)
        self.assertAlmostEqual(e.result["metrics"]["sim3"]["rpe_estimate_scale_applied"], .5)

    def test_synthetic_rpe_independent_homogeneous_matrix_oracle(self):
        def matrices(traj):
            poses = np.tile(np.eye(4), (len(traj.t_ns), 1, 1))
            poses[:, :3, :3] = te.quaternion_matrices(traj.quaternions)
            poses[:, :3, 3] = traj.positions
            return poses
        qe = self.rng.normal(size=(40, 4))
        qr = self.rng.normal(size=(40, 4))
        qe /= np.linalg.norm(qe, axis=1)[:, None]
        qr /= np.linalg.norm(qr, axis=1)[:, None]
        est = synthetic_trajectory(self.points, quaternions=qe)
        ref = synthetic_trajectory(self.rng.normal(size=(40, 3)), quaternions=qr)
        e = synthetic_evaluate(est, ref, align="none", rpe_unit="frames", rpe_delta="3")
        pe, pr = matrices(est), matrices(ref)
        expected_t, expected_r = [], []
        for i in range(37):
            dr = np.linalg.inv(pr[i]) @ pr[i+3]
            de = np.linalg.inv(pe[i]) @ pe[i+3]
            error = np.linalg.inv(dr) @ de
            expected_t.append(np.linalg.norm(error[:3, 3]))
            expected_r.append(np.degrees(np.arccos(np.clip((np.trace(error[:3, :3])-1)/2, -1, 1))))
        np.testing.assert_allclose(e.series["none"]["rpe_translation_m"], expected_t, atol=1e-12)
        np.testing.assert_allclose(e.series["none"]["rpe_rotation_deg"], expected_r, atol=1e-10)

    def test_synthetic_rotation_angles_known_and_pi(self):
        angles = np.array([0, 2, 90, 179.999, 180])
        radians = np.radians(angles)
        quats = np.column_stack((np.zeros((len(angles), 2)), np.sin(radians/2), np.cos(radians/2)))
        np.testing.assert_allclose(te.rotation_angles_deg(te.quaternion_matrices(quats)), angles, atol=1e-10)
        ref = synthetic_trajectory(np.zeros((5, 3)))
        est = synthetic_trajectory(np.zeros((5, 3)), quaternions=quats)
        e = synthetic_evaluate(est, ref, align="none", rpe_unit="frames", rotation_ate=True)
        np.testing.assert_allclose(e.series["none"]["rpe_rotation_deg"], np.diff(angles), atol=1e-10)

    def test_synthetic_no_matches_empty_rpe_and_frame_warning(self):
        ref = synthetic_trajectory(self.points)
        with self.assertRaisesRegex(ValueError, "no timestamp matches"):
            synthetic_evaluate(ref, ref, t_offset="100")
        e = te.evaluate(ref, ref, clock_assumption="  SYNTHETIC text preserved  ", est_frame="a", ref_frame="b", rpe_delta="100")
        self.assertEqual(e.result["assumptions"]["clock_assumption"], "  SYNTHETIC text preserved  ")
        self.assertFalse(e.result["assumptions"]["extrinsics_applied"])
        self.assertTrue(e.result["warnings"][0].startswith("WARNING:"))
        self.assertIsNone(e.result["metrics"]["se3"]["rpe_translation_m"]["rmse"])
        self.assertEqual(e.result["trajectory"]["duration_s"], 39)

    def test_synthetic_length_includes_unmatched_reference_samples(self):
        ref = synthetic_trajectory([[0, 0, 0], [0, 1, 0], [1, 0, 0]])
        est = synthetic_trajectory([[0, 0, 0], [1, 0, 0]], [0, 2_000_000_000])
        e = synthetic_evaluate(est, ref, align="none")
        self.assertAlmostEqual(e.result["trajectory"]["reference_length_matched_span_m"], 1 + np.sqrt(2))
        self.assertEqual(e.result["trajectory"]["reference_matched_polyline_length_m"], 1)

    def test_synthetic_cli_required_assumptions(self):
        for missing in ("--clock-assumption", "--est-frame", "--ref-frame"):
            args = self.common_cli("missing.tum", "missing.tum", self.root / "out")
            index = args.index(missing)
            del args[index:index+2]
            code, _, errors = self.cli(args)
            self.assertEqual(code, 2)
            self.assertIn(missing, errors)
            self.assertFalse((self.root / "out").exists())

    def test_synthetic_cli_outputs_provenance_csv_and_overwrite(self):
        est, ref = self.write_tum("est.tum"), self.write_tum("ref.tum")
        out = self.root / "out"
        args = self.common_cli(est, ref, out) + ["--align-both", "--rotation-ate"]
        code, output, errors = self.cli(args)
        self.assertEqual(code, 0, errors)
        self.assertIn("se3", output)
        result = json.loads((out / "result.json").read_text())
        self.assertEqual(result["cli"]["argv"], args)
        self.assertEqual(result["evidence_label"], "CALCULATED")
        self.assertEqual(result["inputs"]["estimate"]["sha256"], hashlib.sha256(est.read_bytes()).hexdigest())
        self.assertEqual(result["plots"]["status"], "disabled")
        self.assertEqual(result["cli"]["args"]["clock_assumption"], "SYNTHETIC shared clock")
        self.assertTrue((out / "ate_errors.csv").read_text().startswith("t_ns,error_m_se3,error_m_sim3"))
        self.assertIn("rotation_error_deg_sim3", (out / "rpe_errors.csv").read_text())
        self.assertEqual(self.cli(args)[0], 2)
        (out / "keep.txt").write_text("SYNTHETIC unrelated existing file")
        self.assertEqual(self.cli(args + ["--overwrite"])[0], 0)
        self.assertEqual((out / "keep.txt").read_text(), "SYNTHETIC unrelated existing file")

    def test_synthetic_cli_frame_warning_printed(self):
        est = self.write_tum("est.tum")
        args = self.common_cli(est, est, self.root / "out")
        args[args.index("--ref-frame") + 1] = "different SYNTHETIC label"
        code, _, errors = self.cli(args)
        self.assertEqual(code, 0)
        self.assertIn("WARNING: frame labels differ", errors)
        self.assertIn("no extrinsics", errors)

    def test_synthetic_batch_per_run_and_unweighted_aggregate(self):
        ref = self.write_tum("ref.tum")
        a = self.write_tum("a.tum", self.points + [0.1, 0, 0])
        b = self.write_tum("b.tum", self.points + [0.3, 0, 0])
        args = self.common_cli(a, ref, self.root / "batch")
        args[args.index("--est") + 1] = f"a={a}"
        args = ["batch"] + args + ["--est", f"b={b}", "--align", "none"]
        code, _, errors = self.cli(args)
        self.assertEqual(code, 0, errors)
        self.assertTrue((self.root / "batch/a/result.json").exists())
        self.assertTrue((self.root / "batch/b/result.json").exists())
        result = json.loads((self.root / "batch/result.json").read_text())
        aggregate = result["aggregate"]["none"]["ate_translation_m.rmse"]
        self.assertAlmostEqual(aggregate["mean"], .2)
        self.assertAlmostEqual(aggregate["min"], .1)
        self.assertAlmostEqual(aggregate["max"], .3)
        self.assertIn("across runs: mean", (self.root / "batch/summary.md").read_text())

    def test_synthetic_batch_bad_name_and_late_input_fail_without_output(self):
        ref = self.write_tum("ref.tum")
        for entries in ([f"../escape={ref}"], [f"ok={ref}", "bad=missing.tum"], [f"ok={ref}", f"ok={ref}"]):
            args = self.common_cli(ref, ref, self.root / "batch")
            del args[:2]
            args = ["batch"] + args + [item for entry in entries for item in ("--est", entry)]
            code, _, _ = self.cli(args)
            self.assertEqual(code, 2)
            self.assertFalse((self.root / "batch").exists())

    def test_synthetic_no_matplotlib_core_and_output_fallback(self):
        original_import = builtins.__import__
        previous_cache = {key: os.environ.get(key) for key in ("MPLCONFIGDIR", "XDG_CACHE_HOME")}
        def without_matplotlib(name, *args, **kwargs):
            if name == "matplotlib" or name.startswith("matplotlib."):
                raise ImportError("SYNTHETIC unavailable matplotlib")
            return original_import(name, *args, **kwargs)
        with patch("builtins.__import__", side_effect=without_matplotlib):
            e = synthetic_evaluate(synthetic_trajectory(self.points), synthetic_trajectory(self.points))
            te.write_evaluation(e, self.root / "out")
        result = json.loads((self.root / "out/result.json").read_text())
        self.assertEqual(result["plots"]["status"], "unavailable")
        self.assertIn("SYNTHETIC unavailable", result["plots"]["reason"])
        self.assertEqual({key: os.environ.get(key) for key in previous_cache}, previous_cache)

    def test_synthetic_output_symlinks_and_input_overlap_rejected(self):
        ref = self.write_tum("ref.tum")
        self.assertEqual(self.cli(self.common_cli(ref, ref, self.root) + ["--overwrite"])[0], 2)
        link = self.root / "linked-output"
        try:
            link.symlink_to(self.root, target_is_directory=True)
        except OSError as error:
            if getattr(error, "winerror", None) == 1314:
                self.skipTest("Windows host cannot create symbolic links")
            raise
        self.assertEqual(self.cli(self.common_cli(ref, ref, link) + ["--overwrite"])[0], 2)
        out = self.root / "out"
        out.mkdir()
        (out / "result.json").symlink_to(ref)
        before = ref.read_bytes()
        self.assertEqual(self.cli(self.common_cli(ref, ref, out) + ["--overwrite"])[0], 2)
        self.assertEqual(ref.read_bytes(), before)


class RealDataValidationTests(unittest.TestCase):
    """REAL-DATA: read-only regression against historical evo summary evidence."""

    def test_real_data_mh01_evo_and_tum_crosscheck(self):
        root = REPO / "experiments/2026-10-13-n4a-euroc-mh01"
        files = [root / "eval/gt_data.csv", root / "eval/ape_summary.txt",
                 root / "eval/run1.tum", root / "eval/gt_data.tum"]
        files += [root / f"run{run}/f_mh01_stereoi.txt" for run in (1, 2, 3)]
        missing = [str(path) for path in files if not path.is_file()]
        if missing:
            self.skipTest("REAL-DATA MH01 fixtures missing: " + ", ".join(missing))
        expected = {}
        run, mode = None, None
        for line in (root / "eval/ape_summary.txt").read_text().splitlines():
            if line.startswith("== run"):
                run = int(line.split()[1][3:])
                mode = "se3"
                expected[run] = {"se3": {}, "sim3": {}}
            elif line.startswith("-- Sim3"):
                mode = "sim3"
            elif line.strip():
                key, value = line.split()
                expected[run][mode][key] = float(value)
        ref = te.read_trajectory(root / "eval/gt_data.csv", "euroc_gt")
        options = dict(clock_assumption="REAL-DATA EuRoC dataset timestamp domain assumed common; zero estimate offset.",
                       est_frame="imu_body (ORB-SLAM3 body)", ref_frame="imu_body (EuRoC state_groundtruth_estimate0)",
                       t_max_diff="0.02", t_offset="0", align_both=True)
        evaluations = {}
        for run in (1, 2, 3):
            with self.subTest(run=run, evidence="REAL-DATA"):
                est = te.read_trajectory(root / f"run{run}/f_mh01_stereoi.txt", "orbslam3")
                e = te.evaluate(est, ref, **options)
                evaluations[run] = e
                self.assertEqual(e.result["association"]["matched_pairs"], 3639)
                for mode, metrics in expected[run].items():
                    for key, target in metrics.items():
                        actual = e.result["metrics"][mode]["ate_translation_m"][key]
                        self.assertLess(abs(actual - target), 5e-5,
                            f"REAL-DATA run{run} {mode} {key}: calculated {actual}, evo {target}; "
                            f"association={e.result['association']}, alignment={e.result['metrics'][mode]['alignment']}")
        tum_est = te.read_trajectory(root / "eval/run1.tum", "tum", "s")
        tum_ref = te.read_trajectory(root / "eval/gt_data.tum", "tum", "s")
        converted = te.evaluate(tum_est, tum_ref, **options)
        for mode in ("se3", "sim3"):
            for key in ("rmse", "mean", "median", "max"):
                self.assertLess(abs(converted.result["metrics"][mode]["ate_translation_m"][key] -
                                    evaluations[1].result["metrics"][mode]["ate_translation_m"][key]), 1e-12,
                                f"REAL-DATA raw ns vs converted TUM discrepancy: {mode} {key}")
            np.testing.assert_allclose(converted.series[mode]["ate_m"], evaluations[1].series[mode]["ate_m"], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
