import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import deploy
import smoke

class DeployTests(unittest.TestCase):
    def git(self, root, *args):
        return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()

    def fixture_repo(self, root):
        root.mkdir()
        self.git(root, "init", "-q")
        self.git(root, "config", "user.email", "")
        self.git(root, "config", "user.name", "Deploy Test")
        (root / "lab/check.py").parent.mkdir(parents=True)
        (root / "lab/check.py").write_text("print('fixture')\n")
        (root / "tools/link_bench/host").mkdir(parents=True)
        (root / "tools/link_bench/host/sim.c").write_text("host only\n")
        (root / "tools/link_bench/pi").mkdir(parents=True)
        (root / "tools/link_bench/pi/run.py").write_text("pi tool\n")
        self.git(root, "add", "."); self.git(root, "commit", "-qm", "fixture")

    def test_builds_manifest_from_temp_git_fixture_and_excludes_host_bits(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td) / "repo"; self.fixture_repo(root)
            bundle = deploy.make_bundle(root, Path(td) / "out")
            import tarfile
            with tarfile.open(bundle) as tar:
                manifest = json.load(tar.extractfile("MANIFEST.json"))
                names = tar.getnames()
            self.assertEqual(manifest["git_commit"], self.git(root, "rev-parse", "HEAD"))
            self.assertIn("lab/check.py", [x["path"] for x in manifest["files"]])
            self.assertTrue(any(x["path"] == "tools/link_bench/pi/run.py" for x in manifest["files"]))
            self.assertFalse(any("/host/" in x for x in names))
            self.assertTrue(all(len(x["sha256"]) == 64 for x in manifest["files"]))

    def test_dirty_tree_refused_unless_explicitly_allowed(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td) / "repo"; self.fixture_repo(root)
            (root / "lab/check.py").write_text("changed\n")
            with self.assertRaisesRegex(ValueError, "dirty"):
                deploy.make_bundle(root, Path(td) / "out")
            self.assertTrue(deploy.make_bundle(root, Path(td) / "out", allow_dirty=True).is_file())

    def test_manifest_verification_detects_tampered_file(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); (root / "sample").write_text("original")
            (root / "MANIFEST.json").write_text(json.dumps({"git_commit": "abc", "files": [{"path": "sample", "sha256": hashlib.sha256(b"original").hexdigest()}]}))
            self.assertTrue(deploy.verify_tree(root)[0])
            (root / "sample").write_text("tampered")
            valid, errors, _ = deploy.verify_tree(root)
            self.assertFalse(valid); self.assertIn("sha256 mismatch: sample", errors)

    def test_smoke_writes_blocked_result_when_manifest_is_missing_or_unreadable(self):
        for manifest_text in (None, "not json"):
            with self.subTest(manifest=manifest_text), tempfile.TemporaryDirectory() as td:
                root = Path(td) / "bundle"; root.mkdir()
                if manifest_text is not None:
                    (root / "MANIFEST.json").write_text(manifest_text)
                out = Path(td) / "out"
                proc = subprocess.run([sys.executable, str(HERE / "smoke.py"), "--root", str(root), "--outdir", str(out)],
                                      text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                self.assertEqual(proc.returncode, 2, proc.stdout)
                result = json.loads((out / "result.json").read_text())
                self.assertEqual(result["status"], "BLOCKED")
                self.assertEqual(result["criteria"][0]["status"], "BLOCKED")

    def test_smoke_status_precedence_covers_pass_fail_and_blocked(self):
        self.assertEqual(smoke.overall_status([{"status": "PASS"}]), "PASS")
        self.assertEqual(smoke.overall_status([{"status": "BLOCKED"}]), "BLOCKED")
        self.assertEqual(smoke.overall_status([{"status": "BLOCKED"}, {"status": "FAIL"}]), "FAIL")

    def test_bundle_manifest_retains_real_saved_power_usb_fixture(self):
        rel = "lab/power_usb/tests/fixtures/real-partial/raw/provenance.json"
        with tempfile.TemporaryDirectory() as td:
            archive = deploy.make_bundle(deploy.ROOT, Path(td), allow_dirty=True)
            import tarfile
            with tarfile.open(archive) as tar:
                manifest = json.load(tar.extractfile("MANIFEST.json"))
            listed = {item["path"]: item["sha256"] for item in manifest["files"]}
            self.assertIn(rel, listed)
            self.assertEqual(listed[rel], hashlib.sha256((deploy.ROOT / rel).read_bytes()).hexdigest())

    def test_dry_run_prints_fake_host_transport_commands(self):
        with tempfile.TemporaryDirectory() as td:
            proc = subprocess.run([str(HERE / "deploy.sh"), "fake-pi", "--dry-run", "--allow-dirty", "--outdir", td],
                                  text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertEqual(proc.returncode, 0, proc.stdout)
            self.assertIn("scp ", proc.stdout); self.assertIn("ssh fake-pi", proc.stdout)
            self.assertIn("wbr-tools-", proc.stdout)

    def test_remote_rollback_dry_run_uses_custom_deployment_parent(self):
        proc = subprocess.run([str(HERE / "deploy.sh"), "fake-pi", "--rollback", "--dest",
                                "$HOME/custom/releases/current", "--dry-run"],
                              text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertEqual(proc.returncode, 0, proc.stdout)
        self.assertIn("$HOME/custom/releases/current", proc.stdout)
        self.assertNotIn("$HOME/wbr-tools", proc.stdout)

    def test_shell_local_transport_installs_and_verifies_release(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td) / "pi-root"
            proc = subprocess.run([str(HERE / "deploy.sh"), "--local-root", str(root), "--allow-dirty"],
                                  text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertEqual(proc.returncode, 0, proc.stdout)
            current = root / "current"
            self.assertTrue(current.is_symlink())
            self.assertTrue(deploy.verify_tree(current.resolve())[0])

    def test_local_deploy_keeps_previous_and_shell_rollback_swaps_links(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); releases = []
            for commit, body in (("c1", "one"), ("c2", "two")):
                bundle = root / f"bundle-{commit}"; bundle.mkdir(); (bundle / "x").write_text(body)
                (bundle / "MANIFEST.json").write_text(json.dumps({"git_commit": commit, "files": [{"path": "x", "sha256": hashlib.sha256(body.encode()).hexdigest()}]}))
                releases.append(deploy.local_deploy(bundle, root / "pi"))
            pi = root / "pi"
            self.assertEqual(os.readlink(pi / "current"), "c2")
            self.assertEqual(os.readlink(pi / "previous"), "c1")
            proc = subprocess.run([str(HERE / "deploy.sh"), "--rollback", "--local-root", str(pi)],
                                  text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertEqual(proc.returncode, 0, proc.stdout)
            self.assertEqual(Path(proc.stdout.strip()).name, "c1")
            self.assertEqual(os.readlink(pi / "current"), "c1")
            self.assertEqual(os.readlink(pi / "previous"), "c2")

if __name__ == "__main__": unittest.main()
