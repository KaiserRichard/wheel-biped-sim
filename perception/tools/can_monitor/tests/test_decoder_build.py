"""Compile and run the C RobStride decoder test with the host compiler (lets CI unittest discovery run it)."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


class DecoderCTest(unittest.TestCase):
    def test_decoder(self):
        cc = shutil.which("cc") or shutil.which("gcc")
        if cc is None:
            self.skipTest("no host C compiler")
        with tempfile.TemporaryDirectory() as d:
            exe = Path(d) / "test_decoder"
            subprocess.run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", f"-I{ROOT / 'firmware'}",
                            str(HERE / "test_decoder.c"), str(ROOT / "firmware" / "robstride_decode.c"),
                            "-lm", "-o", str(exe)], check=True)
            out = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
            self.assertIn("passed", out.stdout)


if __name__ == "__main__":
    unittest.main()
