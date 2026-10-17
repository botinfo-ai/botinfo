"""On a Python older than 3.11, Reins says so in one line and exits non-zero (reins/__init__.py). This file stays
in syntax Python 3.9 can parse, like reins/__init__.py."""
import ast
import os
import shutil
import subprocess
import sys
import unittest
from pathlib import Path

from reins import python_too_old

ROOT = Path(__file__).resolve().parents[1]
LINE_396 = ("Reins needs Python 3.11 or newer; you have 3.9.6. Install a newer Python, e.g. from python.org or with "
            "uv.")


def old_python():
    """An interpreter older than 3.11 to try the check on: $REINS_OLD_PYTHON, else macOS's /usr/bin/python3."""
    for cand in (os.environ.get("REINS_OLD_PYTHON"), "/usr/bin/python3", shutil.which("python3.9")):
        if not cand or not Path(cand).exists():
            continue
        out = subprocess.run([cand, "-c", "import sys; print(sys.version_info[:2] < (3, 11))"],
                             capture_output=True, text=True, timeout=30)
        if out.stdout.strip() == "True":
            return cand
    return None


class PythonVersionCheck(unittest.TestCase):
    def test_message(self):
        self.assertEqual(python_too_old((3, 9, 6, "final", 0)), LINE_396)
        self.assertIn("you have 3.10.12.", python_too_old((3, 10, 12)))
        self.assertIsNone(python_too_old((3, 11, 0)))
        self.assertIsNone(python_too_old(sys.version_info))

    def test_the_check_parses_on_python_3_9(self):
        for f in (ROOT / "reins" / "__init__.py", ROOT / "reins" / "__main__.py", Path(__file__)):
            ast.parse(f.read_text(encoding="utf-8"), filename=str(f), feature_version=(3, 9))

    def test_old_python_gets_one_line_and_a_non_zero_exit(self):
        py = old_python()
        if py is None:
            self.skipTest("no Python older than 3.11 here (set REINS_OLD_PYTHON to one)")
        env = {k: v for k, v in os.environ.items() if not k.startswith("PYTHON")}
        for args in (["-m", "reins", "bench"], ["-m", "reins", "--version"],
                     ["-m", "unittest", "discover", "-s", "tests", "-t", "tests"]):
            out = subprocess.run([py, *args], cwd=ROOT, capture_output=True, text=True, timeout=60, env=env)
            lines = (out.stdout + out.stderr).strip().splitlines()
            self.assertNotEqual(out.returncode, 0, args)
            self.assertEqual(len(lines), 1, (args, lines[:5]))
            self.assertRegex(lines[0], r"^Reins needs Python 3\.11 or newer; you have 3\.\d+\.\d+\. Install a "
                                       r"newer Python, e\.g\. from python\.org or with uv\.$")


if __name__ == "__main__":
    unittest.main()
