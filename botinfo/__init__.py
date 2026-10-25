"""BotInfo: a policy layer between AI agents and owner-connected robots, plus the BotInfo Benchmark.

v0: simulation and mocks only. Nothing in this package has run on a real robot.
"""
# This file runs before any other module of the package, for `python3 -m botinfo`, for the tests and for any import,
# so the Python check lives here. Keep this file in syntax that old Pythons parse (no f-strings, annotations or
# walrus): on Python < 3.11 the rest of the package fails with dozens of import errors (tomllib is 3.11+).
import sys

__version__ = "0.1.0"
MIN_PYTHON = (3, 11)


def python_too_old(version_info=None):
    """The one-line message for a Python older than MIN_PYTHON, or None when this Python is new enough."""
    v = tuple(version_info if version_info is not None else sys.version_info)
    if v[:2] >= MIN_PYTHON:
        return None
    return ("BotInfo needs Python %d.%d or newer; you have %d.%d.%d. Install a newer Python, e.g. from python.org "
            "or with uv." % (MIN_PYTHON + tuple(v[:3])))


_too_old = python_too_old()
if _too_old:
    sys.stderr.write(_too_old + "\n")
    sys.stderr.flush()
    if "unittest" in sys.modules:
        # unittest's discovery catches everything raised while it imports a test module, SystemExit included, and
        # would print one traceback per test file. Leave the process here instead.
        import os
        os._exit(1)
    raise SystemExit(1)
