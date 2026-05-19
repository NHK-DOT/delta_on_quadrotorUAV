#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run the standalone Python 3.6 real-machine controller with 8BitDo evdev input."""

from __future__ import print_function

import sys
from pathlib import Path


SRC_DIR = Path(__file__).resolve().parent


def main(argv=None):
    sys.path.insert(0, str(SRC_DIR))
    try:
        import gamepad_controller
    except Exception as exc:
        print("Failed to import standalone controller: %s" % exc)
        return 2

    gamepad_controller.main(sys.argv[1:] if argv is None else argv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
