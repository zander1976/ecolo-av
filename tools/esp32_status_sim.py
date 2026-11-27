#!/usr/bin/env python3
"""
A pretend ESP32 with `wifi-status`, on this computer only: its page and status lines.

The server lives in aav_sim_models (`esp32_page.py`), so the virtual car's ESP32 stand-in
(`sim_esp32`) serves the same page with its real numbers (http://127.0.0.1:8766/). This
script runs it alone, with a made-up drive: `python3 tools/esp32_status_sim.py [--port N]`.
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros/sim/aav_sim_models'))
from aav_sim_models.esp32_page import *  # noqa: E402,F401,F403
from aav_sim_models.esp32_page import main  # noqa: E402

if __name__ == '__main__':
    raise SystemExit(main())
