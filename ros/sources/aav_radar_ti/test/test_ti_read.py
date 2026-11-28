"""The TI reader survives a frame it can't parse (TODO 111, 4 Nov weak links)."""

import time

import pytest

try:
    from aav_radar_ti.TI import get_data as TI
except ImportError:  # serial or numpy missing: the tests below skip
    TI = None

pytestmark = pytest.mark.skipif(TI is None, reason='aav_radar_ti.TI needs serial and numpy')

MAGIC = b'\x02\x01\x04\x03\x06\x05\x08\x07'


class FakePort:
    """A serial port that hands out the queued chunks, then nothing."""

    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.is_open = True

    def read_all(self):
        return self.chunks.pop(0) if self.chunks else b''

    def write(self, data):
        pass

    def close(self):
        self.is_open = False


def reader(chunks):
    """Return a TI with no ports opened: only what read() uses."""
    ti = TI.__new__(TI)
    ti.data_port = FakePort(chunks)
    ti.command_port = FakePort([])
    ti._ms_per_frame = 1.0
    ti._get_data__magicWord = MAGIC
    ti._get_data__timeout = 0.5
    ti._get_data__start_time = time.time()
    ti._get_data__buffer_temp = b''
    ti.cfg_mode = False
    ti.is_clearing = False
    return ti


def test_a_short_frame_is_skipped_and_reading_goes_on():
    # Two frames too short to hold their header, each between two magic words (the
    # reader clears its buffer after each frame).
    ti = reader([MAGIC + b'\x00' * 4 + MAGIC, MAGIC + b'\x00' * 2 + MAGIC])
    frames = ti.read()
    assert next(frames) is None
    assert next(frames) is None, 'the reader must survive the first bad frame'
