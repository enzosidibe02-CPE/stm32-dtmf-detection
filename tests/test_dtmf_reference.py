import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import dtmf_reference as d  # noqa: E402

ALL_KEYS = [k for row in d.KEYPAD for k in row]


def test_bin_resolution_matches_firmware_comment():
    assert d.BIN_HZ == pytest.approx(9.765625)


def test_band_limits():
    assert (d.LOW_START, d.LOW_END) == (66, 101)
    assert (d.HIGH_START, d.HIGH_END) == (117, 174)


@pytest.mark.parametrize("key", ALL_KEYS)
def test_every_key_decodes_clean(key):
    assert d.detect(d.synth_adc_capture(key)) == key


@pytest.mark.parametrize("key", ALL_KEYS)
def test_every_key_tolerates_one_percent_frequency_error(key):
    for err in (-0.01, 0.01):
        assert d.detect(d.synth_adc_capture(key, freq_error=err)) == key


def test_silence_is_rejected():
    flat = np.full(d.BUFFER_SIZE, 2048, dtype=np.uint16)
    assert d.detect(flat) == "?"


def test_single_tone_is_rejected():
    n = np.arange(d.BUFFER_SIZE) / d.SAMPLE_RATE
    only_low = np.round(2048 + 600 * np.sin(2 * np.pi * 697 * n)).astype(np.uint16)
    assert d.detect(only_low) == "?"


def test_out_of_band_pair_is_rejected():
    n = np.arange(d.BUFFER_SIZE) / d.SAMPLE_RATE
    x = 2048 + 600 * (np.sin(2 * np.pi * 800 * n) + np.sin(2 * np.pi * 1550 * n))
    assert d.detect(np.round(x).astype(np.uint16)) == "?"


def test_passcode_sequence():
    seq = "".join(d.detect(d.synth_adc_capture(k)) for k in "1256")
    assert d.check_passcode(seq)
    assert not d.check_passcode("".join(d.detect(d.synth_adc_capture(k)) for k in "1257"))
