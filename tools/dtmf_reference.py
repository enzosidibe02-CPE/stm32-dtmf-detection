"""Host-side reference model of the DTMF detector in firmware/Core/Src/main.c.

The constants and the decision logic mirror `processFFT()` in the firmware so the
algorithm can be tested without hardware. It is a *model of the algorithm*, not
a measurement of the board: the ADC input here is synthetic.

Differences from the firmware that are worth knowing:
  * numpy's float64 rfft replaces CMSIS-DSP arm_rfft_fast_f32 (float32). Both
    return the unnormalised forward DFT, so magnitudes are comparable.
  * No UART output.
"""
from __future__ import annotations

import numpy as np

# --- constants copied from firmware/Core/Src/main.c --------------------------
BUFFER_SIZE = 2048
SAMPLE_RATE = 20_000
BIN_HZ = SAMPLE_RATE / BUFFER_SIZE          # 9.765625 Hz per bin
TONE_TOLERANCE = 45.0                       # Hz
MAG_THRESHOLD = 60.0                        # raw FFT magnitude

LOW_FREQS = (697.0, 770.0, 852.0, 941.0)
HIGH_FREQS = (1209.0, 1336.0, 1477.0, 1633.0)
KEYPAD = (
    ("1", "2", "3", "A"),
    ("4", "5", "6", "B"),
    ("7", "8", "9", "C"),
    ("*", "0", "#", "D"),
)
KEY_TO_FREQS = {
    KEYPAD[r][c]: (LOW_FREQS[r], HIGH_FREQS[c]) for r in range(4) for c in range(4)
}

# Band limits use C's (uint16_t) truncation of a float, i.e. int().
LOW_START, LOW_END = int(650.0 / BIN_HZ), int(990.0 / BIN_HZ)
HIGH_START, HIGH_END = int(1150.0 / BIN_HZ), int(1700.0 / BIN_HZ)

ADC_BITS = 12
ADC_MAX = (1 << ADC_BITS) - 1


def synth_adc_capture(
    key: str,
    *,
    amplitude_counts: float = 600.0,
    freq_error: float = 0.0,
    noise_rms_counts: float = 0.0,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Synthesise one BUFFER_SIZE capture of a DTMF key as 12-bit ADC counts.

    amplitude_counts: peak amplitude of *each* of the two tones.
    freq_error: fractional error applied to both tones (e.g. 0.01 = +1 %).
    noise_rms_counts: additive white Gaussian noise, in ADC counts.
    """
    rng = rng or np.random.default_rng(0)
    f_low, f_high = (f * (1.0 + freq_error) for f in KEY_TO_FREQS[key])
    n = np.arange(BUFFER_SIZE)
    t = n / SAMPLE_RATE
    x = amplitude_counts * (np.sin(2 * np.pi * f_low * t) + np.sin(2 * np.pi * f_high * t))
    x += (ADC_MAX + 1) / 2          # mid-rail DC bias, as with the BoostXL mic stage
    x += rng.normal(0.0, noise_rms_counts, BUFFER_SIZE) if noise_rms_counts else 0.0
    return np.clip(np.round(x), 0, ADC_MAX).astype(np.uint16)


def spectrum(adc: np.ndarray) -> np.ndarray:
    """DC-removed, unwindowed, unnormalised magnitude spectrum (bin 0 zeroed)."""
    x = adc.astype(np.float64)
    x -= x.mean()
    mag = np.abs(np.fft.rfft(x))[: BUFFER_SIZE // 2]
    mag[0] = 0.0
    return mag


def detect(adc: np.ndarray) -> str:
    """Return the detected keypad character, or '?' (same contract as firmware)."""
    mag = spectrum(adc)
    low_bin = LOW_START + int(np.argmax(mag[LOW_START : LOW_END + 1]))
    high_bin = HIGH_START + int(np.argmax(mag[HIGH_START : HIGH_END + 1]))
    if mag[low_bin] < MAG_THRESHOLD or mag[high_bin] < MAG_THRESHOLD:
        return "?"
    low_hz, high_hz = low_bin * BIN_HZ, high_bin * BIN_HZ
    row = next((i for i, f in enumerate(LOW_FREQS) if abs(low_hz - f) <= TONE_TOLERANCE), -1)
    col = next((i for i, f in enumerate(HIGH_FREQS) if abs(high_hz - f) <= TONE_TOLERANCE), -1)
    return "?" if row < 0 or col < 0 else KEYPAD[row][col]


def check_passcode(keys: str, passcode: str = "1256") -> bool:
    """Model of the firmware's 4-key sequence check."""
    return keys == passcode
