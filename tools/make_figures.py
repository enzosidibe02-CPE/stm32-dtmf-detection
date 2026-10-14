"""Generate results/ figures and summary.json from the host-side reference model.

All inputs are synthetic (see dtmf_reference.synth_adc_capture). Run:
    python tools/make_figures.py
"""
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import dtmf_reference as d

OUT = Path(__file__).resolve().parents[1] / "results"
OUT.mkdir(exist_ok=True)
KEYS = [k for row in d.KEYPAD for k in row]
TRIALS = 50
NAVY, BLUE, GREY = "#14213d", "#3a6ea5", "#8d99ae"


def style(ax):
    ax.grid(alpha=0.25)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def fig_spectrum():
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8), sharey=True)
    freqs = np.arange(d.BUFFER_SIZE // 2) * d.BIN_HZ
    for ax, key in zip(axes, ("1", "9")):
        mag = d.spectrum(d.synth_adc_capture(key))
        ax.plot(freqs, mag, color=NAVY, lw=1)
        ax.axvspan(d.LOW_START * d.BIN_HZ, d.LOW_END * d.BIN_HZ, color=BLUE, alpha=0.15, label="low search band")
        ax.axvspan(d.HIGH_START * d.BIN_HZ, d.HIGH_END * d.BIN_HZ, color="#c77d3a", alpha=0.15, label="high search band")
        for f in d.KEY_TO_FREQS[key]:
            ax.axvline(f, color=GREY, ls="--", lw=0.8)
        ax.set_xlim(0, 2200)
        ax.set_title(f"Key '{key}': {d.KEY_TO_FREQS[key][0]:.0f} + {d.KEY_TO_FREQS[key][1]:.0f} Hz")
        ax.set_xlabel("Frequency (Hz)")
        style(ax)
    axes[0].set_ylabel("|X[k]| (unnormalised, ADC counts)")
    axes[0].legend(frameon=False, fontsize=8)
    fig.suptitle("Reference-model spectra, 2048-pt FFT at 20 kHz (synthetic input)", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "spectra_keys_1_and_9.png", dpi=150)
    plt.close(fig)


def sweep(param, values, **fixed):
    rng = np.random.default_rng(42)
    acc = []
    for v in values:
        ok = n = 0
        for key in KEYS:
            for _ in range(TRIALS):
                kw = dict(fixed)
                kw[param] = v
                got = d.detect(d.synth_adc_capture(key, rng=rng, **kw))
                ok += got == key
                n += 1
        acc.append(ok / n)
    return acc


def fig_sweeps():
    noise = [0, 800, 1600, 2400, 3200, 4000, 5000, 6500, 8000]
    ferr = np.round(np.linspace(-0.05, 0.05, 21), 4)
    acc_noise = sweep("noise_rms_counts", noise)
    acc_ferr = sweep("freq_error", ferr)

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    axes[0].plot(noise, np.array(acc_noise) * 100, "o-", color=NAVY)
    axes[0].set_xlabel("Additive noise RMS (ADC counts); each tone amplitude = 600")
    axes[0].set_ylabel("Correct key decisions (%)")
    axes[0].set_title("Robustness to noise")
    axes[1].plot(np.array(ferr) * 100, np.array(acc_ferr) * 100, "o-", color=NAVY)
    axes[1].set_xlabel("Tone frequency error (%) applied to both tones")
    axes[1].set_title("Robustness to tone-generator error")
    for ax in axes:
        ax.set_ylim(-3, 103)
        style(ax)
    fig.suptitle(f"16 keys x {TRIALS} trials per point, synthetic 12-bit input", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "robustness_sweeps.png", dpi=150)
    plt.close(fig)
    return dict(noise_rms=noise, acc_vs_noise=acc_noise, freq_error=ferr.tolist(), acc_vs_freq_error=acc_ferr)


if __name__ == "__main__":
    fig_spectrum()
    summary = fig_sweeps()
    summary["trials_per_point"] = len(KEYS) * TRIALS
    summary["note"] = "Synthetic inputs through a host-side model of the firmware algorithm; not hardware measurements."
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps({k: v for k, v in summary.items() if k != "note"}, indent=1))
