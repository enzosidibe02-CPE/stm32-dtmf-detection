# CPE 3500 — DTMF Audio-Activated Entry System

Real-time DTMF tone detection on an **STM32 Nucleo-L476RG** (80 MHz Cortex-M4F)
with the TI BoostXL-Audio shield. Audio is sampled at 20 kHz via ADC + DMA,
transformed with a 2048-point real FFT (CMSIS-DSP), and matched against a
4-digit passcode that drives an LED access response.

> Course: CPE 3500 — Embedded Digital Signal Processing
> Author: Etienne Enzo Sidibe · Kennesaw State University

## Pipeline
1. **Acquire** — push-button B1 triggers `HAL_ADC_Start_DMA`; TIM6 TRGO clocks the
   ADC at exactly 20 kHz into a 2048-sample half-word buffer.
2. **Transform** — remove DC offset, run `arm_rfft_fast_f32` (2048-pt real FFT),
   compute the magnitude spectrum.
3. **Detect** — peak-search the low (697–941 Hz) and high (1209–1633 Hz) DTMF
   bands, map (row, col) to a keypad character within `TONE_TOLERANCE`.
4. **Validate** — append digits to a running sequence; on 4 digits compare to the
   stored passcode and drive LD2 (solid = granted, blink = denied).

## Key parameters
| Param | Value |
|-------|-------|
| Sample rate | 20 kHz |
| FFT size | 2048 (≈9.77 Hz/bin) |
| Tone tolerance | 45 Hz |
| Magnitude threshold | 60.0 |
| Passcode | `1256` (`PASSCODE_LEN = 4`) |

## Build
STM32CubeIDE project. Requires CMSIS-DSP (`arm_math.h`). The peripheral init
(`MX_*_Init`, `SystemClock_Config`) is CubeMX-generated — regenerate from the
`.ioc` if not present. Application + DSP logic lives in `Core/Src/main.c`.

## Notes
`main.c` here is reconstructed from the project report and contains the complete
DSP/application logic. The full CubeIDE project tree (HAL drivers, startup, .ioc)
should be committed alongside it for a buildable project.
