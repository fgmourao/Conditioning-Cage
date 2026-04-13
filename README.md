# Conditioning Cage v2.0

Control software and firmware for a classical fear-conditioning cage.  
Arduino DUE stimulus generator with a Python/PyQt5 desktop interface.

**Status: active development**

---

## Origin

This project is a full rewrite of the system described in:

> Amaral Junior PA, Mourao FAG, Moraes MFD (2019).  
> *A Custom Microcontrolled and Wireless-Operated Chamber for Auditory Fear Conditioning.*  
> Frontiers in Neuroscience  
> https://doi.org/10.3389/fnins.2019.01193

The original v1.0 architecture used an ESP8266 as a Wi-Fi master controller communicating with the Arduino DUE via SPI bus. The browser-based HTML interface sent parameters over Wi-Fi; the ESP8266 then forwarded timing signals to the DUE via GPIO pins.

Version 2.0 removes the ESP8266 entirely.

---

## What changed from v1.0 to v2.0

### Architecture

| | v1.0 | v2.0 |
|---|---|---|
| Master controller | ESP8266 (Wi-Fi) | Python on PC (USB) |
| Interface | Browser / HTML page | PyQt5 desktop app |
| Communication | Wi-Fi HTTP + SPI | SerialUSB (native port, 115200 baud) |
| Timing authority | ESP8266 os_timer | Arduino DUE (micros(), 500 us resolution) |
| Protocol | GPIO trigger signals | JSON commands over serial |

The DUE now stores the complete trial list in RAM and executes the experiment autonomously after receiving a single `start` command. Python is only responsible for programming parameters and monitoring status.

### Sound generation

v1.0 used a fixed 8-sample carrier table, which produced visible harmonic distortion at frequencies commonly used in fear conditioning (1-5 kHz).

v2.0 introduces adaptive lookup tables in `Waveforms.h`:

| Carrier frequency | Table size | DAC rate |
|---|---|---|
| <= 1000 Hz | 32 samples | 32 kHz |
| <= 3000 Hz | 16 samples | 48 kHz |
| <= 10000 Hz | 8 samples | 80 kHz |
| <= 20000 Hz | 4 samples | 80 kHz |

The 8-sample table is retained from v1.0 but its maximum carrier is capped at 10 kHz (80 kHz DAC rate). Above 10 kHz the firmware drops to 4 samples rather than pushing the DAC beyond its reliable range.

RAM cost of the new buffer: 32 x 32 = 1024 uint16_t = 2 KB, compared to 32 KB for equivalent runtime sinf() tables.

### Stimuli

Three fully independent stimuli with individual onset and duration per trial:

- **SOUND** — DAC1, AM-modulated sine, pure sine, or square wave. Carrier 0-20 kHz, modulator 0-500 Hz.
- **SHOCK** — 8 bar pins, round-robin, configurable pulse HIGH/LOW timing (ms).
- **LED** — pin 45, square wave 50% duty cycle, configurable frequency. `led_freq = 9999` drives the pin permanently HIGH (DC).

### Trial structure

Each trial stores 15 float parameters:

```
baseline, silence, onset_sound, sound_duration, carrier_freq,
modulator_freq, volume, waveform_type, onset_shock, shock_duration,
pulse_high, pulse_low, onset_led, led_duration, led_freq
```

`baseline` is a new field absent in v1.0. It defines a quiet period at the start of the session before the first trial begins, without counting as an inter-trial interval.

### Python interface

- PyQt5 desktop application (replaces the ESP8266 HTML page)
- Trial configuration panel with per-stimulus onset, duration, and parameters
- Recorded trials table with per-row delete
- Timeline infographic: block diagram of all trials on a shared time axis, with a real-time progress line during experiment execution
- Protocol save/load (semicolon-delimited .txt)
- Calibration dialogs for Sound, LED, and Shock (infinite-duration single trial, stopped by ABORT)
- Poll timer paused during experiment to prevent USB interrupt glitch on DAC Timer4 (Native Port)
- 4-second timeout warning if DUE does not respond after connection

### Serial protocol

All communication uses newline-terminated JSON:

```
{"cmd":"ping"}                              -> {"ok":true,"msg":"pong","version":"2.0"}
{"cmd":"program","n":N,"data":"f0;f1;..."}  -> {"ok":true,"msg":"N trials programmed"}
{"cmd":"start"}                             -> {"ok":true,"msg":"started"}
{"cmd":"abort"}                             -> {"ok":true,"msg":"aborted"}
{"cmd":"status"}                            -> {"ok":true,"status":N,"trial":N,...}
```

### Sync outputs

All four sync pins are active HIGH while the corresponding stimulus is active:

| Pin | Signal |
|---|---|
| 50 | SOUND_SYN |
| 51 | LED_SYN |
| 52 | SHOCK_SYN |
| 53 | MOD_SYN (square wave at modulator_freq, phase-locked to AM envelope) |

---

## Hardware requirements

- Arduino DUE
- USB connection to the Native port (second USB connector on the DUE)
- 8 shock bar outputs on pins 23, 25, 27, 29, 31, 33, 35, 37
- LED output on pin 45
- Hardware ABORT button between pin 48 and GND (INPUT_PULLUP, active LOW)
- Recommended: AC coupling capacitor (10 uF) in series on DAC1 output to prevent DC offset at speaker

---

## Software requirements

```
pip install pyserial PyQt5
```

Python 3.8 or later.

---

## Files

| File | Description |
|---|---|
| `cage_app_dark.py` | Main application, dark theme |
| `cage_app_light.py` | Main application, light theme |
| `cage_app.ui` | Qt Designer layout file |
| `Stimuli_PY_DUE.ino` | Arduino DUE firmware |
| `Waveforms.h` | Adaptive DAC lookup tables |
| `DueTimer.h / .cpp` | Timer library (Ivan Seidel) |

---

## Known limitations and possible future directions

- Waveform type selector is not yet exposed in the UI (currently fixed to SINE_AM)
- Noise waveforms (white, pink, brown) are not implemented. The current lookup-table architecture is periodic and does not support stochastic generation. A possible approach: generate noise buffers in Python (numpy/scipy), send as a custom table via serial, and replay in loop on the DUE
- The round-robin shock bar scheme means each individual bar operates at `1000 / (pulse_high + pulse_low)` Hz, while the full 8-bar system cycles at that rate divided by 8
- Theme switching (dark/light) at runtime is not implemented. The two separate files are the recommended approach given that stylesheet values are distributed across individual widget calls

---

## License

Documentation licensed under  
Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)

---

## Authors

**v1.0 (2019)**  
Paulo Aparecido Amaral Junior, Marcio Flavio Dutra Moraes, Flavio Afonso Goncalves Mourao  
Nucleo de Neurociencias, UFMG, Brazil

**v2.0 (2026)**  
Flavio Mourao (mourao.fg@gmail.com)
Federal University of Minas Gerais, Brazil

Development started: December 2023
Last update: March 2026

