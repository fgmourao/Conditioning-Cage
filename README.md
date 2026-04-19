# Conditioning Cage v2.0

Hardware, firmware and desktop interface for classical fear conditioning.  
Arduino DUE stimulus generator controlled by a Python/PyQt5 application over USB.

---

## Origin

This project is a full rewrite of the system described in:

> Amaral Junior PA, Mourao FAG, Moraes MFD (2019).  
> *A Custom Microcontrolled and Wireless-Operated Chamber for Auditory Fear Conditioning.*  
> Frontiers in Neuroscience.  
> https://doi.org/10.3389/fnins.2019.01193

The original v1.0 architecture used an ESP8266 as a Wi-Fi master communicating with the Arduino DUE via SPI, with a browser-based HTML interface. Version 2.0 removes the ESP8266 entirely — a PyQt5 desktop application communicates directly with the DUE over its native USB port.

---

## Architecture

```
Python / PyQt5  (PC)
      │
      │  SerialUSB — native USB port (USB CDC)
      │
Arduino DUE
      ├─ DAC1         →  Sound  (AM sine, 0–20 kHz)
      ├─ Timer4/5     →  DAC clock / AM modulator ISR
      ├─ Timer6       →  Shock clock (10 kHz)
      ├─ Timer7       →  Light square wave
      ├─ Pins 23–37   →  Shock bars (8 outputs, round-robin)
      ├─ Pin 45       →  Light output
      ├─ Pin 46       →  Watchdog fault output
      ├─ Pin 48       →  Hardware ABORT input
      ├─ Pins 50–53   →  Sync outputs (SOUND, LIGHT, SHOCK, MOD)
      └─ Pins 20–21   →  I²C — OLED display (SDA / SCL)
```

The DUE stores the complete trial list in RAM and executes the experiment autonomously after a single `start` command. Python is responsible only for programming parameters, triggering execution, and monitoring status.

---

## Stimuli

Three independent stimuli with individual onset and duration per trial:

### Sound
- Output: DAC1 (12-bit, 0–3.3 V)
- Waveform: AM-modulated sine (`SINE_AM`) or pure sine
- Carrier: 0–20 kHz; modulator: 0–500 Hz
- Adaptive lookup tables (see [Sound generation](#sound-generation))
- AC coupling capacitor (10 µF) recommended in series on DAC1 output

### Light
- Output: pin 45, square wave 50% duty cycle
- Frequency: configurable; `light_freq = 9999` = DC HIGH (constant ON)

### Shock
- Outputs: pins 23, 25, 27, 29, 31, 33, 35, 37 (8 bars)
- Round-robin activation or fixed single bar (calibration)
- Pulse HIGH and LOW times configurable in ms (0.1 ms resolution at 10 kHz clock)

---

## Trial structure

Each trial stores 16 float parameters, transmitted semicolon-separated over serial:

| Field | Description | Unit |
|---|---|---|
| `baseline` | Quiet period before first trial | s |
| `silence` | Inter-trial interval | s |
| `onset_sound` | Sound onset within trial | s |
| `sound_duration` | Sound duration | s |
| `carrier_freq` | DAC carrier frequency; 0 = no sound | Hz |
| `modulator_freq` | AM modulator frequency; 0 = pure sine | Hz |
| `volume` | Sound amplitude | % |
| `waveform_type` | 0 = SINE_AM, 1 = SINE, 2 = SQUARE | — |
| `onset_shock` | Shock onset within trial | s |
| `shock_duration` | Shock duration; 0 = no shock | s |
| `pulse_high` | Shock bar ON time per pulse | ms |
| `pulse_low` | Shock bar OFF time between pulses | ms |
| `onset_light` | Light onset within trial | s |
| `light_duration` | Light duration; 0 = no light | s |
| `light_freq` | Light frequency; 9999 = DC HIGH | Hz |
| `bar_select` | 0 = round-robin, 1–8 = fixed bar | — |

---

## Serial protocol

All communication uses newline-terminated JSON objects:

```
{"cmd":"ping"}
    -> {"ok":true,"msg":"pong","version":"2.0"}

{"cmd":"program","n":N,"data":"f0;f1;..."}
    -> {"ok":true,"msg":"N trials programmed"}

{"cmd":"start"}
    -> {"ok":true,"msg":"started"}

{"cmd":"abort"}
    -> {"ok":true,"msg":"aborted"}

{"cmd":"status"}
    -> {"ok":true,"status":N,"trial":N,"total":N,
        "running":bool,"ready":bool,"fault":bool}
```

Status codes: `0` IDLE · `1` READY · `2` RUNNING · `3` DONE · `4` FAULT · `5` ABORTED

---

## Sync outputs

All sync pins are active HIGH while the corresponding stimulus is active.  
**Use these pins as the authoritative timing reference for external equipment** (electrophysiology, cameras). The `onset_*` fields in seconds carry a worst-case jitter of ~500 µs (one loop iteration); the sync pins do not.

| Pin | Signal | Description |
|---|---|---|
| 50 | SOUND_SYN | HIGH while sound is playing |
| 51 | LIGHT_SYN | HIGH while light is active |
| 52 | SHOCK_SYN | HIGH while shock is being delivered |
| 53 | MOD_SYN | Square wave at `modulator_freq`, phase-locked to AM envelope |

---

## OLED status display

Optional SSD1306 128×32 OLED on I²C (SDA = pin 20, SCL = pin 21, address 0x3C).  
Shows experiment state at each transition:

| State | Display |
|---|---|
| Startup | `Conditioning Cage / v2.0 Ready` |
| Trial onset | `TRIAL X/N` / `CS` · `CS + US` · `US` |
| Done | `Conditioning Cage / Done` |
| Aborted | `Conditioning Cage / Aborted` |

The firmware runs normally if the display is not connected.

---

## Sound generation

v2.0 uses adaptive lookup tables (`Waveforms.h`) to minimise harmonic distortion across the full frequency range:

| Carrier frequency | Table size | DAC update rate |
|---|---|---|
| > 10 000 Hz | 4 samples | up to 80 kHz |
| > 3 000 Hz | 8 samples | up to 80 kHz |
| > 1 000 Hz | 16 samples | up to 48 kHz |
| ≤ 1 000 Hz | 32 samples | up to 32 kHz |

All rates are within the SAM3X8E DAC hardware limit of 1 MHz.  
Buffer RAM cost: 32 × 32 × 2 bytes = 2 KB.

AM modulation uses a 32-sample unipolar envelope table. When `modulator_freq = 0` the carrier is output as a pure sine with no modulation.

---

## DAC signal conditioning

The DAC1 output is 0–3.3 V (unipolar). For audio output:

- Place a **10 µF electrolytic capacitor** in series (positive pole toward DAC1) to block DC and pass the AC signal
- Place a **10 kΩ resistor** between the amplifier input and GND to anchor the input when the DAC is disabled
- Use a **potentiometer (10–50 kΩ)** between the capacitor and the amplifier input for level control
- Connect the cable shield to GND on the DUE side only (single-end grounding prevents ground loops)

The firmware disables DAC1 between sounds (`dacc_disable_channel`) and pre-charges the coupling capacitor to midscale before each trial onset to prevent transients.

---

## Python application

### Requirements

```
pip install pyserial PyQt5
```

Python 3.8 or later.

### Features

- Dark/Light-theme PyQt5 desktop application
- Trial configuration panel: per-stimulus onset, duration, frequency, volume, pulse timing
- Recorded trials table with per-row delete
- Timeline preview: block diagram of all trials on a shared time axis with a real-time progress line during execution
- Event log with timestamps (`YYYY-MM-DD HH:MM:SS`) — exportable via Save Log
- **Protocol menu:** Save Protocol, Load Protocol, Clear All Trials, Save Log
- **Calibration menu:** Sound, Light, Shock — continuous single-trial stimulus stopped by ABORT; shock calibration allows bar selection and configurable pulse timing
- **Help menu:** Info dialog
- Port selector with refresh button
- 4-second timeout warning if DUE does not respond after connection
- Firmware version check on connect (warns if not v2.0)
- Watchdog fault popup with critical alert
- Hardware ABORT button (pin 48) is read inside the trial timing loop and during inter-trial silences — responsive at any point during experiment execution

### Poll strategy

The poll timer (`{"cmd":"status"}` every 1500 ms) is stopped when the experiment starts to prevent USB interrupt collisions with Timer4 on the Native Port. One poll is scheduled at the calculated onset of each trial using `QTimer.singleShot`. A generation counter invalidates scheduled polls that remain pending after an ABORT. The poll timer resumes when the experiment ends or is aborted.

---

## Hardware requirements

| Component | Specification |
|---|---|
| Microcontroller | Arduino DUE |
| USB connection | Native port (second USB connector) |
| Shock bars | 8 outputs — pins 23, 25, 27, 29, 31, 33, 35, 37 |
| Light output | Pin 45 |
| ABORT button | Momentary switch — pin 48 to GND |
| DAC coupling | 10 µF electrolytic capacitor in series on DAC1 |
| Amplifier | Class-D module (e.g. TDA8932); 10–50 kΩ potentiometer at input |
| Speaker | Tweeter (2–20 kHz); 4 Ω or 8 Ω |
| OLED (optional) | SSD1306 128×32, I²C, address 0x3C |

---

## Files

| File | Description |
|---|---|
| `cage_app_dark.py` | Main application — dark theme |
| `Stimuli_PY_DUE.ino` | Arduino DUE firmware |
| `Waveforms.h` | Adaptive DAC lookup tables |
| `image.png` | Logo displayed in the application header |

Arduino library dependencies (install via Arduino IDE Library Manager):
- **DueTimer** (Ivan Seidel)
- **Adafruit GFX Library**
- **Adafruit SSD1306**

---

## Known limitations

- Noise waveforms (white, pink) are not implemented. The lookup-table architecture is periodic; a possible approach is to generate noise buffers in Python (NumPy/SciPy) and send them as custom tables over serial.
- If the USB cable is disconnected during an experiment, the DUE continues executing autonomously. The only way to stop it is the hardware ABORT button (pin 48).

---

## License

Documentation licensed under Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0).

---

## Author

**v2.0 (2026)**  
Flavio Afonso Goncalves Mourao — [mourao.fg@gmail.com](mailto:mourao.fg@gmail.com)  
*CNPq/MCTI/FNDCT Nº 21/2024 — Processo 446467/2024-3*  
*Federal University of Minas Gerais, Brazil*

---

*Last update: April 2026*
