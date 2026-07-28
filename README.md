# 🏎️ MicroPython High-Speed Line Follower Robot (LFR) Firmware

A modular, deterministic, high-frequency MicroPython firmware designed for 16-channel line follower competition robots powered by the **Raspberry Pi Pico (RP2040)**.

This project is a 1:1 source-to-source translation of an optimized Arduino C++ firmware, adapted for native MicroPython standard hardware APIs (`machine`, `time`, `neopixel`).

---

## 📌 Features

- **16-Channel IR Reflectance Array**: Interfaced via a **CD74HC4067** 16-channel analog multiplexer.
- **Fixed-Point PID Control & Gain Scheduling**: Real-time adaptive PID gains tailored for straights, mild curves, sharp turns, and 90° corners.
- **TB6612FNG Dual H-Bridge Driver**: Motor hardware management featuring dead-zone compensation, motor speed balancing, acceleration slew-rate limiting, and PWM saturation control.
- **Speed Planning & Track Classification**: Real-time detection of track features (Straights, Curves, 90° Turns, T-Junctions, Crossings, and Line Gaps).
- **Non-Blocking State Machine**: Single multi-function button UI (short-press for Calibration/Start/Stop, long-press actions) with status LED animations via WS2812B / NeoPixel and non-blocking countdown timers.
- **Interactive Serial CLI Telemetry**: Zero-overhead passive telemetry with live command-line diagnostics and runtime statistics monitoring over USB UART.

---

## 📂 Project Architecture

The project maintains strict modular separation across six MicroPython modules:

| Module | Filename | Responsibility |
| :--- | :--- | :--- |
| **Config** | `config.py` | Pin assignments, hardware constants, speed profiles, PID gains, and global state variables. |
| **Sensors** | `sensors.py` | MUX channel address selection, 14-bit ADC scaling, sensor calibration, normalization (0–1000), and EMA filtering. |
| **Control** | `control.py` | Weighted-average position calculation, PID computation, track segment classification, curve severity estimation, and motor speed planning. |
| **Motor** | `motor.py` | Hardware PWM output to TB6612FNG, motor balancing, dead-zone handling, acceleration slew limiting, and braking/emergency stops. |
| **Telemetry** | `telemetry.py` | Loop timing tracking, runtime metrics, diagnostic printing, and interactive Serial CLI parser. |
| **Main** | `main.py` | System boot sequence, top-level state machine, button debouncing, status LED animations, and main loop orchestration. |

---

## 🔌 Updated Hardware Pinout (RP2040 / Raspberry Pi Pico)

### 1. CD74HC4067 Multiplexer & IR Array
| Pico Pin | GPIO Pin | Function | Description |
| :--- | :--- | :--- | :--- |
| **Pin 1** | `GPIO 0` | `MUX_S0` | MUX Address Bit 0 |
| **Pin 2** | `GPIO 1` | `MUX_S1` | MUX Address Bit 1 |
| **Pin 4** | `GPIO 2` | `MUX_S2` | MUX Address Bit 2 |
| **Pin 5** | `GPIO 3` | `MUX_S3` | MUX Address Bit 3 |
| **Pin 31** | `GPIO 26 / ADC0` | `MUX_SIG` | Analog Signal Input |
| — | — | `VCC` | Power supply rail |
| — | — | `GND` | Common Ground |

### 2. TB6612FNG Dual H-Bridge Motor Driver
| Pico Pin | GPIO Pin | Function | Description |
| :--- | :--- | :--- | :--- |
| **Pin 9** | `GPIO 6` | `AIN1` | Left Motor Direction 1 |
| **Pin 10** | `GPIO 7` | `AIN2` | Left Motor Direction 2 |
| **Pin 11** | `GPIO 8` | `PWMA` | Left Motor Speed (PWM) |
| **Pin 12** | `GPIO 9` | `BIN1` | Right Motor Direction 1 |
| **Pin 14** | `GPIO 10` | `BIN2` | Right Motor Direction 2 |
| **Pin 15** | `GPIO 11` | `PWMB` | Right Motor Speed (PWM) |
| **Pin 16** | `GPIO 12` | `STBY` | Driver Standby Enable (Active HIGH) |
| — | — | `VM` | Direct 2S LiPo power supply (~7.4V–8.4V) |
| — | — | `VCC` | Logic power supply (3.3V from Pico) |

### 3. User Interface & Status Indicator
| Pico Pin | GPIO Pin | Function | Circuit Logic / Description |
| :--- | :--- | :--- | :--- |
| **Pin 17** | `GPIO 13` | `SYSTEM_BTN` | Multi-Function Push Button (Connect between GPIO 13 & GND, internal pull-up enabled). |
| **Pin 20** | `GPIO 15` | `STATUS_LED` | WS2812B NeoPixel Signal Line (Include inline ~330Ω protection resistor). |

*(Note: The OLED Display on GPIO 4/5 and secondary push button on GPIO 14 have been removed from the circuit layout.)*

---

## 🚀 Getting Started

### Prerequisites
1. **Raspberry Pi Pico (RP2040)** flashed with the latest [MicroPython firmware v1.19+](https://micropython.org/download/rp2-pico/).
2. A Python IDE supporting MicroPython upload (e.g., [Thonny IDE](https://thonny.org/) or VS Code with MicroPico extension).

### Installation
1. Download or clone this repository.
2. Upload all `.py` files (`config.py`, `sensors.py`, `control.py`, `motor.py`, `telemetry.py`, `main.py`) directly to the root directory (`/`) of your Raspberry Pi Pico.
3. Power on or reset the board. `main.py` will automatically run on boot.

---

## 🎮 Operating Modes & User Interface

The robot operates as a non-blocking state machine visually indicated by the WS2812B Status LED:

| Robot Mode | LED Pattern | Action / Trigger |
| :--- | :--- | :--- |
| **Booting** | White Pulse | Powering on / Initializing system peripherals. |
| **Idle** | Blue Pulse | Standby mode. Waiting for user interaction. |
| **Calibration** | Yellow Fast Blink | Short-press **System Button** (`GPIO 13`). Sweep sensors across black/white surfaces during the 5-second window. |
| **Ready** | Green Slow Blink | Calibration complete. Press **System Button** (`GPIO 13`) to prime the run. |
| **Running** | Solid Green | 3-second visual countdown finishes, PID loop activates, and motors drive. Press **System Button** to stop. |
| **Recovery** | Magenta Fast Blink | Active line loss recovery maneuver. |
| **Error / E-Stop** | Red Blink / Solid Red | Hardware fault or manual emergency stop triggered. |

---

## 💻 Serial CLI Commands

Connect to the Pico via Serial/UART at **115200 Baud** using Thonny, PuTTY, or Serial Monitor. Type `help` to display available live telemetry commands:

```text
=== LFR Telemetry Commands ===
  raw         - print raw ADC values
  norm        - print normalized values
  filter      - print filtered values
  calib       - print min/max calibration bounds
  pid         - print PID terms
  motors      - print motor PWM outputs
  track       - print track classification
  state       - print robot operational state
  loop        - print loop timing statistics
  memory      - print heap memory usage
  perf        - print maximum performance metrics
  all         - print all telemetry parameters
  off / stop  - disable periodic telemetry output
  start       - enable telemetry output
  clear       - clear running statistics
  status      - print telemetry system status