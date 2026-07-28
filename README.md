# 🏎️ MicroPython High-Speed Line Follower Robot (LFR) Firmware

A modular, deterministic, high-frequency MicroPython firmware designed for 16-channel line follower competition robots powered by the **Raspberry Pi Pico (RP2040)**.

This project is a 1:1 source-to-source translation of an optimized Arduino C++ firmware, adapted for native MicroPython standard hardware APIs (`machine`, `time`, `neopixel`).

---

## 📌 Features

- **16-Channel IR Reflectance Array**: Interfaced via a **CD74HC4067** 16-channel analog multiplexer.
- **Fixed-Point PID Control & Gain Scheduling**: Real-time adaptive PID gains tailored for straights, mild curves, sharp turns, and 90° corners.
- **TB6612FNG Dual H-Bridge Driver**: Motor hardware management featuring dead-zone compensation, motor speed balancing, acceleration slew-rate limiting, and PWM saturation control.
- **Speed Planning & Track Classification**: Real-time detection of track features (Straights, Curves, 90° Turns, T-Junctions, Crossings, and Line Gaps).
- **Non-Blocking State Machine**: Built-in debounced multi-button inputs (Calibration & Start), status LED animations via WS2812B / NeoPixel, and countdown timers.
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

## 🔌 Hardware Pinout (RP2040 / Raspberry Pi Pico)

### 1. CD74HC4067 Multiplexer & IR Array
| Pico Pin | Function | Description |
| :--- | :--- | :--- |
| `GPIO 0` | `MUX_S0` | MUX Address Bit 0 |
| `GPIO 1` | `MUX_S1` | MUX Address Bit 1 |
| `GPIO 2` | `MUX_S2` | MUX Address Bit 2 |
| `GPIO 3` | `MUX_S3` | MUX Address Bit 3 |
| `GPIO 26` (ADC0) | `MUX_SIG` | Analog Signal Input |

### 2. TB6612FNG Motor Driver
| Pico Pin | Function | Description |
| :--- | :--- | :--- |
| `GPIO 6` | `AIN1` | Left Motor Direction 1 |
| `GPIO 7` | `AIN2` | Left Motor Direction 2 |
| `GPIO 8` | `PWMA` | Left Motor Speed (PWM) |
| `GPIO 9` | `BIN1` | Right Motor Direction 1 |
| `GPIO 10` | `BIN2` | Right Motor Direction 2 |
| `GPIO 11` | `PWMB` | Right Motor Speed (PWM) |
| `GPIO 12` | `STBY` | Driver Standby Enable (HIGH) |

### 3. User Interface & Auxiliary
| Pico Pin | Function | Description |
| :--- | :--- | :--- |
| `GPIO 13` | `CALIB_BTN` | Calibration Push Button (Internal Pull-Up) |
| `GPIO 14` | `START_BTN` | Start / Stop Push Button (Internal Pull-Up) |
| `GPIO 15` | `STATUS_LED` | WS2812B / NeoPixel Signal Pin |
| `GPIO 27` (ADC1) | `BATTERY_SENSE` | Optional Battery Voltage Divider |

---

## 🚀 Getting Started

### Prerequisites
1. **Raspberry Pi Pico (RP2040)** flashed with the latest [MicroPython firmware v1.19+](https://micropython.org/download/rp2-pico/).
2. A Python IDE supporting MicroPython upload (e.g., [Thonny IDE](https://thonny.org/) or VS Code with MicroPico extension).

### Installation
1. Download or clone this repository.
2. Upload all `.py` files (`config.py`, `sensors.py`, `control.py`, `motor.py`, `telemetry.py`, `main.py`) directly to the root folder (`/`) of your Raspberry Pi Pico.
3. Power on or reset the board. `main.py` will automatically run on boot.

---

## 🎮 Operating Modes & User Interface

The robot operates as a state machine visually indicated by the WS2812B Status LED:

| Robot Mode | LED Pattern | Action / Trigger |
| :--- | :--- | :--- |
| **Booting** | White Pulse | Powering on / Initializing peripherals. |
| **Idle** | Blue Pulse | Standby mode. Waiting for user input. |
| **Calibration** | Yellow Fast Blink | Short-press **Calibration Button** (`GPIO 13`). Move sensors over line/surface during the 5-second window. |
| **Ready** | Green Slow Blink | Calibration complete. Press **Start Button** (`GPIO 14`) to begin run. |
| **Running** | Solid Green | 3-second visual countdown finishes, PID active, and motors driving. Press **Start Button** to stop. |
| **Recovery** | Magenta Fast Blink | Active line loss recovery maneuver. |
| **Error / E-Stop** | Red Blink / Solid Red | Hardware fault or manual emergency stop triggered. |

---

## 💻 Serial CLI Commands

Connect to the Pico via Serial/UART at **115200 Baud** using Thonny, PuTTY, or Serial Monitor. Type `help` to see available live telemetry commands:

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