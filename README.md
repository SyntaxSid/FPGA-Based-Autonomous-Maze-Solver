# 🤖 FPGA Autonomous Maze-Solving Robot

### eYantra Robotics Competition 2025 — IIT Guwahati

[![Verilog](https://img.shields.io/badge/HDL-Verilog-blue?style=flat)](https://en.wikipedia.org/wiki/Verilog)
[![FPGA](https://img.shields.io/badge/FPGA-Intel_Cyclone_IV_E-0071C5?style=flat&logo=intel)](https://www.intel.com/content/www/us/en/products/details/fpga/cyclone/iv.html)
[![Board](https://img.shields.io/badge/Board-DE0--Nano-red?style=flat)](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&No=593)
[![Tool](https://img.shields.io/badge/Tool-Quartus_Prime_20.1-0071C5?style=flat&logo=intel)](https://www.intel.com/content/www/us/en/software/programmable/quartus-prime/overview.html)
[![Lines of Verilog](https://img.shields.io/badge/Verilog-~3000_lines-green?style=flat)]()

A fully autonomous maze-solving robot implemented entirely in **synthesizable Verilog** on an **Intel DE0-Nano FPGA (Cyclone IV E — EP4CE22F17C6)**. Built for the **eYantra Robotics Competition (eYRC)** hosted by IIT Bombay, the system navigates an unknown grid maze, detects environmental data at dead-ends (temperature, humidity, soil moisture), and reports findings over Bluetooth — all running on bare FPGA hardware with **no processor, no OS, and no software stack**.

## 🎥 Demo

<p align="center">
  <a href="https://www.youtube.com/watch?v=VIkbek5VtMM">
    <img src="https://img.youtube.com/vi/VIkbek5VtMM/maxresdefault.jpg" alt="Maze Solver Demo — Full Theme Run" width="640"/>
  </a>
  <br/>
  <em>▶ Click to watch full theme run on YouTube</em>
</p>

### Turns & U-Turns
> The robot detects junctions via IR sensors, centers itself, and executes arc turns with P-controlled deceleration. U-turns use point-turn (in-place rotation) after aligning to the corridor center.

https://github.com/user-attachments/assets/a144096b-928d-4b18-bcd0-fbe8050e7a1b

### Straight-Line Navigation
> Outer PD loop keeps the robot centered between walls using 3 ultrasonic sensors. Inner P loop corrects heading drift using encoder feedback — the two loops run simultaneously in hardware.

https://github.com/user-attachments/assets/db3ad4da-7333-4085-bd8e-ad22f17e8c8b

### Crash Recovery
> When the encoder stall detector or sonar slip detector triggers, the FSM reverses 600 ticks, executes a 30° evasion turn, snaps heading to the nearest grid angle, and resumes navigation automatically.

https://github.com/user-attachments/assets/5fba521f-146b-40bc-bd9b-dee6c7dc3dc3

---

## ✨ What Makes This Interesting

Most robotics competition entries use a microcontroller (Arduino, STM32) running C/C++. This project does **everything in hardware**:

- **No CPU, no instruction set, no software.** Every decision — wall-follow, turn, U-turn, crash recovery — is a hardware state machine clocked at 50 MHz.
- **Cascaded closed-loop control** — a PD outer loop (ultrasonic wall-following) feeds a heading setpoint into a P inner loop (encoder-based heading correction), running simultaneously as parallel hardware.
- **Tick-based differential odometry** with cosine/sine lookup tables for X/Y grid tracking — the bot knows its position in block coordinates without GPS or external localization.
- **Multi-layer fault tolerance** — encoder stall detection, sonar slip detection, state timeouts, and crash-recovery FSMs that reverse, rotate, and retry automatically.

---

## 🏗️ System Architecture

```
                        ┌─────────────────────────────────┐
                        │         Top_System.v            │
                        │       (50 MHz Clock Domain)     │
                        └──────────────┬──────────────────┘
                                       │
           ┌───────────────────────────┼───────────────────────────┐
           │                           │                           │
    ╔══════╧══════╗            ╔═══════╧═══════╗           ╔═══════╧═══════╗
    ║  PERCEPTION ║            ║   CONTROL     ║           ║  ACTUATION    ║
    ╚══════╤══════╝            ╚═══════╤═══════╝           ╚═══════╤═══════╝
           │                           │                           │
  ┌────────┼────────┐         ┌────────┼────────┐         ┌────────┼────────┐
  │ 3× HC-SR04      │         │ Outer PD Loop   │         │ Motor PWM      │
  │ Ultrasonic       │────────▶│ (wall-follow)   │         │ Controller     │
  │ (median-filtered)│         │                 │         │ (500 Hz, 4-bit │
  └──────────────────┘         │    heading      │         │  duty cycle)   │
                               │    setpoint     │         └────────────────┘
  ┌──────────────────┐         │        │        │                 ▲
  │ 2× IR Sensors    │         │        ▼        │                 │
  │ (MH Flying Fish) │────────▶│ Heading P Loop  │─────────────────┘
  │ junction detect  │         │ (encoder-based) │
  └──────────────────┘         └─────────────────┘
                                       ▲
  ┌──────────────────┐                 │
  │ 2× Quadrature    │                 │
  │ Encoders (1400   │─────────────────┘
  │ PPR, 100 Hz)     │
  └──────────────────┘

  ┌──────────────────┐         ┌─────────────────┐
  │ Odometry         │         │ Navigation FSM  │
  │ Processor        │◀────────│ (17 states,     │
  │ (heading, X/Y    │────────▶│  840 lines)     │
  │  grid coords)    │         └─────────────────┘
  └──────────────────┘

  ┌──────────────────────────────────────────────┐
  │ IR3 Subsystem (dead-end docking)             │
  │ IR3 sensor → Servo dip → DHT11 read →       │
  │ Soil moisture ADC → Bluetooth TX             │
  └──────────────────────────────────────────────┘
```

### Cascaded Control Loop

The core navigation uses a **two-stage cascaded controller** — the same architecture used in industrial servo drives and flight controllers:

| Loop | Sensor | Output | Rate | Purpose |
|------|--------|--------|------|---------|
| **Outer (PD)** | 3× Ultrasonic | Heading adjustment (±35°) | 20 Hz | Keep centered between walls |
| **Inner (P)** | 2× Encoders | Motor PWM differential | 100 Hz | Hit the heading setpoint precisely |

The outer loop looks at the walls and says *"drift 5° right"*. The inner loop uses encoder feedback to make the motors actually achieve that heading, compensating for friction, motor bias, and wheel slip.

---

## 🧠 Module Breakdown

### Core Navigation Pipeline

| Module | Lines | Description |
|--------|-------|-------------|
| [`test_fsm.v`](test_fsm.v) | 840 | **Core autonomous FSM** — 17-state machine handling forward drive, junction detection, arc turns, point turns, U-turns, dead-end docking, and multi-layer crash recovery. Uses left-wall-following with `Left > Front > Right` priority |
| [`odometry_processor.v`](odometry_processor.v) | 320 | **Differential odometry** — computes heading (0–359°), X/Y grid coordinates (signed block units), velocity (mm/s), and inter-wheel drift. Uses a cosine LUT for heading-to-XY projection. Supports FSM-triggered heading snaps and position rounding |
| [`Heading_Controller.v`](Heading_Controller.v) | 70 | **Inner P loop** — computes shortest-path heading error with 360° wraparound, applies a stepped PWM correction (0/1/2/5/7/10) based on error magnitude. Directly outputs per-wheel speed |
| [`Outer_Loop_PD.v`](Outer_Loop_PD.v) | 136 | **Outer PD loop** — samples 3 ultrasonic sensors at 20 Hz, computes centering error between walls, applies proportional + derivative correction. Handles single-wall, dual-wall, and sensor saturation cases |

### Sensor Drivers

| Module | Lines | Description |
|--------|-------|-------------|
| [`Ultrasonic.v`](Ultrasonic.v) | 280 | **3× HC-SR04 driver** — round-robin sequencing (Left→Front→Right) to prevent acoustic crosstalk. 10µs trigger pulses, echo timing at 50 MHz (2941 cycles/cm). **Median-3 filter** on each channel to kill single-sample spikes |
| [`encoder_processor.v`](encoder_processor.v) | 132 | **Quadrature decoder** — two-stage metastability synchronizer, dual-edge counting (Phase-A), direction detection via A⊕B. Outputs absolute ticks (signed 32-bit), wheel angle (0–359°), 100ms velocity window, and 10ms fast window for the heading controller |
| [`ir_sensor_flying_fish.v`](ir_sensor_flying_fish.v) | 86 | **IR sensor interface** — two-stage synchronizer + 5ms debounce counter. Active-LOW input inverted to active-HIGH output with edge-detect pulse |
| [`adc_controller.v`](adc_controller.v) | 80 | **SPI ADC interface** — drives MCP3204-compatible ADC for soil moisture sensing. 12-bit output with wet/dry threshold classification |

### Actuation & Communication

| Module | Lines | Description |
|--------|-------|-------------|
| [`Motor_controller.v`](Motor_controller.v) | 108 | **PWM motor driver** — 50 MHz → 8 kHz frequency divider → 16-step (4-bit) PWM at 500 Hz. Direction via H-bridge pin mapping (IN1–IN4). Contains both autonomous interface (`motor_controller`) and manual switch test wrapper (`Motor_controller`) |
| [`bt_uart_rx.v`](bt_uart_rx.v) | 164 | **Bluetooth UART receiver** — 115200 baud, 8N1. Two-stage synchronizer, bit-level UART recovery, and a 9-state command parser matching `START-<N>-#` format. Also parses `E` (emergency stop) and `R` (FPGA reset) single-char commands |
| [`ir3_servo_bt_subsystem.v`](ir3_servo_bt_subsystem.v) | 274 | **Dead-end docking subsystem** — orchestrates a timed sequence: servo dips to 90° → waits 1s → triggers DHT11 + Bluetooth data send → waits 2s → servo returns to neutral. Fully handshaked with the main FSM via `ext_start`/`ext_done` |
| [`dht_bluetooth.v`](dht_bluetooth.v) | 335 | **DHT11 + Bluetooth TX** — reads temperature/humidity from DHT11, formats multi-line ASCII messages (`MPIM-<id>-#`, `MM-<id>-<soil>-<adc>-#`, `TH-<id>-<temp>-<hum>-#`), and transmits via UART at 115200 baud. Includes MPI ID counter that wraps at the dead-end count |
| [`dht.v`](dht.v) | ~250 | **DHT11 single-wire driver** — implements the full DHT11 protocol (18ms host pull-low, 80µs response, 40-bit data read with timing-based 0/1 discrimination, checksum verification) |

### Utilities

| Module | Lines | Description |
|--------|-------|-------------|
| [`cos_lut.v`](cos_lut.v) | ~150 | **Cosine ROM** — 360-entry lookup table, values scaled ×1024 (signed 12-bit). Used by odometry for heading→XY projection. Sin is derived as cos(θ−90°) |
| [`seven_seg_decoder.v`](seven_seg_decoder.v) | ~25 | **7-segment BCD decoder** — maps 4-bit value to 7-segment cathode pattern for on-board debug display |
| [`sonar_bt_debug.v`](sonar_bt_debug.v) | ~130 | **Debug module** — streams live ultrasonic readings over Bluetooth UART for real-time debugging during development |

---

## 🗺️ Navigation FSM — How It Solves the Maze

The `test_fsm.v` implements a **left-wall-following** algorithm with these key behaviors:

### State Machine Overview

```
                         ┌──────────┐
                  BT     │  S_IDLE  │
                START───▶│          │
                         └────┬─────┘
                              │
                         ┌────▼─────┐
                         │ SEQ_INIT │ Anchor heading, start hunting
                         └────┬─────┘
                              │
                   ┌──────────▼──────────┐
                   │     SEQ_FWD         │◀──────────────────────────┐
                   │  (Hunt Mode)        │                           │
                   │  Drive straight,    │                           │
                   │  outer loop active  │                           │
                   └──┬────┬────┬───┬────┘                           │
                      │    │    │   │                                │
              Junction│    │    │   │Dead-end                        │
              detected│    │    │   │(front<12cm)                    │
                      │    │    │   │                                │
                 ┌────▼──┐ │  ┌▼───▼────┐                           │
                 │PRE_   │ │  │UTURN_   │                           │
                 │TURN   │ │  │STOP     │                           │
                 │(center│ │  │Sample   │                           │
                 │on jnc)│ │  │sensors  │                           │
                 └──┬────┘ │  └────┬────┘                           │
                    │      │       │                                │
           ┌────────▼──┐   │  ┌────▼─────┐   ┌──────────┐          │
           │ Direction │   │  │UTURN_    │   │IR3_ACTIVE│          │
           │ Decision  │   │  │CENTER   │   │Servo dip │          │
           │L > F > R  │   │  │Align +  │   │DHT + BT  │          │
           └─┬───┬───┬─┘   │  │center   │   └────┬─────┘          │
             │   │   │     │  └────┬────┘        │               │
         Left│Fwd│Right    │       │              │               │
             │   │   │     │  ┌────▼─────┐        │               │
        ┌────▼┐  │ ┌─▼───┐│  │POINT_    │◀───────┘               │
        │ARC  │  │ │ARC  ││  │TURN      │                        │
        │TURN │  │ │TURN ││  │(180° U)  │                        │
        └──┬──┘  │ └──┬──┘│  └────┬─────┘                        │
           │     │    │   │       │                               │
           └─────┴────┴───┴───────┴──────▶ SEQ_DONE ─────────────┘
                                           (re-enter hunt mode)
```

### Key Decisions at Junctions

1. **Junction detection**: IR sensors (Flying Fish) detect when a side wall disappears
2. **Pre-turn centering**: Drive forward ~1000 ticks to center the robot on the junction
3. **Priority**: `Left open → turn left` > `Front open → drive straight` > `Right open → turn right`
4. **Turns**: Arc turns (outer wheel drives, inner stops) for smooth cornering with P-controlled deceleration

### Fault Tolerance

The FSM includes **three independent stall detectors** running in parallel:

| Detector | Trigger | Recovery |
|----------|---------|----------|
| **Encoder stall** | Wheels commanded but no encoder ticks for 0.5s | Reverse 600 ticks → 30° evasion turn → resume |
| **Sonar slip** | Wheels spinning but wall distances unchanged for 1.0s | Same as encoder stall |
| **State timeout** | Any state stuck for >3s | Force transition to bump-reverse |

After a crash recovery, the FSM snaps the heading to the nearest 90° grid angle and resumes hunting — the robot self-corrects from wall collisions without human intervention.

---

## ⚙️ Hardware Stack

| Component | Specification | Purpose |
|-----------|---------------|---------|
| **FPGA** | Intel DE0-Nano (Cyclone IV E — EP4CE22F17C6, 22K LEs) | All processing |
| **Motors** | DC geared motors + quadrature encoders (1400 PPR) | Differential drive |
| **Ultrasonic** | 3× HC-SR04 (left, front, right) | Wall distance sensing |
| **IR Sensors** | 2× MH Flying Fish (left + right) | Junction/wall detection |
| **IR3 Sensor** | 1× MH Flying Fish (forward-facing) | Dead-end object detection |
| **Bluetooth** | HC-05/06 module (115200 baud UART) | Start/stop commands + telemetry |
| **Env. Sensing** | DHT11 (temperature + humidity) | Dead-end data collection |
| **Soil Sensing** | MCP3204 SPI ADC + soil moisture probe | Dead-end data collection |
| **Servo** | Standard hobby servo (PWM 1.0–1.5ms) | IR3 docking arm actuation |
| **Debug** | 8× onboard LEDs (encoder tick display) | Real-time debugging |

### Physical Constants

| Parameter | Value | Used In |
|-----------|-------|---------|
| Wheel base | 94 mm (center-to-center) | Odometry heading calculation |
| Wheel diameter | 44 mm | Ticks-to-distance conversion |
| Encoder PPR | 1400 pulses/revolution | All distance/velocity math |
| Grid block size | 250 mm (25 cm) | Navigation tick targets |
| Ticks per block | 2532 | Forward drive distance |

---

## 📁 Repository Structure

```
maze_solver_final/
│
├── Top_System.v               # Top-level integration — wires everything together
│
├── ── Navigation ──────────────────────────────────────────
├── test_fsm.v                 # 17-state autonomous navigation FSM
├── odometry_processor.v       # Heading + X/Y grid coordinate tracking
├── Heading_Controller.v       # Inner P loop (encoder → motor correction)
├── Outer_Loop_PD.v            # Outer PD loop (ultrasonic → heading adjust)
│
├── ── Sensor Drivers ──────────────────────────────────────
├── Ultrasonic.v               # 3× HC-SR04 round-robin driver + median filter
├── encoder_processor.v        # Quadrature decoder (1400 PPR, 50 MHz)
├── ir_sensor_flying_fish.v    # Digital IR sensor + debounce
├── adc_controller.v           # SPI ADC for soil moisture
│
├── ── Actuation & Comms ───────────────────────────────────
├── Motor_controller.v         # PWM H-bridge driver (500 Hz, 4-bit)
├── bt_uart_rx.v               # Bluetooth UART RX + command parser
├── ir3_servo_bt_subsystem.v   # Dead-end: IR3 → servo → DHT → BT
├── dht_bluetooth.v            # DHT11 + Bluetooth telemetry TX
├── dht.v                      # DHT11 single-wire protocol driver
│
├── ── Utilities ───────────────────────────────────────────
├── cos_lut.v                  # 360-entry cosine ROM (×1024 scale)
├── seven_seg_decoder.v        # 7-segment BCD decoder
├── sonar_bt_debug.v           # Live ultrasonic → Bluetooth streamer
│
├── ── Quartus Project ─────────────────────────────────────
├── Maze_solver_final.qpf      # Quartus project file
├── Maze_solver_final.qsf      # Pin assignments + device settings
└── .gitignore
```

---

## 📐 Tunable Parameters

Key parameters that can be adjusted without changing architecture:

| Parameter | Location | Default | What It Controls |
|-----------|----------|---------|------------------|
| `TICKS_PER_BLOCK` | `test_fsm.v` | 2532 | Encoder ticks per 25cm grid block |
| `MOTOR_SPEED` | `test_fsm.v` | 15 | Base drive speed (0–15 PWM scale) |
| `ARC_ENTRY_TICKS` | `test_fsm.v` | 500 | Pre-turn centering distance |
| `WHEEL_BASE_MM` | `odometry_processor.v` | 94 | Wheel center-to-center distance (mm) |
| `Kp` | `Outer_Loop_PD.v` | 4 | Wall-following proportional gain |
| `Kd` | `Outer_Loop_PD.v` | 6 | Wall-following derivative gain |
| `TARGET_LEFT_CM` | `Outer_Loop_PD.v` | 6 | Desired distance from left wall (cm) |
| `TARGET_RIGHT_CM` | `Outer_Loop_PD.v` | 6 | Desired distance from right wall (cm) |
| `MAX_ADJUST_DEG` | `Outer_Loop_PD.v` | 35 | Max heading correction from outer loop |
| `DEBOUNCE_MS` | `ir_sensor_flying_fish.v` | 5 | IR sensor debounce window (ms) |

---

## 🚀 How to Build & Flash

### Prerequisites
- Intel Quartus Prime Lite Edition (20.1 or later)
- DE0-Nano FPGA board
- USB Blaster cable

### Steps

1. **Clone the repo**
   ```bash
   git clone https://github.com/SyntaxSid/eyantra-maze-solver-fpga.git
   cd eyantra-maze-solver-fpga
   ```

2. **Open in Quartus**
   - Launch Quartus Prime
   - File → Open Project → select `Maze_solver_final.qpf`

3. **Compile**
   - Processing → Start Compilation (`Ctrl+L`)
   - Wait for successful compilation (~2–5 minutes on the Cyclone IV E)

4. **Flash to FPGA**
   - Tools → Programmer
   - Select your USB Blaster
   - Click **Start** to program the DE0-Nano

5. **Run**
   - Connect via Bluetooth terminal at **115200 baud**
   - Send `START-3-#` to begin autonomous navigation (3 = expected dead-end count)
   - Send `E` for emergency stop
   - Send `R` for full FPGA reset

---

## 🔌 Pin Mapping (DE0-Nano)

<details>
<summary>Click to expand full pin assignments</summary>

| Signal | FPGA Pin | Connector |
|--------|----------|-----------|
| `CLOCK_50` | PIN_R8 | Onboard 50 MHz |
| **Ultrasonic** | | |
| `trig_left` | PIN_B12 | GPIO |
| `echo_left` | PIN_D11 | GPIO |
| `trig_front` | PIN_B11 | GPIO |
| `echo_front` | PIN_E10 | GPIO |
| `trig_right` | PIN_E9 | GPIO |
| `echo_right` | PIN_F8 | GPIO |
| **Motors** | | |
| `MD_ENA` | PIN_T15 | GPIO |
| `MD_IN1` | PIN_R11 | GPIO |
| `MD_IN2` | PIN_T11 | GPIO |
| `MD_IN3` | PIN_T12 | GPIO |
| `MD_IN4` | PIN_T13 | GPIO |
| `MD_ENB` | PIN_F13 | GPIO |
| **Encoders** | | |
| `ENC_A_L` | PIN_J16 | GPIO |
| `ENC_B_L` | PIN_M10 | GPIO |
| `ENC_A_R` | PIN_L14 | GPIO |
| `ENC_B_R` | PIN_N15 | GPIO |
| **IR Sensors** | | |
| `ir_in` | PIN_C6 | GPIO (junction right) |
| `ir_in_2` | PIN_D6 | GPIO (junction left) |
| `ir_in_3` | PIN_D12 | GPIO (IR3 forward) |
| **IR3 Subsystem** | | |
| `servo_pwm_3` | PIN_R13 | GPIO |
| `dht_sensor_3` | PIN_A12 | GPIO (bidirectional) |
| `bt_tx_3` | PIN_J14 | GPIO |
| `bt_rx_3` | PIN_J13 | GPIO |
| **LEDs** | | |
| `LED[7:0]` | A15, A13, B13, A11, D1, F3, B1, L3 | Onboard |

</details>

---

## 📊 Bluetooth Telemetry Protocol

The robot reports environmental data over Bluetooth at each dead-end:

```
MPIM-1-#          ← Dead-end marker (MPI ID = 1)
MM-1-M-0842-#    ← Soil moisture (M=moist, D=dry, raw ADC = 842)
TH-1-28-65-#     ← Temperature 28°C, Humidity 65%
```

Navigation events:
```
PT-DIST           ← Pre-turn ended by tick distance
PT-US             ← Pre-turn ended by ultrasonic (wall too close)
END-#             ← Navigation complete signal
```

Commands (phone → robot):
```
START-3-#         ← Start navigation, expect 3 dead-ends
E                 ← Emergency stop
R                 ← Full FPGA reset
```

---

## 🛠️ Design Decisions & Lessons Learned

1. **Why no microcontroller?** — eYRC rules required FPGA-only solutions. But it turned out that pure-hardware control loops have near-zero latency — the heading controller responds within 2 clock cycles (40 ns) vs. milliseconds on an MCU.

2. **Why cascaded control?** — A single PID from ultrasonic→motors oscillated badly because the sonar updates at 20 Hz but encoder ticks arrive at kHz rates. Splitting into outer (slow, sonar) and inner (fast, encoder) loops made each independently stable.

3. **Why median filter on ultrasonics?** — HC-SR04 sensors produce occasional single-sample spikes (from acoustic reflections or crosstalk). A 3-sample median eliminates these without adding the phase lag of a moving average.

4. **Why tick-based odometry instead of time-based?** — At 50 MHz, a time-based approach would need a hardware divider (ticks/time). Tick-based counting with a gate window avoids division entirely and gives consistent results regardless of motor speed.

5. **Why arc turns instead of point turns?** — Point turns (spin in place) caused the wheels to skid on the arena surface, making heading overshoot unpredictable. Arc turns (one wheel drives, other stops) have better traction and more predictable geometry.

---

## 🏆 Competition Context

This project was built for **[eYantra Robotics Competition (eYRC)](https://www.e-yantra.org/)**, a national-level competition organized by **IIT Bombay** under the MHRD. The theme involved:

- Navigating an unknown grid maze autonomously
- Detecting and reporting environmental parameters (temperature, humidity, soil moisture) at designated dead-end nodes
- Bluetooth-based telemetry for real-time data reporting to a monitoring station
- All processing on FPGA — no microcontrollers allowed

---

## 📄 License

This project was developed for educational and competition purposes. Feel free to reference the architecture and techniques for your own FPGA robotics projects.

---

*Built with ~3000 lines of Verilog and a lot of late nights. 🌙*
