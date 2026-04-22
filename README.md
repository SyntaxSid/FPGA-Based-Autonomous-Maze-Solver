# 🤖 FPGA Autonomous Maze-Solving Robot
### eYantra Robotics Competition 2025

[![Verilog](https://img.shields.io/badge/HDL-Verilog-blue?style=flat)](https://en.wikipedia.org/wiki/Verilog)
[![FPGA](https://img.shields.io/badge/FPGA-Intel_Cyclone_IV_E-0071C5?style=flat&logo=intel)](https://www.intel.com/content/www/us/en/products/details/fpga/cyclone/iv.html)
[![Board](https://img.shields.io/badge/Board-DE0--Nano-red?style=flat)](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&No=593)
[![Tool](https://img.shields.io/badge/Tool-Quartus_Prime-0071C5?style=flat&logo=intel)](https://www.intel.com/content/www/us/en/software/programmable/quartus-prime/overview.html)

A fully autonomous maze-solving robot implemented entirely in **synthesizable Verilog** on an **Intel DE0-Nano FPGA (Cyclone IV E — EP4CE22F17C6)**. Developed for the **eYantra Robotics Competition**, the system integrates a cascaded PD/PID control architecture, differential odometry, multi-sensor fusion, and a Bluetooth-triggered FSM — all running on bare FPGA hardware with no processor or OS.

---

## ✨ System Architecture

The design uses a **cascaded closed-loop control** approach:

```
Ultrasonic Sensors → [Outer PD Loop] → Heading Setpoint
Encoders          → [Odometry]       → Current Heading
                                        ↓
                    [Heading Controller (Inner PID)] → Motor PWM
```

### Key Design Decisions
- **No microcontroller or soft-core CPU** — every module is pure RTL running at 50 MHz
- **Cascaded control** separates wall-following (outer loop) from heading correction (inner loop), making each independently tunable
- **Tick-based odometry** instead of time-based, giving displacement accuracy independent of speed variations

---

## 🧠 Module Breakdown

| Module | Description |
|--------|-------------|
| `Top_System.v` | Top-level integration of all subsystems; port-maps sensors, motors, and all sub-modules |
| `test_fsm.v` | Core autonomous FSM — handles forward drive, 90°/180° turns, arc turns, U-turns, crash recovery, and IR3 docking. Triggered via Bluetooth |
| `odometry_processor.v` | Dual-encoder odometry brain — computes heading (0–359°), X/Y grid coordinates, velocity, and inter-wheel drift at 50 MHz |
| `Heading_Controller.v` | Inner PID loop — corrects motor differential to hit target heading using encoder feedback |
| `Outer_Loop_PD.v` | Outer PD loop — reads three ultrasonic sensors and outputs a heading adjustment request |
| `Motor_controller.v` | PWM motor driver — converts speed (0–15) + direction commands into H-bridge signals |
| `encoder_processor.v` | Quadrature decoder — decodes dual-channel encoder signals with direction detection |
| `Ultrasonic.v` | HC-SR04 driver — generates 10µs trigger pulses, measures echo width, outputs distance in cm |
| `bt_uart_rx.v` | Bluetooth UART receiver — parses START/STOP commands to trigger/halt the FSM |
| `ir_sensor_flying_fish.v` | Digital IR sensor interface (MH Flying Fish) for object detection on left/right |
| `ir3_servo_bt_subsystem.v` | IR3 subsystem — coordinates servo sweep, DHT11 sensing, and Bluetooth reporting as a state-machine handshake |
| `dht.v` / `dht_bluetooth.v` | Single-wire DHT11 driver for temperature & humidity sensing with Bluetooth reporting |
| `adc_controller.v` | SPI ADC interface for soil moisture analog sensor |
| `cos_lut.v` | ROM-based cosine lookup table (used in odometry for heading calculations) |
| `seven_seg_decoder.v` | 7-segment display decoder for on-board debug display |
| `sonar_bt_debug.v` | Debug module — streams live ultrasonic readings over Bluetooth UART |

---

## 🗺️ FSM State Machine

The main `test_fsm.v` implements 17 states handling the full autonomous navigation sequence:

| State | Behaviour |
|-------|-----------|
| `S_IDLE` | Waiting for Bluetooth START command |
| `S_SEQ_FWD` | Drive forward one block (tick-counted) |
| `S_SEQ_ARC` | Smooth arc/rounded turn |
| `S_SEQ_POINT_TURN` | Precise 90° or 180° point turn |
| `S_SEQ_UTURN_STOP` | Stop, sample sensors, then U-turn |
| `S_SEQ_BUMP_REVERSE` | Crash recovery — reverse and retry |
| `S_SEQ_IR3_ACTIVE` | Trigger IR3 docking subsystem and wait for completion handshake |
| `S_SEQ_UTURN_CENTER` | Arc to center between walls before U-turn |

---

## ⚙️ Hardware Stack

| Component | Details |
|-----------|---------|
| FPGA Board | Intel DE0-Nano (Cyclone IV E — EP4CE22F17C6) |
| Motors | DC geared motors with quadrature encoders (1400 PPR) |
| Distance Sensors | 3× HC-SR04 Ultrasonic (left, front, right) |
| IR Sensors | MH Flying Fish digital IR (left + right) |
| Communication | HC-05/06 Bluetooth module (UART) |
| Environment | DHT11 temperature & humidity sensor |
| Soil Sensing | SPI ADC (MCP3xxx-compatible) |
| Actuation | Servo motor (IR3 subsystem) |
| Display | On-board DE0-Nano 8 LEDs + 7-segment |
| Wheel Base | 94 mm center-to-center |
| Wheel Diameter | 44 mm |
| Block Size | 250 mm (25 cm grid) |

---

## 📁 Repository Structure

```
maze_solver_final/
├── Top_System.v              # Top-level module
├── test_fsm.v                # Autonomous navigation FSM
├── odometry_processor.v      # Encoder-based odometry
├── Heading_Controller.v      # Inner PID heading loop
├── Outer_Loop_PD.v           # Outer ultrasonic PD loop
├── Motor_controller.v        # PWM H-bridge driver
├── encoder_processor.v       # Quadrature encoder decoder
├── Ultrasonic.v              # HC-SR04 ultrasonic driver
├── bt_uart_rx.v              # Bluetooth UART receiver
├── ir_sensor_flying_fish.v   # IR object detection
├── ir3_servo_bt_subsystem.v  # IR3 docking subsystem
├── dht.v / dht_bluetooth.v   # DHT11 sensor driver
├── adc_controller.v          # SPI ADC interface
├── cos_lut.v                 # Cosine ROM LUT
├── seven_seg_decoder.v       # 7-segment decoder
├── sonar_bt_debug.v          # Bluetooth debug streamer
├── Maze_solver_final.qpf     # Quartus project file
└── Maze_solver_final.qsf     # Quartus settings & pin assignments
```

---

## 🚀 How to Build & Flash

### Prerequisites
- Intel Quartus Prime (Lite Edition is sufficient)
- DE0-Nano FPGA board
- USB Blaster cable

### Steps

1. **Clone the repo**
   ```bash
   git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git
   cd YOUR_REPO_NAME
   ```

2. **Open in Quartus**
   - Launch Quartus Prime
   - File → Open Project → select `Maze_solver_final.qpf`

3. **Compile**
   - Processing → Start Compilation (or press `Ctrl+L`)
   - Wait for successful compilation (~2–5 minutes)

4. **Flash to FPGA**
   - Tools → Programmer
   - Select your USB Blaster
   - Click **Start** to program the DE0-Nano

5. **Run**
   - Send `START` over Bluetooth to begin autonomous navigation
   - Send `STOP` to halt at any time

---

## 📐 Tunable Parameters

Key parameters that can be adjusted without changing the logic:

| Parameter | Location | Default | Description |
|-----------|----------|---------|-------------|
| `TICKS_PER_BLOCK` | `test_fsm.v` | 2532 | Encoder ticks per 25cm grid block |
| `MOTOR_SPEED` | `test_fsm.v` | 15 | Base drive speed (0–15 scale) |
| `WHEEL_BASE_MM` | `odometry_processor.v` | 94 | Wheel base in mm |
| `Kp`, `Kd` | `Outer_Loop_PD.v` | 5, 1 | Wall-following PD gains |
| `KP_HEADING` | `Heading_Controller.v` | 1 | Heading correction proportional gain |
| `TARGET_LEFT_CM` | `Outer_Loop_PD.v` | 6 | Desired distance from left wall (cm) |
| `TARGET_RIGHT_CM` | `Outer_Loop_PD.v` | 6 | Desired distance from right wall (cm) |
