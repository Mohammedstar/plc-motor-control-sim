# ⚙️ PLC Motor Control Simulator (Arduino / Ladder Logic)

> **Career Stage:** Instrumentation → Automation Engineer  
> **Author:** Mohammed Satar  
> **Skills demonstrated:** PLC logic, safety interlocks, ISA standards, industrial motor control

---

## 📋 Overview

This project implements **PLC-equivalent ladder logic** for industrial motor control using Arduino/ESP32.  
It simulates a **pump motor starter** used in Oil & Gas facilities — complete with:

- **DOL (Direct-On-Line) & Star-Delta starting** sequences
- **Safety interlock chains** (pressure, temperature, vibration)
- **Emergency stop (E-Stop)** per IEC 60947-5-5
- **Fault detection & auto-restart** logic
- **HMI serial interface** for monitoring & control

This directly mirrors how a **Siemens S7 PLC** or **Allen-Bradley** would control a centrifugal pump motor in a field application.

---

## 🔧 Simulated Control System

```
     ┌─────────────────────────────────────┐
     │         CONTROL PANEL               │
     │  [START]  [STOP]  [E-STOP]  [RESET] │
     └────────────────┬────────────────────┘
                      │ Digital Inputs
     ┌────────────────▼────────────────────┐
     │        ESP32 / Arduino              │
     │  (PLC-equivalent control logic)     │
     │                                     │
     │  Inputs:  Pressure switch           │
     │           Temperature switch        │
     │           Vibration sensor          │
     │           Current transformer       │
     │           Level float switch        │
     │                                     │
     │  Outputs: KM1 (Main contactor)      │
     │           KM2 (Star contactor)      │
     │           KM3 (Delta contactor)     │
     │           Alarm buzzer              │
     │           Status LEDs              │
     └─────────────────────────────────────┘
```

---

## 🔌 Pin Configuration

| Signal | Pin | Direction | Description |
|--------|-----|-----------|-------------|
| START_PB | D2 | INPUT | Start pushbutton (N.O.) |
| STOP_PB | D3 | INPUT | Stop pushbutton (N.C.) |
| ESTOP | D4 | INPUT | Emergency stop (N.C., safety rated) |
| RESET_PB | D5 | INPUT | Fault reset button |
| PRESS_SW | D6 | INPUT | Low pressure switch (N.C.) |
| TEMP_SW | D7 | INPUT | High temperature switch (N.C.) |
| VIBR_SW | D8 | INPUT | High vibration switch (N.C.) |
| CURR_OK | A0 | INPUT | Current transformer (overcurrent) |
| LEVEL_SW | D9 | INPUT | Low level float switch |
| KM1_MAIN | D10 | OUTPUT | Main motor contactor |
| KM2_STAR | D11 | OUTPUT | Star contactor (star-delta) |
| KM3_DELTA| D12 | OUTPUT | Delta contactor (star-delta) |
| ALARM_OUT| D13 | OUTPUT | Alarm buzzer/beacon |

---

## 🎮 Motor Control Logic (Ladder Diagram Equivalent)

```
Rung 1: START PERMISSIVE
──[STOP_PB]──[ESTOP]──[PRESS_OK]──[TEMP_OK]──[VIBR_OK]──[LEVEL_OK]──[NO_FAULT]──
                                                                                   
Rung 2: MOTOR START LATCH
──[START_PB]──────────────────────────────────────────────────( KM1_COIL )──
      │                                                                      
──[KM1_AUX]───────────────────────────────────────────────────────────────

Rung 3: STAR CONTACTOR (first 5 seconds)
──[KM1]──[NOT STAR_TIMER_DONE]──( KM2_STAR )──

Rung 4: DELTA CONTACTOR (after 5 seconds)
──[KM1]──[STAR_TIMER_DONE]──( KM3_DELTA )──

Rung 5: FAULT ALARM
──[ANY_FAULT]──( ALARM_OUT )──
```

---

## 📁 Files

```
plc-motor-control-sim/
├── firmware/
│   ├── motor_control.ino      # Main control logic
│   ├── ladder_logic.h         # Rung/coil abstraction
│   └── pin_config.h           # I/O mapping
├── simulation/
│   └── motor_sim.py           # Python fault injection simulator
├── docs/
│   ├── ladder_diagram.pdf     # Ladder logic drawing
│   └── wiring_schematic.png   # Hardware wiring
└── README.md
```

---

## 🔐 Safety Standards Referenced

- **IEC 61511** – Functional Safety for Process Industry
- **IEC 60947-5-5** – Emergency stop requirements
- **ISA-84** – Safety Instrumented Systems

---

## 📜 License

MIT License — Mohammed Satar, 2024
