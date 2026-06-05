/*
 * pin_config.h - Motor Control PLC Simulator Pin Definitions
 * Author: Mohammed Satar
 */
#pragma once

// ─── DIGITAL INPUTS ──────────────────────────────────────
#define PIN_START_PB  2   // Start pushbutton (N.O., active LOW)
#define PIN_STOP_PB   3   // Stop pushbutton  (N.C., active LOW)
#define PIN_ESTOP     4   // Emergency stop   (N.C., active LOW)
#define PIN_RESET_PB  5   // Fault reset      (N.O., active LOW)
#define PIN_PRESS_SW  6   // Low pressure switch (N.C.)
#define PIN_TEMP_SW   7   // High temperature switch (N.C.)
#define PIN_VIBR_SW   8   // High vibration switch (N.C.)
#define PIN_LEVEL_SW  9   // Low level float switch (N.C.)

// ─── ANALOG INPUTS ───────────────────────────────────────
#define PIN_CURR_SENSE  A0  // Current transformer ADC input

// ─── DIGITAL OUTPUTS ─────────────────────────────────────
#define PIN_KM1_MAIN   10  // Main motor contactor coil
#define PIN_KM2_STAR   11  // Star contactor coil
#define PIN_KM3_DELTA  12  // Delta contactor coil
#define PIN_ALARM_OUT  13  // Alarm beacon / buzzer

// ─── CALIBRATION ─────────────────────────────────────────
#define CURRENT_FULL_SCALE_A     25.0   // CT full scale (A)
#define MOTOR_RATED_CURRENT_A    12.0   // Motor FLA (A)
