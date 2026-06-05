/*
 * =========================================================
 * PLC MOTOR CONTROL SIMULATOR
 * =========================================================
 * Author:   Mohammed Satar | Instrumentation & Automation
 * Version:  2.0
 * Hardware: Arduino Mega 2560 / ESP32
 *
 * Description:
 *   Implements full industrial motor control sequence with:
 *   - Direct-On-Line (DOL) start
 *   - Star-Delta transition (5-second timer)
 *   - Safety interlock chain (ISA-84 / IEC 61511)
 *   - Emergency stop (IEC 60947-5-5)
 *   - Auto-fault detection and alarm management
 *   - HMI serial interface (JSON status output)
 *
 * This replicates the logic used in Siemens S7-300/400 PLC
 * programs for pump motor control in Oil & Gas facilities.
 * =========================================================
 */

#include <Arduino.h>
#include "pin_config.h"
#include "ladder_logic.h"

// ─── Motor State Machine ─────────────────────────────────
enum MotorState {
  STATE_IDLE,           // Motor stopped, ready
  STATE_STAR,           // Star-Delta: star phase (0-5s)
  STATE_TRANSITION,     // Star-Delta: transition pause (50ms)
  STATE_DELTA,          // Running in delta (normal operation)
  STATE_STOPPING,       // Controlled ramp-down
  STATE_FAULT,          // Fault condition - requires reset
  STATE_ESTOP           // Emergency stop - requires manual reset
};

// ─── Fault Register (bit-mapped) ─────────────────────────
struct FaultRegister {
  bool lowPressure    : 1;
  bool highTemp       : 1;
  bool highVibration  : 1;
  bool overcurrent    : 1;
  bool lowLevel       : 1;
  bool contactorFault : 1;   // KM2 & KM3 both ON = fault
  bool reserved       : 2;
};

// ─── System Variables ────────────────────────────────────
MotorState   motorState  = STATE_IDLE;
FaultRegister faults     = {0};
bool         eStopActive = false;

unsigned long stateTimer       = 0;  // State entry timestamp
unsigned long lastHMIUpdate    = 0;
unsigned long lastSafetyCheck  = 0;
unsigned long runTimeMs        = 0;  // Total motor run time

const unsigned long STAR_DELTA_TIME_MS  = 5000;   // 5s star phase
const unsigned long TRANSITION_PAUSE_MS = 50;     // 50ms gap
const unsigned long HMI_INTERVAL_MS     = 500;    // HMI update rate
const unsigned long SAFETY_INTERVAL_MS  = 100;    // Safety scan rate

// ─── Output State ────────────────────────────────────────
bool km1Active = false;  // Main contactor
bool km2Active = false;  // Star contactor
bool km3Active = false;  // Delta contactor
bool alarmActive = false;

// ─────────────────────────────────────────────────────────
// SAFETY INTERLOCK CHAIN
// Returns TRUE if all permissives are satisfied
// (equivalent to PLC safety rung)
// ─────────────────────────────────────────────────────────
bool checkPermissives() {
  bool pressOK = digitalRead(PIN_PRESS_SW) == HIGH;  // N.C. = HIGH when OK
  bool tempOK  = digitalRead(PIN_TEMP_SW)  == HIGH;
  bool vibrOK  = digitalRead(PIN_VIBR_SW)  == HIGH;
  bool levelOK = digitalRead(PIN_LEVEL_SW) == HIGH;

  // Read current transformer via ADC
  int rawCurrent = analogRead(PIN_CURR_SENSE);
  float current_A = (rawCurrent / 1023.0) * CURRENT_FULL_SCALE_A;
  bool currentOK = (current_A < MOTOR_RATED_CURRENT_A * 1.2); // 120% OC trip

  faults.lowPressure   = !pressOK;
  faults.highTemp      = !tempOK;
  faults.highVibration = !vibrOK;
  faults.lowLevel      = !levelOK;
  faults.overcurrent   = !currentOK;
  
  return (pressOK && tempOK && vibrOK && levelOK && currentOK);
}

// ─────────────────────────────────────────────────────────
// READ PUSHBUTTONS (with debounce)
// ─────────────────────────────────────────────────────────
bool pbStart, pbStop, pbEStop, pbReset;

void readInputs() {
  static unsigned long lastDebounce = 0;
  if (millis() - lastDebounce < 20) return; // 20ms debounce
  lastDebounce = millis();

  pbStart = (digitalRead(PIN_START_PB) == LOW);  // N.O., active LOW
  pbStop  = (digitalRead(PIN_STOP_PB)  == LOW);  // N.C., active LOW = pressed
  pbEStop = (digitalRead(PIN_ESTOP)    == LOW);  // N.C., active LOW = ESTOP!
  pbReset = (digitalRead(PIN_RESET_PB) == LOW);
}

// ─────────────────────────────────────────────────────────
// OUTPUT CONTROL (enforce contactor interlocking)
// Prevents KM2 and KM3 from being ON simultaneously
// (would cause short circuit — electrical interlock)
// ─────────────────────────────────────────────────────────
void writeOutputs() {
  // SAFETY: Never allow KM2 and KM3 simultaneously
  if (km2Active && km3Active) {
    // Contactor fault - immediate trip
    km1Active = km2Active = km3Active = false;
    faults.contactorFault = true;
    motorState = STATE_FAULT;
  }

  digitalWrite(PIN_KM1_MAIN,  km1Active  ? HIGH : LOW);
  digitalWrite(PIN_KM2_STAR,  km2Active  ? HIGH : LOW);
  digitalWrite(PIN_KM3_DELTA, km3Active  ? HIGH : LOW);
  digitalWrite(PIN_ALARM_OUT, alarmActive ? HIGH : LOW);
}

// ─────────────────────────────────────────────────────────
// STATE MACHINE: Main motor control logic
// ─────────────────────────────────────────────────────────
void runStateMachine() {
  unsigned long now = millis();
  bool permissivesOK = checkPermissives();
  bool anyFault = (*(uint8_t*)&faults) != 0;

  switch (motorState) {
    
    // ══ IDLE ════════════════════════════════════════════
    case STATE_IDLE:
      km1Active = km2Active = km3Active = false;
      alarmActive = false;
      
      if (pbEStop) {
        motorState = STATE_ESTOP;
      } else if (pbStart && permissivesOK && !anyFault) {
        // Start sequence: engage main + star contactors
        km1Active = true;
        km2Active = true;
        km3Active = false;
        stateTimer = now;
        motorState = STATE_STAR;
        Serial.println("{\"event\":\"MOTOR_START\",\"phase\":\"STAR\"}");
      } else if (pbStart && !permissivesOK) {
        alarmActive = true;  // Blink alarm - permissive not met
      }
      break;

    // ══ STAR PHASE (0 to STAR_DELTA_TIME_MS) ════════════
    case STATE_STAR:
      km1Active = true;
      km2Active = true;
      km3Active = false;
      
      if (pbEStop || pbStop || !permissivesOK) {
        motorState = pbEStop ? STATE_ESTOP : STATE_FAULT;
        break;
      }
      
      if (now - stateTimer >= STAR_DELTA_TIME_MS) {
        // Timer complete: switch to transition
        km2Active = false;  // Drop star contactor first
        stateTimer = now;
        motorState = STATE_TRANSITION;
        Serial.println("{\"event\":\"STAR_DELTA_TRANSITION\"}");
      }
      break;

    // ══ TRANSITION PAUSE (50ms gap) ══════════════════════
    case STATE_TRANSITION:
      km1Active = true;
      km2Active = false;
      km3Active = false;
      
      if (now - stateTimer >= TRANSITION_PAUSE_MS) {
        km3Active = true;  // Engage delta contactor
        stateTimer = now;
        runTimeMs  = 0;
        motorState = STATE_DELTA;
        Serial.println("{\"event\":\"RUNNING_DELTA\"}");
      }
      break;

    // ══ DELTA (normal running) ════════════════════════════
    case STATE_DELTA:
      km1Active = true;
      km2Active = false;
      km3Active = true;
      runTimeMs = now - stateTimer;
      
      if (pbEStop) {
        motorState = STATE_ESTOP;
      } else if (pbStop || !permissivesOK) {
        km1Active = km3Active = false;
        motorState = anyFault ? STATE_FAULT : STATE_IDLE;
        Serial.println("{\"event\":\"MOTOR_STOP\"}");
      }
      break;

    // ══ FAULT STATE ═══════════════════════════════════════
    case STATE_FAULT:
      km1Active = km2Active = km3Active = false;
      alarmActive = true;
      
      if (pbEStop) {
        motorState = STATE_ESTOP;
      } else if (pbReset && !anyFault) {
        // Manual reset after faults cleared
        memset(&faults, 0, sizeof(faults));
        alarmActive = false;
        motorState = STATE_IDLE;
        Serial.println("{\"event\":\"FAULT_RESET\"}");
      }
      break;

    // ══ EMERGENCY STOP ════════════════════════════════════
    case STATE_ESTOP:
      km1Active = km2Active = km3Active = false;
      alarmActive = true;
      eStopActive = true;
      
      // Requires physical E-Stop to be released AND reset button
      if (!pbEStop && pbReset) {
        eStopActive = false;
        alarmActive = false;
        memset(&faults, 0, sizeof(faults));
        motorState = STATE_IDLE;
        Serial.println("{\"event\":\"ESTOP_RESET\"}");
      }
      break;
  }
}

// ─────────────────────────────────────────────────────────
// HMI SERIAL OUTPUT (JSON format for Python monitoring)
// ─────────────────────────────────────────────────────────
void sendHMIStatus() {
  const char* stateNames[] = {
    "IDLE", "STAR", "TRANSITION", "DELTA", "STOPPING", "FAULT", "ESTOP"
  };
  
  Serial.print("{");
  Serial.print("\"state\":\""); Serial.print(stateNames[motorState]); Serial.print("\",");
  Serial.print("\"km1\":"); Serial.print(km1Active ? "true" : "false"); Serial.print(",");
  Serial.print("\"km2\":"); Serial.print(km2Active ? "true" : "false"); Serial.print(",");
  Serial.print("\"km3\":"); Serial.print(km3Active ? "true" : "false"); Serial.print(",");
  Serial.print("\"alarm\":"); Serial.print(alarmActive ? "true" : "false"); Serial.print(",");
  Serial.print("\"estop\":"); Serial.print(eStopActive ? "true" : "false"); Serial.print(",");
  Serial.print("\"runtime_ms\":"); Serial.print(runTimeMs); Serial.print(",");
  Serial.print("\"faults\":{");
  Serial.print("\"low_pressure\":"); Serial.print(faults.lowPressure ? "true" : "false"); Serial.print(",");
  Serial.print("\"high_temp\":"); Serial.print(faults.highTemp ? "true" : "false"); Serial.print(",");
  Serial.print("\"overcurrent\":"); Serial.print(faults.overcurrent ? "true" : "false");
  Serial.print("}}");
  Serial.println();
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  
  // Configure inputs (with internal pullups)
  pinMode(PIN_START_PB, INPUT_PULLUP);
  pinMode(PIN_STOP_PB,  INPUT_PULLUP);
  pinMode(PIN_ESTOP,    INPUT_PULLUP);
  pinMode(PIN_RESET_PB, INPUT_PULLUP);
  pinMode(PIN_PRESS_SW, INPUT_PULLUP);
  pinMode(PIN_TEMP_SW,  INPUT_PULLUP);
  pinMode(PIN_VIBR_SW,  INPUT_PULLUP);
  pinMode(PIN_LEVEL_SW, INPUT_PULLUP);
  
  // Configure outputs
  pinMode(PIN_KM1_MAIN,  OUTPUT);
  pinMode(PIN_KM2_STAR,  OUTPUT);
  pinMode(PIN_KM3_DELTA, OUTPUT);
  pinMode(PIN_ALARM_OUT, OUTPUT);
  
  // Safe initial state
  digitalWrite(PIN_KM1_MAIN,  LOW);
  digitalWrite(PIN_KM2_STAR,  LOW);
  digitalWrite(PIN_KM3_DELTA, LOW);
  digitalWrite(PIN_ALARM_OUT, LOW);
  
  Serial.println("{\"event\":\"PLC_INIT\",\"version\":\"2.0\",\"status\":\"READY\"}");
}

void loop() {
  unsigned long now = millis();
  
  // Read all inputs (debounced)
  readInputs();
  
  // Safety check (high-frequency scan)
  if (now - lastSafetyCheck >= SAFETY_INTERVAL_MS) {
    lastSafetyCheck = now;
    runStateMachine();
    writeOutputs();
  }
  
  // HMI update (lower frequency)
  if (now - lastHMIUpdate >= HMI_INTERVAL_MS) {
    lastHMIUpdate = now;
    sendHMIStatus();
  }
}
