/**
 * Aimbot Nerf Gun – dual‑axis firmware  ❚ 2025‑05‑06
 * ---------------------------------------------------
 * Implements:
 *   1. Sensor‑less homing (motorAHome, motorBHome)
 *   2. Coordinate system zeroing after homing
 *   3. Serial command interface (newline‑terminated)
 *   4. Soft‑limit enforcement and run‑time tuning of max travel, speed & accel
 *   5. Debug prints of position before & after every commanded move
 *
 *  Supported commands (type one per line in Serial Monitor):
 *    home                 → run homing routine for both motors
 *    homea                → run homing routine for motor A only
 *    homeb                → run homing routine for motor B only
 *    mov <N>              → move motor A to absolute step N (0 ≤ N ≤ max)
 *    movb <N>             → move motor B to absolute step N (0 ≤ N ≤ max)
 *    setmax <N>           → set positive soft‑limit for motor A in steps
 *    setmaxb <N>          → set positive soft‑limit for motor B in steps
 *    setspeed <N>         → set max speed for motor A (steps/s)
 *    setspeedb <N>        → set max speed for motor B (steps/s)
 *    setaccel <N>         → set acceleration for motor A (steps/s²)
 *    setaccelb <N>        → set acceleration for motor B (steps/s²)
 *    sett <N>             → set StallGuard SGTHRS for motor A (0‑255)
 *    settb <N>            → set StallGuard SGTHRS for motor B (0‑255)
 *    pos?                 → report current position of motor A
 *    posb?                → report current position of motor B
 *    t?                   → report current SGTHRS value for motor A
 *    tb?                  → report current SGTHRS value for motor B
 *
 *  Example session:
 *    home
 *    sett 6
 *    settb 6
 *    setmax 9500
 *    setmaxb 9500
 *    setspeed 4000
 *    setspeedb 4000
 *    setaccel 12000
 *    setaccelb 12000
 *    mov 3000
 *    movb 3000
 *    pos?
 *    posb?
 */

#include <TMCStepper.h>
#include <AccelStepper.h>

//---------------------------------------------
// Pinout & driver parameters
//---------------------------------------------
#define LED_BUILTIN       2
#define RX1               19
#define TX1               21
#define RX2               22
#define TX2               25
#define LASER_PIN         15
#define FIRE_PIN 34
#define BUTTON_FIRE_PIN 35

#define EN_PIN            23   // Enable
#define STEP_PIN          18   // Step
#define DIR_PIN           5    // Direction
#define STALL_PIN_X       17   // DIAG/STALL output from TMC2209 A

#define STEP_PIN_B        26   // Step for motor B
#define DIR_PIN_B         27   // Direction for motor B
#define STALL_PIN_B       4    // DIAG/STALL output from TMC2209 B

#define SERIAL_PORT Serial1     // UART for TMC2209 A
#define SERIAL_PORT_B Serial2   // UART for TMC2209 B
#define USB Serial              // USB‑CDC for user commands / debug

constexpr float  R_SENSE         = 0.11f;   // Ω
constexpr uint8_t DRIVER_ADDR    = 0b00;    // MS1=0, MS2=0

//---------------------------------------------
// Globals
//---------------------------------------------
TMC2209Stepper driverA(&SERIAL_PORT, R_SENSE, DRIVER_ADDR);
TMC2209Stepper driverB(&SERIAL_PORT_B, R_SENSE, DRIVER_ADDR);
AccelStepper   stepperA(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
AccelStepper   stepperB(AccelStepper::DRIVER, STEP_PIN_B, DIR_PIN_B);

volatile bool stalled_A = false;
volatile bool stalled_B = false;
long          softMax   = 1250;      // positive limit in steps for motor A
long          softMaxB  = 1250;      // positive limit in steps for motor B
uint8_t       sgthrs    = 9;          // StallGuard threshold for motor A (0‑255)
uint8_t       sgthrsB   = 9;          // StallGuard threshold for motor B (0‑255)
long    prevSpeed;
long    prevAccel;
long    prevSpeedB;
long    prevAccelB;
bool laserOn = false;
bool fireOn = false;
bool prevButtonFire = false;

//---------------------------------------------
// ISR – stall flags
//---------------------------------------------
void IRAM_ATTR stallInterruptA() { stalled_A = true; }
void IRAM_ATTR stallInterruptB() { stalled_B = true; }

//---------------------------------------------
// Homing routines (blocking)
//---------------------------------------------
void motorAHome() {
  USB.println("[HOMING A] start");
  driverA.en_spreadCycle(false);         // StealthChop for StallGuard
  driverA.microsteps(16);

  const long HOMING_SPEED     = 1000;    // steps/s
  const long HOMING_ACCEL     = 5000;    // steps/s²
  const long HOMING_TRAVEL_ST = -100000; // toward switch

  // cache run‑mode settings
  prevSpeed = stepperA.maxSpeed();
  prevAccel = stepperA.acceleration();

  // set homing params
  stepperA.setMaxSpeed(HOMING_SPEED);
  stepperA.setAcceleration(HOMING_ACCEL);

  // start homing move
  stalled_A = false;
  stepperA.move(HOMING_TRAVEL_ST);
  while (!stalled_A) stepperA.run();     // wait for stall

  // brick‑wall stop
  stepperA.stop();                       // stop motion immediately
  stepperA.setCurrentPosition(0);        // zero at switch
  driverA.en_spreadCycle(true);          // restore SpreadCycle

  // --- NEW: back off 50 steps ---
  stepperA.moveTo(50);                   // +50 steps away from switch
  while (stepperA.distanceToGo())        // wait for retract
    stepperA.run();

  stepperA.setCurrentPosition(0);        // now zero at back‑off point

  // restore run‑mode params
  stepperA.setMaxSpeed(prevSpeed);
  stepperA.setAcceleration(prevAccel);

  USB.println("[HOMING A] done – backed off 50 steps, position = 0");
}

void motorBHome() {
  USB.println("[HOMING B] start");
  driverB.en_spreadCycle(false);         // StealthChop for StallGuard
  driverB.microsteps(16);

  const long HOMING_SPEED     = 1000;    // steps/s
  const long HOMING_ACCEL     = 5000;    // steps/s²
  const long HOMING_TRAVEL_ST = -100000; // toward switch

  // cache run‑mode settings
  prevSpeedB = stepperB.maxSpeed();
  prevAccelB = stepperB.acceleration();

  // set homing params
  stepperB.setMaxSpeed(HOMING_SPEED);
  stepperB.setAcceleration(HOMING_ACCEL);

  // start homing move
  stalled_B = false;
  stepperB.move(HOMING_TRAVEL_ST);
  while (!stalled_B) stepperB.run();     // wait for stall

  // brick‑wall stop
  stepperB.stop();                       // stop motion immediately
  stepperB.setCurrentPosition(0);        // zero at switch
  driverB.en_spreadCycle(true);          // restore SpreadCycle

  // --- NEW: back off 50 steps ---
  stepperB.moveTo(50);                   // +50 steps away from switch
  while (stepperB.distanceToGo())        // wait for retract
    stepperB.run();

  stepperB.setCurrentPosition(0);        // now zero at back‑off point

  // restore run‑mode params
  stepperB.setMaxSpeed(prevSpeedB);
  stepperB.setAcceleration(prevAccelB);

  USB.println("[HOMING B] done – backed off 50 steps, position = 0");
}

//---------------------------------------------
// Command parsing helpers
//---------------------------------------------
long getNumber(String &tok) { tok.trim(); return tok.toInt(); }

void handleMove(long tgt) {
  if (tgt < 0 || tgt > softMax) {
    USB.printf("ERR: target %ld outside [0,%ld]\n", tgt, softMax);
    return;
  }
  USB.printf("POS BEFORE: %ld\n", stepperA.currentPosition());
  stepperA.moveTo(tgt);
  while (stepperA.distanceToGo()) stepperA.run();
  USB.printf("POS AFTER : %ld\n", stepperA.currentPosition());
}

void handleMoveB(long tgt) {
  if (tgt < 0 || tgt > softMaxB) {
    USB.printf("ERR: target %ld outside [0,%ld]\n", tgt, softMaxB);
    return;
  }
  USB.printf("POS BEFORE B: %ld\n", stepperB.currentPosition());
  stepperB.moveTo(tgt);
  while (stepperB.distanceToGo()) stepperB.run();
  USB.printf("POS AFTER B : %ld\n", stepperB.currentPosition());
}

void processCommand(String line) {
  line.trim(); line.toLowerCase();
  if (!line.length()) return;

  int sp = line.indexOf(' ');
  String cmd  = (sp == -1) ? line : line.substring(0, sp);
  String argS = (sp == -1) ? ""   : line.substring(sp + 1);

  if      (cmd == "home")      { motorAHome(); motorBHome(); }
  else if (cmd == "homea")     motorAHome();
  else if (cmd == "homeb")     motorBHome();
  else if (cmd == "mov")       handleMove(getNumber(argS));
  else if (cmd == "movb")      handleMoveB(getNumber(argS));
  else if (cmd == "setmax") {
    long v = getNumber(argS);
    if (v > 0) { softMax = v; USB.printf("softMax = %ld\n", softMax); }
    else USB.println("ERR: value must be >0");
  }
  else if (cmd == "setmaxb") {
    long v = getNumber(argS);
    if (v > 0) { softMaxB = v; USB.printf("softMaxB = %ld\n", softMaxB); }
    else USB.println("ERR: value must be >0");
  }
  else if (cmd == "setspeed") {
    long v = getNumber(argS);
    if (v > 0) { stepperA.setMaxSpeed(v); USB.printf("maxSpeed = %ld\n", v); }
    else USB.println("ERR: value must be >0");
  }
  else if (cmd == "setspeedb") {
    long v = getNumber(argS);
    if (v > 0) { stepperB.setMaxSpeed(v); USB.printf("maxSpeedB = %ld\n", v); }
    else USB.println("ERR: value must be >0");
  }
  else if (cmd == "setaccel") {
    long v = getNumber(argS);
    if (v > 0) { stepperA.setAcceleration(v); USB.printf("accel = %ld\n", v); }
    else USB.println("ERR: value must be >0");
  }
  else if (cmd == "setaccelb") {
    long v = getNumber(argS);
    if (v > 0) { stepperB.setAcceleration(v); USB.printf("accelB = %ld\n", v); }
    else USB.println("ERR: value must be >0");
  }
  else if (cmd == "sett") {                 // set StallGuard threshold for motor A
    long v = getNumber(argS);
    if (v >= 0 && v <= 255) {
      sgthrs = v;
      driverA.SGTHRS((uint8_t)sgthrs);
      USB.printf("SGTHRS = %u\n", sgthrs);
    } else USB.println("ERR: value must be 0‑255");
  }
  else if (cmd == "settb") {                // set StallGuard threshold for motor B
    long v = getNumber(argS);
    if (v >= 0 && v <= 255) {
      sgthrsB = v;
      driverB.SGTHRS((uint8_t)sgthrsB);
      USB.printf("SGTHRSB = %u\n", sgthrsB);
    } else USB.println("ERR: value must be 0‑255");
  }
  else if (cmd == "t?") {                    // query threshold for motor A
    uint8_t cur = driverA.SGTHRS();
    USB.printf("SGTHRS = %u\n", cur);
  }
  else if (cmd == "tb?") {                   // query threshold for motor B
    uint8_t cur = driverB.SGTHRS();
    USB.printf("SGTHRSB = %u\n", cur);
  }
  else if (cmd == "pos?") {
    USB.printf("POS = %ld\n", stepperA.currentPosition());
  }
  else if (cmd == "posb?") {
    USB.printf("POSB = %ld\n", stepperB.currentPosition());
  }
  else if (cmd == "laser") {
    laserOn = !laserOn;
    digitalWrite(LASER_PIN, laserOn ? HIGH : LOW);
    USB.printf("Laser %s\n", laserOn ? "ON" : "OFF");
  }
  else if (cmd == "fire") {
    fireOn = !fireOn;
    digitalWrite(FIRE_PIN, fireOn ? HIGH : LOW);
    USB.printf("Fire %s\n", fireOn ? "ON" : "OFF");
  }
  else {
    USB.printf("ERR: unknown cmd '%s'\n", cmd.c_str());
  }
}

//---------------------------------------------
// Setup & main loop
//---------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW); // Laser off at startup
  pinMode(FIRE_PIN, OUTPUT);
  digitalWrite(FIRE_PIN, LOW); // DC motors off at startup
  pinMode(BUTTON_FIRE_PIN, INPUT_PULLUP); // Manual fire button
  USB.begin(115200);
  while (!USB);

  SERIAL_PORT.begin(115200, SERIAL_8N1, RX1, TX1);
  SERIAL_PORT_B.begin(115200, SERIAL_8N1, RX2, TX2);
  attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptA, RISING);
  attachInterrupt(digitalPinToInterrupt(STALL_PIN_B), stallInterruptB, RISING);

  driverA.begin();
  driverA.rms_current(1880);
  driverA.pwm_autoscale(true);
  driverA.microsteps(16);
  driverA.TCOOLTHRS(0xFFFFF);
  driverA.SGTHRS(sgthrs);        // apply initial threshold

  driverB.begin();
  driverB.rms_current(1880);
  driverB.pwm_autoscale(true);
  driverB.microsteps(16);
  driverB.TCOOLTHRS(0xFFFFF);
  driverB.SGTHRS(sgthrsB);       // apply initial threshold

  stepperA.setEnablePin(EN_PIN);
  stepperA.setPinsInverted(false, false, true);
  stepperA.enableOutputs();

  stepperB.setPinsInverted(false, false, true);
  stepperB.enableOutputs();

  stepperA.setMaxSpeed(4000);
  stepperA.setAcceleration(12000);
  stepperB.setMaxSpeed(4000);
  stepperB.setAcceleration(12000);
}

void loop() {
  if (USB.available()) {
    String line = USB.readStringUntil('\n');
    processCommand(line);
  }

  // Manual fire button logic with state change message
  bool buttonFire = (digitalRead(BUTTON_FIRE_PIN) == LOW); // Button pressed (active low)
  if (buttonFire != prevButtonFire) {
    if (buttonFire) {
      USB.println("Firing from button");
    } else {
      USB.println("Firing from command");
    }
    prevButtonFire = buttonFire;
  }
  if (buttonFire) {
    digitalWrite(FIRE_PIN, HIGH); // Force fire ON
  } else {
    digitalWrite(FIRE_PIN, fireOn ? HIGH : LOW); // Use toggle state
  }
}
