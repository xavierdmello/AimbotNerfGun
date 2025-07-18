/**
 * Aimbot Nerf Gun – single‑axis firmware  ❚ 2025‑05‑06
 * ---------------------------------------------------
 * Implements:
 *   1. Sensor‑less homing (motorAHome)
 *   2. Coordinate system zeroing after homing
 *   3. Serial command interface (newline‑terminated)
 *   4. Soft‑limit enforcement and run‑time tuning of max travel, speed & accel
 *   5. Debug prints of position before & after every commanded move
 *
 *  Supported commands (type one per line in Serial Monitor):
 *    home                 → run homing routine
 *    mov <N>              → move to absolute step N (0 ≤ N ≤ max)
 *    setmax <N>           → set positive soft‑limit in steps
 *    setspeed <N>         → set max speed (steps/s)
 *    setaccel <N>         → set acceleration (steps/s²)
 *    sett <N>             → set StallGuard SGTHRS (0‑255)
 *    pos?                 → report current position
 *    t?                   → report current SGTHRS value
 *
 *  Example session:
 *    home
 *    sett 6
 *    setmax 9500
 *    setspeed 4000
 *    setaccel 12000
 *    mov 3000
 *    pos?
 */

#include <TMCStepper.h>
#include <AccelStepper.h>

//---------------------------------------------
// Pinout & driver parameters
//---------------------------------------------
#define LED_BUILTIN       2
#define RX1               19
#define TX1               21
#define RX2               22   // UART2 RX
#define TX2               25   // UART2 TX

#define STEP_PIN_A          18   // Step
#define DIR_PIN_A           5    // Direction
#define STALL_PIN_X_A       17   // DIAG/STALL output from TMC2209

#define STEP_PIN_B        26   // Step for motor B
#define DIR_PIN_B         27   // Direction for motor B
#define STALL_PIN_X_B     28   // DIAG/STALL output from TMC2209


#define SERIAL_PORT Serial1     // UART for TMC2209
#define USB Serial              // USB‑CDC for user commands / debug

constexpr float  R_SENSE         = 0.11f;   // Ω
constexpr uint8_t DRIVER_ADDR    = 0b00;    // MS1=0, MS2=0

//---------------------------------------------
// Globals
//---------------------------------------------
TMC2209Stepper driverA(&SERIAL_PORT, R_SENSE, DRIVER_ADDR);
AccelStepper   stepperA(AccelStepper::DRIVER, STEP_PIN_A, DIR_PIN);

volatile bool stalled_A = false;
long          softMax   = 1250;      // positive limit in steps
uint8_t       sgthrs    = 5;          // StallGuard threshold (0‑255)
long    prevSpeed;
long    prevAccel;

//---------------------------------------------
// ISR – stall flag
//---------------------------------------------
void IRAM_ATTR stallInterruptA() { stalled_A = true; }

//---------------------------------------------
// Homing routine (blocking)
//---------------------------------------------
void motorAHome() {
  USB.println("[HOMING] start");
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
  stepperA.stop();
  stepperA.setCurrentPosition(0);        // temporarily zero at switch
  driverA.en_spreadCycle(true);          // restore SpreadCycle


  // --- NEW: back off 50 steps ---
  stepperA.moveTo(50);                   // +50 steps away from switch
  while (stepperA.distanceToGo())        // wait for retract
    stepperA.run();

  stepperA.setCurrentPosition(0);        // now zero at back‑off point

  // restore run‑mode params
  stepperA.setMaxSpeed(prevSpeed);
  stepperA.setAcceleration(prevAccel);

  USB.println("[HOMING] done – backed off 50 steps, position = 0");
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

void processCommand(String line) {
  line.trim(); line.toLowerCase();
  if (!line.length()) return;

  int sp = line.indexOf(' ');
  String cmd  = (sp == -1) ? line : line.substring(0, sp);
  String argS = (sp == -1) ? ""   : line.substring(sp + 1);

  if      (cmd == "home")      motorAHome();
  else if (cmd == "mov")       handleMove(getNumber(argS));
  else if (cmd == "setmax") {
    long v = getNumber(argS);
    if (v > 0) { softMax = v; USB.printf("softMax = %ld\n", softMax); }
    else USB.println("ERR: value must be >0");
  }
  else if (cmd == "setspeed") {
    long v = getNumber(argS);
    if (v > 0) { stepperA.setMaxSpeed(v); USB.printf("maxSpeed = %ld\n", v); }
    else USB.println("ERR: value must be >0");
  }
  else if (cmd == "setaccel") {
    long v = getNumber(argS);
    if (v > 0) { stepperA.setAcceleration(v); USB.printf("accel = %ld\n", v); }
    else USB.println("ERR: value must be >0");
  }
  else if (cmd == "sett") {                 // --- NEW: set StallGuard threshold
    long v = getNumber(argS);
    if (v >= 0 && v <= 255) {
      sgthrs = v;
      driverA.SGTHRS((uint8_t)sgthrs);
      USB.printf("SGTHRS = %u\n", sgthrs);
    } else USB.println("ERR: value must be 0‑255");
  }
  else if (cmd == "t?") {                    // query threshold
    uint8_t cur = driverA.SGTHRS();
    USB.printf("SGTHRS = %u\n", cur);
  }
  else if (cmd == "pos?") {
    USB.printf("POS = %ld\n", stepperA.currentPosition());
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
  USB.begin(115200);
  while (!USB);

  SERIAL_PORT.begin(115200, SERIAL_8N1, RX1, TX1);
  attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptA, RISING);

  driverA.begin();
  driverA.rms_current(1880);
  driverA.pwm_autoscale(true);
  driverA.microsteps(16);
  driverA.TCOOLTHRS(0xFFFFF);
  driverA.SGTHRS(sgthrs);        // apply initial threshold


  stepperA.setPinsInverted(false, false, true);
  stepperA.enableOutputs();

  stepperA.setMaxSpeed(4000);
  stepperA.setAcceleration(12000);
}

void loop() {
  if (USB.available()) {
    String line = USB.readStringUntil('\n');
    processCommand(line);
  }

  stepperA.run();
}
