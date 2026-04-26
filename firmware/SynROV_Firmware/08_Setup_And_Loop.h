// =====================================================================
// SynROV Firmware - Setup and main loop
// ---------------------------------------------------------------------
// Purpose:
//   Arduino setup and loop entry points that initialize the platform and
//   keep the runtime tasks ticking.
//
// Serial strategy
// ---------------
//   Serial0 (USB) is the single control/telemetry port.
//   Boot and config phases use plain ASCII before runtime HEX starts.
//   Runtime switches to HEX-framed protocol via serialHexMode flag.
//   Serial1 is reserved for GPS input and never shares the control stream.
// =====================================================================

static void printBootBanner() {
  Serial.println(F("============================================================"));
  Serial.println(F("SynROV - Multi Robot Firmware - by Douglas Santana // @spidoug"));
  Serial.println(F("============================================================"));
  Serial.println(F(""));
}

static void setupBootI2CBus() {
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000UL, true);
#endif
  Wire.setClock(I2C_CLOCK_SPEED);
}

// Prints the hardware summary collected during boot.
static void printBootSummary() {
  Serial.println(F("PCA9685: "));
  Serial.println(rt.pwmReady ? F("FOUND") : F("NOT FOUND -> USING ARDUINO FALLBACK"));
  Serial.println(F("INA219 #1: "));
  Serial.println(rt.ina1Ready ? F("FOUND") : F("NOT FOUND"));
  Serial.println(F("INA219 #2: "));
  Serial.println(rt.ina2Ready ? F("FOUND") : F("NOT FOUND"));
  Serial.println(F("MPU6050 #1: "));
  Serial.println(rt.mpu1Ready ? F("FOUND") : F("NOT FOUND"));
  Serial.println(F("MPU6050 #2: "));
  Serial.println(rt.mpu2Ready ? F("FOUND") : F("NOT FOUND"));
  Serial.println(F("GPS Serial1: "));
  Serial.println(gpsRuntime.portReady ? F("READY") : F("NOT AVAILABLE"));
  Serial.println(F("GY-273 / COMPASS: "));
  if (rt.compassReady) {
    Serial.println(F("FOUND"));
    Serial.print(F("ADDR=")); printI2CAddressHex(compassAddr); Serial.println();
    Serial.print(F("TYPE=")); Serial.println((uint8_t)compassType);
  } else {
    Serial.println(F("NOT FOUND"));
  }
  Serial.println(F(""));
}

// Utility: setup.
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  // Serial0 is reserved for Processing. GPS is isolated on Serial1.
  initGpsSerial1();
  // The single-serial HEX protocol uses Serial0 (USB) for all I/O.

  // Firmware startup is non-blocking; the desktop handshake already tolerates boot timing.

  printBootBanner();

  setupBootI2CBus();

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(STEPPER_COIL_PINS[i], OUTPUT);
    digitalWrite(STEPPER_COIL_PINS[i], LOW);
  }
  setupHighFreqPwmPin46();

  pinMode(DC_MOTOR_DIR_PINS[0], OUTPUT);
  pinMode(DC_MOTOR_DIR_PINS[1], OUTPUT);
  digitalWrite(DC_MOTOR_DIR_PINS[0], LOW);
  digitalWrite(DC_MOTOR_DIR_PINS[1], LOW);

  stepperMotor.setMaxSpeed(500.0f);
  stepperMotor.setSpeed(0.0f);
  stepperMotor.disableOutputs();

  pinMode(STEPPER_FEEDBACK_PIN, INPUT);
  configureFallbackOutputs();
  releaseStepperCoils();
  stepperSetDriverPWM(0);

  initServoBank();
  initStepperState();
  initRuntimeState();
  initAutoStepperState();

  loadConfigFromEEPROM();
  ensureSafeStartupPoseIfColliding();
  rebindI2CDevices();
  detectAndInitCompass();
  configureRobotModeHardware();
  configureCh0PinsForCurrentMode();
  initAutoStepperState();
  initStepperCalState();

  printBootSummary();
  printBootHelp();
  printRobotSpecificConfigSummary();

  // Emit a short machine-readable identity line so the desktop app can
  // recognize the board even if the first READY? arrives later.
  printRobotControlModeStatus();

  // Reset all serial-related state
  rt.active              = false;
  rt.configMode          = false;
  rt.stepperArmed        = false;
  rt.servoOutputsArmed   = false;
  controlOwner           = CONTROL_OWNER_NONE;
  serialHexMode          = false;   // start in ASCII mode
  serialIndex            = 0;
  currentTextCommandSource = 0;
  bootCommandIgnoreUntilMs = 0;
}

// Utility: loop.
void loop() {
  unsigned long nowMs = millis();
  pollGpsSerial1();

  // Keep telemetry responsive even if the host sends a dense control stream.
  // Serial RX is budgeted in processSerial(), so command parsing and collision
  // checks cannot starve the telemetry transmitter.
  if (rt.active && !autoStepper.active && (nowMs - rt.lastSensorMs) >= SENSOR_INTERVAL_MS) {
    rt.lastSensorMs = nowMs;
    sendTelemetryHex();
    maybeSendGpsTelemetryHex();
  }

  processSerial();

  updateDirectStepperTest();

  if (rt.active) {
    updateStepperMotorPID();
    autoStepperUpdate();

    nowMs = millis();

    if (robotControlMode == ROBOT_MODE_MANIPULATOR) {
      updateWristGimbalRuntime();
      updateArmStabilization();
      updateGripPressureModel();
    }
    updateCompassCalibration();
    updateGenericRobotRuntime();
    updateSafetyAutonomy();
    enforceRuntimeStreamFailSafe();
    updateDirectRobotHardwareOutputs();

    if ((nowMs - rt.lastServoUpdateMs) >= SERVO_UPDATE_MS) {
      rt.lastServoUpdateMs = nowMs;
      updateServosSmooth();
    }

  }
}
