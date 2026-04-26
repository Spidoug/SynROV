// =====================================================================
// SynROV Firmware - Stepper and DC motor low level control
// ---------------------------------------------------------------------
// Purpose:
//   Coil drive, PWM output, feedback interpretation, PID support and
//   automatic calibration support for channel 0.
// =====================================================================

// Utility: DC motor stop.
void dcMotorStop() {
  digitalWrite(DC_MOTOR_DIR_PINS[0], LOW);
  digitalWrite(DC_MOTOR_DIR_PINS[1], LOW);
  stepperSetDriverPWM(0);
}

// Utility: DC motor drive.
void dcMotorDrive(bool forward, uint16_t pwmVal) {
  bool dir = forward;
  if (STEPPER_NATIVE_DIRECTION_INVERTED) dir = !dir;
  if (servos.direction[0] == -1) dir = !dir;
  digitalWrite(DC_MOTOR_DIR_PINS[0], dir ? HIGH : LOW);
  digitalWrite(DC_MOTOR_DIR_PINS[1], dir ? LOW : HIGH);
  stepperSetDriverPWM(pwmVal);
}

// Releases stepper coils.
void releaseStepperCoils() {
  if (ch0DriveMode == CH0_MODE_DC_MOTOR) {
    dcMotorStop();
    return;
  }
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(STEPPER_COIL_PINS[i], LOW);
  }
}

// Stepper helper for set coils.
void stepperSetCoils(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  if (ch0DriveMode == CH0_MODE_DC_MOTOR) return;
  digitalWrite(STEPPER_COIL_PINS[0], a ? HIGH : LOW);
  digitalWrite(STEPPER_COIL_PINS[1], b ? HIGH : LOW);
  digitalWrite(STEPPER_COIL_PINS[2], c ? HIGH : LOW);
  digitalWrite(STEPPER_COIL_PINS[3], d ? HIGH : LOW);
}

// Stepper helper for set driver PWM.
void stepperSetDriverPWM(uint16_t pwmVal) {
  ensureStepperPwmTimerReady();
  OCR5A = clampValue<uint16_t>(pwmVal, 0, ICR5);
}

// Returns stepper phase count.
uint8_t getStepperPhaseCount() {
  switch (stepperDriveMode) {
    case STEPPER_MODE_HALFSTEP_BIPOLAR:
    case STEPPER_MODE_HALFSTEP_UNIPOLAR:
      return 8;
    case STEPPER_MODE_FULLSTEP_BIPOLAR:
    case STEPPER_MODE_FULLSTEP_UNIPOLAR:
    default:
      return 4;
  }
}

// Utility: accel step forward.
void accelStepForward() {
  int8_t delta = 1;

  if (STEPPER_NATIVE_DIRECTION_INVERTED) {
    delta = -delta;
  }

  if (servos.direction[0] == -1) {
    delta = -delta;
  }

  int8_t phaseCount = (int8_t)getStepperPhaseCount();
  int8_t nextPhase = (int8_t)stepper.seq + delta;
  while (nextPhase < 0) nextPhase += phaseCount;
  while (nextPhase >= phaseCount) nextPhase -= phaseCount;
  stepper.seq = (StepperSequence)nextPhase;
  applyStepPhase(stepper.seq);
}

// Utility: accel step backward.
void accelStepBackward() {
  int8_t delta = -1;

  if (STEPPER_NATIVE_DIRECTION_INVERTED) {
    delta = -delta;
  }

  if (servos.direction[0] == -1) {
    delta = -delta;
  }

  int8_t phaseCount = (int8_t)getStepperPhaseCount();
  int8_t nextPhase = (int8_t)stepper.seq + delta;
  while (nextPhase < 0) nextPhase += phaseCount;
  while (nextPhase >= phaseCount) nextPhase -= phaseCount;
  stepper.seq = (StepperSequence)nextPhase;
  applyStepPhase(stepper.seq);
}

// Applies step phase.
void applyStepPhase(StepperSequence seq) {
  if (ch0DriveMode == CH0_MODE_DC_MOTOR) return;
  switch (stepperDriveMode) {
    case STEPPER_MODE_FULLSTEP_BIPOLAR:
      switch (seq) {
        case PHASE_0: stepperSetCoils(1, 0, 1, 0); break; // A+ / B+
        case PHASE_1: stepperSetCoils(0, 1, 1, 0); break; // A- / B+
        case PHASE_2: stepperSetCoils(0, 1, 0, 1); break; // A- / B-
        default:      stepperSetCoils(1, 0, 0, 1); break; // A+ / B-
      }
      break;

    case STEPPER_MODE_HALFSTEP_BIPOLAR:
      switch (seq) {
        case PHASE_0: stepperSetCoils(1, 0, 0, 0); break; // A+
        case PHASE_1: stepperSetCoils(1, 0, 1, 0); break; // A+ / B+
        case PHASE_2: stepperSetCoils(0, 0, 1, 0); break; // B+
        case PHASE_3: stepperSetCoils(0, 1, 1, 0); break; // A- / B+
        case PHASE_4: stepperSetCoils(0, 1, 0, 0); break; // A-
        case PHASE_5: stepperSetCoils(0, 1, 0, 1); break; // A- / B-
        case PHASE_6: stepperSetCoils(0, 0, 0, 1); break; // B-
        default:      stepperSetCoils(1, 0, 0, 1); break; // A+ / B-
      }
      break;

    case STEPPER_MODE_FULLSTEP_UNIPOLAR:
      switch (seq) {
        case PHASE_0: stepperSetCoils(1, 0, 1, 0); break; // A1 + B1
        case PHASE_1: stepperSetCoils(0, 1, 1, 0); break; // A2 + B1
        case PHASE_2: stepperSetCoils(0, 1, 0, 1); break; // A2 + B2
        default:      stepperSetCoils(1, 0, 0, 1); break; // A1 + B2
      }
      break;

    case STEPPER_MODE_HALFSTEP_UNIPOLAR:
    default:
      switch (seq) {
        case PHASE_0: stepperSetCoils(1, 0, 0, 0); break; // A1
        case PHASE_1: stepperSetCoils(1, 0, 1, 0); break; // A1 + B1
        case PHASE_2: stepperSetCoils(0, 0, 1, 0); break; // B1
        case PHASE_3: stepperSetCoils(0, 1, 1, 0); break; // A2 + B1
        case PHASE_4: stepperSetCoils(0, 1, 0, 0); break; // A2
        case PHASE_5: stepperSetCoils(0, 1, 0, 1); break; // A2 + B2
        case PHASE_6: stepperSetCoils(0, 0, 0, 1); break; // B2
        default:      stepperSetCoils(1, 0, 0, 1); break; // A1 + B2
      }
      break;
  }
}

// Applies step output.
void applyStepOutput(StepperSequence seq, uint16_t pwmVal) {
  applyStepPhase(seq);
  stepperSetDriverPWM(pwmVal);
}

// Utility: perform step.
void performStep(bool forward, uint16_t pwmVal) {
  stepperSetDriverPWM(pwmVal);
  if (ch0DriveMode == CH0_MODE_DC_MOTOR) {
    dcMotorDrive(forward, pwmVal);
    return;
  }
  if (forward) {
    accelStepForward();
  } else {
    accelStepBackward();
  }
}

// Reads stepper angle.
float readStepperAngle() {
  unsigned long nowUs = micros();
  if (lastStepperFeedbackReadUs != 0 && (nowUs - lastStepperFeedbackReadUs) < STEPPER_FEEDBACK_MIN_INTERVAL_US) {
    return stepper.currentAngle;
  }
  lastStepperFeedbackReadUs = nowUs;

  long sum = 0;
  const uint8_t samples = STEPPER_FEEDBACK_SAMPLES;

  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(STEPPER_FEEDBACK_PIN);
  }

  float raw = (float)sum / (float)samples;

  if (stepper.filteredPot == 0.0f) {
    stepper.filteredPot = raw;
  }

  stepper.filteredPot = 0.10f * raw + 0.90f * stepper.filteredPot;

  float rawSpan = getStepperRawSpan();
  float minRaw = (float)stepperCalibrationZeroADC + servos.offset[0];
  float maxRaw = minRaw + rawSpan;
  float sensedRaw = stepper.filteredPot;
  float marginCounts = max(2.0f, rawSpan * 0.03f);

  stepperFeedbackInsideWindow =
    (sensedRaw >= (minRaw - marginCounts)) &&
    (sensedRaw <= (maxRaw + marginCounts));

  // Keep the angular reading continuous even outside the normal work window.
  // This keeps the PID active so it can drive the shaft back into range.
  float mappedAngle = ((sensedRaw - minRaw) * STEPPER_INPUT_MAX) / rawSpan;

  if (STEPPER_NATIVE_SENSOR_INVERTED) {
    mappedAngle = STEPPER_INPUT_MAX - mappedAngle;
  }

  if (servos.direction[0] == -1) {
    mappedAngle = STEPPER_INPUT_MAX - mappedAngle;
  }

  stepper.filteredAngle = angleFilterAlpha * mappedAngle +
                          (1.0f - angleFilterAlpha) * stepper.filteredAngle;
  stepper.filteredAngle = wrapAbsoluteDeg(stepper.filteredAngle);

  stepper.currentAngle = stepper.filteredAngle;
  servos.actual[0] = stepper.currentAngle;
  return stepper.currentAngle;
}

// Resets stepper PID.
void resetStepperPID() {
  stepper.integral = 0.0f;
  stepper.filteredSpeed = 0.0f;
  stepper.lastAngle = stepper.filteredAngle;
  stepper.inHoldBand = false;
  stepper.lastControlUs = 0;
  stepper.lastAngleUs = 0;
  stepper.pwmValue = holdPwm;
}

// Returns stepper raw span.
float getStepperRawSpan() {
  float rawSpan = (float)stepperOneTurnSpanRaw * getServoSpan(0);
  if (rawSpan < 5.0f) rawSpan = 5.0f;
  return rawSpan;
}

// Returns stepper degrees per count.
float getStepperDegPerCount() {
  return STEPPER_INPUT_MAX / getStepperRawSpan();
}

// Sanitizes and clamps stepper runtime params.
void sanitizeStepperRuntimeParams() {
  if (stepperOneTurnSpanRaw < 10) stepperOneTurnSpanRaw = 10;

  minStepDelayUs = (uint16_t)clampValue((long)minStepDelayUs, 500L, 60000L);
  maxStepDelayUs = (uint16_t)clampValue((long)maxStepDelayUs, 500L, 60000L);
  if (maxStepDelayUs < minStepDelayUs) {
    uint16_t tmp = minStepDelayUs;
    minStepDelayUs = maxStepDelayUs;
    maxStepDelayUs = tmp;
  }

  movePwmFar = (uint16_t)clampValue((int)movePwmFar, 0, (int)ICR5);
  movePwmNear = (uint16_t)clampValue((int)movePwmNear, 0, (int)ICR5);
  holdPwm = (uint16_t)clampValue((int)holdPwm, 0, (int)ICR5);

  if (movePwmNear < holdPwm) movePwmNear = holdPwm;
  if (movePwmFar < movePwmNear) movePwmFar = movePwmNear;

  stepperMotor.setMaxSpeed(max(1.0f, 1000000.0f / (float)minStepDelayUs));
  stepperMotor.setSpeed(0.0f);

  angleFilterAlpha = clampValue(angleFilterAlpha, 0.01f, 1.0f);
  speedFilterAlpha = clampValue(speedFilterAlpha, 0.01f, 1.0f);
  controlIntervalUs = (uint32_t)clampValue((long)controlIntervalUs, 500L, 100000L);

  stepperStopEnter = clampValue(stepperStopEnter, 0.5f, 45.0f);
  stepperStopExit  = clampValue(stepperStopExit, 0.5f, 60.0f);
  if (stepperStopExit <= stepperStopEnter) {
    stepperStopExit = stepperStopEnter + 1.0f;
  }

  if (stepper.kp < 0.0f) stepper.kp = 0.0f;
  if (stepper.ki < 0.0f) stepper.ki = 0.0f;
  if (stepper.kd < 0.0f) stepper.kd = 0.0f;
}

// Returns stepper adaptive bands.
void getStepperAdaptiveBands(float &holdBand, float &enterBand, float &exitBand, float &nearBand, float &farBand) {
  const float degPerCount = getStepperDegPerCount();

  holdBand  = max(stepperStopEnter,           degPerCount * 2.0f);
  enterBand = max(stepperStopEnter + 1.5f,    degPerCount * 3.2f);
  exitBand  = max(stepperStopExit  + 2.5f,    degPerCount * 5.2f);
  nearBand  = max(12.0f,                      degPerCount * 6.0f);
  farBand   = max(28.0f,                      degPerCount * 11.0f);

  if (exitBand <= enterBand) {
    exitBand = enterBand + max(4.0f, degPerCount * 2.0f);
  }
  if (farBand <= nearBand) {
    farBand = nearBand + max(10.0f, degPerCount * 3.0f);
  }
}

