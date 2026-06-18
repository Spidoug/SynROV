// =====================================================================
// SynROV Firmware - Motor low level control
// ---------------------------------------------------------------------
// Purpose:
//   Coil drive, PWM output, feedback interpretation, DC PID support and
//   automatic calibration support for channel 0.
// =====================================================================

// Utility: returns the manual PWM percent used to mirror CH12 on pin 46 in DC mode.
uint8_t getDcMotorPwmMirrorPercent() {
  return (uint8_t)clampValue((int)roundf(servos.actual[FIRST_PWM_CH]), 0, 100);
}

// Utility: converts a 0..100% PWM value to the Timer5 TOP scale used by pin 46.
uint16_t pwmPercentToTimer5Duty(uint8_t percent) {
  ensureMotorPwmTimerReady();
  uint8_t pct = (uint8_t)clampValue((int)percent, 0, 100);
  return (uint16_t)map((long)pct, 0L, 100L, 0L, (long)ICR5);
}

// Utility: returns the pin-46 duty that mirrors PWM channel 12.
uint16_t getDcMotorMirrorPwmDutyFromCh12() {
  return pwmPercentToTimer5Duty(getDcMotorPwmMirrorPercent());
}

void mirrorPwm46FromCh12ForDcMotor() {
  if (motorType != MOTOR_TYPE_DC_MOTOR) return;
  if (rt.active && rt.motorArmed) return;
  uint16_t mirrorDuty = getDcMotorMirrorPwmDutyFromCh12();
  motor.pwmValue = mirrorDuty;
  motorSetDriverPWM(mirrorDuty);
}

void dcMotorStop() {
  digitalWrite(DC_MOTOR_DIR_PINS[0], LOW);
  digitalWrite(DC_MOTOR_DIR_PINS[1], LOW);
  motor.pwmValue = 0;
  motorDcDriveDirectionValid = false;
  motorSetDriverPWM(0);
}

uint16_t slewDcMotorDuty(uint16_t desiredDuty, bool logicalForward) {
  desiredDuty = clampValue<uint16_t>(desiredDuty, 0, ICR5);

  if (!motorDcDriveDirectionValid) {
    motorDcDriveDirectionValid = true;
    motorDcLastDriveForward = logicalForward;
    return (desiredDuty < MOTOR_DC_PWM_SLEW_TICKS_PER_CONTROL) ? desiredDuty : MOTOR_DC_PWM_SLEW_TICKS_PER_CONTROL;
  }

  if (logicalForward != motorDcLastDriveForward) {
    motorDcLastDriveForward = logicalForward;
    return 0;
  }

  uint16_t previousDuty = clampValue<uint16_t>(motor.pwmValue, 0, ICR5);
  if (desiredDuty > previousDuty) {
    uint16_t maxNext = previousDuty + MOTOR_DC_PWM_SLEW_TICKS_PER_CONTROL;
    if (maxNext < previousDuty) maxNext = ICR5;
    return (desiredDuty < maxNext) ? desiredDuty : maxNext;
  }

  uint16_t maxDownStep = (uint16_t)(MOTOR_DC_PWM_SLEW_TICKS_PER_CONTROL * 2u);
  if ((previousDuty - desiredDuty) > maxDownStep) {
    return previousDuty - maxDownStep;
  }
  return desiredDuty;
}

void dcMotorDrive(bool forward, uint16_t pwmVal) {
  bool dir = forward;
  if (MOTOR_NATIVE_DIRECTION_INVERTED) dir = !dir;
  if (servos.direction[0] == -1) dir = !dir;
  motor.pwmValue = slewDcMotorDuty(pwmVal, forward);
  digitalWrite(DC_MOTOR_DIR_PINS[0], dir ? HIGH : LOW);
  digitalWrite(DC_MOTOR_DIR_PINS[1], dir ? LOW : HIGH);
  motorSetDriverPWM(motor.pwmValue);
}

// Releases motor coils.
void releaseMotorCoils() {
  if (motorType == MOTOR_TYPE_DC_MOTOR) {
    dcMotorStop();
    return;
  }
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(MOTOR_COIL_PINS[i], LOW);
  }
}

// Motor helper for set coils.
void motorSetCoils(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  if (motorType == MOTOR_TYPE_DC_MOTOR) return;
  digitalWrite(MOTOR_COIL_PINS[0], a ? HIGH : LOW);
  digitalWrite(MOTOR_COIL_PINS[1], b ? HIGH : LOW);
  digitalWrite(MOTOR_COIL_PINS[2], c ? HIGH : LOW);
  digitalWrite(MOTOR_COIL_PINS[3], d ? HIGH : LOW);
}

// Motor helper for set driver PWM.
void motorSetDriverPWM(uint16_t pwmVal) {
  ensureMotorPwmTimerReady();
  OCR5A = clampValue<uint16_t>(pwmVal, 0, ICR5);
}

// Returns motor phase count.
uint8_t getMotorPhaseCount() {
  switch (motorDriveMode) {
    case MOTOR_MODE_HALFSTEP_BIPOLAR:
    case MOTOR_MODE_HALFSTEP_UNIPOLAR:
      return 8;
    case MOTOR_MODE_FULLSTEP_BIPOLAR:
    case MOTOR_MODE_FULLSTEP_UNIPOLAR:
    default:
      return 4;
  }
}

// Utility: accel step forward.
void accelStepForward() {
  int8_t delta = 1;

  if (MOTOR_NATIVE_DIRECTION_INVERTED) {
    delta = -delta;
  }

  if (servos.direction[0] == -1) {
    delta = -delta;
  }

  int8_t phaseCount = (int8_t)getMotorPhaseCount();
  int8_t nextPhase = (int8_t)motor.seq + delta;
  while (nextPhase < 0) nextPhase += phaseCount;
  while (nextPhase >= phaseCount) nextPhase -= phaseCount;
  motor.seq = (MotorSequence)nextPhase;
  applyStepPhase(motor.seq);
}

// Utility: accel step backward.
void accelStepBackward() {
  int8_t delta = -1;

  if (MOTOR_NATIVE_DIRECTION_INVERTED) {
    delta = -delta;
  }

  if (servos.direction[0] == -1) {
    delta = -delta;
  }

  int8_t phaseCount = (int8_t)getMotorPhaseCount();
  int8_t nextPhase = (int8_t)motor.seq + delta;
  while (nextPhase < 0) nextPhase += phaseCount;
  while (nextPhase >= phaseCount) nextPhase -= phaseCount;
  motor.seq = (MotorSequence)nextPhase;
  applyStepPhase(motor.seq);
}

// Applies step phase.
void applyStepPhase(MotorSequence seq) {
  if (motorType == MOTOR_TYPE_DC_MOTOR) return;
  switch (motorDriveMode) {
    case MOTOR_MODE_FULLSTEP_BIPOLAR:
      switch (seq) {
        case PHASE_0: motorSetCoils(1, 0, 1, 0); break; // A+ / B+
        case PHASE_1: motorSetCoils(0, 1, 1, 0); break; // A- / B+
        case PHASE_2: motorSetCoils(0, 1, 0, 1); break; // A- / B-
        default:      motorSetCoils(1, 0, 0, 1); break; // A+ / B-
      }
      break;

    case MOTOR_MODE_HALFSTEP_BIPOLAR:
      switch (seq) {
        case PHASE_0: motorSetCoils(1, 0, 0, 0); break; // A+
        case PHASE_1: motorSetCoils(1, 0, 1, 0); break; // A+ / B+
        case PHASE_2: motorSetCoils(0, 0, 1, 0); break; // B+
        case PHASE_3: motorSetCoils(0, 1, 1, 0); break; // A- / B+
        case PHASE_4: motorSetCoils(0, 1, 0, 0); break; // A-
        case PHASE_5: motorSetCoils(0, 1, 0, 1); break; // A- / B-
        case PHASE_6: motorSetCoils(0, 0, 0, 1); break; // B-
        default:      motorSetCoils(1, 0, 0, 1); break; // A+ / B-
      }
      break;

    case MOTOR_MODE_FULLSTEP_UNIPOLAR:
      switch (seq) {
        case PHASE_0: motorSetCoils(1, 0, 1, 0); break; // A1 + B1
        case PHASE_1: motorSetCoils(0, 1, 1, 0); break; // A2 + B1
        case PHASE_2: motorSetCoils(0, 1, 0, 1); break; // A2 + B2
        default:      motorSetCoils(1, 0, 0, 1); break; // A1 + B2
      }
      break;

    case MOTOR_MODE_HALFSTEP_UNIPOLAR:
    default:
      switch (seq) {
        case PHASE_0: motorSetCoils(1, 0, 0, 0); break; // A1
        case PHASE_1: motorSetCoils(1, 0, 1, 0); break; // A1 + B1
        case PHASE_2: motorSetCoils(0, 0, 1, 0); break; // B1
        case PHASE_3: motorSetCoils(0, 1, 1, 0); break; // A2 + B1
        case PHASE_4: motorSetCoils(0, 1, 0, 0); break; // A2
        case PHASE_5: motorSetCoils(0, 1, 0, 1); break; // A2 + B2
        case PHASE_6: motorSetCoils(0, 0, 0, 1); break; // B2
        default:      motorSetCoils(1, 0, 0, 1); break; // A1 + B2
      }
      break;
  }
}

// Applies step output.
void applyStepOutput(MotorSequence seq, uint16_t pwmVal) {
  applyStepPhase(seq);
  motorSetDriverPWM(pwmVal);
}

// Utility: perform step.
void performStep(bool forward, uint16_t pwmVal) {
  if (motorType == MOTOR_TYPE_DC_MOTOR) {
    dcMotorDrive(forward, pwmVal);
    return;
  }
  motorSetDriverPWM(pwmVal);
  if (forward) {
    accelStepForward();
  } else {
    accelStepBackward();
  }
}

// Applies a small Schmitt-like deadband to the base feedback published from A0.
// The alpha filter still runs continuously, but motor.currentAngle only changes
// after the filtered reading has moved farther than A0DB. When it does move,
// the deadband width is subtracted from the step so the reported angle does not
// jump by the full threshold.
float applyBaseA0FeedbackDeadband(float candidateAngle) {
  float deadband = clampValue(motorBaseA0DeadbandDeg,
                              MOTOR_BASE_A0_DEADBAND_MIN_DEG,
                              MOTOR_BASE_A0_DEADBAND_MAX_DEG);
  float previous = clampValue(motor.currentAngle, MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);
  float delta = candidateAngle - previous;

  if (fabsf(delta) <= deadband) {
    return previous;
  }

  float trackedAngle = candidateAngle - ((delta > 0.0f) ? deadband : -deadband);
  return clampValue(trackedAngle, MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);
}

// Reads motor angle.
float readMotorAngle() {
  unsigned long nowUs = micros();
  if (lastMotorFeedbackReadUs != 0 && (nowUs - lastMotorFeedbackReadUs) < MOTOR_FEEDBACK_MIN_INTERVAL_US) {
    return motor.currentAngle;
  }
  lastMotorFeedbackReadUs = nowUs;

  long sum = 0;
  const uint8_t samples = MOTOR_FEEDBACK_SAMPLES;

  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(MOTOR_FEEDBACK_PIN);
  }

  float raw = (float)sum / (float)samples;

  if (motor.filteredPot == 0.0f) {
    motor.filteredPot = raw;
  }

  motor.filteredPot = 0.10f * raw + 0.90f * motor.filteredPot;

  float rawSpan = getMotorRawSpan();
  float minRaw = (float)motorCalibrationZeroADC + servos.offset[0];
  float maxRaw = minRaw + rawSpan;
  float sensedRaw = motor.filteredPot;
  float marginCounts = max(2.0f, rawSpan * 0.03f);

  motorFeedbackInsideWindow =
    (sensedRaw >= (minRaw - marginCounts)) &&
    (sensedRaw <= (maxRaw + marginCounts));

  // BASECH=0 is a physical servo-like axis: absolute 0..359 degrees.
  // Do not wrap the potentiometer reading; outside the calibrated window it
  // saturates at the nearest mechanical end instead of jumping 0 <-> 359.
  float mappedAngle = ((sensedRaw - minRaw) * MOTOR_INPUT_MAX) / rawSpan;

  if (MOTOR_NATIVE_SENSOR_INVERTED) {
    mappedAngle = MOTOR_INPUT_MAX - mappedAngle;
  }

  if (servos.direction[0] == -1) {
    mappedAngle = MOTOR_INPUT_MAX - mappedAngle;
  }

  motor.filteredAngle = angleFilterAlpha * mappedAngle +
                          (1.0f - angleFilterAlpha) * motor.filteredAngle;
  motor.filteredAngle = clampValue(motor.filteredAngle, MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);

  motor.currentAngle = applyBaseA0FeedbackDeadband(motor.filteredAngle);
  servos.actual[0] = motor.currentAngle;
  return motor.currentAngle;
}

// Resets motor control state.
void resetMotorControlState() {
  motor.integral = 0.0f;
  motor.filteredSpeed = 0.0f;
  motor.lastAngle = motor.filteredAngle;
  motor.controlTargetAngle = motor.currentAngle;
  motor.controlTargetInitialized = false;
  motor.commandTrackerInitialized = false;
  motor.lastCommandAngle = motor.targetAngle;
  motor.commandVelocityDps = 0.0f;
  motor.lastCommandChangeMs = 0;
  motor.breakawaySign = 0;
  motor.breakawayUntilMs = 0;
  motor.lastBreakawayMs = 0;
  motor.inHoldBand = false;
  motor.lastControlUs = 0;
  motor.lastAngleUs = 0;
  motor.pwmValue = holdPwm;
  motorDcDriveDirectionValid = false;
}

// Returns motor raw span.
float getMotorRawSpan() {
  float rawSpan = (float)motorOneTurnSpanRaw * getServoSpan(0);
  if (rawSpan < 5.0f) rawSpan = 5.0f;
  return rawSpan;
}

// Returns motor degrees per count.
float getMotorDegPerCount() {
  return MOTOR_INPUT_MAX / getMotorRawSpan();
}

// Sanitizes and clamps motor runtime params.
void sanitizeMotorRuntimeParams() {
  if (motorOneTurnSpanRaw < 10) motorOneTurnSpanRaw = 10;

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

  motorPulse.setMaxSpeed(max(1.0f, 1000000.0f / (float)minStepDelayUs));
  motorPulse.setSpeed(0.0f);

  angleFilterAlpha = clampValue(angleFilterAlpha, 0.01f, 1.0f);
  speedFilterAlpha = clampValue(speedFilterAlpha, 0.01f, 1.0f);
  controlIntervalUs = (uint32_t)clampValue((long)controlIntervalUs, 500L, 100000L);

  motorStopEnter = clampValue(motorStopEnter, 0.5f, 45.0f);
  motorStopExit  = clampValue(motorStopExit, 0.5f, 60.0f);
  if (motorStopExit <= motorStopEnter) {
    motorStopExit = motorStopEnter + 1.0f;
  }

  // DC PID gains must be finite and bounded.
  if (motor.kp != motor.kp) motor.kp = 0.9f;
  if (motor.ki != motor.ki) motor.ki = 0.03f;
  if (motor.kd != motor.kd) motor.kd = 0.0f;
  motor.kp = clampValue(motor.kp, 0.0f, 50.0f);
  motor.ki = clampValue(motor.ki, 0.0f, 10.0f);
  motor.kd = clampValue(motor.kd, 0.0f, 10.0f);

  motorDcPidEnterBandDeg = clampValue(motorDcPidEnterBandDeg,
                                       MOTOR_DC_PID_BAND_MIN_DEG,
                                       MOTOR_DC_PID_BAND_MAX_DEG);
  motorDcPidExitBandDeg = clampValue(motorDcPidExitBandDeg,
                                      MOTOR_DC_PID_BAND_MIN_DEG,
                                      MOTOR_DC_PID_BAND_MAX_DEG);
  if (motorDcPidExitBandDeg <= motorDcPidEnterBandDeg) {
    motorDcPidExitBandDeg = min(MOTOR_DC_PID_BAND_MAX_DEG, motorDcPidEnterBandDeg + 0.6f);
  }
}

// Returns motor adaptive bands.  STOPENTER/STOPEXIT are intentionally used
// only by the stepper profile.  The DC profile uses PIDENTER/PIDEXIT so PID
// stop behavior can be tuned without reusing stepper-only parameters.
void getMotorAdaptiveBands(float &holdBand, float &enterBand, float &exitBand, float &nearBand, float &farBand) {
  const float degPerCount = getMotorDegPerCount();

  if (motorType == MOTOR_TYPE_STEPPER) {
    holdBand  = max(motorStopEnter,        degPerCount * 2.0f);
    enterBand = max(motorStopEnter + 1.5f, degPerCount * 3.2f);
    exitBand  = max(motorStopExit  + 2.5f, degPerCount * 5.2f);
  } else {
    holdBand  = max(motorDcPidEnterBandDeg, degPerCount * 0.85f);
    enterBand = max(motorDcPidExitBandDeg,  holdBand + max(0.45f, degPerCount * 0.85f));
    exitBand  = max(enterBand + 0.75f,         degPerCount * 2.5f);
  }

  nearBand  = max(12.0f, degPerCount * 6.0f);
  farBand   = max(28.0f, degPerCount * 11.0f);

  if (exitBand <= enterBand) {
    exitBand = enterBand + max(4.0f, degPerCount * 2.0f);
  }
  if (farBand <= nearBand) {
    farBand = nearBand + max(10.0f, degPerCount * 3.0f);
  }
}

