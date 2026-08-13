// =====================================================================
// SynROV Firmware V1 - Motor low level control
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

// Resets only the A0 feedback filters. Use this whenever ZEROADC, SPANRAW or
// the CH0 feedback direction changes so the next sample is interpreted directly
// in the new one-turn calibration window instead of being blended with stale data.
void resetMotorFeedbackFilterState() {
  motor.filteredPot = 0.0f;
  motor.filteredAngle = clampValue(motor.currentAngle, MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);
  motor.controlFeedbackAngle = motor.filteredAngle;
  motorFeedbackFilterInitialized = false;
  lastMotorFeedbackReadUs = 0;
}

// Settles the Mega2560 ADC mux on A0 before using a feedback value.
// Telemetry also scans A1..A5, so the first conversion after returning to A0
// is intentionally discarded instead of being allowed to bias CH0 position.
static inline void settleMotorFeedbackAdcInput() {
  for (uint8_t i = 0; i < MOTOR_ADC_SETTLE_DISCARD_SAMPLES; i++) {
    (void)analogRead(MOTOR_FEEDBACK_PIN);
  }
}

// Returns an averaged A0 reading after ADC-mux settling. The returned float is
// deliberately fractional; averaging can retain useful sub-count information
// from natural ADC noise even when the selected one-turn window is relatively
// narrow.
float readMotorFeedbackAdcAverage(uint8_t samples) {
  if (samples < 1) samples = 1;
  settleMotorFeedbackAdcInput();
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += (uint16_t)analogRead(MOTOR_FEEDBACK_PIN);
  }
  return (float)sum / (float)samples;
}

// Reads motor angle from the operator-selected one-turn ADC window.
float readMotorAngle() {
  unsigned long nowUs = micros();
  if (lastMotorFeedbackReadUs != 0 && (nowUs - lastMotorFeedbackReadUs) < MOTOR_FEEDBACK_MIN_INTERVAL_US) {
    return motor.currentAngle;
  }
  lastMotorFeedbackReadUs = nowUs;

  const float raw = readMotorFeedbackAdcAverage(MOTOR_FEEDBACK_SAMPLES);

  const bool firstSample = !motorFeedbackFilterInitialized;
  if (firstSample) {
    motor.filteredPot = raw;
    motorFeedbackFilterInitialized = true;
  } else {
    // Keep the closed-loop feedback responsive while CH0 is moving. The
    // previous fixed 0.10 EMA behaved like a heavy low-pass in front of the
    // PID and made a continuously changing target feel sticky. Once the axis
    // has genuinely entered HOLD we trade some response for extra noise
    // suppression. The five-sample ADC average remains the first noise filter.
    const float rawAlpha =
      (rt.active && rt.motorArmed && !motor.inHoldBand)
        ? MOTOR_FEEDBACK_RAW_ALPHA_MOVING
        : MOTOR_FEEDBACK_RAW_ALPHA_HOLD;
    motor.filteredPot = rawAlpha * raw +
                        (1.0f - rawAlpha) * motor.filteredPot;
  }

  const float rawSpan = getMotorRawSpan();
  const float minRaw = (float)motorCalibrationZeroADC;
  const float maxRaw = (float)motorCalibrationSpanRaw;
  const float sensedRaw = motor.filteredPot;

  // This flag describes the selected turn, not the electrical limits of the
  // multi-turn potentiometer. A one-count tolerance avoids endpoint chatter.
  motorFeedbackInsideWindow =
    (sensedRaw >= (minRaw - 1.0f)) &&
    (sensedRaw <= (maxRaw + 1.0f));

  // Preserve a small, bounded amount of endpoint information for the PID.
  // Clamp the converted angle rather than the raw A0 sample.
  // That made every value beyond an endpoint look exactly like 0 or 359 and
  // created a control blind zone.  We now map the filtered raw value first,
  // retain only a small overrange for the internal controller, and clamp only
  // the public/Processing angle. Extra potentiometer turns still cannot become
  // valid public positions.
  float rawMappedAngle = ((sensedRaw - minRaw) * MOTOR_INPUT_MAX) / rawSpan;

  if (MOTOR_NATIVE_SENSOR_INVERTED) {
    rawMappedAngle = MOTOR_INPUT_MAX - rawMappedAngle;
  }
  if (servos.direction[0] == -1) {
    rawMappedAngle = MOTOR_INPUT_MAX - rawMappedAngle;
  }

  const float degPerCount = MOTOR_INPUT_MAX / rawSpan;
  const float controlOverrangeDeg = clampValue(
    degPerCount * MOTOR_ENDPOINT_CONTROL_OVERRANGE_RAW_COUNTS,
    MOTOR_ENDPOINT_CONTROL_OVERRANGE_MIN_DEG,
    MOTOR_ENDPOINT_CONTROL_OVERRANGE_MAX_DEG);
  motor.controlFeedbackAngle = clampValue(
    rawMappedAngle,
    MOTOR_INPUT_MIN - controlOverrangeDeg,
    MOTOR_INPUT_MAX + controlOverrangeDeg);

  const float mappedAngle = clampValue(rawMappedAngle,
                                       MOTOR_INPUT_MIN,
                                       MOTOR_INPUT_MAX);

  if (firstSample) {
    // First reading after boot/calibration/direction change must be absolute.
    // Blending it with the previous coordinate system would create a false angle.
    motor.filteredAngle = mappedAngle;
    motor.currentAngle = mappedAngle;
  } else {
    // Preserve the configured public-angle filter for Processing/telemetry.
    // The PID does not depend on this extra stage; it uses controlFeedbackAngle
    // derived directly from the already filtered A0 value above.
    motor.filteredAngle = angleFilterAlpha * mappedAngle +
                          (1.0f - angleFilterAlpha) * motor.filteredAngle;
    motor.filteredAngle = clampValue(motor.filteredAngle, MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);
    motor.currentAngle = applyBaseA0FeedbackDeadband(motor.filteredAngle);
  }

  servos.actual[0] = motor.currentAngle;
  return motor.currentAngle;
}

// Resets motor control state.
void resetMotorControlState() {
  motor.integral = 0.0f;
  motor.filteredSpeed = 0.0f;
  motor.lastAngle = motor.controlFeedbackAngle;
  motor.controlTargetAngle = motor.controlFeedbackAngle;
  motor.controlTargetInitialized = false;
  motor.commandTrackerInitialized = false;
  motor.lastCommandAngle = motor.targetAngle;
  motor.commandVelocityDps = 0.0f;
  motor.lastCommandChangeMs = 0;
  motor.stictionSign = 0;
  motor.stictionStartMs = 0;
  motor.inHoldBand = false;
  motor.lastControlUs = 0;
  motor.lastAngleUs = 0;
  motor.pwmValue = holdPwm;
  motorDcDriveDirectionValid = false;
}

// Returns the ADC width of exactly the selected logical turn.
// The current calibration stores ZEROADC as the lower endpoint and SPANRAW as the upper endpoint.
float getMotorRawSpan() {
  int32_t low = clampValue<int32_t>((int32_t)motorCalibrationZeroADC, MOTOR_ADC_MIN, MOTOR_ADC_MAX);
  int32_t high = clampValue<int32_t>(motorCalibrationSpanRaw, MOTOR_ADC_MIN, MOTOR_ADC_MAX);
  int32_t width = high - low;
  if (width < MOTOR_CAL_MIN_SPAN_RAW) width = MOTOR_CAL_MIN_SPAN_RAW;
  return (float)width;
}

// Returns logical CH0 degrees per ADC count inside the selected one-turn window.
float getMotorDegPerCount() {
  return MOTOR_INPUT_MAX / getMotorRawSpan();
}

// Sanitizes and clamps motor runtime params.
void sanitizeMotorRuntimeParams() {
  // Keep the explicit ADC window ZEROADC..SPANRAW inside the Mega2560
  // 10-bit range. SPANRAW is the upper endpoint, not a width.
  motorCalibrationZeroADC = (int16_t)clampValue<int32_t>(
    (int32_t)motorCalibrationZeroADC,
    (int32_t)MOTOR_ADC_MIN,
    (int32_t)MOTOR_ADC_MAX - MOTOR_CAL_MIN_SPAN_RAW);

  int32_t minUpper = (int32_t)motorCalibrationZeroADC + MOTOR_CAL_MIN_SPAN_RAW;
  motorCalibrationSpanRaw = clampValue<int32_t>(
    motorCalibrationSpanRaw,
    minUpper,
    (int32_t)MOTOR_ADC_MAX);

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
  if (motor.kp != motor.kp) motor.kp = MOTOR_DC_KP_DEFAULT;
  if (motor.ki != motor.ki) motor.ki = MOTOR_DC_KI_DEFAULT;
  if (motor.kd != motor.kd) motor.kd = MOTOR_DC_KD_DEFAULT;
  motor.kp = clampValue(motor.kp, 0.0f, 50.0f);
  motor.ki = clampValue(motor.ki, 0.0f, 10.0f);
  motor.kd = clampValue(motor.kd, 0.0f, 10.0f);

  motorDcPidStopBandDeg = clampValue(motorDcPidStopBandDeg,
                                       MOTOR_DC_PID_BAND_MIN_DEG,
                                       MOTOR_DC_PID_BAND_MAX_DEG);
  motorDcPidExitBandDeg = clampValue(motorDcPidExitBandDeg,
                                      MOTOR_DC_PID_BAND_MIN_DEG,
                                      MOTOR_DC_PID_BAND_MAX_DEG);
  if (motorDcPidExitBandDeg <= motorDcPidStopBandDeg) {
    motorDcPidExitBandDeg = min(MOTOR_DC_PID_BAND_MAX_DEG, motorDcPidStopBandDeg + 0.6f);
  }
}

// Returns motor adaptive bands.  STOPENTER/STOPEXIT are intentionally used
// only by the stepper profile.  The DC profile uses PIDSTOP/PIDEXIT so PID
// stop behavior can be tuned without reusing stepper-only parameters.
void getMotorAdaptiveBands(float &holdBand, float &enterBand, float &exitBand, float &nearBand, float &farBand) {
  const float degPerCount = getMotorDegPerCount();

  if (motorType == MOTOR_TYPE_STEPPER) {
    holdBand  = max((float)motorStopEnter,        degPerCount * 2.0f);
    enterBand = max(motorStopEnter + 1.5f, degPerCount * 3.2f);
    exitBand  = max(motorStopExit  + 2.5f, degPerCount * 5.2f);
  } else {
    holdBand  = max((float)motorDcPidStopBandDeg, degPerCount * 0.85f);
    enterBand = max((float)motorDcPidExitBandDeg,  holdBand + max(0.45f, degPerCount * 0.85f));
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

