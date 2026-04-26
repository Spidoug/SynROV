// =====================================================================
// SynROV Firmware - Runtime commands and diagnostics
// ---------------------------------------------------------------------
// Purpose:
//   Command parsing, status panels, runtime configuration, calibration
//   flows and operator-facing diagnostics.
// =====================================================================

// Initializes auto stepper state.
void initAutoStepperState() {
  memset(&autoStepper, 0, sizeof(autoStepper));
  autoStepper.stage = AUTO_STAGE_IDLE;
  autoStepper.cachedStage = AUTO_STAGE_IDLE;
  autoStepper.best.valid = false;
  autoStepper.best.score = -1000000000.0f;
  autoStepper.modeAttemptIndex = 0;
  autoStepper.originalMode = stepperDriveMode;
}

// Initializes stepper cal state.
void initStepperCalState() {
  memset(&stepperCal, 0, sizeof(stepperCal));
  stepperCal.stage = STEPPER_CAL_IDLE;
}

// Reads stepper raw ADC.
int16_t readStepperRawADC() {
  long sum = 0;
  const uint8_t samples = 16;
  for (uint8_t i = 0; i < samples; i++) sum += analogRead(STEPPER_FEEDBACK_PIN);
  return (int16_t)(sum / samples);
}

// Starts stepper calibration wizard.
void startStepperCalibrationWizard() {
  if (!rt.configMode) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
    return;
  }
  autoStepperStop(false);
  stepperCal.active = true;
  stepperCal.stage = STEPPER_CAL_WAIT_LIMIT1;
  stepperCal.limit1Adc = 0;
  stepperCal.limit2Adc = 0;
  Serial.println(F("LIMITSTEPPERCALIB STARTED"));
  Serial.println(F("MOVE THE MOTOR TO THE FIRST MECHANICAL LIMIT AND SEND LIMITSTEPPERCALIB"));
}

// Utility: capture stepper calibration limit 1.
void captureStepperCalibrationLimit1() {
  if (!stepperCal.active || stepperCal.stage != STEPPER_CAL_WAIT_LIMIT1) {
    Serial.println(F("LIMITSTEPPERCALIB IS NOT WAITING FOR THE FIRST LIMIT"));
    return;
  }
  stepperCal.limit1Adc = readStepperRawADC();
  stepperCal.stage = STEPPER_CAL_WAIT_LIMIT2;
  Serial.print(F("LIMITSTEPPERCALIB LIMIT_A ADC=")); Serial.println(stepperCal.limit1Adc);
  Serial.println(F("MOVE THE MOTOR TO THE OPPOSITE MECHANICAL LIMIT AND SEND LIMITSTEPPERCALIB AGAIN"));
}

// Utility: capture stepper calibration limit 2.
void captureStepperCalibrationLimit2() {
  if (!stepperCal.active || stepperCal.stage != STEPPER_CAL_WAIT_LIMIT2) {
    Serial.println(F("LIMITSTEPPERCALIB IS NOT WAITING FOR THE SECOND LIMIT"));
    return;
  }
  stepperCal.limit2Adc = readStepperRawADC();
  int16_t low = min(stepperCal.limit1Adc, stepperCal.limit2Adc);
  int16_t high = max(stepperCal.limit1Adc, stepperCal.limit2Adc);
  int32_t span = (int32_t)high - (int32_t)low;
  if (span < 20) {
    Serial.println(F("LIMITSTEPPERCALIB FAILED - LIMITS ARE TOO CLOSE"));
    initStepperCalState();
    return;
  }
  stepperCalibrationZeroADC = low;
  stepperOneTurnSpanRaw = span;
  sanitizeStepperRuntimeParams();
  stepper.filteredPot = 0.0f;
  readStepperAngle();
  stepper.targetAngle = clampValue(stepper.currentAngle, STEPPER_INPUT_MIN, STEPPER_INPUT_MAX);
  servos.target[0] = stepper.targetAngle;
  resetStepperPID();
  stepperCal.active = false;
  stepperCal.stage = STEPPER_CAL_DONE;
  rt.active = false;
  rt.configMode = true;
  rt.stepperArmed = false;
  rt.servoOutputsArmed = false;
  releaseStepperCoils();
  stepperSetDriverPWM(0);
  stepperMotor.setSpeed(0.0f);
  Serial.println(F("LIMITSTEPPERCALIB DONE"));
  Serial.print(F("ZEROADC=")); Serial.println(stepperCalibrationZeroADC);
  Serial.print(F("SPANRAW=")); Serial.println(stepperOneTurnSpanRaw);
  configSessionDirty = true;
  Serial.println(F("VALUES ARE READY IN CFG - USE SAVE TO STORE EEPROM OR EXIT TO LEAVE CFG"));
}

// Handles callimit stepper command.
void handleCallimitStepperCommand() {
  if (!stepperCal.active || stepperCal.stage == STEPPER_CAL_IDLE || stepperCal.stage == STEPPER_CAL_DONE) {
    startStepperCalibrationWizard();
    return;
  }
  if (stepperCal.stage == STEPPER_CAL_WAIT_LIMIT1) {
    captureStepperCalibrationLimit1();
    return;
  }
  if (stepperCal.stage == STEPPER_CAL_WAIT_LIMIT2) {
    captureStepperCalibrationLimit2();
    return;
  }
}

// Prints stepper calibration status.
void printStepperCalibrationStatus() {
  Serial.print(F("LIMITSTEPPERCALIB ACTIVE=")); Serial.println(stepperCal.active ? 1 : 0);
  Serial.print(F("LIMITSTEPPERCALIB STAGE=")); Serial.println((int)stepperCal.stage);
  Serial.print(F("LIMITSTEPPERCALIB LIMIT_A=")); Serial.println(stepperCal.limit1Adc);
  Serial.print(F("LIMITSTEPPERCALIB LIMIT_B=")); Serial.println(stepperCal.limit2Adc);
}

// Returns auto stepper safe min angle.
float getAutoStepperSafeMinAngle() {
  return 18.0f;
}

// Returns auto stepper safe max angle.
float getAutoStepperSafeMaxAngle() {
  return 342.0f;
}

// Utility: auto stepper apply candidate.
void autoStepperApplyCandidate(const AutoStepperCandidate& c) {
  stepper.kp = c.kp;
  stepper.ki = c.ki;
  stepper.kd = c.kd;
  minStepDelayUs = c.minStepUs;
  maxStepDelayUs = c.maxStepUs;
  stepperStopEnter = c.stopEnter;
  stepperStopExit = c.stopExit;
  movePwmFar = c.pwmFar;
  movePwmNear = c.pwmNear;
  holdPwm = c.pwmHold;
  sanitizeStepperRuntimeParams();
  resetStepperPID();
}

// Utility: auto stepper capture seed.
void autoStepperCaptureSeed() {
  autoStepper.seed.kp = stepper.kp;
  autoStepper.seed.ki = stepper.ki;
  autoStepper.seed.kd = stepper.kd;
  autoStepper.seed.minStepUs = minStepDelayUs;
  autoStepper.seed.maxStepUs = maxStepDelayUs;
  autoStepper.seed.stopEnter = stepperStopEnter;
  autoStepper.seed.stopExit = stepperStopExit;
  autoStepper.seed.pwmFar = movePwmFar;
  autoStepper.seed.pwmNear = movePwmNear;
  autoStepper.seed.pwmHold = holdPwm;
}

// Utility: auto stepper build candidates.
uint8_t autoStepperBuildCandidates(AutoStepperStage stage, AutoStepperCandidate* out, uint8_t maxCount) {
  uint8_t n = 0;
  AutoStepperCandidate b = autoStepper.seed;
  const uint16_t capFar = getAutoPwmCapFar();
  const uint16_t capNear = getAutoPwmCapNear();
  const uint16_t capHold = getAutoPwmCapHold();
  b.pwmFar = (uint16_t)clampValue((int)b.pwmFar, 0, (int)capFar);
  b.pwmNear = (uint16_t)clampValue((int)b.pwmNear, 0, (int)capNear);
  b.pwmHold = (uint16_t)clampValue((int)b.pwmHold, 0, (int)capHold);

  if (stage == AUTO_STAGE_TORQUE) {
    const int16_t pwmNearDelta[] = { -24, -8, 12, 28};
    const int16_t pwmFarDelta[]  = { -28, -10, 14, 30};
    const int16_t pwmHoldDelta[] = { -12, -4, 6, 14};
    const uint16_t minVals[] = {2600, 2200, 1800, 1500};
    const uint16_t maxVals[] = {9200, 7600, 6200, 5200};
    for (uint8_t i = 0; i < 4 && n < maxCount; i++) {
      AutoStepperCandidate c = b;
      c.ki = 0.0f;
      c.kd = max(0.10f, b.kd);
      c.pwmNear = (uint16_t)clampValue((int)b.pwmNear + pwmNearDelta[i], 90, (int)capNear);
      c.pwmFar  = (uint16_t)clampValue((int)b.pwmFar  + pwmFarDelta[i], (int)c.pwmNear, (int)capFar);
      c.pwmHold = (uint16_t)clampValue((int)b.pwmHold + pwmHoldDelta[i], 70, (int)capHold);
      if (c.pwmNear < c.pwmHold + 12) c.pwmNear = (uint16_t)clampValue((int)c.pwmHold + 12, 90, (int)capNear);
      if (c.pwmFar < c.pwmNear + 12) c.pwmFar = (uint16_t)clampValue((int)c.pwmNear + 12, (int)c.pwmNear, (int)capFar);
      c.minStepUs = minVals[i];
      c.maxStepUs = maxVals[i];
      out[n++] = c;
    }
  } else if (stage == AUTO_STAGE_BANDS) {
    AutoStepperCandidate base = autoStepper.best.valid ? autoStepper.best.params : b;
    const float enterBase = clampValue(base.stopEnter, 0.6f, 8.0f);
    const float exitBase  = clampValue(base.stopExit, 1.2f, 14.0f);
    const float enterDelta[] = { -0.4f, -0.1f, 0.2f, 0.5f};
    const float exitDelta[]  = { -0.7f, -0.2f, 0.4f, 0.9f};
    for (uint8_t i = 0; i < 4 && n < maxCount; i++) {
      AutoStepperCandidate c = base;
      c.stopEnter = clampValue(enterBase + enterDelta[i], 0.5f, 12.0f);
      c.stopExit = clampValue(exitBase + exitDelta[i], c.stopEnter + 0.5f, 18.0f);
      out[n++] = c;
    }
  } else if (stage == AUTO_STAGE_PID) {
    const float kpScale[] = {0.80f, 0.95f, 1.08f, 1.20f};
    const float kdVals[]  = {0.10f, 0.18f, 0.26f, 0.34f};
    for (uint8_t i = 0; i < 4 && n < maxCount; i++) {
      AutoStepperCandidate c = autoStepper.best.valid ? autoStepper.best.params : b;
      c.kp = clampValue(c.kp * kpScale[i], 0.1f, 10.0f);
      c.ki = 0.0f;
      c.kd = kdVals[i];
      out[n++] = c;
    }
  }
  return n;
}

// Utility: auto stepper prepare stage.
void autoStepperPrepareStage(AutoStepperStage stage) {
  autoStepper.stage = stage;
  autoStepper.cachedStage = AUTO_STAGE_IDLE;
  autoStepper.candidateIndex = 0;
  autoStepper.candidateCount = 0;
  autoStepper.motionIndex = 0;
  autoStepper.motionRunning = false;
  autoStepper.motionMoved = false;
  autoStepper.stageStartMs = millis();
  autoStepper.lastProgressMs = 0;
  memset(autoStepper.candidates, 0, sizeof(autoStepper.candidates));

  if (stage == AUTO_STAGE_TORQUE) {
    autoStepper.testMotions[0] = 18.0f;
    autoStepper.testMotions[1] = 54.0f;
    autoStepper.testMotions[2] = 108.0f;
    autoStepper.testMotions[3] = 180.0f;
    autoStepper.motionCount = 4;
  } else if (stage == AUTO_STAGE_BANDS) {
    autoStepper.testMotions[0] = 36.0f;
    autoStepper.testMotions[1] = 120.0f;
    autoStepper.testMotions[2] = 220.0f;
    autoStepper.motionCount = 3;
  } else if (stage == AUTO_STAGE_PID) {
    autoStepper.testMotions[0] = 60.0f;
    autoStepper.testMotions[1] = 180.0f;
    autoStepper.testMotions[2] = 260.0f;
    autoStepper.testMotions[3] = 300.0f;
    autoStepper.motionCount = 4;
  } else {
    autoStepper.motionCount = 0;
  }

  Serial.print(F("AUTOSTEPPER STAGE="));
  Serial.println((int)stage);
}

// Utility: auto stepper start.
void autoStepperStart() {
  if (!rt.configMode) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
    return;
  }

  if (ch0DriveMode != CH0_MODE_STEPPER) {
    Serial.println(F("DENIED: AUTOSTEPPER REQUIRES CH0MODE=STEPPER"));
    return;
  }

  if (autoStepper.active) {
    Serial.println(F("AUTOSTEPPER ALREADY ACTIVE"));
    return;
  }

  sanitizeStepperRuntimeParams();
  readStepperAngle();

  if (stepperOneTurnSpanRaw < 20) {
    Serial.println(F("DENIED: SPANRAW TOO SMALL - RUN LIMITSTEPPERCALIB FIRST"));
    return;
  }

  if (!stepperFeedbackInsideWindow) {
    Serial.println(F("WARNING: FEEDBACK OUTSIDE CALIBRATED WINDOW - AUTOSTEPPER MAY FAIL"));
  }

  if (!rt.active) enterOperationMode();

  autoStepperCaptureSeed();
  autoStepper.best.valid = false;
  autoStepper.best.score = -1000000000.0f;
  autoStepper.modeAttemptIndex = 0;
  autoStepper.originalMode = stepperDriveMode;
  autoStepper.cachedStage = AUTO_STAGE_IDLE;
  autoStepper.candidateIndex = 0;
  autoStepper.candidateCount = 0;
  autoStepper.motionIndex = 0;
  autoStepper.motionCount = 0;
  autoStepper.motionRunning = false;
  autoStepper.motionMoved = false;
  memset(autoStepper.candidates, 0, sizeof(autoStepper.candidates));
  autoStepper.active = true;
  autoStepper.stopRequested = false;
  rt.stepperArmed = true;
  rt.servoOutputsArmed = false;
  if (!collisionRuntimeEnabled) {
    collisionFlag = false;
    invalidateCollisionPoseCache();
  }
  readStepperAngle();
  stepper.targetAngle = stepper.currentAngle;
  servos.target[0] = stepper.currentAngle;
  servos.actual[0] = stepper.currentAngle;
  resetStepperPID();
  autoStepperPrepareStage(AUTO_STAGE_TORQUE);
  Serial.print(F("AUTOSTEPPER STARTED SAFE_RANGE=")); Serial.print(getAutoStepperSafeMinAngle(), 1); Serial.print(F("..")); Serial.println(getAutoStepperSafeMaxAngle(), 1);
  Serial.print(F("AUTOSTEPPER STEPMODE=")); Serial.print((int)stepperDriveMode); Serial.print(F(" [")); Serial.print(getStepperDriveModeName(stepperDriveMode)); Serial.println(F("]"));
  Serial.print(F("AUTOSTEPPER RAWADC=")); Serial.print(readStepperRawADC());
  Serial.print(F(" ZEROADC=")); Serial.print(stepperCalibrationZeroADC);
  Serial.print(F(" SPANRAW=")); Serial.println(stepperOneTurnSpanRaw);
  Serial.println(F("AUTOSTEPPER WILL TRY OTHER STEPPER MODES AUTOMATICALLY IF THE ACTIVE MODE DOES NOT RESPOND"));
  Serial.println(F("AUTOSTEPPER USES LARGE SWEEPS TO CHARACTERIZE START, BRAKE, NEAR AND HOLD BEHAVIOR"));
}

// Utility: auto stepper stop.
void autoStepperStop(bool userRequested) {
  bool hadSession = autoStepper.active || autoStepper.candidateCount > 0 || autoStepper.best.valid;

  if (hadSession) {
    setStepperDriveMode(autoStepper.originalMode, false);
    autoStepperApplyCandidate(autoStepper.seed);
  }

  autoStepper.active = false;
  autoStepper.stopRequested = false;
  autoStepper.stage = AUTO_STAGE_IDLE;
  autoStepper.cachedStage = AUTO_STAGE_IDLE;
  autoStepper.candidateIndex = 0;
  autoStepper.candidateCount = 0;
  autoStepper.motionIndex = 0;
  autoStepper.motionCount = 0;
  autoStepper.motionRunning = false;
  autoStepper.motionMoved = false;
  autoStepper.best.valid = false;
  autoStepper.best.score = -1000000000.0f;
  memset(autoStepper.candidates, 0, sizeof(autoStepper.candidates));

  rt.stepperArmed = false;
  rt.servoOutputsArmed = false;
  rt.active = false;
  rt.configMode = true;

  readStepperAngle();
  stepper.targetAngle = stepper.currentAngle;
  servos.target[0] = stepper.currentAngle;
  servos.actual[0] = stepper.currentAngle;
  resetStepperPID();

  releaseStepperCoils();
  stepperSetDriverPWM(0);
  stepperMotor.setSpeed(0.0f);
  stepperMotor.disableOutputs();

  if (userRequested) {
    Serial.println(F("AUTOSTEPPER STOPPED - ORIGINAL VALUES RESTORED"));
    Serial.println(F("CFG MODE REMAINS ACTIVE"));
  }
}

// Utility: auto stepper start motion.
void autoStepperStartMotion(float deltaDeg) {
  readStepperAngle();
  autoStepper.motionStartAngle = stepper.currentAngle;
  float safeMin = getAutoStepperSafeMinAngle();
  float safeMax = getAutoStepperSafeMaxAngle();
  float requested = autoStepper.motionStartAngle + deltaDeg;
  if (requested < safeMin || requested > safeMax) {
    float plusRoom = safeMax - autoStepper.motionStartAngle;
    float minusRoom = autoStepper.motionStartAngle - safeMin;
    if (fabsf(plusRoom) >= fabsf(minusRoom)) requested = clampValue((float)(autoStepper.motionStartAngle + fabsf(deltaDeg)), safeMin, safeMax);
    else requested = clampValue((float)(autoStepper.motionStartAngle - fabsf(deltaDeg)), safeMin, safeMax);
  }
  autoStepper.motionTargetAngle = clampValue(requested, safeMin, safeMax);
  stepper.targetAngle = autoStepper.motionTargetAngle;
  servos.target[0] = autoStepper.motionTargetAngle;
  autoStepper.motionMaxOvershoot = 0.0f;
  autoStepper.motionSumAbsError = 0.0f;
  autoStepper.motionJitter = 0.0f;
  autoStepper.holdMeanAbsError = 0.0f;
  autoStepper.holdMinAngle = 999999.0f;
  autoStepper.holdMaxAngle = -999999.0f;
  autoStepper.motionSamples = 0;
  autoStepper.holdSamples = 0;
  autoStepper.motionMoved = false;
  autoStepper.motionStartMoveErr = fabs(deltaDeg);
  autoStepper.motionStartMs = millis();
  autoStepper.settleStartMs = 0;
  autoStepper.motionSettleMs = 0;
  autoStepper.motionRunning = true;
}

// Utility: auto stepper finish motion.
void autoStepperFinishMotion(bool success) {
  float avgErr = (autoStepper.motionSamples > 0) ? (autoStepper.motionSumAbsError / (float)autoStepper.motionSamples) : 999.0f;
  float holdAvgErr = (autoStepper.holdSamples > 0) ? (autoStepper.holdMeanAbsError / (float)autoStepper.holdSamples) : avgErr;
  float holdP2P = (autoStepper.holdSamples > 1) ? (autoStepper.holdMaxAngle - autoStepper.holdMinAngle) : 0.0f;
  float jitterPenalty = autoStepper.motionJitter;
  if (holdP2P <= 0.9f && holdAvgErr <= max(0.8f, stepperStopEnter * 0.85f)) {
    jitterPenalty *= 0.30f;
  } else if (holdP2P <= 1.8f && holdAvgErr <= max(1.1f, stepperStopExit * 0.65f)) {
    jitterPenalty *= 0.60f;
  }

  float score = 0.0f;
  score -= avgErr * 22.0f;
  score -= autoStepper.motionMaxOvershoot * 22.0f;
  score -= jitterPenalty * 18.0f;
  score -= autoStepper.motionStartMoveErr * 28.0f;
  score -= holdAvgErr * 14.0f;
  score -= holdP2P * 10.0f;
  score -= ((float)autoStepper.motionSettleMs) * 0.01f;
  if (!success) score -= 1800.0f;

  if (success && score > autoStepper.best.score) {
    autoStepper.best.valid = true;
    autoStepper.best.score = score;
    autoStepper.best.params = autoStepper.current;
  }

  autoStepper.motionRunning = false;
  autoStepper.motionIndex++;
}

// Utility: auto stepper finalize.
void autoStepperFinalize() {
  if (!autoStepper.best.valid && autoStepperAdvanceModeAttempt()) {
    return;
  }

  autoStepper.active = false;
  autoStepper.stopRequested = false;
  autoStepper.cachedStage = AUTO_STAGE_IDLE;
  autoStepper.candidateCount = 0;
  autoStepper.motionRunning = false;
  memset(autoStepper.candidates, 0, sizeof(autoStepper.candidates));
  rt.stepperArmed = false;
  rt.servoOutputsArmed = false;
  rt.active = false;
  rt.configMode = true;
  if (autoStepper.best.valid) {
    autoStepperApplyCandidate(autoStepper.best.params);
    readStepperAngle();
    stepper.targetAngle = stepper.currentAngle;
    servos.target[0] = stepper.currentAngle;
    servos.actual[0] = stepper.currentAngle;
    resetStepperPID();
    Serial.println(F("AUTOSTEPPER DONE"));
    Serial.println(F("FOUND VALUES (NOT SAVED):"));
    Serial.print(F("CH0MODE=")); Serial.println((int)ch0DriveMode);
    Serial.print(F("CH0MODE_NAME: ")); Serial.println(getCh0DriveModeName(ch0DriveMode));
    Serial.print(F("STEPMODE=")); Serial.println((int)stepperDriveMode);
    Serial.print(F("STEPMODE_NAME: ")); Serial.println(getStepperDriveModeName(stepperDriveMode));
    Serial.print(F("STEPRAWADC=")); Serial.println(readStepperRawADC());
    Serial.print(F("STEPWIN=")); Serial.println(stepperFeedbackInsideWindow ? 1 : 0);
    Serial.print(F("MINSTEPUS=")); Serial.println(minStepDelayUs);
    Serial.print(F("MAXSTEPUS=")); Serial.println(maxStepDelayUs);
    Serial.print(F("PWMFAR=")); Serial.println(movePwmFar);
    Serial.print(F("PWMNEAR=")); Serial.println(movePwmNear);
    Serial.print(F("PWMHOLD=")); Serial.println(holdPwm);
    Serial.print(F("STOPMOTORENTER=")); Serial.println(stepperStopEnter, 3);
    Serial.print(F("STOPMOTOREXIT=")); Serial.println(stepperStopExit, 3);
    Serial.print(F("AUTO PWM CAPS F/N/H=")); Serial.print(getAutoPwmCapFar()); Serial.print('/'); Serial.print(getAutoPwmCapNear()); Serial.print('/'); Serial.println(getAutoPwmCapHold());
    configSessionDirty = true;
    Serial.println(F("BEST PARAMETERS ARE READY IN CFG - USE SHOW TO REVIEW AND SAVE TO STORE EEPROM"));
  } else {
    setStepperDriveMode(autoStepper.originalMode, false);
    autoStepperApplyCandidate(autoStepper.seed);
    Serial.println(F("AUTOSTEPPER FAILED - SEED RESTORED"));
    Serial.println(F("CFG MODE REMAINS ACTIVE - REVIEW VALUES, SAVE IF DESIRED, OR EXIT"));
  }
  releaseStepperCoils();
  stepperSetDriverPWM(0);
  stepperMotor.setSpeed(0.0f);
}

// Prints auto stepper status.
void printAutoStepperStatus() {
  Serial.print(F("AUTOSTEPPER ACTIVE=")); Serial.println(autoStepper.active ? 1 : 0);
  Serial.print(F("STAGE=")); Serial.println((int)autoStepper.stage);
  Serial.print(F("CANDIDATE=")); Serial.print(autoStepper.candidateIndex + 1); Serial.print(F("/")); Serial.println(autoStepper.candidateCount);
  Serial.print(F("MOTION=")); Serial.print(autoStepper.motionIndex + 1); Serial.print(F("/")); Serial.println(autoStepper.motionCount);
  Serial.print(F("BESTVALID=")); Serial.println(autoStepper.best.valid ? 1 : 0);
  Serial.print(F("BESTSCORE=")); Serial.println(autoStepper.best.score, 2);
  Serial.print(F("CH0MODE=")); Serial.print((int)ch0DriveMode); Serial.print(F(" [")); Serial.print(getCh0DriveModeName(ch0DriveMode)); Serial.println(F("]"));
  Serial.print(F("STEPMODE=")); Serial.print((int)stepperDriveMode); Serial.print(F(" [")); Serial.print(getStepperDriveModeName(stepperDriveMode)); Serial.println(F("]"));
  Serial.print(F("RAWADC=")); Serial.println(readStepperRawADC());
  Serial.print(F("ZEROADC=")); Serial.println(stepperCalibrationZeroADC);
  Serial.print(F("SPANRAW=")); Serial.println(stepperOneTurnSpanRaw);
  Serial.print(F("ANGLE=")); Serial.println(stepper.currentAngle, 3);
  Serial.print(F("TARGET=")); Serial.println(stepper.targetAngle, 3);
  Serial.print(F("WIN=")); Serial.println(stepperFeedbackInsideWindow ? 1 : 0);
  Serial.print(F("MINSTEPUS=")); Serial.println(minStepDelayUs);
  Serial.print(F("MAXSTEPUS=")); Serial.println(maxStepDelayUs);
  Serial.print(F("ALPHAANG=")); Serial.println(angleFilterAlpha, 3);
  Serial.print(F("PWMFAR=")); Serial.println(movePwmFar);
  Serial.print(F("PWMNEAR=")); Serial.println(movePwmNear);
  Serial.print(F("PWMHOLD=")); Serial.println(holdPwm);
  Serial.print(F("STOPMOTORENTER=")); Serial.println(stepperStopEnter, 3);
  Serial.print(F("STOPMOTOREXIT=")); Serial.println(stepperStopExit, 3);
  Serial.print(F("AUTO_PWM_CAPS=")); Serial.print(getAutoPwmCapFar()); Serial.print('/'); Serial.print(getAutoPwmCapNear()); Serial.print('/'); Serial.println(getAutoPwmCapHold());
  Serial.print(F("SAFE_MIN=")); Serial.println(getAutoStepperSafeMinAngle(), 1);
  Serial.print(F("SAFE_MAX=")); Serial.println(getAutoStepperSafeMaxAngle(), 1);
}

// Utility: auto stepper update.
void autoStepperUpdate() {
  if (!autoStepper.active) return;

  if (autoStepper.cachedStage != autoStepper.stage) {
    autoStepper.cachedStage = autoStepper.stage;
    autoStepper.candidateCount = autoStepperBuildCandidates(autoStepper.stage, autoStepper.candidates, 4);
    autoStepper.candidateIndex = 0;
    autoStepper.motionIndex = 0;
    if (autoStepper.candidateCount > 0) {
      autoStepper.current = autoStepper.candidates[0];
      autoStepperApplyCandidate(autoStepper.current);
    }
  }

  if (autoStepper.stopRequested) {
    autoStepperFinalize();
    return;
  }

  if (autoStepper.stage == AUTO_STAGE_DONE) {
    autoStepperFinalize();
    return;
  }

  if (autoStepper.candidateIndex >= autoStepper.candidateCount) {
    if (autoStepper.stage == AUTO_STAGE_TORQUE) autoStepperPrepareStage(AUTO_STAGE_BANDS);
    else autoStepper.stage = AUTO_STAGE_DONE;
    autoStepper.cachedStage = AUTO_STAGE_IDLE;
    return;
  }

  if (!autoStepper.motionRunning) {
    if (autoStepper.motionIndex >= autoStepper.motionCount) {
      autoStepper.candidateIndex++;
      autoStepper.motionIndex = 0;
      if (autoStepper.candidateIndex < autoStepper.candidateCount) {
        autoStepper.current = autoStepper.candidates[autoStepper.candidateIndex];
        autoStepperApplyCandidate(autoStepper.current);
      }
      return;
    }

    float delta = autoStepper.testMotions[autoStepper.motionIndex];
    float safeMin = getAutoStepperSafeMinAngle();
    float safeMax = getAutoStepperSafeMaxAngle();
    float plusRoom = safeMax - stepper.currentAngle;
    float minusRoom = stepper.currentAngle - safeMin;
    float sign = (plusRoom >= minusRoom) ? 1.0f : -1.0f;
    if ((autoStepper.motionIndex & 1) != 0) sign = -sign;
    autoStepperStartMotion(delta * sign);
    return;
  }

  unsigned long nowMs = millis();
  float absError = fabs(stepper.targetAngle - stepper.currentAngle);
  float moved = fabs(stepper.currentAngle - autoStepper.motionStartAngle);
  autoStepper.motionSamples++;
  autoStepper.motionSumAbsError += absError;

  float overshoot = 0.0f;
  if ((autoStepper.motionTargetAngle >= autoStepper.motionStartAngle && stepper.currentAngle > autoStepper.motionTargetAngle) ||
      (autoStepper.motionTargetAngle <= autoStepper.motionStartAngle && stepper.currentAngle < autoStepper.motionTargetAngle)) {
    overshoot = fabs(stepper.currentAngle - autoStepper.motionTargetAngle);
    if (overshoot > autoStepper.motionMaxOvershoot) autoStepper.motionMaxOvershoot = overshoot;
  }

  if (!autoStepper.motionMoved && moved >= 1.0f) {
    autoStepper.motionMoved = true;
    autoStepper.motionStartMoveErr = absError;
  }

  float instJitter = fabs(stepper.filteredSpeed);
  bool buzzNoMove = ((nowMs - autoStepper.motionStartMs) > 1800UL) && (moved < 1.2f);
  bool oscillatory = ((nowMs - autoStepper.motionStartMs) > 2200UL) && (moved < 4.0f) && (autoStepper.motionSamples > 16) && (instJitter > 12.0f);
  bool extremeOvershoot = autoStepper.motionMaxOvershoot > 24.0f;
  bool safeLimitHit = (stepper.currentAngle <= (getAutoStepperSafeMinAngle() + 1.5f)) || (stepper.currentAngle >= (getAutoStepperSafeMaxAngle() - 1.5f));
  bool feedbackLost = ((nowMs - autoStepper.motionStartMs) > 1200UL) && !stepperFeedbackInsideWindow;
  bool timeout = (nowMs - autoStepper.motionStartMs) > 6200UL;
  bool noResponseEarly = (buzzNoMove || (timeout && moved < 1.0f) || feedbackLost) &&
                         (autoStepper.stage == AUTO_STAGE_TORQUE) &&
                         (autoStepper.motionIndex == 0) &&
                         (autoStepper.candidateIndex == 0) &&
                         !autoStepper.best.valid;
  if (noResponseEarly && autoStepperAdvanceModeAttempt()) {
    return;
  }
  if (buzzNoMove || oscillatory || extremeOvershoot || safeLimitHit || feedbackLost || timeout) {
    autoStepperFinishMotion(false);
    return;
  }

  float holdBandEval = max(0.7f, stepperStopEnter * 1.15f);
  if (absError <= holdBandEval) {
    if (autoStepper.settleStartMs == 0) autoStepper.settleStartMs = nowMs;

    if ((nowMs - autoStepper.settleStartMs) >= 180UL) {
      autoStepper.holdSamples++;
      autoStepper.holdMeanAbsError += absError;
      if (stepper.currentAngle < autoStepper.holdMinAngle) autoStepper.holdMinAngle = stepper.currentAngle;
      if (stepper.currentAngle > autoStepper.holdMaxAngle) autoStepper.holdMaxAngle = stepper.currentAngle;

      float holdP2PNow = (autoStepper.holdSamples > 1) ? (autoStepper.holdMaxAngle - autoStepper.holdMinAngle) : 0.0f;
      float residualAllowance = 1.20f + (float)autoStepper.current.pwmHold * 0.0025f;
      autoStepper.motionJitter += max(0.0f, holdP2PNow - residualAllowance) * 0.25f;

      if ((nowMs - autoStepper.settleStartMs) >= 520UL) {
        float holdAvgErrNow = autoStepper.holdMeanAbsError / (float)max((uint16_t)1, autoStepper.holdSamples);
        bool acceptableResidual = (holdP2PNow <= residualAllowance) && (holdAvgErrNow <= max(0.9f, stepperStopExit * 0.60f));
        bool lowResidual = (holdP2PNow <= (residualAllowance * 0.55f)) && (holdAvgErrNow <= max(0.6f, stepperStopEnter));
        if (lowResidual || acceptableResidual) {
          autoStepper.motionSettleMs = nowMs - autoStepper.motionStartMs;
          autoStepperFinishMotion(true);
          return;
        }
      }
    }
  } else {
    autoStepper.motionJitter += instJitter * 0.08f;
    autoStepper.settleStartMs = 0;
    autoStepper.holdSamples = 0;
    autoStepper.holdMeanAbsError = 0.0f;
    autoStepper.holdMinAngle = 999999.0f;
    autoStepper.holdMaxAngle = -999999.0f;
  }

  if ((nowMs - autoStepper.lastProgressMs) >= 600UL) {
    autoStepper.lastProgressMs = nowMs;
    Serial.print(F("AUTOSTEPPER STAGE=")); Serial.print((int)autoStepper.stage);
    Serial.print(F(" CAND=")); Serial.print(autoStepper.candidateIndex + 1);
    Serial.print(F("/")); Serial.print(autoStepper.candidateCount);
    Serial.print(F(" MOV=")); Serial.print(autoStepper.motionIndex + 1);
    Serial.print(F("/")); Serial.print(autoStepper.motionCount);
    Serial.print(F(" ANG=")); Serial.print(stepper.currentAngle, 2);
    Serial.print(F(" TGT=")); Serial.print(stepper.targetAngle, 2);
    Serial.print(F(" ERR=")); Serial.println(absError, 2);
  }
}





// Servo helper for angle within configured limits.
bool servoAngleWithinConfiguredLimits(uint8_t ch, float value, float tol) {
  if (!isServoChannel(ch)) return true;
  float mn = getServoMinLimit(ch) - tol;
  float mx = getServoMaxLimit(ch) + tol;
  return value >= mn && value <= mx;
}



// Utility: pose violates configured limits.
bool poseViolatesConfiguredLimits(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg) {
  uint8_t baseCh = getManipulatorMemberChannel(MANIP_MEMBER_BASE);
  if (baseCh == 0) {
    if (baseDeg < STEPPER_INPUT_MIN || baseDeg > STEPPER_INPUT_MAX) return true;
  } else if (!servoAngleWithinConfiguredLimits(baseCh, baseDeg)) return true;

  if (!servoAngleWithinConfiguredLimits(getManipulatorMemberChannel(MANIP_MEMBER_UPPER), upperDeg)) return true;
  if (!servoAngleWithinConfiguredLimits(getManipulatorMemberChannel(MANIP_MEMBER_FORE), foreDeg)) return true;
  if (!servoAngleWithinConfiguredLimits(getManipulatorMemberChannel(MANIP_MEMBER_FOREARM_ROLL), forearmRollDeg)) return true;
  if (!servoAngleWithinConfiguredLimits(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_PITCH), wristPitchDeg)) return true;
  if (!servoAngleWithinConfiguredLimits(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_ROLL), wristRotDeg)) return true;
  if (!servoAngleWithinConfiguredLimits(getManipulatorMemberChannel(MANIP_MEMBER_GRIPPER), gripDeg)) return true;
  return false;
}



// Clamps servo target to limits.
float clampServoTargetToLimits(uint8_t ch, float value) {
  if (!isServoChannel(ch)) return value;
  return clampValue(value, getServoMinLimit(ch), getServoMaxLimit(ch));
}



// Returns limited servo target.
float getLimitedServoTarget(uint8_t ch) {
  if (!isServoChannel(ch)) return 0.0f;
  return clampServoTargetToLimits(ch, clampServoInputToChannelRange(ch, servos.target[ch]));
}



// Returns structure reach factor.
float getStructureReachFactor() {
  float totalLen = armUpperH + armForeH + armWristH + armFingerH;
  const float nominalLen = 61.6f + 33.6f + 33.6f + 28.0f;
  float factor = totalLen / nominalLen;
  return clampValue(factor, 0.60f, 1.80f);
}



// Returns structure mass factor.
float getStructureMassFactor() {
  float sum = armUpperW + armUpperD + armForeW + armForeD + armWristW + armWristD + armFingerW + armFingerD;
  const float nominal = 28.0f + 18.2f + 16.8f + 15.4f + 11.2f + 14.0f + 5.6f + 4.76f;
  float factor = sum / nominal;
  return clampValue(factor, 0.60f, 1.80f);
}



// Utility: try set stabilized target.
bool trySetStabilizedTarget(uint8_t memberIdx, float desired) {
  if (memberIdx >= MANIP_MEMBER_COUNT || memberIdx == MANIP_MEMBER_BASE) return false;
  uint8_t ch = getManipulatorMemberChannel(memberIdx);
  if (!isServoChannel(ch)) return false;
  float limitedMember = clampManipulatorMemberTargetToLimits(memberIdx, desired);
  if (collisionRuntimeEnabled && !candidateMoveAllowed(memberIdx, limitedMember)) {
    collisionFlag = true;
    return false;
  }
  float servoTarget = convertManipulatorMemberToServoDeg(memberIdx, limitedMember);
  servos.target[ch] = clampServoTargetToLimits(ch, clampServoInputToChannelRange(ch, servoTarget));
  return true;
}







// Utility: process incoming stream.
void processIncomingStream(Stream& port, char* buffer, uint8_t bufferSize, uint8_t& index, uint8_t sourceId) {
  // Keep command RX bounded per loop so the telemetry cadence stays stable even
  // when the desktop is continuously streaming pose updates.
  const uint8_t maxCommandsPerPass = 2;
  uint8_t commandsProcessed = 0;

  while (port.available() > 0 && commandsProcessed < maxCommandsPerPass) {
    char c = (char)port.read();
    if (c == '\r' || c == '\n') {
      if (index > 0) {
        buffer[index] = '\0';
        currentTextCommandSource = sourceId;
        handleCommand(buffer);
        index = 0;
        commandsProcessed++;
      }
    } else if (c >= 32 && c <= 126) {
      if (index + 1 < bufferSize) {
        buffer[index++] = c;
      }
    }
  }
}




// Manual PWM outputs must not be rewritten by arm joint motion.
// This compatibility stub intentionally does nothing; CH12..CH15 only change
// through explicit PWM commands or configured startup defaults.
void updateOptimizePowerForServo(uint8_t ch, float prevActual, float newActual) {
  (void)ch;
  (void)prevActual;
  (void)newActual;
}



// Utility: sanitize collision evaluation state.
void sanitizeCollisionEvaluationState() {
  syncProcessingRobotProfiles();
  sanitizeManipulatorProfile();
  sanitizeManipulatorHomePose(manipHomePose);
  sanitizeArmGeometryConfig(armGeom);
}

// Utility: set collision runtime enabled safely.
void setCollisionRuntimeEnabledSafely(bool enable, bool evaluateImmediately) {
  sanitizeCollisionEvaluationState();
  collisionRuntimeEnabled = enable;
  invalidateCollisionPoseCache();

  if (!collisionRuntimeEnabled || robotControlMode != ROBOT_MODE_MANIPULATOR) {
    collisionFlag = false;
    return;
  }

  collisionFlag = false;
  if (evaluateImmediately) {
    applyCollisionGuard();
  }
}

// Utility: pose has collision.
bool poseHasCollision(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg) {
  if (robotControlMode != ROBOT_MODE_MANIPULATOR) return false;

  sanitizeCollisionEvaluationState();

  if (poseViolatesConfiguredLimits(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg)) {
    return true;
  }

  uint8_t baseCh = getManipulatorMemberChannel(MANIP_MEMBER_BASE);
  baseDeg = (baseCh == 0)
            ? clampValue(baseDeg, STEPPER_INPUT_MIN, STEPPER_INPUT_MAX)
            : clampServoTargetToLimits(baseCh, clampServoInputToChannelRange(baseCh, baseDeg));
  upperDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_UPPER), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_UPPER), upperDeg));
  foreDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_FORE), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_FORE), foreDeg));
  forearmRollDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_FOREARM_ROLL), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_FOREARM_ROLL), forearmRollDeg));
  wristPitchDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_PITCH), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_PITCH), wristPitchDeg));
  wristRotDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_ROLL), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_ROLL), wristRotDeg));
  gripDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_GRIPPER), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_GRIPPER), gripDeg));

  return armPoseHasOOBBCollision(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
}




// Utility: invalidate collision pose cache.
void invalidateCollisionPoseCache() {
  collisionPoseCacheValid = false;
  lastCollisionGuardEvalMs = 0;
}

// Utility: capture limited actual collision pose.
bool captureLimitedActualCollisionPose(float& baseDeg, float& upperDeg, float& foreDeg, float& forearmRollDeg, float& wristPitchDeg, float& wristRotDeg, float& gripDeg) {
  sanitizeCollisionEvaluationState();
  baseDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_BASE);
  upperDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_UPPER);
  foreDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_FORE);
  forearmRollDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_FOREARM_ROLL);
  wristPitchDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_PITCH);
  wristRotDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_ROLL);
  gripDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_GRIPPER);
  return true;
}

// Utility: collision pose changed.
bool collisionPoseChanged(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg) {
  if (!collisionPoseCacheValid) return true;
  int16_t baseCenti = packCentiDeg16(baseDeg);
  int16_t upperCenti = packCentiDeg16(upperDeg);
  int16_t foreCenti = packCentiDeg16(foreDeg);
  int16_t forearmRollCenti = packCentiDeg16(forearmRollDeg);
  int16_t wristPitchCenti = packCentiDeg16(wristPitchDeg);
  int16_t wristRotCenti = packCentiDeg16(wristRotDeg);
  int16_t gripCenti = packCentiDeg16(gripDeg);
  if (abs(baseCenti - collisionPoseCache.baseCenti) > COLLISION_POSE_EPS_CENTI) return true;
  if (abs(upperCenti - collisionPoseCache.upperCenti) > COLLISION_POSE_EPS_CENTI) return true;
  if (abs(foreCenti - collisionPoseCache.foreCenti) > COLLISION_POSE_EPS_CENTI) return true;
  if (abs(forearmRollCenti - collisionPoseCache.forearmRollCenti) > COLLISION_POSE_EPS_CENTI) return true;
  if (abs(wristPitchCenti - collisionPoseCache.wristPitchCenti) > COLLISION_POSE_EPS_CENTI) return true;
  if (abs(wristRotCenti - collisionPoseCache.wristRotCenti) > COLLISION_POSE_EPS_CENTI) return true;
  if (abs(gripCenti - collisionPoseCache.gripCenti) > COLLISION_POSE_EPS_CENTI) return true;
  return false;
}

// Utility: updating the cached collision pose.
void updateCollisionPoseCache(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg) {
  collisionPoseCache.baseCenti = packCentiDeg16(baseDeg);
  collisionPoseCache.upperCenti = packCentiDeg16(upperDeg);
  collisionPoseCache.foreCenti = packCentiDeg16(foreDeg);
  collisionPoseCache.forearmRollCenti = packCentiDeg16(forearmRollDeg);
  collisionPoseCache.wristPitchCenti = packCentiDeg16(wristPitchDeg);
  collisionPoseCache.wristRotCenti = packCentiDeg16(wristRotDeg);
  collisionPoseCache.gripCenti = packCentiDeg16(gripDeg);
  collisionPoseCacheValid = true;
  lastCollisionGuardEvalMs = millis();
}

// Utility: candidate pose collision guard.
// A pose that creates the first collision is blocked. If the arm is already
// colliding, poses that reduce or keep the collision severity are accepted so
// the operator can back out of the collision instead of being trapped.
bool collisionGuardAllowsCandidatePose(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg) {
  if (!collisionRuntimeEnabled || robotControlMode != ROBOT_MODE_MANIPULATOR) {
    collisionFlag = false;
    return true;
  }

  sanitizeCollisionEvaluationState();

  uint8_t proposedSeverity = poseCollisionSeverity(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
  if (proposedSeverity == 0u) {
    collisionFlag = false;
    updateCollisionPoseCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
    return true;
  }

  // 0xFF means configured joint limits were violated, not a recoverable
  // self-collision. Keep those blocked.
  if (proposedSeverity == 0xFFu) {
    collisionFlag = true;
    emitCollisionBlockedNotice();
    updateCollisionPoseCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
    return false;
  }

  float curBase, curUpper, curFore, curForearmRoll, curWristPitch, curWristRot, curGrip;
  captureLimitedActualCollisionPose(curBase, curUpper, curFore, curForearmRoll, curWristPitch, curWristRot, curGrip);
  uint8_t currentSeverity = poseCollisionSeverity(curBase, curUpper, curFore, curForearmRoll, curWristPitch, curWristRot, curGrip);

  if (currentSeverity > 0u && currentSeverity != 0xFFu && proposedSeverity <= currentSeverity) {
    collisionFlag = true;
    updateCollisionPoseCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
    return true;
  }

  collisionFlag = true;
  emitCollisionBlockedNotice();
  updateCollisionPoseCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
  return false;
}

// Utility: candidate move allowed.
bool candidateMoveAllowed(uint8_t memberIdx, float candidate) {
  sanitizeCollisionEvaluationState();

  float base = getManipulatorMemberActualServoDeg(MANIP_MEMBER_BASE);
  float upper = getManipulatorMemberActualServoDeg(MANIP_MEMBER_UPPER);
  float fore = getManipulatorMemberActualServoDeg(MANIP_MEMBER_FORE);
  float forearmRoll = getManipulatorMemberActualServoDeg(MANIP_MEMBER_FOREARM_ROLL);
  float wristPitch = getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_PITCH);
  float wristRot = getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_ROLL);
  float grip = getManipulatorMemberActualServoDeg(MANIP_MEMBER_GRIPPER);

  float candidateServo = convertManipulatorMemberToServoDeg(memberIdx, candidate);
  uint8_t candidateCh = getManipulatorMemberChannel(memberIdx);
  if (memberIdx == MANIP_MEMBER_BASE && candidateCh == 0) {
    if (candidateServo < STEPPER_INPUT_MIN || candidateServo > STEPPER_INPUT_MAX) return false;
  } else if (isServoChannel(candidateCh) && !servoAngleWithinConfiguredLimits(candidateCh, candidateServo)) {
    return false;
  }

  if (memberIdx == MANIP_MEMBER_BASE) {
    base = candidateServo;
  } else if (memberIdx == MANIP_MEMBER_UPPER) {
    upper = candidateServo;
  } else if (memberIdx == MANIP_MEMBER_FORE) {
    fore = candidateServo;
  } else if (memberIdx == MANIP_MEMBER_FOREARM_ROLL) {
    forearmRoll = candidateServo;
  } else if (memberIdx == MANIP_MEMBER_WRIST_PITCH) {
    wristPitch = candidateServo;
  } else if (memberIdx == MANIP_MEMBER_WRIST_ROLL) {
    wristRot = candidateServo;
  } else if (memberIdx == MANIP_MEMBER_GRIPPER) {
    grip = candidateServo;
  }

  return collisionGuardAllowsCandidatePose(base, upper, fore, forearmRoll, wristPitch, wristRot, grip);
}

// Utility: emit collision blocked notice.
void emitCollisionBlockedNotice() {
  // Intentionally silent: collision state travels in telemetry flags/fields only.
  // Do not emit ad-hoc ASCII notices here, otherwise the runtime serial stream gets polluted.
  lastCollisionBlockedNoticeMs = millis();
}

// Applies manipulator pose runtime member values.
bool applyManipulatorPoseRuntimeMemberValues(const int memberVals[7]) {
  if (memberVals == NULL) return false;

  int servoVals[7] = {0, 0, 0, 0, 0, 0, 0};
  bool neutralMask[7] = {false, false, false, false, false, false, false};

  readStepperAngle();
  if (memberVals[0] == 1000) {
    neutralMask[0] = true;
    servoVals[0] = (int)roundf(stepper.currentAngle);
  } else {
    servoVals[0] = (int)roundf(convertManipulatorMemberToServoDeg(MANIP_MEMBER_BASE, (float)memberVals[0]));
  }

  if (memberVals[1] == 1000) {
    neutralMask[1] = true;
    servoVals[1] = 90;
  } else {
    servoVals[1] = (int)roundf(convertManipulatorMemberToServoDeg(MANIP_MEMBER_UPPER, (float)memberVals[1]));
  }

  if (memberVals[2] == 1000) {
    neutralMask[2] = true;
    servoVals[2] = 90;
  } else {
    servoVals[2] = (int)roundf(convertManipulatorMemberToServoDeg(MANIP_MEMBER_FORE, (float)memberVals[2]));
  }

  if (memberVals[3] == 1000) {
    neutralMask[3] = true;
    servoVals[3] = 90;
  } else {
    servoVals[3] = (int)roundf(convertManipulatorMemberToServoDeg(MANIP_MEMBER_FOREARM_ROLL, (float)memberVals[3]));
  }

  if (memberVals[4] == 1000) {
    neutralMask[4] = true;
    servoVals[4] = 90;
  } else {
    servoVals[4] = (int)roundf(convertManipulatorMemberToServoDeg(MANIP_MEMBER_WRIST_PITCH, (float)memberVals[4]));
  }

  if (memberVals[5] == 1000) {
    neutralMask[5] = true;
    servoVals[5] = 90;
  } else {
    servoVals[5] = (int)roundf(convertManipulatorMemberToServoDeg(MANIP_MEMBER_WRIST_ROLL, (float)memberVals[5]));
  }

  if (memberVals[6] == 1000) {
    neutralMask[6] = true;
    servoVals[6] = 90;
  } else {
    servoVals[6] = (int)roundf(convertManipulatorMemberToServoDeg(MANIP_MEMBER_GRIPPER, (float)memberVals[6]));
  }

  return applyManipulatorPoseRuntimeTargets(servoVals, neutralMask);
}

// Applies manipulator pose runtime targets.
bool applyManipulatorPoseRuntimeTargets(const int servoVals[7]) {
  return applyManipulatorPoseRuntimeTargets(servoVals, NULL);
}

// Applies manipulator runtime home pose.
bool applyManipulatorRuntimeHomePose() {
  readStepperAngle();
  sanitizeManipulatorHomePose(manipHomePose);
  int homeVals[7] = {
    manipHomePose[MANIP_MEMBER_BASE],
    manipHomePose[MANIP_MEMBER_UPPER],
    manipHomePose[MANIP_MEMBER_FORE],
    manipHomePose[MANIP_MEMBER_FOREARM_ROLL],
    manipHomePose[MANIP_MEMBER_WRIST_PITCH],
    manipHomePose[MANIP_MEMBER_WRIST_ROLL],
    manipHomePose[MANIP_MEMBER_GRIPPER]
  };
  bool neutralMask[7] = {false, false, false, false, false, false, false};
  return applyManipulatorPoseRuntimeTargets(homeVals, neutralMask);
}

// Applies manipulator pose runtime targets.
bool applyManipulatorPoseRuntimeTargets(const int servoVals[7], const bool neutralMask[7]) {
  if (servoVals == NULL) return false;

  sanitizeCollisionEvaluationState();
  rt.servoOutputsArmed = true;

  readStepperAngle();
  bool baseNeutral = (neutralMask != NULL && neutralMask[0]);
  uint8_t baseCh = getManipulatorMemberChannel(MANIP_MEMBER_BASE);
  float base = baseNeutral
               ? getManipulatorMemberActualServoDeg(MANIP_MEMBER_BASE)
               : ((baseCh == 0)
                  ? clampValue((float)servoVals[0], STEPPER_INPUT_MIN, STEPPER_INPUT_MAX)
                  : clampServoTargetToLimits(baseCh, clampServoInputToChannelRange(baseCh, (float)servoVals[0])));
  float upper = (neutralMask != NULL && neutralMask[1]) ? getManipulatorMemberActualServoDeg(MANIP_MEMBER_UPPER) : clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_UPPER), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_UPPER), (float)servoVals[1]));
  float fore = (neutralMask != NULL && neutralMask[2]) ? getManipulatorMemberActualServoDeg(MANIP_MEMBER_FORE) : clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_FORE), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_FORE), (float)servoVals[2]));
  float forearmRoll = (neutralMask != NULL && neutralMask[3]) ? getManipulatorMemberActualServoDeg(MANIP_MEMBER_FOREARM_ROLL) : clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_FOREARM_ROLL), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_FOREARM_ROLL), (float)servoVals[3]));
  float wristPitch = (neutralMask != NULL && neutralMask[4]) ? getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_PITCH) : clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_PITCH), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_PITCH), (float)servoVals[4]));
  float wristRot = (neutralMask != NULL && neutralMask[5]) ? getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_ROLL) : clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_ROLL), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_ROLL), (float)servoVals[5]));
  float grip = (neutralMask != NULL && neutralMask[6]) ? getManipulatorMemberActualServoDeg(MANIP_MEMBER_GRIPPER) : clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_GRIPPER), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_GRIPPER), (float)servoVals[6]));

  if (collisionRuntimeEnabled) {
    if (!collisionGuardAllowsCandidatePose(base, upper, fore, forearmRoll, wristPitch, wristRot, grip)) {
      return false;
    }
  } else {
    collisionFlag = false;
  }

  if (baseCh == 0) {
    setServoNeutral(0, baseNeutral);
    rt.stepperArmed = !baseNeutral;
    if (baseNeutral) {
      stepper.targetAngle = stepper.currentAngle;
      servos.target[0] = stepper.currentAngle;
      servos.actual[0] = stepper.currentAngle;
      releaseStepperCoils();
      stepperSetDriverPWM(0);
    } else {
      applyServoValue(0, base, APPLY_TARGET);
    }
  } else {
    if (!getServoNeutral(0)) setServoNeutral(0, true);
    if (rt.stepperArmed) {
      rt.stepperArmed = false;
      stepper.targetAngle = stepper.currentAngle;
      servos.target[0] = stepper.currentAngle;
      servos.actual[0] = stepper.currentAngle;
      releaseStepperCoils();
      stepperSetDriverPWM(0);
    }
    setServoNeutral(baseCh, baseNeutral);
    if (baseNeutral) {
      setServoWriteValid(baseCh, false);
      forceServoNeutralOutput(baseCh);
    } else {
      applyServoValue(baseCh, base, APPLY_TARGET);
    }
  }

  struct MemberApply {
    uint8_t member;
    float value;
  } memberApply[6] = {
    {MANIP_MEMBER_UPPER, upper},
    {MANIP_MEMBER_FORE, fore},
    {MANIP_MEMBER_FOREARM_ROLL, forearmRoll},
    {MANIP_MEMBER_WRIST_PITCH, wristPitch},
    {MANIP_MEMBER_WRIST_ROLL, wristRot},
    {MANIP_MEMBER_GRIPPER, grip}
  };

  for (uint8_t i = 0; i < 6; i++) {
    uint8_t member = memberApply[i].member;
    uint8_t ch = getManipulatorMemberChannel(member);
    bool memberNeutral = (neutralMask != NULL && neutralMask[member]);
    setServoNeutral(ch, memberNeutral);
    if (memberNeutral) {
      setServoWriteValid(ch, false);
      forceServoNeutralOutput(ch);
    } else {
      applyServoValue(ch, memberApply[i].value, APPLY_TARGET);
    }
  }

  // Do not release inactive channels or invalidate collision cache on every
  // streamed ARMJ frame. Those operations are handled by mode/channel changes;
  // repeating them here made BASECH=1 and telemetry feel slow.
  applyCollisionGuard();
  return true;
}

// Applies collision guard.
void applyCollisionGuard() {
  if (!collisionRuntimeEnabled || robotControlMode != ROBOT_MODE_MANIPULATOR) {
    collisionFlag = false;
    invalidateCollisionPoseCache();
    return;
  }

  sanitizeCollisionEvaluationState();

  unsigned long nowMs = millis();
  if (collisionPoseCacheValid && (nowMs - lastCollisionGuardEvalMs) < COLLISION_GUARD_INTERVAL_MS) {
    return;
  }

  float baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg;
  captureLimitedActualCollisionPose(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);

  if (!collisionPoseChanged(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg)) {
    return;
  }

  collisionFlag = poseHasCollision(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
  updateCollisionPoseCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
}



// Returns current PWM percents.
void getCurrentPwmPercents(uint8_t outPerc[4]) {
  for (uint8_t i = 0; i < 4; i++) outPerc[i] = pwmBootPercent[i];
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t ch = 12 + i;
    outPerc[i] = (uint8_t)clampValue((int)roundf(servos.actual[ch]), 0, 100);
  }
}











// Utility: estimate pitch degrees.
float estimatePitchDeg(int16_t ax, int16_t ay, int16_t az) {
  return atan2f((float)ax, sqrtf((float)ay * (float)ay + (float)az * (float)az)) * 57.2957795f;
}



// Utility: estimate roll degrees.
float estimateRollDeg(int16_t ax, int16_t ay, int16_t az) {
  return atan2f((float)ay, sqrtf((float)ax * (float)ax + (float)az * (float)az)) * 57.2957795f;
}



// Updates wrist gimbal runtime.
void updateWristGimbalRuntime() {
  if (!wristGimbalEnabled || !rt.mpu1Ready || !rt.active) return;
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  mpu6050_1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float pitch = estimatePitchDeg(ax, ay, az);
  float roll  = estimateRollDeg(ax, ay, az);
  if (!wristGimbalRefValid) {
    wristGimbalPitchRef = pitch;
    wristGimbalRollRef = roll;
    wristGimbalRefValid = true;
  }

  float reachFactor = getStructureReachFactor();
  float massFactor = getStructureMassFactor();
  float wristPitchGain = wristGimbalGainPitch / (0.65f + 0.35f * reachFactor);
  float wristRollGain  = wristGimbalGainRoll  / (0.65f + 0.35f * massFactor);

  float pitchErr = pitch - wristGimbalPitchRef;
  float rollErr  = roll - wristGimbalRollRef;

  float desiredPitch = getManipulatorMemberTargetDeg(MANIP_MEMBER_WRIST_PITCH) - pitchErr * wristPitchGain;
  float desiredRoll  = getManipulatorMemberTargetDeg(MANIP_MEMBER_WRIST_ROLL) - rollErr * wristRollGain;

  trySetStabilizedTarget(MANIP_MEMBER_WRIST_PITCH, desiredPitch);
  trySetStabilizedTarget(MANIP_MEMBER_WRIST_ROLL, desiredRoll);
}



// Updates grip pressure model.
void updateGripPressureModel() {
  if (!gripPressureEnable || !rt.ina1Ready) return;
  int current = (int)ina219_1.getCurrent_mA();
  bool settled =
    fabsf(getManipulatorMemberTargetServoDeg(MANIP_MEMBER_WRIST_PITCH) - getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_PITCH)) < 0.8f &&
    fabsf(getManipulatorMemberTargetServoDeg(MANIP_MEMBER_WRIST_ROLL) - getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_ROLL)) < 0.8f &&
    fabsf(getManipulatorMemberTargetServoDeg(MANIP_MEMBER_GRIPPER) - getManipulatorMemberActualServoDeg(MANIP_MEMBER_GRIPPER)) < 0.8f;
  if (settled) {
    if (gripIdleCurrentAvg <= 0.1f) gripIdleCurrentAvg = (float)current;
    else gripIdleCurrentAvg = gripIdleCurrentAvg * 0.98f + (float)current * 0.02f;
  }
  gripPressureDeltaMa = current - (int)gripIdleCurrentAvg;
  if (gripPressureDeltaMa < 0) gripPressureDeltaMa = 0;
  gripPressureEstimate = gripPressureDeltaMa * gripPressureGain;
}



// Updates arm stabilization.
void updateArmStabilization() {
  if (!armStabilizeEnable || !rt.mpu2Ready || !rt.active) return;
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  mpu6050_2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float pitch = estimatePitchDeg(ax, ay, az);
  float roll  = estimateRollDeg(ax, ay, az);

  float reachFactor = getStructureReachFactor();
  float massFactor = getStructureMassFactor();
  float shoulderGain = armStabilizeGain / (0.60f + 0.40f * reachFactor);
  float foreGain = (armStabilizeGain * 0.5f) / (0.60f + 0.40f * massFactor);
  float wristRotGain = (armStabilizeGain * 0.5f) / (0.65f + 0.35f * reachFactor);

  if (fabsf(pitch) > armStabilizeThreshold) {
    float desiredShoulder = getManipulatorMemberTargetDeg(MANIP_MEMBER_UPPER) - pitch * shoulderGain;
    float desiredFore = getManipulatorMemberTargetDeg(MANIP_MEMBER_FORE) - pitch * foreGain;
    trySetStabilizedTarget(MANIP_MEMBER_UPPER, desiredShoulder);
    trySetStabilizedTarget(MANIP_MEMBER_FORE, desiredFore);
  }
  if (fabsf(roll) > armStabilizeThreshold) {
    float desiredWristRot = getManipulatorMemberTargetDeg(MANIP_MEMBER_WRIST_ROLL) - roll * wristRotGain;
    trySetStabilizedTarget(MANIP_MEMBER_WRIST_ROLL, desiredWristRot);
  }
}

// Prints robot specific runtime examples.
void printRobotSpecificRuntimeExamples() {
}

// Prints robot specific config summary.
void printRobotSpecificConfigSummary() {
  syncProcessingRobotProfiles();
  switch (robotControlMode) {
    case ROBOT_MODE_VEHICLE:
      Serial.println(F("[VEHICLE STATUS]"));
      Serial.print(F("geometry_units=")); Serial.println(F("cm"));
      Serial.print(F("vehicle_body_length=")); Serial.println(vehicleProfile.bodyLength, 2);
      Serial.print(F("vehicle_body_width=")); Serial.println(vehicleProfile.bodyWidth, 2);
      Serial.print(F("vehicle_body_height=")); Serial.println(vehicleProfile.bodyHeight, 2);
      Serial.print(F("vehicle_track_gauge=")); Serial.println(vehicleProfile.trackGauge, 2);
      Serial.print(F("vehicle_track_run=")); Serial.println(vehicleProfile.trackRun, 2);
      Serial.print(F("vehicle_drive_radius=")); Serial.println(vehicleProfile.driveRadius, 2);
      Serial.print(F("vehicle_support_radius=")); Serial.println(vehicleProfile.supportRadius, 2);
      Serial.print(F("vehicle_top_roller_radius=")); Serial.println(vehicleProfile.topRollerRadius, 2);
      Serial.print(F("vehicle_track_width=")); Serial.println(vehicleProfile.trackWidth, 2);
      Serial.print(F("vehicle_track_link_length=")); Serial.println(vehicleProfile.trackLinkLength, 2);
      Serial.print(F("vehicle_track_link_thickness=")); Serial.println(vehicleProfile.trackLinkThickness, 2);
      Serial.print(F("vehicle_track_height=")); Serial.println(vehicleProfile.trackHeight, 2);
      Serial.print(F("vehicle_track_links=")); Serial.println(vehicleProfile.trackLinks);
      Serial.print(F("vehicle_track_sag=")); Serial.println(vehicleProfile.trackSag, 2);
      Serial.print(F("vehicle_body_center_y=")); Serial.println(vehicleProfile.bodyCenterYOffset, 2);
      Serial.print(F("vehicle_camera_default_pan_deg=")); Serial.println(vehicleProfile.cameraPanDefaultDeg, 2);
      Serial.print(F("vehicle_camera_default_tilt_deg=")); Serial.println(vehicleProfile.cameraTiltDefaultDeg, 2);
      Serial.print(F("vehicle_lidar_mast_y=")); Serial.println(vehicleProfile.lidarMastYOffset, 2);
      Serial.print(F("vehicle_lidar_mast_z=")); Serial.println(vehicleProfile.lidarMastZOffset, 2);
      Serial.print(F("vehicle_camera_head_y=")); Serial.println(vehicleProfile.cameraHeadYOffset, 2);
      Serial.print(F("vehicle_camera_head_z=")); Serial.println(vehicleProfile.cameraHeadZOffset, 2);
      Serial.print(F("vehicle_motor_deadband_pct=")); Serial.println(vehicleControlProfile.motorDeadbandPct, 2);
      Serial.print(F("vehicle_motor_pwm_min=")); Serial.println(vehicleControlProfile.motorPwmMin);
      Serial.print(F("vehicle_motor_pwm_max=")); Serial.println(vehicleControlProfile.motorPwmMax);
      Serial.print(F("vehicle_motor_slew_pct_per_update=")); Serial.println(vehicleControlProfile.motorSlewPctPerUpdate, 2);
      Serial.print(F("vehicle_left_motor_invert=")); Serial.println(vehicleControlProfile.leftMotorInvert ? 1 : 0);
      Serial.print(F("vehicle_right_motor_invert=")); Serial.println(vehicleControlProfile.rightMotorInvert ? 1 : 0);
      Serial.print(F("vehicle_camera_pan_min_deg=")); Serial.println(vehicleControlProfile.cameraPanMinDeg, 2);
      Serial.print(F("vehicle_camera_pan_max_deg=")); Serial.println(vehicleControlProfile.cameraPanMaxDeg, 2);
      Serial.print(F("vehicle_camera_tilt_min_deg=")); Serial.println(vehicleControlProfile.cameraTiltMinDeg, 2);
      Serial.print(F("vehicle_camera_tilt_max_deg=")); Serial.println(vehicleControlProfile.cameraTiltMaxDeg, 2);
      Serial.print(F("vehicle_lidar_sweep_min_deg=")); Serial.println(vehicleControlProfile.lidarSweepMinDeg);
      Serial.print(F("vehicle_lidar_sweep_max_deg=")); Serial.println(vehicleControlProfile.lidarSweepMaxDeg);
      Serial.print(F("vehicle_lidar_sweep_cycle_ms=")); Serial.println(vehicleControlProfile.lidarSweepCycleMs);
      Serial.print(F("left_track_pct=")); Serial.println(vehicleLeftTrackPct);
      Serial.print(F("right_track_pct=")); Serial.println(vehicleRightTrackPct);
      Serial.print(F("light=")); Serial.println(vehicleLightsCommand ? 1 : 0);
      Serial.print(F("lscan=")); Serial.println(vehicleLidarScanCommand ? 1 : 0);
      Serial.print(F("vehicle_cam_pan_deg=")); Serial.println(vehicleCameraPanDeg);
      Serial.print(F("vehicle_cam_tilt_deg=")); Serial.println(vehicleCameraTiltDeg);
      Serial.print(F("vehicle_x=")); Serial.println(vehicleTelemetryX, 2);
      Serial.print(F("vehicle_z=")); Serial.println(vehicleTelemetryZ, 2);
      Serial.print(F("vehicle_yaw_deg=")); Serial.println(vehicleTelemetryYawDeg, 2);
      break;
    case ROBOT_MODE_DRONE:
      Serial.println(F("[DRONE STATUS]"));
      Serial.print(F("geometry_units=")); Serial.println(F("cm"));
      Serial.print(F("drone_body_length=")); Serial.println(droneProfile.bodyLength, 2);
      Serial.print(F("drone_body_width=")); Serial.println(droneProfile.bodyWidth, 2);
      Serial.print(F("drone_body_height=")); Serial.println(droneProfile.bodyHeight, 2);
      Serial.print(F("drone_arm_length=")); Serial.println(droneProfile.armLength, 2);
      Serial.print(F("drone_arm_thickness=")); Serial.println(droneProfile.armThickness, 2);
      Serial.print(F("drone_motor_radius=")); Serial.println(droneProfile.motorRadius, 2);
      Serial.print(F("drone_motor_height=")); Serial.println(droneProfile.motorHeight, 2);
      Serial.print(F("drone_prop_radius=")); Serial.println(droneProfile.propRadius, 2);
      Serial.print(F("drone_prop_thickness=")); Serial.println(droneProfile.propThickness, 2);
      Serial.print(F("drone_leg_height=")); Serial.println(droneProfile.legHeight, 2);
      Serial.print(F("drone_leg_span=")); Serial.println(droneProfile.legSpan, 2);
      Serial.print(F("drone_rest_y=")); Serial.println(droneProfile.restYOffset, 2);
      Serial.print(F("drone_visual_yaw_offset_deg=")); Serial.println(droneProfile.visualYawOffsetDeg, 2);
      Serial.print(F("drone_camera_y=")); Serial.println(droneProfile.cameraYOffset, 2);
      Serial.print(F("drone_camera_z=")); Serial.println(droneProfile.cameraZOffset, 2);
      Serial.print(F("drone_lamp_y=")); Serial.println(droneProfile.lampYOffset, 2);
      Serial.print(F("drone_lamp_z=")); Serial.println(droneProfile.lampZOffset, 2);
      Serial.print(F("drone_esc_min_us=")); Serial.println(droneControlProfile.escMinUs);
      Serial.print(F("drone_esc_max_us=")); Serial.println(droneControlProfile.escMaxUs);
      Serial.print(F("drone_esc_idle_us=")); Serial.println(droneControlProfile.escIdleUs);
      Serial.print(F("drone_throttle_range_us=")); Serial.println(droneControlProfile.throttleRangeUs);
      Serial.print(F("drone_pitch_mix_us=")); Serial.println(droneControlProfile.pitchMixUs);
      Serial.print(F("drone_roll_mix_us=")); Serial.println(droneControlProfile.rollMixUs);
      Serial.print(F("drone_yaw_mix_us=")); Serial.println(droneControlProfile.yawMixUs);
      Serial.print(F("drone_command_deadband_pct=")); Serial.println(droneControlProfile.commandDeadbandPct, 2);
      Serial.print(F("drone_spool_step_us_per_update=")); Serial.println(droneControlProfile.spoolStepUsPerUpdate);
      Serial.print(F("camera_record=")); Serial.println(droneCameraRecordCommand ? 1 : 0);
      Serial.print(F("throttle_pct=")); Serial.println(droneThrottlePct);
      Serial.print(F("yaw_pct=")); Serial.println(droneYawPct);
      Serial.print(F("pitch_pct=")); Serial.println(dronePitchPct);
      Serial.print(F("roll_pct=")); Serial.println(droneRollPct);
      Serial.print(F("strafe_pct=")); Serial.println(droneStrafePct);
      Serial.print(F("forward_pct=")); Serial.println(droneForwardPct);
      Serial.print(F("drone_x=")); Serial.println(droneTelemetryX, 2);
      Serial.print(F("drone_y=")); Serial.println(droneTelemetryY, 2);
      Serial.print(F("drone_z=")); Serial.println(droneTelemetryZ, 2);
      Serial.print(F("drone_yaw_deg=")); Serial.println(droneTelemetryYawDeg, 2);
      Serial.print(F("drone_pitch_deg=")); Serial.println(droneTelemetryPitchDeg, 2);
      Serial.print(F("drone_roll_deg=")); Serial.println(droneTelemetryRollDeg, 2);
      break;
    case ROBOT_MODE_MANIPULATOR:
    default:
      Serial.println(F("[MANIPULATOR STATUS]"));
      Serial.print(F("geometry_units=")); Serial.println(F("cm"));
      Serial.print(F("arm_base_cylinder_y=")); Serial.println(armSceneToCm(armBaseCylinderYOffset), 3);
      Serial.print(F("arm_base_block_y=")); Serial.println(armSceneToCm(armBaseBlockYOffset), 3);
      Serial.print(F("arm_gripper_y=")); Serial.println(armSceneToCm(armGripperYOffset), 3);
      Serial.print(F("arm_base_radius=")); Serial.println(armSceneToCm(armBaseRadius), 3);
      Serial.print(F("arm_base_height=")); Serial.println(armSceneToCm(armBaseHeight), 3);
      Serial.print(F("arm_base_block_w=")); Serial.println(armSceneToCm(armBaseBlockW), 3);
      Serial.print(F("arm_base_block_h=")); Serial.println(armSceneToCm(armBaseBlockH), 3);
      Serial.print(F("arm_base_block_d=")); Serial.println(armSceneToCm(armBaseBlockD), 3);
      Serial.print(F("arm_upper_w=")); Serial.println(armSceneToCm(armUpperW), 3);
      Serial.print(F("arm_upper_h=")); Serial.println(armSceneToCm(armUpperH), 3);
      Serial.print(F("arm_upper_d=")); Serial.println(armSceneToCm(armUpperD), 3);
      Serial.print(F("arm_fore_w=")); Serial.println(armSceneToCm(armForeW), 3);
      Serial.print(F("arm_fore_h=")); Serial.println(armSceneToCm(armForeH), 3);
      Serial.print(F("arm_fore_d=")); Serial.println(armSceneToCm(armForeD), 3);
      Serial.print(F("arm_wrist_w=")); Serial.println(armSceneToCm(armWristW), 3);
      Serial.print(F("arm_wrist_h=")); Serial.println(armSceneToCm(armWristH), 3);
      Serial.print(F("arm_wrist_d=")); Serial.println(armSceneToCm(armWristD), 3);
      Serial.print(F("arm_finger_w=")); Serial.println(armSceneToCm(armFingerW), 3);
      Serial.print(F("arm_finger_h=")); Serial.println(armSceneToCm(armFingerH), 3);
      Serial.print(F("arm_finger_d=")); Serial.println(armSceneToCm(armFingerD), 3);
      Serial.print(F("arm_map_base_channel=")); Serial.println(manipProfile.baseChannel);
      Serial.print(F("arm_map_upper_channel=")); Serial.println(manipProfile.upperChannel);
      Serial.print(F("arm_map_fore_channel=")); Serial.println(manipProfile.foreChannel);
      Serial.print(F("arm_map_forearm_roll_channel=")); Serial.println(manipProfile.forearmRollChannel);
      Serial.print(F("arm_map_wrist_pitch_channel=")); Serial.println(manipProfile.wristPitchChannel);
      Serial.print(F("arm_map_wrist_rot_channel=")); Serial.println(manipProfile.wristRotChannel);
      Serial.print(F("arm_map_gripper_channel=")); Serial.println(manipProfile.gripperChannel);
      Serial.print(F("base_source_mode=")); Serial.println((manipProfile.baseChannel == 0) ? F("POT") : F("CHANNEL"));
      Serial.print(F("armlim="));
      for (uint8_t i = 0; i < MANIP_MEMBER_COUNT; i++) {
        if (i > 0) Serial.print(',');
        Serial.print(getManipulatorMemberMinLimit(i), 1);
        Serial.print(',');
        Serial.print(getManipulatorMemberMaxLimit(i), 1);
      }
      Serial.println();
      Serial.print(F("armoff="));
      Serial.print(armBaseServoZeroDeg, 3); Serial.print(','); Serial.print(armBaseMemberZeroDeg, 3); Serial.print(','); Serial.print(armBaseServoSign, 3); Serial.print(',');
      Serial.print(armUpperServoZeroDeg, 3); Serial.print(','); Serial.print(armUpperMemberZeroDeg, 3); Serial.print(','); Serial.print(armUpperServoSign, 3); Serial.print(',');
      Serial.print(armForeServoZeroDeg, 3); Serial.print(','); Serial.print(armForeMemberZeroDeg, 3); Serial.print(','); Serial.print(armForeServoSign, 3); Serial.print(',');
      Serial.print(armForearmRollServoZeroDeg, 3); Serial.print(','); Serial.print(armForearmRollMemberZeroDeg, 3); Serial.print(','); Serial.print(armForearmRollServoSign, 3); Serial.print(',');
      Serial.print(armWristPitchServoZeroDeg, 3); Serial.print(','); Serial.print(armWristPitchMemberZeroDeg, 3); Serial.print(','); Serial.print(armWristPitchServoSign, 3); Serial.print(',');
      Serial.print(armWristRollServoZeroDeg, 3); Serial.print(','); Serial.print(armWristRollMemberZeroDeg, 3); Serial.print(','); Serial.print(armWristRollServoSign, 3); Serial.print(',');
      Serial.print(armGripServoZeroDeg, 3); Serial.print(','); Serial.print(armGripMemberZeroDeg, 3); Serial.print(','); Serial.println(armGripServoSign, 3);
      break;
  }
  Serial.println(F(""));
  Serial.println(F("============================================================"));
  Serial.println(F(""));
}

// Prints vehicle config panels.
void printVehicleConfigPanels() {
  Serial.println(F("[VEHICLE GEOMETRY]"));
  Serial.println(F("  VEHBODYL=<v>    -> body length (cm)"));
  Serial.println(F("  VEHBODYW=<v>    -> body width (cm)"));
  Serial.println(F("  VEHBODYH=<v>    -> body height (cm)"));
  Serial.println(F("  VEHTGAUGE=<v>   -> track gauge / distance between tracks (cm)"));
  Serial.println(F("  VEHTRUN=<v>     -> track ground run length (cm)"));
  Serial.println(F("  VEHDRIVE=<v>    -> drive sprocket radius (cm)"));
  Serial.println(F("  VEHSUPR=<v>     -> support wheel radius (cm)"));
  Serial.println(F("  VEHTOPR=<v>     -> top roller radius (cm)"));
  Serial.println(F("  VEHTWIDTH=<v>   -> track belt width (cm)"));
  Serial.println(F("  VEHTLINKL=<v>   -> track link length (cm)"));
  Serial.println(F("  VEHTLINKT=<v>   -> track link thickness (cm)"));
  Serial.println(F("  VEHTHEIGHT=<v>  -> visual track height (cm)"));
  Serial.println(F("  VEHTLINKS=<v>   -> rendered track link count"));
  Serial.println(F("  VEHTSAG=<v>     -> track sag amount"));
  Serial.println(F("  VEHCAMPAN0=<v>  -> default camera pan angle"));
  Serial.println(F("  VEHCAMTILT0=<v> -> default camera tilt angle"));
  Serial.println(F("  VEHLIDARY=<v>   -> lidar mast Y offset"));
  Serial.println(F("  VEHLIDARZ=<v>   -> lidar mast Z offset"));
  Serial.println(F("  VEHCAMY=<v>     -> camera head Y offset"));
  Serial.println(F("  VEHCAMZ=<v>     -> camera head Z offset"));
  Serial.println(F(""));
  Serial.print(F("geometry_units=")); Serial.println(F("cm"));
  Serial.print(F("vehicle_body_length=")); Serial.println(vehicleProfile.bodyLength, 2);
  Serial.print(F("vehicle_body_width=")); Serial.println(vehicleProfile.bodyWidth, 2);
  Serial.print(F("vehicle_body_height=")); Serial.println(vehicleProfile.bodyHeight, 2);
  Serial.print(F("vehicle_track_gauge=")); Serial.println(vehicleProfile.trackGauge, 2);
  Serial.print(F("vehicle_track_run=")); Serial.println(vehicleProfile.trackRun, 2);
  Serial.print(F("vehicle_drive_radius=")); Serial.println(vehicleProfile.driveRadius, 2);
  Serial.print(F("vehicle_support_radius=")); Serial.println(vehicleProfile.supportRadius, 2);
  Serial.print(F("vehicle_top_roller_radius=")); Serial.println(vehicleProfile.topRollerRadius, 2);
  Serial.print(F("vehicle_track_width=")); Serial.println(vehicleProfile.trackWidth, 2);
  Serial.print(F("vehicle_track_link_length=")); Serial.println(vehicleProfile.trackLinkLength, 2);
  Serial.print(F("vehicle_track_link_thickness=")); Serial.println(vehicleProfile.trackLinkThickness, 2);
  Serial.print(F("vehicle_track_height=")); Serial.println(vehicleProfile.trackHeight, 2);
  Serial.print(F("vehicle_track_links=")); Serial.println(vehicleProfile.trackLinks);
  Serial.print(F("vehicle_track_sag=")); Serial.println(vehicleProfile.trackSag, 2);
  Serial.print(F("vehicle_body_center_y=")); Serial.println(vehicleProfile.bodyCenterYOffset, 2);
  Serial.print(F("vehicle_camera_default_pan_deg=")); Serial.println(vehicleProfile.cameraPanDefaultDeg, 2);
  Serial.print(F("vehicle_camera_default_tilt_deg=")); Serial.println(vehicleProfile.cameraTiltDefaultDeg, 2);
  Serial.print(F("vehicle_lidar_mast_y=")); Serial.println(vehicleProfile.lidarMastYOffset, 2);
  Serial.print(F("vehicle_lidar_mast_z=")); Serial.println(vehicleProfile.lidarMastZOffset, 2);
  Serial.print(F("vehicle_camera_head_y=")); Serial.println(vehicleProfile.cameraHeadYOffset, 2);
  Serial.print(F("vehicle_camera_head_z=")); Serial.println(vehicleProfile.cameraHeadZOffset, 2);
  Serial.println(F(""));

  Serial.println(F("[VEHICLE CONTROL]"));
  Serial.println(F("  VEHDEAD=<v>       -> motor command deadband (%)"));
  Serial.println(F("  VEHPWMMIN=<v>     -> minimum motor PWM output"));
  Serial.println(F("  VEHPWMMAX=<v>     -> maximum motor PWM output"));
  Serial.println(F("  VEHSLEW=<v>       -> motor slew rate per update (%)"));
  Serial.println(F("  VEHINVLEFT=0/1    -> invert left motor direction"));
  Serial.println(F("  VEHINVRIGHT=0/1   -> invert right motor direction"));
  Serial.println(F("  VEHCAMPANMIN=<v>  -> camera pan minimum"));
  Serial.println(F("  VEHCAMPANMAX=<v>  -> camera pan maximum"));
  Serial.println(F("  VEHCAMTILTMIN=<v> -> camera tilt minimum"));
  Serial.println(F("  VEHCAMTILTMAX=<v> -> camera tilt maximum"));
  Serial.println(F("  VEHLIDARMIN=<v>   -> lidar sweep minimum"));
  Serial.println(F("  VEHLIDARMAX=<v>   -> lidar sweep maximum"));
  Serial.println(F("  VEHLIDARMS=<v>    -> lidar sweep cycle time (ms)"));
  Serial.println(F(""));
  Serial.print(F("vehicle_motor_deadband_pct=")); Serial.println(vehicleControlProfile.motorDeadbandPct, 2);
  Serial.print(F("vehicle_motor_pwm_min=")); Serial.println(vehicleControlProfile.motorPwmMin);
  Serial.print(F("vehicle_motor_pwm_max=")); Serial.println(vehicleControlProfile.motorPwmMax);
  Serial.print(F("vehicle_motor_slew_pct_per_update=")); Serial.println(vehicleControlProfile.motorSlewPctPerUpdate, 2);
  Serial.print(F("vehicle_left_motor_invert=")); Serial.println(vehicleControlProfile.leftMotorInvert ? 1 : 0);
  Serial.print(F("vehicle_right_motor_invert=")); Serial.println(vehicleControlProfile.rightMotorInvert ? 1 : 0);
  Serial.print(F("vehicle_camera_pan_min_deg=")); Serial.println(vehicleControlProfile.cameraPanMinDeg, 2);
  Serial.print(F("vehicle_camera_pan_max_deg=")); Serial.println(vehicleControlProfile.cameraPanMaxDeg, 2);
  Serial.print(F("vehicle_camera_tilt_min_deg=")); Serial.println(vehicleControlProfile.cameraTiltMinDeg, 2);
  Serial.print(F("vehicle_camera_tilt_max_deg=")); Serial.println(vehicleControlProfile.cameraTiltMaxDeg, 2);
  Serial.print(F("vehicle_lidar_sweep_min_deg=")); Serial.println(vehicleControlProfile.lidarSweepMinDeg);
  Serial.print(F("vehicle_lidar_sweep_max_deg=")); Serial.println(vehicleControlProfile.lidarSweepMaxDeg);
  Serial.print(F("vehicle_lidar_sweep_cycle_ms=")); Serial.println(vehicleControlProfile.lidarSweepCycleMs);
  Serial.println(F(""));

  Serial.println(F("[VEHICLE SAFETY]"));
  Serial.println(F("  VEHRTH=0/1          -> low-battery GPS return-to-base"));
  Serial.println(F("  VEHICLELINKHOLD=0/1 -> invert last track movement while runtime link is lost"));
  Serial.println(F("  SAFEBAT=<pct>       -> battery threshold for active ROBOT RTH"));
  Serial.println(F("  VEHINCL=0/1         -> MPU incline PWM assist for tracks"));
  Serial.println(F("  VEHINCLBASE=<v>     -> incline assist base power (%)"));
  Serial.println(F("  VEHINCLMAX=<v>      -> incline assist maximum power (%)"));
  Serial.println(F("  VEHINCLFULL=<deg>   -> incline angle for full assist"));
  Serial.println(F("  VEHINCLDB=<deg>     -> incline assist deadband"));
  Serial.println(F("  VEHINCLALPHA=<v>    -> incline assist filter alpha"));
  Serial.println(F(""));
  Serial.print(F("vehicle_safe_return_enabled=")); Serial.println(vehicleSafeReturnEnabled ? 1 : 0);
  Serial.print(F("vehicle_link_hold_enabled=")); Serial.println(vehicleHoldLastMotionOnLinkLoss ? 1 : 0);
  Serial.print(F("vehicle_safe_return_battery_threshold_pct=")); Serial.println(vehicleSafeReturnBatteryThresholdPct, 1);
  Serial.print(F("vehicle_home_valid=")); Serial.println(vehicleHomeValid ? 1 : 0);
  Serial.print(F("vehicle_home_lat_e7=")); Serial.println(vehicleHomeLatitudeE7);
  Serial.print(F("vehicle_home_lon_e7=")); Serial.println(vehicleHomeLongitudeE7);
  Serial.print(F("vehicle_incline_power_enabled=")); Serial.println(vehicleInclinePowerEnabled ? 1 : 0);
  Serial.print(F("vehicle_incline_base_power_pct=")); Serial.println(vehicleInclineBasePowerPct, 1);
  Serial.print(F("vehicle_incline_max_power_pct=")); Serial.println(vehicleInclineMaxPowerPct, 1);
  Serial.print(F("vehicle_incline_full_scale_deg=")); Serial.println(vehicleInclineFullScaleDeg, 1);
  Serial.print(F("vehicle_incline_deadband_deg=")); Serial.println(vehicleInclineDeadbandDeg, 1);
  Serial.print(F("vehicle_incline_alpha=")); Serial.println(vehicleInclinePowerAlpha, 3);
  Serial.print(F("vehicle_incline_now_deg=")); Serial.println(vehicleInclineMagnitudeDeg, 2);
  Serial.print(F("vehicle_incline_auto_power_pct=")); Serial.println(vehicleInclineAutoPowerPct, 1);
  Serial.println(F(""));

  Serial.println(F("[VEHICLE HARDWARE]"));
  Serial.println(F("  TRACK=l,r,CAM=p,t -> left/right tracks + camera"));
  Serial.println(F("  MOVE=thr,steer    -> throttle/steer mix"));
  Serial.println(F("  LIGHT=0/1         -> lights OFF/ON"));
  Serial.println(F("  SCAN=0/1          -> lidar scan OFF/ON"));
  Serial.println(F(""));
  Serial.print(F("vehicle_left_pwm_pin=")); Serial.println(VEHICLE_LEFT_TRACK_PWM_PIN);
  Serial.print(F("vehicle_left_dir_pin=")); Serial.println(VEHICLE_LEFT_TRACK_DIR_PIN);
  Serial.print(F("vehicle_right_pwm_pin=")); Serial.println(VEHICLE_RIGHT_TRACK_PWM_PIN);
  Serial.print(F("vehicle_right_dir_pin=")); Serial.println(VEHICLE_RIGHT_TRACK_DIR_PIN);
  Serial.print(F("vehicle_light_pin=")); Serial.println(VEHICLE_LIGHT_PIN);
  Serial.print(F("vehicle_camera_pan_pin=")); Serial.println(VEHICLE_CAMERA_PAN_PIN);
  Serial.print(F("vehicle_camera_tilt_pin=")); Serial.println(VEHICLE_CAMERA_TILT_PIN);
  Serial.print(F("vehicle_lidar_servo_pin=")); Serial.println(VEHICLE_LIDAR_SCAN_SERVO_PIN);
  Serial.println(F(""));
}

// Prints drone config panels.
void printDroneConfigPanels() {
  Serial.println(F("[DRONE GEOMETRY]"));
  Serial.println(F("  DRNBODYL=<v>   -> body length (cm)"));
  Serial.println(F("  DRNBODYW=<v>   -> body width (cm)"));
  Serial.println(F("  DRNBODYH=<v>   -> body height (cm)"));
  Serial.println(F("  DRNARML=<v>    -> arm length (cm)"));
  Serial.println(F("  DRNARMT=<v>    -> arm thickness (cm)"));
  Serial.println(F("  DRNMOTR=<v>    -> motor radius (cm)"));
  Serial.println(F("  DRNMOTH=<v>    -> motor height (cm)"));
  Serial.println(F("  DRNPROPR=<v>   -> propeller radius (cm)"));
  Serial.println(F("  DRNPROPT=<v>   -> propeller thickness (cm)"));
  Serial.println(F("  DRNLEGH=<v>    -> landing leg height (cm)"));
  Serial.println(F("  DRNLEGS=<v>    -> landing leg span (cm)"));
  Serial.println(F("  DRNRESTY=<v>   -> rest Y offset"));
  Serial.println(F("  DRNVISYAW=<v>  -> visual yaw offset"));
  Serial.println(F("  DRNCAMY=<v>    -> camera Y offset"));
  Serial.println(F("  DRNCAMZ=<v>    -> camera Z offset"));
  Serial.println(F("  DRNLAMPY=<v>   -> lamp Y offset"));
  Serial.println(F("  DRNLAMPZ=<v>   -> lamp Z offset"));
  Serial.println(F("  DRNSONARX=<v>  -> downward sonar X offset"));
  Serial.println(F("  DRNSONARY=<v>  -> downward sonar Y offset"));
  Serial.println(F("  DRNSONARZ=<v>  -> downward sonar Z offset"));
  Serial.println(F(""));
  Serial.print(F("geometry_units=")); Serial.println(F("cm"));
  Serial.print(F("drone_body_length=")); Serial.println(droneProfile.bodyLength, 2);
  Serial.print(F("drone_body_width=")); Serial.println(droneProfile.bodyWidth, 2);
  Serial.print(F("drone_body_height=")); Serial.println(droneProfile.bodyHeight, 2);
  Serial.print(F("drone_arm_length=")); Serial.println(droneProfile.armLength, 2);
  Serial.print(F("drone_arm_thickness=")); Serial.println(droneProfile.armThickness, 2);
  Serial.print(F("drone_motor_radius=")); Serial.println(droneProfile.motorRadius, 2);
  Serial.print(F("drone_motor_height=")); Serial.println(droneProfile.motorHeight, 2);
  Serial.print(F("drone_prop_radius=")); Serial.println(droneProfile.propRadius, 2);
  Serial.print(F("drone_prop_thickness=")); Serial.println(droneProfile.propThickness, 2);
  Serial.print(F("drone_leg_height=")); Serial.println(droneProfile.legHeight, 2);
  Serial.print(F("drone_leg_span=")); Serial.println(droneProfile.legSpan, 2);
  Serial.print(F("drone_rest_y=")); Serial.println(droneProfile.restYOffset, 2);
  Serial.print(F("drone_visual_yaw_offset_deg=")); Serial.println(droneProfile.visualYawOffsetDeg, 2);
  Serial.print(F("drone_camera_y=")); Serial.println(droneProfile.cameraYOffset, 2);
  Serial.print(F("drone_camera_z=")); Serial.println(droneProfile.cameraZOffset, 2);
  Serial.print(F("drone_lamp_y=")); Serial.println(droneProfile.lampYOffset, 2);
  Serial.print(F("drone_lamp_z=")); Serial.println(droneProfile.lampZOffset, 2);
  Serial.print(F("drone_sonar_x=")); Serial.println(droneProfile.sonarXOffset, 2);
  Serial.print(F("drone_sonar_y=")); Serial.println(droneProfile.sonarYOffset, 2);
  Serial.print(F("drone_sonar_z=")); Serial.println(droneProfile.sonarZOffset, 2);
  Serial.println(F(""));

  Serial.println(F("[DRONE CONTROL]"));
  Serial.println(F("  DRNESCMIN=<v>   -> ESC minimum pulse (us)"));
  Serial.println(F("  DRNESCMAX=<v>   -> ESC maximum pulse (us)"));
  Serial.println(F("  DRNESCIDLE=<v>  -> ESC idle pulse (us)"));
  Serial.println(F("  DRNTHRSPAN=<v>  -> throttle span above idle (us)"));
  Serial.println(F("  DRNPITCHMIX=<v> -> pitch mixer gain (us)"));
  Serial.println(F("  DRNROLLMIX=<v>  -> roll mixer gain (us)"));
  Serial.println(F("  DRNYAWMIX=<v>   -> yaw mixer gain (us)"));
  Serial.println(F("  DRNDEAD=<v>     -> command deadband (%)"));
  Serial.println(F("  DRNSPOOL=<v>    -> ESC spool slew per update (us)"));
  Serial.println(F(""));
  Serial.print(F("drone_esc_min_us=")); Serial.println(droneControlProfile.escMinUs);
  Serial.print(F("drone_esc_max_us=")); Serial.println(droneControlProfile.escMaxUs);
  Serial.print(F("drone_esc_idle_us=")); Serial.println(droneControlProfile.escIdleUs);
  Serial.print(F("drone_throttle_range_us=")); Serial.println(droneControlProfile.throttleRangeUs);
  Serial.print(F("drone_pitch_mix_us=")); Serial.println(droneControlProfile.pitchMixUs);
  Serial.print(F("drone_roll_mix_us=")); Serial.println(droneControlProfile.rollMixUs);
  Serial.print(F("drone_yaw_mix_us=")); Serial.println(droneControlProfile.yawMixUs);
  Serial.print(F("drone_command_deadband_pct=")); Serial.println(droneControlProfile.commandDeadbandPct, 2);
  Serial.print(F("drone_spool_step_us_per_update=")); Serial.println(droneControlProfile.spoolStepUsPerUpdate);
  Serial.println(F(""));

  Serial.println(F("[DRONE SAFETY]"));
  Serial.println(F("  DRNRTH=0/1        -> low-battery GPS return-to-base"));
  Serial.println(F("  DRONELINKHOLD=0/1 -> invert last flight movement while runtime link is lost"));
  Serial.println(F("  SAFEBAT=<pct>     -> battery threshold for active ROBOT RTH"));
  Serial.println(F(""));
  Serial.print(F("drone_safe_return_enabled=")); Serial.println(droneSafeReturnEnabled ? 1 : 0);
  Serial.print(F("drone_link_hold_enabled=")); Serial.println(droneHoldLastMotionOnLinkLoss ? 1 : 0);
  Serial.print(F("drone_safe_return_battery_threshold_pct=")); Serial.println(droneSafeReturnBatteryThresholdPct, 1);
  Serial.print(F("drone_home_valid=")); Serial.println(droneHomeValid ? 1 : 0);
  Serial.print(F("drone_home_lat_e7=")); Serial.println(droneHomeLatitudeE7);
  Serial.print(F("drone_home_lon_e7=")); Serial.println(droneHomeLongitudeE7);
  Serial.println(F(""));

  Serial.println(F("[DRONE HARDWARE]"));
  Serial.println(F("  FLY=t,y,p,r,s,f -> throttle,yaw,pitch,roll,strafe,forward"));
  Serial.println(F("  CAMREC=0/1      -> camera recording OFF/ON"));
  Serial.println(F("  TAKEOFF         -> automatic takeoff request"));
  Serial.println(F("  LAND            -> automatic landing request"));
  Serial.println(F("  STOP            -> stop flight outputs"));
  Serial.println(F(""));
  Serial.print(F("drone_motor_fl_pin=")); Serial.println(DRONE_ESC_PINS[0]);
  Serial.print(F("drone_motor_fr_pin=")); Serial.println(DRONE_ESC_PINS[1]);
  Serial.print(F("drone_motor_rr_pin=")); Serial.println(DRONE_ESC_PINS[2]);
  Serial.print(F("drone_motor_rl_pin=")); Serial.println(DRONE_ESC_PINS[3]);
  Serial.print(F("drone_camrec_pin=")); Serial.println(DRONE_CAMERA_RECORD_PIN);
  Serial.print(F("drone_status_led_pin=")); Serial.println(DRONE_STATUS_LED_PIN);
  Serial.print(F("drone_motor_fl_us=")); Serial.println(droneMotorMicros[0]);
  Serial.print(F("drone_motor_fr_us=")); Serial.println(droneMotorMicros[1]);
  Serial.print(F("drone_motor_rr_us=")); Serial.println(droneMotorMicros[2]);
  Serial.print(F("drone_motor_rl_us=")); Serial.println(droneMotorMicros[3]);
  Serial.println(F(""));
}

// Prints manipulator geometry panel.
void printManipulatorGeometryPanel() {
  Serial.println(F("[ARM GEOMETRY]"));
  Serial.println(F("  ARMBASER=<v>    -> base cylinder radius"));
  Serial.println(F("  ARMBASEH=<v>    -> base cylinder height"));
  Serial.println(F("  ARMBBLW=<v>     -> base block width"));
  Serial.println(F("  ARMBBLH=<v>     -> base block height"));
  Serial.println(F("  ARMBBLD=<v>     -> base block depth"));
  Serial.println(F("  ARMUPW=<v>      -> upper arm width"));
  Serial.println(F("  ARMUPH=<v>      -> upper arm height"));
  Serial.println(F("  ARMUPD=<v>      -> upper arm depth"));
  Serial.println(F("  ARMFRW=<v>      -> forearm width"));
  Serial.println(F("  ARMFRH=<v>      -> forearm height"));
  Serial.println(F("  ARMFRD=<v>      -> forearm depth"));
  Serial.println(F("  ARMWRW=<v>      -> wrist width"));
  Serial.println(F("  ARMWRH=<v>      -> wrist height"));
  Serial.println(F("  ARMWRD=<v>      -> wrist depth"));
  Serial.println(F("  ARMFGW=<v>      -> finger width"));
  Serial.println(F("  ARMFGH=<v>      -> finger height"));
  Serial.println(F("  ARMFGD=<v>      -> finger depth"));
  Serial.println(F("  ARMBCY=<v>      -> base cylinder Y offset"));
  Serial.println(F("  ARMBBY=<v>      -> base block Y offset"));
  Serial.println(F("  ARMGRY=<v>      -> gripper Y offset"));
  Serial.println(F("  ARMOFFBASE is fixed: base servo angle == base yaw"));
  Serial.println(F("  ARMOFFUPPER=<servoZero,memberZero,sign>  -> upper arm joint mapping"));
  Serial.println(F("  ARMOFFFORE=<servoZero,memberZero,sign>   -> forearm joint mapping"));
  Serial.println(F("  ARMOFFFORER=<servoZero,memberZero,sign>  -> forearm roll joint mapping"));
  Serial.println(F("  ARMOFFWRISTP=<servoZero,memberZero,sign> -> wrist pitch joint mapping"));
  Serial.println(F("  ARMOFFWRISTR=<servoZero,memberZero,sign> -> wrist roll joint mapping"));
  Serial.println(F("  ARMOFFGRIP=<servoZero,memberZero,sign>   -> gripper joint mapping"));
  Serial.println(F(""));
  Serial.print(F("ARMBASER=")); Serial.println(armBaseRadius, 2);
  Serial.print(F("ARMBASEH=")); Serial.println(armBaseHeight, 2);
  Serial.print(F("ARMBBLW=")); Serial.println(armBaseBlockW, 2);
  Serial.print(F("ARMBBLH=")); Serial.println(armBaseBlockH, 2);
  Serial.print(F("ARMBBLD=")); Serial.println(armBaseBlockD, 2);
  Serial.print(F("ARMUPW=")); Serial.println(armUpperW, 2);
  Serial.print(F("ARMUPH=")); Serial.println(armUpperH, 2);
  Serial.print(F("ARMUPD=")); Serial.println(armUpperD, 2);
  Serial.print(F("ARMFRW=")); Serial.println(armForeW, 2);
  Serial.print(F("ARMFRH=")); Serial.println(armForeH, 2);
  Serial.print(F("ARMFRD=")); Serial.println(armForeD, 2);
  Serial.print(F("ARMWRW=")); Serial.println(armWristW, 2);
  Serial.print(F("ARMWRH=")); Serial.println(armWristH, 2);
  Serial.print(F("ARMWRD=")); Serial.println(armWristD, 2);
  Serial.print(F("ARMFGW=")); Serial.println(armFingerW, 2);
  Serial.print(F("ARMFGH=")); Serial.println(armFingerH, 2);
  Serial.print(F("ARMFGD=")); Serial.println(armFingerD, 2);
  Serial.print(F("ARMBCY=")); Serial.println(armBaseCylinderYOffset, 2);
  Serial.print(F("ARMBBY=")); Serial.println(armBaseBlockYOffset, 2);
  Serial.print(F("ARMGRY=")); Serial.println(armGripperYOffset, 2);
  Serial.print(F("ARMOFFBASE=")); Serial.print(armBaseServoZeroDeg, 2); Serial.print(','); Serial.print(armBaseMemberZeroDeg, 2); Serial.print(','); Serial.println(armBaseServoSign, 2);
  Serial.print(F("ARMOFFUPPER=")); Serial.print(armUpperServoZeroDeg, 2); Serial.print(','); Serial.print(armUpperMemberZeroDeg, 2); Serial.print(','); Serial.println(armUpperServoSign, 2);
  Serial.print(F("ARMOFFFORE=")); Serial.print(armForeServoZeroDeg, 2); Serial.print(','); Serial.print(armForeMemberZeroDeg, 2); Serial.print(','); Serial.println(armForeServoSign, 2);
  Serial.print(F("ARMOFFFORER=")); Serial.print(armForearmRollServoZeroDeg, 2); Serial.print(','); Serial.print(armForearmRollMemberZeroDeg, 2); Serial.print(','); Serial.println(armForearmRollServoSign, 2);
  Serial.print(F("ARMOFFWRISTP=")); Serial.print(armWristPitchServoZeroDeg, 2); Serial.print(','); Serial.print(armWristPitchMemberZeroDeg, 2); Serial.print(','); Serial.println(armWristPitchServoSign, 2);
  Serial.print(F("ARMOFFWRISTR=")); Serial.print(armWristRollServoZeroDeg, 2); Serial.print(','); Serial.print(armWristRollMemberZeroDeg, 2); Serial.print(','); Serial.println(armWristRollServoSign, 2);
  Serial.print(F("ARMOFFGRIP=")); Serial.print(armGripServoZeroDeg, 2); Serial.print(','); Serial.print(armGripMemberZeroDeg, 2); Serial.print(','); Serial.println(armGripServoSign, 2);
  Serial.println(F(""));
}

// Prints manipulator hardware panel.
void printManipulatorHardwarePanel() {
  Serial.println(F("[MANIPULATOR HARDWARE]"));
  Serial.println(F("  BASE source = CH0 (stepper/DC block) or CH1 (servo block)"));
  Serial.println(F("  JOINTS upper/fore/roll/wrist/gripper -> runtime servo channels"));
  Serial.println(F("  AUX PWM 12..15 -> base, upper, forearm, forearm roll force outputs"));
  Serial.println(F(""));
  Serial.print(F("stepper_coil_pins="));
  Serial.print(STEPPER_COIL_PINS[0]); Serial.print(','); Serial.print(STEPPER_COIL_PINS[1]); Serial.print(','); Serial.print(STEPPER_COIL_PINS[2]); Serial.print(','); Serial.println(STEPPER_COIL_PINS[3]);
  Serial.print(F("dc_motor_dir_pins="));
  Serial.print(DC_MOTOR_DIR_PINS[0]); Serial.print(','); Serial.println(DC_MOTOR_DIR_PINS[1]);
  Serial.print(F("stepper_power_pwm_pin=")); Serial.println(STEPPER_POWER_PWM_PIN);
  Serial.print(F("stepper_feedback_pin=")); Serial.println(STEPPER_FEEDBACK_PIN);
  Serial.print(F("fallback_servo_pins="));
  for (uint8_t i = 0; i < 11; i++) {
    if (i) Serial.print(',');
    Serial.print(FALLBACK_SERVO_PINS[i]);
  }
  Serial.println();
  Serial.print(F("fallback_pwm_pins="));
  for (uint8_t i = 0; i < 4; i++) {
    if (i) Serial.print(',');
    Serial.print(FALLBACK_PWM_PINS[i]);
  }
  Serial.println();
  Serial.print(F("manip_base_channel=")); Serial.println(manipProfile.baseChannel);
  Serial.print(F("manip_upper_channel=")); Serial.println(manipProfile.upperChannel);
  Serial.print(F("manip_fore_channel=")); Serial.println(manipProfile.foreChannel);
  Serial.print(F("manip_forearm_roll_channel=")); Serial.println(manipProfile.forearmRollChannel);
  Serial.print(F("manip_wrist_pitch_channel=")); Serial.println(manipProfile.wristPitchChannel);
  Serial.print(F("manip_wrist_roll_channel=")); Serial.println(manipProfile.wristRotChannel);
  Serial.print(F("manip_gripper_channel=")); Serial.println(manipProfile.gripperChannel);
  Serial.print(F("aux_force_pwm_base_pin=")); Serial.println(FALLBACK_PWM_PINS[0]);
  Serial.print(F("aux_force_pwm_upper_pin=")); Serial.println(FALLBACK_PWM_PINS[1]);
  Serial.print(F("aux_force_pwm_forearm_pin=")); Serial.println(FALLBACK_PWM_PINS[2]);
  Serial.print(F("aux_force_pwm_forearm_roll_pin=")); Serial.println(FALLBACK_PWM_PINS[3]);
  Serial.println(F(""));
}

// Prints one I2C address in 0xNN format.
static void printI2CAddressHex(uint8_t addr) {
  static const char kHexDigits[] = "0123456789ABCDEF";
  Serial.print(F("0x"));
  Serial.print(kHexDigits[(addr >> 4) & 0x0F]);
  Serial.print(kHexDigits[addr & 0x0F]);
}

// Parses strict I2C address text in hexadecimal format (accepts 0xNN or NN).
static bool parseStrictI2CAddressHexValue(const char* text, uint8_t& value) {
  if (text == NULL) return false;

  while (*text != '\0' && isspace((unsigned char)*text)) {
    ++text;
  }
  if (*text == '\0') return false;

  if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
    text += 2;
  }

  if (*text == '\0') return false;

  uint16_t parsed = 0;
  uint8_t digits = 0;
  while (*text != '\0' && isxdigit((unsigned char)*text)) {
    char c = *text++;
    uint8_t nibble = 0;
    if (c >= '0' && c <= '9') nibble = (uint8_t)(c - '0');
    else if (c >= 'a' && c <= 'f') nibble = (uint8_t)(10 + (c - 'a'));
    else nibble = (uint8_t)(10 + (c - 'A'));

    parsed = (uint16_t)((parsed << 4) | nibble);
    if (parsed > 0x7F) return false;
    ++digits;
  }

  if (digits == 0) return false;

  while (*text != '\0' && isspace((unsigned char)*text)) {
    ++text;
  }
  if (*text != '\0') return false;

  value = (uint8_t)parsed;
  return true;
}

// Prints I2C config panel.
void printI2CConfigPanel() {
  Serial.println(F("[I2C]"));
  Serial.println(F("  PCAADDR=<0xNN>  -> PCA9685 address"));
  Serial.println(F("  INAADDR=<0xNN>  -> INA219 #1 address"));
  Serial.println(F("  INA2ADDR=<0xNN> -> INA219 #2 address"));
  Serial.println(F("  MPUADDR=<0xNN>  -> MPU6050 #1 address"));
  Serial.println(F("  MPU2ADDR=<0xNN> -> MPU6050 #2 address (0x00 disables)"));
  Serial.println(F("  MAGADDR=<0xNN>  -> GY-273 address (0x00 = auto detect)"));
  Serial.println(F(""));
  Serial.print(F("PCAADDR=")); printI2CAddressHex(pca9685Addr); Serial.println();
  Serial.print(F("INAADDR=")); printI2CAddressHex(ina219Addr1); Serial.println();
  Serial.print(F("INA2ADDR=")); printI2CAddressHex(ina219Addr2); Serial.println();
  Serial.print(F("MPUADDR=")); printI2CAddressHex(mpu6050Addr1); Serial.println();
  Serial.print(F("MPU2ADDR=")); printI2CAddressHex(mpu6050Addr2); Serial.println();
  Serial.print(F("MAGADDR=")); printI2CAddressHex(compassAddr); Serial.println();
  Serial.println(F(""));
}

// Prints servo channels config panel.
void printServoChannelsConfigPanel() {
  syncProcessingRobotProfiles();
  Serial.println(F("[SERVO CHANNELS]"));
  Serial.println(F("  CH0 = base via potentiometer/stepper // CH1 = base via channel 1"));
  Serial.println(F("  CH2..11 = servo channels"));
  Serial.println(F("  CH12..15 = PWM channels"));
  Serial.println(F("  Logical arm mapping:"));
  Serial.println(F("    BASECH=<0|1>       -> select base source: 0=POT/stepper, 1=channel 1"));
  Serial.println(F("    UPPERCH=<2..11>     -> map upper arm member"));
  Serial.println(F("    FORECH=<2..11>      -> map forearm member"));
  Serial.println(F("    FOREROLLCH=<2..11>  -> map forearm roll member"));
  Serial.println(F("    WRISTPITCHCH=<2..11>-> map wrist pitch member"));
  Serial.println(F("    WRISTROTCH=<2..11>  -> map wrist rotation member"));
  Serial.println(F("    GRIPCH=<2..11>      -> map gripper member"));
  Serial.println(F(""));
  Serial.print(F("BASECH=")); Serial.println(manipProfile.baseChannel);
  Serial.print(F("UPPERCH=")); Serial.println(manipProfile.upperChannel);
  Serial.print(F("FORECH=")); Serial.println(manipProfile.foreChannel);
  Serial.print(F("FOREROLLCH=")); Serial.println(manipProfile.forearmRollChannel);
  Serial.print(F("WRISTPITCHCH=")); Serial.println(manipProfile.wristPitchChannel);
  Serial.print(F("WRISTROTCH=")); Serial.println(manipProfile.wristRotChannel);
  Serial.print(F("GRIPCH=")); Serial.println(manipProfile.gripperChannel);
  Serial.println(F(""));
  Serial.println(F("  Servo calibration examples:"));
  Serial.println(F("    RAMPSPD=1.0   -> max ramp step in deg/update"));
  Serial.println(F("    RAMPSMOOTH=1.0-> ramp smoothing factor (lower = softer)"));
  Serial.println(F("    OFFSET1=10    -> add +10 degrees on CH1"));
  Serial.println(F("    SPAN1=1.050   -> scale CH1 movement by 1.05"));
  Serial.println(F("    DIR1=-1       -> invert CH1 direction"));
  Serial.println(F("    RAMP1=1       -> enable smooth ramp on CH1"));
  Serial.println(F("    Auto torque / OPT removed; PWM CH12..15 starts fixed at 50%"));
  Serial.println(F("    MIN1=20       -> minimum command angle for CH1"));
  Serial.println(F("    MAX1=160      -> maximum command angle for CH1"));
  Serial.println(F(""));
  Serial.print(F("RAMPSPD=")); Serial.print(servoMoveStepDeg, 3); Serial.println(F("     -> sets the maximum movement speed"));
  Serial.print(F("RAMPSMOOTH=")); Serial.print(servoRampSmoothing, 3); Serial.println(F("  -> makes movement softer and smoother"));
  Serial.println(F(""));
  for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++) {
    printChannelConfig(ch);
  }
}

// Prints PWM boot config panel.
void printPwmBootConfigPanel() {
  Serial.println(F("[PWM BOOT CH12..15]"));
  Serial.println(F("  PWMBOOT12..15 are fixed at 50% on startup."));
  Serial.println(F("  Runtime PWM commands can still change CH12..15 after boot."));
  Serial.println(F(""));
  Serial.print(F("PWMBOOT12=")); Serial.println(DEFAULT_MANUAL_PWM_PERCENT);
  Serial.print(F("PWMBOOT13=")); Serial.println(DEFAULT_MANUAL_PWM_PERCENT);
  Serial.print(F("PWMBOOT14=")); Serial.println(DEFAULT_MANUAL_PWM_PERCENT);
  Serial.print(F("PWMBOOT15=")); Serial.println(DEFAULT_MANUAL_PWM_PERCENT);
  Serial.println(F(""));
}

// Prints stepper config panel.
void printStepperConfigPanel() {
  Serial.println(F("[STEPPER]"));
  Serial.println(F("  "));
  Serial.println(F("  ---[AUTOMATION]---"));
  Serial.println(F("  CH0MODE=<0|1|name> -> 0 STEPPER, 1 DC_MOTOR"));
  Serial.println(F("  STEPMODE=<0..3|name> -> 0 FULLSTEP_BIPOLAR, 1 HALFSTEP_BIPOLAR, 2 FULLSTEP_UNIPOLAR, 3 HALFSTEP_UNIPOLAR"));
  Serial.println(F("  LIMITSTEPPERCALIB -> automatic stepper limit calibration inside CFG"));
  Serial.println(F("  AUTOSTEPPER     -> automatic stepper tuning inside CFG"));
  Serial.println(F("  AUTOSTATUS      -> show automatic tuning progress / best values / feedback window"));
  Serial.println(F("  AUTOSTOP        -> stop automatic stepper tuning and restore the original values"));
  Serial.println(F("  STEPDIAG        -> show direct stepper diagnostic block"));
  Serial.println(F("  STEPTEST=F,200,80,2200 -> direct open-loop step test"));
  Serial.println(F("  ------------------"));
  Serial.println(F(""));
  Serial.println(F("  ZEROADC=<v>     -> potentiometer zero point"));
  Serial.println(F("  SPANRAW=<v>     -> raw span for one turn"));
  Serial.println(F("  MINSTEPUS=<v>   -> minimum step delay"));
  Serial.println(F("  MAXSTEPUS=<v>   -> maximum step delay"));
  Serial.println(F("  STOPENTER=<v>   -> hold band enter threshold"));
  Serial.println(F("  STOPEXIT=<v>    -> hold band exit threshold"));
  Serial.println(F("  STOPMOTORENTER=<v> / STOPMOTOREXIT=<v> -> explicit aliases"));
  Serial.println(F("  PWMFAR=<v>      -> PWM when angle error is large"));
  Serial.println(F("  PWMNEAR=<v>     -> PWM when angle error is moderate"));
  Serial.println(F("  PWMHOLD=<v>     -> holding PWM"));
  Serial.println(F("  ALPHAANG=<v>    -> angle filter factor"));
  Serial.println(F(""));
  Serial.print(F("CH0MODE=")); Serial.print((int)ch0DriveMode); Serial.print(F(" [")); Serial.print(getCh0DriveModeName(ch0DriveMode)); Serial.println(F("]"));
  Serial.print(F("STEPMODE=")); Serial.print((int)stepperDriveMode); Serial.print(F(" [")); Serial.print(getStepperDriveModeName(stepperDriveMode)); Serial.println(F("]"));
  Serial.print(F("ZEROADC=")); Serial.println(stepperCalibrationZeroADC);
  Serial.print(F("SPANRAW=")); Serial.println(stepperOneTurnSpanRaw);
  Serial.print(F("MINSTEPUS=")); Serial.println(minStepDelayUs);
  Serial.print(F("MAXSTEPUS=")); Serial.println(maxStepDelayUs);
  Serial.print(F("STOPENTER=")); Serial.println(stepperStopEnter, 3);
  Serial.print(F("STOPEXIT=")); Serial.println(stepperStopExit, 3);
  Serial.print(F("PWMFAR=")); Serial.println(movePwmFar);
  Serial.print(F("PWMNEAR=")); Serial.println(movePwmNear);
  Serial.print(F("PWMHOLD=")); Serial.println(holdPwm);
  Serial.print(F("ALPHAANG=")); Serial.println(angleFilterAlpha, 3);
  Serial.println(F(""));
}

// Prints serial runtime panel.
void printSerialRuntimePanel() {
  Serial.println(F("[SERIAL RUNTIME]"));
  Serial.println(F("  READY?          -> complete ASCII handshake and enter HEX runtime"));
  Serial.println(F("  Runtime RX/TX   -> A55A HEX frames on Serial0"));
  Serial.println(F("  CFG=1234        -> return to ASCII configuration mode"));
  Serial.println(F(""));
}

// Prints compass calibration panel.
void printCompassCalibrationPanel() {
  Serial.println(F("[COMPASS CALIBRATION]"));
  Serial.println(F("  MAGCAL=0/1      -> stop/start live compass calibration"));
  Serial.println(F("  MAGCALCFG=minX,maxX,minY,maxY,minZ,maxZ,offX,offY,offZ,scaleX,scaleY,scaleZ"));
  Serial.println(F(""));
  Serial.print(F("MAGCALCFG="));
  Serial.print(compassCal.minX); Serial.print(','); Serial.print(compassCal.maxX); Serial.print(',');
  Serial.print(compassCal.minY); Serial.print(','); Serial.print(compassCal.maxY); Serial.print(',');
  Serial.print(compassCal.minZ); Serial.print(','); Serial.print(compassCal.maxZ); Serial.print(',');
  Serial.print(compassCal.offsetX, 3); Serial.print(','); Serial.print(compassCal.offsetY, 3); Serial.print(',');
  Serial.print(compassCal.offsetZ, 3); Serial.print(',');
  Serial.print(compassCal.scaleX, 5); Serial.print(','); Serial.print(compassCal.scaleY, 5); Serial.print(','); Serial.println(compassCal.scaleZ, 5);
  Serial.print(F("MAGCAL_ACTIVE=")); Serial.println(compassCalRuntime.active ? 1 : 0);
  Serial.println(F(""));
}

// Prints stabilization pressure collision panel.
void printStabilizationPressureCollisionPanel() {
  Serial.println(F("[STABILIZATION / PRESSURE / COLLISION]"));
  Serial.println(F("  GSTAB=0/1       -> gripper/wrist stabilizer from MPU1 (runtime)"));
  Serial.println(F("  ASTAB=0/1       -> arm stabilization from MPU2 (saved)"));
  Serial.println(F("  ASTABGAIN=<v>   -> arm stabilization gain"));
  Serial.println(F("  ASTABTH=<v>     -> arm stabilization threshold"));
  Serial.println(F("  GPRESS=0/1      -> grip pressure profiling using INA1 (saved)"));
  Serial.println(F("  GPGAIN=<v>      -> grip pressure gain"));
  Serial.println(F("  GIDLEMA / GPRESSMA / GPRESSVAL -> runtime diagnostics only (read-only)"));
  Serial.println(F("  COLL=0/1        -> collision DISABLED/ENABLED (CFG persistent)"));
  Serial.println(F(""));
  Serial.print(F("GSTAB=")); Serial.println(wristGimbalEnabled ? 1 : 0);
  Serial.print(F("ASTAB=")); Serial.println(armStabilizeEnable ? 1 : 0);
  Serial.print(F("ASTABGAIN=")); Serial.println(armStabilizeGain, 3);
  Serial.print(F("ASTABTH=")); Serial.println(armStabilizeThreshold, 3);
  Serial.print(F("GPRESS=")); Serial.println(gripPressureEnable ? 1 : 0);
  Serial.print(F("GPGAIN=")); Serial.println(gripPressureGain, 3);
  Serial.print(F("GIDLEMA[mA]: ")); Serial.println(gripIdleCurrentAvg, 1);
  Serial.print(F("GPRESSMA[mA]: ")); Serial.println(gripPressureDeltaMa);
  Serial.print(F("GPRESSVAL: ")); Serial.println(gripPressureEstimate, 2);
  Serial.print(F("COLL=")); Serial.println(collisionRuntimeEnabled ? 1 : 0);
  Serial.print(F("COLL_STATE: ")); Serial.println(collisionRuntimeEnabled ? F("ENABLED/YES") : F("DISABLED/NO"));
  Serial.println(F(""));
}

// Prints active robot runtime state summary.
void printActiveRobotRuntimeStateSummary() {
  Serial.println(F(""));
  Serial.print(F("ACTIVE ROBOT: "));
  Serial.println(getRobotControlModeName(robotControlMode));
  switch (robotControlMode) {
    case ROBOT_MODE_VEHICLE:
      Serial.print(F("LIGHT FOR VEHICLE: "));
      Serial.println(vehicleLightsCommand ? F("ENABLED (YES)") : F("DISABLED (NO)"));
      Serial.print(F("LIDAR SCAN FOR VEHICLE: "));
      Serial.println(vehicleLidarScanCommand ? F("ENABLED (YES)") : F("DISABLED (NO)"));
      break;
    case ROBOT_MODE_DRONE:
      Serial.print(F("CAMERA RECORD FOR DRONE: "));
      Serial.println(droneCameraRecordCommand ? F("ENABLED (YES)") : F("DISABLED (NO)"));
      break;
    case ROBOT_MODE_MANIPULATOR:
    default:
      Serial.print(F("COLLISION FOR MANIPULATOR: "));
      Serial.println(collisionRuntimeEnabled ? F("ENABLED (YES)") : F("DISABLED (NO)"));
      break;
  }
}

// Prints boot help.
void printBootHelp() {
  Serial.println(F("=============== QUICK HELP ==============="));
  Serial.println(F(""));
  Serial.println(F("[MODE ENTRY]"));
  Serial.println(F("  'BOOT?' / 'BOOTRESET' -> reopen boot screen without hardware reset"));
  Serial.println(F("  'READY?' -> start runtime"));
  Serial.println(F("  'CFG=1234' -> configuration mode"));
  Serial.println(F(""));
}

// Prints channel config.
void printChannelConfig(uint8_t ch) {
  Serial.print(F("CH"));
  Serial.print(ch);
  Serial.print(F(" TYPE: "));
  if (ch == 0) {
    Serial.println(F("STEPPER"));
  } else if (isServoChannel(ch)) {
    Serial.println(F("SERVO"));
  } else {
    Serial.println(F("PWM"));
  }

  if (channelSupportsCalibration(ch)) {
    Serial.print(F("OFFSET")); Serial.print(ch); Serial.print(F("=")); Serial.println(servos.offset[ch]);
    Serial.print(F("SPAN"));   Serial.print(ch); Serial.print(F("=")); Serial.println(getServoSpan(ch), 3);
    Serial.print(F("DIR"));    Serial.print(ch); Serial.print(F("=")); Serial.println(servos.direction[ch]);
    if (isServoChannel(ch)) {
      Serial.print(F("RAMP")); Serial.print(ch); Serial.print(F("=")); Serial.println(getServoRamp(ch) ? 1 : 0);
      Serial.print(F("MIN"));  Serial.print(ch); Serial.print(F("=")); Serial.println(getServoMinLimit(ch), 1);
      Serial.print(F("MAX"));  Serial.print(ch); Serial.print(F("=")); Serial.println(getServoMaxLimit(ch), 1);
    }
  } else {
    Serial.print(F("PWMBOOT")); Serial.print(ch); Serial.print(F("=")); Serial.println(DEFAULT_MANUAL_PWM_PERCENT);
  }
  Serial.println();
}

// Prints current config.
void printCurrentConfig() {
  Serial.println(F(""));
  Serial.println(F("================== CONFIG MENU =================="));
  Serial.print(F("active_robot=")); Serial.println(getRobotControlModeName(robotControlMode));
  Serial.println(F("-------------------------------------------------"));
  Serial.println(F("[SYSTEM]"));
  Serial.println(F("  ERASEEEPROM     -> erase EEPROM"));
  Serial.println(F("  DEFAULT         -> restore default values"));
  Serial.println(F("  SAVE            -> save to EEPROM"));
  Serial.println(F("  LOAD            -> load from EEPROM"));
  Serial.println(F("  SHOW            -> print current settings"));
  Serial.println(F("  EXIT / CFGOFF   -> leave config mode"));
  Serial.println(F("  ROBOT=<0|1|2|MANIPULATOR|VEHICLE|DRONE>"));
  Serial.println(F(""));
  Serial.println(F("==================== SUMMARY ===================="));
  Serial.println(F(""));

  switch (robotControlMode) {
    case ROBOT_MODE_VEHICLE:
      printVehicleConfigPanels();
      printI2CConfigPanel();
      printCompassCalibrationPanel();
      printSerialRuntimePanel();
      break;
    case ROBOT_MODE_DRONE:
      printDroneConfigPanels();
      printI2CConfigPanel();
      printCompassCalibrationPanel();
      printSerialRuntimePanel();
      break;
    case ROBOT_MODE_MANIPULATOR:
    default:
      printManipulatorGeometryPanel();
      printManipulatorHardwarePanel();
      printI2CConfigPanel();
      printCompassCalibrationPanel();
      printServoChannelsConfigPanel();
      printPwmBootConfigPanel();
      printStepperConfigPanel();
      printStabilizationPressureCollisionPanel();
      printSerialRuntimePanel();
      break;
  }

  Serial.println(F("================================================="));
}

// Utility: runtime ready for commands.
bool runtimeReadyForCommands() {
  return (rt.active || rt.configMode);
}

// Checks whether ready for runtime commands.
bool requireReadyForRuntimeCommands() {
  if (runtimeReadyForCommands()) {
    return true;
  }

  if (currentTextCommandSource == 1 && controlOwner != CONTROL_OWNER_RUNTIME_HEX) {
    controlOwner = CONTROL_OWNER_PRIMARY_ASCII;
    enterOperationMode();
    return true;
  }

  Serial.println(F("DENIED: SEND READY? OR CFG=1234 FIRST"));
  return false;
}

// Utility: runtime ascii telemetry only.
bool runtimeAsciiTelemetryOnly() {
  // Single-serial HEX architecture: runtime is always HEX-framed on Serial0.
  return false;
}

// Checks whether primary ascii status messages.
bool allowPrimaryAsciiStatusMessages() {
  // During runtime the primary serial port must emit only the telemetry stream.
  // Keep all extra ASCII status/debug text disabled until runtime exits.
  return !rt.active;
}

// Utility: canonicalize command text.
void canonicalizeCommandText(char* cmd) {
  if (cmd == NULL) return;

  char* start = cmd;
  while (*start != '\0' && isspace((unsigned char)*start)) {
    ++start;
  }

  if (start != cmd) {
    memmove(cmd, start, strlen(start) + 1);
  }

  char* end = cmd + strlen(cmd);
  while (end > cmd && isspace((unsigned char)end[-1])) {
    --end;
    *end = '\0';
  }

  for (char* p = cmd; *p != '\0'; ++p) {
    *p = (char)toupper((unsigned char) * p);
  }
}

// Checks whether immediate config command.
bool isImmediateConfigCommand(const char* cmd) {
  if (cmd == NULL || cmd[0] == '\0') return false;

  return strncmp(cmd, "CFG=", 4) == 0 ||
         strcmp(cmd, "SHOW") == 0 ||
         strcmp(cmd, "SAVE") == 0 ||
         strcmp(cmd, "LOAD") == 0 ||
         strcmp(cmd, "DEFAULT") == 0 ||
         strcmp(cmd, "ERASEEEPROM") == 0 ||
         strcmp(cmd, "EXIT") == 0 ||
         strcmp(cmd, "CFGOFF") == 0 ||
         strcmp(cmd, "LIMITSTEPPERCALIB") == 0 || strcmp(cmd, "CALLIMITSTEPPER") == 0 ||
         strcmp(cmd, "LIMITSTEPPERCALIBSTATUS") == 0 || strcmp(cmd, "CALLIMITSTATUS") == 0 ||
         strcmp(cmd, "STEPMODE?") == 0 ||
         strcmp(cmd, "CH0MODE?") == 0 ||
         strcmp(cmd, "AUTOSTEPPER") == 0 ||
         strcmp(cmd, "AUTOSTATUS") == 0 ||
         strcmp(cmd, "AUTOSTOP") == 0 ||
         strcmp(cmd, "STEPPERCAL") == 0 ||
         strcmp(cmd, "STEPPERCALSTATUS") == 0;
}

// Checks whether known config assignment command.
bool isKnownConfigAssignmentCommand(const char* cmd) {
  if (cmd == NULL || cmd[0] == '\0') return false;
  if (strchr(cmd, '=') == NULL) return false;

  return strncmp(cmd, "ROBOT=", 6) == 0 ||
         strncmp(cmd, "STEPMODE=", 9) == 0 ||
         strncmp(cmd, "CH0MODE=", 8) == 0 ||
         strncmp(cmd, "COLL=", 5) == 0 ||
         strncmp(cmd, "PCAADDR=", 8) == 0 ||
         strncmp(cmd, "INAADDR=", 8) == 0 ||
         strncmp(cmd, "INA2ADDR=", 9) == 0 ||
         strncmp(cmd, "MPUADDR=", 8) == 0 ||
         strncmp(cmd, "MPU2ADDR=", 9) == 0 ||
         strncmp(cmd, "MAGADDR=", 8) == 0 ||
         strncmp(cmd, "ZEROADC=", 8) == 0 ||
         strncmp(cmd, "SPANRAW=", 8) == 0 ||
         strncmp(cmd, "MINSTEPUS=", 10) == 0 ||
         strncmp(cmd, "MAXSTEPUS=", 10) == 0 ||
         strncmp(cmd, "STOPENTER=", 10) == 0 ||
         strncmp(cmd, "STOPEXIT=", 9) == 0 ||
         strncmp(cmd, "STOPMOTORENTER=", 15) == 0 ||
         strncmp(cmd, "STOPMOTOREXIT=", 14) == 0 ||
         strncmp(cmd, "PWMFAR=", 7) == 0 ||
         strncmp(cmd, "PWMNEAR=", 8) == 0 ||
         strncmp(cmd, "PWMHOLD=", 8) == 0 ||
         strncmp(cmd, "ALPHAANG=", 9) == 0 ||
         strncmp(cmd, "RAMPSPD=", 8) == 0 ||
         strncmp(cmd, "RAMPSMOOTH=", 11) == 0 ||
         strncmp(cmd, "PWMBOOT12=", 10) == 0 ||
         strncmp(cmd, "PWMBOOT13=", 10) == 0 ||
         strncmp(cmd, "PWMBOOT14=", 10) == 0 ||
         strncmp(cmd, "PWMBOOT15=", 10) == 0 ||
         strncmp(cmd, "BASECH=", 7) == 0 ||
         strncmp(cmd, "UPPERCH=", 8) == 0 ||
         strncmp(cmd, "UPPERCHANNEL=", 13) == 0 ||
         strncmp(cmd, "FORECH=", 7) == 0 ||
         strncmp(cmd, "FORECHANNEL=", 12) == 0 ||
         strncmp(cmd, "FOREROLLCH=", 11) == 0 ||
         strncmp(cmd, "FOREARMROLLCH=", 14) == 0 ||
         strncmp(cmd, "WRISTPITCHCH=", 13) == 0 ||
         strncmp(cmd, "WRISTVERTCH=", 12) == 0 ||
         strncmp(cmd, "WRISTROTCH=", 11) == 0 ||
         strncmp(cmd, "WRISTROLLCH=", 12) == 0 ||
         strncmp(cmd, "GRIPCH=", 7) == 0 ||
         strncmp(cmd, "GRIPPERCH=", 10) == 0 ||
         strncmp(cmd, "ARMBASER=", 9) == 0 ||
         strncmp(cmd, "ARMBASEH=", 9) == 0 ||
         strncmp(cmd, "ARMBBLW=", 8) == 0 ||
         strncmp(cmd, "ARMBBLH=", 8) == 0 ||
         strncmp(cmd, "ARMBBLD=", 8) == 0 ||
         strncmp(cmd, "ARMUPW=", 7) == 0 ||
         strncmp(cmd, "ARMUPH=", 7) == 0 ||
         strncmp(cmd, "ARMUPD=", 7) == 0 ||
         strncmp(cmd, "ARMFRW=", 7) == 0 ||
         strncmp(cmd, "ARMFRH=", 7) == 0 ||
         strncmp(cmd, "ARMFRD=", 7) == 0 ||
         strncmp(cmd, "ARMWRW=", 7) == 0 ||
         strncmp(cmd, "ARMWRH=", 7) == 0 ||
         strncmp(cmd, "ARMWRD=", 7) == 0 ||
         strncmp(cmd, "ARMFGW=", 7) == 0 ||
         strncmp(cmd, "ARMFGH=", 7) == 0 ||
         strncmp(cmd, "ARMFGD=", 7) == 0 ||
         strncmp(cmd, "ARMBCY=", 7) == 0 ||
         strncmp(cmd, "ARMBBY=", 7) == 0 ||
         strncmp(cmd, "ARMGRY=", 7) == 0 ||
         strncmp(cmd, "ASTAB=", 6) == 0 ||
         strncmp(cmd, "ASTABGAIN=", 10) == 0 ||
         strncmp(cmd, "ASTABTH=", 8) == 0 ||
         strncmp(cmd, "GPRESS=", 7) == 0 ||
         strncmp(cmd, "GPGAIN=", 7) == 0 ||
         strncmp(cmd, "MAGCALCFG=", 10) == 0 ||
         strncmp(cmd, "ARMOFFBASE=", 11) == 0 ||
         strncmp(cmd, "ARMOFFUPPER=", 12) == 0 ||
         strncmp(cmd, "ARMOFFFORE=", 11) == 0 ||
         strncmp(cmd, "ARMOFFFORER=", 12) == 0 ||
         strncmp(cmd, "ARMOFFWRISTP=", 13) == 0 ||
         strncmp(cmd, "ARMOFFWRISTR=", 13) == 0 ||
         strncmp(cmd, "ARMOFFGRIP=", 11) == 0 ||
         strncmp(cmd, "VEH", 3) == 0 ||
         strncmp(cmd, "DRN", 3) == 0 ||
         strncmp(cmd, "DRONELINKHOLD=", 14) == 0 ||
         strncmp(cmd, "SAFEBAT=", 8) == 0;
}

// Checks whether numeric calibration command.
bool isNumericCalibrationCommand(const char* cmd) {
  if (cmd == NULL || cmd[0] == '\0') return false;

  return strncmp(cmd, "OFFSET", 6) == 0 ||
         strncmp(cmd, "OFF", 3) == 0 ||
         strncmp(cmd, "SPAN", 4) == 0 ||
         strncmp(cmd, "RAMP", 4) == 0 ||
         strncmp(cmd, "SERVOMIN", 8) == 0 ||
         strncmp(cmd, "SERVOMAX", 8) == 0 ||
         strncmp(cmd, "MIN", 3) == 0 ||
         strncmp(cmd, "MAX", 3) == 0 ||
         strncmp(cmd, "DIR", 3) == 0 ||
         strncmp(cmd, "STEPTEST=", 9) == 0;
}

// Checks whether robot runtime command text.
bool isRobotRuntimeCommandText(const char* cmd) {
  if (cmd == NULL || cmd[0] == '\0') return false;

  if (strncmp(cmd, "ARMJ=", 5) == 0 || strcmp(cmd, "HOME") == 0) return true;
  if (strncmp(cmd, "GSTAB=", 6) == 0 || strncmp(cmd, "STAB=", 5) == 0) return true;
  if (strncmp(cmd, "TRACK=", 6) == 0) return true;
  if (strncmp(cmd, "CAM=", 4) == 0) return true;
  if (strncmp(cmd, "LIGHT=", 6) == 0 || strncmp(cmd, "SCAN=", 5) == 0) return true;
  if (strncmp(cmd, "MOVE=", 5) == 0 || strncmp(cmd, "FLY=", 4) == 0) return true;
  if (strncmp(cmd, "CAMREC=", 7) == 0 || strcmp(cmd, "TAKEOFF") == 0 || strcmp(cmd, "LAND") == 0 || strcmp(cmd, "STOP") == 0) return true;
  if (strncmp(cmd, "MAGCAL=", 7) == 0) return true;
  return false;
}

// Utility: should route to config command.
bool shouldRouteToConfigCommand(const char* cmd) {
  if (cmd == NULL || cmd[0] == '\0') return false;
  if (isImmediateConfigCommand(cmd)) return true;
  if (isKnownConfigAssignmentCommand(cmd)) return true;
  if (isNumericCalibrationCommand(cmd)) return false;
  if (isRobotRuntimeCommandText(cmd)) return false;

  if (rt.configMode && strchr(cmd, '=') != NULL) {
    return true;
  }

  return false;
}

// Applies servo value.
void applyServoValue(uint8_t ch, float value, ValueMode mode) {
  if (ch >= MAX_CHANNELS) return;

  switch (mode) {
    case APPLY_TARGET:
      if (ch == 0) {
        float ang = clampValue(value, STEPPER_INPUT_MIN, STEPPER_INPUT_MAX);
        if (fabsf(ang - servos.target[0]) < servoTargetEpsilonDeg) break;
        if (((ang - stepper.currentAngle) * stepper.integral) < 0.0f) {
          stepper.integral *= 0.35f;
        }
        stepper.inHoldBand = false;
        stepper.holdStartMs = 0;
        stepper.targetAngle = ang;
        servos.target[0] = ang;
      } else if (isServoChannel(ch)) {
        float limited = clampServoTargetToLimits(ch, clampServoInputToChannelRange(ch, value));
        if (!getServoNeutral(ch) && fabsf(limited - servos.target[ch]) < servoTargetEpsilonDeg) break;
        servos.target[ch] = limited;
      } else if (isPwmChannel(ch)) {
        float limited = clampValue(value, 0.0f, 100.0f);
        if (fabsf(limited - servos.target[ch]) < 0.5f) break;
        servos.target[ch] = limited;
      }
      break;

    case APPLY_OFFSET:
      if (channelSupportsCalibration(ch)) {
        servos.offset[ch] = (int16_t)value;
      }
      break;

    case APPLY_SPAN:
      if (channelSupportsCalibration(ch)) {
        setServoSpan(ch, value);
      }
      break;

    case APPLY_RAMP:
      if (isServoChannel(ch)) {
        setServoRamp(ch, ((int)value != 0));
      }
      break;
    case APPLY_OPTIMIZE:
      // Auto torque / smart power optimization was removed. Keep legacy
      // command paths harmless by forcing optimization off.
      if (isServoChannel(ch)) {
        servos.optimize[ch] = 0;
      }
      break;

    case APPLY_MIN_LIMIT:
      if (isServoChannel(ch)) {
        setServoMinLimit(ch, value);
        if (getServoMinLimit(ch) > getServoMaxLimit(ch)) setServoMaxLimit(ch, getServoMinLimit(ch));
        servos.target[ch] = clampServoTargetToLimits(ch, servos.target[ch]);
        servos.actual[ch] = clampServoTargetToLimits(ch, servos.actual[ch]);
      }
      break;

    case APPLY_MAX_LIMIT:
      if (isServoChannel(ch)) {
        setServoMaxLimit(ch, value);
        if (getServoMaxLimit(ch) < getServoMinLimit(ch)) setServoMinLimit(ch, getServoMaxLimit(ch));
        servos.target[ch] = clampServoTargetToLimits(ch, servos.target[ch]);
        servos.actual[ch] = clampServoTargetToLimits(ch, servos.actual[ch]);
      }
      break;
  }
}

// Parses strict float value.
bool parseStrictFloatValue(const char* text, float& value) {
  if (text == NULL) return false;

  while (*text != '\0' && isspace((unsigned char)*text)) {
    ++text;
  }
  if (*text == '\0') return false;

  char* endPtr = NULL;
  value = (float)strtod(text, &endPtr);
  if (endPtr == text) return false;

  while (*endPtr != '\0' && isspace((unsigned char)*endPtr)) {
    ++endPtr;
  }

  return (*endPtr == '\0');
}

// Parses strict bool 0/1 value.
bool parseStrictBool01Value(const char* text, bool& value) {
  if (text == NULL) return false;

  while (*text != '\0' && isspace((unsigned char)*text)) {
    ++text;
  }
  if (*text == '\0') return false;

  if (text[0] == '0') {
    const char* endPtr = text + 1;
    while (*endPtr != '\0' && isspace((unsigned char)*endPtr)) {
      ++endPtr;
    }
    if (*endPtr != '\0') return false;
    value = false;
    return true;
  }

  if (text[0] == '1') {
    const char* endPtr = text + 1;
    while (*endPtr != '\0' && isspace((unsigned char)*endPtr)) {
      ++endPtr;
    }
    if (*endPtr != '\0') return false;
    value = true;
    return true;
  }

  return false;
}

// Parses channel value command.
bool parseChannelValueCommand(const char* text, const char* prefix, uint8_t& ch, float& value) {
  const size_t prefixLen = strlen(prefix);
  if (strncmp(text, prefix, prefixLen) != 0) return false;

  const char* p = text + prefixLen;
  char* endPtr = NULL;
  long chLong = strtol(p, &endPtr, 10);
  if (endPtr == p || chLong < 0 || chLong > 255) return false;

  while (*endPtr == ' ') endPtr++;
  if (*endPtr != '.' && *endPtr != '=') return false;
  endPtr++;
  while (*endPtr == ' ') endPtr++;

  if (!parseStrictFloatValue(endPtr, value)) return false;

  ch = (uint8_t)chLong;
  return true;
}

bool parseManipulatorJointMapCommandValue(const char* text, float& servoZeroDeg, float& memberZeroDeg, float& servoSign) {
  if (text == NULL) return false;

  char buffer[96];
  strncpy(buffer, text, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  char* tokenA = strtok(buffer, ",");
  char* tokenB = strtok(NULL, ",");
  char* tokenC = strtok(NULL, ",");
  char* tokenExtra = strtok(NULL, ",");

  if (tokenA == NULL || tokenB == NULL || tokenC == NULL || tokenExtra != NULL) {
    return false;
  }

  if (!parseStrictFloatValue(tokenA, servoZeroDeg)) return false;
  if (!parseStrictFloatValue(tokenB, memberZeroDeg)) return false;
  if (!parseStrictFloatValue(tokenC, servoSign)) return false;

  servoZeroDeg = clampValue(servoZeroDeg, -360.0f, 360.0f);
  memberZeroDeg = clampValue(memberZeroDeg, -360.0f, 360.0f);
  servoSign = (servoSign < 0.0f) ? -1.0f : 1.0f;
  return true;
}


// Forward declarations for config echo helpers.
void printConfigAcceptedEcho(const char* echoText);
void printConfigInvalidEcho(const char* echoText);

// Handles numeric command.
void handleNumericCommand(const char* prefix, char* cmd, ValueMode mode) {
  if (!rt.configMode) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
    return;
  }

  uint8_t ch = 0;
  float value = 0.0f;
  if (!parseChannelValueCommand(cmd, prefix, ch, value)) {
    if (allowPrimaryAsciiStatusMessages()) printConfigInvalidEcho(cmd);
    return;
  }

  if (mode == APPLY_MIN_LIMIT || mode == APPLY_MAX_LIMIT) {
    if (!isServoChannel(ch)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("LIMITS VALID ONLY FOR CH1..11"));
      return;
    }
  } else {
    if (!channelSupportsCalibration(ch)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("CHANNEL DOES NOT SUPPORT THIS CALIBRATION"));
      return;
    }
    if (mode == APPLY_RAMP && ch == 0) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("RAMP NOT SUPPORTED FOR CH0"));
      return;
    }
  }

  applyServoValue(ch, value, mode);
  noteConfigSessionEdited(false);
  if (allowPrimaryAsciiStatusMessages()) printConfigAcceptedEcho(cmd);
}

// Handles direction command.
void handleDirectionCommand(char* cmd) {
  if (!rt.configMode) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
    return;
  }

  uint8_t ch = 0;
  float value = 0.0f;
  if (!parseChannelValueCommand(cmd, "DIR", ch, value)) {
    if (allowPrimaryAsciiStatusMessages()) printConfigInvalidEcho(cmd);
    return;
  }

  if (ch > LAST_SERVO_CH) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DIR VALID ONLY FOR CH0..11"));
    return;
  }

  servos.direction[ch] = (value < 0.0f) ? -1 : 1;
  noteConfigSessionEdited(false);
  if (allowPrimaryAsciiStatusMessages()) printConfigAcceptedEcho(cmd);
}


// Applies runtime servo target.
bool applyRuntimeServoTarget(uint8_t ch, float value) {
  if (!requireReadyForRuntimeCommands()) return false;
  if (ch > LAST_SERVO_CH) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("SERVO COMMAND VALID ONLY FOR CH0..11"));
    return false;
  }

  rt.servoOutputsArmed = true;

  if (!rt.active && rt.configMode) {
    servos.actual[ch] = clampServoTargetToLimits(ch, clampServoInputToChannelRange(ch, (float)value));
    writeChannel(ch, servos.actual[ch]);
  }

  if (ch == 0) {
    rt.stepperArmed = true;
  } else if (robotControlMode == ROBOT_MODE_MANIPULATOR && ch == getManipulatorMemberChannel(MANIP_MEMBER_BASE)) {
    rt.stepperArmed = false;
  }

  if (robotControlMode == ROBOT_MODE_MANIPULATOR && collisionRuntimeEnabled) {
    int8_t memberIdx = getManipulatorMemberIndexForChannel(ch);
    if (memberIdx >= 0) {
      float candidate = (ch == 0)
                        ? clampValue((float)value, STEPPER_INPUT_MIN, STEPPER_INPUT_MAX)
                        : clampServoTargetToLimits(ch, clampServoInputToChannelRange(ch, (float)value));
      if (!candidateMoveAllowed((uint8_t)memberIdx, candidate)) {
        collisionFlag = true;
        emitCollisionBlockedNotice();
        return false;
      }
    }
  } else if (!collisionRuntimeEnabled) {
    collisionFlag = false;
  }

  applyServoValue(ch, (float)value, APPLY_TARGET);

  if (VERBOSE_RUNTIME_ACKS && controlOwner != CONTROL_OWNER_RUNTIME_HEX) {
    Serial.print(F("TARGET CH"));
    Serial.print(ch);
    Serial.print(F("="));
    Serial.println(value);
  }
  return true;
}

// Applies runtime neutral state.
bool applyRuntimeNeutralState(uint8_t ch, bool enabled) {
  if (!requireReadyForRuntimeCommands()) return false;
  if (ch >= MAX_CHANNELS) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID CHANNEL"));
    return false;
  }

  if (!channelSupportsNeutral(ch)) {
    if (allowPrimaryAsciiStatusMessages()) {
      Serial.print(F("CHANNEL CH"));
      Serial.print(ch);
      Serial.println(F(" DOES NOT SUPPORT NEUTRAL"));
    }
    return false;
  }

  rt.servoOutputsArmed = true;
  setServoNeutral(ch, enabled);

  if (ch == 0) {
    readStepperAngle();
    stepper.targetAngle = stepper.currentAngle;
    servos.target[0] = stepper.currentAngle;
    servos.actual[0] = stepper.currentAngle;
    rt.stepperArmed = !getServoNeutral(ch);
    if (getServoNeutral(ch)) {
      releaseStepperCoils();
      stepperSetDriverPWM(0);
    }
    return true;
  }

  if (getServoNeutral(ch)) {
    setServoWriteValid(ch, false);
    forceServoNeutralOutput(ch);
  } else {
    writeChannel(ch, servos.actual[ch]);
  }

  if (VERBOSE_RUNTIME_ACKS && controlOwner != CONTROL_OWNER_RUNTIME_HEX) {
    Serial.print(F("NEUTRAL CH"));
    Serial.print(ch);
    Serial.print(F("="));
    Serial.println(getServoNeutral(ch) ? 1 : 0);
  }
  return true;
}

// Applies runtime PWM value.
bool applyRuntimePwmValue(uint8_t ch, float value) {
  if (!requireReadyForRuntimeCommands()) return false;
  if (!isPwmChannel(ch)) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("PWM COMMAND VALID ONLY FOR CH12..15"));
    return false;
  }

  rt.servoOutputsArmed = true;
  writeChannel(ch, value);

  if (controlOwner != CONTROL_OWNER_RUNTIME_HEX && allowPrimaryAsciiStatusMessages()) {
    Serial.print(F("PWM CH"));
    Serial.print(ch);
    Serial.print(F("="));
    Serial.println(value, 1);
  }
  return true;
}



// Clears config session dirty.
void clearConfigSessionDirty() {
  configSessionDirty = false;
}

// Notes config session edited.
void noteConfigSessionEdited(bool showConfigNow) {
  (void)showConfigNow;
  configSessionDirty = true;
}

// Prints accepted config echo.
void printConfigAcceptedEcho(const char* echoText) {
  Serial.print(F("ACCEPTED "));
  Serial.println(echoText);
}

// Prints invalid config echo.
void printConfigInvalidEcho(const char* echoText) {
  Serial.print(F("INVALID "));
  Serial.println(echoText);
}

// Leaves config/runtime control without forcing MCU reset.
void leaveInteractiveControlModes(bool keepConfigMode, const __FlashStringHelper* statusText) {
  rt.active = false;
  rt.configMode = keepConfigMode;
  rt.servoOutputsArmed = false;
  rt.stepperArmed = false;
  runtimeStreamActive = false;
  runtimeStreamFailsafeLatched = false;
  resetGenericRobotRuntimeState();
  releaseStepperCoils();
  stepperSetDriverPWM(0);
  stepperMotor.setSpeed(0.0f);
  stepperMotor.disableOutputs();
  controlOwner = keepConfigMode ? CONTROL_OWNER_PRIMARY_ASCII : CONTROL_OWNER_NONE;
  serialHexMode = false;  // return to ASCII mode for boot/cfg
  serialIndex = 0;
  while (Serial.available() > 0) Serial.read();
  // No secondary UART state to reset.
  if (statusText != NULL) {
    Serial.println(statusText);
  }
}

// Reopens the boot/banner screen without requiring a physical MCU reset.
void restartBootHandshakeScreen() {
  leaveInteractiveControlModes(false, NULL);
  controlOwner = CONTROL_OWNER_PRIMARY_ASCII;
  currentTextCommandSource = 1;
  serialHexMode = false;
  serialIndex = 0;
  bootCommandIgnoreUntilMs = 0;

  Serial.println(F(""));
  printBootBanner();
  printBootSummary();
  printBootHelp();
  printRobotSpecificConfigSummary();
  printRobotControlModeStatus();
}

// Handles config command.
void handleConfigCommand(char* cmd) {

  if (strncmp(cmd, "CFG=", 4) == 0) {
    if (currentTextCommandSource == 1) controlOwner = CONTROL_OWNER_PRIMARY_ASCII;
    long code = 0;
    if (!parseStrictLongValue(cmd + 4, code)) {
      Serial.println(F("INVALID CONFIG CODE"));
      return;
    }
    if (code == CONFIG_UNLOCK_CODE) {
      rt.active = false;
      rt.configMode = true;
      serialHexMode = false;
      serialIndex = 0;
      rt.servoOutputsArmed = false;
      rt.stepperArmed = false;
      runtimeStreamActive = false;
      runtimeStreamFailsafeLatched = false;
      resetGenericRobotRuntimeState();
      releaseStepperCoils();
      stepperSetDriverPWM(0);
      clearConfigSessionDirty();
      Serial.println(F("CONFIG MODE ENABLED"));
      Serial.println(F("USE 'SHOW' TO VIEW CURRENT SETTINGS"));
    } else {
      Serial.println(F("INVALID CONFIG CODE"));
    }
    return;
  }

  if (strcmp(cmd, "LIMITSTEPPERCALIB") == 0 || strcmp(cmd, "CALLIMITSTEPPER") == 0 || strcmp(cmd, "STEPPERCAL") == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    const bool wasActive = stepperCal.active;
    const StepperCalStage beforeStage = stepperCal.stage;
    handleCallimitStepperCommand();
    bool accepted = false;
    if ((!wasActive || beforeStage == STEPPER_CAL_IDLE || beforeStage == STEPPER_CAL_DONE) &&
        stepperCal.active && stepperCal.stage == STEPPER_CAL_WAIT_LIMIT1) {
      accepted = true;
    } else if (beforeStage == STEPPER_CAL_WAIT_LIMIT1 && stepperCal.stage == STEPPER_CAL_WAIT_LIMIT2) {
      accepted = true;
    } else if (beforeStage == STEPPER_CAL_WAIT_LIMIT2 && stepperCal.stage == STEPPER_CAL_DONE) {
      accepted = true;
    }
    if (allowPrimaryAsciiStatusMessages()) {
      if (accepted) printConfigAcceptedEcho(cmd);
      else printConfigInvalidEcho(cmd);
    }
    return;
  }
  if (strcmp(cmd, "LIMITSTEPPERCALIBSTATUS") == 0 || strcmp(cmd, "CALLIMITSTATUS") == 0) {
    printStepperCalibrationStatus();
    return;
  }
  if (strcmp(cmd, "STEPMODE?") == 0) {
    setStepperDriveMode(stepperDriveMode, true);
    return;
  }
  if (strcmp(cmd, "CH0MODE?") == 0) {
    setCh0DriveMode(ch0DriveMode, true);
    return;
  }
  if (strcmp(cmd, "AUTOSTEPPER") == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    const bool wasActive = autoStepper.active;
    autoStepperStart();
    if (allowPrimaryAsciiStatusMessages()) {
      if (!wasActive && autoStepper.active) printConfigAcceptedEcho(cmd);
      else printConfigInvalidEcho(cmd);
    }
    return;
  }
  if (strcmp(cmd, "AUTOSTOP") == 0) {
    const bool hadSession = autoStepper.active || autoStepper.candidateCount > 0 || autoStepper.best.valid;
    autoStepperStop(true);
    if (allowPrimaryAsciiStatusMessages()) {
      if (hadSession) printConfigAcceptedEcho(cmd);
      else printConfigInvalidEcho(cmd);
    }
    return;
  }
  if (strcmp(cmd, "AUTOSTATUS") == 0) {
    printAutoStepperStatus();
    return;
  }
  if (strcmp(cmd, "STEPPERCALSTATUS") == 0) {
    printStepperCalibrationStatus();
    return;
  }

  if (strncmp(cmd, "STEPMODE=", 9) == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    StepperDriveMode newMode;
    if (!parseStepperDriveModeValue(cmd + 9, &newMode)) {
      printConfigInvalidEcho(cmd);
      return;
    }
    setStepperDriveMode(newMode, false);
    noteConfigSessionEdited(false);
    printConfigAcceptedEcho(cmd);
    return;
  }

  if (strncmp(cmd, "ROBOT=", 6) == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    RobotControlMode newMode;
    if (!parseRobotControlModeValue(cmd + 6, &newMode)) {
      printConfigInvalidEcho(cmd);
      return;
    }
    applyRobotControlMode(newMode, false);
    noteConfigSessionEdited(false);
    printConfigAcceptedEcho(cmd);
    return;
  }

  if (strcmp(cmd, "SHOW") == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    printCurrentConfig();
    return;
  }

  if (strcmp(cmd, "SAVE") == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    saveConfigToEEPROM();
    clearConfigSessionDirty();
    Serial.println(F("SAVED TO EEPROM"));
    return;
  }

  if (strcmp(cmd, "LOAD") == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    loadConfigFromEEPROM();
    clearConfigSessionDirty();
    ensureSafeStartupPoseIfColliding();
    rebindI2CDevices();
    Serial.println(F("LOADED FROM EEPROM"));
    return;
  }

  if (strcmp(cmd, "DEFAULT") == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    PersistentConfig cfg;
    PersistentConfigExtras extras;
    fillDefaultConfig(cfg);
    applyConfigToRuntime(cfg);
    fillDefaultConfigExtras(extras);
    applyConfigExtrasToRuntime(extras);
    resetCompassCalibrationData();
    configSessionDirty = true;
    ensureSafeStartupPoseIfColliding();
    rebindI2CDevices();
    Serial.println(F("DEFAULTS APPLIED (NOT SAVED)"));
    if (allowPrimaryAsciiStatusMessages()) printConfigAcceptedEcho(cmd);
    return;
  }

  if (strcmp(cmd, "ERASEEEPROM") == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    eraseEEPROMRegion();
    Serial.println(F("EEPROM ERASED"));
    if (allowPrimaryAsciiStatusMessages()) printConfigAcceptedEcho(cmd);
    return;
  }

  if (strcmp(cmd, "EXIT") == 0 || strcmp(cmd, "CFGOFF") == 0) {
    leaveInteractiveControlModes(false, F("CONFIG/RUNTIME DISABLED"));
    return;
  }

  if (strncmp(cmd, "COLL=", 5) == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    bool parsedCollisionEnable = false;
    if (!parseStrictBool01Value(cmd + 5, parsedCollisionEnable)) {
      printConfigInvalidEcho(cmd);
      return;
    }
    setCollisionRuntimeEnabledSafely(parsedCollisionEnable, false);
    noteConfigSessionEdited(false);
    printConfigAcceptedEcho(cmd);
    return;
  }

  if (strncmp(cmd, "MAGCALCFG=", 10) == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    CompassCalibrationData newCal;
    if (!parseCompassCalibrationConfigValue(cmd + 10, newCal)) {
      printConfigInvalidEcho(cmd);
      return;
    }
    compassCal = newCal;
    noteConfigSessionEdited(false);
    printConfigAcceptedEcho(cmd);
    return;
  }

  if (strncmp(cmd, "CH0MODE=", 8) == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    Ch0DriveMode newMode;
    if (!parseCh0DriveModeValue(cmd + 8, &newMode)) {
      printConfigInvalidEcho(cmd);
      return;
    }
    setCh0DriveMode(newMode, false);
    noteConfigSessionEdited(false);
    printConfigAcceptedEcho(cmd);
    return;
  }

  if (strncmp(cmd, "OPT", 3) == 0) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("AUTO TORQUE/OPT REMOVED; USE PWM CH12..15 MANUALLY"));
    printConfigInvalidEcho(cmd);
    return;
  }

  char* eq = strchr(cmd, '=');
  if (!eq) {
    printConfigInvalidEcho(cmd);
    return;
  }

  char rawCmd[160];
  strncpy(rawCmd, cmd, sizeof(rawCmd) - 1);
  rawCmd[sizeof(rawCmd) - 1] = '\0';

  *eq = '\0';
  char* key = cmd;
  char* valueStr = eq + 1;


  if (strcmp(key, "ARMOFFBASE") == 0 || strcmp(key, "ARMOFFUPPER") == 0 || strcmp(key, "ARMOFFFORE") == 0 ||
      strcmp(key, "ARMOFFFORER") == 0 || strcmp(key, "ARMOFFWRISTP") == 0 || strcmp(key, "ARMOFFWRISTR") == 0 ||
      strcmp(key, "ARMOFFGRIP") == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }

    float servoZeroDeg = 0.0f;
    float memberZeroDeg = 0.0f;
    float servoSign = 1.0f;
    if (!parseManipulatorJointMapCommandValue(valueStr, servoZeroDeg, memberZeroDeg, servoSign)) {
      printConfigInvalidEcho(rawCmd);
      return;
    }

    if (strcmp(key, "ARMOFFBASE") == 0) {
      armBaseServoZeroDeg = 0.0f; armBaseMemberZeroDeg = 0.0f; armBaseServoSign = 1.0f;
    } else if (strcmp(key, "ARMOFFUPPER") == 0) {
      armUpperServoZeroDeg = servoZeroDeg; armUpperMemberZeroDeg = memberZeroDeg; armUpperServoSign = servoSign;
    } else if (strcmp(key, "ARMOFFFORE") == 0) {
      armForeServoZeroDeg = servoZeroDeg; armForeMemberZeroDeg = memberZeroDeg; armForeServoSign = servoSign;
    } else if (strcmp(key, "ARMOFFFORER") == 0) {
      armForearmRollServoZeroDeg = servoZeroDeg; armForearmRollMemberZeroDeg = memberZeroDeg; armForearmRollServoSign = servoSign;
    } else if (strcmp(key, "ARMOFFWRISTP") == 0) {
      armWristPitchServoZeroDeg = servoZeroDeg; armWristPitchMemberZeroDeg = memberZeroDeg; armWristPitchServoSign = servoSign;
    } else if (strcmp(key, "ARMOFFWRISTR") == 0) {
      armWristRollServoZeroDeg = servoZeroDeg; armWristRollMemberZeroDeg = memberZeroDeg; armWristRollServoSign = servoSign;
    } else {
      armGripServoZeroDeg = servoZeroDeg; armGripMemberZeroDeg = memberZeroDeg; armGripServoSign = servoSign;
    }

    noteConfigSessionEdited(false);
    printConfigAcceptedEcho(rawCmd);
    return;
  }

  if (strcmp(key, "PCAADDR") == 0 ||
      strcmp(key, "INAADDR") == 0 ||
      strcmp(key, "INA2ADDR") == 0 ||
      strcmp(key, "MPUADDR") == 0 ||
      strcmp(key, "MPU2ADDR") == 0 ||
      strcmp(key, "MAGADDR") == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }

    uint8_t parsedAddr = 0;
    if (!parseStrictI2CAddressHexValue(valueStr, parsedAddr)) {
      printConfigInvalidEcho(rawCmd);
      return;
    }

    if ((strcmp(key, "PCAADDR") == 0 || strcmp(key, "INAADDR") == 0 || strcmp(key, "INA2ADDR") == 0 || strcmp(key, "MPUADDR") == 0) && parsedAddr == 0) {
      printConfigInvalidEcho(rawCmd);
      return;
    }

    if      (strcmp(key, "PCAADDR") == 0)  pca9685Addr = parsedAddr;
    else if (strcmp(key, "INAADDR") == 0)  ina219Addr1 = parsedAddr;
    else if (strcmp(key, "INA2ADDR") == 0) ina219Addr2 = parsedAddr;
    else if (strcmp(key, "MPUADDR") == 0)  mpu6050Addr1 = parsedAddr;
    else if (strcmp(key, "MPU2ADDR") == 0) mpu6050Addr2 = parsedAddr;
    else                                        compassAddr = parsedAddr;

    sanitizeStepperRuntimeParams();
    markProcessingProfilesDirty();
    syncProcessingRobotProfiles();
    releaseInactiveManipulatorChannels();
    noteConfigSessionEdited(false);
    printConfigAcceptedEcho(rawCmd);
    return;
  }

  float value = 0.0f;
  if (!parseStrictFloatValue(valueStr, value)) {
    printConfigInvalidEcho(rawCmd);
    return;
  }

  if (strcmp(key, "RAMPSPD") == 0) {
    servoMoveStepDeg = constrain(value, 0.01f, 10.0f);
    noteConfigSessionEdited(false);
    printConfigAcceptedEcho(rawCmd);
    return;
  }
  if (strcmp(key, "RAMPSMOOTH") == 0) {
    servoRampSmoothing = clampValue(value, 0.05f, 1.0f);
    noteConfigSessionEdited(false);
    printConfigAcceptedEcho(rawCmd);
    return;
  }

  if (!rt.configMode) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
    return;
  }

  bool manipGeometryChanged = false;
  bool genericGeometryChanged = false;

  if      (strcmp(key, "PCAADDR") == 0)   pca9685Addr = (uint8_t)clampValue((int)value, 1, 127);
  else if (strcmp(key, "INAADDR") == 0)   ina219Addr1 = (uint8_t)clampValue((int)value, 1, 127);
  else if (strcmp(key, "INA2ADDR") == 0)  ina219Addr2 = (uint8_t)clampValue((int)value, 1, 127);
  else if (strcmp(key, "MPUADDR") == 0)   mpu6050Addr1 = (uint8_t)clampValue((int)value, 1, 127);
  else if (strcmp(key, "MPU2ADDR") == 0)  mpu6050Addr2 = (uint8_t)clampValue((int)value, 0, 127);
  else if (strcmp(key, "MAGADDR") == 0)   compassAddr = (uint8_t)clampValue((int)value, 0, 127);
  else if (strcmp(key, "ZEROADC") == 0)   stepperCalibrationZeroADC = (int16_t)value;
  else if (strcmp(key, "SPANRAW") == 0)   stepperOneTurnSpanRaw = (int32_t)value;
  else if (strcmp(key, "MINSTEPUS") == 0) minStepDelayUs = (uint16_t)clampValue((long)value, 500L, 60000L);
  else if (strcmp(key, "MAXSTEPUS") == 0) maxStepDelayUs = (uint16_t)clampValue((long)value, 500L, 60000L);
  else if (strcmp(key, "STOPENTER") == 0 || strcmp(key, "STOPMOTORENTER") == 0) stepperStopEnter = clampValue(value, 0.5f, 45.0f);
  else if (strcmp(key, "STOPEXIT") == 0 || strcmp(key, "STOPMOTOREXIT") == 0)  stepperStopExit  = clampValue(value, 0.5f, 60.0f);
  else if (strcmp(key, "PWMFAR") == 0)    movePwmFar = (uint16_t)clampValue((int)value, 0, (int)ICR5);
  else if (strcmp(key, "PWMNEAR") == 0)   movePwmNear = (uint16_t)clampValue((int)value, 0, (int)ICR5);
  else if (strcmp(key, "PWMHOLD") == 0)   holdPwm = (uint16_t)clampValue((int)value, 0, (int)ICR5);
  else if (strcmp(key, "ALPHAANG") == 0)  angleFilterAlpha = clampValue(value, 0.01f, 1.0f);
  else if (strcmp(key, "RAMPSPD") == 0)   servoMoveStepDeg = clampValue(value, 0.01f, 10.0f);
  else if (strcmp(key, "RAMPSMOOTH") == 0) servoRampSmoothing = clampValue(value, 0.05f, 1.0f);
  else if (strcmp(key, "PWMBOOT12") == 0 || strcmp(key, "PWMBOOT13") == 0 ||
           strcmp(key, "PWMBOOT14") == 0 || strcmp(key, "PWMBOOT15") == 0) {
    if ((int)roundf(value) != DEFAULT_MANUAL_PWM_PERCENT) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    setAllPwmBootPercentToNeutral();
  }
  else if (strcmp(key, "BASECH") == 0) {
    if (!setManipulatorBaseChannelFlexible((uint8_t)clampValue((int)value, 0, 1))) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
  }
  else if (strcmp(key, "UPPERCH") == 0 || strcmp(key, "UPPERCHANNEL") == 0) {
    if (!setManipulatorMemberChannel(MANIP_MEMBER_UPPER, (uint8_t)clampValue((int)value, (int)kDefaultManipProfile.upperChannel, (int)LAST_SERVO_CH))) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
  }
  else if (strcmp(key, "FORECH") == 0 || strcmp(key, "FORECHANNEL") == 0) {
    if (!setManipulatorMemberChannel(MANIP_MEMBER_FORE, (uint8_t)clampValue((int)value, (int)kDefaultManipProfile.upperChannel, (int)LAST_SERVO_CH))) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
  }
  else if (strcmp(key, "FOREROLLCH") == 0 || strcmp(key, "FOREARMROLLCH") == 0) {
    if (!setManipulatorMemberChannel(MANIP_MEMBER_FOREARM_ROLL, (uint8_t)clampValue((int)value, (int)kDefaultManipProfile.upperChannel, (int)LAST_SERVO_CH))) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
  }
  else if (strcmp(key, "WRISTPITCHCH") == 0 || strcmp(key, "WRISTVERTCH") == 0) {
    if (!setManipulatorMemberChannel(MANIP_MEMBER_WRIST_PITCH, (uint8_t)clampValue((int)value, (int)kDefaultManipProfile.upperChannel, (int)LAST_SERVO_CH))) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
  }
  else if (strcmp(key, "WRISTROTCH") == 0 || strcmp(key, "WRISTROLLCH") == 0) {
    if (!setManipulatorMemberChannel(MANIP_MEMBER_WRIST_ROLL, (uint8_t)clampValue((int)value, (int)kDefaultManipProfile.upperChannel, (int)LAST_SERVO_CH))) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
  }
  else if (strcmp(key, "GRIPCH") == 0 || strcmp(key, "GRIPPERCH") == 0) {
    if (!setManipulatorMemberChannel(MANIP_MEMBER_GRIPPER, (uint8_t)clampValue((int)value, (int)kDefaultManipProfile.upperChannel, (int)LAST_SERVO_CH))) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
  }
  else if (strcmp(key, "ARMBASER") == 0) {
    armBaseRadius = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMBASEH") == 0) {
    armBaseHeight = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMBBLW") == 0) {
    armBaseBlockW = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMBBLH") == 0) {
    armBaseBlockH = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMBBLD") == 0) {
    armBaseBlockD = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMUPW") == 0) {
    armUpperW = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMUPH") == 0) {
    armUpperH = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMUPD") == 0) {
    armUpperD = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMFRW") == 0) {
    armForeW = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMFRH") == 0) {
    armForeH = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMFRD") == 0) {
    armForeD = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMWRW") == 0) {
    armWristW = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMWRH") == 0) {
    armWristH = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMWRD") == 0) {
    armWristD = clampValue(value, 1.0f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMFGW") == 0) {
    armFingerW = clampValue(value, 0.1f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMFGH") == 0) {
    armFingerH = clampValue(value, 0.1f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMFGD") == 0) {
    armFingerD = clampValue(value, 0.1f, 500.0f);
    manipGeometryChanged = true;
  }
  else if (strcmp(key, "ARMBCY") == 0) armBaseCylinderYOffset = clampValue(value, -1000.0f, 1000.0f);
  else if (strcmp(key, "ARMBBY") == 0) armBaseBlockYOffset = clampValue(value, -1000.0f, 1000.0f);
  else if (strcmp(key, "ARMGRY") == 0) armGripperYOffset = clampValue(value, -1000.0f, 1000.0f);
  else if (strcmp(key, "VEHBODYL") == 0) {
    vehicleProfile.bodyLength = clampValue(value, 1.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHBODYW") == 0) {
    vehicleProfile.bodyWidth = clampValue(value, 1.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHBODYH") == 0) {
    vehicleProfile.bodyHeight = clampValue(value, 1.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHTGAUGE") == 0) {
    vehicleProfile.trackGauge = clampValue(value, 1.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHTRUN") == 0) {
    vehicleProfile.trackRun = clampValue(value, 1.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHDRIVE") == 0) {
    vehicleProfile.driveRadius = clampValue(value, 1.0f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHSUPR") == 0) {
    vehicleProfile.supportRadius = clampValue(value, 1.0f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHTOPR") == 0) {
    vehicleProfile.topRollerRadius = clampValue(value, 1.0f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHTWIDTH") == 0) {
    vehicleProfile.trackWidth = clampValue(value, 1.0f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHTLINKL") == 0) {
    vehicleProfile.trackLinkLength = clampValue(value, 1.0f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHTLINKT") == 0) {
    vehicleProfile.trackLinkThickness = clampValue(value, 0.1f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHTHEIGHT") == 0) {
    vehicleProfile.trackHeight = clampValue(value, 0.1f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHTLINKS") == 0) {
    vehicleProfile.trackLinks = max(16, (int)roundf(clampValue(value, 16.0f, 512.0f)));
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHTSAG") == 0) {
    vehicleProfile.trackSag = clampValue(value, 0.0f, 200.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "VEHCAMPAN0") == 0) {
    vehicleProfile.cameraPanDefaultDeg = clampValue(value, -180.0f, 180.0f);
  }
  else if (strcmp(key, "VEHCAMTILT0") == 0) {
    vehicleProfile.cameraTiltDefaultDeg = clampValue(value, -90.0f, 90.0f);
  }
  else if (strcmp(key, "VEHLIDARY") == 0) {
    vehicleProfile.lidarMastYOffset = clampValue(value, -1000.0f, 1000.0f);
  }
  else if (strcmp(key, "VEHLIDARZ") == 0) {
    vehicleProfile.lidarMastZOffset = clampValue(value, -1000.0f, 1000.0f);
  }
  else if (strcmp(key, "VEHCAMY") == 0) {
    vehicleProfile.cameraHeadYOffset = clampValue(value, -1000.0f, 1000.0f);
  }
  else if (strcmp(key, "VEHCAMZ") == 0) {
    vehicleProfile.cameraHeadZOffset = clampValue(value, -1000.0f, 1000.0f);
  }
  else if (strcmp(key, "VEHDEAD") == 0) {
    vehicleControlProfile.motorDeadbandPct = clampValue(value, 0.0f, 40.0f);
  }
  else if (strcmp(key, "VEHPWMMIN") == 0) {
    vehicleControlProfile.motorPwmMin = (uint8_t)clampValue((int)value, 0, 255);
  }
  else if (strcmp(key, "VEHPWMMAX") == 0) {
    vehicleControlProfile.motorPwmMax = (uint8_t)clampValue((int)value, (int)vehicleControlProfile.motorPwmMin, 255);
  }
  else if (strcmp(key, "VEHSLEW") == 0) {
    vehicleControlProfile.motorSlewPctPerUpdate = clampValue(value, 0.5f, 100.0f);
  }
  else if (strcmp(key, "VEHINVLEFT") == 0) {
    vehicleControlProfile.leftMotorInvert = (value >= 0.5f);
  }
  else if (strcmp(key, "VEHINVRIGHT") == 0) {
    vehicleControlProfile.rightMotorInvert = (value >= 0.5f);
  }
  else if (strcmp(key, "VEHCAMPANMIN") == 0) {
    vehicleControlProfile.cameraPanMinDeg = clampValue(value, -180.0f, 180.0f);
  }
  else if (strcmp(key, "VEHCAMPANMAX") == 0) {
    vehicleControlProfile.cameraPanMaxDeg = clampValue(value, vehicleControlProfile.cameraPanMinDeg, 180.0f);
  }
  else if (strcmp(key, "VEHCAMTILTMIN") == 0) {
    vehicleControlProfile.cameraTiltMinDeg = clampValue(value, -180.0f, 180.0f);
  }
  else if (strcmp(key, "VEHCAMTILTMAX") == 0) {
    vehicleControlProfile.cameraTiltMaxDeg = clampValue(value, vehicleControlProfile.cameraTiltMinDeg, 180.0f);
  }
  else if (strcmp(key, "VEHLIDARMIN") == 0) {
    vehicleControlProfile.lidarSweepMinDeg = (uint8_t)clampValue((int)value, 0, 180);
  }
  else if (strcmp(key, "VEHLIDARMAX") == 0) {
    vehicleControlProfile.lidarSweepMaxDeg = (uint8_t)clampValue((int)value, (int)vehicleControlProfile.lidarSweepMinDeg, 180);
  }
  else if (strcmp(key, "VEHLIDARMS") == 0) {
    vehicleControlProfile.lidarSweepCycleMs = (uint16_t)clampValue((long)value, 250L, 10000L);
  }
  else if (strcmp(key, "VEHRTH") == 0) {
    if (robotControlMode != ROBOT_MODE_VEHICLE) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    bool parsedEnable = false;
    if (!parseStrictBool01Value(valueStr, parsedEnable)) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    vehicleSafeReturnEnabled = parsedEnable;
  }
  else if (strcmp(key, "VEHINCL") == 0) {
    if (robotControlMode != ROBOT_MODE_VEHICLE) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    bool parsedEnable = false;
    if (!parseStrictBool01Value(valueStr, parsedEnable)) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    vehicleInclinePowerEnabled = parsedEnable;
  }
  else if (strcmp(key, "VEHINCLBASE") == 0) {
    if (robotControlMode != ROBOT_MODE_VEHICLE) {
      printConfigInvalidEcho(rawCmd);
      return;
    } vehicleInclineBasePowerPct = clampValue(value, 0.0f, 100.0f);
  }
  else if (strcmp(key, "VEHINCLMAX") == 0) {
    if (robotControlMode != ROBOT_MODE_VEHICLE) {
      printConfigInvalidEcho(rawCmd);
      return;
    } vehicleInclineMaxPowerPct = clampValue(value, vehicleInclineBasePowerPct, 100.0f);
  }
  else if (strcmp(key, "VEHINCLFULL") == 0) {
    if (robotControlMode != ROBOT_MODE_VEHICLE) {
      printConfigInvalidEcho(rawCmd);
      return;
    } vehicleInclineFullScaleDeg = clampValue(value, vehicleInclineDeadbandDeg + 1.0f, 80.0f);
  }
  else if (strcmp(key, "VEHINCLDB") == 0) {
    if (robotControlMode != ROBOT_MODE_VEHICLE) {
      printConfigInvalidEcho(rawCmd);
      return;
    } vehicleInclineDeadbandDeg = clampValue(value, 0.0f, 45.0f);
  }
  else if (strcmp(key, "VEHINCLALPHA") == 0) {
    if (robotControlMode != ROBOT_MODE_VEHICLE) {
      printConfigInvalidEcho(rawCmd);
      return;
    } vehicleInclinePowerAlpha = clampValue(value, 0.01f, 1.0f);
  }
  else if (strcmp(key, "VEHICLELINKHOLD") == 0) {
    if (robotControlMode != ROBOT_MODE_VEHICLE) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    bool parsedEnable = false;
    if (!parseStrictBool01Value(valueStr, parsedEnable)) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    vehicleHoldLastMotionOnLinkLoss = parsedEnable;
  }
  else if (strcmp(key, "SAFEBAT") == 0) {
    float parsedThreshold = clampValue(value, 1.0f, 80.0f);
    if (robotControlMode == ROBOT_MODE_DRONE) droneSafeReturnBatteryThresholdPct = parsedThreshold;
    else if (robotControlMode == ROBOT_MODE_VEHICLE) vehicleSafeReturnBatteryThresholdPct = parsedThreshold;
    else {
      printConfigInvalidEcho(rawCmd);
      return;
    }
  }
  else if (strcmp(key, "DRNBODYL") == 0) {
    droneProfile.bodyLength = clampValue(value, 1.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNBODYW") == 0) {
    droneProfile.bodyWidth = clampValue(value, 1.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNBODYH") == 0) {
    droneProfile.bodyHeight = clampValue(value, 1.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNARML") == 0) {
    droneProfile.armLength = clampValue(value, 1.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNARMT") == 0) {
    droneProfile.armThickness = clampValue(value, 0.1f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNMOTR") == 0) {
    droneProfile.motorRadius = clampValue(value, 0.1f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNMOTH") == 0) {
    droneProfile.motorHeight = clampValue(value, 0.1f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNPROPR") == 0) {
    droneProfile.propRadius = clampValue(value, 0.1f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNPROPT") == 0) {
    droneProfile.propThickness = clampValue(value, 0.1f, 100.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNLEGH") == 0) {
    droneProfile.legHeight = clampValue(value, 0.1f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNLEGS") == 0) {
    droneProfile.legSpan = clampValue(value, 0.1f, 500.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNRESTY") == 0) {
    droneProfile.restYOffset = clampValue(value, -1000.0f, 1000.0f);
  }
  else if (strcmp(key, "DRNVISYAW") == 0) {
    droneProfile.visualYawOffsetDeg = clampValue(value, -180.0f, 180.0f);
  }
  else if (strcmp(key, "DRNCAMY") == 0) {
    droneProfile.cameraYOffset = clampValue(value, -1000.0f, 1000.0f);
  }
  else if (strcmp(key, "DRNCAMZ") == 0) {
    droneProfile.cameraZOffset = clampValue(value, -1000.0f, 1000.0f);
  }
  else if (strcmp(key, "DRNLAMPY") == 0) {
    droneProfile.lampYOffset = clampValue(value, -1000.0f, 1000.0f);
  }
  else if (strcmp(key, "DRNLAMPZ") == 0) {
    droneProfile.lampZOffset = clampValue(value, -1000.0f, 1000.0f);
  }
  else if (strcmp(key, "DRNSONARX") == 0) {
    droneProfile.sonarXOffset = clampValue(value, -1000.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNSONARY") == 0) {
    droneProfile.sonarYOffset = clampValue(value, -1000.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNSONARZ") == 0) {
    droneProfile.sonarZOffset = clampValue(value, -1000.0f, 1000.0f);
    genericGeometryChanged = true;
  }
  else if (strcmp(key, "DRNESCMIN") == 0) {
    droneControlProfile.escMinUs = (uint16_t)clampValue((long)value, 700L, 2000L);
  }
  else if (strcmp(key, "DRNESCMAX") == 0) {
    droneControlProfile.escMaxUs = (uint16_t)clampValue((long)value, (long)droneControlProfile.escMinUs + 50L, 2500L);
  }
  else if (strcmp(key, "DRNESCIDLE") == 0) {
    droneControlProfile.escIdleUs = (uint16_t)clampValue((long)value, (long)droneControlProfile.escMinUs, (long)droneControlProfile.escMaxUs);
  }
  else if (strcmp(key, "DRNTHRSPAN") == 0) {
    uint16_t maxRange = (droneControlProfile.escMaxUs > droneControlProfile.escIdleUs) ? (uint16_t)(droneControlProfile.escMaxUs - droneControlProfile.escIdleUs) : 0;
    droneControlProfile.throttleRangeUs = (uint16_t)clampValue((long)value, 0L, (long)maxRange);
  }
  else if (strcmp(key, "DRNPITCHMIX") == 0) {
    droneControlProfile.pitchMixUs = (uint16_t)clampValue((long)value, 0L, 600L);
  }
  else if (strcmp(key, "DRNROLLMIX") == 0) {
    droneControlProfile.rollMixUs = (uint16_t)clampValue((long)value, 0L, 600L);
  }
  else if (strcmp(key, "DRNYAWMIX") == 0) {
    droneControlProfile.yawMixUs = (uint16_t)clampValue((long)value, 0L, 600L);
  }
  else if (strcmp(key, "DRNDEAD") == 0) {
    droneControlProfile.commandDeadbandPct = clampValue(value, 0.0f, 20.0f);
  }
  else if (strcmp(key, "DRNSPOOL") == 0) {
    droneControlProfile.spoolStepUsPerUpdate = (uint16_t)clampValue((long)value, 1L, 400L);
  }
  else if (strcmp(key, "DRNRTH") == 0) {
    if (robotControlMode != ROBOT_MODE_DRONE) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    bool parsedEnable = false;
    if (!parseStrictBool01Value(valueStr, parsedEnable)) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    droneSafeReturnEnabled = parsedEnable;
  }
  else if (strcmp(key, "DRONELINKHOLD") == 0) {
    if (robotControlMode != ROBOT_MODE_DRONE) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    bool parsedEnable = false;
    if (!parseStrictBool01Value(valueStr, parsedEnable)) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    droneHoldLastMotionOnLinkLoss = parsedEnable;
  }
  else if (strcmp(key, "ASTAB") == 0) {
    bool parsedEnable = false;
    if (!parseStrictBool01Value(valueStr, parsedEnable)) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    armStabilizeEnable = parsedEnable;
  }
  else if (strcmp(key, "ASTABGAIN") == 0) armStabilizeGain = clampValue(value, 0.0f, 2.0f);
  else if (strcmp(key, "ASTABTH") == 0) armStabilizeThreshold = clampValue(value, 0.0f, 45.0f);
  else if (strcmp(key, "GPRESS") == 0) {
    bool parsedEnable = false;
    if (!parseStrictBool01Value(valueStr, parsedEnable)) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    gripPressureEnable = parsedEnable;
  }
  else if (strcmp(key, "GPGAIN") == 0) gripPressureGain = clampValue(value, 0.0f, 10.0f);
  else {
    printConfigInvalidEcho(rawCmd);
    return;
  }

  sanitizeStepperRuntimeParams();
  markProcessingProfilesDirty();
  syncProcessingRobotProfiles();
  if (manipGeometryChanged) {
    recomputeManipulatorOffsetsFromProcessingModel();
  }
  if (genericGeometryChanged) {
    recomputeProcessingRobotDerivedOffsets();
  }
  releaseInactiveManipulatorChannels();
  noteConfigSessionEdited(false);
  printConfigAcceptedEcho(rawCmd);
}

// Utility: enter operation mode silent.
void enterOperationModeSilent() {
  rt.servoOutputsArmed = false;
  rt.stepperArmed = false;

  for (uint8_t ch = FIRST_SERVO_CH; ch <= LAST_SERVO_CH; ch++) {
    servos.actual[ch] = servos.target[ch];
  }

  readStepperAngle();
  stepper.targetAngle = stepper.currentAngle;
  servos.target[0] = stepper.currentAngle;
  servos.actual[0] = stepper.currentAngle;
  resetStepperPID();

  releaseStepperCoils();
  stepperSetDriverPWM(0);

  rt.lastSensorMs = millis();
  rt.lastServoUpdateMs = millis();
  rt.active = true;
  serialHexMode = true;  // switch Serial0 to HEX-framed protocol
}

// Utility: enter operation mode.
void enterOperationMode() {
  // Enter runtime without causing any physical movement
  rt.servoOutputsArmed = false;
  rt.stepperArmed = false;

  for (uint8_t ch = FIRST_SERVO_CH; ch <= LAST_SERVO_CH; ch++) {
    servos.actual[ch] = servos.target[ch];
  }

  readStepperAngle();
  stepper.targetAngle = stepper.currentAngle;
  servos.target[0] = stepper.currentAngle;
  servos.actual[0] = stepper.currentAngle;
  resetStepperPID();

  releaseStepperCoils();
  stepperSetDriverPWM(0);

  rt.lastSensorMs = millis();
  rt.lastServoUpdateMs = millis();
  rt.active = true;
  serialHexMode = true;  // switch Serial0 to HEX-framed protocol
}

// Handles command.
void handleCommand(char* cmd) {
  if (cmd == NULL) return;

  canonicalizeCommandText(cmd);
  if (cmd[0] == '\0') return;

  if (!rt.active && !rt.configMode) {
    if (strcmp(cmd, "READY?") != 0 &&
        strcmp(cmd, "BOOT?") != 0 &&
        strcmp(cmd, "BOOTRESET") != 0 &&
        strncmp(cmd, "CFG=", 4) != 0) {
      return;
    }
  }

  if (currentTextCommandSource == 1 && controlOwner == CONTROL_OWNER_RUNTIME_HEX) {
    return;
  }

  const bool streamedCommand = unwrapStreamCommand(cmd);
  canonicalizeCommandText(cmd);
  if (cmd[0] == '\0') return;

  const bool exitCommand = (strcmp(cmd, "EXIT") == 0 || strcmp(cmd, "CFGOFF") == 0);

  if (runtimeAsciiTelemetryOnly()) {
    // Strict ASCII runtime contract:
    //   RX -> #CMD|ROBOT:<name>|SEQ:<n>|TS:<ms>|CMD:<BODY>
    //   TX -> #TEL|...
    // Raw runtime commands on the primary ASCII stream are ignored.
    if (!streamedCommand) {
      return;
    }

    if (exitCommand) {
      leaveInteractiveControlModes(false, F("CONFIG/RUNTIME DISABLED"));
      return;
    }

    if (strncmp(cmd, "ROBOT=", 6) == 0) {
      RobotControlMode newMode;
      if (parseRobotControlModeValue(cmd + 6, &newMode)) {
        applyRobotControlMode(newMode, false);
        sendTelemetry();
      }
      return;
    }
    if (shouldRouteToConfigCommand(cmd)) {
      handleConfigCommand(cmd);
      return;
    }
    if (robotControlMode == ROBOT_MODE_MANIPULATOR) {
      if (strncmp(cmd, "ARMJ=", 5) == 0) {
        handleManipulatorRuntimeCommand(cmd);
        return;
      }
      if (strcmp(cmd, "HOME") == 0) {
        applyManipulatorRuntimeHomePose();
        return;
      }
      if (strncmp(cmd, "GSTAB=", 6) == 0 || strncmp(cmd, "STAB=", 5) == 0) {
        const char* valueText = (cmd[0] == 'G') ? (cmd + 6) : (cmd + 5);
        bool parsedEnable = false;
        if (!parseStrictBool01Value(valueText, parsedEnable)) {
          return;
        }
        wristGimbalEnabled = parsedEnable;
        wristGimbalRefValid = false;
        return;
      }
      return;
    }
    if (strncmp(cmd, "MAGCAL=", 7) == 0) {
      bool parsedEnable = false;
      if (!parseStrictBool01Value(cmd + 7, parsedEnable)) {
        return;
      }
      if (parsedEnable) startCompassCalibration();
      else cancelCompassCalibration(false);
      return;
    }
    if (robotControlMode == ROBOT_MODE_VEHICLE && handleVehicleRuntimeCommand(cmd)) {
      return;
    }
    if (robotControlMode == ROBOT_MODE_DRONE && handleDroneRuntimeCommand(cmd)) {
      return;
    }
    return;
  }

  if (strcmp(cmd, "BOOT?") == 0 || strcmp(cmd, "BOOTRESET") == 0) {
    restartBootHandshakeScreen();
    return;
  }

  if (strcmp(cmd, "READY?") == 0) {
    controlOwner = CONTROL_OWNER_PRIMARY_ASCII;
    if (!rt.active) enterOperationMode();  // sets serialHexMode = true
    // Automatic runtime startup must stay lightweight: send a minimal identity
    // snapshot and telemetry only. The full config snapshot is String-heavy on
    // AVR and can stall or reset Serial0 immediately after the first telemetry.
    sendMinimalHexRuntimeSnapshot();
    sendTelemetryHex();
    sendGpsTelemetryHex(true);
    return;
  }

  if (handleStepperDiagnosticCommand(cmd)) {
    return;
  }

  if (shouldRouteToConfigCommand(cmd)) {
    handleConfigCommand(cmd);
    return;
  }

  if (strncmp(cmd, "OFFSET", 6) == 0) {
    handleNumericCommand("OFFSET", cmd, APPLY_OFFSET);
    return;
  }

  if (strncmp(cmd, "MAGCAL=", 7) == 0) {
    bool parsedEnable = false;
    if (!parseStrictBool01Value(cmd + 7, parsedEnable)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID MAGCAL COMMAND"));
      return;
    }
    if (parsedEnable) startCompassCalibration();
    else cancelCompassCalibration(false);
    return;
  }

  if (robotControlMode == ROBOT_MODE_MANIPULATOR &&
      (strncmp(cmd, "GSTAB=", 6) == 0 || strncmp(cmd, "STAB=", 5) == 0)) {
    const char* valueText = (cmd[0] == 'G') ? (cmd + 6) : (cmd + 5);
    bool parsedEnable = false;
    if (!parseStrictBool01Value(valueText, parsedEnable)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID GSTAB COMMAND"));
      return;
    }
    wristGimbalEnabled = parsedEnable;
    wristGimbalRefValid = false;
    return;
  }

  if (robotControlMode == ROBOT_MODE_MANIPULATOR && handleManipulatorRuntimeCommand(cmd)) {
    return;
  }

  if (robotControlMode == ROBOT_MODE_VEHICLE && handleVehicleRuntimeCommand(cmd)) {
    return;
  }

  if (robotControlMode == ROBOT_MODE_DRONE && handleDroneRuntimeCommand(cmd)) {
    return;
  }

  if (strncmp(cmd, "OFF", 3) == 0) {
    handleNumericCommand("OFF", cmd, APPLY_OFFSET);
    return;
  }

  if (strncmp(cmd, "SPAN", 4) == 0) {
    handleNumericCommand("SPAN", cmd, APPLY_SPAN);
    return;
  }

  if (strncmp(cmd, "RAMP", 4) == 0) {
    handleNumericCommand("RAMP", cmd, APPLY_RAMP);
    return;
  }

  if (strncmp(cmd, "SERVOMIN", 8) == 0) {
    handleNumericCommand("SERVOMIN", cmd, APPLY_MIN_LIMIT);
    return;
  }

  if (strncmp(cmd, "SERVOMAX", 8) == 0) {
    handleNumericCommand("SERVOMAX", cmd, APPLY_MAX_LIMIT);
    return;
  }

  if (strncmp(cmd, "MIN", 3) == 0) {
    handleNumericCommand("MIN", cmd, APPLY_MIN_LIMIT);
    return;
  }

  if (strncmp(cmd, "MAX", 3) == 0) {
    handleNumericCommand("MAX", cmd, APPLY_MAX_LIMIT);
    return;
  }

  if (strncmp(cmd, "DIR", 3) == 0) {
    handleDirectionCommand(cmd);
    return;
  }

  if (allowPrimaryAsciiStatusMessages()) {
    Serial.print(F("UNKNOWN COMMAND: "));
    Serial.println(cmd);
  }
}
