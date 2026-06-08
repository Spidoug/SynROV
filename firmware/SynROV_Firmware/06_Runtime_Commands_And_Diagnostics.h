// =====================================================================
// SynROV Firmware - Runtime commands and diagnostics
// ---------------------------------------------------------------------
// Purpose:
//   Command parsing, status panels, runtime configuration, calibration
//   flows and operator-facing diagnostics.
// =====================================================================

// Initializes auto motor state.
void initAutoMotorState() {
  memset(&autoMotor, 0, sizeof(autoMotor));
  autoMotor.stage = AUTO_STAGE_IDLE;
  autoMotor.cachedStage = AUTO_STAGE_IDLE;
  autoMotor.best.valid = false;
  autoMotor.best.score = -1000000000.0f;
  autoMotor.modeAttemptIndex = 0;
  autoMotor.originalMode = motorDriveMode;
}

// Initializes motor cal state.
void initMotorCalState() {
  memset(&motorCal, 0, sizeof(motorCal));
  motorCal.stage = MOTOR_CAL_IDLE;
}

// Reads motor raw ADC.
int16_t readMotorRawADC() {
  long sum = 0;
  const uint8_t samples = 16;
  for (uint8_t i = 0; i < samples; i++) sum += analogRead(MOTOR_FEEDBACK_PIN);
  return (int16_t)(sum / samples);
}

// Starts motor calibration wizard.
void startMotorCalibrationWizard() {
  if (!rt.configMode) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
    return;
  }
  autoMotorStop(false);
  motorCal.active = true;
  motorCal.stage = MOTOR_CAL_WAIT_LIMIT1;
  motorCal.limit1Adc = 0;
  motorCal.limit2Adc = 0;
  Serial.println(F("LIMITMOTORCALIB STARTED"));
  Serial.println(F("MOVE THE MOTOR TO THE FIRST MECHANICAL LIMIT AND SEND LIMITMOTORCALIB"));
}

// Utility: capture motor calibration limit 1.
void captureMotorCalibrationLimit1() {
  if (!motorCal.active || motorCal.stage != MOTOR_CAL_WAIT_LIMIT1) {
    Serial.println(F("LIMITMOTORCALIB IS NOT WAITING FOR THE FIRST LIMIT"));
    return;
  }
  motorCal.limit1Adc = readMotorRawADC();
  motorCal.stage = MOTOR_CAL_WAIT_LIMIT2;
  Serial.print(F("LIMITMOTORCALIB LIMIT_A ADC=")); Serial.println(motorCal.limit1Adc);
  Serial.println(F("MOVE THE MOTOR TO THE OPPOSITE MECHANICAL LIMIT AND SEND LIMITMOTORCALIB AGAIN"));
}

// Utility: capture motor calibration limit 2.
void captureMotorCalibrationLimit2() {
  if (!motorCal.active || motorCal.stage != MOTOR_CAL_WAIT_LIMIT2) {
    Serial.println(F("LIMITMOTORCALIB IS NOT WAITING FOR THE SECOND LIMIT"));
    return;
  }
  motorCal.limit2Adc = readMotorRawADC();
  int16_t low = min(motorCal.limit1Adc, motorCal.limit2Adc);
  int16_t high = max(motorCal.limit1Adc, motorCal.limit2Adc);
  int32_t span = (int32_t)high - (int32_t)low;
  if (span < 20) {
    Serial.println(F("LIMITMOTORCALIB FAILED - LIMITS ARE TOO CLOSE"));
    initMotorCalState();
    return;
  }
  motorCalibrationZeroADC = low;
  motorOneTurnSpanRaw = span;
  sanitizeMotorRuntimeParams();
  motor.filteredPot = 0.0f;
  readMotorAngle();
  motor.targetAngle = clampValue(motor.currentAngle, MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);
  servos.target[0] = motor.targetAngle;
  resetMotorControlState();
  motorCal.active = false;
  motorCal.stage = MOTOR_CAL_DONE;
  rt.active = false;
  rt.configMode = true;
  rt.motorArmed = false;
  rt.servoOutputsArmed = false;
  releaseMotorCoils();
  motorSetDriverPWM(0);
  motorPulse.setSpeed(0.0f);
  Serial.println(F("LIMITMOTORCALIB DONE"));
  Serial.print(F("ZEROADC=")); Serial.println(motorCalibrationZeroADC);
  Serial.print(F("SPANRAW=")); Serial.println(motorOneTurnSpanRaw);
  configSessionDirty = true;
  Serial.println(F("VALUES ARE READY IN CFG - USE SAVE TO STORE EEPROM OR EXIT TO LEAVE CFG"));
}

// Handles callimit motor command.
void handleCallimitMotorCommand() {
  if (!motorCal.active || motorCal.stage == MOTOR_CAL_IDLE || motorCal.stage == MOTOR_CAL_DONE) {
    startMotorCalibrationWizard();
    return;
  }
  if (motorCal.stage == MOTOR_CAL_WAIT_LIMIT1) {
    captureMotorCalibrationLimit1();
    return;
  }
  if (motorCal.stage == MOTOR_CAL_WAIT_LIMIT2) {
    captureMotorCalibrationLimit2();
    return;
  }
}

// Prints motor calibration status.
void printMotorCalibrationStatus() {
  Serial.print(F("LIMITMOTORCALIB ACTIVE=")); Serial.println(motorCal.active ? 1 : 0);
  Serial.print(F("LIMITMOTORCALIB STAGE=")); Serial.println((int)motorCal.stage);
  Serial.print(F("LIMITMOTORCALIB LIMIT_A=")); Serial.println(motorCal.limit1Adc);
  Serial.print(F("LIMITMOTORCALIB LIMIT_B=")); Serial.println(motorCal.limit2Adc);
}

// Returns auto motor safe min angle.
float getAutoMotorSafeMinAngle() {
  return 18.0f;
}

// Returns auto motor safe max angle.
float getAutoMotorSafeMaxAngle() {
  return 342.0f;
}

// Utility: auto motor apply candidate.
void autoMotorApplyCandidate(const AutoMotorCandidate& c) {
  minStepDelayUs = c.minStepUs;
  maxStepDelayUs = c.maxStepUs;
  motorStopEnter = c.stopEnter;
  motorStopExit = c.stopExit;
  movePwmFar = c.pwmFar;
  movePwmNear = c.pwmNear;
  holdPwm = c.pwmHold;
  sanitizeMotorRuntimeParams();
  resetMotorControlState();
}

// Utility: auto motor capture seed.
void autoMotorCaptureSeed() {
  autoMotor.seed.minStepUs = minStepDelayUs;
  autoMotor.seed.maxStepUs = maxStepDelayUs;
  autoMotor.seed.stopEnter = motorStopEnter;
  autoMotor.seed.stopExit = motorStopExit;
  autoMotor.seed.pwmFar = movePwmFar;
  autoMotor.seed.pwmNear = movePwmNear;
  autoMotor.seed.pwmHold = holdPwm;
}

// Utility: auto motor build candidates.
uint8_t autoMotorBuildCandidates(AutoMotorStage stage, AutoMotorCandidate* out, uint8_t maxCount) {
  uint8_t n = 0;
  AutoMotorCandidate b = autoMotor.seed;
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
      AutoMotorCandidate c = b;
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
    AutoMotorCandidate base = autoMotor.best.valid ? autoMotor.best.params : b;
    const float enterBase = clampValue(base.stopEnter, 0.6f, 8.0f);
    const float exitBase  = clampValue(base.stopExit, 1.2f, 14.0f);
    const float enterDelta[] = { -0.4f, -0.1f, 0.2f, 0.5f};
    const float exitDelta[]  = { -0.7f, -0.2f, 0.4f, 0.9f};
    for (uint8_t i = 0; i < 4 && n < maxCount; i++) {
      AutoMotorCandidate c = base;
      c.stopEnter = clampValue(enterBase + enterDelta[i], 0.5f, 12.0f);
      c.stopExit = clampValue(exitBase + exitDelta[i], c.stopEnter + 0.5f, 18.0f);
      out[n++] = c;
    }
  }
  return n;
}

// Utility: auto motor prepare stage.
void autoMotorPrepareStage(AutoMotorStage stage) {
  autoMotor.stage = stage;
  autoMotor.cachedStage = AUTO_STAGE_IDLE;
  autoMotor.candidateIndex = 0;
  autoMotor.candidateCount = 0;
  autoMotor.motionIndex = 0;
  autoMotor.motionRunning = false;
  autoMotor.motionMoved = false;
  autoMotor.stageStartMs = millis();
  autoMotor.lastProgressMs = 0;
  memset(autoMotor.candidates, 0, sizeof(autoMotor.candidates));

  if (stage == AUTO_STAGE_TORQUE) {
    autoMotor.testMotions[0] = 18.0f;
    autoMotor.testMotions[1] = 54.0f;
    autoMotor.testMotions[2] = 108.0f;
    autoMotor.testMotions[3] = 180.0f;
    autoMotor.motionCount = 4;
  } else if (stage == AUTO_STAGE_BANDS) {
    autoMotor.testMotions[0] = 36.0f;
    autoMotor.testMotions[1] = 120.0f;
    autoMotor.testMotions[2] = 220.0f;
    autoMotor.motionCount = 3;
  } else {
    autoMotor.motionCount = 0;
  }

  Serial.print(F("AUTOMOTOR STAGE="));
  Serial.println((int)stage);
}

// Utility: auto motor start.
void autoMotorStart() {
  if (!rt.configMode) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
    return;
  }

  if (motorType != MOTOR_TYPE_STEPPER) {
    Serial.println(F("DENIED: AUTOMOTOR REQUIRES MOTORTYPE=STEPPER"));
    return;
  }

  if (autoMotor.active) {
    Serial.println(F("AUTOMOTOR ALREADY ACTIVE"));
    return;
  }

  sanitizeMotorRuntimeParams();
  readMotorAngle();

  if (motorOneTurnSpanRaw < 20) {
    Serial.println(F("DENIED: SPANRAW TOO SMALL - RUN LIMITMOTORCALIB FIRST"));
    return;
  }

  if (!motorFeedbackInsideWindow) {
    Serial.println(F("WARNING: FEEDBACK OUTSIDE CALIBRATED WINDOW - AUTOMOTOR MAY FAIL"));
  }

  if (!rt.active) enterOperationMode();

  autoMotorCaptureSeed();
  autoMotor.best.valid = false;
  autoMotor.best.score = -1000000000.0f;
  autoMotor.modeAttemptIndex = 0;
  autoMotor.originalMode = motorDriveMode;
  autoMotor.cachedStage = AUTO_STAGE_IDLE;
  autoMotor.candidateIndex = 0;
  autoMotor.candidateCount = 0;
  autoMotor.motionIndex = 0;
  autoMotor.motionCount = 0;
  autoMotor.motionRunning = false;
  autoMotor.motionMoved = false;
  memset(autoMotor.candidates, 0, sizeof(autoMotor.candidates));
  autoMotor.active = true;
  autoMotor.stopRequested = false;
  rt.motorArmed = true;
  rt.servoOutputsArmed = false;
  if (!collisionRuntimeEnabled) {
    collisionFlag = false;
    invalidateCollisionPoseCache();
  }
  readMotorAngle();
  motor.targetAngle = motor.currentAngle;
  servos.target[0] = motor.currentAngle;
  servos.actual[0] = motor.currentAngle;
  resetMotorControlState();
  autoMotorPrepareStage(AUTO_STAGE_TORQUE);
  Serial.print(F("AUTOMOTOR STARTED SAFE_RANGE=")); Serial.print(getAutoMotorSafeMinAngle(), 1); Serial.print(F("..")); Serial.println(getAutoMotorSafeMaxAngle(), 1);
  Serial.print(F("AUTOMOTOR STEPPERMODE=")); Serial.print((int)motorDriveMode); Serial.print(F(" [")); Serial.print(getMotorDriveModeName(motorDriveMode)); Serial.println(F("]"));
  Serial.print(F("AUTOMOTOR RAWADC=")); Serial.print(readMotorRawADC());
  Serial.print(F(" ZEROADC=")); Serial.print(motorCalibrationZeroADC);
  Serial.print(F(" SPANRAW=")); Serial.println(motorOneTurnSpanRaw);
  Serial.println(F("AUTOMOTOR WILL TRY OTHER MOTOR MODES AUTOMATICALLY IF THE ACTIVE MODE DOES NOT RESPOND"));
  Serial.println(F("AUTOMOTOR USES LARGE SWEEPS TO CHARACTERIZE START, BRAKE, NEAR AND HOLD BEHAVIOR"));
}

// Utility: auto motor stop.
void autoMotorStop(bool userRequested) {
  bool hadSession = autoMotor.active || autoMotor.candidateCount > 0 || autoMotor.best.valid;

  if (hadSession) {
    setMotorDriveMode(autoMotor.originalMode, false);
    autoMotorApplyCandidate(autoMotor.seed);
  }

  autoMotor.active = false;
  autoMotor.stopRequested = false;
  autoMotor.stage = AUTO_STAGE_IDLE;
  autoMotor.cachedStage = AUTO_STAGE_IDLE;
  autoMotor.candidateIndex = 0;
  autoMotor.candidateCount = 0;
  autoMotor.motionIndex = 0;
  autoMotor.motionCount = 0;
  autoMotor.motionRunning = false;
  autoMotor.motionMoved = false;
  autoMotor.best.valid = false;
  autoMotor.best.score = -1000000000.0f;
  memset(autoMotor.candidates, 0, sizeof(autoMotor.candidates));

  rt.motorArmed = false;
  rt.servoOutputsArmed = false;
  rt.active = false;
  rt.configMode = true;

  readMotorAngle();
  motor.targetAngle = motor.currentAngle;
  servos.target[0] = motor.currentAngle;
  servos.actual[0] = motor.currentAngle;
  resetMotorControlState();

  releaseMotorCoils();
  motorSetDriverPWM(0);
  motorPulse.setSpeed(0.0f);
  motorPulse.disableOutputs();

  if (userRequested) {
    Serial.println(F("AUTOMOTOR STOPPED - ORIGINAL VALUES RESTORED"));
    Serial.println(F("CFG MODE REMAINS ACTIVE"));
  }
}

// Utility: auto motor start motion.
void autoMotorStartMotion(float deltaDeg) {
  readMotorAngle();
  autoMotor.motionStartAngle = motor.currentAngle;
  float safeMin = getAutoMotorSafeMinAngle();
  float safeMax = getAutoMotorSafeMaxAngle();
  float requested = autoMotor.motionStartAngle + deltaDeg;
  if (requested < safeMin || requested > safeMax) {
    float plusRoom = safeMax - autoMotor.motionStartAngle;
    float minusRoom = autoMotor.motionStartAngle - safeMin;
    if (fabsf(plusRoom) >= fabsf(minusRoom)) requested = clampValue((float)(autoMotor.motionStartAngle + fabsf(deltaDeg)), safeMin, safeMax);
    else requested = clampValue((float)(autoMotor.motionStartAngle - fabsf(deltaDeg)), safeMin, safeMax);
  }
  autoMotor.motionTargetAngle = clampValue(requested, safeMin, safeMax);
  motor.targetAngle = autoMotor.motionTargetAngle;
  servos.target[0] = autoMotor.motionTargetAngle;
  autoMotor.motionMaxOvershoot = 0.0f;
  autoMotor.motionSumAbsError = 0.0f;
  autoMotor.motionJitter = 0.0f;
  autoMotor.holdMeanAbsError = 0.0f;
  autoMotor.holdMinAngle = 999999.0f;
  autoMotor.holdMaxAngle = -999999.0f;
  autoMotor.motionSamples = 0;
  autoMotor.holdSamples = 0;
  autoMotor.motionMoved = false;
  autoMotor.motionStartMoveErr = fabs(deltaDeg);
  autoMotor.motionStartMs = millis();
  autoMotor.settleStartMs = 0;
  autoMotor.motionSettleMs = 0;
  autoMotor.motionRunning = true;
}

// Utility: auto motor finish motion.
void autoMotorFinishMotion(bool success) {
  float avgErr = (autoMotor.motionSamples > 0) ? (autoMotor.motionSumAbsError / (float)autoMotor.motionSamples) : 999.0f;
  float holdAvgErr = (autoMotor.holdSamples > 0) ? (autoMotor.holdMeanAbsError / (float)autoMotor.holdSamples) : avgErr;
  float holdP2P = (autoMotor.holdSamples > 1) ? (autoMotor.holdMaxAngle - autoMotor.holdMinAngle) : 0.0f;
  float jitterPenalty = autoMotor.motionJitter;
  if (holdP2P <= 0.9f && holdAvgErr <= max(0.8f, motorStopEnter * 0.85f)) {
    jitterPenalty *= 0.30f;
  } else if (holdP2P <= 1.8f && holdAvgErr <= max(1.1f, motorStopExit * 0.65f)) {
    jitterPenalty *= 0.60f;
  }

  float score = 0.0f;
  score -= avgErr * 22.0f;
  score -= autoMotor.motionMaxOvershoot * 22.0f;
  score -= jitterPenalty * 18.0f;
  score -= autoMotor.motionStartMoveErr * 28.0f;
  score -= holdAvgErr * 14.0f;
  score -= holdP2P * 10.0f;
  score -= ((float)autoMotor.motionSettleMs) * 0.01f;
  if (!success) score -= 1800.0f;

  if (success && score > autoMotor.best.score) {
    autoMotor.best.valid = true;
    autoMotor.best.score = score;
    autoMotor.best.params = autoMotor.current;
  }

  autoMotor.motionRunning = false;
  autoMotor.motionIndex++;
}

// Utility: auto motor finalize.
void autoMotorFinalize() {
  if (!autoMotor.best.valid && autoMotorAdvanceModeAttempt()) {
    return;
  }

  autoMotor.active = false;
  autoMotor.stopRequested = false;
  autoMotor.cachedStage = AUTO_STAGE_IDLE;
  autoMotor.candidateCount = 0;
  autoMotor.motionRunning = false;
  memset(autoMotor.candidates, 0, sizeof(autoMotor.candidates));
  rt.motorArmed = false;
  rt.servoOutputsArmed = false;
  rt.active = false;
  rt.configMode = true;
  if (autoMotor.best.valid) {
    autoMotorApplyCandidate(autoMotor.best.params);
    readMotorAngle();
    motor.targetAngle = motor.currentAngle;
    servos.target[0] = motor.currentAngle;
    servos.actual[0] = motor.currentAngle;
    resetMotorControlState();
    Serial.println(F("AUTOMOTOR DONE"));
    Serial.println(F("FOUND VALUES (NOT SAVED):"));
    Serial.print(F("MOTORTYPE=")); Serial.println((int)motorType);
    Serial.print(F("MOTORTYPE_NAME: ")); Serial.println(getMotorTypeName(motorType));
    Serial.print(F("STEPPERMODE=")); Serial.println((int)motorDriveMode);
    Serial.print(F("STEPPERMODE_NAME: ")); Serial.println(getMotorDriveModeName(motorDriveMode));
    Serial.print(F("STEPRAWADC=")); Serial.println(readMotorRawADC());
    Serial.print(F("STEPWIN=")); Serial.println(motorFeedbackInsideWindow ? 1 : 0);
    Serial.print(F("MINSTEPUS=")); Serial.println(minStepDelayUs);
    Serial.print(F("MAXSTEPUS=")); Serial.println(maxStepDelayUs);
    Serial.print(F("PWMFAR=")); Serial.println(movePwmFar);
    Serial.print(F("PWMNEAR=")); Serial.println(movePwmNear);
    Serial.print(F("PWMHOLD=")); Serial.println(holdPwm);
    Serial.print(F("AUTO PWM CAPS F/N/H=")); Serial.print(getAutoPwmCapFar()); Serial.print('/'); Serial.print(getAutoPwmCapNear()); Serial.print('/'); Serial.println(getAutoPwmCapHold());
    configSessionDirty = true;
    Serial.println(F("BEST PARAMETERS ARE READY IN CFG - USE SHOW TO REVIEW AND SAVE TO STORE EEPROM"));
  } else {
    setMotorDriveMode(autoMotor.originalMode, false);
    autoMotorApplyCandidate(autoMotor.seed);
    Serial.println(F("AUTOMOTOR FAILED - SEED RESTORED"));
    Serial.println(F("CFG MODE REMAINS ACTIVE - REVIEW VALUES, SAVE IF DESIRED, OR EXIT"));
  }
  releaseMotorCoils();
  motorSetDriverPWM(0);
  motorPulse.setSpeed(0.0f);
}

// Prints auto motor status.
void printAutoMotorStatus() {
  Serial.print(F("AUTOMOTOR ACTIVE=")); Serial.println(autoMotor.active ? 1 : 0);
  Serial.print(F("STAGE=")); Serial.println((int)autoMotor.stage);
  Serial.print(F("CANDIDATE=")); Serial.print(autoMotor.candidateIndex + 1); Serial.print(F("/")); Serial.println(autoMotor.candidateCount);
  Serial.print(F("MOTION=")); Serial.print(autoMotor.motionIndex + 1); Serial.print(F("/")); Serial.println(autoMotor.motionCount);
  Serial.print(F("BESTVALID=")); Serial.println(autoMotor.best.valid ? 1 : 0);
  Serial.print(F("BESTSCORE=")); Serial.println(autoMotor.best.score, 2);
  Serial.print(F("MOTORTYPE=")); Serial.print((int)motorType); Serial.print(F(" [")); Serial.print(getMotorTypeName(motorType)); Serial.println(F("]"));
  Serial.print(F("STEPPERMODE=")); Serial.print((int)motorDriveMode); Serial.print(F(" [")); Serial.print(getMotorDriveModeName(motorDriveMode)); Serial.println(F("]"));
  Serial.print(F("RAWADC=")); Serial.println(readMotorRawADC());
  Serial.print(F("ZEROADC=")); Serial.println(motorCalibrationZeroADC);
  Serial.print(F("SPANRAW=")); Serial.println(motorOneTurnSpanRaw);
  Serial.print(F("ANGLE=")); Serial.println(motor.currentAngle, 3);
  Serial.print(F("TARGET=")); Serial.println(motor.targetAngle, 3);
  Serial.print(F("WIN=")); Serial.println(motorFeedbackInsideWindow ? 1 : 0);
  Serial.print(F("MINSTEPUS=")); Serial.println(minStepDelayUs);
  Serial.print(F("MAXSTEPUS=")); Serial.println(maxStepDelayUs);
  Serial.print(F("ALPHAANG=")); Serial.println(angleFilterAlpha, 3);
  Serial.print(F("A0DB=")); Serial.println(motorBaseA0DeadbandDeg, 3);
  Serial.print(F("PIDENTER=")); Serial.println(motorDcPidEnterBandDeg, 3);
  Serial.print(F("PIDEXIT=")); Serial.println(motorDcPidExitBandDeg, 3);
  Serial.print(F("PWMFAR=")); Serial.println(movePwmFar);
  Serial.print(F("PWMNEAR=")); Serial.println(movePwmNear);
  Serial.print(F("PWMHOLD=")); Serial.println(holdPwm);
  Serial.print(F("AUTO_PWM_CAPS=")); Serial.print(getAutoPwmCapFar()); Serial.print('/'); Serial.print(getAutoPwmCapNear()); Serial.print('/'); Serial.println(getAutoPwmCapHold());
  Serial.print(F("SAFE_MIN=")); Serial.println(getAutoMotorSafeMinAngle(), 1);
  Serial.print(F("SAFE_MAX=")); Serial.println(getAutoMotorSafeMaxAngle(), 1);
}

// Utility: auto motor update.
void autoMotorUpdate() {
  if (!autoMotor.active) return;

  if (autoMotor.cachedStage != autoMotor.stage) {
    autoMotor.cachedStage = autoMotor.stage;
    autoMotor.candidateCount = autoMotorBuildCandidates(autoMotor.stage, autoMotor.candidates, 4);
    autoMotor.candidateIndex = 0;
    autoMotor.motionIndex = 0;
    if (autoMotor.candidateCount > 0) {
      autoMotor.current = autoMotor.candidates[0];
      autoMotorApplyCandidate(autoMotor.current);
    }
  }

  if (autoMotor.stopRequested) {
    autoMotorFinalize();
    return;
  }

  if (autoMotor.stage == AUTO_STAGE_DONE) {
    autoMotorFinalize();
    return;
  }

  if (autoMotor.candidateIndex >= autoMotor.candidateCount) {
    if (autoMotor.stage == AUTO_STAGE_TORQUE) autoMotorPrepareStage(AUTO_STAGE_BANDS);
    else autoMotor.stage = AUTO_STAGE_DONE;
    autoMotor.cachedStage = AUTO_STAGE_IDLE;
    return;
  }

  if (!autoMotor.motionRunning) {
    if (autoMotor.motionIndex >= autoMotor.motionCount) {
      autoMotor.candidateIndex++;
      autoMotor.motionIndex = 0;
      if (autoMotor.candidateIndex < autoMotor.candidateCount) {
        autoMotor.current = autoMotor.candidates[autoMotor.candidateIndex];
        autoMotorApplyCandidate(autoMotor.current);
      }
      return;
    }

    float delta = autoMotor.testMotions[autoMotor.motionIndex];
    float safeMin = getAutoMotorSafeMinAngle();
    float safeMax = getAutoMotorSafeMaxAngle();
    float plusRoom = safeMax - motor.currentAngle;
    float minusRoom = motor.currentAngle - safeMin;
    float sign = (plusRoom >= minusRoom) ? 1.0f : -1.0f;
    if ((autoMotor.motionIndex & 1) != 0) sign = -sign;
    autoMotorStartMotion(delta * sign);
    return;
  }

  unsigned long nowMs = millis();
  float absError = fabs(motor.targetAngle - motor.currentAngle);
  float moved = fabs(motor.currentAngle - autoMotor.motionStartAngle);
  autoMotor.motionSamples++;
  autoMotor.motionSumAbsError += absError;

  float overshoot = 0.0f;
  if ((autoMotor.motionTargetAngle >= autoMotor.motionStartAngle && motor.currentAngle > autoMotor.motionTargetAngle) ||
      (autoMotor.motionTargetAngle <= autoMotor.motionStartAngle && motor.currentAngle < autoMotor.motionTargetAngle)) {
    overshoot = fabs(motor.currentAngle - autoMotor.motionTargetAngle);
    if (overshoot > autoMotor.motionMaxOvershoot) autoMotor.motionMaxOvershoot = overshoot;
  }

  if (!autoMotor.motionMoved && moved >= 1.0f) {
    autoMotor.motionMoved = true;
    autoMotor.motionStartMoveErr = absError;
  }

  float instJitter = fabs(motor.filteredSpeed);
  bool buzzNoMove = ((nowMs - autoMotor.motionStartMs) > 1800UL) && (moved < 1.2f);
  bool oscillatory = ((nowMs - autoMotor.motionStartMs) > 2200UL) && (moved < 4.0f) && (autoMotor.motionSamples > 16) && (instJitter > 12.0f);
  bool extremeOvershoot = autoMotor.motionMaxOvershoot > 24.0f;
  bool safeLimitHit = (motor.currentAngle <= (getAutoMotorSafeMinAngle() + 1.5f)) || (motor.currentAngle >= (getAutoMotorSafeMaxAngle() - 1.5f));
  bool feedbackLost = ((nowMs - autoMotor.motionStartMs) > 1200UL) && !motorFeedbackInsideWindow;
  bool timeout = (nowMs - autoMotor.motionStartMs) > 6200UL;
  bool noResponseEarly = (buzzNoMove || (timeout && moved < 1.0f) || feedbackLost) &&
                         (autoMotor.stage == AUTO_STAGE_TORQUE) &&
                         (autoMotor.motionIndex == 0) &&
                         (autoMotor.candidateIndex == 0) &&
                         !autoMotor.best.valid;
  if (noResponseEarly && autoMotorAdvanceModeAttempt()) {
    return;
  }
  if (buzzNoMove || oscillatory || extremeOvershoot || safeLimitHit || feedbackLost || timeout) {
    autoMotorFinishMotion(false);
    return;
  }

  float holdBandEval = max(0.7f, motorStopEnter * 1.15f);
  if (absError <= holdBandEval) {
    if (autoMotor.settleStartMs == 0) autoMotor.settleStartMs = nowMs;

    if ((nowMs - autoMotor.settleStartMs) >= 180UL) {
      autoMotor.holdSamples++;
      autoMotor.holdMeanAbsError += absError;
      if (motor.currentAngle < autoMotor.holdMinAngle) autoMotor.holdMinAngle = motor.currentAngle;
      if (motor.currentAngle > autoMotor.holdMaxAngle) autoMotor.holdMaxAngle = motor.currentAngle;

      float holdP2PNow = (autoMotor.holdSamples > 1) ? (autoMotor.holdMaxAngle - autoMotor.holdMinAngle) : 0.0f;
      float residualAllowance = 1.20f + (float)autoMotor.current.pwmHold * 0.0025f;
      autoMotor.motionJitter += max(0.0f, holdP2PNow - residualAllowance) * 0.25f;

      if ((nowMs - autoMotor.settleStartMs) >= 520UL) {
        float holdAvgErrNow = autoMotor.holdMeanAbsError / (float)max((uint16_t)1, autoMotor.holdSamples);
        bool acceptableResidual = (holdP2PNow <= residualAllowance) && (holdAvgErrNow <= max(0.9f, motorStopExit * 0.60f));
        bool lowResidual = (holdP2PNow <= (residualAllowance * 0.55f)) && (holdAvgErrNow <= max(0.6f, motorStopEnter));
        if (lowResidual || acceptableResidual) {
          autoMotor.motionSettleMs = nowMs - autoMotor.motionStartMs;
          autoMotorFinishMotion(true);
          return;
        }
      }
    }
  } else {
    autoMotor.motionJitter += instJitter * 0.08f;
    autoMotor.settleStartMs = 0;
    autoMotor.holdSamples = 0;
    autoMotor.holdMeanAbsError = 0.0f;
    autoMotor.holdMinAngle = 999999.0f;
    autoMotor.holdMaxAngle = -999999.0f;
  }

  if ((nowMs - autoMotor.lastProgressMs) >= 600UL) {
    autoMotor.lastProgressMs = nowMs;
    Serial.print(F("AUTOMOTOR STAGE=")); Serial.print((int)autoMotor.stage);
    Serial.print(F(" CAND=")); Serial.print(autoMotor.candidateIndex + 1);
    Serial.print(F("/")); Serial.print(autoMotor.candidateCount);
    Serial.print(F(" MOV=")); Serial.print(autoMotor.motionIndex + 1);
    Serial.print(F("/")); Serial.print(autoMotor.motionCount);
    Serial.print(F(" ANG=")); Serial.print(motor.currentAngle, 2);
    Serial.print(F(" TGT=")); Serial.print(motor.targetAngle, 2);
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
    if (baseDeg < MOTOR_INPUT_MIN || baseDeg > MOTOR_INPUT_MAX) return true;
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
  cancelManipulatorFirmwareHome();
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
  const uint8_t maxCommandsPerPass = SERIAL_RX_LINE_BUDGET;
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
            ? clampValue(baseDeg, MOTOR_INPUT_MIN, MOTOR_INPUT_MAX)
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
  for (uint8_t i = 0; i < COLLISION_SEVERITY_CACHE_SLOTS; i++) {
    collisionSeverityCacheValid[i] = false;
    collisionSeverityCacheValue[i] = 0;
    collisionSeverityCacheMs[i] = 0;
  }
  collisionSeverityCacheNextSlot = 0;
  collisionCandidateDecisionValid = false;
  collisionCandidateDecisionAllowed = true;
  collisionCandidateDecisionCollisionFlag = false;
  collisionCandidateDecisionMs = 0;
  lastCollisionGuardEvalMs = 0;
}

// Utility: capture limited actual collision pose.
bool captureLimitedActualCollisionPose(float& baseDeg, float& upperDeg, float& foreDeg, float& forearmRollDeg, float& wristPitchDeg, float& wristRotDeg, float& gripDeg) {
  baseDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_BASE);
  upperDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_UPPER);
  foreDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_FORE);
  forearmRollDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_FOREARM_ROLL);
  wristPitchDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_PITCH);
  wristRotDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_ROLL);
  gripDeg = getManipulatorMemberActualServoDeg(MANIP_MEMBER_GRIPPER);
  return true;
}

// Utility: store collision pose in a cache struct.
void storeCollisionPoseCache(CollisionPoseCache& cache, float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg) {
  cache.baseCenti = packCentiDeg16(baseDeg);
  cache.upperCenti = packCentiDeg16(upperDeg);
  cache.foreCenti = packCentiDeg16(foreDeg);
  cache.forearmRollCenti = packCentiDeg16(forearmRollDeg);
  cache.wristPitchCenti = packCentiDeg16(wristPitchDeg);
  cache.wristRotCenti = packCentiDeg16(wristRotDeg);
  cache.gripCenti = packCentiDeg16(gripDeg);
}

// Utility: compare pose against a cache with a configurable angular epsilon.
bool collisionPoseMatchesCache(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg, const CollisionPoseCache& cache, int16_t epsCenti) {
  if (abs(packCentiDeg16(baseDeg) - cache.baseCenti) > epsCenti) return false;
  if (abs(packCentiDeg16(upperDeg) - cache.upperCenti) > epsCenti) return false;
  if (abs(packCentiDeg16(foreDeg) - cache.foreCenti) > epsCenti) return false;
  if (abs(packCentiDeg16(forearmRollDeg) - cache.forearmRollCenti) > epsCenti) return false;
  if (abs(packCentiDeg16(wristPitchDeg) - cache.wristPitchCenti) > epsCenti) return false;
  if (abs(packCentiDeg16(wristRotDeg) - cache.wristRotCenti) > epsCenti) return false;
  if (abs(packCentiDeg16(gripDeg) - cache.gripCenti) > epsCenti) return false;
  return true;
}

// Utility: collision pose changed.
bool collisionPoseChanged(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg) {
  if (!collisionPoseCacheValid) return true;
  return !collisionPoseMatchesCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg, collisionPoseCache, COLLISION_POSE_EPS_CENTI);
}

// Utility: updating the cached collision pose.
void updateCollisionPoseCache(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg) {
  storeCollisionPoseCache(collisionPoseCache, baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
  collisionPoseCacheValid = true;
  lastCollisionGuardEvalMs = millis();
}

// Utility: remember a verified collision-free pose. This gives the guard a
// real recovery vector instead of relying only on collision-pair count.
void updateCollisionLastSafePose(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg) {
  storeCollisionPoseCache(collisionLastSafePose, baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
  collisionLastSafePoseValid = true;
}

// Utility: angular distance from a pose to a cached pose, in centi-degrees.
// SUM distance is intentional here: recovery may involve several joints moving
// slightly rather than one joint making a large jump.
int32_t collisionAbsCentiDelta(int16_t a, int16_t b) {
  int32_t d = (int32_t)a - (int32_t)b;
  return (d < 0) ? -d : d;
}

int32_t collisionPoseDistanceToCacheCenti(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg, const CollisionPoseCache& cache) {
  int32_t total = 0;
  total += collisionAbsCentiDelta(packCentiDeg16(baseDeg), cache.baseCenti);
  total += collisionAbsCentiDelta(packCentiDeg16(upperDeg), cache.upperCenti);
  total += collisionAbsCentiDelta(packCentiDeg16(foreDeg), cache.foreCenti);
  total += collisionAbsCentiDelta(packCentiDeg16(forearmRollDeg), cache.forearmRollCenti);
  total += collisionAbsCentiDelta(packCentiDeg16(wristPitchDeg), cache.wristPitchCenti);
  total += collisionAbsCentiDelta(packCentiDeg16(wristRotDeg), cache.wristRotCenti);
  total += collisionAbsCentiDelta(packCentiDeg16(gripDeg), cache.gripCenti);
  return total;
}

// Utility: true when the proposed pose makes measurable progress back toward
// the last safe pose. This prevents trapping the arm inside a collision volume
// when pair-count severity does not immediately decrease.
bool collisionPoseMovesTowardLastSafe(float curBase, float curUpper, float curFore, float curForearmRoll, float curWristPitch, float curWristRot, float curGrip,
                                      float proposedBase, float proposedUpper, float proposedFore, float proposedForearmRoll, float proposedWristPitch, float proposedWristRot, float proposedGrip) {
  if (!collisionLastSafePoseValid) return false;
  int32_t currentDist = collisionPoseDistanceToCacheCenti(curBase, curUpper, curFore, curForearmRoll, curWristPitch, curWristRot, curGrip, collisionLastSafePose);
  int32_t proposedDist = collisionPoseDistanceToCacheCenti(proposedBase, proposedUpper, proposedFore, proposedForearmRoll, proposedWristPitch, proposedWristRot, proposedGrip, collisionLastSafePose);
  return (proposedDist + COLLISION_RECOVERY_PROGRESS_EPS_CENTI) < currentDist;
}

// Utility: cached severity for runtime streams.
// The OBB/cylinder collision solver is the heaviest part of the firmware loop.
// Runtime ARMJ streams often repeat almost the same pose many times before the
// servos physically move, so this cache preserves the same collision decision
// without recomputing all collider pairs for noise-sized changes.
uint8_t poseCollisionSeverityCached(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg, bool forceRefresh) {
  unsigned long nowMs = millis();

  if (!forceRefresh) {
    for (uint8_t i = 0; i < COLLISION_SEVERITY_CACHE_SLOTS; i++) {
      if (collisionSeverityCacheValid[i] &&
          ((uint32_t)(nowMs - collisionSeverityCacheMs[i]) <= (uint32_t)COLLISION_SEVERITY_CACHE_MAX_MS) &&
          collisionPoseMatchesCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg, collisionSeverityCachePose[i], COLLISION_SEVERITY_CACHE_EPS_CENTI)) {
        return collisionSeverityCacheValue[i];
      }
    }
  }

  uint8_t severity = poseCollisionSeverity(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
  uint8_t slot = collisionSeverityCacheNextSlot;
  collisionSeverityCacheNextSlot = (uint8_t)((collisionSeverityCacheNextSlot + 1u) % COLLISION_SEVERITY_CACHE_SLOTS);
  storeCollisionPoseCache(collisionSeverityCachePose[slot], baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
  collisionSeverityCacheValue[slot] = severity;
  collisionSeverityCacheMs[slot] = nowMs;
  collisionSeverityCacheValid[slot] = true;
  return severity;
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

  unsigned long nowMs = millis();
  // Fast-path reuse is safe only while the previous candidate was collision-free.
  // Once a collision exists, every recovery command must be checked against the
  // latest actual pose and the last-safe vector; otherwise a stale blocked
  // decision can deny the small steps needed to escape.
  if (collisionCandidateDecisionValid &&
      collisionCandidateDecisionAllowed &&
      !collisionCandidateDecisionCollisionFlag &&
      ((uint32_t)(nowMs - collisionCandidateDecisionMs) <= (uint32_t)COLLISION_CANDIDATE_REUSE_MAX_MS) &&
      collisionPoseMatchesCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg,
                                collisionCandidateDecisionPose, COLLISION_CANDIDATE_REUSE_EPS_CENTI)) {
    collisionFlag = false;
    return true;
  }

  bool allowed = false;
  uint8_t proposedSeverity = poseCollisionSeverityCached(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg, false);
  if (proposedSeverity == 0u) {
    collisionFlag = false;
    updateCollisionPoseCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
    updateCollisionLastSafePose(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
    allowed = true;
  } else if (proposedSeverity == 0xFFu) {
    // 0xFF means configured joint limits were violated, not a recoverable
    // self-collision. Keep those blocked.
    collisionFlag = true;
    emitCollisionBlockedNotice();
    updateCollisionPoseCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
    allowed = false;
  } else {
    float curBase, curUpper, curFore, curForearmRoll, curWristPitch, curWristRot, curGrip;
    captureLimitedActualCollisionPose(curBase, curUpper, curFore, curForearmRoll, curWristPitch, curWristRot, curGrip);
    uint8_t currentSeverity = poseCollisionSeverityCached(curBase, curUpper, curFore, curForearmRoll, curWristPitch, curWristRot, curGrip, false);

    bool recoveryTowardSafe = collisionPoseMovesTowardLastSafe(curBase, curUpper, curFore, curForearmRoll, curWristPitch, curWristRot, curGrip,
                                                                baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
    bool severityRecovery = (currentSeverity > 0u && currentSeverity != 0xFFu && proposedSeverity < currentSeverity);
    bool legacyRecoveryWithoutSafePose = (!collisionLastSafePoseValid && currentSeverity > 0u && currentSeverity != 0xFFu && proposedSeverity <= currentSeverity);

    if (severityRecovery || recoveryTowardSafe || legacyRecoveryWithoutSafePose) {
      // Allow the operator to back out. Keep collisionFlag true until a fully
      // collision-free pose is reached, but do not block the escaping motion.
      collisionFlag = true;
      updateCollisionPoseCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
      allowed = true;
    } else {
      collisionFlag = true;
      emitCollisionBlockedNotice();
      updateCollisionPoseCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
      allowed = false;
    }
  }

  storeCollisionPoseCache(collisionCandidateDecisionPose, baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
  collisionCandidateDecisionAllowed = allowed;
  collisionCandidateDecisionCollisionFlag = collisionFlag;
  collisionCandidateDecisionMs = nowMs;
  collisionCandidateDecisionValid = true;
  return allowed;
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
    if (candidateServo < MOTOR_INPUT_MIN || candidateServo > MOTOR_INPUT_MAX) return false;
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
  cancelManipulatorFirmwareHome();
  if (memberVals == NULL) return false;

  int servoVals[7] = {0, 0, 0, 0, 0, 0, 0};
  bool neutralMask[7] = {false, false, false, false, false, false, false};

  readMotorAngle();
  if (memberVals[0] == 1000) {
    neutralMask[0] = true;
    servoVals[0] = (int)roundf(motor.currentAngle);
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

// Cancels firmware-driven HOME convergence.
void cancelManipulatorFirmwareHome() {
  manipulatorFirmwareHomeActive = false;
  manipulatorFirmwareHomeLastStepMs = 0;
}

// Returns signed HOME delta in member coordinates.
float firmwareHomeMemberDeltaDeg(uint8_t memberIdx, float currentDeg, float targetDeg) {
  currentDeg = clampManipulatorMemberTargetToLimits(memberIdx, currentDeg);
  targetDeg = clampManipulatorMemberTargetToLimits(memberIdx, targetDeg);

  // CH0/A0 is a linear absolute base axis. Do not wrap it across 0/359.
  if (memberIdx == MANIP_MEMBER_BASE && getManipulatorMemberChannel(MANIP_MEMBER_BASE) != 0) {
    return wrapSignedDeg(targetDeg - currentDeg);
  }
  return targetDeg - currentDeg;
}

// Smooths one firmware HOME member with guaranteed convergence.
float smoothManipulatorFirmwareHomeTarget(uint8_t memberIdx, float currentDeg, float targetDeg, float factor) {
  factor = clampValue(factor, 0.0f, 1.0f);
  float boundedCurrent = clampManipulatorMemberTargetToLimits(memberIdx, currentDeg);
  float boundedTarget = clampManipulatorMemberTargetToLimits(memberIdx, targetDeg);
  float delta = firmwareHomeMemberDeltaDeg(memberIdx, boundedCurrent, boundedTarget);

  if (fabsf(delta) <= 0.001f) return boundedTarget;

  float stepSize = fabsf(delta) * factor;
  if (stepSize < MANIPULATOR_FIRMWARE_HOME_MIN_STEP_DEG) {
    stepSize = MANIPULATOR_FIRMWARE_HOME_MIN_STEP_DEG;
  }
  if (fabsf(delta) <= stepSize) return boundedTarget;

  return clampManipulatorMemberTargetToLimits(memberIdx, boundedCurrent + ((delta > 0.0f) ? stepSize : -stepSize));
}

// Checks whether the measured manipulator pose is close enough to HOME.
bool manipulatorFirmwareHomePoseClose(float toleranceDeg) {
  sanitizeManipulatorHomePose(manipHomePose);
  readMotorAngle();
  toleranceDeg = max(0.1f, toleranceDeg);

  for (uint8_t member = 0; member < MANIP_MEMBER_COUNT; member++) {
    float currentDeg = getManipulatorMemberActualDeg(member);
    float targetDeg = clampManipulatorMemberTargetToLimits(member, (float)manipHomePose[member]);
    if (fabsf(firmwareHomeMemberDeltaDeg(member, currentDeg, targetDeg)) > toleranceDeg) {
      return false;
    }
  }
  return true;
}

// Advances firmware HOME as a coherent full-pose step.
bool serviceManipulatorFirmwareHome(bool force) {
  if (!manipulatorFirmwareHomeActive) return true;
  if (robotControlMode != ROBOT_MODE_MANIPULATOR) {
    cancelManipulatorFirmwareHome();
    return false;
  }

  unsigned long nowMs = millis();
  if (!force && manipulatorFirmwareHomeLastStepMs != 0 &&
      (nowMs - manipulatorFirmwareHomeLastStepMs) < MANIPULATOR_FIRMWARE_HOME_STEP_MS) {
    return true;
  }
  manipulatorFirmwareHomeLastStepMs = nowMs;

  readMotorAngle();
  sanitizeManipulatorHomePose(manipHomePose);

  int stepServoVals[MANIP_MEMBER_COUNT];
  bool neutralMask[MANIP_MEMBER_COUNT] = {false, false, false, false, false, false, false};

  for (uint8_t member = 0; member < MANIP_MEMBER_COUNT; member++) {
    float currentMemberDeg = getManipulatorMemberActualDeg(member);
    float targetMemberDeg = clampManipulatorMemberTargetToLimits(member, (float)manipHomePose[member]);
    float steppedMemberDeg = smoothManipulatorFirmwareHomeTarget(
      member,
      currentMemberDeg,
      targetMemberDeg,
      MANIPULATOR_FIRMWARE_HOME_SMOOTH_FACTOR
    );
    float steppedServoDeg = convertManipulatorMemberToServoDeg(member, steppedMemberDeg);
    stepServoVals[member] = (int)roundf(steppedServoDeg);
  }

  bool accepted = applyManipulatorPoseRuntimeTargets(stepServoVals, neutralMask);
  if (!accepted) {
    collisionFlag = true;
    return false;
  }

  if (manipulatorFirmwareHomePoseClose(MANIPULATOR_FIRMWARE_HOME_COMPLETE_TOLERANCE_DEG)) {
    int finalServoVals[MANIP_MEMBER_COUNT];
    for (uint8_t member = 0; member < MANIP_MEMBER_COUNT; member++) {
      finalServoVals[member] = (int)roundf(convertManipulatorMemberToServoDeg(member, (float)manipHomePose[member]));
    }
    bool finalAccepted = applyManipulatorPoseRuntimeTargets(finalServoVals, neutralMask);
    if (finalAccepted) {
      cancelManipulatorFirmwareHome();
    } else {
      collisionFlag = true;
      return false;
    }
  }

  return true;
}

// Applies manipulator runtime home pose.
bool applyManipulatorRuntimeHomePose() {
  sanitizeManipulatorHomePose(manipHomePose);
  manipulatorFirmwareHomeActive = true;
  manipulatorFirmwareHomeLastStepMs = 0;
  return serviceManipulatorFirmwareHome(true);
}

// Applies manipulator pose runtime targets.
bool applyManipulatorPoseRuntimeTargets(const int servoVals[7], const bool neutralMask[7]) {
  if (servoVals == NULL) return false;

  sanitizeCollisionEvaluationState();
  rt.servoOutputsArmed = true;

  readMotorAngle();
  bool baseNeutral = (neutralMask != NULL && neutralMask[0]);
  uint8_t baseCh = getManipulatorMemberChannel(MANIP_MEMBER_BASE);
  float base = baseNeutral
               ? getManipulatorMemberActualServoDeg(MANIP_MEMBER_BASE)
               : ((baseCh == 0)
                  ? resolveLinearBaseTargetDeg((float)servoVals[0], motor.currentAngle)
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
    if (baseNeutral) {
      rt.motorArmed = false;
      motor.targetAngle = motor.currentAngle;
      servos.target[0] = motor.currentAngle;
      servos.actual[0] = motor.currentAngle;
      releaseMotorCoils();
      if (motorType == MOTOR_TYPE_DC_MOTOR) {
        mirrorPwm46FromCh12ForDcMotor();
      } else {
        motorSetDriverPWM(0);
      }
    } else {
      // BASECH=0 means the base uses CH0 with A0 feedback.  In MOTORTYPE=1 the
      // same closed-loop target must arm the DC H-bridge path, not a servo/external-PWM
      // channel.  Reassert the CH0 pin mode here because pins 42/43/46 are
      // shared with the other robot modes.
      configureCh0PinsForCurrentMode();
      rt.motorArmed = true;
      if (motorType == MOTOR_TYPE_DC_MOTOR) {
        motorPulse.setSpeed(0.0f);
        motorPulse.disableOutputs();
      }
      applyServoValue(0, base, APPLY_TARGET);
      // Command handlers only publish the new reference.  The loop() task owns
      // the motor controller, so every transport/input path shares the same
      // timing, telemetry remains periodic, and the base behavior stays
      // independent of the origin of the reference.
    }
  } else {
    if (!getServoNeutral(0)) setServoNeutral(0, true);
    if (rt.motorArmed) {
      rt.motorArmed = false;
      motor.targetAngle = motor.currentAngle;
      servos.target[0] = motor.currentAngle;
      servos.actual[0] = motor.currentAngle;
      releaseMotorCoils();
      motorSetDriverPWM(0);
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

  // Do not release inactive channels or run the full collision guard again on
  // every streamed ARMJ frame. The candidate pose was already validated above;
  // the periodic servo/control task refreshes the actual-pose guard on a fixed
  // budget, keeping telemetry and CH0 control responsive.
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

  uint8_t actualSeverity = poseCollisionSeverityCached(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg, false);
  collisionFlag = (actualSeverity > 0u);
  updateCollisionPoseCache(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
  if (actualSeverity == 0u) {
    updateCollisionLastSafePose(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg);
  }
}



// Returns current PWM percents.
void getCurrentPwmPercents(uint8_t outPerc[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t ch = FIRST_PWM_CH + i;
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



// Returns true while base CH0 motion should not generate stabilization writes
// on the other servo channels.  Manual targets still pass normally; this only
// suppresses IMU-derived correction noise during base movement.
bool manipulatorBaseMotionQuietingServos() {
  if (robotControlMode != ROBOT_MODE_MANIPULATOR || !rt.active) return false;

  unsigned long nowMs = millis();
  float baseError = fabsf(motor.targetAngle - motor.currentAngle);
  bool baseMoving = rt.motorArmed && (
    !motor.inHoldBand ||
    baseError > MANIP_BASE_SERVO_QUIET_ERROR_DEG ||
    fabsf(motor.filteredSpeed) > MANIP_BASE_SERVO_QUIET_SPEED_DPS
  );

  if (baseMoving) {
    manipBaseServoQuietUntilMs = nowMs + MANIP_BASE_SERVO_QUIET_HOLD_MS;
  }

  return ((long)(manipBaseServoQuietUntilMs - nowMs) > 0L);
}

// Updates wrist gimbal runtime.
void updateWristGimbalRuntime() {
  if (!wristGimbalEnabled || !rt.mpu1Ready || !rt.active) return;
  if (manipulatorBaseMotionQuietingServos()) return;
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
  if (manipulatorBaseMotionQuietingServos()) return;
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
  Serial.println(F("  ARMOFFBASE locks base servo angle to base yaw"));
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
  Serial.println(F("  BASE source = CH0 (motor/DC block) or CH1 (servo block)"));
  Serial.println(F("  JOINTS upper/fore/roll/wrist/gripper -> runtime servo channels"));
  Serial.println(F("  AUX PWM 12..15 -> base, upper, forearm, forearm roll force outputs"));
  Serial.println(F(""));
  Serial.print(F("motor_coil_pins="));
  Serial.print(MOTOR_COIL_PINS[0]); Serial.print(','); Serial.print(MOTOR_COIL_PINS[1]); Serial.print(','); Serial.print(MOTOR_COIL_PINS[2]); Serial.print(','); Serial.println(MOTOR_COIL_PINS[3]);
  Serial.print(F("dc_motor_dir_pins="));
  Serial.print(DC_MOTOR_DIR_PINS[0]); Serial.print(','); Serial.println(DC_MOTOR_DIR_PINS[1]);
  Serial.print(F("motor_power_pwm_pin=")); Serial.println(MOTOR_POWER_PWM_PIN);
  Serial.print(F("motor_feedback_pin=")); Serial.println(MOTOR_FEEDBACK_PIN);
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
  Serial.println(F("  PWMADDR=<0xNN>  -> external PWM expander address"));
  Serial.println(F("  PCAADDR=<0xNN>  -> legacy alias for PWMADDR"));
  Serial.println(F("  INAADDR=<0xNN>  -> INA219 #1 address"));
  Serial.println(F("  INA2ADDR=<0xNN> -> INA219 #2 address"));
  Serial.println(F("  MPUADDR=<0xNN>  -> MPU6050 #1 address"));
  Serial.println(F("  MPU2ADDR=<0xNN> -> MPU6050 #2 address (0x00 disables)"));
  Serial.println(F("  MAGADDR=<0xNN>  -> GY-273 address (0x00 = auto detect)"));
  Serial.println(F(""));
  Serial.print(F("PWMADDR=")); printI2CAddressHex(pca9685Addr); Serial.println();
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
  Serial.println(F("  CH0 = base via potentiometer/motor // CH1 = base via channel 1"));
  Serial.println(F("  CH2..11 = servo channels"));
  Serial.println(F("  CH12..15 = PWM channels"));
  Serial.println(F("  Logical arm mapping:"));
  Serial.println(F("    BASECH=<0|1>       -> select base source: 0=POT/motor, 1=channel 1"));
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
  Serial.println(F("    Processing controls Auto Torque; runtime PWM CH12..15 starts from EEPROM values"));
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
  Serial.println(F("  PWMBOOT12..15 are loaded from EEPROM on startup."));
  Serial.println(F("  Runtime PWM commands can still change CH12..15 after boot."));
  Serial.println(F(""));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F("PWMBOOT")); Serial.print((uint8_t)(FIRST_PWM_CH + i)); Serial.print(F("="));
    Serial.println(clampValue(pwmBootPercent[i], (uint8_t)0, (uint8_t)100));
  }
  Serial.println(F(""));
}

// Prints motor config panel.
void printMotorConfigPanel() {
  Serial.println(F("[MOTOR BASE]"));
  Serial.println(F(""));
  Serial.println(F("  ---[TYPE]---"));
  Serial.println(F("  MOTORTYPE=<0|1|name> -> 0 STEPPER, 1 DC_MOTOR"));
  Serial.println(F(""));

  if (motorType == MOTOR_TYPE_STEPPER) {
    Serial.println(F("  ---[STEPPER ONLY]---"));
    Serial.println(F("  STEPPERMODE=<0..3|name> -> 0 FULLSTEP_BIPOLAR, 1 HALFSTEP_BIPOLAR, 2 FULLSTEP_UNIPOLAR, 3 HALFSTEP_UNIPOLAR"));
    Serial.println(F("  AUTOMOTOR     -> automatic motor tuning inside CFG"));
    Serial.println(F("  AUTOSTATUS    -> show automatic tuning progress / best values / feedback window"));
    Serial.println(F("  AUTOSTOP      -> stop automatic motor tuning and restore the original values"));
    Serial.println(F("  STEPDIAG      -> show direct motor diagnostic block"));
    Serial.println(F("  STEPTEST=F,200,80,2200 -> direct open-loop step test"));
    Serial.println(F("  MINSTEPUS=<v> -> minimum step delay"));
    Serial.println(F("  MAXSTEPUS=<v> -> maximum step delay"));
    Serial.println(F("  STOPENTER=<v> -> hold band enter threshold"));
    Serial.println(F("  STOPEXIT=<v>  -> hold band exit threshold"));
    Serial.println(F("  PWMFAR=<v>    -> stepper PWM when angle error is large"));
    Serial.println(F("  PWMNEAR=<v>   -> stepper PWM when angle error is moderate"));
    Serial.println(F("  PWMHOLD=<v>   -> stepper holding PWM"));
    Serial.println(F(""));
  }

  if (motorType == MOTOR_TYPE_DC_MOTOR) {
    Serial.println(F("  ---[DC_MOTOR ONLY]---"));
    Serial.println(F("  KP=<v>        -> base motor proportional gain"));
    Serial.println(F("  KI=<v>        -> base motor integral gain"));
    Serial.println(F("  KD=<v>        -> base motor derivative/damping gain"));
    Serial.println(F("  PIDENTER=<deg> -> DC PID hold/arrival entry band"));
    Serial.println(F("  PIDEXIT=<deg>  -> DC PID release/leave band"));
    Serial.println(F("  PWM channel 12 defines the PID PWM control ceiling on pin 46"));
    Serial.println(F(""));
  }

  Serial.println(F("  ---[BOTH TYPES]---"));
  Serial.println(F("  LIMITMOTORCALIB -> automatic motor limit calibration inside CFG"));
  Serial.println(F("  ZEROADC=<v>     -> potentiometer zero point"));
  Serial.println(F("  SPANRAW=<v>     -> raw span for one turn"));
  Serial.println(F("  ALPHAANG=<v>    -> angle filter factor"));
  Serial.println(F("  A0DB=<deg>      -> base A0 feedback deadband for noise rejection"));
  Serial.println(F(""));
  Serial.print(F("MOTORTYPE=")); Serial.print((int)motorType); Serial.print(F(" [")); Serial.print(getMotorTypeName(motorType)); Serial.println(F("]"));
  if (motorType == MOTOR_TYPE_STEPPER) {
    Serial.print(F("STEPPERMODE=")); Serial.print((int)motorDriveMode); Serial.print(F(" [")); Serial.print(getMotorDriveModeName(motorDriveMode)); Serial.println(F("]"));
    Serial.print(F("MINSTEPUS=")); Serial.println(minStepDelayUs);
    Serial.print(F("MAXSTEPUS=")); Serial.println(maxStepDelayUs);
    Serial.print(F("STOPENTER=")); Serial.println(motorStopEnter, 3);
    Serial.print(F("STOPEXIT=")); Serial.println(motorStopExit, 3);
    Serial.print(F("PWMFAR=")); Serial.println(movePwmFar);
    Serial.print(F("PWMNEAR=")); Serial.println(movePwmNear);
    Serial.print(F("PWMHOLD=")); Serial.println(holdPwm);
  }
  if (motorType == MOTOR_TYPE_DC_MOTOR) {
    Serial.print(F("KP=")); Serial.println(motor.kp, 4);
    Serial.print(F("KI=")); Serial.println(motor.ki, 4);
    Serial.print(F("KD=")); Serial.println(motor.kd, 4);
    Serial.print(F("PIDENTER=")); Serial.println(motorDcPidEnterBandDeg, 3);
    Serial.print(F("PIDEXIT=")); Serial.println(motorDcPidExitBandDeg, 3);
  }
  Serial.print(F("ZEROADC=")); Serial.println(motorCalibrationZeroADC);
  Serial.print(F("SPANRAW=")); Serial.println(motorOneTurnSpanRaw);
  Serial.print(F("ALPHAANG=")); Serial.println(angleFilterAlpha, 3);
  Serial.print(F("A0DB=")); Serial.println(motorBaseA0DeadbandDeg, 3);
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
    Serial.println(getMotorTypeName(motorType));
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
    uint8_t idx = ch - FIRST_PWM_CH;
    uint8_t value = (idx < 4) ? clampValue(pwmBootPercent[idx], (uint8_t)0, (uint8_t)100) : DEFAULT_MANUAL_PWM_PERCENT;
    Serial.print(F("PWMBOOT")); Serial.print(ch); Serial.print(F("=")); Serial.println(value);
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
      printMotorConfigPanel();
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
         strcmp(cmd, "LIMITMOTORCALIB") == 0 || strcmp(cmd, "CALLIMITMOTOR") == 0 ||
         strcmp(cmd, "LIMITMOTORCALIBSTATUS") == 0 || strcmp(cmd, "CALLIMITSTATUS") == 0 ||
         strcmp(cmd, "STEPPERMODE?") == 0 ||
         strcmp(cmd, "MOTORTYPE?") == 0 ||
         strcmp(cmd, "AUTOMOTOR") == 0 ||
         strcmp(cmd, "AUTOSTATUS") == 0 ||
         strcmp(cmd, "AUTOSTOP") == 0 ||
         strcmp(cmd, "MOTORCAL") == 0 ||
         strcmp(cmd, "MOTORCALSTATUS") == 0;
}

// Checks whether known config assignment command.
bool isKnownConfigAssignmentCommand(const char* cmd) {
  if (cmd == NULL || cmd[0] == '\0') return false;
  if (strchr(cmd, '=') == NULL) return false;

  return strncmp(cmd, "ROBOT=", 6) == 0 ||
         strncmp(cmd, "STEPPERMODE=", 12) == 0 ||
         strncmp(cmd, "MOTORTYPE=", 10) == 0 ||
         strncmp(cmd, "COLL=", 5) == 0 ||
         strncmp(cmd, "PWMADDR=", 8) == 0 ||
         strncmp(cmd, "PCAADDR=", 8) == 0 ||
         strncmp(cmd, "INAADDR=", 8) == 0 ||
         strncmp(cmd, "INA2ADDR=", 9) == 0 ||
         strncmp(cmd, "MPUADDR=", 8) == 0 ||
         strncmp(cmd, "MPU2ADDR=", 9) == 0 ||
         strncmp(cmd, "MAGADDR=", 8) == 0 ||
         strncmp(cmd, "KP=", 3) == 0 ||
         strncmp(cmd, "KI=", 3) == 0 ||
         strncmp(cmd, "KD=", 3) == 0 ||
         strncmp(cmd, "PIDENTER=", 9) == 0 ||
         strncmp(cmd, "PIDEXIT=", 8) == 0 ||
         strncmp(cmd, "ZEROADC=", 8) == 0 ||
         strncmp(cmd, "SPANRAW=", 8) == 0 ||
         strncmp(cmd, "MINSTEPUS=", 10) == 0 ||
         strncmp(cmd, "MAXSTEPUS=", 10) == 0 ||
         strncmp(cmd, "STOPENTER=", 10) == 0 ||
         strncmp(cmd, "STOPEXIT=", 9) == 0 ||
         strncmp(cmd, "PWMFAR=", 7) == 0 ||
         strncmp(cmd, "PWMNEAR=", 8) == 0 ||
         strncmp(cmd, "PWMHOLD=", 8) == 0 ||
         strncmp(cmd, "ALPHAANG=", 9) == 0 ||
         strncmp(cmd, "A0DB=", 5) == 0 ||
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
        float ang = resolveLinearBaseTargetDeg(value, motor.currentAngle);
        if (fabsf(ang - servos.target[0]) < servoTargetEpsilonDeg) break;
        if (((ang - motor.currentAngle) * motor.integral) < 0.0f) {
          motor.integral *= 0.35f;
        }
        motor.inHoldBand = false;
        motor.holdStartMs = 0;
        motor.targetAngle = ang;
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
      // OPT command paths remain safe by keeping servo optimization off.
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
  cancelManipulatorFirmwareHome();
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
    rt.motorArmed = true;
  } else if (robotControlMode == ROBOT_MODE_MANIPULATOR && ch == getManipulatorMemberChannel(MANIP_MEMBER_BASE)) {
    rt.motorArmed = false;
  }

  if (robotControlMode == ROBOT_MODE_MANIPULATOR && collisionRuntimeEnabled) {
    int8_t memberIdx = getManipulatorMemberIndexForChannel(ch);
    if (memberIdx >= 0) {
      float candidate = (ch == 0)
                        ? resolveLinearBaseTargetDeg((float)value, motor.currentAngle)
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
  if (ch == 0 && rt.active) {
    configureCh0PinsForCurrentMode();
    // Do not call updateMotorControl() from the command path.  All CH0
    // references are consumed by the periodic loop controller, which keeps
    // behavior identical for packet, text, UI, script or any future source.
  }

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
    readMotorAngle();
    motor.targetAngle = motor.currentAngle;
    servos.target[0] = motor.currentAngle;
    servos.actual[0] = motor.currentAngle;
    rt.motorArmed = !getServoNeutral(ch);
    if (getServoNeutral(ch)) {
      releaseMotorCoils();
      if (motorType == MOTOR_TYPE_DC_MOTOR) {
        mirrorPwm46FromCh12ForDcMotor();
      } else {
        motorSetDriverPWM(0);
      }
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
  rt.motorArmed = false;
  runtimeStreamActive = false;
  runtimeStreamFailsafeLatched = false;
  resetGenericRobotRuntimeState();
  releaseMotorCoils();
  motorSetDriverPWM(0);
  motorPulse.setSpeed(0.0f);
  motorPulse.disableOutputs();
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
      rt.motorArmed = false;
      runtimeStreamActive = false;
      runtimeStreamFailsafeLatched = false;
      resetGenericRobotRuntimeState();
      releaseMotorCoils();
      motorSetDriverPWM(0);
      clearConfigSessionDirty();
      Serial.println(F("CONFIG MODE ENABLED"));
      Serial.println(F("USE 'SHOW' TO VIEW CURRENT SETTINGS"));
    } else {
      Serial.println(F("INVALID CONFIG CODE"));
    }
    return;
  }

  if (strcmp(cmd, "LIMITMOTORCALIB") == 0 || strcmp(cmd, "CALLIMITMOTOR") == 0 || strcmp(cmd, "MOTORCAL") == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    const bool wasActive = motorCal.active;
    const MotorCalStage beforeStage = motorCal.stage;
    handleCallimitMotorCommand();
    bool accepted = false;
    if ((!wasActive || beforeStage == MOTOR_CAL_IDLE || beforeStage == MOTOR_CAL_DONE) &&
        motorCal.active && motorCal.stage == MOTOR_CAL_WAIT_LIMIT1) {
      accepted = true;
    } else if (beforeStage == MOTOR_CAL_WAIT_LIMIT1 && motorCal.stage == MOTOR_CAL_WAIT_LIMIT2) {
      accepted = true;
    } else if (beforeStage == MOTOR_CAL_WAIT_LIMIT2 && motorCal.stage == MOTOR_CAL_DONE) {
      accepted = true;
    }
    if (allowPrimaryAsciiStatusMessages()) {
      if (accepted) printConfigAcceptedEcho(cmd);
      else printConfigInvalidEcho(cmd);
    }
    return;
  }
  if (strcmp(cmd, "LIMITMOTORCALIBSTATUS") == 0 || strcmp(cmd, "CALLIMITSTATUS") == 0) {
    printMotorCalibrationStatus();
    return;
  }
  if (strcmp(cmd, "MOTORTYPE?") == 0) {
    setMotorType(motorType, true);
    return;
  }
  if (strcmp(cmd, "STEPPERMODE?") == 0) {
    if (motorType != MOTOR_TYPE_STEPPER) {
      Serial.println(F("DENIED: STEPPERMODE REQUIRES MOTORTYPE=STEPPER"));
      return;
    }
    setMotorDriveMode(motorDriveMode, true);
    return;
  }
  if (strcmp(cmd, "AUTOMOTOR") == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    if (motorType != MOTOR_TYPE_STEPPER) {
      Serial.println(F("DENIED: AUTOMOTOR REQUIRES MOTORTYPE=STEPPER"));
      return;
    }
    const bool wasActive = autoMotor.active;
    autoMotorStart();
    if (allowPrimaryAsciiStatusMessages()) {
      if (!wasActive && autoMotor.active) printConfigAcceptedEcho(cmd);
      else printConfigInvalidEcho(cmd);
    }
    return;
  }
  if (strcmp(cmd, "AUTOSTOP") == 0) {
    if (motorType != MOTOR_TYPE_STEPPER) {
      Serial.println(F("DENIED: AUTOSTOP REQUIRES MOTORTYPE=STEPPER"));
      return;
    }
    const bool hadSession = autoMotor.active || autoMotor.candidateCount > 0 || autoMotor.best.valid;
    autoMotorStop(true);
    if (allowPrimaryAsciiStatusMessages()) {
      if (hadSession) printConfigAcceptedEcho(cmd);
      else printConfigInvalidEcho(cmd);
    }
    return;
  }
  if (strcmp(cmd, "AUTOSTATUS") == 0) {
    if (motorType != MOTOR_TYPE_STEPPER) {
      Serial.println(F("DENIED: AUTOSTATUS REQUIRES MOTORTYPE=STEPPER"));
      return;
    }
    printAutoMotorStatus();
    return;
  }
  if (strcmp(cmd, "MOTORCALSTATUS") == 0) {
    printMotorCalibrationStatus();
    return;
  }

  if (strncmp(cmd, "STEPPERMODE=", 12) == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    if (motorType != MOTOR_TYPE_STEPPER) {
      Serial.println(F("DENIED: STEPPERMODE REQUIRES MOTORTYPE=STEPPER"));
      return;
    }
    MotorDriveMode newMode;
    if (!parseMotorDriveModeValue(cmd + 12, &newMode)) {
      printConfigInvalidEcho(cmd);
      return;
    }
    setMotorDriveMode(newMode, false);
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

  if (strncmp(cmd, "MOTORTYPE=", 10) == 0) {
    if (!rt.configMode) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("DENIED: CFG MODE REQUIRED"));
      return;
    }
    MotorType newMode;
    if (!parseMotorTypeValue(cmd + 10, &newMode)) {
      printConfigInvalidEcho(cmd);
      return;
    }
    setMotorType(newMode, false);
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

  if (strcmp(key, "PWMADDR") == 0 ||
      strcmp(key, "PCAADDR") == 0 ||
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

    if ((strcmp(key, "PWMADDR") == 0 || strcmp(key, "PCAADDR") == 0 || strcmp(key, "INAADDR") == 0 || strcmp(key, "INA2ADDR") == 0 || strcmp(key, "MPUADDR") == 0) && parsedAddr == 0) {
      printConfigInvalidEcho(rawCmd);
      return;
    }

    if      (strcmp(key, "PWMADDR") == 0 || strcmp(key, "PCAADDR") == 0)  pca9685Addr = parsedAddr;
    else if (strcmp(key, "INAADDR") == 0)  ina219Addr1 = parsedAddr;
    else if (strcmp(key, "INA2ADDR") == 0) ina219Addr2 = parsedAddr;
    else if (strcmp(key, "MPUADDR") == 0)  mpu6050Addr1 = parsedAddr;
    else if (strcmp(key, "MPU2ADDR") == 0) mpu6050Addr2 = parsedAddr;
    else                                        compassAddr = parsedAddr;

    sanitizeMotorRuntimeParams();
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
  // Reject NaN/Inf-like or absurd values before they can reach runtime or EEPROM.
  if (value != value || value < -1000000.0f || value > 1000000.0f) {
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

  bool stepperOnlyConfigKey =
      strcmp(key, "MINSTEPUS") == 0 ||
      strcmp(key, "MAXSTEPUS") == 0 ||
      strcmp(key, "STOPENTER") == 0 ||
      strcmp(key, "STOPEXIT") == 0 ||
      strcmp(key, "PWMFAR") == 0 ||
      strcmp(key, "PWMNEAR") == 0 ||
      strcmp(key, "PWMHOLD") == 0;

  bool dcOnlyConfigKey =
      strcmp(key, "KP") == 0 ||
      strcmp(key, "KI") == 0 ||
      strcmp(key, "KD") == 0 ||
      strcmp(key, "PIDENTER") == 0 ||
      strcmp(key, "PIDEXIT") == 0;

  if (stepperOnlyConfigKey && motorType != MOTOR_TYPE_STEPPER) {
    Serial.println(F("DENIED: COMMAND REQUIRES MOTORTYPE=STEPPER"));
    printConfigInvalidEcho(rawCmd);
    return;
  }

  if (dcOnlyConfigKey && motorType != MOTOR_TYPE_DC_MOTOR) {
    Serial.println(F("DENIED: PID REQUIRES MOTORTYPE=DC_MOTOR"));
    printConfigInvalidEcho(rawCmd);
    return;
  }

  bool manipGeometryChanged = false;
  bool genericGeometryChanged = false;
  bool dcPidChanged = false;

  if      (strcmp(key, "PWMADDR") == 0 || strcmp(key, "PCAADDR") == 0)   pca9685Addr = (uint8_t)clampValue((int)value, 1, 127);
  else if (strcmp(key, "INAADDR") == 0)   ina219Addr1 = (uint8_t)clampValue((int)value, 1, 127);
  else if (strcmp(key, "INA2ADDR") == 0)  ina219Addr2 = (uint8_t)clampValue((int)value, 1, 127);
  else if (strcmp(key, "MPUADDR") == 0)   mpu6050Addr1 = (uint8_t)clampValue((int)value, 1, 127);
  else if (strcmp(key, "MPU2ADDR") == 0)  mpu6050Addr2 = (uint8_t)clampValue((int)value, 0, 127);
  else if (strcmp(key, "MAGADDR") == 0)   compassAddr = (uint8_t)clampValue((int)value, 0, 127);
  else if (strcmp(key, "KP") == 0) { motor.kp = clampValue(value, 0.0f, 50.0f); dcPidChanged = true; }
  else if (strcmp(key, "KI") == 0) { motor.ki = clampValue(value, 0.0f, 10.0f); dcPidChanged = true; }
  else if (strcmp(key, "KD") == 0) { motor.kd = clampValue(value, 0.0f, 10.0f); dcPidChanged = true; }
  else if (strcmp(key, "PIDENTER") == 0) {
    motorDcPidEnterBandDeg = clampValue(value, MOTOR_DC_PID_BAND_MIN_DEG, MOTOR_DC_PID_BAND_MAX_DEG);
    if (motorDcPidExitBandDeg <= motorDcPidEnterBandDeg) {
      motorDcPidExitBandDeg = min(MOTOR_DC_PID_BAND_MAX_DEG, motorDcPidEnterBandDeg + 0.6f);
    }
    dcPidChanged = true;
  }
  else if (strcmp(key, "PIDEXIT") == 0) {
    motorDcPidExitBandDeg = clampValue(value, MOTOR_DC_PID_BAND_MIN_DEG, MOTOR_DC_PID_BAND_MAX_DEG);
    if (motorDcPidExitBandDeg <= motorDcPidEnterBandDeg) {
      motorDcPidEnterBandDeg = max(MOTOR_DC_PID_BAND_MIN_DEG, motorDcPidExitBandDeg - 0.6f);
    }
    dcPidChanged = true;
  }
  else if (strcmp(key, "ZEROADC") == 0)   motorCalibrationZeroADC = (int16_t)value;
  else if (strcmp(key, "SPANRAW") == 0)   motorOneTurnSpanRaw = (int32_t)value;
  else if (strcmp(key, "MINSTEPUS") == 0) minStepDelayUs = (uint16_t)clampValue((long)value, 500L, 60000L);
  else if (strcmp(key, "MAXSTEPUS") == 0) maxStepDelayUs = (uint16_t)clampValue((long)value, 500L, 60000L);
  else if (strcmp(key, "STOPENTER") == 0) motorStopEnter = clampValue(value, 0.5f, 45.0f);
  else if (strcmp(key, "STOPEXIT") == 0)  motorStopExit  = clampValue(value, 0.5f, 60.0f);
  else if (strcmp(key, "PWMFAR") == 0)    movePwmFar = (uint16_t)clampValue((int)value, 0, (int)ICR5);
  else if (strcmp(key, "PWMNEAR") == 0)   movePwmNear = (uint16_t)clampValue((int)value, 0, (int)ICR5);
  else if (strcmp(key, "PWMHOLD") == 0)   holdPwm = (uint16_t)clampValue((int)value, 0, (int)ICR5);
  else if (strcmp(key, "ALPHAANG") == 0)  angleFilterAlpha = clampValue(value, 0.01f, 1.0f);
  else if (strcmp(key, "A0DB") == 0) {
    motorBaseA0DeadbandDeg = clampValue(value,
                                        MOTOR_BASE_A0_DEADBAND_MIN_DEG,
                                        MOTOR_BASE_A0_DEADBAND_MAX_DEG);
  }
  else if (strcmp(key, "RAMPSPD") == 0)   servoMoveStepDeg = clampValue(value, 0.01f, 10.0f);
  else if (strcmp(key, "RAMPSMOOTH") == 0) servoRampSmoothing = clampValue(value, 0.05f, 1.0f);
  else if (strcmp(key, "PWMBOOT12") == 0 || strcmp(key, "PWMBOOT13") == 0 ||
           strcmp(key, "PWMBOOT14") == 0 || strcmp(key, "PWMBOOT15") == 0) {
    // "PWMBOOT12" -> idx 0, "PWMBOOT15" -> idx 3
    uint8_t idx = (uint8_t)(atoi(key + 7) - 12);
    if (idx >= 4) {
      printConfigInvalidEcho(rawCmd);
      return;
    }
    pwmBootPercent[idx] = (uint8_t)clampValue((int)roundf(value), 0, 100);
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

  sanitizeMotorRuntimeParams();
  markProcessingProfilesDirty();
  syncProcessingRobotProfiles();
  if (manipGeometryChanged) {
    recomputeManipulatorOffsetsFromProcessingModel();
  }
  if (genericGeometryChanged) {
    recomputeProcessingRobotDerivedOffsets();
  }
  if (dcPidChanged) {
    resetMotorControlState();
  }
  releaseInactiveManipulatorChannels();
  noteConfigSessionEdited(false);
  printConfigAcceptedEcho(rawCmd);
}

// Applies the manipulator home pose as soon as a runtime connection opens.
void applyConnectionHomePoseIfNeeded() {
  if (robotControlMode != ROBOT_MODE_MANIPULATOR) return;
  prepareManipulatorHardwareForActiveProfile();
  applyManipulatorRuntimeHomePose();
}

// Utility: enter operation mode silent.
void enterOperationModeSilent() {
  rt.servoOutputsArmed = false;
  rt.motorArmed = false;

  for (uint8_t ch = FIRST_SERVO_CH; ch <= LAST_SERVO_CH; ch++) {
    servos.actual[ch] = servos.target[ch];
  }

  readMotorAngle();
  motor.targetAngle = motor.currentAngle;
  servos.target[0] = motor.currentAngle;
  servos.actual[0] = motor.currentAngle;
  resetMotorControlState();

  releaseMotorCoils();
  motorSetDriverPWM(0);

  rt.lastSensorMs = millis();
  rt.lastServoUpdateMs = millis();
  rt.active = true;
  serialHexMode = true;  // switch Serial0 to HEX-framed protocol
  noteRuntimeLinkActivity(0);
  applyConnectionHomePoseIfNeeded();
}

// Utility: enter operation mode.
void enterOperationMode() {
  // Enter runtime and drive the manipulator to its configured home pose.
  rt.servoOutputsArmed = false;
  rt.motorArmed = false;

  for (uint8_t ch = FIRST_SERVO_CH; ch <= LAST_SERVO_CH; ch++) {
    servos.actual[ch] = servos.target[ch];
  }

  readMotorAngle();
  motor.targetAngle = motor.currentAngle;
  servos.target[0] = motor.currentAngle;
  servos.actual[0] = motor.currentAngle;
  resetMotorControlState();

  releaseMotorCoils();
  motorSetDriverPWM(0);

  rt.lastSensorMs = millis();
  rt.lastServoUpdateMs = millis();
  rt.active = true;
  serialHexMode = true;  // switch Serial0 to HEX-framed protocol
  noteRuntimeLinkActivity(0);
  applyConnectionHomePoseIfNeeded();
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
    else noteRuntimeLinkActivity(0);
    // Deterministic runtime transition for the Processing bridge:
    // ACK + tiny 0x20 heartbeat only. Do not send String snapshots or
    // sensor-heavy telemetry until the host has accepted the stream.
    sendHexRuntimeAckOk();
    armRuntimeHexHandshakeGuard();
    return;
  }

  if (handleMotorDiagnosticCommand(cmd)) {
    return;
  }

  // Runtime mode switching must not require CFG mode.  Processing may send
  // ROBOT=<...> as a plain ASCII command or wrapped in HEX frame 0x17 after
  // READY?/HELLO, so handle it before routing assignments to config.
  if (rt.active && !rt.configMode && strncmp(cmd, "ROBOT=", 6) == 0) {
    RobotControlMode newMode;
    if (parseRobotControlModeValue(cmd + 6, &newMode)) {
      switchRobotModeForRuntime(newMode, false, true);
    }
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
