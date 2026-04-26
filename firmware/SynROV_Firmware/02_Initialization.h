// =====================================================================
// SynROV Firmware - Initialization helpers
// ---------------------------------------------------------------------
// Purpose:
//   Runtime state initialization, startup safety, hardware bootstrap and
//   low-level setup helpers executed before normal control begins.
// =====================================================================

// Clears all neutral states.
void clearAllNeutralStates() {
  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    setServoNeutral(i, false);
  }
}

// Enforces gripper closed/open range on the active manipulator channel.
void enforceManipulatorGripperRange() {
  uint8_t gripCh = getManipulatorMemberChannel(MANIP_MEMBER_GRIPPER);
  if (!isServoChannel(gripCh)) return;
  // Standard gripper convention:
  //   0°   = fully closed
  //   100° = fully open
  // Force normal servo direction here so stale EEPROM configs cannot flip it
  // to use the 0-to-100 degree gripper travel.
  servos.direction[gripCh] = 1;
  setServoMinLimit(gripCh, 0.0f);
  setServoMaxLimit(gripCh, MANIP_GRIPPER_MAX_DEG);
  servos.target[gripCh] = clampServoTargetToLimits(gripCh, clampServoInputToChannelRange(gripCh, servos.target[gripCh]));
  servos.actual[gripCh] = clampServoTargetToLimits(gripCh, clampServoInputToChannelRange(gripCh, servos.actual[gripCh]));
}

// Initializes servo bank.
void initServoBank() {
  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    servos.target[i] = 90.0f;
    servos.actual[i] = 90.0f;
    servoLastWrittenDeg[i] = 9999.0f;
    setServoWriteValid(i, false);
    servos.offset[i] = 0;
    setServoSpan(i, 1.0f);
    setServoRamp(i, false);
    servos.optimize[i] = 0;
    servos.direction[i] = 1;
    setServoNeutral(i, false);
    setServoMinLimit(i, SERVO_INPUT_MIN);
    setServoMaxLimit(i, getChannelInputMax(i));
  }

  servos.target[0] = 180.0f;
  servos.actual[0] = 180.0f;
  servos.target[SERVO_EXTENDED_RANGE_CHANNEL] = 180.0f;
  servos.actual[SERVO_EXTENDED_RANGE_CHANNEL] = 180.0f;

  enforceManipulatorGripperRange();
  applySafeStartupPose();
  invalidateCollisionPoseCache();
}

// Initializes stepper state.
void initStepperState() {
  stepper.kp = 3.0000f;
  stepper.ki = 0.5000f;
  stepper.kd = 0.0100f;
  stepper.targetAngle = 180.0f;
  stepper.currentAngle = 180.0f;
  stepper.filteredPot = 0.0f;
  stepper.filteredAngle = 180.0f;
  stepper.filteredSpeed = 0.0f;
  stepper.lastAngle = 180.0f;
  stepper.integral = 0.0f;
  stepper.pwmValue = holdPwm;
  stepper.inHoldBand = false;
  stepper.directionForward = true;
  stepper.lastControlUs = 0;
  stepper.lastAngleUs = 0;
  stepper.lastStepUs = 0;
  stepper.holdStartMs = 0;
  stepper.stepDelayUs = maxStepDelayUs;
  stepper.seq = PHASE_0;
}

// Initializes runtime state.
void initRuntimeState() {
  rt.active = false;
  rt.configMode = false;
  rt.pwmReady = false;
  rt.ina1Ready = false;
  rt.ina2Ready = false;
  rt.mpu1Ready = false;
  rt.mpu2Ready = false;
  rt.compassReady = false;
  rt.servoOutputsArmed = false;
  rt.stepperArmed = false;
  rt.lastSensorMs = 0;
  rt.lastServoUpdateMs = 0;
  compassInit.active = false;
  compassInit.resultReported = false;
  compassInit.candidateCount = 0;
  compassInit.candidateIndex = 0;
  compassInit.candidateAddrs[0] = 0;
  compassInit.candidateAddrs[1] = 0;
  compassInit.currentAddr = 0;
  compassInit.stage = COMPASS_INIT_IDLE;
  compassInit.waitStartMs = 0;
  directStepperTest.active = false;
  directStepperTest.forward = true;
  directStepperTest.totalSteps = 0;
  directStepperTest.stepsRemaining = 0;
  directStepperTest.pwmRaw = 0;
  directStepperTest.stepDelayUs = 0;
  directStepperTest.lastStepUs = 0;
}

// Checks whether stepper PWM timer healthy.
bool isStepperPwmTimerHealthy() {
  return ((TCCR5A & (1 << COM5A1)) != 0) &&
         ((TCCR5A & (1 << WGM51)) != 0) &&
         ((TCCR5B & (1 << WGM53)) != 0) &&
         ((TCCR5B & (1 << CS50)) != 0) &&
         ((TCCR5B & ((1 << CS51) | (1 << CS52))) == 0) &&
         (ICR5 == 400);
}

// Utility: setup high freq PWM pin 46.
void setupHighFreqPwmPin46() {
  pinMode(STEPPER_POWER_PWM_PIN, OUTPUT);
  // Timer5 channel A -> pin 46 (OC5A).
  // Phase-correct PWM with a static TOP in ICR5 = 400 (~20 kHz at 16 MHz / 1).
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;
  ICR5   = 400;
  TCCR5A |= (1 << WGM51);
  TCCR5B |= (1 << WGM53);
  TCCR5A |= (1 << COM5A1);
  TCCR5B |= (1 << CS50);
  OCR5A = 0;
}

// Ensures stepper PWM timer ready.
void ensureStepperPwmTimerReady() {
  if (!isStepperPwmTimerHealthy()) {
    uint16_t previousDuty = OCR5A;
    setupHighFreqPwmPin46();
    OCR5A = clampValue<uint16_t>(previousDuty, 0, ICR5);
  }
}

// Configures fallback outputs.
void configureFallbackOutputs() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(FALLBACK_PWM_PINS[i], OUTPUT);
    analogWrite(FALLBACK_PWM_PINS[i], 0);
  }
}

// Attaches fallback servos.
void attachFallbackServos() {
  for (uint8_t i = 0; i < 11; i++) {
    if (!isFallbackServoAttached(i)) {
      fallbackServos[i].attach(FALLBACK_SERVO_PINS[i]);
      setFallbackServoAttached(i, true);
    }
  }
}

// Detaches fallback servos.
void detachFallbackServos() {
  for (uint8_t i = 0; i < 11; i++) {
    if (isFallbackServoAttached(i)) {
      fallbackServos[i].detach();
      setFallbackServoAttached(i, false);
    }
  }
}

// Releases vehicle hardware.
void releaseVehicleHardware() {
  analogWrite(VEHICLE_LEFT_TRACK_PWM_PIN, 0);
  analogWrite(VEHICLE_RIGHT_TRACK_PWM_PIN, 0);
  digitalWrite(VEHICLE_LEFT_TRACK_DIR_PIN, LOW);
  digitalWrite(VEHICLE_RIGHT_TRACK_DIR_PIN, LOW);
  digitalWrite(VEHICLE_LIGHT_PIN, LOW);

  if (vehicleCameraPanAttached) { vehicleCameraPanServo.detach(); vehicleCameraPanAttached = false; }
  if (vehicleCameraTiltAttached) { vehicleCameraTiltServo.detach(); vehicleCameraTiltAttached = false; }
  if (vehicleLidarScanAttached) { vehicleLidarScanServo.detach(); vehicleLidarScanAttached = false; }

  pinMode(VEHICLE_CAMERA_PAN_PIN, INPUT);
  pinMode(VEHICLE_CAMERA_TILT_PIN, INPUT);
  pinMode(VEHICLE_LIDAR_SCAN_SERVO_PIN, INPUT);
}

// Releases drone hardware.
void releaseDroneHardware() {
  for (uint8_t i = 0; i < 4; i++) {
    droneMotorMicros[i] = droneControlProfile.escMinUs;
    if (droneEscAttached[i]) {
      droneEscServos[i].writeMicroseconds(droneControlProfile.escMinUs);
      droneEscServos[i].detach();
      droneEscAttached[i] = false;
    }
    pinMode(DRONE_ESC_PINS[i], INPUT);
  }
  if (droneCameraPanAttached) { droneCameraPanServo.detach(); droneCameraPanAttached = false; }
  if (droneCameraTiltAttached) { droneCameraTiltServo.detach(); droneCameraTiltAttached = false; }
  pinMode(SHARED_GIMBAL_PAN_PIN, INPUT);
  pinMode(SHARED_GIMBAL_TILT_PIN, INPUT);
  digitalWrite(DRONE_CAMERA_RECORD_PIN, LOW);
  digitalWrite(DRONE_STATUS_LED_PIN, LOW);
}

// Releases all manipulator actuators.
void releaseAllManipulatorActuators() {
  setServoNeutral(0, true);
  rt.stepperArmed = false;
  readStepperAngle();
  stepper.targetAngle = stepper.currentAngle;
  servos.target[0] = stepper.currentAngle;
  servos.actual[0] = stepper.currentAngle;
  releaseStepperCoils();
  stepperSetDriverPWM(0);

  for (uint8_t ch = FIRST_SERVO_CH; ch <= LAST_SERVO_CH; ch++) {
    setServoNeutral(ch, true);
    setServoWriteValid(ch, false);
    forceServoNeutralOutput(ch);
  }
}


// Configures channel 0 pins for current mode.
void configureCh0PinsForCurrentMode() {
  pinMode(DC_MOTOR_DIR_PINS[0], OUTPUT);
  pinMode(DC_MOTOR_DIR_PINS[1], OUTPUT);

  if (robotControlMode == ROBOT_MODE_MANIPULATOR) {
    for (uint8_t i = 0; i < 4; i++) {
      pinMode(STEPPER_COIL_PINS[i], OUTPUT);
      digitalWrite(STEPPER_COIL_PINS[i], LOW);
    }
  } else {
    // Outside manipulator mode these pins are shared with vehicle/drone outputs.
    digitalWrite(DC_MOTOR_DIR_PINS[0], LOW);
    digitalWrite(DC_MOTOR_DIR_PINS[1], LOW);
  }

  if (ch0DriveMode == CH0_MODE_DC_MOTOR) {
    digitalWrite(DC_MOTOR_DIR_PINS[0], LOW);
    digitalWrite(DC_MOTOR_DIR_PINS[1], LOW);
  }
}

// Configures robot mode hardware.
void configureRobotModeHardware() {
  releaseVehicleHardware();
  releaseDroneHardware();

  if (robotControlMode == ROBOT_MODE_MANIPULATOR) {
    configureCh0PinsForCurrentMode();
    if (!rt.pwmReady) {
      attachFallbackServos();
    }
    return;
  }

  detachFallbackServos();
  releaseAllManipulatorActuators();

  if (robotControlMode == ROBOT_MODE_VEHICLE) {
    pinMode(VEHICLE_LEFT_TRACK_PWM_PIN, OUTPUT);
    pinMode(VEHICLE_RIGHT_TRACK_PWM_PIN, OUTPUT);
    pinMode(VEHICLE_LEFT_TRACK_DIR_PIN, OUTPUT);
    pinMode(VEHICLE_RIGHT_TRACK_DIR_PIN, OUTPUT);
    pinMode(VEHICLE_LIGHT_PIN, OUTPUT);
    analogWrite(VEHICLE_LEFT_TRACK_PWM_PIN, 0);
    analogWrite(VEHICLE_RIGHT_TRACK_PWM_PIN, 0);
    digitalWrite(VEHICLE_LEFT_TRACK_DIR_PIN, LOW);
    digitalWrite(VEHICLE_RIGHT_TRACK_DIR_PIN, LOW);
    digitalWrite(VEHICLE_LIGHT_PIN, LOW);

    if (!vehicleCameraPanAttached) { vehicleCameraPanServo.attach(VEHICLE_CAMERA_PAN_PIN); vehicleCameraPanAttached = true; }
    if (!vehicleCameraTiltAttached) { vehicleCameraTiltServo.attach(VEHICLE_CAMERA_TILT_PIN); vehicleCameraTiltAttached = true; }
    if (!vehicleLidarScanAttached) { vehicleLidarScanServo.attach(VEHICLE_LIDAR_SCAN_SERVO_PIN); vehicleLidarScanAttached = true; }

    vehicleCameraPanDeg = 0;
    vehicleCameraTiltDeg = -10;
    vehicleCameraPanServo.write(90);
    vehicleCameraTiltServo.write(80);
    vehicleLidarScanServo.write(90);
    return;
  }

  if (robotControlMode == ROBOT_MODE_DRONE) {
    pinMode(DRONE_CAMERA_RECORD_PIN, OUTPUT);
    pinMode(DRONE_STATUS_LED_PIN, OUTPUT);
    digitalWrite(DRONE_CAMERA_RECORD_PIN, LOW);
    digitalWrite(DRONE_STATUS_LED_PIN, LOW);
    if (!droneCameraPanAttached) { droneCameraPanServo.attach(SHARED_GIMBAL_PAN_PIN); droneCameraPanAttached = true; }
    if (!droneCameraTiltAttached) { droneCameraTiltServo.attach(SHARED_GIMBAL_TILT_PIN); droneCameraTiltAttached = true; }
    droneCameraPanDeg = 0;
    droneCameraTiltDeg = -10;
    droneCameraPanServo.write(90);
    droneCameraTiltServo.write(80);
    droneStabRefValid = false;
    for (uint8_t i = 0; i < 4; i++) {
      if (!droneEscAttached[i]) {
        droneEscServos[i].attach(DRONE_ESC_PINS[i], droneControlProfile.escMinUs, droneControlProfile.escMaxUs);
        droneEscAttached[i] = true;
      }
      droneMotorMicros[i] = droneControlProfile.escMinUs;
      droneEscServos[i].writeMicroseconds(droneControlProfile.escMinUs);
    }
  }
  configureCh0PinsForCurrentMode();
}

// Updates vehicle hardware outputs.
void updateVehicleHardwareOutputs() {
  int leftTargetPct = constrain(vehicleLeftTrackPct, -100, 100);
  int rightTargetPct = constrain(vehicleRightTrackPct, -100, 100);
  if (abs(leftTargetPct) < vehicleControlProfile.motorDeadbandPct) leftTargetPct = 0;
  if (abs(rightTargetPct) < vehicleControlProfile.motorDeadbandPct) rightTargetPct = 0;
  applyVehicleInclinePowerAssist(leftTargetPct, rightTargetPct);
  if (vehicleControlProfile.leftMotorInvert) leftTargetPct = -leftTargetPct;
  if (vehicleControlProfile.rightMotorInvert) rightTargetPct = -rightTargetPct;

  int leftPct = leftTargetPct;
  int rightPct = rightTargetPct;
  if (vehicleOutputCacheValid) {
    int slew = max(1, (int)roundf(vehicleControlProfile.motorSlewPctPerUpdate));
    leftPct = vehicleLastAppliedLeftPct + constrain(leftTargetPct - vehicleLastAppliedLeftPct, -slew, slew);
    rightPct = vehicleLastAppliedRightPct + constrain(rightTargetPct - vehicleLastAppliedRightPct, -slew, slew);
  }

  int panDeg = (int)round(clampValue((float)vehicleCameraPanDeg, vehicleControlProfile.cameraPanMinDeg, vehicleControlProfile.cameraPanMaxDeg));
  int tiltDeg = (int)round(clampValue((float)vehicleCameraTiltDeg, vehicleControlProfile.cameraTiltMinDeg, vehicleControlProfile.cameraTiltMaxDeg));
  int panCmd = constrain(90 + panDeg, 0, 180);
  int tiltCmd = constrain(90 + tiltDeg, 0, 180);

  if (!vehicleOutputCacheValid || leftPct != vehicleLastAppliedLeftPct) {
    int leftPwm = 0;
    if (leftPct != 0) {
      leftPwm = map(abs(leftPct), 0, 100, vehicleControlProfile.motorPwmMin, vehicleControlProfile.motorPwmMax);
      leftPwm = constrain(leftPwm, vehicleControlProfile.motorPwmMin, vehicleControlProfile.motorPwmMax);
    }
    digitalWrite(VEHICLE_LEFT_TRACK_DIR_PIN, (leftPct >= 0) ? HIGH : LOW);
    analogWrite(VEHICLE_LEFT_TRACK_PWM_PIN, leftPwm);
    vehicleLastAppliedLeftPct = leftPct;
  }

  if (!vehicleOutputCacheValid || rightPct != vehicleLastAppliedRightPct) {
    int rightPwm = 0;
    if (rightPct != 0) {
      rightPwm = map(abs(rightPct), 0, 100, vehicleControlProfile.motorPwmMin, vehicleControlProfile.motorPwmMax);
      rightPwm = constrain(rightPwm, vehicleControlProfile.motorPwmMin, vehicleControlProfile.motorPwmMax);
    }
    digitalWrite(VEHICLE_RIGHT_TRACK_DIR_PIN, (rightPct >= 0) ? HIGH : LOW);
    analogWrite(VEHICLE_RIGHT_TRACK_PWM_PIN, rightPwm);
    vehicleLastAppliedRightPct = rightPct;
  }

  if (!vehicleOutputCacheValid || vehicleLightsCommand != vehicleLastAppliedLights) {
    digitalWrite(VEHICLE_LIGHT_PIN, vehicleLightsCommand ? HIGH : LOW);
    vehicleLastAppliedLights = vehicleLightsCommand;
  }

  if (vehicleCameraPanAttached && (!vehicleOutputCacheValid || panCmd != vehicleLastAppliedPanDeg)) {
    vehicleCameraPanServo.write(panCmd);
    vehicleLastAppliedPanDeg = panCmd;
  }
  if (vehicleCameraTiltAttached && (!vehicleOutputCacheValid || tiltCmd != vehicleLastAppliedTiltDeg)) {
    vehicleCameraTiltServo.write(tiltCmd);
    vehicleLastAppliedTiltDeg = tiltCmd;
  }
  if (vehicleLidarScanAttached) {
    int lidarAngle = 90;
    if (vehicleLidarScanCommand) {
      unsigned long cycleMs = max(250UL, (unsigned long)vehicleControlProfile.lidarSweepCycleMs);
      unsigned long phase = millis() % cycleMs;
      long startAngle = (long)vehicleControlProfile.lidarSweepMinDeg;
      long endAngle = (long)vehicleControlProfile.lidarSweepMaxDeg;
      if (phase < (cycleMs / 2UL)) {
        lidarAngle = map((long)phase, 0L, (long)(cycleMs / 2UL), startAngle, endAngle);
      } else {
        lidarAngle = map((long)phase, (long)(cycleMs / 2UL), (long)cycleMs, endAngle, startAngle);
      }
    }
    lidarAngle = constrain(lidarAngle, 0, 180);
    if (!vehicleOutputCacheValid || lidarAngle != vehicleLastAppliedLidarAngle) {
      vehicleLidarScanServo.write(lidarAngle);
      vehicleLastAppliedLidarAngle = lidarAngle;
    }
  }

  vehicleOutputCacheValid = true;
}

// Updates drone hardware outputs.
void updateDroneHardwareOutputs() {
  float measuredAltCm = readDroneAutoAltitudeCm();
  float throttleCmdPct = computeDroneAutoThrottlePct(measuredAltCm);
  float yawCmdPct = constrain((float)droneYawPct, -100.0f, 100.0f);
  float pitchCmdPct = constrain((float)(dronePitchPct + droneForwardPct), -100.0f, 100.0f);
  float rollCmdPct = constrain((float)(droneRollPct + droneStrafePct), -100.0f, 100.0f);
  if (droneAutoTakeoffActive || droneAutoLandActive) {
    yawCmdPct = 0.0f;
    pitchCmdPct = 0.0f;
    rollCmdPct = 0.0f;
  }
  if (fabsf(throttleCmdPct) < droneControlProfile.commandDeadbandPct) throttleCmdPct = 0.0f;
  if (fabsf(yawCmdPct) < droneControlProfile.commandDeadbandPct) yawCmdPct = 0.0f;
  if (fabsf(pitchCmdPct) < droneControlProfile.commandDeadbandPct) pitchCmdPct = 0.0f;
  if (fabsf(rollCmdPct) < droneControlProfile.commandDeadbandPct) rollCmdPct = 0.0f;

  bool anyActive = (throttleCmdPct > 0.0f) || fabsf(yawCmdPct) > 0.0f || fabsf(pitchCmdPct) > 0.0f || fabsf(rollCmdPct) > 0.0f;

  int16_t ax = 0, ay = 0, az = 16384, gx = 0, gy = 0, gz = 0;
  float measuredPitchDeg = droneTelemetryPitchDeg;
  float measuredRollDeg = droneTelemetryRollDeg;
  if (rt.mpu1Ready) {
    mpu6050_1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    measuredPitchDeg = estimatePitchDeg(ax, ay, az);
    measuredRollDeg = estimateRollDeg(ax, ay, az);
  }
  droneTelemetryPitchDeg = blendFloat(droneTelemetryPitchDeg, measuredPitchDeg, 0.28f);
  droneTelemetryRollDeg = blendFloat(droneTelemetryRollDeg, measuredRollDeg, 0.28f);

  bool compassAvailable = false;
  float measuredHeadingDeg = droneTelemetryYawDeg;
  int16_t mx = 0, my = 0, mz = 0;
  if (rt.compassReady && readCompassRaw(mx, my, mz)) {
    compassX = mx;
    compassY = my;
    compassZ = mz;
    compassHeadingDeg = computeCompassHeadingDeg(mx, my);
    measuredHeadingDeg = compassHeadingDeg;
    compassAvailable = true;
  }
  if (!compassAvailable) {
    measuredHeadingDeg = wrapSignedDeg(droneTelemetryYawDeg + yawCmdPct * 0.08f);
  }
  droneTelemetryYawDeg = wrapSignedDeg(blendFloat(droneTelemetryYawDeg, measuredHeadingDeg, compassAvailable ? 0.20f : 0.08f));

  if (!droneStabRefValid || !anyActive || throttleCmdPct <= 0.0f) {
    droneStabRefPitchDeg = droneTelemetryPitchDeg;
    droneStabRefRollDeg = droneTelemetryRollDeg;
    droneStabRefHeadingDeg = droneTelemetryYawDeg;
    droneStabRefValid = true;
  }
  if (fabsf(yawCmdPct) > droneControlProfile.commandDeadbandPct) {
    droneStabRefHeadingDeg = wrapSignedDeg(droneStabRefHeadingDeg + yawCmdPct * 0.22f);
  }

  float lift = max(0.0f, throttleCmdPct) * 0.01f;
  float baseUs = anyActive ? (float)droneControlProfile.escIdleUs + lift * (float)droneControlProfile.throttleRangeUs : (float)droneControlProfile.escMinUs;

  float targetPitchDeg = droneStabRefPitchDeg + pitchCmdPct * 0.12f;
  float targetRollDeg = droneStabRefRollDeg + rollCmdPct * 0.12f;
  float pitchErrorDeg = targetPitchDeg - droneTelemetryPitchDeg;
  float rollErrorDeg = targetRollDeg - droneTelemetryRollDeg;
  float headingErrorDeg = wrapSignedDeg(droneStabRefHeadingDeg - droneTelemetryYawDeg);

  float pitchMixCmd = pitchCmdPct * 0.01f * (float)droneControlProfile.pitchMixUs;
  float rollMixCmd = rollCmdPct * 0.01f * (float)droneControlProfile.rollMixUs;
  float yawMixCmd = yawCmdPct * 0.01f * (float)droneControlProfile.yawMixUs;

  float pitchMixStab = clampValue(pitchErrorDeg * 6.0f, -(float)droneControlProfile.pitchMixUs * 0.8f, (float)droneControlProfile.pitchMixUs * 0.8f);
  float rollMixStab = clampValue(rollErrorDeg * 6.0f, -(float)droneControlProfile.rollMixUs * 0.8f, (float)droneControlProfile.rollMixUs * 0.8f);
  float yawMixStab = clampValue(headingErrorDeg * 3.0f, -(float)droneControlProfile.yawMixUs * 0.7f, (float)droneControlProfile.yawMixUs * 0.7f);
  if (!compassAvailable) yawMixStab *= 0.35f;
  if (!anyActive) {
    pitchMixStab = 0.0f;
    rollMixStab = 0.0f;
    yawMixStab = 0.0f;
  }

  float pitchMix = pitchMixCmd + pitchMixStab;
  float rollMix = rollMixCmd + rollMixStab;
  float yawMix = yawMixCmd + yawMixStab;

  float motors[4];
  motors[0] = baseUs - pitchMix + rollMix - yawMix; // FL
  motors[1] = baseUs - pitchMix - rollMix + yawMix; // FR
  motors[2] = baseUs + pitchMix - rollMix - yawMix; // RR
  motors[3] = baseUs + pitchMix + rollMix + yawMix; // RL

  int slewUs = max(1, (int)droneControlProfile.spoolStepUsPerUpdate);
  for (uint8_t i = 0; i < 4; i++) {
    int targetMicro = anyActive ? constrain((int)motors[i], (int)droneControlProfile.escMinUs, (int)droneControlProfile.escMaxUs)
                                : (int)droneControlProfile.escMinUs;
    int micro = targetMicro;
    if (droneOutputCacheValid) {
      micro = droneMotorMicros[i] + constrain(targetMicro - droneMotorMicros[i], -slewUs, slewUs);
    }
    micro = constrain(micro, (int)droneControlProfile.escMinUs, (int)droneControlProfile.escMaxUs);
    droneMotorMicros[i] = micro;
    if (droneEscAttached[i] && (!droneOutputCacheValid || micro != droneLastAppliedMicros[i])) {
      droneEscServos[i].writeMicroseconds(micro);
      droneLastAppliedMicros[i] = micro;
    }
  }

  int panCmd = constrain(90 + constrain(droneCameraPanDeg, -90, 90), 0, 180);
  int tiltCmd = constrain(90 + constrain(droneCameraTiltDeg, -45, 35), 0, 180);
  if (droneCameraPanAttached && (!droneOutputCacheValid || panCmd != droneLastAppliedPanDeg)) {
    droneCameraPanServo.write(panCmd);
    droneLastAppliedPanDeg = panCmd;
  }
  if (droneCameraTiltAttached && (!droneOutputCacheValid || tiltCmd != droneLastAppliedTiltDeg)) {
    droneCameraTiltServo.write(tiltCmd);
    droneLastAppliedTiltDeg = tiltCmd;
  }

  if (!droneOutputCacheValid || droneCameraRecordCommand != droneLastAppliedCameraRecord) {
    digitalWrite(DRONE_CAMERA_RECORD_PIN, droneCameraRecordCommand ? HIGH : LOW);
    droneLastAppliedCameraRecord = droneCameraRecordCommand;
  }
  if (!droneOutputCacheValid || anyActive != droneLastAppliedStatusLed) {
    digitalWrite(DRONE_STATUS_LED_PIN, anyActive ? HIGH : LOW);
    droneLastAppliedStatusLed = anyActive;
  }

  droneOutputCacheValid = true;
}

// Updates direct robot hardware outputs.
void updateDirectRobotHardwareOutputs() {
  if (robotControlMode == ROBOT_MODE_VEHICLE) {
    updateVehicleHardwareOutputs();
  } else if (robotControlMode == ROBOT_MODE_DRONE) {
    updateDroneHardwareOutputs();
  }
}

// Returns stepper drive mode name.
const __FlashStringHelper* getStepperDriveModeName(StepperDriveMode mode) {
  switch (mode) {
    case STEPPER_MODE_FULLSTEP_BIPOLAR:  return F("FULLSTEP_BIPOLAR");
    case STEPPER_MODE_HALFSTEP_BIPOLAR:  return F("HALFSTEP_BIPOLAR");
    case STEPPER_MODE_FULLSTEP_UNIPOLAR: return F("FULLSTEP_UNIPOLAR");
    case STEPPER_MODE_HALFSTEP_UNIPOLAR: return F("HALFSTEP_UNIPOLAR");
    default: return F("HALFSTEP_UNIPOLAR");
  }
}

// Sanitizes and clamps stepper drive mode.
StepperDriveMode sanitizeStepperDriveMode(uint8_t rawMode) {
  if (rawMode > (uint8_t)STEPPER_MODE_HALFSTEP_UNIPOLAR) {
    return STEPPER_MODE_HALFSTEP_UNIPOLAR;
  }
  return (StepperDriveMode)rawMode;
}

// Utility: encode control modes for store.
uint8_t encodeControlModesForStore(StepperDriveMode mode, Ch0DriveMode ch0ModeValue) {
  uint8_t packed = 0;
  packed |= ((uint8_t)sanitizeStepperDriveMode((uint8_t)mode) & 0x03);
  packed |= ((uint8_t)sanitizeCh0DriveMode((uint8_t)ch0ModeValue) & 0x01) << 2;
  return (uint8_t)(STEPMODE_STORE_MAGIC | (packed & 0x0F));
}

// Utility: decode stored stepper drive mode.
StepperDriveMode decodeStoredStepperDriveMode(uint8_t rawMode) {
  if ((rawMode & 0xF0) == STEPMODE_STORE_MAGIC) {
    return sanitizeStepperDriveMode(rawMode & 0x03);
  }
  switch (rawMode) {
    case 0: return STEPPER_MODE_HALFSTEP_UNIPOLAR;
    case 1: return STEPPER_MODE_HALFSTEP_BIPOLAR;
    case 2: return STEPPER_MODE_FULLSTEP_UNIPOLAR;
    case 3: return STEPPER_MODE_HALFSTEP_UNIPOLAR;
    default: return STEPPER_MODE_HALFSTEP_UNIPOLAR;
  }
}

// Sanitizes and clamps channel 0 drive mode.
Ch0DriveMode sanitizeCh0DriveMode(uint8_t rawMode) {
  return (rawMode == (uint8_t)CH0_MODE_DC_MOTOR) ? CH0_MODE_DC_MOTOR : CH0_MODE_STEPPER;
}

// Utility: decode stored channel 0 drive mode.
Ch0DriveMode decodeStoredCh0DriveMode(uint8_t rawMode) {
  if ((rawMode & 0xF0) == STEPMODE_STORE_MAGIC) {
    return sanitizeCh0DriveMode((rawMode >> 2) & 0x01);
  }
  return CH0_MODE_STEPPER;
}

// Returns channel 0 drive mode name.
const __FlashStringHelper* getCh0DriveModeName(Ch0DriveMode mode) {
  return (mode == CH0_MODE_DC_MOTOR) ? F("DC_MOTOR") : F("STEPPER");
}

// Sanitizes and clamps robot control mode.
RobotControlMode sanitizeRobotControlMode(uint8_t rawMode) {
  if (rawMode > (uint8_t)ROBOT_MODE_DRONE) return ROBOT_MODE_MANIPULATOR;
  return (RobotControlMode)rawMode;
}

// Returns robot control mode name.
const __FlashStringHelper* getRobotControlModeName(RobotControlMode mode) {
  switch (sanitizeRobotControlMode((uint8_t)mode)) {
    case ROBOT_MODE_VEHICLE: return F("Vehicle");
    case ROBOT_MODE_DRONE: return F("Drone");
    default: return F("Manipulator");
  }
}

// Parses robot control mode value.
bool parseRobotControlModeValue(const char* value, RobotControlMode* outMode) {
  if (value == NULL || outMode == NULL) return false;
  if (strcmp(value, "0") == 0 || strcmp(value, "MANIPULATOR") == 0) {
    *outMode = ROBOT_MODE_MANIPULATOR;
    return true;
  }
  if (strcmp(value, "1") == 0 || strcmp(value, "VEHICLE") == 0) {
    *outMode = ROBOT_MODE_VEHICLE;
    return true;
  }
  if (strcmp(value, "2") == 0 || strcmp(value, "DRONE") == 0) {
    *outMode = ROBOT_MODE_DRONE;
    return true;
  }
  return false;
}

// Applies the current canonical robot control mode.
void applyRobotControlMode(RobotControlMode mode, bool announce) {
  robotControlMode = sanitizeRobotControlMode((uint8_t)mode);
  syncProcessingRobotProfiles();
  resetGenericRobotRuntimeState();
  if (robotControlMode != ROBOT_MODE_MANIPULATOR) {
    collisionFlag = false;
  }
  configureRobotModeHardware();
  releaseInactiveManipulatorChannels();
  if (announce && allowPrimaryAsciiStatusMessages()) {
    printRobotControlModeStatus();
    if (rt.active) sendStaticTelemetryConfigPacket();
  }
}

// Loads the canonical robot control mode from EEPROM.
void loadRobotControlModeFromEEPROM() {
  uint8_t raw = EEPROM.read(EEPROM_ADDR_ROBOT_MODE);
  if (raw == 0xFF) raw = (uint8_t)ROBOT_MODE_MANIPULATOR;
  applyRobotControlMode(sanitizeRobotControlMode(raw), false);
}

// Saves the canonical robot control mode to EEPROM.
void saveRobotControlModeToEEPROM() {
  EEPROM.write(EEPROM_ADDR_ROBOT_MODE, (uint8_t)robotControlMode);
}

// Prints the canonical robot control mode status.
void printRobotControlModeStatus() {
  Serial.print(F("ROBOT="));
  Serial.println(getRobotControlModeName(robotControlMode));
  Serial.print(F("ROBOT_INDEX="));
  Serial.println((int)robotControlMode);
}

// Resets generic robot runtime state.
void resetGenericRobotRuntimeState() {
  vehicleLeftTrackPct = 0;
  vehicleRightTrackPct = 0;
  vehicleCameraPanDeg = (int)round(clampValue(vehicleProfile.cameraPanDefaultDeg, vehicleControlProfile.cameraPanMinDeg, vehicleControlProfile.cameraPanMaxDeg));
  vehicleCameraTiltDeg = (int)round(clampValue(vehicleProfile.cameraTiltDefaultDeg, vehicleControlProfile.cameraTiltMinDeg, vehicleControlProfile.cameraTiltMaxDeg));
  vehicleLightsCommand = false;
  vehicleLidarScanCommand = true;
  vehicleTelemetryX = 0.0f;
  vehicleTelemetryZ = 0.0f;
  vehicleTelemetryYawDeg = 0.0f;
  vehicleTelemetryLeftTrackActual = 0.0f;
  vehicleTelemetryRightTrackActual = 0.0f;

  droneThrottlePct = 0;
  droneYawPct = 0;
  dronePitchPct = 0;
  droneRollPct = 0;
  droneStrafePct = 0;
  droneForwardPct = 0;
  droneCameraRecordCommand = false;
  droneAutoTakeoffActive = false;
  droneAutoLandActive = false;
  droneAutoTargetAltitudeCm = DRONE_AUTO_TAKEOFF_TARGET_CM;
  droneAutoLastSonarCm = -1;
  droneAutoLastSonarReadMs = 0;
  droneLastRawSonarCm = -1;
  droneSonarVerticalCm = -1.0f;
  droneSonarGroundReferenceCm = -1.0f;
  droneSonarGroundReferenceValid = false;
  droneAltitudeFromGroundCm = 0.0f;
  droneTelemetryX = 0.0f;
  droneTelemetryY = 0.0f;
  droneTelemetryZ = 0.0f;
  droneTelemetryYawDeg = 0.0f;
  droneTelemetryPitchDeg = 0.0f;
  droneTelemetryRollDeg = 0.0f;
  droneTelemetryAltitudeCm = 0.0f;
  droneTelemetryYawCmdFiltered = 0.0f;
  droneTelemetryPitchCmdFiltered = 0.0f;
  droneTelemetryRollCmdFiltered = 0.0f;
  droneTelemetryThrottleCmdFiltered = 0.0f;
  droneTelemetryStrafeCmdFiltered = 0.0f;
  droneTelemetryForwardCmdFiltered = 0.0f;
  droneTelemetryLift = 0.0f;
  droneTelemetryTargetLift = 0.0f;

  vehicleOutputCacheValid = false;
  vehicleLastAppliedLeftPct = 32767;
  vehicleLastAppliedRightPct = 32767;
  vehicleLastAppliedPanDeg = 32767;
  vehicleLastAppliedTiltDeg = 32767;
  vehicleLastAppliedLidarAngle = 32767;
  droneOutputCacheValid = false;
  for (uint8_t i = 0; i < 4; i++) {
    droneMotorMicros[i] = droneControlProfile.escMinUs;
    droneLastAppliedMicros[i] = -1;
  }

  resetSafetyAutonomyRuntimeState();

  genericRobotLastUpdateMs = millis();
}

// Arm helper for finger pivot offset.
float armFingerPivotOffset() {
  return armFingerH * ARM_FINGER_PIVOT_HEIGHT_RATIO;
}

// Arm helper for finger lateral offset.
float armFingerLateralOffset() {
  return (armWristW * 0.5f) + (armFingerW * 0.5f) + ARM_FINGER_LATERAL_CLEARANCE_CM;
}

// Fills default manipulator home pose.
void fillDefaultManipulatorHomePose(int16_t pose[MANIP_MEMBER_COUNT]) {
  if (pose == NULL) return;
  for (uint8_t i = 0; i < MANIP_MEMBER_COUNT; i++) {
    pose[i] = (int16_t)pgm_read_word_near(kDefaultManipHomePose + i);
  }
}

// Sanitizes and clamps manipulator home pose.
void sanitizeManipulatorHomePose(int16_t pose[MANIP_MEMBER_COUNT]) {
  if (pose == NULL) return;
  pose[MANIP_MEMBER_BASE] = (int16_t)clampValue((int)pose[MANIP_MEMBER_BASE], 0, 359);
  for (uint8_t i = 1; i < MANIP_MEMBER_COUNT; i++) {
    int maxDeg = (i == MANIP_MEMBER_GRIPPER) ? (int)MANIP_GRIPPER_MAX_DEG : 359;
    pose[i] = (int16_t)clampValue((int)pose[i], 0, maxDeg);
  }
}

// Sanitizes and clamps vehicle processing profile.
void sanitizeVehicleProcessingProfile(VehicleProcessingProfile &cfg) {
  cfg.bodyLength = clampValue(cfg.bodyLength, 1.0f, 1000.0f);
  cfg.bodyWidth = clampValue(cfg.bodyWidth, 1.0f, 1000.0f);
  cfg.bodyHeight = clampValue(cfg.bodyHeight, 1.0f, 1000.0f);
  cfg.trackGauge = clampValue(cfg.trackGauge, 1.0f, 1000.0f);
  cfg.trackRun = clampValue(cfg.trackRun, 1.0f, 1000.0f);
  cfg.driveRadius = clampValue(cfg.driveRadius, 0.1f, 1000.0f);
  cfg.supportRadius = clampValue(cfg.supportRadius, 0.1f, 1000.0f);
  cfg.topRollerRadius = clampValue(cfg.topRollerRadius, 0.1f, 1000.0f);
  cfg.trackWidth = clampValue(cfg.trackWidth, 0.1f, 1000.0f);
  cfg.trackLinkLength = clampValue(cfg.trackLinkLength, 0.1f, 1000.0f);
  cfg.trackLinkThickness = clampValue(cfg.trackLinkThickness, 0.1f, 1000.0f);
  cfg.trackHeight = clampValue(cfg.trackHeight, 0.1f, 1000.0f);
  cfg.trackLinks = clampValue(cfg.trackLinks, 16, 512);
  cfg.trackSag = clampValue(cfg.trackSag, 0.0f, 1000.0f);
  cfg.bodyCenterYOffset = clampValue(cfg.bodyCenterYOffset, -1000.0f, 1000.0f);
  cfg.cameraPanDefaultDeg = clampValue(cfg.cameraPanDefaultDeg, -180.0f, 180.0f);
  cfg.cameraTiltDefaultDeg = clampValue(cfg.cameraTiltDefaultDeg, -180.0f, 180.0f);
  cfg.lidarMastYOffset = clampValue(cfg.lidarMastYOffset, -1000.0f, 1000.0f);
  cfg.lidarMastZOffset = clampValue(cfg.lidarMastZOffset, -1000.0f, 1000.0f);
  cfg.cameraHeadYOffset = clampValue(cfg.cameraHeadYOffset, -1000.0f, 1000.0f);
  cfg.cameraHeadZOffset = clampValue(cfg.cameraHeadZOffset, -1000.0f, 1000.0f);
}

// Sanitizes and clamps drone processing profile.
void sanitizeDroneProcessingProfile(DroneProcessingProfile &cfg) {
  cfg.bodyLength = clampValue(cfg.bodyLength, 1.0f, 1000.0f);
  cfg.bodyWidth = clampValue(cfg.bodyWidth, 1.0f, 1000.0f);
  cfg.bodyHeight = clampValue(cfg.bodyHeight, 1.0f, 1000.0f);
  cfg.armLength = clampValue(cfg.armLength, 0.1f, 1000.0f);
  cfg.armThickness = clampValue(cfg.armThickness, 0.1f, 1000.0f);
  cfg.motorRadius = clampValue(cfg.motorRadius, 0.1f, 1000.0f);
  cfg.motorHeight = clampValue(cfg.motorHeight, 0.1f, 1000.0f);
  cfg.propRadius = clampValue(cfg.propRadius, 0.1f, 1000.0f);
  cfg.propThickness = clampValue(cfg.propThickness, 0.1f, 1000.0f);
  cfg.legHeight = clampValue(cfg.legHeight, 0.1f, 1000.0f);
  cfg.legSpan = clampValue(cfg.legSpan, 0.1f, 1000.0f);
  cfg.restYOffset = clampValue(cfg.restYOffset, -1000.0f, 1000.0f);
  cfg.visualYawOffsetDeg = clampValue(cfg.visualYawOffsetDeg, -180.0f, 180.0f);
  cfg.cameraYOffset = clampValue(cfg.cameraYOffset, -1000.0f, 1000.0f);
  cfg.cameraZOffset = clampValue(cfg.cameraZOffset, -1000.0f, 1000.0f);
  cfg.lampYOffset = clampValue(cfg.lampYOffset, -1000.0f, 1000.0f);
  cfg.lampZOffset = clampValue(cfg.lampZOffset, -1000.0f, 1000.0f);
  cfg.sonarXOffset = clampValue(cfg.sonarXOffset, -1000.0f, 1000.0f);
  cfg.sonarYOffset = clampValue(cfg.sonarYOffset, -1000.0f, 1000.0f);
  cfg.sonarZOffset = clampValue(cfg.sonarZOffset, -1000.0f, 1000.0f);
}

// Sanitizes and clamps vehicle control profile.
void sanitizeVehicleControlProfile(VehicleControlProfile &cfg) {
  cfg.motorDeadbandPct = clampValue(cfg.motorDeadbandPct, 0.0f, 40.0f);
  cfg.motorPwmMin = (uint8_t)clampValue((int)cfg.motorPwmMin, 0, 255);
  cfg.motorPwmMax = (uint8_t)clampValue((int)cfg.motorPwmMax, (int)cfg.motorPwmMin, 255);
  cfg.motorSlewPctPerUpdate = clampValue(cfg.motorSlewPctPerUpdate, 0.5f, 100.0f);
  cfg.leftMotorInvert = cfg.leftMotorInvert ? true : false;
  cfg.rightMotorInvert = cfg.rightMotorInvert ? true : false;
  cfg.cameraPanMinDeg = clampValue(cfg.cameraPanMinDeg, -180.0f, 180.0f);
  cfg.cameraPanMaxDeg = clampValue(cfg.cameraPanMaxDeg, cfg.cameraPanMinDeg, 180.0f);
  cfg.cameraTiltMinDeg = clampValue(cfg.cameraTiltMinDeg, -180.0f, 180.0f);
  cfg.cameraTiltMaxDeg = clampValue(cfg.cameraTiltMaxDeg, cfg.cameraTiltMinDeg, 180.0f);
  cfg.lidarSweepMinDeg = (uint8_t)clampValue((int)cfg.lidarSweepMinDeg, 0, 180);
  cfg.lidarSweepMaxDeg = (uint8_t)clampValue((int)cfg.lidarSweepMaxDeg, (int)cfg.lidarSweepMinDeg, 180);
  cfg.lidarSweepCycleMs = (uint16_t)clampValue((long)cfg.lidarSweepCycleMs, 250L, 10000L);
}

// Sanitizes and clamps drone control profile.
void sanitizeDroneControlProfile(DroneControlProfile &cfg) {
  cfg.escMinUs = (uint16_t)clampValue((long)cfg.escMinUs, 700L, 2000L);
  cfg.escMaxUs = (uint16_t)clampValue((long)cfg.escMaxUs, (long)cfg.escMinUs + 50L, 2500L);
  cfg.escIdleUs = (uint16_t)clampValue((long)cfg.escIdleUs, (long)cfg.escMinUs, (long)cfg.escMaxUs);
  uint16_t maxRange = (cfg.escMaxUs > cfg.escIdleUs) ? (uint16_t)(cfg.escMaxUs - cfg.escIdleUs) : 0;
  cfg.throttleRangeUs = (uint16_t)clampValue((long)cfg.throttleRangeUs, 0L, (long)maxRange);
  cfg.pitchMixUs = (uint16_t)clampValue((long)cfg.pitchMixUs, 0L, 600L);
  cfg.rollMixUs = (uint16_t)clampValue((long)cfg.rollMixUs, 0L, 600L);
  cfg.yawMixUs = (uint16_t)clampValue((long)cfg.yawMixUs, 0L, 600L);
  cfg.commandDeadbandPct = clampValue(cfg.commandDeadbandPct, 0.0f, 20.0f);
  cfg.spoolStepUsPerUpdate = (uint16_t)clampValue((long)cfg.spoolStepUsPerUpdate, 1L, 400L);
}

// Utility: recompute manipulator offsets from processing model.
void recomputeManipulatorOffsetsFromProcessingModel() {
  sanitizeArmGeometryConfig(armGeom);
  invalidateCollisionPoseCache();
}

// Utility: recompute processing robot derived offsets.
void recomputeProcessingRobotDerivedOffsets() {
  sanitizeVehicleProcessingProfile(vehicleProfile);
  sanitizeDroneProcessingProfile(droneProfile);
  sanitizeVehicleControlProfile(vehicleControlProfile);
  sanitizeDroneControlProfile(droneControlProfile);
  vehicleProfile.trackLinks = max(16, vehicleProfile.trackLinks);
}

// Marks processing profiles dirty.
void markProcessingProfilesDirty() {
  processingProfilesDirty = true;
}

// Synchronizes processing robot profiles.
void syncProcessingRobotProfiles() {
  if (processingProfilesInitialized && !processingProfilesDirty) return;

  sanitizeManipulatorProfile();
  sanitizeManipulatorHomePose(manipHomePose);
  recomputeProcessingRobotDerivedOffsets();

  processingProfilesInitialized = true;
  processingProfilesDirty = false;
}

// Sanitizes and clamps manipulator member channel.
uint8_t sanitizeManipulatorMemberChannel(uint8_t memberIdx, uint8_t ch) {
  if (memberIdx == MANIP_MEMBER_BASE) {
    if (ch <= 1) return ch;
    return kDefaultManipProfile.baseChannel;
  }
  if (ch >= kDefaultManipProfile.upperChannel && ch <= LAST_SERVO_CH) return ch;
  switch (memberIdx) {
    case MANIP_MEMBER_UPPER: return kDefaultManipProfile.upperChannel;
    case MANIP_MEMBER_FORE: return kDefaultManipProfile.foreChannel;
    case MANIP_MEMBER_FOREARM_ROLL: return kDefaultManipProfile.forearmRollChannel;
    case MANIP_MEMBER_WRIST_PITCH: return kDefaultManipProfile.wristPitchChannel;
    case MANIP_MEMBER_WRIST_ROLL: return kDefaultManipProfile.wristRotChannel;
    case MANIP_MEMBER_GRIPPER: return kDefaultManipProfile.gripperChannel;
    default: return kDefaultManipProfile.upperChannel;
  }
}

// Utility: manipulator member channel ptr.
uint8_t* manipulatorMemberChannelPtr(uint8_t memberIdx) {
  switch (memberIdx) {
    case MANIP_MEMBER_BASE: return &manipProfile.baseChannel;
    case MANIP_MEMBER_UPPER: return &manipProfile.upperChannel;
    case MANIP_MEMBER_FORE: return &manipProfile.foreChannel;
    case MANIP_MEMBER_FOREARM_ROLL: return &manipProfile.forearmRollChannel;
    case MANIP_MEMBER_WRIST_PITCH: return &manipProfile.wristPitchChannel;
    case MANIP_MEMBER_WRIST_ROLL: return &manipProfile.wristRotChannel;
    case MANIP_MEMBER_GRIPPER: return &manipProfile.gripperChannel;
    default: return NULL;
  }
}

// Returns manipulator member name.
const __FlashStringHelper* getManipulatorMemberName(uint8_t memberIdx) {
  switch (memberIdx) {
    case MANIP_MEMBER_BASE: return F("BASE");
    case MANIP_MEMBER_UPPER: return F("UPPER");
    case MANIP_MEMBER_FORE: return F("FORE");
    case MANIP_MEMBER_FOREARM_ROLL: return F("FOREROLL");
    case MANIP_MEMBER_WRIST_PITCH: return F("WRISTPITCH");
    case MANIP_MEMBER_WRIST_ROLL: return F("WRISTROT");
    case MANIP_MEMBER_GRIPPER: return F("GRIPPER");
    default: return F("UNKNOWN");
  }
}

// Returns default manipulator member channel.
uint8_t getDefaultManipulatorMemberChannel(uint8_t memberIdx) {
  switch (memberIdx) {
    case MANIP_MEMBER_BASE: return kDefaultManipProfile.baseChannel;
    case MANIP_MEMBER_UPPER: return kDefaultManipProfile.upperChannel;
    case MANIP_MEMBER_FORE: return kDefaultManipProfile.foreChannel;
    case MANIP_MEMBER_FOREARM_ROLL: return kDefaultManipProfile.forearmRollChannel;
    case MANIP_MEMBER_WRIST_PITCH: return kDefaultManipProfile.wristPitchChannel;
    case MANIP_MEMBER_WRIST_ROLL: return kDefaultManipProfile.wristRotChannel;
    case MANIP_MEMBER_GRIPPER: return kDefaultManipProfile.gripperChannel;
    default: return FIRST_SERVO_CH;
  }
}

// Returns manipulator member startup safe degrees.
float getManipulatorMemberStartupSafeDeg(uint8_t memberIdx) {
  sanitizeManipulatorHomePose(manipHomePose);
  switch (memberIdx) {
    case MANIP_MEMBER_BASE: return (float)manipHomePose[MANIP_MEMBER_BASE];
    case MANIP_MEMBER_UPPER: return (float)manipHomePose[MANIP_MEMBER_UPPER];
    case MANIP_MEMBER_FORE: return (float)manipHomePose[MANIP_MEMBER_FORE];
    case MANIP_MEMBER_FOREARM_ROLL: return (float)manipHomePose[MANIP_MEMBER_FOREARM_ROLL];
    case MANIP_MEMBER_WRIST_PITCH: return (float)manipHomePose[MANIP_MEMBER_WRIST_PITCH];
    case MANIP_MEMBER_WRIST_ROLL: return (float)manipHomePose[MANIP_MEMBER_WRIST_ROLL];
    case MANIP_MEMBER_GRIPPER: return (float)manipHomePose[MANIP_MEMBER_GRIPPER];
    default: return 90.0f;
  }
}

// Returns manipulator member channel.
uint8_t getManipulatorMemberChannel(uint8_t memberIdx) {
  uint8_t* ptr = manipulatorMemberChannelPtr(memberIdx);
  if (ptr == NULL) return FIRST_SERVO_CH;
  return *ptr;
}

// Returns manipulator member index for channel.
int8_t getManipulatorMemberIndexForChannel(uint8_t ch) {
  for (uint8_t i = 0; i < MANIP_MEMBER_COUNT; i++) {
    if (getManipulatorMemberChannel(i) == ch) return (int8_t)i;
  }
  return -1;
}

// Utility: manipulator member channel is unique.
bool manipulatorMemberChannelIsUnique(uint8_t memberIdx, uint8_t ch) {
  for (uint8_t i = 0; i < MANIP_MEMBER_COUNT; i++) {
    if (i == memberIdx) continue;
    if (getManipulatorMemberChannel(i) == ch) return false;
  }
  return true;
}

// Sets manipulator member channel.
bool setManipulatorMemberChannel(uint8_t memberIdx, uint8_t ch) {
  uint8_t* ptr = manipulatorMemberChannelPtr(memberIdx);
  if (ptr == NULL) return false;
  uint8_t sanitized = sanitizeManipulatorMemberChannel(memberIdx, ch);
  if (!manipulatorMemberChannelIsUnique(memberIdx, sanitized)) return false;
  *ptr = sanitized;
  sanitizeManipulatorProfile();
  enforceManipulatorGripperRange();
  markProcessingProfilesDirty();
  releaseInactiveManipulatorChannels();
  invalidateCollisionPoseCache();
  return true;
}

// Sets manipulator base channel flexible.
bool setManipulatorBaseChannelFlexible(uint8_t ch) {
  if (ch > 1) return false;
  uint8_t sanitized = sanitizeManipulatorMemberChannel(MANIP_MEMBER_BASE, ch);
  uint8_t previousCh = getManipulatorMemberChannel(MANIP_MEMBER_BASE);
  float seededBaseDeg = 180.0f;
  if (sanitized == 0) {
    if (previousCh == 0) {
      seededBaseDeg = clampValue(readStepperAngle(), STEPPER_INPUT_MIN, STEPPER_INPUT_MAX);
    } else if (isServoChannel(previousCh)) {
      seededBaseDeg = clampServoTargetToLimits(previousCh, clampServoInputToChannelRange(previousCh, servos.actual[previousCh]));
    }
  }

  manipProfile.baseChannel = sanitized;
  if (sanitized == 0) {
    stepper.targetAngle = clampValue(seededBaseDeg, STEPPER_INPUT_MIN, STEPPER_INPUT_MAX);
    stepper.currentAngle = stepper.targetAngle;
    servos.target[0] = stepper.targetAngle;
    servos.actual[0] = stepper.targetAngle;
    setServoWriteValid(0, false);
  } else if (isServoChannel(sanitized)) {
    float seededServoDeg = clampServoTargetToLimits(sanitized, clampServoInputToChannelRange(sanitized, seededBaseDeg));
    servos.target[sanitized] = seededServoDeg;
    servos.actual[sanitized] = seededServoDeg;
    setServoWriteValid(sanitized, false);
  }

  markProcessingProfilesDirty();
  sanitizeManipulatorProfile();
  enforceManipulatorGripperRange();
  releaseInactiveManipulatorChannels();
  invalidateCollisionPoseCache();
  return true;
}

// Sanitizes and clamps manipulator profile.
void sanitizeManipulatorProfile() {
  if (manipProfile.baseChannel > 1) manipProfile.baseChannel = kDefaultManipProfile.baseChannel;
  if (manipProfile.upperChannel < kDefaultManipProfile.upperChannel || manipProfile.upperChannel > LAST_SERVO_CH) manipProfile.upperChannel = kDefaultManipProfile.upperChannel;
  if (manipProfile.foreChannel < kDefaultManipProfile.upperChannel || manipProfile.foreChannel > LAST_SERVO_CH) manipProfile.foreChannel = kDefaultManipProfile.foreChannel;
  if (manipProfile.forearmRollChannel < kDefaultManipProfile.upperChannel || manipProfile.forearmRollChannel > LAST_SERVO_CH) manipProfile.forearmRollChannel = kDefaultManipProfile.forearmRollChannel;
  if (manipProfile.wristPitchChannel < kDefaultManipProfile.upperChannel || manipProfile.wristPitchChannel > LAST_SERVO_CH) manipProfile.wristPitchChannel = kDefaultManipProfile.wristPitchChannel;
  if (manipProfile.wristRotChannel < kDefaultManipProfile.upperChannel || manipProfile.wristRotChannel > LAST_SERVO_CH) manipProfile.wristRotChannel = kDefaultManipProfile.wristRotChannel;
  if (manipProfile.gripperChannel < kDefaultManipProfile.upperChannel || manipProfile.gripperChannel > LAST_SERVO_CH) manipProfile.gripperChannel = kDefaultManipProfile.gripperChannel;

  bool used[LAST_SERVO_CH + 1];
  for (uint8_t i = 0; i <= LAST_SERVO_CH; i++) used[i] = false;
  uint8_t order[MANIP_MEMBER_COUNT] = {
    MANIP_MEMBER_BASE,
    MANIP_MEMBER_UPPER,
    MANIP_MEMBER_FORE,
    MANIP_MEMBER_FOREARM_ROLL,
    MANIP_MEMBER_WRIST_PITCH,
    MANIP_MEMBER_WRIST_ROLL,
    MANIP_MEMBER_GRIPPER
  };
  for (uint8_t idx = 0; idx < MANIP_MEMBER_COUNT; idx++) {
    uint8_t member = order[idx];
    uint8_t ch = sanitizeManipulatorMemberChannel(member, getManipulatorMemberChannel(member));
    if (used[ch]) {
      uint8_t fallback = getDefaultManipulatorMemberChannel(member);
      fallback = sanitizeManipulatorMemberChannel(member, fallback);
      if (fallback <= LAST_SERVO_CH && !used[fallback]) {
        ch = fallback;
      } else {
        uint8_t probeStart = (member == MANIP_MEMBER_BASE) ? 0 : kDefaultManipProfile.upperChannel;
        uint8_t probeEnd = (member == MANIP_MEMBER_BASE) ? 1 : LAST_SERVO_CH;
        for (uint8_t probe = probeStart; probe <= probeEnd; probe++) {
          if (!used[probe]) { ch = probe; break; }
        }
        if (member == MANIP_MEMBER_BASE && !used[0]) ch = 0;
      }
      uint8_t* ptr = manipulatorMemberChannelPtr(member);
      if (ptr != NULL) *ptr = ch;
    }
    used[ch] = true;
  }
}

// Utility: base telemetry uses potentiometer.
bool baseTelemetryUsesPotentiometer() {
  return getManipulatorMemberChannel(MANIP_MEMBER_BASE) == 0;
}

// Checks whether manipulator member channel assigned.
bool isManipulatorMemberChannelAssigned(uint8_t ch) {
  if (ch > LAST_SERVO_CH) return false;
  for (uint8_t i = 0; i < MANIP_MEMBER_COUNT; i++) {
    if (getManipulatorMemberChannel(i) == ch) return true;
  }
  return false;
}

// Releases inactive manipulator channels.
void releaseInactiveManipulatorChannels() {
  if (robotControlMode != ROBOT_MODE_MANIPULATOR) return;
  syncProcessingRobotProfiles();

  if (getManipulatorMemberChannel(MANIP_MEMBER_BASE) != 0 || getServoNeutral(0)) {
    setServoNeutral(0, true);
    rt.stepperArmed = false;
    readStepperAngle();
    stepper.targetAngle = stepper.currentAngle;
    servos.target[0] = stepper.currentAngle;
    servos.actual[0] = stepper.currentAngle;
    releaseStepperCoils();
    stepperSetDriverPWM(0);
  }

  for (uint8_t ch = FIRST_SERVO_CH; ch <= LAST_SERVO_CH; ch++) {
    if (isManipulatorMemberChannelAssigned(ch)) continue;
    setServoNeutral(ch, true);
    setServoWriteValid(ch, false);
    forceServoNeutralOutput(ch);
  }
}

// Returns manipulator member min limit.
float getManipulatorMemberMinLimit(uint8_t memberIdx) {
  uint8_t ch = getManipulatorMemberChannel(memberIdx);
  if (memberIdx == MANIP_MEMBER_BASE && ch == 0) return STEPPER_INPUT_MIN;
  return getServoMinLimit(ch);
}

// Returns manipulator member max limit.
float getManipulatorMemberMaxLimit(uint8_t memberIdx) {
  uint8_t ch = getManipulatorMemberChannel(memberIdx);
  if (memberIdx == MANIP_MEMBER_BASE && ch == 0) return STEPPER_INPUT_MAX;
  return getServoMaxLimit(ch);
}

// Clamps manipulator member target to limits.
float clampManipulatorMemberTargetToLimits(uint8_t memberIdx, float value) {
  return clampValue(value, getManipulatorMemberMinLimit(memberIdx), getManipulatorMemberMaxLimit(memberIdx));
}

// Utility: manipulator member angle within configured limits.
bool manipulatorMemberAngleWithinConfiguredLimits(uint8_t memberIdx, float value, float tol) {
  float mn = getManipulatorMemberMinLimit(memberIdx) - tol;
  float mx = getManipulatorMemberMaxLimit(memberIdx) + tol;
  return value >= mn && value <= mx;
}

// Returns manipulator member actual servo degrees.
float getManipulatorMemberActualServoDeg(uint8_t memberIdx) {
  uint8_t ch = getManipulatorMemberChannel(memberIdx);
  if (memberIdx == MANIP_MEMBER_BASE && baseTelemetryUsesPotentiometer()) {
    return clampValue(readStepperAngle(), STEPPER_INPUT_MIN, STEPPER_INPUT_MAX);
  }
  if (ch >= MAX_CHANNELS) return 0.0f;
  if (memberIdx == MANIP_MEMBER_BASE && ch == 0) {
    return clampValue(stepper.currentAngle, STEPPER_INPUT_MIN, STEPPER_INPUT_MAX);
  }
  if (memberIdx == MANIP_MEMBER_BASE &&
      !getServoWriteValid(ch) &&
      fabsf(servos.actual[ch]) < 0.001f &&
      fabsf(servos.target[ch]) < 0.001f) {
    float startupBaseDeg = clampManipulatorMemberTargetToLimits(memberIdx, getManipulatorMemberStartupSafeDeg(memberIdx));
    servos.actual[ch] = startupBaseDeg;
    servos.target[ch] = startupBaseDeg;
    return startupBaseDeg;
  }
  return clampManipulatorMemberTargetToLimits(memberIdx, clampServoInputToChannelRange(ch, servos.actual[ch]));
}

// Returns manipulator member target servo degrees.
float getManipulatorMemberTargetServoDeg(uint8_t memberIdx) {
  uint8_t ch = getManipulatorMemberChannel(memberIdx);
  if (memberIdx == MANIP_MEMBER_BASE && ch == 0) {
    return clampValue(servos.target[0], STEPPER_INPUT_MIN, STEPPER_INPUT_MAX);
  }
  if (ch >= MAX_CHANNELS) return 0.0f;
  return clampManipulatorMemberTargetToLimits(memberIdx, clampServoInputToChannelRange(ch, servos.target[ch]));
}

// Parses strict manipulator config float.
bool parseStrictManipulatorConfigFloat(const char* text, float& value) {
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

// Parses strict signed long value.
bool parseStrictLongValue(const char* text, long& value) {
  if (text == NULL) return false;

  while (*text != '\0' && isspace((unsigned char)*text)) {
    ++text;
  }
  if (*text == '\0') return false;

  char* endPtr = NULL;
  long parsed = strtol(text, &endPtr, 10);
  if (endPtr == text) return false;

  while (*endPtr != '\0' && isspace((unsigned char)*endPtr)) {
    ++endPtr;
  }
  if (*endPtr != '\0') return false;

  value = parsed;
  return true;
}

// Parses strict bool 0/1 text.
bool parseStrictBool01Text(const char* text, bool& value) {
  long parsed = 0;
  if (!parseStrictLongValue(text, parsed)) return false;
  if (parsed == 0L) {
    value = false;
    return true;
  }
  if (parsed == 1L) {
    value = true;
    return true;
  }
  return false;
}

// Parses strict CSV signed long values.
bool parseStrictCsvLongValues(const char* text, long* values, uint8_t maxCount, uint8_t& outCount) {
  if (text == NULL || values == NULL || maxCount == 0) return false;

  char buf[192];
  strncpy(buf, text, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  outCount = 0;
  char* tok = strtok(buf, ",");
  while (tok != NULL) {
    if (outCount >= maxCount) return false;
    long parsed = 0;
    if (!parseStrictLongValue(tok, parsed)) return false;
    values[outCount++] = parsed;
    tok = strtok(NULL, ",");
  }

  return outCount > 0;
}

// Parses a strict signed integer pair formatted as "A,B".
bool parseSignedPair(const char* text, int &a, int &b) {
  if (text == NULL) return false;

  long values[2] = {0, 0};
  uint8_t count = 0;
  if (!parseStrictCsvLongValues(text, values, 2, count)) return false;
  if (count != 2) return false;

  a = (int)values[0];
  b = (int)values[1];
  return true;
}

// Parses manipulator armj values.
bool parseManipulatorArmjValues(const char* poseText, int jointVals[7], int pwmVals[4], bool &hasPwm) {
  if (poseText == NULL || jointVals == NULL || pwmVals == NULL) return false;

  long values[11] = {0};
  uint8_t count = 0;
  if (!parseStrictCsvLongValues(poseText, values, 11, count)) return false;
  if (count != 7 && count != 11) return false;

  for (uint8_t i = 0; i < 7; i++) {
    jointVals[i] = (int)values[i];
  }
  hasPwm = (count == 11);
  for (uint8_t i = 0; i < 4; i++) {
    pwmVals[i] = hasPwm ? (int)clampValue((int)values[7 + i], 0, 100) : 0;
  }
  return true;
}

// Handles manipulator runtime command.
bool handleManipulatorRuntimeCommand(char* cmd) {
  if (cmd == NULL || cmd[0] == '\0') return false;

  const char* poseText = NULL;
  if (strncmp(cmd, "ARMJ=", 5) == 0) poseText = cmd + 5;

  if (poseText != NULL) {
    int vals[7] = {0, 0, 0, 0, 0, 0, 0};
    int pwmVals[4] = {0, 0, 0, 0};
    bool hasPwm = false;
    if (!parseManipulatorArmjValues(poseText, vals, pwmVals, hasPwm)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID ARMJ COMMAND"));
      return true;
    }
    if (!applyManipulatorPoseRuntimeMemberValues(vals)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("ARMJ DENIED"));
      return true;
    }
    if (hasPwm) {
      for (uint8_t i = 0; i < 4; i++) applyRuntimePwmValue((uint8_t)(FIRST_PWM_CH + i), (float)pwmVals[i]);
    }
    return true;
  }

  return false;
}

// Handles vehicle runtime command.
bool handleVehicleRuntimeCommand(char* cmd) {
  if (!requireReadyForRuntimeCommands()) return true;

  if (strcmp(cmd, "STOP") == 0) {
    vehicleLeftTrackPct = 0;
    vehicleRightTrackPct = 0;
    return true;
  }

  if (strncmp(cmd, "LIGHT=", 6) == 0) {
    bool parsedEnable = false;
    if (!parseStrictBool01Text(cmd + 6, parsedEnable)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID LIGHT COMMAND"));
      return true;
    }
    vehicleLightsCommand = parsedEnable;
    return true;
  }

  if (strncmp(cmd, "SCAN=", 5) == 0) {
    const char* valueText = cmd + 5;
    bool parsedEnable = false;
    if (!parseStrictBool01Text(valueText, parsedEnable)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID SCAN COMMAND"));
      return true;
    }
    vehicleLidarScanCommand = parsedEnable;
    return true;
  }

  if (strncmp(cmd, "CAM=", 4) == 0) {
    int pan = 0, tilt = 0;
    if (!parseSignedPair(cmd + 4, pan, tilt)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID CAM COMMAND"));
      return true;
    }
    vehicleCameraPanDeg = (int)round(clampValue((float)pan, vehicleControlProfile.cameraPanMinDeg, vehicleControlProfile.cameraPanMaxDeg));
    vehicleCameraTiltDeg = (int)round(clampValue((float)tilt, vehicleControlProfile.cameraTiltMinDeg, vehicleControlProfile.cameraTiltMaxDeg));
    return true;
  }

  if (strncmp(cmd, "MOVE=", 5) == 0) {
    const char* driveText = cmd + 5;
    int throttle = 0, steer = 0;
    if (!parseSignedPair(driveText, throttle, steer)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID MOVE COMMAND"));
      return true;
    }
    throttle = constrain(throttle, -100, 100);
    steer = constrain(steer, -100, 100);
    vehicleLeftTrackPct = constrain(throttle + steer, -100, 100);
    vehicleRightTrackPct = constrain(throttle - steer, -100, 100);
    recordVehicleLinkHoldMotion();
    return true;
  }

  if (strncmp(cmd, "MAGCAL=", 7) == 0) {
    bool parsedEnable = false;
    if (!parseStrictBool01Text(cmd + 7, parsedEnable)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID MAGCAL COMMAND"));
      return true;
    }
    if (parsedEnable) startCompassCalibration();
    else cancelCompassCalibration(false);
    return true;
  }

  if (strncmp(cmd, "TRACK=", 6) == 0) {
    char* payload = cmd + 6;
    char* cam = strstr(payload, ",CAM=");
    if (cam != NULL) {
      *cam = '\0';
      int pan = 0, tilt = 0;
      if (parseSignedPair(cam + 5, pan, tilt)) {
        vehicleCameraPanDeg = (int)round(clampValue((float)pan, vehicleControlProfile.cameraPanMinDeg, vehicleControlProfile.cameraPanMaxDeg));
        vehicleCameraTiltDeg = (int)round(clampValue((float)tilt, vehicleControlProfile.cameraTiltMinDeg, vehicleControlProfile.cameraTiltMaxDeg));
      }
    }

    int left = 0, right = 0;
    if (!parseSignedPair(payload, left, right)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID TRACK COMMAND"));
      return true;
    }
    vehicleLeftTrackPct = constrain(left, -100, 100);
    vehicleRightTrackPct = constrain(right, -100, 100);
    recordVehicleLinkHoldMotion();
    return true;
  }

  return false;
}

// Handles drone runtime command.
bool handleDroneRuntimeCommand(char* cmd) {
  if (!requireReadyForRuntimeCommands()) return true;

  if (strcmp(cmd, "STOP") == 0) {
    droneThrottlePct = 0;
    droneYawPct = 0;
    dronePitchPct = 0;
    droneRollPct = 0;
    droneStrafePct = 0;
    droneForwardPct = 0;
    droneAutoTakeoffActive = false;
    droneAutoLandActive = false;
    return true;
  }

  if (strcmp(cmd, "TAKEOFF") == 0) {
    droneThrottlePct = 0;
    droneYawPct = 0;
    dronePitchPct = 0;
    droneRollPct = 0;
    droneStrafePct = 0;
    droneForwardPct = 0;
    droneAutoTakeoffActive = true;
    droneAutoLandActive = false;
    droneAutoTargetAltitudeCm = DRONE_AUTO_TAKEOFF_TARGET_CM;
    return true;
  }

  if (strcmp(cmd, "LAND") == 0) {
    droneThrottlePct = 0;
    droneYawPct = 0;
    dronePitchPct = 0;
    droneRollPct = 0;
    droneStrafePct = 0;
    droneForwardPct = 0;
    droneAutoTakeoffActive = false;
    droneAutoLandActive = true;
    return true;
  }

  if (strncmp(cmd, "CAMREC=", 7) == 0) {
    bool parsedEnable = false;
    if (!parseStrictBool01Text(cmd + 7, parsedEnable)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID CAMREC COMMAND"));
      return true;
    }
    droneCameraRecordCommand = parsedEnable;
    return true;
  }

  if (strncmp(cmd, "CAM=", 4) == 0) {
    long camVals[2] = {0, 0};
    uint8_t camCount = 0;
    if (parseStrictCsvLongValues(cmd + 4, camVals, 2, camCount) && camCount == 2) {
      droneCameraPanDeg = constrain((int)camVals[0], -90, 90);
      droneCameraTiltDeg = constrain((int)camVals[1], -45, 35);
      return true;
    }
    bool parsedEnable = false;
    if (!parseStrictBool01Text(cmd + 4, parsedEnable)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID CAM COMMAND"));
      return true;
    }
    droneCameraRecordCommand = parsedEnable;
    return true;
  }

  if (strncmp(cmd, "MAGCAL=", 7) == 0) {
    bool parsedEnable = false;
    if (!parseStrictBool01Text(cmd + 7, parsedEnable)) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID MAGCAL COMMAND"));
      return true;
    }
    if (parsedEnable) startCompassCalibration();
    else cancelCompassCalibration(false);
    return true;
  }

  if (strncmp(cmd, "FLY=", 4) == 0) {
    char moveBuf[96];
    const char* moveTextSrc = cmd + 4;
    strncpy(moveBuf, moveTextSrc, sizeof(moveBuf) - 1);
    moveBuf[sizeof(moveBuf) - 1] = '\0';

    char* camPart = strstr(moveBuf, ",CAM=");
    if (camPart != NULL) {
      *camPart = '\0';
      camPart += 5;
      long camVals[2] = {0, 0};
      uint8_t camCount = 0;
      if (parseStrictCsvLongValues(camPart, camVals, 2, camCount) && camCount == 2) {
        droneCameraPanDeg = constrain((int)camVals[0], -90, 90);
        droneCameraTiltDeg = constrain((int)camVals[1], -45, 35);
      }
    }

    long vals[6] = {0, 0, 0, 0, 0, 0};
    uint8_t count = 0;
    if (!parseStrictCsvLongValues(moveBuf, vals, 6, count) || count != 6) {
      if (allowPrimaryAsciiStatusMessages()) Serial.println(F("INVALID FLY COMMAND"));
      return true;
    }
    droneThrottlePct = constrain((int)vals[0], -100, 100);
    droneYawPct = constrain((int)vals[1], -100, 100);
    dronePitchPct = constrain((int)vals[2], -100, 100);
    droneRollPct = constrain((int)vals[3], -100, 100);
    droneStrafePct = constrain((int)vals[4], -100, 100);
    droneForwardPct = constrain((int)vals[5], -100, 100);
    if (droneThrottlePct != 0 || droneYawPct != 0 || dronePitchPct != 0 || droneRollPct != 0 || droneStrafePct != 0 || droneForwardPct != 0) {
      droneAutoTakeoffActive = false;
      droneAutoLandActive = false;
      recordDroneLinkHoldMotion();
    }
    return true;
  }

  return false;
}

// Updates generic robot runtime.
void updateGenericRobotRuntime() {
  unsigned long nowMs = millis();
  if (genericRobotLastUpdateMs == 0) genericRobotLastUpdateMs = nowMs;
  float dt = (nowMs - genericRobotLastUpdateMs) / 1000.0f;
  genericRobotLastUpdateMs = nowMs;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.100f) dt = 0.100f;

  // Mirror the Processing-side 3D kinematics as closely as possible so the
  // firmware telemetry remains visually aligned with the desktop scene.
  if (robotControlMode == ROBOT_MODE_VEHICLE) {
    float leftCmd = vehicleLeftTrackPct * 0.01f;
    float rightCmd = vehicleRightTrackPct * 0.01f;

    vehicleTelemetryLeftTrackActual = blendFloat(vehicleTelemetryLeftTrackActual, leftCmd, 0.10f);
    vehicleTelemetryRightTrackActual = blendFloat(vehicleTelemetryRightTrackActual, rightCmd, 0.10f);

    float avgHardware = (vehicleTelemetryLeftTrackActual + vehicleTelemetryRightTrackActual) * 0.5f;
    float avg = -avgHardware;
    float diff = (vehicleTelemetryRightTrackActual - vehicleTelemetryLeftTrackActual) * 0.5f;

    float turnBlend = clampValue((float)fabs(diff), 0.0f, 1.0f);
    float tractionFactor = 1.0f - 0.22f * turnBlend;
    float navYawStepDeg = diff * 0.92f * dt * 60.0f;
    float navForwardStep = avg * 1.85f * dt * 60.0f * tractionFactor;
    if (fabs(avg) < 0.02f && fabs(diff) > 0.02f) navForwardStep = 0.0f;

    float motionYawDeg = vehicleTelemetryYawDeg + navYawStepDeg * 0.5f;
    vehicleTelemetryYawDeg += navYawStepDeg;

    float motionYawRad = radians(motionYawDeg);
    vehicleTelemetryX += sin(motionYawRad) * navForwardStep;
    vehicleTelemetryZ += cos(motionYawRad) * navForwardStep;
  } else if (robotControlMode == ROBOT_MODE_DRONE) {
    float measuredAltCm = readDroneAutoAltitudeCm();
    float throttleCmd = computeDroneAutoThrottlePct(measuredAltCm) * 0.01f;
    float yawCmd = (droneAutoTakeoffActive || droneAutoLandActive) ? 0.0f : (droneYawPct * 0.01f);
    float pitchCmd = (droneAutoTakeoffActive || droneAutoLandActive) ? 0.0f : (dronePitchPct * 0.01f);
    float rollCmd = (droneAutoTakeoffActive || droneAutoLandActive) ? 0.0f : (droneRollPct * 0.01f);
    float strafeCmd = (droneAutoTakeoffActive || droneAutoLandActive) ? 0.0f : (droneStrafePct * 0.01f);
    float forwardCmd = (droneAutoTakeoffActive || droneAutoLandActive) ? 0.0f : (droneForwardPct * 0.01f);

    droneTelemetryYawCmdFiltered = blendFloat(droneTelemetryYawCmdFiltered, yawCmd, 0.12f);
    droneTelemetryPitchCmdFiltered = blendFloat(droneTelemetryPitchCmdFiltered, pitchCmd, 0.11f);
    droneTelemetryRollCmdFiltered = blendFloat(droneTelemetryRollCmdFiltered, rollCmd, 0.11f);
    droneTelemetryThrottleCmdFiltered = blendFloat(droneTelemetryThrottleCmdFiltered, throttleCmd, 0.10f);
    droneTelemetryStrafeCmdFiltered = blendFloat(droneTelemetryStrafeCmdFiltered, strafeCmd, 0.12f);
    droneTelemetryForwardCmdFiltered = blendFloat(droneTelemetryForwardCmdFiltered, forwardCmd, 0.12f);

    bool airborne = (droneTelemetryLift > 1.5f) || (droneTelemetryTargetLift > 1.5f);
    float semanticForward = -droneTelemetryForwardCmdFiltered;

    float yawStepDeg = 0.0f;
    if (airborne) {
      yawStepDeg = droneTelemetryYawCmdFiltered * 2.05f * dt * 60.0f;
      droneTelemetryYawDeg += yawStepDeg;
    }

    float targetPitchDeg = 0.0f;
    float targetRollDeg = 0.0f;
    if (airborne) {
      targetPitchDeg = clampValue((droneTelemetryPitchCmdFiltered + semanticForward * 1.18f) * 13.0f, -17.0f, 17.0f);
      targetRollDeg = clampValue((droneTelemetryRollCmdFiltered + droneTelemetryStrafeCmdFiltered * 1.18f) * 13.0f, -17.0f, 17.0f);
    }

    droneTelemetryPitchDeg = blendFloat(droneTelemetryPitchDeg, targetPitchDeg, 0.11f);
    droneTelemetryRollDeg = blendFloat(droneTelemetryRollDeg, targetRollDeg, 0.11f);

    float liftRate = 0.75f * dt * 60.0f;
    if (droneAutoTakeoffActive) {
      float targetLift = max(0.0f, droneAutoTargetAltitudeCm * 0.5f);
      droneTelemetryTargetLift = min(targetLift, droneTelemetryTargetLift + max(0.35f, droneTelemetryThrottleCmdFiltered) * liftRate);
      if (measuredAltCm >= (droneAutoTargetAltitudeCm - 6.0f)) {
        droneAutoTakeoffActive = false;
      }
    } else if (droneAutoLandActive) {
      droneTelemetryTargetLift = max(0.0f, droneTelemetryTargetLift - max(0.45f, droneTelemetryThrottleCmdFiltered) * liftRate);
      if (measuredAltCm <= 2.0f || droneTelemetryTargetLift <= 0.05f) {
        droneAutoLandActive = false;
        droneTelemetryTargetLift = 0.0f;
      }
    } else if (droneTelemetryThrottleCmdFiltered > 0.02f) {
      droneTelemetryTargetLift = max(0.0f, droneTelemetryTargetLift + droneTelemetryThrottleCmdFiltered * liftRate);
    } else if (droneTelemetryThrottleCmdFiltered < -0.02f) {
      droneTelemetryTargetLift = max(0.0f, droneTelemetryTargetLift + droneTelemetryThrottleCmdFiltered * liftRate);
    }

    float liftAlpha = (droneTelemetryTargetLift >= droneTelemetryLift) ? 0.12f : 0.10f;
    droneTelemetryLift = blendFloat(droneTelemetryLift, droneTelemetryTargetLift, liftAlpha);
    if (droneTelemetryLift < 0.05f && droneTelemetryTargetLift < 0.05f) {
      droneTelemetryLift = 0.0f;
      droneTelemetryTargetLift = 0.0f;
    }

    if (airborne) {
      float motionYawDeg = droneTelemetryYawDeg - yawStepDeg * 0.5f;
      float motionYawRad = radians(motionYawDeg);
      float forwardSpeed = -semanticForward * 2.9f * dt * 60.0f;
      float strafeSpeed = droneTelemetryStrafeCmdFiltered * 2.5f * dt * 60.0f;
      float planarTraction = 1.0f - 0.18f * clampValue((float)fabs(droneTelemetryYawCmdFiltered), 0.0f, 1.0f);
      forwardSpeed *= planarTraction;
      strafeSpeed *= planarTraction;

      droneTelemetryX += sin(motionYawRad) * forwardSpeed + sin(motionYawRad + HALF_PI) * strafeSpeed;
      droneTelemetryZ += cos(motionYawRad) * forwardSpeed + cos(motionYawRad + HALF_PI) * strafeSpeed;
    }

    droneTelemetryAltitudeCm = max(droneAltitudeFromGroundCm, max(0.0f, droneTelemetryLift * 2.0f));
    droneTelemetryY = droneTelemetryAltitudeCm * 0.5f;
  }
}

// Parses stepper drive mode value.
bool parseStepperDriveModeValue(const char* value, StepperDriveMode* outMode) {
  if (value == NULL || outMode == NULL) return false;

  if (strcmp(value, "0") == 0 || strcmp(value, "FULLSTEP_BIPOLAR") == 0 || strcmp(value, "FULLBIPOLAR") == 0) {
    *outMode = STEPPER_MODE_FULLSTEP_BIPOLAR;
    return true;
  }
  if (strcmp(value, "1") == 0 || strcmp(value, "HALFSTEP_BIPOLAR") == 0 || strcmp(value, "HALFBIPOLAR") == 0) {
    *outMode = STEPPER_MODE_HALFSTEP_BIPOLAR;
    return true;
  }
  if (strcmp(value, "2") == 0 || strcmp(value, "FULLSTEP_UNIPOLAR") == 0 || strcmp(value, "FULLUNIPOLAR") == 0) {
    *outMode = STEPPER_MODE_FULLSTEP_UNIPOLAR;
    return true;
  }
  if (strcmp(value, "3") == 0 || strcmp(value, "HALFSTEP_UNIPOLAR") == 0 || strcmp(value, "HALFUNIPOLAR") == 0) {
    *outMode = STEPPER_MODE_HALFSTEP_UNIPOLAR;
    return true;
  }
  return false;
}


// Parses channel 0 drive mode value.
bool parseCh0DriveModeValue(const char* value, Ch0DriveMode* outMode) {
  if (value == NULL || outMode == NULL) return false;
  if (strcmp(value, "0") == 0 || strcmp(value, "STEPPER") == 0 || strcmp(value, "STEPPER_MOTOR") == 0) {
    *outMode = CH0_MODE_STEPPER;
    return true;
  }
  if (strcmp(value, "1") == 0 || strcmp(value, "DC") == 0 || strcmp(value, "DC_MOTOR") == 0) {
    *outMode = CH0_MODE_DC_MOTOR;
    return true;
  }
  return false;
}

// Sets channel 0 drive mode.
void setCh0DriveMode(Ch0DriveMode mode, bool announce) {
  ch0DriveMode = sanitizeCh0DriveMode((uint8_t)mode);
  configureCh0PinsForCurrentMode();
  releaseStepperCoils();
  stepperSetDriverPWM(0);
  if (announce) {
    Serial.print(F("CH0MODE="));
    Serial.print((int)ch0DriveMode);
    Serial.print(F(" ["));
    Serial.print(getCh0DriveModeName(ch0DriveMode));
    Serial.println(F("]"));
  }
}

// Sets stepper drive mode.
void setStepperDriveMode(StepperDriveMode mode, bool announce) {
  stepperDriveMode = sanitizeStepperDriveMode((uint8_t)mode);
  int8_t phaseCount = (int8_t)getStepperPhaseCount();
  if ((int8_t)stepper.seq >= phaseCount) {
    stepper.seq = PHASE_0;
  }
  applyStepPhase(stepper.seq);
  if (announce) {
    Serial.print(F("STEPMODE="));
    Serial.print((int)stepperDriveMode);
    Serial.print(F(" ["));
    Serial.print(getStepperDriveModeName(stepperDriveMode));
    Serial.println(F("]"));
  }
}

// Returns stepper mode attempt by index.
StepperDriveMode getStepperModeAttemptByIndex(StepperDriveMode baseMode, uint8_t attemptIndex) {
  const StepperDriveMode order[4] = {
    baseMode,
    STEPPER_MODE_HALFSTEP_UNIPOLAR,
    STEPPER_MODE_HALFSTEP_BIPOLAR,
    STEPPER_MODE_FULLSTEP_UNIPOLAR
  };

  if (baseMode == STEPPER_MODE_FULLSTEP_BIPOLAR) {
    const StepperDriveMode alt[4] = {
      STEPPER_MODE_FULLSTEP_BIPOLAR,
      STEPPER_MODE_HALFSTEP_UNIPOLAR,
      STEPPER_MODE_HALFSTEP_BIPOLAR,
      STEPPER_MODE_FULLSTEP_UNIPOLAR
    };
    if (attemptIndex < 4) return alt[attemptIndex];
    return alt[3];
  }

  StepperDriveMode unique[4];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 4; i++) {
    bool exists = false;
    for (uint8_t j = 0; j < n; j++) {
      if (unique[j] == order[i]) {
        exists = true;
        break;
      }
    }
    if (!exists) unique[n++] = order[i];
  }
  while (n < 4) unique[n++] = STEPPER_MODE_HALFSTEP_UNIPOLAR;
  if (attemptIndex < 4) return unique[attemptIndex];
  return unique[3];
}

// Utility: auto stepper advance mode attempt.
bool autoStepperAdvanceModeAttempt() {
  if (autoStepper.modeAttemptIndex >= 3) return false;

  autoStepper.modeAttemptIndex++;
  StepperDriveMode nextMode = getStepperModeAttemptByIndex(autoStepper.originalMode, autoStepper.modeAttemptIndex);

  setStepperDriveMode(nextMode, true);
  autoStepperApplyCandidate(autoStepper.seed);

  if (!collisionRuntimeEnabled) {
    collisionFlag = false;
  }

  readStepperAngle();
  stepper.targetAngle = stepper.currentAngle;
  servos.target[0] = stepper.currentAngle;
  servos.actual[0] = stepper.currentAngle;
  resetStepperPID();

  autoStepper.active = true;
  autoStepper.stopRequested = false;
  autoStepper.cachedStage = AUTO_STAGE_IDLE;
  memset(autoStepper.candidates, 0, sizeof(autoStepper.candidates));
  rt.stepperArmed = true;
  rt.servoOutputsArmed = false;

  Serial.print(F("AUTOSTEPPER RETRY STEPMODE="));
  Serial.print((int)stepperDriveMode);
  Serial.print(F(" ["));
  Serial.print(getStepperDriveModeName(stepperDriveMode));
  Serial.println(F("]"));

  autoStepperPrepareStage(AUTO_STAGE_TORQUE);
  return true;
}

