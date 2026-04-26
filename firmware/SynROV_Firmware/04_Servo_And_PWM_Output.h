// =====================================================================
// SynROV Firmware - Servo and PWM outputs
// ---------------------------------------------------------------------
// Purpose:
//   PCA9685 mapping, fallback servo control, smooth output updates and
//   output safety logic.
// =====================================================================

// Utility: angle to PCA ticks.
uint16_t angleToPcaTicks(uint8_t ch, float angleDeg) {
  float chMax = getChannelInputMax(ch);
  float angle = clampValue(angleDeg, SERVO_INPUT_MIN, chMax);
  float pulseRange = (float)SERVO_MAX_PULSE - (float)SERVO_MIN_PULSE;
  float pulse = (float)SERVO_MIN_PULSE + pulseRange * (angle / chMax);
  return (uint16_t)clampValue((int)pulse, 0, 4095);
}

// Utility: angle to fallback servo micros.
uint16_t angleToFallbackServoMicros(uint8_t ch, float angleDeg) {
  uint16_t ticks = angleToPcaTicks(ch, angleDeg);
  unsigned long microsValue = ((unsigned long)ticks * 20000UL + 2048UL) / 4096UL;
  return (uint16_t)clampValue((int)microsValue, 500, 2500);
}

// Forces servo neutral output.
void forceServoNeutralOutput(uint8_t ch) {
  if (!isServoChannel(ch)) return;

  if (rt.pwmReady) {
    // Neutral must release the servo output instead of forcing it to 90 degrees.
    // Keeping a continuous center command makes some joints buzz, hunt and heat up
    // even when that member is intentionally uncontrolled.
    pwm.setPWM(ch, 0, 4096);
  } else {
    uint8_t idx = ch - FIRST_SERVO_CH;
    if (isFallbackServoAttached(idx)) {
      fallbackServos[idx].detach();
      setFallbackServoAttached(idx, false);
    }
  }
}

// Writes channel.
void writeChannel(uint8_t ch, float value) {
  if (ch >= MAX_CHANNELS) return;

  if (getServoNeutral(ch) && isServoChannel(ch)) {
    if (getServoWriteValid(ch)) {
      setServoWriteValid(ch, false);
      forceServoNeutralOutput(ch);
    }
    return;
  }

  if (isPwmChannel(ch)) {
    uint8_t percent = (uint8_t)clampValue((int)value, 0, 100);
    if (rt.pwmReady) {
      uint16_t pwmValue = map(percent, 0, 100, 0, 4095);
      pwm.setPWM(ch, 0, pwmValue);
    } else {
      uint8_t idx = ch - FIRST_PWM_CH;
      if (idx < 4) {
        uint8_t duty = map(percent, 0, 100, 0, 255);
        analogWrite(FALLBACK_PWM_PINS[idx], duty);
      }
    }
    servos.actual[ch] = percent;
    servos.target[ch] = percent;
    return;
  }

  if (!isServoChannel(ch)) return;

  float angle = clampServoInputToChannelRange(ch, value);
  angle *= getServoSpan(ch);
  angle += servos.offset[ch];

  if (servos.direction[ch] == -1) {
    angle = getChannelInputMax(ch) - angle;
  }

  angle = clampValue(angle, SERVO_INPUT_MIN, getChannelInputMax(ch));

  if (getServoWriteValid(ch) && fabsf(angle - servoLastWrittenDeg[ch]) < servoWriteEpsilonDeg) {
    return;
  }

  if (rt.pwmReady) {
    pwm.setPWM(ch, 0, angleToPcaTicks(ch, angle));
  } else {
    uint8_t idx = ch - FIRST_SERVO_CH;
    if (idx < 11) {
      if (!isFallbackServoAttached(idx)) {
        fallbackServos[idx].attach(FALLBACK_SERVO_PINS[idx]);
        setFallbackServoAttached(idx, true);
      }
      fallbackServos[idx].writeMicroseconds((int)angleToFallbackServoMicros(ch, angle));
    }
  }
  servoLastWrittenDeg[ch] = angle;
  setServoWriteValid(ch, true);
}

// Updates servos smooth.
void updateServosSmooth() {
  if (robotControlMode != ROBOT_MODE_MANIPULATOR) return;

  unsigned long nowMs = millis();
  if (!rt.servoOutputsArmed) {
    servoPlannerLastUpdateMs = nowMs;
    return;
  }

  unsigned long elapsedMs = SERVO_UPDATE_MS;
  if (servoPlannerLastUpdateMs != 0 && nowMs >= servoPlannerLastUpdateMs) {
    elapsedMs = nowMs - servoPlannerLastUpdateMs;
  }
  servoPlannerLastUpdateMs = nowMs;

  elapsedMs = clampValue<unsigned long>(elapsedMs, 5UL, 45UL);
  float cycleScale = clampValue((float)elapsedMs / (float)SERVO_UPDATE_MS, 0.35f, 3.0f);
  float scaledMoveStepDeg = max(0.01f, servoMoveStepDeg * cycleScale);
  float scaledMinAppliedDeg = max(0.01f, 0.02f * cycleScale);

  for (uint8_t ch = FIRST_SERVO_CH; ch <= LAST_SERVO_CH; ch++) {
    if (getServoNeutral(ch)) {
      if (getServoWriteValid(ch)) {
        setServoWriteValid(ch, false);
        forceServoNeutralOutput(ch);
      }
      continue;
    }

    float target = clampServoTargetToLimits(ch, clampServoInputToChannelRange(ch, servos.target[ch]));
    servos.target[ch] = target;

    float deltaToTarget = target - servos.actual[ch];
    if (fabsf(deltaToTarget) <= servoHoldDeadbandDeg) {
      servos.actual[ch] = target;
    } else if (getServoRamp(ch)) {
      float delta = deltaToTarget;
      if (delta > scaledMoveStepDeg) delta = scaledMoveStepDeg;
      if (delta < -scaledMoveStepDeg) delta = -scaledMoveStepDeg;

      float applied = delta * servoRampSmoothing;
      if (fabsf(delta) > 0.0001f && fabsf(applied) < scaledMinAppliedDeg) {
        applied = (delta > 0.0f) ? scaledMinAppliedDeg : -scaledMinAppliedDeg;
      }
      if (fabsf(applied) > fabsf(delta)) applied = delta;

      servos.actual[ch] += applied;
      if ((deltaToTarget > 0.0f && servos.actual[ch] > target) ||
          (deltaToTarget < 0.0f && servos.actual[ch] < target)) {
        servos.actual[ch] = target;
      }
    } else {
      servos.actual[ch] = target;
    }

    // PWM channels CH12..CH15 are manual outputs. Do not change them from joint motion.
    writeChannel(ch, servos.actual[ch]);
  }

  applyCollisionGuard();
}

// Applies boot PWM defaults.
void applyBootPwmDefaults() {
  // CH12..CH15 always start at the neutral 50% manual PWM value.
  setAllPwmBootPercentToNeutral();
  for (uint8_t ch = FIRST_PWM_CH; ch <= LAST_PWM_CH; ch++) {
    uint8_t idx = ch - FIRST_PWM_CH;
    uint8_t percent = clampValue(pwmBootPercent[idx], (uint8_t)0, (uint8_t)100);
    if (rt.pwmReady) {
      uint16_t pwmValue = map(percent, 0, 100, 0, 4095);
      pwm.setPWM(ch, 0, pwmValue);
    } else {
      uint8_t duty = map(percent, 0, 100, 0, 255);
      analogWrite(FALLBACK_PWM_PINS[idx], duty);
    }
    servos.actual[ch] = percent;
    servos.target[ch] = percent;
  }
}


// Applies safe startup pose.
void applySafeStartupPose() {
  float safeBaseDeg = clampValue(getManipulatorMemberStartupSafeDeg(MANIP_MEMBER_BASE), STEPPER_INPUT_MIN, STEPPER_INPUT_MAX);
  uint8_t baseCh = getManipulatorMemberChannel(MANIP_MEMBER_BASE);
  if (baseCh == 0) {
    servos.target[0] = safeBaseDeg;
    servos.actual[0] = safeBaseDeg;
    stepper.targetAngle = safeBaseDeg;
    stepper.currentAngle = safeBaseDeg;
    setServoWriteValid(0, false);
  } else if (isServoChannel(baseCh)) {
    float seededBaseDeg = clampServoTargetToLimits(baseCh, clampServoInputToChannelRange(baseCh, safeBaseDeg));
    servos.target[baseCh] = seededBaseDeg;
    servos.actual[baseCh] = seededBaseDeg;
    setServoWriteValid(baseCh, false);
  }

  const uint8_t members[6] = {
    MANIP_MEMBER_UPPER,
    MANIP_MEMBER_FORE,
    MANIP_MEMBER_FOREARM_ROLL,
    MANIP_MEMBER_WRIST_PITCH,
    MANIP_MEMBER_WRIST_ROLL,
    MANIP_MEMBER_GRIPPER
  };

  for (uint8_t i = 0; i < 6; i++) {
    uint8_t member = members[i];
    uint8_t ch = getManipulatorMemberChannel(member);
    if (!isServoChannel(ch)) continue;
    float safeDeg = clampManipulatorMemberTargetToLimits(member, getManipulatorMemberStartupSafeDeg(member));
    servos.target[ch] = safeDeg;
    servos.actual[ch] = safeDeg;
    setServoWriteValid(ch, false);
  }
}

// Ensures safe startup pose if colliding.
void ensureSafeStartupPoseIfColliding() {
  float base = getManipulatorMemberActualServoDeg(MANIP_MEMBER_BASE);
  float upper = getManipulatorMemberTargetServoDeg(MANIP_MEMBER_UPPER);
  float fore = getManipulatorMemberTargetServoDeg(MANIP_MEMBER_FORE);
  float forearmRoll = getManipulatorMemberTargetServoDeg(MANIP_MEMBER_FOREARM_ROLL);
  float wristPitch = getManipulatorMemberTargetServoDeg(MANIP_MEMBER_WRIST_PITCH);
  float wristRot = getManipulatorMemberTargetServoDeg(MANIP_MEMBER_WRIST_ROLL);
  float grip = getManipulatorMemberTargetServoDeg(MANIP_MEMBER_GRIPPER);

  if (poseHasCollision(base, upper, fore, forearmRoll, wristPitch, wristRot, grip)) {
    applySafeStartupPose();
    collisionFlag = false;
    Serial.println(F("SAFE STARTUP POSE APPLIED"));
  }
}

