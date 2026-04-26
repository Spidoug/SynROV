// =====================================================================
// SynROV Processing - Core logic
// ---------------------------------------------------------------------
// Purpose:
//   Robot behavior coordination, state transitions and shared logic.
// =====================================================================


final int SERIAL_ASCII_LINE_MAX = 4096;
final int SERIAL_ASCII_RESYNC_GRACE_MS = 1200;
boolean mainSerialAsciiDiscarding = false;
boolean calibrationSerialAsciiDiscarding = false;
int mainSerialAsciiOverflowCount = 0;
int calibrationSerialAsciiOverflowCount = 0;
long mainSerialAsciiResyncUntilMs = 0;
long calibrationSerialAsciiResyncUntilMs = 0;

final String[] SERIAL_ASCII_FRAME_MARKERS = {
  "#TEL|", "TEL|", "#SENS|", "SENS|", "#CMD|",
  "READY!", "ROBOT=", "ROBOT:", "ROBOTTYPE=", "ROBOTTYPE:", "MODE=", "MODE:",
  "SYNROV", "============================================================"
};

final String[] SERIAL_ASCII_CFG_KEYS = {
  "STATIC:", "GEOMETRY_UNITS:", "ARMPOS:", "ARMSZ:", "ARMMAP:", "ARMOFF:",
  "BASESRC:", "ARMLIM:", "GSTABCFG:", "MAGCALCFG:", "PRESS:", "STEPNEUTRAL:",
  "COLLGUARD:", "MANIPTM:", "RXIDLE:", "SOFTSTOP:", "BASEHOME:", "ENABLE:", "PING:",
  "SENSINT:", "ROBOT:", "ROBOTTYPE:", "MODE:"
};

final String[] SERIAL_ASCII_SENS_KEYS = {
  "SEQ:", "STAMP:", "MPU1:", "MPU2:", "GYR1:", "GYR2:", "AN:", "AN1:", "AN2:", "AN3:", "AN4:", "AN5:", "AN6:", "PWM:", "PRESS:",
  "ARMANG:", "ARMGEO:", "ARMJ:", "MAG:", "MAGCAL_ACTIVE:", "MAGCAL_PROGRESS:",
  "SONAR:", "INA1:", "INA2:", "STEP:", "TGT:", "SPD:", "PWMRAW:", "WIN:",
  "GSTAB:", "ASTAB:", "COLL_EN:", "COLLEN:", "COLLISION_ENABLED:", "COLL:",
  "EX1:", "EX2:", "ROBOT:", "ROBOTTYPE:", "ROBOT_INDEX:", "MODE:"
};

boolean isAsciiTelemetryLine(String raw) {
  if (raw == null) return false;
  return raw.startsWith("#TEL|") || raw.startsWith("TEL|") || raw.startsWith("#SENS|") || raw.startsWith("SENS|");
}

int telemetryAsciiPrefixLength(String raw) {
  if (raw == null) return 0;
  if (raw.startsWith("#TEL|")) return 5;
  if (raw.startsWith("TEL|")) return 4;
  if (raw.startsWith("#SENS|")) return 6;
  if (raw.startsWith("SENS|")) return 5;
  return 0;
}


// Returns the first block separator index (: or =).
int findTelemetryBlockSeparator(String block) {
  if (block == null) return -1;
  int colon = block.indexOf(':');
  int equals = block.indexOf('=');
  if (colon < 0) return equals;
  if (equals < 0) return colon;
  return min(colon, equals);
}

// Returns telemetry block key.
String telemetryBlockKey(String block) {
  int sep = findTelemetryBlockSeparator(block);
  if (sep <= 0) return "";
  return trim(block.substring(0, sep)).toUpperCase();
}

// Returns telemetry block value.
String telemetryBlockValue(String block) {
  int sep = findTelemetryBlockSeparator(block);
  if (sep < 0 || sep + 1 > block.length()) return "";
  return trim(block.substring(sep + 1));
}

// Applies manipulator limits CSV to a sensor snapshot.
void applyManipulatorArmLimitsCsvToSnapshot(JSONObject sens, String value) {
  if (sens == null || value == null) return;
  String[] v = split(value, ',');
  for (int i = 0, joint = 0; i + 1 < v.length && joint < 7; i += 2, joint++) {
    setNumberOrText(sens, "armlim_" + joint + "_min", trim(v[i]));
    setNumberOrText(sens, "armlim_" + joint + "_max", trim(v[i + 1]));
  }
}

// Resets pending manipulator pose command.
void resetPendingManipulatorPoseCommand() {
  manipulatorPoseDirty = false;
  manipulatorPoseCommandSource = CONTROL_SOURCE_LOCAL;
  lastManipulatorPoseSendMs = 0;
  manipulatorTelemetrySuppressUntilMs = 0;
  for (int i = 0; i < lastSentManipulatorPose.length && i < angles.length; i++) {
    lastSentManipulatorPose[i] = angles[i];
  }
  for (int i = 0; i < lastSentManipulatorNeutralMembers.length && i < manipulatorNeutralMembers.length; i++) {
    lastSentManipulatorNeutralMembers[i] = manipulatorNeutralMembers[i];
  }
}

// Checks whether extended firmware telemetry transition should be used.
boolean shouldUseExtendedFirmwareTelemetryTransition() {
  if (!(isManipulatorSelected && systemReady && !simulationMode && myPort != null)) return false;
  return collisionSet || firmwareCollisionRuntimeEnabled || remoteCollisionEnabled || isFirmwareCollisionGuardEnabled();
}

// Returns effective manipulator telemetry suppress milliseconds.
int effectiveManipulatorTelemetrySuppressMs() {
  int suppressMs = max(0, manipulatorTelemetrySuppressMs);
  if (shouldUseExtendedFirmwareTelemetryTransition()) suppressMs = max(suppressMs, 1000);
  return suppressMs;
}

// Utility: queue manipulator pose command.
void queueManipulatorPoseCommand() {
  manipulatorPoseDirty = true;
  manipulatorPoseCommandSource = commandContextSource;
  manipulatorTelemetrySuppressUntilMs = millis() + effectiveManipulatorTelemetrySuppressMs();
}

// Marks manipulator pose dirty.
void markManipulatorPoseDirty() {
  queueManipulatorPoseCommand();
}

// Returns manipulator duty command percentage.
int getManipulatorDutyCommandPercentage(int idx) {
  if (idx < 0 || idx >= dutyTargetPercentages.length) return 0;
  if (dutyTargetsFollowHardware && lastHardwareSensorMillis > 0) {
    return constrain(dutyPercentages[idx], 0, 100);
  }
  return constrain(dutyTargetPercentages[idx], 0, 100);
}

// Builds manipulator pose command.
String buildManipulatorPoseCommand() {
  // Unified manipulator runtime packet:
  // ARMJ=j0,j1,j2,j3,j4,j5,j6,p12,p13,p14,p15
  // The first 7 values are joint targets in manipulator member space and
  // the last 4 values are the auxiliary PWM percentages for channels 12..15.
  // While the diagnostics sliders are still mirroring the hardware after a
  // fresh connection, the packet preserves the live PWM that already exists in
  // the robot instead of pushing the last saved desktop value back into it.
  // A joint value of 1000 means neutral for that joint.
  int baseMember = manipulatorNeutralMembers[BASE_IDX] ? 1000 : round(normalizeAbsoluteAngleDeg(baseServoToForwardYawDeg(angles[BASE_IDX])));
  int upperMember = manipulatorNeutralMembers[UPPERARM_IDX] ? 1000 : round(upperServoToGroundDeg(angles[UPPERARM_IDX]));
  int foreMember = manipulatorNeutralMembers[FOREARM_IDX] ? 1000 : round(forearmServoToDeltaDeg(angles[FOREARM_IDX]));
  int forearmRollMember = manipulatorNeutralMembers[FOREARM_ROLL_IDX] ? 1000 : round(rollServoToDeltaDeg(angles[FOREARM_ROLL_IDX]));
  int wristPitchMember = manipulatorNeutralMembers[WRIST_VERT_IDX] ? 1000 : round(wristServoToDeltaDeg(angles[WRIST_VERT_IDX]));
  int wristRollMember = manipulatorNeutralMembers[WRIST_ROT_IDX] ? 1000 : round(rollServoToDeltaDeg(angles[WRIST_ROT_IDX]));
  int gripMember = manipulatorNeutralMembers[GRIPPER_IDX] ? 1000 : angles[GRIPPER_IDX];

  return "ARMJ=" +
    baseMember + "," +
    upperMember + "," +
    foreMember + "," +
    forearmRollMember + "," +
    wristPitchMember + "," +
    wristRollMember + "," +
    gripMember + "," +
    getManipulatorDutyCommandPercentage(0) + "," +
    getManipulatorDutyCommandPercentage(1) + "," +
    getManipulatorDutyCommandPercentage(2) + "," +
    getManipulatorDutyCommandPercentage(3);
}

// Utility: manipulator pose matches last sent.
boolean manipulatorPoseMatchesLastSent() {
  for (int i = 0; i < lastSentManipulatorPose.length && i < angles.length; i++) {
    if (lastSentManipulatorPose[i] != angles[i]) return false;
  }
  for (int i = 0; i < lastSentManipulatorNeutralMembers.length && i < manipulatorNeutralMembers.length; i++) {
    if (lastSentManipulatorNeutralMembers[i] != manipulatorNeutralMembers[i]) return false;
  }
  return true;
}

// Marks manipulator pose sent.
void markManipulatorPoseSent() {
  for (int i = 0; i < lastSentManipulatorPose.length && i < angles.length; i++) {
    lastSentManipulatorPose[i] = angles[i];
  }
  for (int i = 0; i < lastSentManipulatorNeutralMembers.length && i < manipulatorNeutralMembers.length; i++) {
    lastSentManipulatorNeutralMembers[i] = manipulatorNeutralMembers[i];
  }
  lastManipulatorPoseSendMs = millis();
  manipulatorPoseDirty = false;
}

// Utility: flush pending manipulator pose command.
void flushPendingManipulatorPoseCommand(boolean force) {
  if (!(systemReady && !simulationMode) || myPort == null) return;
  if (!isManipulatorSelected) return;
  if (pendingHardwarePoseSync) {
    if (lastHardwareSensorMillis > 0) {
      boolean poseWasDirty = manipulatorPoseDirty;
      int poseSource = manipulatorPoseCommandSource;
      syncManipulatorLocalPoseToHardware(true);
      if (poseWasDirty) {
        manipulatorPoseDirty = true;
        manipulatorPoseCommandSource = poseSource;
      }
    } else if (!force && !manipulatorPoseDirty) {
      return;
    }
  }

  long now = millis();
  boolean firmwareAuthority = isManipulatorFirmwareCollisionAuthorityActive();
  boolean resendPending = !firmwareAuthority &&
    !manipulatorPoseDirty &&
    manipulatorPoseResendIntervalMs > 0 &&
    lastHardwareSensorMillis > 0 &&
    !manipulatorTelemetryCloseToCommandedPose(3) &&
    (now - lastManipulatorPoseSendMs) >= manipulatorPoseResendIntervalMs;

  // Only transmit when there is an explicit local target update, a forced send,
  // or a periodic re-send while telemetry shows the arm has not yet reached the
  // commanded pose. Runtime telemetry can legitimately differ from the last
  // commanded pose while the arm is still traveling; treating that delta as a new
  // target causes the software to keep chasing the measured pose and re-send
  // intermediate positions, which looks like oscillation or bouncing before
  // settling.
  if (!manipulatorPoseDirty && !force && !resendPending) return;

  if (!force) {
    int minInterval = manipulatorPoseDirty ? manipulatorPoseSendIntervalMs : manipulatorPoseResendIntervalMs;
    if (firmwareAuthority) {
      // Smooth command cadence while the firmware owns collision
      // handling. This reduces UART pressure without changing payload
      // resolution, and it stays constant even during collisions.
      minInterval = 100;
    }
    if (minInterval < 0) minInterval = 0;
    if ((now - lastManipulatorPoseSendMs) < minInterval) return;
  }

  String runtimeCommand = buildManipulatorPoseCommand();
  int previousSource = commandContextSource;
  setCommandContext(manipulatorPoseCommandSource);
  try {
    if (!sendHardwareStreamCommand(runtimeCommand, "manipulator pose", false)) return;
  }
  finally {
    commandContextSource = previousSource;
  }
  markManipulatorPoseSent();
}


// Checks whether manipulator member neutral.
boolean isManipulatorMemberNeutral(int memberIdx) {
  if (memberIdx < 0 || memberIdx >= manipulatorNeutralMembers.length) return false;
  return manipulatorNeutralMembers[memberIdx];
}

// Sets manipulator member neutral.
void setManipulatorMemberNeutral(int memberIdx, boolean neutral) {
  if (memberIdx < 0 || memberIdx >= manipulatorNeutralMembers.length) return;
  if (manipulatorNeutralMembers[memberIdx] == neutral) return;

  // A neutral button press must count as a real manipulator command. Otherwise
  // the auto-home that gets scheduled right after connect can immediately
  // override the new neutral state, which makes the buttons appear dead until a
  // first motion command cancels that pending homing cycle.
  if (!homingStepActive) {
    cancelPendingAutoHome();
    goingHome = false;
  }

  // Right after connect the desktop pose can still be waiting for the first
  // telemetry sync. If that telemetry is already available, pull it in before
  // composing the first ARMJ packet so a neutral toggle works immediately
  // without requiring an initial movement command.
  if (systemReady && myPort != null && pendingHardwarePoseSync && lastHardwareSensorMillis > 0) {
    syncManipulatorLocalPoseToHardware(true);
  }

  if (systemReady && myPort != null && !isManipulatorSelected) {
    selectMode("Manipulator", false);
  }

  manipulatorNeutralMembers[memberIdx] = neutral;
  queueManipulatorPoseCommand();
  if (systemReady && myPort != null && isManipulatorSelected && commandContextSource != CONTROL_SOURCE_WEB) {
    flushPendingManipulatorPoseCommand(true);
  }
}

// Utility: toggle manipulator member neutral.
void toggleManipulatorMemberNeutral(int memberIdx) {
  setManipulatorMemberNeutral(memberIdx, !isManipulatorMemberNeutral(memberIdx));
}

// Clears manipulator member neutral.
void clearManipulatorMemberNeutral(int memberIdx) {
  setManipulatorMemberNeutral(memberIdx, false);
}

// Utility: normalize wrapped 360.
int normalizeWrapped360(int value) {
  int wrapped = value % 360;
  if (wrapped < 0) wrapped += 360;
  return wrapped;
}

// Utility: normalize manipulator target value.
int normalizeManipulatorTargetValue(int servoIndex, int value) {
  if (servoIndex < 0 || servoIndex >= servoLimits.length) return value;

  // The manipulator base must behave like a hard 0..359 degree joint,
  // not like a circular angle that wraps 359 -> 0.
  // This prevents the base from completing a full turn and instantly
  // reappearing at 0 degrees.
  return constrain(value, servoLimits[servoIndex][0], servoLimits[servoIndex][1]);
}

// Utility: shortest base delta degrees.
int shortestBaseDeltaDeg(int current, int target) {
  int boundedCurrent = normalizeManipulatorTargetValue(BASE_IDX, current);
  int boundedTarget = normalizeManipulatorTargetValue(BASE_IDX, target);
  return boundedTarget - boundedCurrent;
}

// Utility: smooth manipulator target.
int smoothManipulatorTarget(int servoIndex, int current, int target, float factor) {
  factor = constrain(factor, 0.0f, 1.0f);
  target = normalizeManipulatorTargetValue(servoIndex, target);
  return normalizeManipulatorTargetValue(servoIndex, round(current * (1.0f - factor) + target * factor));
}

// Utility: ensure manipulator pose synced before first command.
boolean ensureManipulatorPoseSynchronizedBeforeCommand(int[] requestedAngles, boolean[] activeMask, int manipCount) {
  if (!(systemReady && myPort != null)) return true;
  if (!pendingHardwarePoseSync) return true;

  if (lastHardwareSensorMillis <= 0) {
    updateMessage(tr("Aguardando a primeira telemetria do manipulador antes de enviar comando do braço...", "Waiting for first manipulator telemetry before sending arm command..."));
    return false;
  }

  syncManipulatorLocalPoseToHardware(true);

  if (requestedAngles != null && activeMask != null) {
    for (int i = 0; i < manipCount && i < requestedAngles.length && i < activeMask.length && i < angles.length; i++) {
      if (!activeMask[i]) {
        requestedAngles[i] = angles[i];
      }
    }
  }
  return true;
}

// Applies manipulator pose targets.
boolean applyManipulatorPoseTargets(int[] requestedAngles, boolean[] activeMask) {
  if (!isManipulatorSelected) return false;
  if (!canAcceptCommandSource(commandContextSource, true)) return false;
  if (requestedAngles == null || activeMask == null) return false;

  int manipCount = min(GRIPPER_IDX + 1, min(requestedAngles.length, activeMask.length));
  if (manipCount <= 0) return false;
  if (!ensureManipulatorPoseSynchronizedBeforeCommand(requestedAngles, activeMask, manipCount)) return false;

  int[] previousAngles = angles.clone();
  boolean[] previousNeutralMembers = manipulatorNeutralMembers.clone();
  boolean changed = false;
  boolean[] changedMask = new boolean[GRIPPER_IDX + 1];
  boolean preserveNeutralOverrides = commandContextSource != CONTROL_SOURCE_LOCAL;

  for (int i = 0; i < manipCount; i++) {
    if (!activeMask[i]) continue;
    if (manipulatorNeutralMembers[i] && preserveNeutralOverrides) {
      continue;
    }

    int normalizedTarget = normalizeManipulatorTargetValue(i, requestedAngles[i]);
    if (manipulatorNeutralMembers[i]) {
      manipulatorNeutralMembers[i] = false;
      changed = true;
      changedMask[i] = true;
    }
    if (angles[i] != normalizedTarget) {
      angles[i] = normalizedTarget;
      changed = true;
      changedMask[i] = true;
    }
  }

  if (!changed) return false;

  boolean blocked = false;
  boolean blockedBySelfCollision = false;
  boolean blockedByEnvironmentCollision = false;
  boolean localCollisionFilteringActive = isManipulatorLocalCollisionGuardActive() && !isManipulatorFirmwareCollisionAuthorityActive();
  if (localCollisionFilteringActive) {
    if (evaluateManipulatorSelfCollisionForLocalPose()) {
      blocked = true;
      blockedBySelfCollision = true;
    }
  }

  if (!blocked && localCollisionFilteringActive && environmentCollisionEnabled) {
    if (evaluateManipulatorEnvironmentCollisionForLocalPose()) {
      blocked = true;
      blockedByEnvironmentCollision = true;
    }
  }

  if (blocked) {
    arrayCopy(previousAngles, angles);
    arrayCopy(previousNeutralMembers, manipulatorNeutralMembers);
    if (blockedByEnvironmentCollision) {
    } else if (blockedBySelfCollision) {
    }
    return false;
  }

  if (!homingStepActive) {
    cancelPendingAutoHome();
    goingHome = false;
  }

  if (systemReady && myPort != null) {
    if (!isManipulatorSelected) {
      selectMode("Manipulator");
    }
    queueManipulatorPoseCommand();
  }

  for (int i = 0; i < manipCount; i++) {
    if (changedMask[i]) {
      sendServoStatePacket(mapIndexToChannel(i), angles[i]);
    }
  }
  lastServoCommandTime = millis();
  return true;
}


// Utility: move servo.
void moveServo(int channel, int angle) {
  if (!homingStepActive) {
    cancelPendingAutoHome();
    goingHome = false;
  }

  if (systemReady && myPort != null) {
    if (!isManipulatorSelected) {
      selectMode("Manipulator");
    }
    queueManipulatorPoseCommand();
  }
  sendServoStatePacket(channel, angle);
}

// Sets angle.
void setAngle(int channel, int value) {
  int servoIndex = mapServoIndex(channel);
  if (servoIndex == -1) {
    updateMessage(tr("Canal de servo inválido ", "Invalid servo channel ") + channel);
    return;
  }

  int[] requestedAngles = angles.clone();
  boolean[] activeMask = new boolean[GRIPPER_IDX + 1];
  requestedAngles[servoIndex] = value;
  activeMask[servoIndex] = true;
  applyManipulatorPoseTargets(requestedAngles, activeMask);
}

// Sets duty.
void setDuty(int channel, int percentage) {
  if (!canAcceptCommandSource(commandContextSource, true)) return;
  if (channel < 12 || channel > 15) {
    updateMessage(tr("Canal de duty inválido ", "Invalid duty channel ") + channel + tr(" (esperado 12-15)", " (expected 12-15)"));
    return;
  }

  percentage = constrain(percentage, 0, 100);
  int idx = getDutyIndex(channel);
  if (idx >= 0) {
    if (manipulatorAutoPwmPowerControlEnabled()) {
      manipulatorAutoPwmPowerControl = false;
    }
    dutyTargetsInitialized = true;
    dutyTargetsFollowHardware = false;
    dutyTargetPercentages[idx] = percentage;
    if (lastHardwareSensorMillis <= 0) {
      dutyPercentages[idx] = percentage;
    }
  }

  if (isManipulatorSelected) {
    queueManipulatorPoseCommand();
    if (commandContextSource != CONTROL_SOURCE_WEB) {
      flushPendingManipulatorPoseCommand(false);
    }
  } else {
    updateMessage(tr("PWM auxiliar agora segue o pacote unificado ARMJ do manipulador.", "Aux PWM now follows the unified ARMJ manipulator packet."));
  }
  sendDutyStatePacket(channel);
}

// Sets servo offset.
void setServoOffset(int channel, int value) {
  if (!canAcceptCommandSource(commandContextSource, true)) return;
  if (channel < 0 || channel >= hardwareServoOffsets.length) {
    updateMessage(tr("Canal de offset inválido ", "Invalid offset channel ") + channel);
    return;
  }

  hardwareServoOffsets[channel] = value;
  String runtimeCommand = "OFF" + channel + "." + value;
  writeHardwareCommand(runtimeCommand);
  sendOffsetStatePacket(channel);
}

// Sets servo span.
void setServoSpan(int channel, float value) {
  if (!canAcceptCommandSource(commandContextSource, true)) return;
  if (channel < 0 || channel >= hardwareServoSpans.length) {
    updateMessage(tr("Canal de amplitude inválido ", "Invalid span channel ") + channel);
    return;
  }

  hardwareServoSpans[channel] = value;
  String runtimeCommand = "SPAN" + channel + "." + nf(value, 1, 3);
  writeHardwareCommand(runtimeCommand);
  sendSpanStatePacket(channel);
}

// Sets servo ramp.
void setServoRamp(int channel, boolean enabled) {
  if (!canAcceptCommandSource(commandContextSource, true)) return;
  if (channel < 1 || channel > 11) {
    updateMessage(tr("Canal de rampa inválido ", "Invalid ramp channel ") + channel + tr(" (esperado 1-11)", " (expected 1-11)"));
    return;
  }

  hardwareServoRamps[channel] = enabled;
  String runtimeCommand = "RAMP" + channel + "." + (enabled ? 1 : 0);
  writeHardwareCommand(runtimeCommand);
  sendRampStatePacket(channel);
}

// Sets PID gains.
void setPidGains(float kp, float ki, float kd) {
  hardwarePidKp = kp;
  hardwarePidKi = ki;
  hardwarePidKd = kd;
  updateMessage(tr("Comando PID de hardware não faz parte do protocolo runtime.", "PID hardware command is not part of the runtime protocol."));
  sendPidStatePacket();
}

// Returns manipulator binary runtime channel.
int getManipulatorBinaryRuntimeChannel(int memberIdx) {
  String[] keys = {
    "arm_map_base_channel",
    "arm_map_upper_channel",
    "arm_map_fore_channel",
    "arm_map_forearm_roll_channel",
    "arm_map_wrist_pitch_channel",
    "arm_map_wrist_rot_channel",
    "arm_map_gripper_channel"
  };
  int[] defaults = {0, 2, 3, 4, 5, 6, 7};
  if (memberIdx < 0 || memberIdx >= defaults.length) return -1;
  if (latestSensors != null && memberIdx < keys.length && latestSensors.hasKey(keys[memberIdx])) {
    return constrain(round(getSensorFloat(keys[memberIdx], defaults[memberIdx])), 0, 15);
  }
  return defaults[memberIdx];
}

// Returns manipulator member target converted to UI servo degrees.
float manipulatorMemberTargetToUiServoDeg(int memberIdx, float memberDeg) {
  switch (memberIdx) {
  case BASE_IDX:
    return armMemberBaseToUiServoDeg(memberDeg);
  case UPPERARM_IDX:
    return armMemberUpperToUiServoDeg(memberDeg);
  case FOREARM_IDX:
    return armMemberForeToUiServoDeg(memberDeg);
  case FOREARM_ROLL_IDX:
    return armMemberToUiServoDeg(FOREARM_ROLL_IDX, memberDeg);
  case WRIST_VERT_IDX:
    return armMemberWristPitchToUiServoDeg(memberDeg);
  case WRIST_ROT_IDX:
    return armMemberWristRollToUiServoDeg(memberDeg);
  case GRIPPER_IDX:
    return armMemberToUiServoDeg(GRIPPER_IDX, memberDeg);
  default:
    return memberDeg;
  }
}

// Utility: build binary servo payload.
byte[] buildBinaryServoPayload(int channel, float servoDeg) {
  int limited = channel == 0
    ? round(normalizeAbsoluteAngleDeg(servoDeg))
    : round(constrain(servoDeg, 0, 180));
  int raw = constrain(round(limited * 100.0f), 0, 65535);
  return new byte[] {
    (byte)(channel & 0xFF),
    (byte)(raw & 0xFF),
    (byte)((raw >> 8) & 0xFF)
  };
}

// Utility: write binary runtime bytes.
boolean writeBinaryRuntimeBytes(Serial port, byte[] payload, String context, boolean echoToMonitor) {
  if (port == null || payload == null || payload.length == 0) return false;
  try {
    port.write(payload);
    lastHardwareStreamTxMillis = millis();
    if (echoToMonitor) {
      appendCalibrationSerialLine("[TX HEX] " + bytesToHex(payload));
    }
    return true;
  }
  catch (Throwable t) {
    String detail = t.getMessage() == null ? t.toString() : t.getMessage();
    updateMessage(context + " failed: " + detail);
    if (port == calibrationPort) disconnectCalibrationMonitorPort();
    else if (port == myPort) handleMainSerialDisconnect(detail);
    return false;
  }
}

// Sends binary runtime frame.
boolean sendBinaryRuntimeFrame(int type, byte[] payload, String context, boolean echoToMonitor) {
  Serial port = getHexRuntimeSerialPort();
  if (port == null) {
    updateMessage(tr("Bridge/runtime HEX não conectado.", "HEX runtime bridge not connected."));
    return false;
  }
  byte[] frame = buildCalibrationBinaryFrame(type, payload == null ? new byte[0] : payload);
  return writeBinaryRuntimeBytes(port, frame, context, echoToMonitor);
}

// Requests HEX runtime handshake.
void requestHexRuntimeHello(boolean force) {
  if (!isHexCommunicationSelected()) return;
  Serial port = getHexRuntimeSerialPort();
  if (port == null) return;
  long now = millis();
  if (!force && hexRuntimeReady) return;
  if (!force && lastHexRuntimeHelloMillis > 0 && (now - lastHexRuntimeHelloMillis) < 350) return;
  // Current firmware enters runtime from boot through an ASCII READY? handshake
  // on Serial0. After that, the port switches to HEX transport frames.
  requestReadyHandshake(port);
  lastHexRuntimeHelloMillis = now;
}

boolean sendBinaryRuntimeHelloFrame(Serial port, String context, boolean echoToMonitor) {
  if (port == null) return false;
  byte[] frame = buildCalibrationBinaryFrame(0x01, new byte[0]);
  boolean sent = writeBinaryRuntimeBytes(port, frame, context, echoToMonitor);
  if (sent) lastHexRuntimeHelloMillis = millis();
  return sent;
}

void maybeRequestHexRuntimeBootstrapTelemetry() {
  // The runtime handshake is deterministic:
  // BOOT? -> READY? -> one 0x01 HELLO. Do not inject recovery HELLO frames
  // during runtime because that can reopen boot snapshots and congest Serial0.
  return;
}

// Marks HEX runtime ready and synchronizes current robot state.
void markHexRuntimeReady(String sourceLabel) {
  boolean wasReady = hexRuntimeReady;
  hexRuntimeReady = true;
  lastHexRuntimeHelloMillis = millis();
  pendingHexRuntimeBootSync = true;
  if (wasReady) return;
  updateMessage(tr("Runtime HEX ativo na Serial0", "HEX runtime ready on Serial0") + (sourceLabel != null && sourceLabel.length() > 0 ? " (" + sourceLabel + ")" : "") + tr(". Aguardando snapshot de boot.", ". Waiting for boot snapshot."));
  sendSystemStatus();
}

// Utility: parse binary robot mode value.
int parseBinaryRobotModeValue(String value) {
  String normalized = normalizeRobotTypeName(value);
  if (normalized.equals("Vehicle")) return 1;
  if (normalized.equals("Drone")) return 2;
  return 0;
}

// Utility: parse integer list.
int[] parseBinaryIntegerList(String text) {
  if (text == null) return null;
  String[] parts = split(trim(text), ',');
  if (parts == null || parts.length == 0) return null;
  int[] values = new int[parts.length];
  try {
    for (int i = 0; i < parts.length; i++) values[i] = parseInt(trim(parts[i]));
  }
  catch (Exception e) {
    return null;
  }
  return values;
}

// Returns next monotonic hardware stream sequence.
long nextHardwareStreamSequence() {
  long nextSeq = hardwareStreamSeq + 1;
  if (nextSeq < 1) nextSeq = 1;
  hardwareStreamSeq = nextSeq;
  return hardwareStreamSeq;
}

// Sends binary manipulator ARMJ command.
boolean sendManipulatorArmjBinaryCommand(int[] vals, String context, boolean echoToMonitor) {
  if (vals == null || (vals.length != 7 && vals.length != 11)) {
    updateMessage(tr("Comando ARMJ inválido para o runtime HEX.", "Invalid ARMJ command for HEX runtime."));
    return false;
  }
  long seq = nextHardwareStreamSequence();
  byte[] payload = new byte[22];
  payload[0] = (byte)(seq & 0xFF);
  payload[1] = (byte)((seq >> 8) & 0xFF);
  payload[2] = (byte)((seq >> 16) & 0xFF);
  payload[3] = (byte)((seq >> 24) & 0xFF);
  for (int i = 0; i < 7; i++) {
    int value = constrain(vals[i], -32768, 32767);
    int baseIndex = 4 + (i * 2);
    payload[baseIndex] = (byte)(value & 0xFF);
    payload[baseIndex + 1] = (byte)((value >> 8) & 0xFF);
  }
  for (int i = 0; i < 4; i++) {
    int pct = (vals.length >= 11) ? constrain(vals[7 + i], 0, 100) : 0;
    payload[18 + i] = (byte)(pct & 0xFF);
  }
  return sendBinaryRuntimeFrame(0x18, payload, context, echoToMonitor);
}

// Sends binary vehicle runtime command.
boolean sendVehicleBinaryRuntimeCommand(String context, boolean echoToMonitor) {
  VehicleRuntimeCommandState state = buildVehicleRuntimeCommandState();
  long seq = nextHardwareStreamSequence();
  return sendBinaryRuntimeFrame(0x15, new byte[] {
    (byte)(seq & 0xFF), (byte)((seq >> 8) & 0xFF), (byte)((seq >> 16) & 0xFF), (byte)((seq >> 24) & 0xFF),
    (byte)state.leftPct, (byte)state.rightPct, (byte)state.camPanDeg, (byte)state.camTiltDeg, (byte)state.flags
  }, context, echoToMonitor);
}

// Sends binary drone runtime command.
boolean sendDroneBinaryRuntimeCommand(String context, boolean echoToMonitor) {
  DroneRuntimeCommandState state = buildDroneRuntimeCommandState();
  long seq = nextHardwareStreamSequence();
  return sendBinaryRuntimeFrame(0x16, new byte[] {
    (byte)(seq & 0xFF), (byte)((seq >> 8) & 0xFF), (byte)((seq >> 16) & 0xFF), (byte)((seq >> 24) & 0xFF),
    (byte)state.throttlePct, (byte)state.yawPct, (byte)state.pitchPct, (byte)state.rollPct, (byte)state.strafePct, (byte)state.forwardPct, (byte)state.flags, (byte)state.camPanDeg, (byte)state.camTiltDeg
  }, context, echoToMonitor);
}

// Sends binary runtime command.
// Sends HEX runtime stream command.
boolean sendHexRuntimeStreamCommand(String body, String context, boolean echoToMonitor) {
  if (!isHexCommunicationSelected()) return false;
  if (!hexRuntimeReady) {
    requestHexRuntimeHello(false);
    updateMessage(tr("Runtime HEX ainda não está pronto.", "HEX runtime bridge not ready yet."));
    return false;
  }

  String normalized = normalizeRuntimeCommand(body);
  if (normalized.length() == 0) return false;

  String payloadText = normalized.startsWith("#CMD|") ? normalized : wrapHardwareStreamCommand(normalized);
  if (payloadText.length() == 0) return false;

  byte[] payload = payloadText.getBytes();
  boolean sent = sendBinaryRuntimeFrame(0x17, payload, context, echoToMonitor);
  if (sent) {
    if (normalized.equals("EXIT")) {
      hardwareStreamStoppedByExit = true;
      hardwareStreamStoppedAtMillis = millis();
      hardwareTelemetryHealthy = false;
      hardwareTelemetryTimeoutLatched = false;
      lastHardwareStreamTxMillis = hardwareStreamStoppedAtMillis;
    } else {
      hardwareStreamStoppedByExit = false;
      hardwareStreamStoppedAtMillis = 0;
    }
  }
  return sent;
}

// Sends binary runtime command.
boolean sendHardwareBinaryCommand(String normalized, String context, boolean echoToMonitor) {
  if (normalized == null) return false;
  String upper = trim(normalized).toUpperCase();
  if (upper.length() == 0) return false;

  int eqPos = normalized.indexOf('=');
  String detected = normalizeRobotTypeName(detectedRobotType);
  String selected = normalizeRobotTypeName(currentModeName());
  String robot = detected.length() > 0 ? detected : selected;

  if (upper.startsWith("ROBOT=") || upper.startsWith("ROBOT:") || upper.startsWith("ROBOTTYPE=") || upper.startsWith("ROBOTTYPE:")) {
    String value = eqPos >= 0 ? trim(normalized.substring(eqPos + 1)) : trim(normalized.substring(normalized.indexOf(':') + 1));
    int mode = parseBinaryRobotModeValue(value);
    return sendBinaryRuntimeFrame(0x14, new byte[] {(byte)(mode & 0xFF)}, context, echoToMonitor);
  }

  // COLL=0/1 is a firmware CFG persistence command, not a runtime control path.

  if (upper.startsWith("ARMJ=")) {
    int[] vals = parseBinaryIntegerList(eqPos >= 0 ? normalized.substring(eqPos + 1) : "");
    if (vals != null && (vals.length == 7 || vals.length == 11)) {
      return sendManipulatorArmjBinaryCommand(vals, context, echoToMonitor);
    }
  }

  boolean vehicleCommand =
    upper.startsWith("TRACK=") || upper.startsWith("LIGHT=") || upper.startsWith("SCAN=");
  if (upper.startsWith("CAM=") || upper.startsWith("MOVE=")) {
    vehicleCommand = robot.equals("Vehicle");
  }
  if (vehicleCommand) {
    applyVehicleRuntimeMirror(normalized);
    return sendVehicleBinaryRuntimeCommand(context, echoToMonitor);
  }

  boolean droneCommand = upper.startsWith("FLY=") || upper.startsWith("CAMREC=");
  if (upper.startsWith("CAM=")) {
    droneCommand = robot.equals("Drone");
  }
  if (droneCommand) {
    applyDroneRuntimeMirror(normalized);
    return sendDroneBinaryRuntimeCommand(context, echoToMonitor);
  }

  return sendHexRuntimeStreamCommand(normalized, context, echoToMonitor);
}

// Wraps hardware stream command.
String wrapHardwareStreamCommand(String body) {
  String normalized = normalizeRuntimeCommand(body);
  if (normalized.length() == 0) return "";
  long seq = nextHardwareStreamSequence();
  String robotName = lockedHardwareRobotName();
  if (robotName == null || trim(robotName).length() == 0) {
    robotName = currentModeName();
  }
  return "#CMD|ROBOT:" + robotName + "|SEQ:" + seq + "|TS:" + millis() + "|CMD:" + normalized;
}

// Canonicalizes runtime motion commands before newline ASCII transport.
String canonicalizeRuntimeCommandForAsciiTransport(String normalized) {
  if (normalized == null) return "";
  String cmd = normalizeRuntimeCommand(normalized);
  String upper = cmd.toUpperCase();
  if (upper.startsWith("FLY=")) {
    applyDroneRuntimeMirror(cmd);
    return buildDroneRuntimeCommand();
  }
  if (upper.startsWith("TRACK=") || upper.startsWith("MOVE=")) {
    applyVehicleRuntimeMirror(cmd);
    return buildVehicleRuntimeCommand();
  }
  return cmd;
}

// Sends hardware stream command.
boolean sendHardwareStreamCommand(String body, String context, boolean echoToMonitor) {
  if (!canAcceptCommandSource(commandContextSource, true)) return false;
  if (!((systemReady && !simulationMode) && myPort != null)) {
    updateMessage(tr("Hardware não pronto ou porta fechada.", "Hardware not ready or port closed"));
    return false;
  }

  String normalized = normalizeRuntimeCommand(body);
  if (normalized.length() == 0) return false;

  if (shouldUseBinaryRuntimeCommands()) {
    return sendHardwareBinaryCommand(normalized, context, echoToMonitor);
  }

  normalized = canonicalizeRuntimeCommandForAsciiTransport(normalized);
  if (normalized.length() == 0) return false;

  hardwareStreamStoppedByExit = false;
  hardwareStreamStoppedAtMillis = 0;

  String frame = wrapHardwareStreamCommand(normalized);
  if (frame.length() == 0) return false;
  if (!safeSerialWrite(myPort, frame + "\n", context, false)) return false;

  lastHardwareStreamTxMillis = millis();
  if (echoToMonitor) appendCalibrationSerialLine("> " + frame);
  return true;
}

// Sends raw hardware command.
void sendRawHardwareCommand(String serialData) {
  sendHardwareStreamCommand(serialData, "raw command", false);
}

// Utility: map servo index.
int mapServoIndex(int channel) {
  switch (channel) {
  case 0:
    return BASE_IDX;
  case 1:
    return UPPERARM_IDX;
  case 2:
    return FOREARM_IDX;
  case 3:
    return FOREARM_ROLL_IDX;
  case 4:
    return WRIST_VERT_IDX;
  case 5:
    return WRIST_ROT_IDX;
  case 6:
    return GRIPPER_IDX;
  default:
    return -1;
  }
}

// Utility: map index to channel.
int mapIndexToChannel(int index) {
  switch (index) {
  case BASE_IDX:
    return 0;
  case UPPERARM_IDX:
    return 1;
  case FOREARM_IDX:
    return 2;
  case FOREARM_ROLL_IDX:
    return 3;
  case WRIST_VERT_IDX:
    return 4;
  case WRIST_ROT_IDX:
    return 5;
  case GRIPPER_IDX:
    return 6;
  default:
    return -1;
  }
}

// Checks whether playing.
boolean isPlaying() {
  for (boolean pb : playingBack) {
    if (pb) {
      return true;
    }
  }
  return false;
}

// Loads servo limits.
void loadServoLimits() {
  moduleForRobot("Manipulator").loadConfigSafe();
}

// Creates default limits file.
private void createDefaultLimitsFile() {
  JSONArray jsonArray = new JSONArray();
  int[][] defaultLimits = {
    {0, 359},
    {0, 180},
    {0, 180},
    {0, 180},
    {0, 180},
    {0, 180},
    {0, 100}
  };
  for (int i = 0; i < defaultLimits.length; i++) {
    JSONObject obj = new JSONObject();
    obj.setInt("min", defaultLimits[i][0]);
    obj.setInt("max", defaultLimits[i][1]);
    jsonArray.append(obj);
  }
  saveJSONArray(jsonArray, sketchPath(SERVO_LIMITS_FILE));
}

// Loads limits from file.
private void loadLimitsFromFile(String fileName) {
  JSONArray jsonArray = loadJSONArray(fileName);
  if (jsonArray == null) return;
  for (int i = 0; i < jsonArray.size() && i < servoLimits.length; i++) {
    JSONObject obj = jsonArray.getJSONObject(i);
    if (obj == null) continue;
    servoLimits[i][0] = getJsonInt(obj, "min", servoLimits[i][0]);
    servoLimits[i][1] = getJsonInt(obj, "max", servoLimits[i][1]);
  }
  updateMessage(tr("Limites dos servos carregados com sucesso.", "Servo limits loaded successfully"));
}

// ==============================
// Homing
// ==============================

void loadHoming() {
  moduleForRobot("Manipulator").loadConfigSafe();
}

// Creates default homing.
private void createDefaultHoming() {
  JSONArray jsonArray = new JSONArray();
  int[] defaultHomingAngles = {
    180,
    150,
    70,
    90,
    95,
    130,
    0
  };
  for (int i = 0; i < defaultHomingAngles.length; i++) {
    JSONObject obj = new JSONObject();
    obj.setInt("angle", defaultHomingAngles[i]);
    jsonArray.append(obj);
  }
  saveJSONArray(jsonArray, sketchPath(HOMING_FILE));
  println("Default homing file created successfully.");
}

// Loads homing from file.
private void loadHomingFromFile() {
  JSONArray jsonArray = loadJSONArray(HOMING_FILE);
  if (jsonArray == null) {
    throw new RuntimeException("Homing file not found or is empty.");
  }
  for (int i = 0; i < jsonArray.size() && i < homingPositions.length; i++) {
    JSONObject obj = jsonArray.getJSONObject(i);
    if (obj == null) continue;
    homingPositions[i] = getJsonInt(obj, "angle", homingPositions[i]);
  }
  updateMessage(tr("Posição de homing carregada com sucesso.", "Homing position loaded successfully."));
}

// Utility: go home.
void goHome() {
  if (!goingHome) {
    return;
  }

  final int ANGLE_TOLERANCE = 2;

  boolean allJointsAtHome = true;
  int previousSource = commandContextSource;
  homingStepActive = true;
  setCommandContext(homingCommandSource);

  try {
    for (int i = 0; i < homingPositions.length; i++) {
      int channel = mapIndexToChannel(i);
      int smoothedAngle = smooth(angles[i], homingPositions[i], HOMING_SMOOTH_FACTOR);

      setAngle(channel, smoothedAngle);

      if (abs(smoothedAngle - homingPositions[i]) >= ANGLE_TOLERANCE) {
        allJointsAtHome = false;
      }
    }

    boolean homingTimedOutForCollision = pendingManipulatorLocalCollisionHomeTimedOut();
    if (allJointsAtHome || homingTimedOutForCollision) {
      goingHome = false;
      for (int i = 0; i < homingPositions.length; i++) {
        int channel = mapIndexToChannel(i);
        setAngle(channel, homingPositions[i]);
      }
      updateMessage(homingTimedOutForCollision ? "Homing timeout reached; enabling local collision guard." : "Homing done.");
      completePendingManipulatorLocalCollisionGuardAfterHome();
    }
  }
  finally {
    commandContextSource = previousSource;
    homingStepActive = false;
  }
}

// Utility: cancel pending auto home.
void cancelPendingAutoHome() {
  pendingAutoHome = false;
  pendingAutoHomeStartMs = 0;
}

// Utility: request auto home after connect.
void requestAutoHomeAfterConnect() {
  if (!autoHomeOnConnect) {
    cancelPendingAutoHome();
    return;
  }
  if (isFirmwareTelemetrySelected()) {
    cancelPendingAutoHome();
    updateMessage(tr("Auto-home do manipulador ignorado enquanto a telemetria do firmware está ativa.", "Manipulator auto-home skipped while firmware telemetry is active."));
    return;
  }
  if (!isManipulatorSelected && !detectedRobotType.equals("Manipulator")) {
    cancelPendingAutoHome();
    return;
  }
  if (!simulationMode && (!systemReady || myPort == null)) {
    cancelPendingAutoHome();
    return;
  }

  pendingAutoHome = true;
  pendingAutoHomeStartMs = millis() + max(0, autoHomeDelayMs);
  updateMessage(tr("Homing do manipulador agendado...", "Manipulator homing scheduled..."));
}

// Updates pending auto home.
void updatePendingAutoHome() {
  if (!pendingAutoHome) return;
  if (!systemReady && !simulationMode) return;
  if (!isManipulatorSelected && !detectedRobotType.equals("Manipulator")) {
    cancelPendingAutoHome();
    return;
  }
  if (dutyTargetsFollowHardware && !simulationMode && lastHardwareSensorMillis <= 0) return;
  if (millis() < pendingAutoHomeStartMs) return;

  pendingAutoHome = false;
  pendingAutoHomeStartMs = 0;
  goingHome = true;
  markManipulatorPoseDirty();
  updateMessage(tr("Executando auto-homing do manipulador...", "Manipulator auto homing..."));
}

///////////////////////////////////////////////// SERIAL ////////////////////////////////////////////////////////////////

boolean serialBannerDetected = false;
long handshakeStartTime = 0;

final int READY_HANDSHAKE_STAGE_IDLE = 0;
final int READY_HANDSHAKE_STAGE_BOOT_REQUEST = 1;
final int READY_HANDSHAKE_STAGE_READY_REQUEST = 2;
final int READY_HANDSHAKE_STAGE_HELLO_REQUEST = 3;
final int READY_HANDSHAKE_STAGE_WAIT_RUNTIME = 4;
Serial pendingReadyHandshakePort = null;
int pendingReadyHandshakeStage = READY_HANDSHAKE_STAGE_IDLE;

boolean readyHandshakeHasEnteredRuntimePhase() {
  return pendingReadyHandshakeStage == READY_HANDSHAKE_STAGE_WAIT_RUNTIME ||
         pendingReadyHandshakeStage == READY_HANDSHAKE_STAGE_HELLO_REQUEST ||
         pendingReadyHandshakeStage == READY_HANDSHAKE_STAGE_READY_REQUEST;
}

void resetMainSerialRuntimeTransitionParsers() {
  // Must hold serialRxLock: the serialEvent background thread may be
  // assembling a line in mainSerialAsciiBuffer at the same time.
  synchronized (serialRxLock) {
    clearAsciiSerialParserState(true);
  }
  mainSerialBinaryRxState = 0;
  mainSerialBinaryFrameType = 0;
  mainSerialBinaryExpected = 0;
  mainSerialBinaryIndex = 0;
}
long pendingReadyHandshakeDueMs = 0;
int pendingReadyHandshakeAttempts = 0;
final int READY_HANDSHAKE_BOOT_SETTLE_MS = 220;
final int READY_HANDSHAKE_HELLO_DELAY_MS = 120;
final int READY_HANDSHAKE_MAX_ATTEMPTS = 3;

final int SERIAL_BAUD_RATE = 115200;
// Serial timing configuration now lives in ConfigManager.pde
// and is loaded from data/SynROV_config.

PortOpenWorker activePortOpenWorker = null;
long activePortOpenStartedAt = 0;
int activePortOpenIndex = -1;
boolean activePortOpenWasLastSaved = false;
String activeProbePortName = "";

// NOTE:
// Serial probing uses a worker only for the potentially blocking port-open step.
// READY? probing and all serial writes still happen only after the worker reports
// a fully opened port back to the main Processing thread. This keeps the UI from
// freezing on busy COM ports without reintroducing the earlier write-before-open issue.
class PortOpenWorker extends Thread {
  final String portName;
  Serial openedPort = null;
  Throwable error = null;
  volatile boolean finished = false;
  volatile boolean abandoned = false;
  volatile boolean readyConfirmed = false;
  volatile boolean binaryConfirmed = false;
  String robotHint = "";
  ArrayList<String> replayLines = new ArrayList<String>();

  PortOpenWorker(String portName) {
    this.portName = portName;
    setName("SynROV-SerialProbe-" + portName);
    setDaemon(true);
  }

// Utility: close opened port quietly.
  void closeOpenedPortQuietly() {
    if (openedPort != null) {
      try {
        openedPort.stop();
        openedPort.clear();
      }
      catch (Exception ignore) {
      }
      openedPort = null;
    }
  }

// Utility: process probe line.
  void processProbeLine(String rawLine) {
    if (rawLine == null) return;
    String msg = trim(rawLine);
    if (msg.length() == 0) return;

    if (msg.startsWith("A55A")) {
      ParsedHexTransportFrame frame = parseHexTransportFrameLine(msg);
      if (frame != null) {
        binaryConfirmed = true;
        // Do not discard a valid A55A frame consumed during passive probing.
        // Replaying it after finishSerialConnection lets the first 0x20 arm
        // telemetry even if it arrived before the UI switched to runtime.
        replayLines.add(msg);
        if (frame.type == 0x30 && frame.payload != null && frame.len > 0) {
          processProbeLine(new String(frame.payload, 0, frame.len));
        }
      }
      return;
    }

    replayLines.add(msg);

    String hint = extractRobotTypeHint(msg);
    if (hint.length() > 0) robotHint = hint;

    if (msg.equals("READY!") || msg.startsWith("READY!|") || msg.startsWith("READY! ") ||
        isAsciiTelemetryLine(msg) || msg.startsWith("#CFG|") || msg.startsWith("#CTRL|") || msg.startsWith("#PID|") ||
        msg.startsWith("ROBOT=") || msg.startsWith("ROBOT:") || msg.startsWith("ROBOTTYPE=") || msg.startsWith("ROBOTTYPE:") ||
        msg.startsWith("MODE=") || msg.startsWith("MODE:")) {
      readyConfirmed = true;
    }
  }

// Utility: run.
  public void run() {
    try {
      // Use the shared sketch reference instead of a hard-coded class name.
      openedPort = new Serial(sketchApplet, portName, SERIAL_BAUD_RATE);
      openedPort.clear();
      while (openedPort.available() > 0) {
        openedPort.read();
      }

      long startMs = System.currentTimeMillis();
      long hardDeadlineMs = startMs + max(serialBootGraceMs, serialReadyInitialDelayMs + serialReadyHandshakeTimeoutMs + 700);
      StringBuilder lineBuffer = new StringBuilder();

      while (!abandoned && System.currentTimeMillis() <= hardDeadlineMs) {
        while (!abandoned && openedPort != null && openedPort.available() > 0) {
          int value = openedPort.read();
          if (value < 0) break;
          int unsignedByte = value & 0xFF;
          char ch = (char)unsignedByte;

          if (ch == '\n' || ch == '\r') {
            if (lineBuffer.length() > 0) {
              processProbeLine(lineBuffer.toString());
              lineBuffer.setLength(0);
            }
          } else if (((unsignedByte >= 32 && unsignedByte <= 126) || unsignedByte == 9) && lineBuffer.length() < 255) {
            lineBuffer.append(ch);
          } else if (!((unsignedByte >= 32 && unsignedByte <= 126) || unsignedByte == 9)) {
            if (lineBuffer.length() > 0) lineBuffer.setLength(0);
          }

          if (readyConfirmed || binaryConfirmed) break;
        }

        if (readyConfirmed || binaryConfirmed) {
          break;
        }

        try {
          Thread.sleep(10);
        }
        catch (InterruptedException ie) {
          if (abandoned) break;
        }
      }

      if ((!readyConfirmed && !binaryConfirmed) || abandoned) {
        if (error == null && !abandoned) {
          error = new RuntimeException("boot snapshot timeout");
        }
        closeOpenedPortQuietly();
      }
    }
    catch (Throwable t) {
      error = t;
      closeOpenedPortQuietly();
    }
    finally {
      finished = true;
    }
  }
}

// Utility: close serial port.
void closeSerialPort(Serial port, String portName) {
  if (port != null) {
    try {
      port.stop();
      port.clear();
    }
    catch (Exception e) {
      e.printStackTrace();
    }
  }
}

// Utility: cancel active port open.
void cancelActivePortOpen() {
  if (activePortOpenWorker != null) {
    activePortOpenWorker.abandoned = true;
    try {
      activePortOpenWorker.interrupt();
    }
    catch (Exception ignore) {
    }
  }
  activePortOpenWorker = null;
  activePortOpenStartedAt = 0;
  activePortOpenIndex = -1;
  activePortOpenWasLastSaved = false;
  activeProbePortName = "";
}

// Utility: begin port open.
void beginPortOpen(String portName, int index, boolean fromLastSavedPort) {
  cancelActivePortOpen();
  activePortOpenIndex = index;
  activePortOpenWasLastSaved = fromLastSavedPort;
  activePortOpenStartedAt = millis();
  activeProbePortName = portName;
  testPort = null;
  activePortOpenWorker = new PortOpenWorker(portName);
  activePortOpenWorker.start();
  updateMessage(tr("Abrindo porta: ", "Opening port: ") + portName);
  initializationStep = 4;
}

// Utility: poll pending port open.
boolean pollPendingPortOpen() {
  if (activePortOpenWorker == null) return false;

  if (!activePortOpenWorker.finished) {
    if (millis() - activePortOpenStartedAt > serialOpenTimeoutMs + serialBootGraceMs + serialReadyHandshakeTimeoutMs + 1000) {
      String portName = activeProbePortName;
      boolean wasLastSaved = activePortOpenWasLastSaved;
      int failedIndex = activePortOpenIndex;
      activePortOpenWorker.abandoned = true;
      try {
        activePortOpenWorker.interrupt();
      }
      catch (Exception ignore) {
      }
      updateMessage(tr("Tempo esgotado ao testar porta: ", "Timed out probing port: ") + portName);
      activePortOpenWorker = null;
      activePortOpenStartedAt = 0;
      activeProbePortName = "";
      if (wasLastSaved) {
        initializationStep = 2;
      } else {
        portIndex = failedIndex + 1;
        initializationStep = 3;
      }
      startWait(150);
    }
    return false;
  }

  PortOpenWorker worker = activePortOpenWorker;
  String portName = activeProbePortName;
  boolean wasLastSaved = activePortOpenWasLastSaved;
  int openedIndex = activePortOpenIndex;
  activePortOpenWorker = null;
  activePortOpenStartedAt = 0;
  activeProbePortName = "";
  activePortOpenIndex = -1;
  activePortOpenWasLastSaved = false;

  if (worker.openedPort != null && (worker.readyConfirmed || worker.binaryConfirmed)) {
    portIndex = openedIndex;
    if (worker.robotHint != null && trim(worker.robotHint).length() > 0) {
      pendingRobotTypeHint = worker.robotHint;
    }
    pendingBinaryRuntimeHandshakeConfirmed = worker.binaryConfirmed;
    pendingProbeReplayLines = new ArrayList<String>(worker.replayLines);
    finishSerialConnection(worker.openedPort, true);
    return true;
  }

  String failureMessage = worker.error == null ? "probe failed" : (worker.error.getMessage() == null ? worker.error.toString() : worker.error.getMessage());
  String reason = (failureMessage == null || trim(failureMessage).length() == 0) ? "probe failed" : trim(failureMessage);
  updateMessage(tr("Falha ao testar porta: ", "Failed to probe port: ") + portName + " (" + reason + ")");
  testPort = null;
  if (wasLastSaved) {
    initializationStep = 2;
  } else {
    portIndex = openedIndex + 1;
    initializationStep = 3;
  }
  startWait(150);
  return false;
}

// Resets connection runtime state.
void resetConnectionRuntimeState() {
  sentReadyRequest = false;
  readyRequestTime = 0;
  hardwareStartTime = 0;
  waiting = false;
  waitDuration = 0;
  waitStartTime = 0;
  systemReady = false;
  simulationMode = false;
  serialBannerDetected = false;
  handshakeStartTime = 0;
  pendingRobotTypeHint = "Manipulator";
  manualRobotModeOverride = false;
  runtimeAsciiObservedOnAppPort = false;
  runtimeBinaryObservedOnAppPort = false;
  lastHardwareSensorMillis = 0;
  lastHardwareRxMillis = 0;
  lastHardwareStreamTxMillis = 0;
  hardwareTelemetryHealthy = false;
  hardwareTelemetryTimeoutLatched = false;
  hardwareStreamStoppedByExit = false;
  serialMonitorSessionActive = false;
  serialMonitorActivatedAtMillis = 0;
  hardwareStreamStoppedAtMillis = 0;
  resetHardwareModeSyncState();
  resetPendingManipulatorPoseCommand();
  resetPendingVehicleRuntimeCommand();
  resetPendingDroneRuntimeCommand();
  clearPendingAppSerialAsciiLines();
  pendingHardwarePoseSync = false;
  dutyTargetsInitialized = false;
  dutyTargetsFollowHardware = true;
  resetHexRuntimeConnectionState();
  clearActiveRobotStatusSection();
  pendingHexRuntimeBootSync = false;
}

// Utility: connect.
void connect() {
  sentReadyRequest = false;
  readyRequestTime = 0;

  if (initializationStep != 0) {
    if (systemReady && !simulationMode && myPort != null) {
      preserveManipulatorPoseForDisconnect();
    }
    updateMessage(tr("Desconectado. Sistema reiniciado.", "Disconnected. System reset."));
    cancelActivePortOpen();
    closeSerialPort(testPort, "testPort");
    closeSerialPort(myPort, "myPort");
    myPort = null;
    testPort = null;
    resetConnectionRuntimeState();
    initializationStep = 0;
    currentControlOwner = CONTROL_SOURCE_NONE;
    resetDisconnectedFirmwareTelemetryState();
    sendSystemStatus();
  } else {
    updateMessage(tr("Iniciando conexão...", "Starting connection..."));

    cancelActivePortOpen();
    closeSerialPort(testPort, "testPort");
    closeSerialPort(myPort, "myPort");
    testPort = null;
    myPort = null;

    portIndex = 0;
    resetConnectionRuntimeState();
    initializationStep = 1;
    clearLog();

    startWait(1000);
  }
}

// Utility: activate simulation from connect button.
void activateSimulationFromConnectButton() {
  cancelActivePortOpen();
  closeSerialPort(testPort, "testPort");
  closeSerialPort(myPort, "myPort");
  myPort = null;
  testPort = null;
  resetConnectionRuntimeState();
  sentReadyRequest = false;
  readyRequestTime = 0;
  serialBannerDetected = false;
  simulationMode();
  updateMessage(tr("Modo de simulação ativado (duplo clique em Conectar).", "Simulation Mode Enabled (double-click Connect)"));
}

// Utility: prepare candidate port.
void prepareCandidatePort(Serial port) {
  if (port == null) return;
  port.clear();
  while (port.available() > 0) {
    port.read();
  }
  port.bufferUntil('\n');
  flushSerialInputQuietly(port);
  armAsciiSerialResyncWindow(port == myPort, SERIAL_ASCII_RESYNC_GRACE_MS);
  hardwareStartTime = millis();
  handshakeStartTime = hardwareStartTime;
  sentReadyRequest = false;
  readyRequestTime = 0;
  serialBannerDetected = false;
  pendingRobotTypeHint = "Manipulator";
  clearActiveRobotStatusSection();
}

// Utility: check last port.
void checkLastPort() {
  if (!checkWait()) return;

  updateMessage(tr("Verificando última porta usada...", "Checking last used port..."));
  String lastPort = loadLastPort();
  ports = Serial.list();

  if (lastPort != null) {
    for (int i = 0; i < ports.length; i++) {
      if (ports[i].equals(lastPort)) {
        updateMessage(tr("Tentando reconectar à última porta: ", "Attempting to reconnect to last port: ") + lastPort);
        try {
          beginPortOpen(lastPort, i, true);
          return;
        }
        catch (Exception e) {
          updateMessage(tr("Reconexão automática falhou. Testando outras portas.", "Auto-reconnect failed. Testing others"));
        }
      }
    }
  } else {
    updateMessage(tr("Nenhuma última porta salva ou encontrada.", "No last port saved or found"));
  }

  startWait(1000);
  initializationStep = 2;
}

// Utility: list serial ports.
void listSerialPorts() {
  if (!checkWait()) return;

  updateMessage(tr("Procurando portas seriais...", "Searching for serial ports..."));
  ports = Serial.list();
  updateMessage(" " + ports.length + tr(" portas encontradas.", " ports found."));
  startWait(500);
  initializationStep = 3;
}

// Utility: test serial ports.
void testSerialPorts() {
  if (!checkWait()) return;

  if (portIndex < ports.length) {
    updateMessage(tr("Testando porta: ", "Testing port: ") + ports[portIndex]);
    try {
      beginPortOpen(ports[portIndex], portIndex, false);
    }
    catch (Exception e) {
      updateMessage(tr("Falha ao agendar teste da porta: ", "Failed to schedule port test: ") + ports[portIndex] + tr(". Tentando próxima...", ". Trying next..."));
      portIndex++;
      startWait(150);
    }
  } else {
    initializationStep = 6;
    updateMessage(tr("Nenhuma porta SynROV Serial0 atual encontrada.", "No current SynROV Serial0 port found"));
  }
}

// Utility: line looks like syn rov boot.
boolean lineLooksLikeSynROVBoot(String msg) {
  if (msg == null) return false;
  String up = trim(msg).toUpperCase();
  return up.equals("SYNROV") || up.indexOf("SYNROV") >= 0;
}

// Utility: line looks like syn rov identity.
boolean lineLooksLikeSynROVIdentity(String msg) {
  if (msg == null) return false;
  String normalized = trim(msg);
  if (normalized.length() == 0) return false;

  String upper = normalized.toUpperCase();
  return upper.indexOf("SYNROV") >= 0 ||
         upper.startsWith("READY!") ||
         upper.startsWith("ROBOT=") ||
         upper.startsWith("ROBOT:") ||
         upper.startsWith("ROBOTTYPE=") ||
         upper.startsWith("ROBOTTYPE:") ||
         upper.startsWith("MODE=") ||
         upper.startsWith("MODE:") ||
         upper.startsWith("#TEL|") || upper.startsWith("TEL|") || upper.startsWith("#SENS|") || upper.startsWith("SENS|") ||
         upper.startsWith("#CMD|") ||
         upper.startsWith("#CFG|ROBOT:") ||
         upper.startsWith("#CTRL|ROBOT:");
}

// Utility: configure primary serial buffering for active protocol.
void configurePrimarySerialBufferMode() {
  if (myPort == null) return;
  flushSerialInputQuietly(myPort);
  try {
    if (isHexCommunicationSelected()) {
      myPort.buffer(1);
    } else {
      myPort.bufferUntil('\n');
    }
  }
  catch (Exception ignore) {
  }
  armAsciiSerialResyncWindow(true, SERIAL_ASCII_RESYNC_GRACE_MS);
}

// Finishes serial connection.
void finishSerialConnection(Serial confirmedPort, boolean requestRuntimeAfterConnect) {
  if (confirmedPort == null) return;

  myPort = confirmedPort;
  configurePrimarySerialBufferMode();
  testPort = null;
  activeProbePortName = "";
  systemReady = true;
  telemetrySourceMode = TELEMETRY_SOURCE_FIRMWARE;
  disableBootDemoWorlds();
  clearAllEnvironmentMaps();
  manualRobotModeOverride = false;
  setDetectedRobotType(pendingRobotTypeHint);

  if (ports != null && portIndex >= 0 && portIndex < ports.length) {
    saveLastPort(ports[portIndex]);
    updateMessage(tr("Serial conectada: ", "Serial connected: ") + ports[portIndex] + " [" + robotDisplayName(detectedRobotType) + "] [ASCII boot / HEX runtime]");
  } else {
    updateMessage(tr("Serial conectada [", "Serial connected [") + robotDisplayName(detectedRobotType) + "] [ASCII boot / HEX runtime]");
  }

  goingHome = false;
  cancelPendingAutoHome();
  resetPendingManipulatorPoseCommand();
  resetPendingVehicleRuntimeCommand();
  resetPendingDroneRuntimeCommand();
  pendingHardwarePoseSync = detectedRobotType.equals("Manipulator");
  initializationStep = 5;
  sentReadyRequest = false;
  handshakeStartTime = 0;
  serialBannerDetected = true;
  lastHardwareSensorMillis = 0;
  lastHardwareRxMillis = 0;
  lastHardwareStreamTxMillis = 0;
  hardwareTelemetryHealthy = false;
  hardwareTelemetryTimeoutLatched = false;
  hardwareStreamStoppedByExit = false;
  serialMonitorSessionActive = false;
  serialMonitorActivatedAtMillis = 0;
  hardwareStreamStoppedAtMillis = 0;
  syncHardwareControlConfig();

  if (pendingProbeReplayLines != null && pendingProbeReplayLines.size() > 0) {
    ArrayList<String> replayCopy = new ArrayList<String>(pendingProbeReplayLines);
    pendingProbeReplayLines.clear();
    for (String replayLine : replayCopy) {
      if (replayLine == null) continue;
      handleAppSerialAsciiLine(replayLine);
    }
    processPendingAppSerialAsciiLines();
  }

  // A replayed 0x20 frame from the probe may have set hexRuntimeReady = true, which
  // would make requestReadyHandshake() below a no-op and prevent "READY?" from ever
  // being sent to the Arduino.  We want to keep the geometry/config data extracted
  // from the replay, but the live handshake must always happen, so reset the flag now.
  // lastHardwareSensorMillis is also reset so the health monitor doesn't start the
  // timeout clock from stale replay data.
  resetHexRuntimeConnectionState();
  lastHardwareSensorMillis = 0;

  // Build the initial 3D/sensor state from the passive ASCII boot snapshot
  // before opening the runtime stream.
  try {
    syncLatestSensorsForActiveTelemetrySource();
    applySensorState();
  }
  catch (Exception e) {
    e.printStackTrace();
    latestSensors = buildZeroTelemetrySnapshotForRobot(detectedRobotType);
    syncLatestSensorsForActiveTelemetrySource();
    updateMessage(tr("Sincronização da conexão recuperada de estado runtime nulo.", "Connection sync recovered from null runtime state."));
  }

  if (requestRuntimeAfterConnect) {
    updateMessage(tr("Snapshot de boot capturado. Abrindo runtime HEX na Serial0...", "Boot snapshot captured. Opening HEX runtime on Serial0..."));
    requestReadyHandshake(myPort);
  }
  pendingBinaryRuntimeHandshakeConfirmed = false;

  requestHardwareSync();
  sendFullWebState();
}

// Updates robot hint from serial line.
void updateRobotHintFromSerialLine(String msg) {
  if (msg == null) return;
  String hint = extractRobotTypeHint(msg);
  if (hint.length() > 0) {
    pendingRobotTypeHint = hint;
  }
}


// Utility: describe serial port.
String describeSerialPort(Serial port) {
  if (port == null) return "unknown";
  if (port == myPort && ports != null && portIndex >= 0 && portIndex < ports.length) {
    return ports[portIndex];
  }
  if (port == testPort && activeProbePortName != null && activeProbePortName.length() > 0) {
    return activeProbePortName;
  }
  return "serial";
}

// Handles main serial disconnect.
void handleMainSerialDisconnect(String detail) {
  String suffix = (detail == null || trim(detail).length() == 0) ? "" : (": " + trim(detail));
  if (diagnosticsSensorRecording) stopDiagnosticsSensorRecording();
  preserveManipulatorPoseForDisconnect();
  appendCalibrationSerialLine("[ERROR] Serial0 connection lost" + suffix);
  updateMessage(tr("Serial0 desconectada", "Serial0 disconnected") + suffix);
  closeSerialPort(myPort, "myPort");
  myPort = null;
  systemReady = false;
  sentReadyRequest = false;
  serialBannerDetected = false;
  lastHardwareSensorMillis = 0;
  lastHardwareRxMillis = 0;
  lastHardwareStreamTxMillis = 0;
  hardwareTelemetryHealthy = false;
  hardwareTelemetryTimeoutLatched = false;
  hardwareStreamStoppedByExit = false;
  serialMonitorSessionActive = false;
  serialMonitorActivatedAtMillis = 0;
  hardwareStreamStoppedAtMillis = 0;
  resetHardwareModeSyncState();
  currentControlOwner = CONTROL_SOURCE_NONE;
  manualRobotModeOverride = false;
  clearPendingAppSerialAsciiLines();
  resetPendingManipulatorPoseCommand();
  resetPendingVehicleRuntimeCommand();
  resetPendingDroneRuntimeCommand();
  pendingHardwarePoseSync = false;
  dutyTargetsInitialized = false;
  dutyTargetsFollowHardware = true;
  moduleForRobot("Vehicle").resetInputStateSafe();
  moduleForRobot("Drone").resetInputStateSafe();
  resetDisconnectedFirmwareTelemetryState();
  sendSystemStatus();
}

// Handles probe serial failure.
void handleProbeSerialFailure(String detail) {
  String suffix = (detail == null || trim(detail).length() == 0) ? "" : (": " + trim(detail));
  updateMessage(tr("Falha no teste da porta candidata", "Probe failed on candidate port") + suffix);
  closeSerialPort(testPort, "testPort");
  testPort = null;
  activeProbePortName = "";
  sentReadyRequest = false;
  serialBannerDetected = false;
  portIndex++;
  initializationStep = 3;
  startWait(150);
}

// Utility: safe serial write.
boolean safeSerialWrite(Serial port, String payload, String context, boolean isProbePort) {
  if (port == null || payload == null) {
    return false;
  }

  try {
    port.write(payload);
    return true;
  }
  catch (Throwable t) {
    String portLabel = describeSerialPort(port);
    String detail = t.getMessage() == null ? t.toString() : t.getMessage();
    String fullDetail = portLabel + " -> " + detail;

    if (isProbePort || port == testPort) {
      handleProbeSerialFailure(fullDetail);
    } else {
      handleMainSerialDisconnect(fullDetail);
    }
    return false;
  }
}

// Clears any pending staged boot/READY?/HELLO handshake.
void cancelPendingReadyHandshake() {
  pendingReadyHandshakePort = null;
  pendingReadyHandshakeStage = READY_HANDSHAKE_STAGE_IDLE;
  pendingReadyHandshakeDueMs = 0;
  pendingReadyHandshakeAttempts = 0;
}

// Returns true when a staged handshake is already active for the port.
boolean isReadyHandshakePendingForPort(Serial port) {
  return port != null &&
         pendingReadyHandshakeStage != READY_HANDSHAKE_STAGE_IDLE &&
         pendingReadyHandshakePort == port;
}

// Services the staged boot -> READY? -> HELLO handshake without blocking draw().
void servicePendingReadyHandshake() {
  if (pendingReadyHandshakeStage == READY_HANDSHAKE_STAGE_IDLE || pendingReadyHandshakePort == null) return;

  Serial port = pendingReadyHandshakePort;
  long now = millis();

  if (port != myPort && port != testPort && port != calibrationPort) {
    cancelPendingReadyHandshake();
    return;
  }

  if (hexRuntimeReady) {
    cancelPendingReadyHandshake();
    return;
  }

  if (pendingReadyHandshakeStage == READY_HANDSHAKE_STAGE_WAIT_RUNTIME) {
    if (sentReadyRequest && readyRequestTime > 0 && (now - readyRequestTime) >= serialReadyHandshakeTimeoutMs) {
      if (pendingReadyHandshakeAttempts >= READY_HANDSHAKE_MAX_ATTEMPTS) {
        updateMessage(tr("Tempo esgotado para READY? / HELLO HEX em ", "READY? / HEX hello timed out on ") + describeSerialPort(port) + ".");
        cancelPendingReadyHandshake();
        return;
      }
      sentReadyRequest = false;
      readyRequestTime = 0;
      pendingReadyHandshakeStage = READY_HANDSHAKE_STAGE_BOOT_REQUEST;
      pendingReadyHandshakeDueMs = now + 120;
      updateMessage(tr("Repetindo o handshake em etapas em ", "Retrying staged boot handshake on ") + describeSerialPort(port) + "...");
    }
    return;
  }

  if (now < pendingReadyHandshakeDueMs) return;

  if (pendingReadyHandshakeStage == READY_HANDSHAKE_STAGE_BOOT_REQUEST) {
    if (safeSerialWrite(port, "BOOT?\n", "boot pre-handshake", port == testPort)) {
      pendingReadyHandshakeAttempts++;
      pendingReadyHandshakeStage = READY_HANDSHAKE_STAGE_READY_REQUEST;
      pendingReadyHandshakeDueMs = now + READY_HANDSHAKE_BOOT_SETTLE_MS;
      updateMessage(tr("Tela de boot solicitada em ", "Boot screen requested on ") + describeSerialPort(port) + ".");
    } else {
      cancelPendingReadyHandshake();
    }
    return;
  }

  if (pendingReadyHandshakeStage == READY_HANDSHAKE_STAGE_READY_REQUEST) {
    if (safeSerialWrite(port, "READY?\n", "ready handshake", port == testPort)) {
      sentReadyRequest = true;
      if (handshakeStartTime <= 0) handshakeStartTime = now;
      readyRequestTime = now;
      resetMainSerialRuntimeTransitionParsers();
      pendingReadyHandshakeStage = READY_HANDSHAKE_STAGE_HELLO_REQUEST;
      pendingReadyHandshakeDueMs = now + READY_HANDSHAKE_HELLO_DELAY_MS;
      updateMessage(tr("READY? enviado em ", "READY? sent on ") + describeSerialPort(port) + tr(". Enviando HELLO HEX estável...", ". Sending stable HEX HELLO..."));
    } else {
      cancelPendingReadyHandshake();
    }
    return;
  }

  if (pendingReadyHandshakeStage == READY_HANDSHAKE_STAGE_HELLO_REQUEST) {
    if (writeBinaryRuntimeBytes(port, buildCalibrationBinaryFrame(0x01, new byte[0]), "hex hello", port == testPort)) {
      lastHexRuntimeHelloMillis = now;
      pendingReadyHandshakeStage = READY_HANDSHAKE_STAGE_WAIT_RUNTIME;
      pendingReadyHandshakeDueMs = now + serialReadyHandshakeTimeoutMs;
    } else {
      cancelPendingReadyHandshake();
    }
    return;
  }
}

// Utility: request ready handshake.
void requestReadyHandshake(Serial port) {
  if (port == null || hexRuntimeReady || isReadyHandshakePendingForPort(port)) return;

  long now = millis();
  if (handshakeStartTime <= 0) handshakeStartTime = now;
  sentReadyRequest = false;
  readyRequestTime = 0;
  pendingReadyHandshakePort = port;
  pendingReadyHandshakeStage = READY_HANDSHAKE_STAGE_BOOT_REQUEST;
  pendingReadyHandshakeDueMs = now;
  pendingReadyHandshakeAttempts = 0;
}

// Utility: connect hardware.
void connectHardware() {
  pollPendingPortOpen();
}

// Sends WebSocket JSON.
void sendWsJson(JSONObject j) {
  sendWebSocketJson(j);
}

// Sends WebSocket serial line.
void sendWsSerialLine(String line) {
  if (line == null || line.length() == 0) return;
  JSONObject j = new JSONObject();
  j.setString("type", "serial");
  j.setString("serial", line);
  sendWsJson(j);
}

// Queues an ASCII line from the main hardware serial port for main-thread processing.
void enqueuePendingAppSerialAsciiLine(String line) {
  if (line == null || line.length() == 0) return;
  synchronized (pendingAppSerialAsciiLines) {
    pendingAppSerialAsciiLines.add(line);
  }
}

// Clears queued ASCII lines from the main hardware serial port.
void clearPendingAppSerialAsciiLines() {
  synchronized (pendingAppSerialAsciiLines) {
    pendingAppSerialAsciiLines.clear();
  }
}

// Processes a sanitized ASCII line from the main hardware serial port on the animation thread.
void processAppSerialAsciiLineOnMainThread(String line) {
  if (line == null || line.length() == 0) return;

  if (line.startsWith("A55A")) {
    ParsedHexTransportFrame frame = parseHexTransportFrameLine(line);
    if (frame != null) {
      noteBinaryRuntimeObservedOnAppPort();
      handleAppSerialBinaryFrame(frame.type, frame.payload, frame.len);
      return;
    }
  }

  if (consumeRobotStatusAsciiLine(line)) {
    return;
  }

  if (isAsciiTelemetryLine(line)) {
    lastHardwareSensorMillis = millis();
    if (serialMonitorSessionActive) {
      endSerialMonitorSession();
    }
    hardwareTelemetryHealthy = true;
    hardwareTelemetryTimeoutLatched = false;
    JSONObject parsedSnapshot = parseSensorPacket(line);
    if (parsedSnapshot != null && parsedSnapshot.size() > 0) {
      String robotName = getJsonString(parsedSnapshot, "robot", currentModeName());
      JSONObject merged = getOrCreateRobotSensorSnapshot(robotName);
      mergeSensorSnapshotFields(merged, parsedSnapshot);
      merged.setString("robot", robotName);
      storeRobotSensorSnapshot(robotName, merged);
      setDetectedRobotType(robotName);
      syncLatestSensorsFromActiveSelection();
    }
    sendWsSensorSnapshot(parsedSnapshot, line);
    return;
  } else if (line.startsWith("#CTRL|")) {
    parseControlPacket(line);
    return;
  } else if (line.startsWith("#PID|")) {
    parsePidPacket(line);
    return;
  }

  // Strict ASCII runtime: ignore non-runtime #SENS| payloads and any other non-framed lines.
}

// Drains queued ASCII lines from the main hardware serial port on the animation thread.
void processPendingAppSerialAsciiLines() {
  ArrayList<String> lines = null;
  synchronized (pendingAppSerialAsciiLines) {
    if (pendingAppSerialAsciiLines.isEmpty()) return;
    lines = new ArrayList<String>(pendingAppSerialAsciiLines);
    pendingAppSerialAsciiLines.clear();
  }

  for (String line : lines) {
    processAppSerialAsciiLineOnMainThread(line);
  }
}

// Sends WebSocket sensor snapshot.
void sendWsSensorSnapshot(JSONObject sens, String rawLine) {
  if (sens == null) return;
  JSONObject j = new JSONObject();
  String robotName = getJsonString(sens, "robot", currentModeName());
  JSONObject snapshot = buildUnifiedRuntimeSnapshot(robotName, sens);
  boolean isHandshakeSnapshot = (rawLine != null && !isAsciiTelemetryLine(rawLine)) || getSensorJsonInt(sens, "static", 0) > 0;
  j.setString("type", isHandshakeSnapshot ? "handshake" : "telemetry");
  j.setString("robot", robotName);
  if (rawLine != null) {
    j.setString("serial", rawLine);
  }
  j.setJSONObject("sensors", sens);
  j.setJSONObject("snapshot", snapshot);
  j.setJSONObject("control", snapshot.getJSONObject("control"));
  j.setJSONObject("gps", snapshot.getJSONObject("gps"));
  sendWsJson(j);
}

// Utility: clear ASCII serial parser state.
void clearAsciiSerialParserState(boolean appPort) {
  StringBuilder buffer = appPort ? mainSerialAsciiBuffer : calibrationSerialAsciiBuffer;
  buffer.setLength(0);
  if (appPort) {
    mainSerialAsciiDiscarding = false;
  } else {
    calibrationSerialAsciiDiscarding = false;
  }
}

// Utility: arm ASCII resynchronization window.
void armAsciiSerialResyncWindow(boolean appPort, int durationMs) {
  long due = millis() + max(0, durationMs);
  // Guard the buffer clear with the same lock used by processIncomingSerialBytes
  // so we don't corrupt a partially-assembled frame mid-byte.
  synchronized (serialRxLock) {
    clearAsciiSerialParserState(appPort);
  }
  if (appPort) {
    if (due > mainSerialAsciiResyncUntilMs) mainSerialAsciiResyncUntilMs = due;
  } else {
    if (due > calibrationSerialAsciiResyncUntilMs) calibrationSerialAsciiResyncUntilMs = due;
  }
}

// Utility: configure calibration serial buffering for active protocol.
void configureCalibrationSerialBufferMode() {
  if (calibrationPort == null) return;
  try {
    calibrationPort.clear();
  }
  catch (Exception ignore) {
  }
  try {
    calibrationPort.bufferUntil('\n');
  }
  catch (Exception ignore) {
  }
}

// Utility: flush serial bytes quietly.
void flushSerialInputQuietly(Serial port) {
  if (port == null) return;
  try {
    port.clear();
  }
  catch (Exception ignore) {
  }
  try {
    while (port.available() > 0) {
      port.read();
    }
  }
  catch (Exception ignore) {
  }
}

// Returns true when text starts with any known prefix.
boolean startsWithAnyMarker(String text, String[] markers) {
  if (text == null || markers == null) return false;
  for (int i = 0; i < markers.length; i++) {
    String marker = markers[i];
    if (marker != null && marker.length() > 0 && text.startsWith(marker)) {
      return true;
    }
  }
  return false;
}

// Returns the earliest embedded serial frame marker.
int findEmbeddedAsciiMarker(String text) {
  if (text == null || text.length() == 0) return -1;
  int best = -1;
  for (int i = 0; i < SERIAL_ASCII_FRAME_MARKERS.length; i++) {
    String marker = SERIAL_ASCII_FRAME_MARKERS[i];
    if (marker == null || marker.length() == 0) continue;
    int idx = text.indexOf(marker);
    if (idx > 0 && (best < 0 || idx < best)) {
      best = idx;
    }
  }
  return best;
}

// Heuristic for line that looks like a cut #CFG payload.
boolean looksLikeCfgPayloadWithoutPrefix(String text) {
  if (text == null || text.length() == 0) return false;
  String candidate = text.startsWith("|") ? text.substring(1) : text;
  if (candidate.length() == 0) return false;
  return startsWithAnyMarker(candidate, SERIAL_ASCII_CFG_KEYS);
}

// Heuristic for line that looks like a cut telemetry payload.
boolean looksLikeSensPayloadWithoutPrefix(String text) {
  if (text == null || text.length() == 0) return false;
  String candidate = text.startsWith("|") ? text.substring(1) : text;
  if (candidate.length() == 0) return false;
  return startsWithAnyMarker(candidate, SERIAL_ASCII_SENS_KEYS);
}

// Sanitizes incoming ASCII line after port open / monitor attach.
String sanitizeIncomingAsciiLine(String rawLine, boolean appPort) {
  if (rawLine == null) return "";
  String line = trim(rawLine);
  if (line.length() == 0) return "";

  int embedded = findEmbeddedAsciiMarker(line);
  if (embedded > 0) {
    line = trim(line.substring(embedded));
  }

  long resyncUntil = appPort ? mainSerialAsciiResyncUntilMs : calibrationSerialAsciiResyncUntilMs;
  boolean inResyncWindow = millis() <= resyncUntil;

  // Strict ASCII runtime: do not auto-promote partial packets.
  // Runtime telemetry is accepted only when the line already arrives as #TEL|...

  if (startsWithAnyMarker(line, SERIAL_ASCII_FRAME_MARKERS)) {
    if (appPort) {
      mainSerialAsciiResyncUntilMs = 0;
    } else {
      calibrationSerialAsciiResyncUntilMs = 0;
    }
  }

  return line;
}

// Clears active robot status parsing context.
void clearActiveRobotStatusSection() {
  activeRobotStatusSection = "";
  activeRobotStatusRobot = "";
}

// Returns robot type from status header.
String extractRobotTypeFromStatusHeader(String line) {
  if (line == null) return "";
  String normalized = trim(line).toUpperCase();
  if (normalized.equals("[MANIPULATOR STATUS]") || normalized.indexOf("MANIPULATOR STATUS") >= 0) return "Manipulator";
  if (normalized.equals("[VEHICLE STATUS]") || normalized.indexOf("VEHICLE STATUS") >= 0) return "Vehicle";
  if (normalized.equals("[DRONE STATUS]") || normalized.indexOf("DRONE STATUS") >= 0) return "Drone";
  return "";
}

// Consumes robot status boot line.
boolean consumeRobotStatusAsciiLine(String line) {
  if (line == null) return false;
  String headerRobot = extractRobotTypeFromStatusHeader(line);
  if (headerRobot.length() > 0) {
    activeRobotStatusSection = "STATUS";
    activeRobotStatusRobot = headerRobot;
    setDetectedRobotType(headerRobot);
    if (pendingHexRuntimeBootSync) {
      pendingHexRuntimeBootSync = false;
      sendSystemStatus();
    }
    return true;
  }

  if (isAsciiTelemetryLine(line) || line.startsWith("#CMD|") || line.startsWith("#CTRL|") || line.startsWith("#PID|") ||
      line.startsWith("READY!") || line.startsWith("ROBOT=") || line.startsWith("ROBOT:") || line.startsWith("ROBOT_INDEX=") || line.startsWith("ROBOT_INDEX:")) {
    clearActiveRobotStatusSection();
    return false;
  }

  if (line.equals("============================================================")) {
    clearActiveRobotStatusSection();
    return true;
  }

  if (!activeRobotStatusSection.equals("STATUS") || activeRobotStatusRobot.length() == 0) return false;

  int eq = line.indexOf('=');
  if (eq <= 0) return false;

  String key = trim(line.substring(0, eq)).toLowerCase();
  String value = trim(line.substring(eq + 1));
  if (key.length() == 0 || value.length() == 0) return false;

  JSONObject sens = getOrCreateRobotSensorSnapshot(activeRobotStatusRobot);
  sens.setString("robot", activeRobotStatusRobot);
  sens.setInt("static", 1);
  if (key.equals("armoff")) {
    if (applyManipulatorArmjOffsetsCsv(value)) {
      sens.setString(key, value);
    } else {
      setNumberOrText(sens, key, value);
    }
  } else if (key.equals("armlim") || key.equals("srvlim")) {
    applyManipulatorArmLimitsCsvToSnapshot(sens, value);
    sens.setString(key, value);
  } else if (key.startsWith("pwmboot")) {
    try {
      int channel = Integer.parseInt(key.substring(7));
      setNumberOrText(sens, "pwm_" + channel, value);
    }
    catch (Exception e) {
      setNumberOrText(sens, key, value);
    }
    setNumberOrText(sens, key, value);
  } else if (key.equals("basech")) {
    setNumberOrText(sens, "arm_map_base_channel", value);
    setNumberOrText(sens, key, value);
  } else if (key.equals("upperch")) {
    setNumberOrText(sens, "arm_map_upper_channel", value);
    setNumberOrText(sens, key, value);
  } else if (key.equals("forech")) {
    setNumberOrText(sens, "arm_map_fore_channel", value);
    setNumberOrText(sens, key, value);
  } else if (key.equals("forerollch")) {
    setNumberOrText(sens, "arm_map_forearm_roll_channel", value);
    setNumberOrText(sens, key, value);
  } else if (key.equals("wristpitchch")) {
    setNumberOrText(sens, "arm_map_wrist_pitch_channel", value);
    setNumberOrText(sens, key, value);
  } else if (key.equals("wristrotch")) {
    setNumberOrText(sens, "arm_map_wrist_rot_channel", value);
    setNumberOrText(sens, key, value);
  } else if (key.equals("gripch")) {
    setNumberOrText(sens, "arm_map_gripper_channel", value);
    setNumberOrText(sens, key, value);
  } else {
    setNumberOrText(sens, key, value);
  }
  storeRobotSensorSnapshot(activeRobotStatusRobot, sens);
  // Defer latestSensors synchronization and visual/app state application to the
  // normal draw()/processSerialIO() cycle. Running applySensorState() here from
  // the serial callback can trigger matrix-stack work (collectColliders / 3D
  // manipulator rebuild) off the animation thread and freeze or desync the arm.
  sendWsSensorSnapshot(sens, line);
  return true;
}

// Utility: dispatch ASCII line from app port.
void handleAppSerialAsciiLine(String line) {
  if (line == null) return;
  line = sanitizeIncomingAsciiLine(line, true);
  if (line.length() == 0) return;

  if (isAsciiTelemetryLine(line) || line.startsWith("#CMD|") || line.startsWith("#CTRL|") || line.startsWith("#PID|")) {
    noteAsciiRuntimeObservedOnAppPort();
  }
  lastHardwareRxMillis = millis();
  if (hardwareStreamStoppedByExit && !serialMonitorSessionActive) {
    hardwareStreamStoppedByExit = false;
    hardwareStreamStoppedAtMillis = 0;
    hardwareTelemetryTimeoutLatched = false;
  }

  updateRobotHintFromSerialLine(line);

  // HEX-framed lines (A55A...) are decoded and logged on the draw thread
  // inside processAppSerialAsciiLineOnMainThread / handleRuntimeBinaryFrame.
  // Calling appendCalibrationSerialLine or sendWsSerialLine here would touch
  // UI state from the serialEvent background thread, causing glitches and
  // flooding the monitor with raw hex at telemetry rate.
  boolean isHexFrame = line.startsWith("A55A");
  if (!isHexFrame) {
    appendCalibrationSerialLine(line);
    sendWsSerialLine(line);
  }

  // Serial callbacks may run off the animation thread. Queue the sanitized line
  // here and consume it from draw()/processSerialIO() so telemetry, diagnostics,
  // robot snapshots and visual state are all updated from one thread only.
  enqueuePendingAppSerialAsciiLine(line);
}

// Utility: handle calibration-only ASCII line.
void handleCalibrationSerialAsciiLine(String line) {
  if (line == null) return;
  line = sanitizeIncomingAsciiLine(line, false);
  if (line.length() == 0) return;
  appendCalibrationSerialLine(line);
}

// Utility: append binary frame to monitor.
void appendCalibrationBinaryFrameLog(String direction, int type, byte[] payload, int len) {
  String frameLine = buildCalibrationBinaryFrameLine(type, payload == null ? new byte[0] : subset(payload, 0, len));
  appendCalibrationSerialLine("[" + direction + " HEX] " + frameLine);
}

class ParsedHexTransportFrame {
  int type = 0;
  int len = 0;
  byte[] payload = new byte[0];
}

int parseHexTransportNibble(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  return -1;
}

int parseHexTransportByte(String text, int index) {
  if (text == null || index < 0 || index + 1 >= text.length()) return -1;
  int hi = parseHexTransportNibble(text.charAt(index));
  int lo = parseHexTransportNibble(text.charAt(index + 1));
  if (hi < 0 || lo < 0) return -1;
  return (hi << 4) | lo;
}

ParsedHexTransportFrame parseHexTransportFrameLine(String rawLine) {
  if (rawLine == null) return null;
  String line = trim(rawLine);
  if (line.length() < 12 || !line.startsWith("A55A")) return null;

  int type = parseHexTransportByte(line, 4);
  int lenHi = parseHexTransportByte(line, 6);
  int lenLo = parseHexTransportByte(line, 8);
  if (type < 0 || lenHi < 0 || lenLo < 0) return null;

  int payloadLen = ((lenHi & 0xFF) << 8) | (lenLo & 0xFF);
  int expectedChars = 12 + payloadLen * 2;
  // Accept a valid frame at the start of the line and ignore any trailing
  // printable bytes. This makes the runtime parser robust when a USB driver
  // or Bluetooth bridge appends harmless text after a complete HEX frame.
  if (line.length() < expectedChars) return null;

  byte[] payload = new byte[payloadLen];
  int checksum = 0;
  for (int i = 0; i < payloadLen; i++) {
    int value = parseHexTransportByte(line, 10 + i * 2);
    if (value < 0) return null;
    payload[i] = (byte)(value & 0xFF);
    checksum ^= value;
  }

  int rxChecksum = parseHexTransportByte(line, 10 + payloadLen * 2);
  if (rxChecksum < 0 || ((checksum & 0xFF) != (rxChecksum & 0xFF))) return null;

  ParsedHexTransportFrame frame = new ParsedHexTransportFrame();
  frame.type = type & 0xFF;
  frame.len = payloadLen;
  frame.payload = payload;
  return frame;
}

// Utility: summarize binary frame.
String describeBinaryFrameType(int type) {
  switch (type & 0xFF) {
  case 0x01: return "HELLO";
  case 0x10: return "SERVO";
  case 0x11: return "PWM";
  case 0x12: return "GSTAB";
  case 0x13: return "NEUTRAL";
  case 0x14: return "ROBOT";
  case 0x15: return "VEHICLE";
  case 0x16: return "DRONE";
  case 0x17: return "STREAM";
  case 0x18: return "ARMJ";
  case 0x20: return "TELEMETRY";
  case 0x21: return "GPS";
  case 0x30: return "BOOTTEXT";
  case 0x81: return "ACK";
  default: return "TYPE 0x" + hex(type & 0xFF, 2);
  }
}

// Reads little-endian unsigned 16-bit value.
int readBinaryU16(byte[] payload, int idx) {
  if (payload == null || idx < 0 || idx + 1 >= payload.length) return 0;
  return (payload[idx] & 0xFF) | ((payload[idx + 1] & 0xFF) << 8);
}

// Reads little-endian signed 16-bit value.
int readBinaryI16(byte[] payload, int idx) {
  int value = readBinaryU16(payload, idx);
  if ((value & 0x8000) != 0) value -= 0x10000;
  return value;
}

// Reads signed 8-bit value.
int readBinaryI8(byte[] payload, int idx) {
  if (payload == null || idx < 0 || idx >= payload.length) return 0;
  int value = payload[idx] & 0xFF;
  if ((value & 0x80) != 0) value -= 0x100;
  return value;
}

// Reads little-endian unsigned 32-bit value.
long readBinaryU32(byte[] payload, int idx) {
  if (payload == null || idx < 0 || idx + 3 >= payload.length) return 0;
  return ((long)(payload[idx] & 0xFF)) |
         ((long)(payload[idx + 1] & 0xFF) << 8) |
         ((long)(payload[idx + 2] & 0xFF) << 16) |
         ((long)(payload[idx + 3] & 0xFF) << 24);
}

// Reads little-endian signed 32-bit value.
int readBinaryI32(byte[] payload, int idx) {
  long value = readBinaryU32(payload, idx);
  if ((value & 0x80000000L) != 0) value -= 0x100000000L;
  return (int)value;
}

// Checks whether the binary payload still has the requested number of bytes.
boolean binaryPayloadHasBytes(int idx, int needed, int len) {
  return idx >= 0 && needed >= 0 && len >= (idx + needed);
}

// Parses binary runtime telemetry payload.
JSONObject parseBinaryRuntimeTelemetryPayload(byte[] payload, int len) {
  if (payload == null || len < 1) return null;
  JSONObject sens = new JSONObject();

  int idx = 0;
  int robotMode = payload[idx++] & 0xFF;
  String robotName = normalizeRobotTypeName(robotMode == 1 ? "Vehicle" : robotMode == 2 ? "Drone" : "Manipulator");
  sens.setString("robot", robotName);

  if (binaryPayloadHasBytes(idx, 4, len)) {
    long seq = readBinaryU32(payload, idx);
    idx += 4;
    sens.setFloat("seq", seq);
    noteHardwareStreamSequence(seq);
  }
  if (binaryPayloadHasBytes(idx, 4, len)) {
    sens.setFloat("stamp_ms", readBinaryU32(payload, idx));
    idx += 4;
  }
  if (binaryPayloadHasBytes(idx, 2, len)) {
    int linkMsRaw = readBinaryU16(payload, idx);
    idx += 2;
    sens.setFloat("link_ms", linkMsRaw >= 65535 ? -1 : linkMsRaw);
  }
  if (binaryPayloadHasBytes(idx, 4, len)) {
    long rxSeq = readBinaryU32(payload, idx);
    idx += 4;
    sens.setFloat("rxseq", rxSeq);
    noteHardwareStreamSequence(rxSeq);
  }
  int flags = 0;
  if (binaryPayloadHasBytes(idx, 1, len)) {
    flags = payload[idx++] & 0xFF;
  }

  if (robotMode == 0) {
    sens.setFloat("step_window_ok", (flags & 0x01) != 0 ? 1 : 0);
    sens.setFloat("gstab", (flags & 0x02) != 0 ? 1 : 0);
    sens.setFloat("astab", (flags & 0x04) != 0 ? 1 : 0);
    sens.setFloat("collision_enabled", (flags & 0x08) != 0 ? 1 : 0);
    sens.setFloat("collision", (flags & 0x10) != 0 ? 1 : 0);
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("grip_idle_ma", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("grip_pressure_ma", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("grip_pressure_est", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("base_member_deg", readBinaryU16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("upper_member_deg", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("fore_member_deg", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("forearm_roll_member_deg", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("wrist_pitch_member_deg", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("wrist_roll_member_deg", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("grip_member_deg", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { float sonarCm = readBinaryI16(payload, idx); sens.setFloat("sonar_cm", sonarCm); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("ina1_ma", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("ina2_ma", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("step_deg", readBinaryU16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("target_deg", readBinaryU16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("step_speed_dps", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("step_pwm_raw", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("mag_x", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("mag_y", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("mag_z", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("heading_deg", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("pwm_12", payload[idx++] & 0xFF); }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("pwm_13", payload[idx++] & 0xFF); }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("pwm_14", payload[idx++] & 0xFF); }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("pwm_15", payload[idx++] & 0xFF); }
    for (int i = 1; i <= 6 && binaryPayloadHasBytes(idx, 2, len); i++) {
      int analogValue = readBinaryU16(payload, idx); idx += 2;
      sens.setFloat("an" + i, analogValue);
      sens.setFloat("an_" + i, analogValue);
      sens.setFloat("an_" + (i - 1), analogValue);
    }
  } else if (robotMode == 1) {
    sens.setFloat("light", (flags & 0x01) != 0 ? 1 : 0);
    sens.setFloat("lights", (flags & 0x01) != 0 ? 1 : 0);
    sens.setFloat("lscan", (flags & 0x02) != 0 ? 1 : 0);
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("left_track_pct", readBinaryI8(payload, idx)); idx += 1; }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("right_track_pct", readBinaryI8(payload, idx)); idx += 1; }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("vehicle_cam_pan_deg", readBinaryI8(payload, idx)); idx += 1; }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("vehicle_cam_tilt_deg", readBinaryI8(payload, idx)); idx += 1; }
    if (binaryPayloadHasBytes(idx, 2, len)) {
      float lidarCm = readBinaryI16(payload, idx);
      sens.setFloat("sonar_cm", lidarCm);
      sens.setFloat("lidar_cm", lidarCm);
      sens.setFloat("range_cm", lidarCm);
      idx += 2;
    }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("ina1_ma", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("ina2_ma", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("vehicle_x", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("vehicle_z", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("vehicle_yaw_deg", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("heading_deg", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
  } else {
    sens.setFloat("camera_record", (flags & 0x01) != 0 ? 1 : 0);
    sens.setFloat("cam", (flags & 0x01) != 0 ? 1 : 0);
    sens.setFloat("camera", (flags & 0x01) != 0 ? 1 : 0);
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("throttle_pct", readBinaryI8(payload, idx)); idx += 1; }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("yaw_pct", readBinaryI8(payload, idx)); idx += 1; }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("pitch_pct", readBinaryI8(payload, idx)); idx += 1; }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("roll_pct", readBinaryI8(payload, idx)); idx += 1; }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("strafe_pct", readBinaryI8(payload, idx)); idx += 1; }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("forward_pct", readBinaryI8(payload, idx)); idx += 1; }
    if (binaryPayloadHasBytes(idx, 2, len)) {
      float scanCm = readBinaryI16(payload, idx);
      sens.setFloat("sonar_cm", scanCm);
      sens.setFloat("range_cm", scanCm);
      sens.setFloat("drone_scan_cm", scanCm);
      sens.setFloat("drone_sonar_down_cm", scanCm);
      sens.setFloat("sonar_down_cm", scanCm);
      idx += 2;
    }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("ina1_ma", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("ina2_ma", readBinaryI16(payload, idx)); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { float altCm = readBinaryI16(payload, idx) * 0.1f; sens.setFloat("alt_cm", altCm); sens.setFloat("height_cm", altCm); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("drone_x", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("drone_y", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    if (binaryPayloadHasBytes(idx, 2, len)) { sens.setFloat("drone_z", readBinaryI16(payload, idx) * 0.01f); idx += 2; }
    float yawDeg = 0.0f;
    float pitchDeg = 0.0f;
    float rollDeg = 0.0f;
    boolean haveYaw = false;
    if (binaryPayloadHasBytes(idx, 2, len)) { yawDeg = readBinaryI16(payload, idx) * 0.01f; idx += 2; haveYaw = true; sens.setFloat("drone_yaw_deg", yawDeg); sens.setFloat("att_yaw_deg", yawDeg); }
    if (binaryPayloadHasBytes(idx, 2, len)) { pitchDeg = readBinaryI16(payload, idx) * 0.01f; idx += 2; sens.setFloat("drone_pitch_deg", pitchDeg); }
    if (binaryPayloadHasBytes(idx, 2, len)) { rollDeg = readBinaryI16(payload, idx) * 0.01f; idx += 2; sens.setFloat("drone_roll_deg", rollDeg); }
    if (!haveYaw && sens.hasKey("att_yaw_deg")) sens.setFloat("drone_yaw_deg", getJsonFloat(sens, "att_yaw_deg", 0.0f));
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("drone_cam_pan_deg", readBinaryI8(payload, idx)); idx += 1; }
    if (binaryPayloadHasBytes(idx, 1, len)) { sens.setFloat("drone_cam_tilt_deg", readBinaryI8(payload, idx)); idx += 1; }
  }

  if (binaryPayloadHasBytes(idx, 28, len)) {
    sens.setFloat("mpu1_ax", readBinaryI16(payload, idx)); idx += 2;
    sens.setFloat("mpu1_ay", readBinaryI16(payload, idx)); idx += 2;
    sens.setFloat("mpu1_az", readBinaryI16(payload, idx)); idx += 2;
    sens.setFloat("mpu1_gx", readBinaryI16(payload, idx)); idx += 2;
    sens.setFloat("mpu1_gy", readBinaryI16(payload, idx)); idx += 2;
    sens.setFloat("mpu1_gz", readBinaryI16(payload, idx)); idx += 2;
    sens.setFloat("mpu2_ax", readBinaryI16(payload, idx)); idx += 2;
    sens.setFloat("mpu2_ay", readBinaryI16(payload, idx)); idx += 2;
    sens.setFloat("mpu2_az", readBinaryI16(payload, idx)); idx += 2;
    sens.setFloat("mpu2_gx", readBinaryI16(payload, idx)); idx += 2;
    sens.setFloat("mpu2_gy", readBinaryI16(payload, idx)); idx += 2;
    sens.setFloat("mpu2_gz", readBinaryI16(payload, idx)); idx += 2;
    float batteryRaw = readBinaryU16(payload, idx); idx += 2;
    float batteryPct = readBinaryU16(payload, idx) * 0.01f; idx += 2;
    sens.setFloat("battery_raw", batteryRaw);
    sens.setFloat("battery", batteryRaw);
    sens.setFloat("bat_pct", batteryPct);
    sens.setFloat("battery_pct", batteryPct);
  }

  if (robotMode == 2 && binaryPayloadHasBytes(idx, 6, len)) {
    float sonarVerticalCm = readBinaryI16(payload, idx) * 0.1f; idx += 2;
    float sonarGroundRefCm = readBinaryI16(payload, idx) * 0.1f; idx += 2;
    float sonarHeightCm = readBinaryI16(payload, idx) * 0.1f; idx += 2;
    sens.setFloat("sonar_vertical_cm", sonarVerticalCm);
    sens.setFloat("sonar_ground_ref_cm", sonarGroundRefCm);
    sens.setFloat("sonar_height_cm", sonarHeightCm);
    sens.setFloat("drone_height_from_sonar_cm", sonarHeightCm);
    sens.setFloat("alt_cm", sonarHeightCm);
    sens.setFloat("height_cm", sonarHeightCm);
  }

  if (robotMode == 1 && sens.hasKey("sonar_cm") && !sens.hasKey("lidar_cm")) {
    float lidarCm = getJsonFloat(sens, "sonar_cm", 0.0f);
    sens.setFloat("lidar_cm", lidarCm);
    sens.setFloat("range_cm", lidarCm);
  }
  if (robotMode == 2 && sens.hasKey("sonar_cm")) {
    float scanCm = getJsonFloat(sens, "sonar_cm", 0.0f);
    if (!sens.hasKey("range_cm")) sens.setFloat("range_cm", scanCm);
    if (!sens.hasKey("drone_scan_cm")) sens.setFloat("drone_scan_cm", scanCm);
    if (!sens.hasKey("drone_sonar_down_cm")) sens.setFloat("drone_sonar_down_cm", scanCm);
    if (!sens.hasKey("sonar_down_cm")) sens.setFloat("sonar_down_cm", scanCm);
  }

  return sens;
}

// Merges primitive JSON fields from src into target preserving basic types.
void mergeSensorSnapshotFields(JSONObject target, JSONObject src) {
  if (target == null || src == null) return;
  for (Object keyObj : src.keys()) {
    String key = keyObj == null ? "" : String.valueOf(keyObj);
    if (key.length() == 0) continue;

    JSONObject nested = getJsonObjectSafe(src, key);
    if (nested != null) {
      target.setJSONObject(key, nested);
      continue;
    }

    if (key.equals("robot") || key.equals("robot_type") || key.endsWith("_mode") || key.endsWith("_type")) {
      try { target.setString(key, src.getString(key)); continue; } catch (Exception ignored) {}
    }

    try {
      target.setFloat(key, src.getFloat(key));
      continue;
    }
    catch (Exception e1) {
    }
    try {
      target.setInt(key, src.getInt(key));
      continue;
    }
    catch (Exception e2) {
    }
    try {
      target.setBoolean(key, src.getBoolean(key));
      continue;
    }
    catch (Exception e3) {
    }
    try {
      target.setString(key, src.getString(key));
    }
    catch (Exception e4) {
    }
  }
}


// Utility: synthesize manipulator servo angles from member telemetry.
void synthesizeManipulatorServoAnglesFromMembers(JSONObject sens) {
  if (sens == null) return;
  boolean hasMemberTelemetry =
    sens.hasKey("base_member_deg") || sens.hasKey("upper_member_deg") || sens.hasKey("fore_member_deg") ||
    sens.hasKey("forearm_roll_member_deg") || sens.hasKey("wrist_pitch_member_deg") || sens.hasKey("wrist_roll_member_deg") ||
    sens.hasKey("grip_member_deg");
  if (!hasMemberTelemetry) return;

  // In firmware-runtime mode the 0x20 payload is authoritative in member space.
  // Recompute the UI/3D servo aliases on every packet. Do not keep previous
  // ARMANG/bootstrap values, otherwise the sensor table updates but the angle
  // hub and the 3D manipulator remain stuck on the first pose.
  if (sens.hasKey("base_member_deg")) {
    float member = getJsonFloat(sens, "base_member_deg", 0.0f);
    sens.setFloat("base_deg", armMemberBaseToUiServoDeg(member));
    sens.setFloat("base_yaw_deg", normalizeAbsoluteAngleDeg(member));
  }
  if (sens.hasKey("upper_member_deg")) {
    float member = getJsonFloat(sens, "upper_member_deg", 0.0f);
    sens.setFloat("upper_deg", armMemberUpperToUiServoDeg(member));
    sens.setFloat("upper_ground_deg", normalizeSignedAngleDeg(member));
  }
  if (sens.hasKey("fore_member_deg")) {
    float member = getJsonFloat(sens, "fore_member_deg", 0.0f);
    sens.setFloat("fore_deg", armMemberForeToUiServoDeg(member));
  }
  if (sens.hasKey("forearm_roll_member_deg")) {
    sens.setFloat("forearm_roll_deg", armMemberToUiServoDeg(FOREARM_ROLL_IDX, getJsonFloat(sens, "forearm_roll_member_deg", 0.0f)));
  }
  if (sens.hasKey("wrist_pitch_member_deg")) {
    float member = getJsonFloat(sens, "wrist_pitch_member_deg", 0.0f);
    sens.setFloat("wrist_pitch_deg", armMemberWristPitchToUiServoDeg(member));
  }
  if (sens.hasKey("wrist_roll_member_deg")) {
    sens.setFloat("wrist_rot_deg", armMemberWristRollToUiServoDeg(getJsonFloat(sens, "wrist_roll_member_deg", 0.0f)));
  }
  if (sens.hasKey("grip_member_deg")) {
    sens.setFloat("grip_deg", armMemberToUiServoDeg(GRIPPER_IDX, getJsonFloat(sens, "grip_member_deg", 0.0f)));
  }

  float upperGround = getJsonFloat(sens, "upper_member_deg", getJsonFloat(sens, "upper_ground_deg", 0.0f));
  float foreDelta = getJsonFloat(sens, "fore_member_deg", 0.0f);
  float wristPitchDelta = getJsonFloat(sens, "wrist_pitch_member_deg", 0.0f);
  if (sens.hasKey("upper_member_deg") || sens.hasKey("fore_member_deg")) {
    sens.setFloat("fore_ground_deg", normalizeSignedAngleDeg(upperGround + foreDelta));
  }
  if (sens.hasKey("upper_member_deg") || sens.hasKey("fore_member_deg") || sens.hasKey("wrist_pitch_member_deg")) {
    sens.setFloat("wrist_ground_deg", normalizeSignedAngleDeg(upperGround + foreDelta + wristPitchDelta));
  }

  // Aliases consumed by the live angle hub / diagnostics recording.
  publishManipulatorAngleHubAliases(sens);
}

// Publishes live/target manipulator aliases in the active runtime snapshot.
void publishManipulatorAngleHubAliases(JSONObject sens) {
  if (sens == null) return;
  String[] liveKeys = {
    "arm_live_base_deg", "arm_live_upper_deg", "arm_live_fore_deg", "arm_live_forearm_roll_deg",
    "arm_live_wrist_pitch_deg", "arm_live_wrist_rot_deg", "arm_live_grip_deg"
  };
  String[] servoKeys = {
    "base_deg", "upper_deg", "fore_deg", "forearm_roll_deg", "wrist_pitch_deg", "wrist_rot_deg", "grip_deg"
  };
  for (int i = 0; i < min(liveKeys.length, servoKeys.length); i++) {
    if (sens.hasKey(servoKeys[i])) sens.setFloat(liveKeys[i], getJsonFloat(sens, servoKeys[i], 0.0f));
  }

  String[] targetKeys = {
    "arm_target_base_deg", "arm_target_upper_deg", "arm_target_fore_deg", "arm_target_forearm_roll_deg",
    "arm_target_wrist_pitch_deg", "arm_target_wrist_rot_deg", "arm_target_grip_deg"
  };
  for (int i = 0; i < min(targetKeys.length, angles.length); i++) {
    sens.setFloat(targetKeys[i], angles[i]);
  }
}

// Parses the optional GPS telemetry payload (frame 0x21).
JSONObject parseBinaryGpsTelemetryPayload(byte[] payload, int len) {
  if (payload == null || len < 25) return null;
  int idx = 0;
  int robotMode = payload[idx++] & 0xFF;
  long stampMs = readBinaryU32(payload, idx); idx += 4;
  int flags = payload[idx++] & 0xFF;
  int satellites = payload[idx++] & 0xFF;
  int hdopX100 = readBinaryU16(payload, idx); idx += 2;
  int latE7 = readBinaryI32(payload, idx); idx += 4;
  int lonE7 = readBinaryI32(payload, idx); idx += 4;
  int altCm = readBinaryI32(payload, idx); idx += 4;
  int speedCms = readBinaryU16(payload, idx); idx += 2;
  int courseX100 = readBinaryU16(payload, idx);

  String robotName = robotMode == 1 ? "Vehicle" : (robotMode == 2 ? "Drone" : "Manipulator");
  JSONObject gps = new JSONObject();
  gps.setString("robot", robotName);
  gps.setInt("gps_stamp_ms", (int)stampMs);
  gps.setInt("gps_flags", flags);
  gps.setInt("gps_port_ready", (flags & 0x01) != 0 ? 1 : 0);
  gps.setInt("gps_fix_valid", (flags & 0x02) != 0 ? 1 : 0);
  gps.setInt("gps_fix", (flags & 0x02) != 0 ? 1 : 0);
  gps.setInt("gps_fresh", (flags & 0x04) != 0 ? 1 : 0);
  gps.setInt("gps_sentence_valid", (flags & 0x08) != 0 ? 1 : 0);
  gps.setInt("gps_satellites", satellites);
  gps.setFloat("gps_hdop", hdopX100 / 100.0f);
  gps.setFloat("gps_lat", latE7 / 10000000.0f);
  gps.setFloat("gps_lon", lonE7 / 10000000.0f);
  gps.setFloat("gps_alt_m", altCm / 100.0f);
  gps.setFloat("gps_speed_kph", speedCms * 0.036f);
  gps.setFloat("gps_course_deg", courseX100 / 100.0f);
  gps.setJSONObject("gps", buildGpsBlockFromSensors(gps));
  return gps;
}

// Consumes optional GPS telemetry without changing the main 0x20 snapshot.
void consumeBinaryGpsTelemetry(byte[] payload, int len) {
  noteBinaryRuntimeObservedOnAppPort();
  lastHardwareSensorMillis = millis();
  hardwareTelemetryHealthy = true;
  hardwareTelemetryTimeoutLatched = false;
  JSONObject gps = parseBinaryGpsTelemetryPayload(payload, len);
  if (gps == null) return;
  String robotName = getJsonString(gps, "robot", currentModeName());
  JSONObject merged = getOrCreateRobotSensorSnapshot(robotName);
  mergeSensorSnapshotFields(merged, gps);
  merged.setString("robot", robotName);
  storeRobotSensorSnapshot(robotName, merged);
  setDetectedRobotType(robotName);
  syncLatestSensorsFromActiveSelection();
  sendLatestSensorPacket();
}

// Consumes binary runtime telemetry payload.
void consumeBinaryRuntimeTelemetry(byte[] payload, int len) {
  noteBinaryRuntimeObservedOnAppPort();
  // Mark the stream alive as soon as a well-formed 0x20 frame arrives,
  // before parsing, so a partial-parse failure can't cause a false timeout.
  lastHardwareSensorMillis = millis();
  hardwareTelemetryHealthy = true;
  hardwareTelemetryTimeoutLatched = false;

  JSONObject sens = parseBinaryRuntimeTelemetryPayload(payload, len);
  if (sens == null) return;
  String robotName = getJsonString(sens, "robot", currentModeName());
  if (robotName.equalsIgnoreCase("Manipulator")) {
    synthesizeManipulatorServoAnglesFromMembers(sens);
  }
  JSONObject merged = getOrCreateRobotSensorSnapshot(robotName);
  mergeSensorSnapshotFields(merged, sens);
  if (robotName.equalsIgnoreCase("Manipulator")) {
    synthesizeManipulatorServoAnglesFromMembers(merged);
  }
  merged.setString("robot", robotName);
  storeRobotSensorSnapshot(robotName, merged);
  setDetectedRobotType(robotName);
  syncLatestSensorsFromActiveSelection();
  appendDiagnosticsSensorHistorySnapshot(merged);
  sendLatestSensorPacket();
}

// Handles runtime binary frame.
void handleRuntimeBinaryFrame(int type, byte[] payload, int len, String sourceLabel) {
  // Only log non-telemetry frames; logging 0x20/0x21 at telemetry rate
  // (20Hz) floods the monitor buffer and degrades UI performance.
  int t = type & 0xFF;
  if (t != 0x20 && t != 0x21) {
    appendCalibrationBinaryFrameLog("RX", type, payload, len);
  }

  if (t == 0x81) {
    noteBinaryRuntimeObservedOnAppPort();
    String ackText = "";
    if (payload != null && len > 0) ackText = new String(subset(payload, 0, len));
    markHexRuntimeReady(sourceLabel);
    updateMessage(tr("Runtime HEX confirmado", "HEX runtime confirmed") + (ackText.length() > 0 ? ": " + ackText : "."));
    return;
  }
  if (t == 0x30) {
    markHexRuntimeReady(sourceLabel);
    String line = payload == null || len <= 0 ? "" : new String(subset(payload, 0, len));
    handleAppSerialAsciiLine(line);
    return;
  }
  if (t == 0x20) {
    markHexRuntimeReady(sourceLabel);
    if (serialMonitorSessionActive) {
      endSerialMonitorSession();
    }
    consumeBinaryRuntimeTelemetry(payload, len);
    return;
  }
  if (t == 0x21) {
    markHexRuntimeReady(sourceLabel);
    consumeBinaryGpsTelemetry(payload, len);
    return;
  }
  appendCalibrationSerialLine("[INFO] Binary frame received: " + describeBinaryFrameType(type) + " (" + len + " bytes payload).");
}

// Utility: handle binary frame on app port.
void handleAppSerialBinaryFrame(int type, byte[] payload, int len) {
  if (isHexCommunicationSelected()) {
    handleRuntimeBinaryFrame(type, payload, len, "app port");
  } else {
    appendCalibrationBinaryFrameLog("RX", type, payload, len);
    appendCalibrationSerialLine("[INFO] Binary frame received on app port: " + describeBinaryFrameType(type) + " (" + len + " bytes payload).");
  }
}

// Utility: handle binary frame on dedicated calibration port.
void handleCalibrationSerialBinaryFrame(int type, byte[] payload, int len) {
  if (isHexCommunicationSelected() && calibrationPort != null) {
    handleRuntimeBinaryFrame(type, payload, len, calibrationPortName);
  } else {
    appendCalibrationBinaryFrameLog("RX", type, payload, len);
    appendCalibrationSerialLine("[INFO] Dedicated monitor frame: " + describeBinaryFrameType(type) + " (" + len + " bytes payload).");
  }
}

// Raw-binary byte transport is disabled. Runtime uses A55A HEX lines only.

// Utility: record ASCII overflow and keep line framing synchronized.
void noteSerialAsciiOverflow(boolean appPort, int bufferedLength) {
  String portLabel = appPort ? "APP" : "CAL";
  if (appPort) {
    mainSerialAsciiOverflowCount++;
  } else {
    calibrationSerialAsciiOverflowCount++;
  }

  String msg = "[WARN] " + portLabel
    + " ASCII packet exceeded " + SERIAL_ASCII_LINE_MAX
    + " bytes and was discarded (buffered=" + bufferedLength + ")";
  appendCalibrationSerialLine(msg);
  if (appPort) {
    sendWsSerialLine(msg);
  }
}

// Utility: process incoming ASCII byte.
void processIncomingAsciiByte(int unsignedByte, boolean appPort) {
  StringBuilder buffer = appPort ? mainSerialAsciiBuffer : calibrationSerialAsciiBuffer;
  char ch = (char)(unsignedByte & 0xFF);
  boolean discarding = appPort ? mainSerialAsciiDiscarding : calibrationSerialAsciiDiscarding;

  if (ch == '\n' || ch == '\r') {
    if (discarding) {
      int bufferedLength = buffer.length();
      buffer.setLength(0);
      if (appPort) {
        mainSerialAsciiDiscarding = false;
      } else {
        calibrationSerialAsciiDiscarding = false;
      }
      noteSerialAsciiOverflow(appPort, bufferedLength);
      return;
    }

    if (buffer.length() > 0) {
      String line = buffer.toString();
      buffer.setLength(0);
      if (appPort) {
        handleAppSerialAsciiLine(line);
      } else {
        handleCalibrationSerialAsciiLine(line);
      }
    }
    return;
  }

  boolean printable = (unsignedByte >= 32 && unsignedByte <= 126) || unsignedByte == 9;
  if (!printable) {
    long resyncUntil = appPort ? mainSerialAsciiResyncUntilMs : calibrationSerialAsciiResyncUntilMs;
    boolean inResyncWindow = millis() <= resyncUntil;
    if (!inResyncWindow || buffer.length() == 0) {
      buffer.setLength(0);
      if (appPort) {
        mainSerialAsciiDiscarding = false;
      } else {
        calibrationSerialAsciiDiscarding = false;
      }
    }
    return;
  }

  if (discarding) {
    return;
  }

  if (buffer.length() >= SERIAL_ASCII_LINE_MAX) {
    if (appPort) {
      mainSerialAsciiDiscarding = true;
    } else {
      calibrationSerialAsciiDiscarding = true;
    }
    return;
  }

  buffer.append(ch);
}

// Runtime transport selector removed: every phase is parsed as newline-terminated ASCII/A55A HEX line.

// Utility: process serial bytes.
void processIncomingSerialBytes(Serial port, byte[] chunk, boolean appPort) {
  if (port == null || chunk == null || chunk.length == 0) return;

  if (appPort) {
    lastHardwareRxMillis = millis();
    if (hardwareStreamStoppedByExit && !serialMonitorSessionActive) {
      hardwareStreamStoppedByExit = false;
      hardwareStreamStoppedAtMillis = 0;
      hardwareTelemetryTimeoutLatched = false;
    }
  }

  // Current firmware transport is line-oriented on every phase:
  // boot/config use plain ASCII, runtime uses HEX frames encoded as ASCII lines.
  // So the receiver should always assemble newline-terminated text lines first.
  for (int i = 0; i < chunk.length; i++) {
    int unsignedByte = chunk[i] & 0xFF;
    processIncomingAsciiByte(unsignedByte, appPort);
  }
}

// Utility: drain already-buffered serial bytes for the line-oriented HEX transport.
void drainBinarySerialPort(Serial port) {
  if (port == null) return;
  if (port != myPort && port != calibrationPort) return;
  if (port.available() <= 0) return;

  byte[] chunk = null;
  try {
    chunk = port.readBytes();
  }
  catch (Throwable t) {
    String detail = t.getMessage() == null ? t.toString() : t.getMessage();
    if (port == calibrationPort) {
      appendCalibrationSerialLine("[ERROR] Dedicated monitor port read failed: " + detail);
      disconnectCalibrationMonitorPort();
    } else {
      handleMainSerialDisconnect(detail);
    }
    return;
  }

  if (chunk == null || chunk.length == 0) return;
  synchronized (serialRxLock) {
    processIncomingSerialBytes(port, chunk, port == myPort);
  }
}

// Utility: serial event.
void serialEvent(Serial port) {
  if (port == null) return;
  if (port != myPort && port != calibrationPort) return;

  byte[] chunk = null;
  try {
    chunk = port.readBytes();
  }
  catch (Throwable t) {
    String detail = t.getMessage() == null ? t.toString() : t.getMessage();
    if (port == calibrationPort) {
      appendCalibrationSerialLine("[ERROR] Dedicated monitor port read failed: " + detail);
      disconnectCalibrationMonitorPort();
    } else {
      handleMainSerialDisconnect(detail);
    }
    return;
  }

  if (chunk == null || chunk.length == 0) return;
  boolean appPort = (port == myPort);
  synchronized (serialRxLock) {
    // Both the initial chunk and any extra bytes are in one try-catch so
    // an exception from a corrupted buffer can't silently kill the handler.
    try {
      processIncomingSerialBytes(port, chunk, appPort);
      while (port.available() > 0) {
        byte[] extra = port.readBytes();
        if (extra == null || extra.length == 0) break;
        processIncomingSerialBytes(port, extra, port == myPort);
      }
    }
    catch (Throwable t) {
      String detail = t.getMessage() == null ? t.toString() : t.getMessage();
      if (port == calibrationPort) {
        appendCalibrationSerialLine("[ERROR] Dedicated monitor port error in serialEvent: " + detail);
        disconnectCalibrationMonitorPort();
      } else {
        handleMainSerialDisconnect(detail);
      }
    }
  }
}

// Sets boolean or int.
void setBooleanOrInt(JSONObject sens, String key, String value) {
  if (value == null) return;
  String v = trim(value);
  if (v.equalsIgnoreCase("true") || v.equalsIgnoreCase("false")) {
    sens.setBoolean(key, v.equalsIgnoreCase("true"));
    return;
  }

  try {
    sens.setInt(key, int(v));
  }
  catch (Exception e) {
    sens.setString(key, v);
  }
}

// Sets number or text.
void setNumberOrText(JSONObject sens, String key, String value) {
  if (value == null) return;
  String v = trim(value);
  if (v.length() == 0) return;

  try {
    if (v.indexOf('.') >= 0) {
      sens.setFloat(key, float(v));
    } else {
      sens.setInt(key, int(v));
    }
  }
  catch (Exception e) {
    sens.setString(key, v);
  }
}

// Utility: put csv values.
void putCsvValues(JSONObject sens, String prefix, String csv, boolean preferFloat, int baseIndex) {
  if (csv == null || csv.length() == 0) return;
  String[] values = split(csv, ',');
  for (int i = 0; i < values.length; i++) {
    String key = prefix + (i + baseIndex);
    String item = trim(values[i]);
    if (preferFloat) {
      try {
        sens.setFloat(key, float(item));
      }
      catch (Exception e) {
        setNumberOrText(sens, key, item);
      }
    } else {
      setNumberOrText(sens, key, item);
    }
  }
}

// Parses telemetry packet.
JSONObject parseTelemetryPacket(String raw, int prefixLen, boolean appendHistory) {
  String payload = raw.substring(prefixLen);
  String[] blocks = split(payload, '|');

  String incomingRobot = "";
  int incomingSeq = -1;
  for (String b : blocks) {
    if (b == null || b.length() == 0) continue;
    int sep = findTelemetryBlockSeparator(b);
    if (sep < 0) continue;

    String key = telemetryBlockKey(b);
    String value = telemetryBlockValue(b);
    if (key.equals("ROBOT")) {
      incomingRobot = normalizeRobotTypeName(value);
    } else if (key.equals("SEQ")) {
      try {
        incomingSeq = int(value);
        noteHardwareStreamSequence((long)incomingSeq);
      }
      catch (Exception e) {
        incomingSeq = -1;
      }
    }
  }

  if (incomingRobot.length() == 0) {
    incomingRobot = normalizeRobotTypeName(pendingRobotTypeHint);
  }
  if (incomingRobot.length() == 0) {
    incomingRobot = normalizeRobotTypeName(currentModeName());
  }
  if (incomingRobot.length() == 0) {
    incomingRobot = "Manipulator";
  }

  JSONObject cached = getRobotSensorSnapshot(incomingRobot);
  int previousSeq = getSensorJsonInt(cached, "seq", -1);
  boolean resetSnapshot = (cached == null);
  if (!resetSnapshot && incomingSeq >= 0 && previousSeq >= 0 && incomingSeq < previousSeq) {
    resetSnapshot = true;
  }

  JSONObject sens = resetSnapshot ? new JSONObject() : getOrCreateRobotSensorSnapshot(incomingRobot);
  sens.setString("robot", incomingRobot);

  for (String b : blocks) {
    if (b == null || b.length() == 0) continue;

    int sep = findTelemetryBlockSeparator(b);
    if (sep < 0) continue;

    String key = telemetryBlockKey(b);
    String value = telemetryBlockValue(b);

    if (key.equals("MPU1") || key.equals("MPU2")) {
      String base = key.toLowerCase() + "_";
      String[] v = split(value, ',');
      if (v.length >= 3) {
        setNumberOrText(sens, base + "ax", v[0]);
        setNumberOrText(sens, base + "ay", v[1]);
        setNumberOrText(sens, base + "az", v[2]);
      }
      if (v.length >= 6) {
        setNumberOrText(sens, base + "gx", v[3]);
        setNumberOrText(sens, base + "gy", v[4]);
        setNumberOrText(sens, base + "gz", v[5]);
      }
    } else if (key.equals("GYR1") || key.equals("GYR2")) {
      String base = key.toLowerCase() + "_";
      String[] v = split(value, ',');
      if (v.length >= 3) {
        setNumberOrText(sens, base + "x", v[0]);
        setNumberOrText(sens, base + "y", v[1]);
        setNumberOrText(sens, base + "z", v[2]);
      }
    } else if (key.equals("AN")) {
      putCsvValues(sens, "an_", value, false, 0);
      String[] v = split(value, ',');
      for (int i = 0; i < v.length; i++) {
        String item = trim(v[i]);
        setNumberOrText(sens, "an" + (i + 1), item);
        setNumberOrText(sens, "an_" + (i + 1), item);
      }
    } else if (key.startsWith("AN") && key.length() > 2) {
      try {
        int analogIndex = Integer.parseInt(key.substring(2));
        setNumberOrText(sens, "an" + analogIndex, value);
        setNumberOrText(sens, "an_" + analogIndex, value);
        if (analogIndex > 0) {
          setNumberOrText(sens, "an_" + (analogIndex - 1), value);
        }
      }
      catch (Exception e) {
        setNumberOrText(sens, key.toLowerCase(), value);
      }
    } else if (key.equals("PWM")) {
      value = value.replace("%", "");
      putCsvValues(sens, "pwm_", value, false, 12);
    } else if (key.equals("ARMPOS")) {
      String[] v = split(value, ',');
      if (v.length >= 3) {
        setNumberOrText(sens, "arm_base_cylinder_y", v[0]);
        setNumberOrText(sens, "arm_base_block_y", v[1]);
        setNumberOrText(sens, "arm_gripper_y", v[2]);
      }
    } else if (key.equals("ARMSZ")) {
      String[] v = split(value, ',');
      String[] names = {
        "arm_base_radius", "arm_base_height",
        "arm_base_block_w", "arm_base_block_h", "arm_base_block_d",
        "arm_upper_w", "arm_upper_h", "arm_upper_d",
        "arm_fore_w", "arm_fore_h", "arm_fore_d",
        "arm_wrist_w", "arm_wrist_h", "arm_wrist_d",
        "arm_finger_w", "arm_finger_h", "arm_finger_d"
      };
      for (int i = 0; i < min(v.length, names.length); i++) {
        setNumberOrText(sens, names[i], v[i]);
      }
    } else if (key.equals("PRESS")) {
      String[] v = split(value, ',');
      if (v.length >= 1) setNumberOrText(sens, "grip_idle_ma", v[0]);
      if (v.length >= 2) setNumberOrText(sens, "grip_pressure_ma", v[1]);
      if (v.length >= 3) setNumberOrText(sens, "grip_pressure_est", v[2]);
    } else if (key.equals("ARMANG")) {
      String[] v = split(value, ',');
      String[] names = {"base_deg", "upper_deg", "fore_deg", "forearm_roll_deg", "wrist_pitch_deg", "wrist_rot_deg", "grip_deg"};
      for (int i = 0; i < min(v.length, names.length); i++) {
        setNumberOrText(sens, names[i], v[i]);
      }
    } else if (key.equals("ARMGEO")) {
      String[] v = split(value, ',');
      String[] names = {"base_yaw_deg", "upper_ground_deg", "fore_ground_deg", "wrist_ground_deg"};
      for (int i = 0; i < min(v.length, names.length); i++) {
        setNumberOrText(sens, names[i], v[i]);
      }
    } else if (key.equals("ARMJ")) {
      String[] v = split(value, ',');
      String[] names = {"base_member_deg", "upper_member_deg", "fore_member_deg", "forearm_roll_member_deg", "wrist_pitch_member_deg", "wrist_roll_member_deg", "grip_member_deg"};
      for (int i = 0; i < min(v.length, names.length); i++) {
        setNumberOrText(sens, names[i], v[i]);
      }
      if (v.length >= 11) {
        for (int i = 0; i < 4; i++) {
          setNumberOrText(sens, "pwm_" + (12 + i), v[7 + i]);
        }
      }
    } else if (key.equals("ARMMAP")) {
      String[] v = split(value, ',');
      String[] names = {"arm_map_base_channel", "arm_map_upper_channel", "arm_map_fore_channel", "arm_map_forearm_roll_channel", "arm_map_wrist_pitch_channel", "arm_map_wrist_rot_channel", "arm_map_gripper_channel"};
      for (int i = 0; i < min(v.length, names.length); i++) {
        setNumberOrText(sens, names[i], v[i]);
      }
    } else if (key.equals("ARMOFF")) {
      if (applyManipulatorArmjOffsetsCsv(value)) {
        sens.setString("armoff", value);
      }
    } else if (key.equals("BASESRC")) {
      setNumberOrText(sens, "base_source_mode", value);
    } else if (key.equals("MAG")) {
      String[] v = split(value, ',');
      if (v.length >= 1) setNumberOrText(sens, "mag_x", v[0]);
      if (v.length >= 2) setNumberOrText(sens, "mag_y", v[1]);
      if (v.length >= 3) setNumberOrText(sens, "mag_z", v[2]);
      if (v.length >= 4) setNumberOrText(sens, "heading_deg", v[3]);
      if (v.length >= 5) setNumberOrText(sens, "mag_addr", v[4]);
      if (v.length >= 6) setNumberOrText(sens, "mag_type", v[5]);
    } else if (key.equals("MAGCAL_ACTIVE")) {
      setBooleanOrInt(sens, "mag_cal_active", value);
      setBooleanOrInt(sens, "magcal_active", value);
    } else if (key.equals("MAGCAL_PROGRESS")) {
      setNumberOrText(sens, "mag_cal_progress", value);
      setNumberOrText(sens, "magcal_progress", value);
    } else if (key.equals("ARMLIM") || key.equals("SRVLIM")) {
      applyManipulatorArmLimitsCsvToSnapshot(sens, value);
    } else if (key.equals("SONAR")) {
      setNumberOrText(sens, "sonar_cm", value);
    } else if (key.equals("BATTERY")) {
      setNumberOrText(sens, "battery_raw", value);
      setNumberOrText(sens, "battery", value);
    } else if (key.equals("INA1")) {
      setNumberOrText(sens, "ina1_ma", value);
    } else if (key.equals("INA2")) {
      setNumberOrText(sens, "ina2_ma", value);
    } else if (key.equals("STEP")) {
      setNumberOrText(sens, "step_deg", value);
    } else if (key.equals("TGT")) {
      setNumberOrText(sens, "target_deg", value);
    } else if (key.equals("SPD")) {
      setNumberOrText(sens, "step_speed_dps", value);
    } else if (key.equals("PWMRAW")) {
      setNumberOrText(sens, "step_pwm_raw", value);
    } else if (key.equals("WIN")) {
      setBooleanOrInt(sens, "step_window_ok", value);
    } else if (key.equals("GSTAB")) {
      setBooleanOrInt(sens, "gstab", value);
    } else if (key.equals("ASTAB")) {
      setBooleanOrInt(sens, "astab", value);
    } else if (key.equals("COLL_EN") || key.equals("COLLEN") || key.equals("COLLISION_ENABLED")) {
      setBooleanOrInt(sens, "collision_enabled", value);
    } else if (key.equals("COLL")) {
      setBooleanOrInt(sens, "collision", value);
    } else if (key.equals("RXSEQ")) {
      setNumberOrText(sens, "rxseq", value);
      try {
        noteHardwareStreamSequence(Long.parseLong(value));
      }
      catch (Exception e) {
      }
    } else if (key.equals("EX1") || key.equals("EX2")) {
      putCsvValues(sens, key.toLowerCase() + "_", value, false, 0);
    } else {
      setNumberOrText(sens, key.toLowerCase(), value);
    }
  }

  if (incomingRobot.equalsIgnoreCase("Manipulator")) {
    synthesizeManipulatorServoAnglesFromMembers(sens);
  }
  storeRobotSensorSnapshot(incomingRobot, sens);
  setDetectedRobotType(incomingRobot);
  syncLatestSensorsFromActiveSelection();
  if (appendHistory) {
    appendDiagnosticsSensorHistorySnapshot(sens);
  }
  return sens;
}

// Parses sensor packet.
JSONObject parseSensorPacket(String raw) {
  int prefixLen = telemetryAsciiPrefixLength(raw);
  if (raw == null || prefixLen <= 0) return new JSONObject();
  return parseTelemetryPacket(raw, prefixLen, true);
}

// Parses static config packet.
JSONObject parseStaticConfigPacket(String raw) {
  if (raw == null) return new JSONObject();
  int prefixLen = raw.startsWith("#CFG|") ? 5 : telemetryAsciiPrefixLength(raw);
  if (prefixLen <= 0) prefixLen = 5;
  return parseTelemetryPacket(raw, prefixLen, false);
}


// Loads last port.
String loadLastPort() {
  File dataDir = new File(sketchPath("data"));
  if (!dataDir.exists()) {
    dataDir.mkdirs();
  }

  String[] lines = loadStrings(LAST_PORT);
  if (lines != null && lines.length > 0) {
    return lines[0].trim();
  }
  return null;
}

// Saves last port.
void saveLastPort(String port) {
  File dataDir = new File(sketchPath("data"));
  if (!dataDir.exists()) {
    dataDir.mkdirs();
  }
  String[] data = {
    port
  };
  saveStrings(LAST_PORT, data);
}

// Utility: simulation mode.
void simulationMode() {
  if (diagnosticsSensorRecording) stopDiagnosticsSensorRecording();
  simulationMode = true;
  systemReady = false;
  clearRobotSensorSnapshots();
  clearPendingAppSerialAsciiLines();
  clearAllEnvironmentMaps();
  resetPendingManipulatorPoseCommand();
  resetPendingVehicleRuntimeCommand();
  resetPendingDroneRuntimeCommand();
  remoteCollisionEnabled = false;
  firmwareCollisionRuntimeEnabled = false;
  manualRobotModeOverride = false;
  updateMessage(tr("Modo de simulação ativado.", "Simulation Mode Enabled"));
  initializationStep = 5;
  cancelPendingAutoHome();
  goingHome = true;
}


// Parses control packet.
void parseControlPacket(String raw) {
  int prefixLen = raw.startsWith("#CFG|") ? 5 : 6;
  String payload = raw.substring(prefixLen);
  String[] blocks = split(payload, '|');

  for (String b : blocks) {
    int sep = findTelemetryBlockSeparator(b);
    if (sep < 0) continue;

    String key = telemetryBlockKey(b);
    String value = telemetryBlockValue(b);

    if (key.equals("CLIENT")) {
      webClientConnected = value.equals("1") || value.equalsIgnoreCase("true");
      if (webClientConnected) {
        lastWebHeartbeatMillis = millis();
      }
    } else if (key.equals("SRC")) {
      if (value.equalsIgnoreCase("WEB")) {
        currentControlOwner = CONTROL_SOURCE_WEB;
      } else if (value.equalsIgnoreCase("SERIAL") || value.equalsIgnoreCase("SERIAL0")) {
        currentControlOwner = CONTROL_SOURCE_LOCAL;
      } else {
        currentControlOwner = CONTROL_SOURCE_NONE;
      }
    }
  }

  sendSystemStatus();
}

// Parses PID packet.
void parsePidPacket(String raw) {
  String payload = raw.substring(5);
  String[] vals = split(payload, ',');
  if (vals.length >= 3) {
    try {
      hardwarePidKp = float(trim(vals[0]));
      hardwarePidKi = float(trim(vals[1]));
      hardwarePidKd = float(trim(vals[2]));
      sendPidStatePacket();
    }
    catch (Exception e) {
    }
  }
}


// =====================================================================
// SynROV Processing - Runtime bridge
// ---------------------------------------------------------------------
// Purpose:
//   Data exchange between Processing runtime state and external interfaces.
// =====================================================================

float mpu1RollDeg = 0;
float mpu1PitchDeg = 0;
float mpu1YawDeg = 0;
float mpu2RollDeg = 0;
float mpu2PitchDeg = 0;
float mpu2YawDeg = 0;

float[][] mpuAccelRaw = {{0, 0, 16384}, {0, 0, 16384}};
float[][] mpuGyroRaw = {{0, 0, 0}, {0, 0, 0}};
float[][] mpuAccelG = {{0, 0, 1}, {0, 0, 1}};
float[][] mpuGyroDps = {{0, 0, 0}, {0, 0, 0}};
float[][] mpuMotionBaselineG = {{0, 0, 1}, {0, 0, 1}};
float[][] mpuVelocityCms = {{0, 0, 0}, {0, 0, 0}};
float[] mpuVelocityMagCms = {0, 0};
long lastImuDerivedUpdateMs = 0;

// Checks whether diagnostics panel visible.
boolean isDiagnosticsPanelVisible() {
  return diagnosticsPanelVisible();
}

// Utility: enable simulation mode.
void enableSimulationMode() {
  simulationMode();
}


// Builds local telemetry snapshot for robot.
JSONObject buildLocalTelemetrySnapshotForRobot(String robotName) {
  String normalized = normalizeRobotTypeName(robotName);
  if (normalized.length() == 0) normalized = normalizeRobotTypeName(currentModeName());
  if (normalized.length() == 0) normalized = "Manipulator";

  JSONObject sens = new JSONObject();
  sens.setString("robot", normalized);

  if (normalized.equals("Manipulator")) {
    sens.setFloat("base_deg", angles[BASE_IDX]);
    sens.setFloat("base_yaw_deg", baseServoToForwardYawDeg(angles[BASE_IDX]));
    sens.setFloat("upper_deg", angles[UPPERARM_IDX]);
    sens.setFloat("fore_deg", angles[FOREARM_IDX]);
    sens.setFloat("forearm_roll_deg", angles[FOREARM_ROLL_IDX]);
    sens.setFloat("wrist_pitch_deg", angles[WRIST_VERT_IDX]);
    sens.setFloat("wrist_rot_deg", angles[WRIST_ROT_IDX]);
    sens.setFloat("grip_deg", angles[GRIPPER_IDX]);
    sens.setFloat("heading_deg", mpu1YawDeg);
    sens.setFloat("sonar_cm", sonarDistanceCm);
    sens.setFloat("ina1_ma", ina1mA);
    sens.setFloat("ina2_ma", ina2mA);
    sens.setFloat("gstab", gimbalStabEnabled ? 1 : 0);
    sens.setFloat("astab", armStabEnabled ? 1 : 0);
    sens.setFloat("collision", isColliding ? 1 : 0);
    sens.setFloat("collision_enabled", collisionSet ? 1 : 0);
  } else if (normalized.equals("Vehicle")) {
    sens.setFloat("bat_pct", vehicleBatteryPct);
    sens.setFloat("lidar_cm", vehicleLidarDistanceCm);
    sens.setFloat("heading_deg", degrees(vehicleNavYaw));
    sens.setFloat("light", vehicleLightsEnabled ? 1 : 0);
    sens.setFloat("lights", vehicleLightsEnabled ? 1 : 0);
    sens.setFloat("vehicle_x", vehicleNavX);
    sens.setFloat("vehicle_z", vehicleNavZ);
    sens.setFloat("vehicle_yaw_deg", degrees(vehicleNavYaw));
    sens.setFloat("vehicle_cam_pan_deg", vehicleCameraPanDeg);
    sens.setFloat("vehicle_cam_tilt_deg", vehicleCameraTiltDeg);
    sens.setFloat("vehicle_body_length", VEH_BODY_LENGTH);
    sens.setFloat("vehicle_body_width", VEH_BODY_WIDTH);
    sens.setFloat("vehicle_body_height", VEH_BODY_HEIGHT);
    sens.setFloat("vehicle_track_gauge", VEH_TRACK_GAUGE);
    sens.setFloat("vehicle_track_run", VEH_TRACK_RUN);
    sens.setFloat("vehicle_drive_radius", VEH_DRIVE_R);
    sens.setFloat("vehicle_support_radius", VEH_SUPPORT_R);
    sens.setFloat("vehicle_top_roller_radius", VEH_TOP_ROLLER_R);
    sens.setFloat("vehicle_track_width", VEH_TRACK_WIDTH);
    sens.setFloat("vehicle_track_link_length", VEH_TRACK_LINK_LEN);
    sens.setFloat("vehicle_track_link_thickness", VEH_TRACK_LINK_THICK);
    sens.setFloat("vehicle_track_height", VEH_TRACK_HEIGHT);
    sens.setInt("vehicle_track_links", VEH_TRACK_LINKS);
    sens.setFloat("vehicle_track_sag", VEH_TRACK_SAG);
    sens.setFloat("vehicle_body_center_y", VEH_BODY_CENTER_Y_OFFSET);
    sens.setFloat("vehicle_camera_default_pan_deg", VEH_CAMERA_DEFAULT_PAN_DEG);
    sens.setFloat("vehicle_camera_default_tilt_deg", VEH_CAMERA_DEFAULT_TILT_DEG);
    sens.setFloat("vehicle_lidar_mast_y", VEH_LIDAR_MAST_Y);
    sens.setFloat("vehicle_lidar_mast_z", VEH_LIDAR_MAST_Z);
    sens.setFloat("vehicle_camera_head_y", VEH_CAMERA_HEAD_Y);
    sens.setFloat("vehicle_camera_head_z", VEH_CAMERA_HEAD_Z);
    sens.setFloat("vehicle_motor_deadband_pct", 5.0f);
    sens.setInt("vehicle_motor_pwm_min", 36);
    sens.setInt("vehicle_motor_pwm_max", 255);
    sens.setFloat("vehicle_motor_slew_pct_per_update", 12.0f);
    sens.setInt("vehicle_left_motor_invert", 0);
    sens.setInt("vehicle_right_motor_invert", 0);
    sens.setFloat("vehicle_camera_pan_min_deg", -90.0f);
    sens.setFloat("vehicle_camera_pan_max_deg", 90.0f);
    sens.setFloat("vehicle_camera_tilt_min_deg", -45.0f);
    sens.setFloat("vehicle_camera_tilt_max_deg", 35.0f);
    sens.setInt("vehicle_lidar_sweep_min_deg", 30);
    sens.setInt("vehicle_lidar_sweep_max_deg", 150);
    sens.setInt("vehicle_lidar_sweep_cycle_ms", 1800);
  } else {
    sens.setFloat("bat_pct", droneBatteryPct);
    sens.setFloat("alt_cm", max(droneAltitudeCm, max(0, droneFlightLift * 2.0f)));
    sens.setFloat("cam", droneCameraStreamingEnabled ? 1 : 0);
    sens.setFloat("camera", droneCameraStreamingEnabled ? 1 : 0);
    sens.setFloat("att_yaw_deg", degrees(droneNavYaw));
    sens.setFloat("drone_scan_cm", droneScannerDistanceCm);
    sens.setFloat("drone_x", droneNavX);
    sens.setFloat("drone_y", droneNavY);
    sens.setFloat("drone_z", droneNavZ);
    sens.setFloat("drone_yaw_deg", degrees(droneNavYaw));
    sens.setFloat("drone_pitch_deg", degrees(dronePitch));
    sens.setFloat("drone_roll_deg", degrees(droneRoll));
    sens.setFloat("drone_cam_pan_deg", droneCameraPanDeg);
    sens.setFloat("drone_cam_tilt_deg", droneCameraTiltDeg);
    sens.setFloat("drone_camera_pan_min_deg", -90.0f);
    sens.setFloat("drone_camera_pan_max_deg", 90.0f);
    sens.setFloat("drone_camera_tilt_min_deg", -45.0f);
    sens.setFloat("drone_camera_tilt_max_deg", 35.0f);
    sens.setFloat("drone_body_length", DRONE_BODY_L);
    sens.setFloat("drone_body_width", DRONE_BODY_W);
    sens.setFloat("drone_body_height", DRONE_BODY_H);
    sens.setFloat("drone_arm_length", DRONE_ARM_LENGTH);
    sens.setFloat("drone_arm_thickness", DRONE_ARM_THICKNESS);
    sens.setFloat("drone_motor_radius", DRONE_MOTOR_RADIUS);
    sens.setFloat("drone_motor_height", DRONE_MOTOR_HEIGHT);
    sens.setFloat("drone_prop_radius", DRONE_PROP_RADIUS);
    sens.setFloat("drone_prop_thickness", DRONE_PROP_THICKNESS);
    sens.setFloat("drone_leg_height", DRONE_LEG_HEIGHT);
    sens.setFloat("drone_leg_span", DRONE_LEG_SPAN);
    sens.setFloat("drone_rest_y", DRONE_REST_Y);
    sens.setFloat("drone_visual_yaw_offset_deg", degrees(DRONE_VISUAL_YAW_OFFSET));
    sens.setFloat("drone_camera_y", DRONE_CAMERA_Y);
    sens.setFloat("drone_camera_z", DRONE_CAMERA_Z);
    sens.setFloat("drone_lamp_y", DRONE_LAMP_Y);
    sens.setFloat("drone_lamp_z", DRONE_LAMP_Z);
    sens.setFloat("drone_sonar_x", DRONE_SONAR_X);
    sens.setFloat("drone_sonar_y", DRONE_SONAR_Y);
    sens.setFloat("drone_sonar_z", DRONE_SONAR_Z);
    sens.setFloat("drone_sonar_down_cm", droneScannerDistanceCm > 0 ? droneScannerDistanceCm : max(droneAltitudeCm, max(0, droneFlightLift * 2.0f)));
    sens.setInt("drone_esc_min_us", 1000);
    sens.setInt("drone_esc_max_us", 2000);
    sens.setInt("drone_esc_idle_us", 1080);
    sens.setInt("drone_throttle_range_us", 800);
    sens.setInt("drone_pitch_mix_us", 180);
    sens.setInt("drone_roll_mix_us", 180);
    sens.setInt("drone_yaw_mix_us", 120);
    sens.setFloat("drone_command_deadband_pct", 2.0f);
    sens.setInt("drone_spool_step_us_per_update", 18);
  }

  return sens;
}

// Builds a zeroed telemetry snapshot for robot.
JSONObject buildZeroTelemetrySnapshotForRobot(String robotName) {
  String normalized = normalizeRobotTypeName(robotName);
  if (normalized.length() == 0) normalized = normalizeRobotTypeName(currentModeName());
  if (normalized.length() == 0) normalized = "Manipulator";

  JSONObject sens = new JSONObject();
  sens.setString("robot", normalized);
  sens.setFloat("heading_deg", 0);
  sens.setFloat("att_yaw_deg", 0);
  sens.setFloat("base_yaw_deg", 0);
  sens.setFloat("mpu1_ax", 0);
  sens.setFloat("mpu1_ay", 0);
  sens.setFloat("mpu1_az", 16384);
  sens.setFloat("mpu1_gx", 0);
  sens.setFloat("mpu1_gy", 0);
  sens.setFloat("mpu1_gz", 0);
  sens.setFloat("mpu2_ax", 0);
  sens.setFloat("mpu2_ay", 0);
  sens.setFloat("mpu2_az", 16384);
  sens.setFloat("mpu2_gx", 0);
  sens.setFloat("mpu2_gy", 0);
  sens.setFloat("mpu2_gz", 0);

  if (normalized.equals("Manipulator")) {
    sens.setFloat("base_deg", STARTUP_POSE[BASE_IDX]);
    sens.setFloat("base_yaw_deg", baseServoToForwardYawDeg(STARTUP_POSE[BASE_IDX]));
    sens.setFloat("upper_deg", 0);
    sens.setFloat("fore_deg", 0);
    sens.setFloat("forearm_roll_deg", 0);
    sens.setFloat("wrist_pitch_deg", 0);
    sens.setFloat("wrist_rot_deg", 0);
    sens.setFloat("grip_deg", 0);
    sens.setFloat("sonar_cm", 0);
    sens.setFloat("ina1_ma", 0);
    sens.setFloat("ina2_ma", 0);
    sens.setFloat("grip_pressure_ma", 0);
    sens.setFloat("grip_pressure_est", 0);
    sens.setFloat("gstab", 0);
    sens.setFloat("astab", 0);
    sens.setFloat("collision", 0);
    sens.setFloat("collision_enabled", 0);
    for (int i = 0; i < 4; i++) {
      sens.setFloat("pwm_" + (12 + i), 0);
    }
  } else if (normalized.equals("Vehicle")) {
    sens.setFloat("bat_pct", 0);
    sens.setFloat("battery_pct", 0);
    sens.setFloat("battery_raw", 0);
    sens.setFloat("lidar_cm", 0);
    sens.setFloat("lidar", 0);
    sens.setFloat("range_cm", 0);
    sens.setFloat("light", 0);
    sens.setFloat("lights", 0);
    sens.setFloat("vehicle_x", 0);
    sens.setFloat("vehicle_z", 0);
    sens.setFloat("vehicle_yaw_deg", 0);
    sens.setFloat("vehicle_cam_pan_deg", 0);
    sens.setFloat("vehicle_cam_tilt_deg", 0);
  } else {
    sens.setFloat("bat_pct", 0);
    sens.setFloat("battery_pct", 0);
    sens.setFloat("battery_raw", 0);
    sens.setFloat("alt_cm", 0);
    sens.setFloat("alt", 0);
    sens.setFloat("cam", 0);
    sens.setFloat("camera", 0);
    sens.setFloat("drone_scan_cm", 0);
    sens.setFloat("scan_cm", 0);
    sens.setFloat("range_cm", 0);
    sens.setFloat("depth_cm", 0);
    sens.setFloat("sonar_cm", 0);
    sens.setFloat("drone_x", 0);
    sens.setFloat("drone_y", 0);
    sens.setFloat("drone_z", 0);
    sens.setFloat("drone_yaw_deg", 0);
    sens.setFloat("drone_pitch_deg", 0);
    sens.setFloat("drone_roll_deg", 0);
  }

  return sens;
}

// Synchronizes latest sensors for active telemetry source.
void syncLatestSensorsForActiveTelemetrySource() {
  if (isLocalTelemetrySelected()) {
    latestSensors = buildLocalTelemetrySnapshotForRobot(currentModeName());
  } else if (systemReady && !simulationMode && myPort != null) {
    syncLatestSensorsFromActiveSelection();
  } else {
    latestSensors = new JSONObject();
  }
}

// Preserves the current manipulator pose when hardware disconnects.
void preserveManipulatorPoseForDisconnect() {
  for (int i = 0; i < min(angles.length, hardwareAngles.length); i++) {
    float visualAngle = getManipulatorVisualServoAngle(i);
    int preserved = round(i == BASE_IDX ? normalizeAbsoluteAngleDeg(visualAngle) : visualAngle);
    angles[i] = preserved;
    hardwareAngles[i] = preserved;
  }
  resetPendingManipulatorPoseCommand();
  pendingHardwarePoseSync = false;
  manipulatorTelemetrySuppressUntilMs = 0;
  lastServoCommandTime = 0;
}

// Clears live telemetry readouts after a firmware disconnect.
void resetTelemetryReadoutsForDisconnect() {
  sonarDistanceCm = 0;
  gripPressureValue = 0;
  gripPressureDelta = 0;
  lastGripPressure = 0;
  ina1mA = 0;
  ina2mA = 0;

  gimbalStabEnabled = false;
  armStabEnabled = false;
  remoteCollisionEnabled = false;
  firmwareCollisionRuntimeEnabled = false;
  isColliding = false;

  mpu1RollDeg = 0;
  mpu1PitchDeg = 0;
  mpu1YawDeg = 0;
  mpu2RollDeg = 0;
  mpu2PitchDeg = 0;
  mpu2YawDeg = 0;
  for (int sensor = 0; sensor < 2; sensor++) {
    for (int axis = 0; axis < 3; axis++) {
      mpuAccelRaw[sensor][axis] = axis == 2 ? 16384 : 0;
      mpuGyroRaw[sensor][axis] = 0;
      mpuAccelG[sensor][axis] = axis == 2 ? 1 : 0;
      mpuGyroDps[sensor][axis] = 0;
      mpuMotionBaselineG[sensor][axis] = axis == 2 ? 1 : 0;
      mpuVelocityCms[sensor][axis] = 0;
    }
    mpuVelocityMagCms[sensor] = 0;
  }
  lastImuDerivedUpdateMs = 0;

  vehicleLidarDistanceCm = 0;
  vehicleBatteryPct = 0;
  vehicleHeadingTelemetryDeg = 0;
  vehicleGpsPortReady = false;
  vehicleGpsFixValid = false;
  vehicleGpsSatellites = 0;
  vehicleGpsLatitudeDeg = 0.0;
  vehicleGpsLongitudeDeg = 0.0;
  vehicleGpsAltitudeM = 0.0f;
  vehicleGpsSpeedKph = 0.0f;
  vehicleGpsCourseDeg = 0.0f;
  vehicleGpsHdop = 0.0f;
  vehicleGpsOriginSet = false;
  vehicleGpsOriginLatitudeDeg = 0.0;
  vehicleGpsOriginLongitudeDeg = 0.0;
  vehicleGpsOriginAltitudeM = 0.0f;

  droneBatteryPct = 0;
  droneAltitudeCm = 0;
  droneScannerDistanceCm = 0;
  droneCameraOnline = false;
  droneHeadingTelemetryDeg = 0;
  droneGpsPortReady = false;
  droneGpsFixValid = false;
  droneGpsSatellites = 0;
  droneGpsLatitudeDeg = 0.0;
  droneGpsLongitudeDeg = 0.0;
  droneGpsAltitudeM = 0.0f;
  droneGpsSpeedKph = 0.0f;
  droneGpsCourseDeg = 0.0f;
  droneGpsHdop = 0.0f;
  droneGpsOriginSet = false;
  droneGpsOriginLatitudeDeg = 0.0;
  droneGpsOriginLongitudeDeg = 0.0;
  droneGpsOriginAltitudeM = 0.0f;

  for (int i = 0; i < dutyPercentages.length; i++) {
    dutyPercentages[i] = 0;
  }

  clearDiagnosticsSensorState();
  diagnosticsGraphsWindowOpen = false;
}

// Clears firmware telemetry state while keeping the current local pose.
void resetDisconnectedFirmwareTelemetryState() {
  clearRobotSensorSnapshots();
  latestSensors = new JSONObject();
  resetTelemetryReadoutsForDisconnect();
}

// Utility: refresh local collision state.
void refreshLocalCollisionState() {
  if (!isManipulatorLocalCollisionGuardActive()) {
    isColliding = isManipulatorFirmwareCollisionAuthorityActive() ? remoteCollisionEnabled : false;
    return;
  }
  boolean restoreLocalPose = forceLocalManipulatorPoseForCollision;
  if (!restoreLocalPose && isManipulatorTelemetrySuppressed()) {
    forceLocalManipulatorPoseForCollision = true;
  }
  collectColliders();
  isColliding = checkCollisions();
  forceLocalManipulatorPoseForCollision = restoreLocalPose;
}

// Utility: begin serial monitor session.
void beginSerialMonitorSession() {
  serialMonitorSessionActive = true;
  serialMonitorActivatedAtMillis = millis();
  hardwareStreamStoppedByExit = true;
  hardwareStreamStoppedAtMillis = serialMonitorActivatedAtMillis;
  hardwareTelemetryHealthy = false;
  hardwareTelemetryTimeoutLatched = false;
  moduleForRobot("Vehicle").resetInputStateSafe();
  moduleForRobot("Drone").resetInputStateSafe();
  resetPendingVehicleRuntimeCommand();
  resetPendingDroneRuntimeCommand();
  sendSystemStatus();
}

// Utility: end serial monitor session.
void endSerialMonitorSession() {
  serialMonitorSessionActive = false;
  serialMonitorActivatedAtMillis = 0;
  hardwareStreamStoppedByExit = false;
  hardwareStreamStoppedAtMillis = 0;
  hardwareTelemetryTimeoutLatched = false;
  resetPendingVehicleRuntimeCommand();
  resetPendingDroneRuntimeCommand();
  sendSystemStatus();
}

// Utility: queue serial command.
void queueSerialCommand(String command) {
  String normalized = normalizeRuntimeCommand(command);
  if (normalized.length() == 0) return;

  if (systemReady && !simulationMode && myPort != null) {
    setCommandContext(CONTROL_SOURCE_LOCAL);
    if (serialMonitorSessionActive || normalized.equals("EXIT")) {
      if (normalized.equals("EXIT")) {
        beginSerialMonitorSession();
      }

      boolean sent = false;
      if (isHexCommunicationSelected()) {
        byte[] hexPayload = parseCalibrationHexInput(normalized);
        if (hexPayload != null && hexPayload.length > 0) {
          sent = writeCalibrationBytes(hexPayload, "serial monitor HEX command", true);
        }
      }

      if (!sent) {
        writeCalibrationAsciiCommand(normalized, true);
      }
    } else {
      sendRawHardwareCommand(normalized);
    }
    restoreLocalCommandContext();
  } else {
    updateMessage(tr("Serial0 não está conectada.", "Serial0 is not connected"));
  }
}

// Utility: normalize runtime command.
String normalizeRuntimeCommand(String command) {
  if (command == null) return "";
  String normalized = trim(command);
  if (normalized.equalsIgnoreCase("exit")) return "EXIT";
  return normalized;
}

// Returns sensor float.
float getSensorFloat(String key, float fallback) {
  if (latestSensors == null || key == null || !latestSensors.hasKey(key)) return fallback;
  try {
    return latestSensors.getFloat(key);
  }
  catch (Exception e1) {
    try {
      return latestSensors.getInt(key);
    }
    catch (Exception e2) {
      return fallback;
    }
  }
}

// Returns first available sensor float from a list of possible keys.
float getSensorFloatAny(String[] keys, float fallback) {
  if (keys == null) return fallback;
  for (int i = 0; i < keys.length; i++) {
    String key = keys[i];
    if (latestSensors != null && key != null && latestSensors.hasKey(key)) {
      return getSensorFloat(key, fallback);
    }
  }
  return fallback;
}

// Returns true when any key in a list exists in the latest telemetry packet.
boolean hasAnySensorKey(String[] keys) {
  if (latestSensors == null || keys == null) return false;
  for (int i = 0; i < keys.length; i++) {
    String key = keys[i];
    if (key != null && latestSensors.hasKey(key)) return true;
  }
  return false;
}

// Returns sensor text.
String getSensorText(String key, String fallback) {
  if (latestSensors == null || key == null || !latestSensors.hasKey(key)) return fallback;
  try {
    return latestSensors.getString(key);
  }
  catch (Exception e1) {
    try {
      return str(latestSensors.getInt(key));
    }
    catch (Exception e2) {
      try {
        return str(latestSensors.getFloat(key));
      }
      catch (Exception e3) {
        return fallback;
      }
    }
  }
}

// Utility: manipulator telemetry servo key.
String manipulatorTelemetryServoKey(int jointIndex) {
  switch (jointIndex) {
  case BASE_IDX:
    return "base_deg";
  case UPPERARM_IDX:
    return "upper_deg";
  case FOREARM_IDX:
    return "fore_deg";
  case FOREARM_ROLL_IDX:
    return "forearm_roll_deg";
  case WRIST_VERT_IDX:
    return "wrist_pitch_deg";
  case WRIST_ROT_IDX:
    return "wrist_rot_deg";
  case GRIPPER_IDX:
    return "grip_deg";
  default:
    return "";
  }
}

// Returns manipulator telemetry servo angle.
float getManipulatorTelemetryServoAngle(int jointIndex, float fallback) {
  String key = manipulatorTelemetryServoKey(jointIndex);
  if (key.length() == 0 || latestSensors == null || !latestSensors.hasKey(key)) return fallback;

  float value = getSensorFloat(key, fallback);
  if (jointIndex == BASE_IDX) {
    return normalizeAbsoluteAngleDeg(value);
  }
  return normalizeManipulatorTargetValue(jointIndex, round(value));
}

// Checks whether manipulator telemetry suppressed.
boolean isManipulatorTelemetrySuppressed() {
  return manipulatorTelemetrySuppressUntilMs > millis();
}

// Checks whether manipulator telemetry fresh for visual pose.
boolean isManipulatorTelemetryFreshForVisualPose() {
  if (lastHardwareSensorMillis <= 0) return false;
  int freshWindowMs = max(180, min(hardwareTelemetryTimeoutMs, manipulatorPoseResendIntervalMs + 160));
  return (millis() - lastHardwareSensorMillis) <= freshWindowMs;
}

// Checks whether manipulator telemetry settled for visual pose.
boolean isManipulatorTelemetrySettledForVisualPose() {
  // Visual handoff is stricter than the resend/collision path. Even a small
  // 1..2 degree telemetry lag is enough to look like the joint briefly moves
  // backwards when the local command stops, so only release the visual pose
  // once telemetry has effectively reached the exact commanded position.
  return manipulatorTelemetryCloseToCommandedPose(0);
}

// Utility: should prefer local manipulator visual pose.
boolean shouldPreferLocalManipulatorVisualPose() {
  if (forceLocalManipulatorPoseForCollision) return true;
  if (!isTelemetryDrivenVisualMode()) return true;
  if (!shouldApplyManipulatorTelemetry()) return true;
  if (pendingHardwarePoseSync) return false;
  if (isManipulatorTelemetrySuppressed() && lastServoCommandTime > 0) return true;
  return false;
}

// Checks whether active local drone animation command.
boolean hasActiveLocalDroneAnimationCommand() {
  return abs(droneThrottleCmd) > 0.02f ||
         abs(droneYawCmd) > 0.02f ||
         abs(droneStrafeCmd) > 0.02f ||
         abs(droneForwardCmd) > 0.02f;
}

// Utility: evaluate manipulator self collision for local pose.
boolean evaluateManipulatorSelfCollisionForLocalPose() {
  boolean restore = forceLocalManipulatorPoseForCollision;
  forceLocalManipulatorPoseForCollision = true;
  collectColliders();
  boolean collided = checkCollisions();
  forceLocalManipulatorPoseForCollision = restore;
  return collided;
}

// Utility: evaluate manipulator environment collision for local pose.
boolean evaluateManipulatorEnvironmentCollisionForLocalPose() {
  boolean restore = forceLocalManipulatorPoseForCollision;
  forceLocalManipulatorPoseForCollision = true;
  boolean collided = checkEnvironmentCollisionForCurrentPose();
  forceLocalManipulatorPoseForCollision = restore;
  return collided;
}

// Returns manipulator visual servo angle.
float getManipulatorVisualServoAngle(int jointIndex) {
  float localFallback = (jointIndex >= 0 && jointIndex < angles.length) ? angles[jointIndex] : 0.0f;
  float hardwareFallback = (jointIndex >= 0 && jointIndex < hardwareAngles.length) ? hardwareAngles[jointIndex] : localFallback;
  float fallback = isManipulatorFirmwareCollisionAuthorityActive() ? hardwareFallback : localFallback;
  if (!isTelemetryDrivenVisualMode()) return fallback;
  if (!shouldApplyManipulatorTelemetry()) return fallback;
  if (shouldPreferLocalManipulatorVisualPose()) return fallback;
  return getManipulatorTelemetryServoAngle(jointIndex, fallback);
}

// Checks whether arm telemetry geometry.
boolean hasArmTelemetryGeometry() {
  return latestSensors != null && latestSensors.hasKey("arm_base_radius") && latestSensors.hasKey("arm_base_cylinder_y");
}

// Returns telemetry geometry scene value.
float getTelemetryGeometrySceneValue(String key, float fallbackSceneValue) {
  float cmToScene = max(0.0001f, telemetryVisualNormalization);
  float fallbackCmValue = fallbackSceneValue / cmToScene;
  return getSensorFloat(key, fallbackCmValue) * cmToScene;
}

// Applies telemetry geometry from sensors.
void applyTelemetryGeometryFromSensors() {
  if (!hasArmTelemetryGeometry()) {
    useTelemetryGeometry = false;
    telemetryVisualNormalization = 1.0f;
    return;
  }

  // Firmware sends manipulator geometry in centimeters. Processing keeps the
  // manipulator rendered in its own scene scale, so normalize cm -> scene here.
  telemetryVisualNormalization = MODEL_SCALE_3D;

  baseCylinderRadius = getTelemetryGeometrySceneValue("arm_base_radius", baseCylinderRadius);
  baseCylinderHeight = getTelemetryGeometrySceneValue("arm_base_height", baseCylinderHeight);
  baseBlockW = getTelemetryGeometrySceneValue("arm_base_block_w", baseBlockW);
  baseBlockH = getTelemetryGeometrySceneValue("arm_base_block_h", baseBlockH);
  baseBlockD = getTelemetryGeometrySceneValue("arm_base_block_d", baseBlockD);
  upperArmW = getTelemetryGeometrySceneValue("arm_upper_w", upperArmW);
  upperArmH = getTelemetryGeometrySceneValue("arm_upper_h", upperArmH);
  upperArmD = getTelemetryGeometrySceneValue("arm_upper_d", upperArmD);
  forearmW = getTelemetryGeometrySceneValue("arm_fore_w", forearmW);
  forearmH = getTelemetryGeometrySceneValue("arm_fore_h", forearmH);
  forearmD = getTelemetryGeometrySceneValue("arm_fore_d", forearmD);
  wristVerticalW = getTelemetryGeometrySceneValue("arm_wrist_w", wristVerticalW);
  wristVerticalH = getTelemetryGeometrySceneValue("arm_wrist_h", wristVerticalH);
  wristVerticalD = getTelemetryGeometrySceneValue("arm_wrist_d", wristVerticalD);
  gripperFingerW = getTelemetryGeometrySceneValue("arm_finger_w", gripperFingerW);
  gripperFingerH = getTelemetryGeometrySceneValue("arm_finger_h", gripperFingerH);
  gripperFingerD = getTelemetryGeometrySceneValue("arm_finger_d", gripperFingerD);

  updateDimensions();

  baseCylinderYOffset = getTelemetryGeometrySceneValue("arm_base_cylinder_y", baseCylinderYOffset);
  baseBlockYOffset = getTelemetryGeometrySceneValue("arm_base_block_y", baseBlockYOffset);
  gripperYOffset = getTelemetryGeometrySceneValue("arm_gripper_y", gripperYOffset);

  useTelemetryGeometry = true;
}


// Utility: manipulator telemetry close to commanded pose.
boolean manipulatorTelemetryCloseToCommandedPose(int toleranceDeg) {
  for (int i = 0; i < min(angles.length, hardwareAngles.length); i++) {
    if (i < manipulatorNeutralMembers.length && manipulatorNeutralMembers[i]) continue;
    if (i == BASE_IDX) {
      if (abs(hardwareAngles[i] - angles[i]) > toleranceDeg) return false;
    } else {
      if (abs(hardwareAngles[i] - angles[i]) > toleranceDeg) return false;
    }
  }
  return true;
}

// Utility: estimate roll from accel.
float estimateRollFromAccel(float ax, float ay, float az) {
  return degrees(atan2(ay, az));
}

// Utility: estimate pitch from accel.
float estimatePitchFromAccel(float ax, float ay, float az) {
  return degrees(atan2(-ax, sqrt(ay * ay + az * az)));
}

// Utility: convert MPU accel raw to g.
float mpuAccelRawToG(float raw) {
  return raw / 16384.0f;
}

// Utility: convert MPU gyro raw to deg/s.
float mpuGyroRawToDps(float raw) {
  return raw / 131.0f;
}

// Updates derived MPU motion state.
void updateImuDerivedMotionState() {
  long now = millis();
  float dt = 0.02f;
  if (lastImuDerivedUpdateMs > 0) {
    dt = constrain((now - lastImuDerivedUpdateMs) / 1000.0f, 0.005f, 0.08f);
  }
  lastImuDerivedUpdateMs = now;

  for (int sensor = 0; sensor < 2; sensor++) {
    for (int axis = 0; axis < 3; axis++) {
      mpuAccelG[sensor][axis] = mpuAccelRawToG(mpuAccelRaw[sensor][axis]);
      mpuGyroDps[sensor][axis] = mpuGyroRawToDps(mpuGyroRaw[sensor][axis]);
      float baselineAlpha = axis == 2 ? 0.025f : 0.035f;
      mpuMotionBaselineG[sensor][axis] = lerp(mpuMotionBaselineG[sensor][axis], mpuAccelG[sensor][axis], baselineAlpha);
      float dynamicAccelCms2 = (mpuAccelG[sensor][axis] - mpuMotionBaselineG[sensor][axis]) * 980.665f;
      if (abs(dynamicAccelCms2) < 4.0f) dynamicAccelCms2 = 0.0f;
      mpuVelocityCms[sensor][axis] = (mpuVelocityCms[sensor][axis] + dynamicAccelCms2 * dt) * 0.92f;
    }
    float vx = mpuVelocityCms[sensor][0];
    float vy = mpuVelocityCms[sensor][1];
    float vz = mpuVelocityCms[sensor][2];
    mpuVelocityMagCms[sensor] = sqrt(vx * vx + vy * vy + vz * vz);
  }
}

// Checks whether arm runtime telemetry.
boolean hasArmRuntimeTelemetry() {
  if (latestSensors == null || latestSensors.size() == 0) return false;
  return latestSensors.hasKey("base_deg") || latestSensors.hasKey("upper_deg") || latestSensors.hasKey("fore_deg") || latestSensors.hasKey("forearm_roll_deg") || hasArmTelemetryGeometry();
}

// Utility: should apply manipulator telemetry.
boolean shouldApplyManipulatorTelemetry() {
  return isManipulatorSelected || detectedRobotType.equals("Manipulator") || hasArmRuntimeTelemetry();
}

// Synchronizes manipulator local pose to hardware.
void syncManipulatorLocalPoseToHardware(boolean resetPendingState) {
  for (int i = 0; i < min(angles.length, hardwareAngles.length); i++) {
    angles[i] = hardwareAngles[i];
  }
  if (resetPendingState) {
    resetPendingManipulatorPoseCommand();
    pendingHardwarePoseSync = false;
    manipulatorTelemetrySuppressUntilMs = 0;
    lastServoCommandTime = 0;
  }
}

// Utility: rebuild manipulator from runtime telemetry.
void rebuildManipulatorFromRuntimeTelemetry() {
  if (!shouldApplyManipulatorTelemetry()) return;
  if (isManipulatorFirmwareCollisionAuthorityActive()) {
    isColliding = remoteCollisionEnabled;
    return;
  }
  collectColliders();
  isColliding = checkCollisions();
}

// Applies manipulator sensor state.
void applyManipulatorSensorState() {
  hardwareAngles[BASE_IDX] = round(normalizeAbsoluteAngleDeg(getSensorFloat("base_deg", hardwareAngles[BASE_IDX])));
  hardwareAngles[UPPERARM_IDX] = round(getSensorFloat("upper_deg", hardwareAngles[UPPERARM_IDX]));
  hardwareAngles[FOREARM_IDX] = round(getSensorFloat("fore_deg", hardwareAngles[FOREARM_IDX]));
  hardwareAngles[FOREARM_ROLL_IDX] = round(getSensorFloat("forearm_roll_deg", hardwareAngles[FOREARM_ROLL_IDX]));
  hardwareAngles[WRIST_VERT_IDX] = round(getSensorFloat("wrist_pitch_deg", hardwareAngles[WRIST_VERT_IDX]));
  hardwareAngles[WRIST_ROT_IDX] = round(getSensorFloat("wrist_rot_deg", hardwareAngles[WRIST_ROT_IDX]));
  hardwareAngles[GRIPPER_IDX] = constrain(round(getSensorFloat("grip_deg", hardwareAngles[GRIPPER_IDX])), 0, 100);

  if (pendingHardwarePoseSync) {
    syncManipulatorLocalPoseToHardware(true);
    updateMessage(tr("Pose do manipulador sincronizada a partir do hardware.", "Manipulator pose synchronized from hardware."));
  }

  gripPressureDelta = getSensorFloat("grip_pressure_ma", gripPressureDelta);
  gripPressureValue = getSensorFloat("grip_pressure_est", gripPressureValue);
  lastGripPressure = gripPressureValue;
  ina1mA = getSensorFloat("ina1_ma", ina1mA);
  ina2mA = getSensorFloat("ina2_ma", ina2mA);
  for (int i = 0; i < 4; i++) {
    float fallbackPct = dutyPercentages[i];
    if (latestSensors != null && latestSensors.hasKey("pwm_" + (12 + i))) {
      fallbackPct = getSensorFloat("pwm_" + (12 + i), fallbackPct);
    }
    dutyPercentages[i] = round(fallbackPct);
    if ((dutyTargetsFollowHardware || !dutyTargetsInitialized) && diagnosticsActiveSlider < 0) {
      dutyTargetPercentages[i] = dutyPercentages[i];
    }
  }
  dutyTargetsInitialized = true;
  gimbalStabEnabled = getSensorFloat("gstab", gimbalStabEnabled ? 1 : 0) > 0.5f;
  armStabEnabled = getSensorFloat("astab", armStabEnabled ? 1 : 0) > 0.5f;
  firmwareCollisionRuntimeEnabled = getSensorFloat("collision_enabled", firmwareCollisionRuntimeEnabled ? 1 : 0) > 0.5f;
  remoteCollisionEnabled = getSensorFloat("collision", remoteCollisionEnabled ? 1 : 0) > 0.5f;

  if (isFirmwareCollisionGuardEnabled() && !collisionSet && remoteCollisionEnabled) {
    syncManipulatorLocalPoseToHardware(true);
  } else if (!isManipulatorTelemetrySuppressed() && isManipulatorTelemetryFreshForVisualPose()) {
    syncManipulatorLocalPoseToHardware(false);
  }

  boolean gotLimits = false;
  for (int i = 0; i < min(servoLimits.length, 7); i++) {
    String minKey = "armlim_" + i + "_min";
    String maxKey = "armlim_" + i + "_max";
    if (!latestSensors.hasKey(minKey) || !latestSensors.hasKey(maxKey)) {
      minKey = "srvlim_" + i + "_min";
      maxKey = "srvlim_" + i + "_max";
    }
    if (latestSensors.hasKey(minKey) && latestSensors.hasKey(maxKey)) {
      int minLimit = round(getSensorFloat(minKey, servoLimits[i][0]));
      int maxLimit = round(getSensorFloat(maxKey, servoLimits[i][1]));

      servoLimits[i][0] = minLimit;
      servoLimits[i][1] = maxLimit;
      gotLimits = true;
    }
  }
  limitsFromTelemetry = gotLimits;

  applyTelemetryGeometryFromSensors();
  rebuildManipulatorFromRuntimeTelemetry();
}

// Applies common IMU state.
void applyCommonImuState() {
  mpuAccelRaw[0][0] = getSensorFloat("mpu1_ax", mpuAccelRaw[0][0]);
  mpuAccelRaw[0][1] = getSensorFloat("mpu1_ay", mpuAccelRaw[0][1]);
  mpuAccelRaw[0][2] = getSensorFloat("mpu1_az", mpuAccelRaw[0][2]);
  mpuGyroRaw[0][0] = getSensorFloat("mpu1_gx", getSensorFloat("gyr1_x", mpuGyroRaw[0][0]));
  mpuGyroRaw[0][1] = getSensorFloat("mpu1_gy", getSensorFloat("gyr1_y", mpuGyroRaw[0][1]));
  mpuGyroRaw[0][2] = getSensorFloat("mpu1_gz", getSensorFloat("gyr1_z", mpuGyroRaw[0][2]));
  mpuAccelRaw[1][0] = getSensorFloat("mpu2_ax", mpuAccelRaw[1][0]);
  mpuAccelRaw[1][1] = getSensorFloat("mpu2_ay", mpuAccelRaw[1][1]);
  mpuAccelRaw[1][2] = getSensorFloat("mpu2_az", mpuAccelRaw[1][2]);
  mpuGyroRaw[1][0] = getSensorFloat("mpu2_gx", getSensorFloat("gyr2_x", mpuGyroRaw[1][0]));
  mpuGyroRaw[1][1] = getSensorFloat("mpu2_gy", getSensorFloat("gyr2_y", mpuGyroRaw[1][1]));
  mpuGyroRaw[1][2] = getSensorFloat("mpu2_gz", getSensorFloat("gyr2_z", mpuGyroRaw[1][2]));

  mpu1RollDeg = estimateRollFromAccel(mpuAccelRaw[0][0], mpuAccelRaw[0][1], mpuAccelRaw[0][2]);
  mpu1PitchDeg = estimatePitchFromAccel(mpuAccelRaw[0][0], mpuAccelRaw[0][1], mpuAccelRaw[0][2]);
  mpu2RollDeg = estimateRollFromAccel(mpuAccelRaw[1][0], mpuAccelRaw[1][1], mpuAccelRaw[1][2]);
  mpu2PitchDeg = estimatePitchFromAccel(mpuAccelRaw[1][0], mpuAccelRaw[1][1], mpuAccelRaw[1][2]);

  mpu1YawDeg = getSensorFloat("heading_deg", getSensorFloat("att_yaw_deg", getSensorFloat("base_yaw_deg", mpu1YawDeg)));
  mpu2YawDeg = getSensorFloat("wrist_rot_deg", getSensorFloat("cam_yaw_deg", getSensorFloat("gimbal_yaw_deg", mpu2YawDeg)));
  updateImuDerivedMotionState();
}

// Updates GPS state from the active telemetry snapshot.
void updateGpsStateFromSensorsForRobot(int robotMode) {
  if (latestSensors == null) return;
  boolean ready = getSensorJsonInt(latestSensors, "gps_port_ready", 0) != 0 || getSensorJsonInt(latestSensors, "gps_serial1", 0) != 0;
  boolean fix = getSensorJsonInt(latestSensors, "gps_fix_valid", 0) != 0 || getSensorJsonInt(latestSensors, "gps_fix", 0) != 0;
  int satellites = getSensorJsonInt(latestSensors, "gps_satellites", 0);
  double lat = getJsonFloat(latestSensors, "gps_lat", 0.0f);
  double lon = getJsonFloat(latestSensors, "gps_lon", 0.0f);
  float alt = getJsonFloat(latestSensors, "gps_alt_m", 0.0f);
  float speed = getJsonFloat(latestSensors, "gps_speed_kph", 0.0f);
  float course = getJsonFloat(latestSensors, "gps_course_deg", 0.0f);
  float hdop = getJsonFloat(latestSensors, "gps_hdop", 0.0f);

  if (robotMode == ROBOT_MODE_VEHICLE) {
    vehicleGpsPortReady = ready;
    vehicleGpsFixValid = fix;
    vehicleGpsSatellites = satellites;
    vehicleGpsLatitudeDeg = lat;
    vehicleGpsLongitudeDeg = lon;
    vehicleGpsAltitudeM = alt;
    vehicleGpsSpeedKph = speed;
    vehicleGpsCourseDeg = course;
    vehicleGpsHdop = hdop;
    if (fix && !vehicleGpsOriginSet) {
      vehicleGpsOriginSet = true;
      vehicleGpsOriginLatitudeDeg = lat;
      vehicleGpsOriginLongitudeDeg = lon;
      vehicleGpsOriginAltitudeM = alt;
    }
  } else if (robotMode == ROBOT_MODE_DRONE) {
    droneGpsPortReady = ready;
    droneGpsFixValid = fix;
    droneGpsSatellites = satellites;
    droneGpsLatitudeDeg = lat;
    droneGpsLongitudeDeg = lon;
    droneGpsAltitudeM = alt;
    droneGpsSpeedKph = speed;
    droneGpsCourseDeg = course;
    droneGpsHdop = hdop;
    if (fix && !droneGpsOriginSet) {
      droneGpsOriginSet = true;
      droneGpsOriginLatitudeDeg = lat;
      droneGpsOriginLongitudeDeg = lon;
      droneGpsOriginAltitudeM = alt;
    }
  }
}

// Uses GPS as a world anchor only when local position telemetry is absent.
void applyGpsWorldAnchorForRobot(int robotMode) {
  double lat = robotMode == ROBOT_MODE_VEHICLE ? vehicleGpsLatitudeDeg : droneGpsLatitudeDeg;
  double lon = robotMode == ROBOT_MODE_VEHICLE ? vehicleGpsLongitudeDeg : droneGpsLongitudeDeg;
  double originLat = robotMode == ROBOT_MODE_VEHICLE ? vehicleGpsOriginLatitudeDeg : droneGpsOriginLatitudeDeg;
  double originLon = robotMode == ROBOT_MODE_VEHICLE ? vehicleGpsOriginLongitudeDeg : droneGpsOriginLongitudeDeg;
  boolean fix = robotMode == ROBOT_MODE_VEHICLE ? vehicleGpsFixValid : droneGpsFixValid;
  boolean origin = robotMode == ROBOT_MODE_VEHICLE ? vehicleGpsOriginSet : droneGpsOriginSet;
  if (!fix || !origin) return;

  double metersPerDegLat = 111320.0;
  double metersPerDegLon = 111320.0 * cos(radians((float)originLat));
  float sceneX = (float)((lon - originLon) * metersPerDegLon * 100.0 * robotSceneUnitsPerCm(robotMode));
  float sceneZ = (float)((lat - originLat) * metersPerDegLat * 100.0 * robotSceneUnitsPerCm(robotMode));

  if (robotMode == ROBOT_MODE_VEHICLE) {
    vehicleNavX = lerp(vehicleNavX, sceneX, 0.08f);
    vehicleNavZ = lerp(vehicleNavZ, sceneZ, 0.08f);
  } else if (robotMode == ROBOT_MODE_DRONE) {
    droneNavX = lerp(droneNavX, sceneX, 0.08f);
    droneNavZ = lerp(droneNavZ, sceneZ, 0.08f);
  }
}

// Applies vehicle sensor state.
void applyVehicleSensorState() {
  vehicleLidarDistanceCm = getSensorFloat("lidar_cm", getSensorFloat("lidar", getSensorFloat("range_cm", getSensorFloat("sonar_cm", vehicleLidarDistanceCm))));
  float batteryRaw = getSensorFloat("battery_raw", getSensorFloat("battery", 0));
  vehicleBatteryPct = getSensorFloat("bat_pct", getSensorFloat("battery_pct", batteryRaw > 0 ? map(batteryRaw, 0, 1023, 0, 100) : vehicleBatteryPct));

  updateGpsStateFromSensorsForRobot(ROBOT_MODE_VEHICLE);

  float headingCandidate = getSensorFloat("heading_deg", -1.0f);
  if (headingCandidate < 0.0f) headingCandidate = getSensorFloat("att_yaw_deg", -1.0f);
  if (headingCandidate < 0.0f) headingCandidate = getSensorFloat("vehicle_yaw_deg", degrees(vehicleNavYaw));
  vehicleHeadingTelemetryDeg = normalizeAbsoluteAngleDeg(headingCandidate);

  vehicleLightsEnabled = getSensorFloat("light", vehicleLightsEnabled ? 1 : 0) > 0.5f || getSensorFloat("lights", vehicleLightsEnabled ? 1 : 0) > 0.5f;

  applyVehicleTelemetryGeometryFromSensors();

  boolean hasVehiclePosition = latestSensors.hasKey("vehicle_x") || latestSensors.hasKey("vehicle_z");
  if (latestSensors.hasKey("vehicle_x")) vehicleNavX = lerp(vehicleNavX, getSensorFloat("vehicle_x", vehicleNavX), 0.18f);
  if (latestSensors.hasKey("vehicle_z")) vehicleNavZ = lerp(vehicleNavZ, getSensorFloat("vehicle_z", vehicleNavZ), 0.18f);
  if (!hasVehiclePosition) applyGpsWorldAnchorForRobot(ROBOT_MODE_VEHICLE);

  if (latestSensors.hasKey("vehicle_yaw_deg")) vehicleNavYaw = lerp(vehicleNavYaw, radians(getSensorFloat("vehicle_yaw_deg", degrees(vehicleNavYaw))), 0.20f);
  if (latestSensors.hasKey("vehicle_cam_pan_deg")) vehicleCameraPanDeg = lerp(vehicleCameraPanDeg, getSensorFloat("vehicle_cam_pan_deg", vehicleCameraPanDeg), 0.28f);
  if (latestSensors.hasKey("vehicle_cam_tilt_deg")) vehicleCameraTiltDeg = lerp(vehicleCameraTiltDeg, getSensorFloat("vehicle_cam_tilt_deg", vehicleCameraTiltDeg), 0.28f);
}

// Applies drone sensor state.
void applyDroneSensorState() {
  float batteryRaw = getSensorFloat("battery_raw", getSensorFloat("battery", 0));
  droneBatteryPct = getSensorFloat("bat_pct", getSensorFloat("battery_pct", batteryRaw > 0 ? map(batteryRaw, 0, 1023, 0, 100) : droneBatteryPct));
  droneAltitudeCm = getSensorFloat("alt_cm", getSensorFloat("alt", droneAltitudeCm));
  droneCameraOnline = getSensorFloat("cam", droneCameraOnline ? 1 : 0) > 0.5f || getSensorFloat("camera", droneCameraOnline ? 1 : 0) > 0.5f || getSensorFloat("camera_record", droneCameraOnline ? 1 : 0) > 0.5f;

  updateGpsStateFromSensorsForRobot(ROBOT_MODE_DRONE);

  float headingCandidate = getSensorFloat("att_yaw_deg", -1.0f);
  if (headingCandidate < 0.0f) headingCandidate = getSensorFloat("heading_deg", -1.0f);
  if (headingCandidate < 0.0f) headingCandidate = getSensorFloat("drone_yaw_deg", degrees(droneNavYaw));
  droneHeadingTelemetryDeg = normalizeAbsoluteAngleDeg(headingCandidate);

  droneScannerDistanceCm = getSensorFloat("drone_scan_cm",
    getSensorFloat("drone_sonar_down_cm",
    getSensorFloat("sonar_down_cm",
    getSensorFloat("height_cm",
    getSensorFloat("alt_cm",
    getSensorFloat("scan_cm",
    getSensorFloat("range_cm",
    getSensorFloat("depth_cm",
    getSensorFloat("sonar_cm", droneScannerDistanceCm)))))))));

  applyDroneTelemetryGeometryFromSensors();

  boolean hasDronePosition = latestSensors.hasKey("drone_x") || latestSensors.hasKey("drone_y") || latestSensors.hasKey("drone_z");
  if (latestSensors.hasKey("drone_x")) droneNavX = lerp(droneNavX, getSensorFloat("drone_x", droneNavX), 0.16f);
  if (latestSensors.hasKey("drone_y")) droneNavY = lerp(droneNavY, getSensorFloat("drone_y", droneNavY), 0.16f);
  if (latestSensors.hasKey("drone_z")) droneNavZ = lerp(droneNavZ, getSensorFloat("drone_z", droneNavZ), 0.16f);
  if (!hasDronePosition) applyGpsWorldAnchorForRobot(ROBOT_MODE_DRONE);

  if (latestSensors.hasKey("drone_yaw_deg")) droneNavYaw = lerp(droneNavYaw, radians(getSensorFloat("drone_yaw_deg", degrees(droneNavYaw))), 0.18f);
  if (latestSensors.hasKey("drone_cam_pan_deg")) droneCameraPanDeg = lerp(droneCameraPanDeg, getSensorFloat("drone_cam_pan_deg", droneCameraPanDeg), 0.28f);
  if (latestSensors.hasKey("drone_cam_tilt_deg")) droneCameraTiltDeg = lerp(droneCameraTiltDeg, getSensorFloat("drone_cam_tilt_deg", droneCameraTiltDeg), 0.28f);
  if (!hasActiveLocalDroneAnimationCommand()) {
    if (latestSensors.hasKey("drone_pitch_deg")) dronePitch = lerp(dronePitch, radians(getSensorFloat("drone_pitch_deg", degrees(dronePitch))), 0.18f);
    if (latestSensors.hasKey("drone_roll_deg")) droneRoll = lerp(droneRoll, radians(getSensorFloat("drone_roll_deg", degrees(droneRoll))), 0.18f);
  }
}

// Applies sensor state.
void applySensorState() {
  if (latestSensors == null || latestSensors.size() == 0) return;

  String robotHint = getSensorText("robot", getSensorText("robot_type", ""));
  if (robotHint.length() > 0) {
    setDetectedRobotType(robotHint);
  }

  sonarDistanceCm = getSensorFloat("sonar_cm", sonarDistanceCm);
  applyCommonImuState();
  applyVehicleSensorState();
  applyDroneSensorState();

  if (shouldApplyManipulatorTelemetry()) {
    applyManipulatorSensorState();
  }
}

// Updates hardware stream health.
void updateHardwareStreamHealth() {
  syncCalibrationMonitorWindowState();

  if (!(systemReady && !simulationMode) || myPort == null) {
    hardwareTelemetryHealthy = false;
    hardwareTelemetryTimeoutLatched = false;
    return;
  }

  if (calibrationWindowVisible || serialMonitorSessionActive || hardwareStreamStoppedByExit) {
    hardwareTelemetryHealthy = false;
    hardwareTelemetryTimeoutLatched = false;
    return;
  }

  long now = millis();
  long telemetryAge = now - lastHardwareSensorMillis;
  long rxAge = now - lastHardwareRxMillis;

  int effectiveTelemetryTimeoutMs = hardwareTelemetryTimeoutMs;
  int effectiveDisconnectTimeoutMs = hardwareDisconnectTimeoutMs;

  boolean manipFirmwareLink = isManipulatorFirmwareTelemetryAuthorityActive();
  if (manipFirmwareLink) {
    if (effectiveTelemetryTimeoutMs < 2600) effectiveTelemetryTimeoutMs = 2600;
    if (effectiveDisconnectTimeoutMs < 10000) effectiveDisconnectTimeoutMs = 10000;

    if (isFirmwareCollisionGuardEnabled()) {
      if (effectiveTelemetryTimeoutMs < 3200) effectiveTelemetryTimeoutMs = 3200;
      if (effectiveDisconnectTimeoutMs < 12000) effectiveDisconnectTimeoutMs = 12000;
    }

    if (remoteCollisionEnabled) {
      if (effectiveTelemetryTimeoutMs < 4200) effectiveTelemetryTimeoutMs = 4200;
      if (effectiveDisconnectTimeoutMs < 15000) effectiveDisconnectTimeoutMs = 15000;
    }
  }

  if (lastHardwareSensorMillis > 0 && telemetryAge <= effectiveTelemetryTimeoutMs) {
    hardwareTelemetryHealthy = true;
    hardwareTelemetryTimeoutLatched = false;
    return;
  }

  if (hexRuntimeReady && lastHardwareSensorMillis <= 0 && lastHardwareRxMillis > 0) {
    hardwareTelemetryHealthy = false;
    hardwareTelemetryTimeoutLatched = false;
    return;
  }

  hardwareTelemetryHealthy = false;
  if (!hardwareTelemetryTimeoutLatched && lastHardwareSensorMillis > 0) {
    hardwareTelemetryTimeoutLatched = true;
    updateMessage(tr("Timeout de telemetria/stream detectado.", "Telemetry/stream timeout detected"));
    appendCalibrationSerialLine("[WARN] Telemetry timeout detected");
    if (isVehicleSelected) stopVehicleMotion(true);
    if (isDroneSelected) stopDroneMotion(true);
    sendSystemStatus();
  }

  if (lastHardwareRxMillis > 0 && rxAge > effectiveDisconnectTimeoutMs) {
    handleMainSerialDisconnect("serial stream lost (" + rxAge + " ms)");
  }
}

// Utility: process serial io.
void processSerialIO() {
  serviceCalibrationMonitorTransitions();
  if (isHexCommunicationSelected() && systemReady && !simulationMode) {
    if (myPort != null) {
      drainBinarySerialPort(myPort);
    }
    if (calibrationPort != null) {
      drainBinarySerialPort(calibrationPort);
    }
  }
  processPendingAppSerialAsciiLines();
  maybeRequestHexRuntimeBootstrapTelemetry();
  syncLatestSensorsForActiveTelemetrySource();
  applySensorState();
  updateHardwareStreamHealth();
  updateControlLocks();
}
