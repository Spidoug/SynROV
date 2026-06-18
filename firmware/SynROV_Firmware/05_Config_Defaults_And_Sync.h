// =====================================================================
// SynROV Firmware - Configuration defaults and runtime synchronization
// ---------------------------------------------------------------------
// Purpose:
//   Default values, runtime/apply helpers and configuration sanitation.
// =====================================================================

// Fills default config.
void fillDefaultConfig(PersistentConfig& cfg) {
  cfg.magic = EEPROM_MAGIC;
  cfg.schemaId = EEPROM_SCHEMA_ID;

  cfg.payload.pca9685Addr = 0x50;
  cfg.payload.ina219Addr1 = 0x41;
  cfg.payload.ina219Addr2 = 0x44;
  cfg.payload.mpu6050Addr1 = 0x68;
  cfg.payload.mpu6050Addr2 = 0x69;
  cfg.payload.compassAddr = 0;
  cfg.payload.compassType = 0;

  cfg.payload.kp = 5.9f;   // DC PWM 70%: rastreamento geral de referência
  cfg.payload.ki = 0.000f;  // integral desligado por padrão para evitar hunting no atrito
  cfg.payload.kd = 0.90f;   // amortecimento sem amplificar ruído do A0

  cfg.payload.zeroAdc = 50;
  cfg.payload.spanRaw = 380;

  cfg.payload.minStepDelayUs = 1500;  // era 2800 — velocidade máxima stepper ~667 steps/s
  cfg.payload.maxStepDelayUs = 16000;
  cfg.payload.stopEnter = 0.8f;
  cfg.payload.stopExit = 1.4f;
  cfg.payload.pwmFar = 285;
  cfg.payload.pwmNear = 215;
  cfg.payload.pwmHold = 155;
  cfg.payload.angleFilterAlpha = 0.700f;
  cfg.payload.speedFilterAlpha = 0.240f;
  cfg.payload.controlIntervalUs = 3000UL;
  cfg.payload.servoMoveStep = 2.000f;
  cfg.payload.servoRampSmoothing = 1.000f;
  cfg.payload.reserved0 = encodeControlModesForStore(MOTOR_MODE_HALFSTEP_UNIPOLAR, MOTOR_TYPE_DC_MOTOR);
  cfg.payload.manipMemberChannel[MANIP_MEMBER_BASE] = kDefaultManipProfile.baseChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_UPPER] = kDefaultManipProfile.upperChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_FORE] = kDefaultManipProfile.foreChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_FOREARM_ROLL] = kDefaultManipProfile.forearmRollChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_WRIST_PITCH] = kDefaultManipProfile.wristPitchChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_WRIST_ROLL] = kDefaultManipProfile.wristRotChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_GRIPPER] = kDefaultManipProfile.gripperChannel;
  cfg.payload.reserved1[0] = encodeBaseA0DeadbandDeg(MOTOR_BASE_A0_DEADBAND_DEFAULT_DEG);
  cfg.payload.reserved1[1] = encodeDcPidBandDeg(MOTOR_DC_PID_ENTER_BAND_DEFAULT_DEG);
  cfg.payload.reserved1[2] = encodeDcPidBandDeg(MOTOR_DC_PID_EXIT_BAND_DEFAULT_DEG);

  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    cfg.payload.servoOffset[i] = 0;
    cfg.payload.servoSpan[i] = 1.0f;
    cfg.payload.servoRamp[i] = false;
    cfg.payload.servoOptimize[i] = 0;
    cfg.payload.servoDirection[i] = 1;
    cfg.payload.servoMinLimit[i] = SERVO_INPUT_MIN;
    cfg.payload.servoMaxLimit[i] = getChannelInputMax(i);
  }
  cfg.payload.servoMinLimit[SERVO_EXTENDED_RANGE_CHANNEL] = 0.0f;
  cfg.payload.servoMaxLimit[SERVO_EXTENDED_RANGE_CHANNEL] = SERVO_EXTENDED_INPUT_MAX;
  cfg.payload.servoMinLimit[3] = 30.0f;
  cfg.payload.servoMaxLimit[3] = 180.0f;
  cfg.payload.servoMinLimit[4] = 0.0f;
  cfg.payload.servoMaxLimit[4] = 164.6f;
  cfg.payload.servoMinLimit[kDefaultManipProfile.gripperChannel] = 0.0f;
  cfg.payload.servoMaxLimit[kDefaultManipProfile.gripperChannel] = MANIP_GRIPPER_MAX_DEG;
  cfg.payload.collisionEnabled = true;
  cfg.payload.vehicleHoldLastMotionOnLinkLoss = false;
  cfg.payload.droneHoldLastMotionOnLinkLoss = false;
  cfg.payload.seqLossReserved[0] = 0;
  cfg.payload.seqLossReserved[1] = 0;

  armGeom = makeDefaultArmGeometryConfig();
  recomputeManipulatorOffsetsFromProcessingModel();
  sanitizeArmGeometryConfig(armGeom);
  copyArmGeometryToPayload(cfg.payload, armGeom);

  for (uint8_t i = 0; i < 4; i++) {
    cfg.payload.pwmBoot[i] = DEFAULT_MANUAL_PWM_PERCENT;
  }
  cfg.payload.armStabilizeEnable = false;
  cfg.payload.armStabilizeGain = 0.12f;
  cfg.payload.armStabilizeThreshold = 5.0f;
  cfg.payload.gripPressureEnable = true;
  cfg.payload.gripPressureGain = 1.00f;
  cfg.payload.vehicleProfile = kDefaultVehicleProfile;
  cfg.payload.droneProfile = kDefaultDroneProfile;
  cfg.payload.vehicleControlProfile = kDefaultVehicleControlProfile;
  cfg.payload.droneControlProfile = kDefaultDroneControlProfile;
  sanitizeVehicleProcessingProfile(cfg.payload.vehicleProfile);
  sanitizeDroneProcessingProfile(cfg.payload.droneProfile);
  sanitizeVehicleControlProfile(cfg.payload.vehicleControlProfile);
  sanitizeDroneControlProfile(cfg.payload.droneControlProfile);

  cfg.crc = calculateConfigCRC(cfg);
}

// Applies config to runtime.
void applyConfigToRuntime(const PersistentConfig& cfg) {
  pca9685Addr = cfg.payload.pca9685Addr;
  ina219Addr1 = cfg.payload.ina219Addr1;
  ina219Addr2 = cfg.payload.ina219Addr2;
  mpu6050Addr1 = cfg.payload.mpu6050Addr1;
  mpu6050Addr2 = cfg.payload.mpu6050Addr2;
  compassAddr = cfg.payload.compassAddr;
  compassType = (CompassType)cfg.payload.compassType;

  motor.kp = cfg.payload.kp;
  motor.ki = cfg.payload.ki;
  motor.kd = cfg.payload.kd;

  motorCalibrationZeroADC = cfg.payload.zeroAdc;
  motorOneTurnSpanRaw = cfg.payload.spanRaw;

  minStepDelayUs = cfg.payload.minStepDelayUs;
  maxStepDelayUs = cfg.payload.maxStepDelayUs;
  motorStopEnter = cfg.payload.stopEnter;
  motorStopExit = cfg.payload.stopExit;
  movePwmFar = cfg.payload.pwmFar;
  movePwmNear = cfg.payload.pwmNear;
  holdPwm = cfg.payload.pwmHold;
  angleFilterAlpha = cfg.payload.angleFilterAlpha;
  motorBaseA0DeadbandDeg = decodeBaseA0DeadbandDeg(cfg.payload.reserved1[0]);
  motorDcPidEnterBandDeg = decodeDcPidBandDeg(cfg.payload.reserved1[1], MOTOR_DC_PID_ENTER_BAND_DEFAULT_DEG);
  motorDcPidExitBandDeg = decodeDcPidBandDeg(cfg.payload.reserved1[2], MOTOR_DC_PID_EXIT_BAND_DEFAULT_DEG);
  if (motorDcPidExitBandDeg <= motorDcPidEnterBandDeg) {
    motorDcPidExitBandDeg = min(MOTOR_DC_PID_BAND_MAX_DEG, motorDcPidEnterBandDeg + 0.6f);
  }
  speedFilterAlpha = cfg.payload.speedFilterAlpha;
  controlIntervalUs = cfg.payload.controlIntervalUs;
  servoMoveStepDeg = cfg.payload.servoMoveStep;
  servoRampSmoothing = clampValue(cfg.payload.servoRampSmoothing, 0.05f, 1.0f);
  motorDriveMode = decodeStoredMotorDriveMode(cfg.payload.reserved0);
  motorType = decodeStoredMotorType(cfg.payload.reserved0);
  manipProfile.baseChannel = cfg.payload.manipMemberChannel[MANIP_MEMBER_BASE];
  manipProfile.upperChannel = cfg.payload.manipMemberChannel[MANIP_MEMBER_UPPER];
  manipProfile.foreChannel = cfg.payload.manipMemberChannel[MANIP_MEMBER_FORE];
  manipProfile.forearmRollChannel = cfg.payload.manipMemberChannel[MANIP_MEMBER_FOREARM_ROLL];
  manipProfile.wristPitchChannel = cfg.payload.manipMemberChannel[MANIP_MEMBER_WRIST_PITCH];
  manipProfile.wristRotChannel = cfg.payload.manipMemberChannel[MANIP_MEMBER_WRIST_ROLL];
  manipProfile.gripperChannel = cfg.payload.manipMemberChannel[MANIP_MEMBER_GRIPPER];
  sanitizeManipulatorProfile();
  sanitizeMotorRuntimeParams();

  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    servos.offset[i] = cfg.payload.servoOffset[i];
    setServoSpan(i, cfg.payload.servoSpan[i]);
    setServoRamp(i, cfg.payload.servoRamp[i]);
    servos.optimize[i] = 0;
    servos.direction[i] = cfg.payload.servoDirection[i];
    setServoMinLimit(i, cfg.payload.servoMinLimit[i]);
    setServoMaxLimit(i, cfg.payload.servoMaxLimit[i]);
  }
  setServoMinLimit(SERVO_EXTENDED_RANGE_CHANNEL, 0.0f);
  setServoMaxLimit(SERVO_EXTENDED_RANGE_CHANNEL, SERVO_EXTENDED_INPUT_MAX);
  enforceManipulatorGripperRange();


  // Load per-channel boot percentages from EEPROM so each channel starts at
  // the value the operator configured, not a forced 50% neutral.
  for (uint8_t i = 0; i < 4; i++) {
    pwmBootPercent[i] = clampValue(cfg.payload.pwmBoot[i], (uint8_t)0, (uint8_t)100);
  }
  armStabilizeEnable = cfg.payload.armStabilizeEnable;
  armStabilizeGain = cfg.payload.armStabilizeGain;
  armStabilizeThreshold = cfg.payload.armStabilizeThreshold;
  gripPressureEnable = cfg.payload.gripPressureEnable;
  gripPressureGain = cfg.payload.gripPressureGain;
  collisionRuntimeEnabled = cfg.payload.collisionEnabled;
  vehicleHoldLastMotionOnLinkLoss = cfg.payload.vehicleHoldLastMotionOnLinkLoss;
  droneHoldLastMotionOnLinkLoss = cfg.payload.droneHoldLastMotionOnLinkLoss;
  vehicleProfile = cfg.payload.vehicleProfile;
  droneProfile = cfg.payload.droneProfile;
  vehicleControlProfile = cfg.payload.vehicleControlProfile;
  droneControlProfile = cfg.payload.droneControlProfile;
  sanitizeVehicleProcessingProfile(vehicleProfile);
  sanitizeDroneProcessingProfile(droneProfile);
  sanitizeVehicleControlProfile(vehicleControlProfile);
  sanitizeDroneControlProfile(droneControlProfile);
  processingProfilesInitialized = true;
  processingProfilesDirty = false;
  fillDefaultManipulatorHomePose(manipHomePose);
  sanitizeManipulatorHomePose(manipHomePose);
  invalidateCollisionPoseCache();
  if (!collisionRuntimeEnabled) {
    collisionFlag = false;
    invalidateCollisionPoseCache();
  }

  loadArmGeometryFromPayload(cfg.payload, armGeom);
  // Keep explicit geometry/visual offsets exactly as stored.
  recomputeManipulatorOffsetsFromProcessingModel();
  recomputeProcessingRobotDerivedOffsets();
  applySafeStartupPose();

  clearAllNeutralStates();
  releaseInactiveManipulatorChannels();
  uint16_t nearPwmLimited = (movePwmNear > ICR5) ? ICR5 : movePwmNear;
  motorRecoveryPwm = (uint16_t)clampValue((int)nearPwmLimited, 0, (int)ICR5);
  resetMotorControlState();
}

// Synchronizes runtime to config.
void syncRuntimeToConfig(PersistentConfig& cfg) {
  cfg.magic = EEPROM_MAGIC;
  cfg.schemaId = EEPROM_SCHEMA_ID;

  cfg.payload.pca9685Addr = pca9685Addr;
  cfg.payload.ina219Addr1 = ina219Addr1;
  cfg.payload.ina219Addr2 = ina219Addr2;
  cfg.payload.mpu6050Addr1 = mpu6050Addr1;
  cfg.payload.mpu6050Addr2 = mpu6050Addr2;
  cfg.payload.compassAddr = compassAddr;
  cfg.payload.compassType = (uint8_t)compassType;

  cfg.payload.kp = motor.kp;
  cfg.payload.ki = motor.ki;
  cfg.payload.kd = motor.kd;

  cfg.payload.zeroAdc = motorCalibrationZeroADC;
  cfg.payload.spanRaw = motorOneTurnSpanRaw;

  cfg.payload.minStepDelayUs = minStepDelayUs;
  cfg.payload.maxStepDelayUs = maxStepDelayUs;
  cfg.payload.stopEnter = motorStopEnter;
  cfg.payload.stopExit = motorStopExit;
  cfg.payload.pwmFar = movePwmFar;
  cfg.payload.pwmNear = movePwmNear;
  cfg.payload.pwmHold = holdPwm;
  cfg.payload.angleFilterAlpha = angleFilterAlpha;
  cfg.payload.speedFilterAlpha = speedFilterAlpha;
  cfg.payload.controlIntervalUs = controlIntervalUs;
  cfg.payload.servoMoveStep = servoMoveStepDeg;
  cfg.payload.servoRampSmoothing = servoRampSmoothing;
  cfg.payload.reserved0 = encodeControlModesForStore(motorDriveMode, motorType);
  syncProcessingRobotProfiles();
  cfg.payload.manipMemberChannel[MANIP_MEMBER_BASE] = manipProfile.baseChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_UPPER] = manipProfile.upperChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_FORE] = manipProfile.foreChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_FOREARM_ROLL] = manipProfile.forearmRollChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_WRIST_PITCH] = manipProfile.wristPitchChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_WRIST_ROLL] = manipProfile.wristRotChannel;
  cfg.payload.manipMemberChannel[MANIP_MEMBER_GRIPPER] = manipProfile.gripperChannel;
  cfg.payload.reserved1[0] = encodeBaseA0DeadbandDeg(motorBaseA0DeadbandDeg);
  cfg.payload.reserved1[1] = encodeDcPidBandDeg(motorDcPidEnterBandDeg);
  cfg.payload.reserved1[2] = encodeDcPidBandDeg(motorDcPidExitBandDeg);

  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    cfg.payload.servoOffset[i] = servos.offset[i];
    cfg.payload.servoSpan[i] = getServoSpan(i);
    cfg.payload.servoRamp[i] = getServoRamp(i);
    cfg.payload.servoOptimize[i] = 0;
    cfg.payload.servoDirection[i] = servos.direction[i];
    cfg.payload.servoMinLimit[i] = getServoMinLimit(i);
    cfg.payload.servoMaxLimit[i] = getServoMaxLimit(i);
  }
  cfg.payload.servoMinLimit[SERVO_EXTENDED_RANGE_CHANNEL] = 0.0f;
  cfg.payload.servoMaxLimit[SERVO_EXTENDED_RANGE_CHANNEL] = SERVO_EXTENDED_INPUT_MAX;
  cfg.payload.servoMinLimit[getManipulatorMemberChannel(MANIP_MEMBER_GRIPPER)] = 0.0f;
  cfg.payload.servoMaxLimit[getManipulatorMemberChannel(MANIP_MEMBER_GRIPPER)] = MANIP_GRIPPER_MAX_DEG;

  recomputeManipulatorOffsetsFromProcessingModel();
  sanitizeArmGeometryConfig(armGeom);
  copyArmGeometryToPayload(cfg.payload, armGeom);

  for (uint8_t i = 0; i < 4; i++) {
    cfg.payload.pwmBoot[i] = clampValue(pwmBootPercent[i], (uint8_t)0, (uint8_t)100);
  }
  cfg.payload.armStabilizeEnable = armStabilizeEnable;
  cfg.payload.armStabilizeGain = armStabilizeGain;
  cfg.payload.armStabilizeThreshold = armStabilizeThreshold;
  cfg.payload.gripPressureEnable = gripPressureEnable;
  cfg.payload.gripPressureGain = gripPressureGain;
  cfg.payload.collisionEnabled = collisionRuntimeEnabled;
  cfg.payload.vehicleHoldLastMotionOnLinkLoss = vehicleHoldLastMotionOnLinkLoss;
  cfg.payload.droneHoldLastMotionOnLinkLoss = droneHoldLastMotionOnLinkLoss;
  cfg.payload.seqLossReserved[0] = 0;
  cfg.payload.seqLossReserved[1] = 0;
  sanitizeVehicleProcessingProfile(vehicleProfile);
  sanitizeDroneProcessingProfile(droneProfile);
  sanitizeVehicleControlProfile(vehicleControlProfile);
  sanitizeDroneControlProfile(droneControlProfile);
  cfg.payload.vehicleProfile = vehicleProfile;
  cfg.payload.droneProfile = droneProfile;
  cfg.payload.vehicleControlProfile = vehicleControlProfile;
  cfg.payload.droneControlProfile = droneControlProfile;

  cfg.crc = calculateConfigCRC(cfg);
}

// Fills default config extras.
void fillDefaultConfigExtras(PersistentConfigExtras& cfg) {
  cfg.magic = CONFIG_EXTRAS_MAGIC;
  cfg.schemaId = CONFIG_EXTRAS_SCHEMA_ID;
  cfg.payload.wristGimbalEnabled = false;
  memset(cfg.payload.reserved0, 0, sizeof(cfg.payload.reserved0));

  cfg.payload.manipJointMap[MANIP_MEMBER_BASE] = {0.0f, 0.0f, 1.0f};
  cfg.payload.manipJointMap[MANIP_MEMBER_UPPER] = {45.0f, 0.0f, 1.0f};
  cfg.payload.manipJointMap[MANIP_MEMBER_FORE] = {180.0f, 0.0f, 1.0f};
  cfg.payload.manipJointMap[MANIP_MEMBER_FOREARM_ROLL] = {90.0f, 0.0f, 1.0f};
  cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_PITCH] = {90.0f, 0.0f, 1.0f};
  cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_ROLL] = {90.0f, 0.0f, 1.0f};
  cfg.payload.manipJointMap[MANIP_MEMBER_GRIPPER] = {0.0f, 0.0f, 1.0f};
  cfg.crc = calculateConfigExtrasCRC(cfg);
}

// Applies config extras to runtime.
void applyConfigExtrasToRuntime(const PersistentConfigExtras& cfg) {
  wristGimbalEnabled = cfg.payload.wristGimbalEnabled;
  wristGimbalRefValid = false;

  // Base yaw is intentionally identity-mapped: base servo angle == base yaw.
  // Keep stored EEPROM settings aligned with the 0-to-100 degree gripper travel.
  armBaseServoZeroDeg = 0.0f;
  armBaseMemberZeroDeg = 0.0f;
  armBaseServoSign = 1.0f;
  armUpperServoZeroDeg = cfg.payload.manipJointMap[MANIP_MEMBER_UPPER].servoZeroDeg;
  armUpperMemberZeroDeg = cfg.payload.manipJointMap[MANIP_MEMBER_UPPER].memberZeroDeg;
  armUpperServoSign = (fabsf(cfg.payload.manipJointMap[MANIP_MEMBER_UPPER].servoSign) < 0.0001f) ? 1.0f : cfg.payload.manipJointMap[MANIP_MEMBER_UPPER].servoSign;
  armForeServoZeroDeg = cfg.payload.manipJointMap[MANIP_MEMBER_FORE].servoZeroDeg;
  armForeMemberZeroDeg = cfg.payload.manipJointMap[MANIP_MEMBER_FORE].memberZeroDeg;
  armForeServoSign = (fabsf(cfg.payload.manipJointMap[MANIP_MEMBER_FORE].servoSign) < 0.0001f) ? 1.0f : cfg.payload.manipJointMap[MANIP_MEMBER_FORE].servoSign;
  armForearmRollServoZeroDeg = cfg.payload.manipJointMap[MANIP_MEMBER_FOREARM_ROLL].servoZeroDeg;
  armForearmRollMemberZeroDeg = cfg.payload.manipJointMap[MANIP_MEMBER_FOREARM_ROLL].memberZeroDeg;
  armForearmRollServoSign = (fabsf(cfg.payload.manipJointMap[MANIP_MEMBER_FOREARM_ROLL].servoSign) < 0.0001f) ? 1.0f : cfg.payload.manipJointMap[MANIP_MEMBER_FOREARM_ROLL].servoSign;
  armWristPitchServoZeroDeg = cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_PITCH].servoZeroDeg;
  armWristPitchMemberZeroDeg = cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_PITCH].memberZeroDeg;
  armWristPitchServoSign = (fabsf(cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_PITCH].servoSign) < 0.0001f) ? 1.0f : cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_PITCH].servoSign;
  armWristRollServoZeroDeg = cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_ROLL].servoZeroDeg;
  armWristRollMemberZeroDeg = cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_ROLL].memberZeroDeg;
  armWristRollServoSign = (fabsf(cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_ROLL].servoSign) < 0.0001f) ? 1.0f : cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_ROLL].servoSign;
  // Standardize the gripper mapping across firmware and Processing:
  //   servo 0°   = gripper closed
  //   servo 100° = gripper fully open
  armGripServoZeroDeg = 0.0f;
  armGripMemberZeroDeg = 0.0f;
  armGripServoSign = 1.0f;
}

// Synchronizes runtime to config extras.
void syncRuntimeToConfigExtras(PersistentConfigExtras& cfg) {
  cfg.magic = CONFIG_EXTRAS_MAGIC;
  cfg.schemaId = CONFIG_EXTRAS_SCHEMA_ID;
  cfg.payload.wristGimbalEnabled = wristGimbalEnabled;
  memset(cfg.payload.reserved0, 0, sizeof(cfg.payload.reserved0));
  cfg.payload.manipJointMap[MANIP_MEMBER_BASE] = {0.0f, 0.0f, 1.0f};
  cfg.payload.manipJointMap[MANIP_MEMBER_UPPER] = {armUpperServoZeroDeg, armUpperMemberZeroDeg, armUpperServoSign};
  cfg.payload.manipJointMap[MANIP_MEMBER_FORE] = {armForeServoZeroDeg, armForeMemberZeroDeg, armForeServoSign};
  cfg.payload.manipJointMap[MANIP_MEMBER_FOREARM_ROLL] = {armForearmRollServoZeroDeg, armForearmRollMemberZeroDeg, armForearmRollServoSign};
  cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_PITCH] = {armWristPitchServoZeroDeg, armWristPitchMemberZeroDeg, armWristPitchServoSign};
  cfg.payload.manipJointMap[MANIP_MEMBER_WRIST_ROLL] = {armWristRollServoZeroDeg, armWristRollMemberZeroDeg, armWristRollServoSign};
  cfg.payload.manipJointMap[MANIP_MEMBER_GRIPPER] = {0.0f, 0.0f, 1.0f};
  cfg.crc = calculateConfigExtrasCRC(cfg);
}

// Loads config extras from EEPROM.
bool loadConfigExtrasFromEEPROM() {
  PersistentConfigExtras cfg;
  EEPROM.get(EEPROM_ADDR_CONFIG_EXTRAS, cfg);
  if (cfg.magic == CONFIG_EXTRAS_MAGIC &&
      cfg.schemaId == CONFIG_EXTRAS_SCHEMA_ID &&
      calculateConfigExtrasCRC(cfg) == cfg.crc) {
    applyConfigExtrasToRuntime(cfg);
    return true;
  }
  fillDefaultConfigExtras(cfg);
  applyConfigExtrasToRuntime(cfg);
  return false;
}

// Saves config extras to EEPROM.
void saveConfigExtrasToEEPROM() {
  PersistentConfigExtras cfg;
  syncRuntimeToConfigExtras(cfg);
  EEPROM.put(EEPROM_ADDR_CONFIG_EXTRAS, cfg);
}

// Loads config from EEPROM.
bool loadConfigFromEEPROM() {
  PersistentConfig cfg;
  bool loaded = false;
  EEPROM.get(EEPROM_ADDR, cfg);

  if (cfg.magic == EEPROM_MAGIC &&
      cfg.schemaId == EEPROM_SCHEMA_ID &&
      calculateConfigCRC(cfg) == cfg.crc) {
    applyConfigToRuntime(cfg);
    loaded = true;
  } else {
    fillDefaultConfig(cfg);
    applyConfigToRuntime(cfg);
  }

  loadConfigExtrasFromEEPROM();
  loadCompassCalibrationFromEEPROM();
  loadSafetyConfigFromEEPROM();
  loadRobotControlModeFromEEPROM();
  return loaded;
}

// Saves config to EEPROM.
void saveConfigToEEPROM() {
  PersistentConfig cfg;
  syncRuntimeToConfig(cfg);
  EEPROM.put(EEPROM_ADDR, cfg);
  saveConfigExtrasToEEPROM();
  saveCompassCalibrationToEEPROM();
  saveSafetyConfigToEEPROM();
  saveRobotControlModeToEEPROM();
}

// Utility: erase EEPROM region.
void eraseEEPROMRegion() {
  for (unsigned int i = 0; i < sizeof(PersistentConfig); i++) {
    EEPROM.write(EEPROM_ADDR + i, 0xFF);
  }
  for (unsigned int i = 0; i < 8; i++) {
    EEPROM.write(EEPROM_ADDR_ROBOT_MODE + i, 0xFF);
  }
  for (unsigned int i = 0; i < sizeof(CompassCalibrationData); i++) {
    EEPROM.write(EEPROM_ADDR_COMPASS_CAL + i, 0xFF);
  }
  for (unsigned int i = 0; i < sizeof(PersistentConfigExtras); i++) {
    EEPROM.write(EEPROM_ADDR_CONFIG_EXTRAS + i, 0xFF);
  }
  for (unsigned int i = 0; i < sizeof(PersistentSafetyConfig); i++) {
    EEPROM.write(EEPROM_ADDR_SAFETY_CONFIG + i, 0xFF);
  }
}

// Utility: rebind I2C devices through the generic guarded I2C layer.
void rebindI2CDevices() {
  // PWM expander / external PWM device.  The guarded path keeps setup() alive
  // even when the device, cable or pull-ups leave the I2C bus marginal.
  rt.pwmReady = false;
  if (i2cPrepareDevice(pca9685Addr)) {
    rt.pwmReady = i2cCompleteDeviceInit(initExternalPwmControllerDirect(pca9685Addr));
  }
  if (rt.pwmReady) {
    detachFallbackServos();
  } else {
    externalPwmFailoverToFallback();
  }
  applyBootPwmDefaults();

  // INA219 #1.  All library calls are wrapped by the same generic guard so a
  // current sensor fault cannot freeze the whole firmware.
  rt.ina1Ready = false;
  if (i2cPrepareDevice(ina219Addr1)) {
    ina219_1 = Adafruit_INA219(ina219Addr1);
    rt.ina1Ready = i2cCompleteDeviceInit(ina219_1.begin());
    if (rt.ina1Ready) {
      ina219_1.setCalibration_16V_400mA();
      rt.ina1Ready = i2cCompleteDeviceInit(true);
    }
  }

  // INA219 #2.
  rt.ina2Ready = false;
  if (i2cPrepareDevice(ina219Addr2)) {
    ina219_2 = Adafruit_INA219(ina219Addr2);
    rt.ina2Ready = i2cCompleteDeviceInit(ina219_2.begin());
    if (rt.ina2Ready) {
      ina219_2.setCalibration_16V_400mA();
      rt.ina2Ready = i2cCompleteDeviceInit(true);
    }
  }

  // MPU6050 #1.
  rt.mpu1Ready = false;
  if (i2cPrepareDevice(mpu6050Addr1)) {
    mpu6050_1 = MPU6050(mpu6050Addr1);
    mpu6050_1.initialize();
    rt.mpu1Ready = i2cCompleteDeviceInit(mpu6050_1.testConnection());
  }

  // MPU6050 #2 (0 disables).
  rt.mpu2Ready = false;
  if (i2cPrepareDevice(mpu6050Addr2)) {
    mpu6050_2 = MPU6050(mpu6050Addr2);
    mpu6050_2.initialize();
    rt.mpu2Ready = i2cCompleteDeviceInit(mpu6050_2.testConnection());
  }

  startCompassInitialization();
}



// Returns true when a 7-bit I2C address is usable by the generic guard.
static bool i2cAddressValid(uint8_t addr) {
  return addr >= 1 && addr <= 0x7F;
}

// Clears Wire timeout state without tying the firmware to one specific Wire core.
static void i2cClearTimeoutFlag() {
#if defined(WIRE_HAS_TIMEOUT)
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
  }
#endif
}

// Returns true when the last Wire transaction hit the configured timeout.
static bool i2cTimedOut() {
#if defined(WIRE_HAS_TIMEOUT)
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    return true;
  }
#endif
  return false;
}

// Returns true when the physical I2C lines appear released.
static bool i2cBusLooksIdle() {
#if defined(SDA) && defined(SCL)
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  return digitalRead(SDA) == HIGH && digitalRead(SCL) == HIGH;
#else
  return true;
#endif
}

// Releases a stuck I2C slave by clocking SCL and sending a STOP condition.
// This routine is intentionally generic: it is safe to call before accessing
// any I2C peripheral, not only the external PWM board.
static void recoverI2CBusPins() {
#if defined(SDA) && defined(SCL)
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(50);

  if (digitalRead(SDA) == HIGH && digitalRead(SCL) == HIGH) return;

  pinMode(SDA, INPUT_PULLUP);
  for (uint8_t i = 0; i < 18 && digitalRead(SDA) == LOW; i++) {
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, LOW);
    delayMicroseconds(8);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(8);
  }

  // STOP condition: SDA low while SCL is high, then release SDA.
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(8);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(8);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(50);
#endif
}

// Prepares the I2C bus for any transaction.  It never blocks indefinitely;
// callers receive false and can mark that peripheral unavailable.
static bool i2cPrepareBus() {
  i2cClearTimeoutFlag();
  if (!i2cBusLooksIdle()) recoverI2CBusPins();
  if (!i2cBusLooksIdle()) return false;
  i2cClearTimeoutFlag();
  return true;
}

// Finalizes one Wire operation and centralizes timeout recovery.
static bool i2cTransactionOk(uint8_t wireErr) {
  if (i2cTimedOut()) {
    recoverI2CBusPins();
    return false;
  }
  if (wireErr != 0) {
    return false;
  }
  return true;
}

// Utility: guarded I2C device probe.
static bool i2cDevicePresent(uint8_t addr) {
  if (!i2cAddressValid(addr)) return false;
  if (!i2cPrepareBus()) return false;
  Wire.beginTransmission(addr);
  return i2cTransactionOk(Wire.endTransmission());
}

// Generic guard to be used before initializing any I2C peripheral or library.
static bool i2cPrepareDevice(uint8_t addr) {
  if (!i2cAddressValid(addr)) return false;
  return i2cDevicePresent(addr);
}

// Generic guard to be used after any I2C peripheral/library initialization.
static bool i2cCompleteDeviceInit(bool initOk) {
  if (i2cTimedOut()) {
    recoverI2CBusPins();
    return false;
  }
  if (!i2cBusLooksIdle()) {
    recoverI2CBusPins();
    return false;
  }
  Wire.setClock(I2C_CLOCK_SPEED);
  return initOk;
}

// Utility: checked I2C write 8.
static bool i2cWrite8Checked(uint8_t addr, uint8_t reg, uint8_t val) {
  return i2cWriteBlockChecked(addr, reg, &val, 1);
}

// Utility: I2C write 8.
static void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  (void)i2cWrite8Checked(addr, reg, val);
}

// Utility: I2C read 8.
static bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t& val) {
  return i2cReadBytes(addr, reg, &val, 1);
}

// Utility: guarded I2C read bytes.
static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  if (!i2cAddressValid(addr) || buf == 0 || len == 0) return false;
  if (!i2cPrepareBus()) return false;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (!i2cTransactionOk(Wire.endTransmission(false))) return false;

  uint8_t got = Wire.requestFrom((int)addr, (int)len);
  if (i2cTimedOut()) {
    recoverI2CBusPins();
    return false;
  }
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// Utility: guarded I2C block write for any device register.
static bool i2cWriteBlockChecked(uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len) {
  if (!i2cAddressValid(addr) || data == 0 || len == 0) return false;
  if (!i2cPrepareBus()) return false;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++) Wire.write(data[i]);
  return i2cTransactionOk(Wire.endTransmission());
}

static void externalPwmFailoverToFallback() {
  rt.pwmReady = false;
  if (robotControlMode == ROBOT_MODE_MANIPULATOR) {
    attachFallbackServos();
  } else {
    detachFallbackServos();
  }
  configureFallbackOutputs();
}

static bool externalPwmWriteBlock(uint8_t reg, const uint8_t* data, uint8_t len) {
  return i2cWriteBlockChecked(pca9685Addr, reg, data, len);
}

// Direct initialization for the external PWM expander at 50 Hz.  This avoids
// a blocking library begin() path and keeps setup() alive even when the I2C bus
// is marginal.  All register constants use local project names to avoid macro
// collisions with third-party libraries.
static bool initExternalPwmControllerDirect(uint8_t addr) {
  if (!i2cPrepareDevice(addr)) return false;

  const uint8_t kExtPwmRegMode1    = 0x00;
  const uint8_t kExtPwmRegMode2    = 0x01;
  const uint8_t kExtPwmRegPrescale = 0xFE;
  const uint8_t kExtPwmModeSleep   = 0x10;
  const uint8_t kExtPwmModeAutoInc = 0x20;
  const uint8_t kExtPwmModeRestart = 0x80;
  const uint8_t kExtPwmModeOutDrv  = 0x04;
  const uint8_t kExtPwmPrescale50Hz = 121; // round(25 MHz / (4096 * 50 Hz)) - 1

  uint8_t oldMode = 0x00;
  if (!i2cRead8(addr, kExtPwmRegMode1, oldMode)) return false;

  uint8_t sleepMode = (oldMode & 0x7F) | kExtPwmModeSleep;
  if (!i2cWrite8Checked(addr, kExtPwmRegMode1, sleepMode)) return false;
  if (!i2cWrite8Checked(addr, kExtPwmRegPrescale, kExtPwmPrescale50Hz)) return false;
  if (!i2cWrite8Checked(addr, kExtPwmRegMode2, kExtPwmModeOutDrv)) return false;
  if (!i2cWrite8Checked(addr, kExtPwmRegMode1, (oldMode & ~kExtPwmModeSleep) | kExtPwmModeAutoInc)) return false;
  delay(5);
  if (!i2cWrite8Checked(addr, kExtPwmRegMode1, kExtPwmModeRestart | kExtPwmModeAutoInc)) return false;

  return i2cCompleteDeviceInit(true);
}

static bool externalPwmSetPWM(uint8_t ch, uint16_t on, uint16_t off) {
  if (!rt.pwmReady || ch >= 16) return false;
  on = clampValue(on, (uint16_t)0, (uint16_t)4096);
  off = clampValue(off, (uint16_t)0, (uint16_t)4096);

  uint8_t payload[4] = {
    (uint8_t)(on & 0xFF),
    (uint8_t)(on >> 8),
    (uint8_t)(off & 0xFF),
    (uint8_t)(off >> 8)
  };
  bool ok = externalPwmWriteBlock((uint8_t)(0x06 + 4 * ch), payload, 4);
  if (!ok) {
    externalPwmFailoverToFallback();
  }
  return ok;
}

// Initializes 5883.
bool initHMC5883(uint8_t addr) {
  if (!i2cDevicePresent(addr)) return false;
  uint8_t id[3] = {0, 0, 0};
  if (!i2cReadBytes(addr, 0x0A, id, 3)) return false;
  if (!(id[0] == 'H' && id[1] == '4' && id[2] == '3')) return false;
  i2cWrite8(addr, 0x00, 0x70); // 8 samples @15Hz
  i2cWrite8(addr, 0x01, 0x20); // gain
  i2cWrite8(addr, 0x02, 0x00); // continuous
  compassType = COMPASS_HMC5883;
  compassAddr = addr;
  return true;
}

// Initializes 5883.
bool initQMC5883(uint8_t addr) {
  if (!i2cDevicePresent(addr)) return false;
  // Soft reset + continuous 200Hz 8G OSR512.
  // The async initializer applies the required settle windows without blocking.
  i2cWrite8(addr, 0x0B, 0x01);
  i2cWrite8(addr, 0x09, 0x1D);
  i2cWrite8(addr, 0x0A, 0x00);
  uint8_t buf[6];
  if (!i2cReadBytes(addr, 0x00, buf, 6)) return false;
  compassType = COMPASS_QMC5883;
  compassAddr = addr;
  return true;
}

// Starts compass initialization.
void startCompassInitialization() {
  rt.compassReady = false;
  compassType = COMPASS_NONE;

  compassInit.active = true;
  compassInit.resultReported = false;
  compassInit.candidateIndex = 0;
  compassInit.currentAddr = 0;
  compassInit.waitStartMs = 0;
  compassInit.stage = COMPASS_INIT_SELECT_CANDIDATE;

  if (compassAddr != 0) {
    compassInit.candidateCount = 1;
    compassInit.candidateAddrs[0] = compassAddr;
    compassInit.candidateAddrs[1] = 0;
  } else {
    compassInit.candidateCount = 2;
    compassInit.candidateAddrs[0] = 0x1E;
    compassInit.candidateAddrs[1] = 0x0D;
  }
}

// Checks whether compass initialization busy.
bool isCompassInitializationBusy() {
  return compassInit.active;
}

// Utility: advance compass candidate.
static void advanceCompassCandidate() {
  compassInit.candidateIndex++;
  compassInit.currentAddr = 0;
  compassInit.waitStartMs = 0;
  compassInit.stage = COMPASS_INIT_SELECT_CANDIDATE;
}

// Finishes compass initialization success.
static void finishCompassInitializationSuccess(CompassType type, uint8_t addr) {
  compassType = type;
  compassAddr = addr;
  rt.compassReady = true;
  compassInit.active = false;
  compassInit.stage = COMPASS_INIT_IDLE;
}

// Finishes compass initialization failure.
static void finishCompassInitializationFailure() {
  compassType = COMPASS_NONE;
  rt.compassReady = false;
  compassInit.active = false;
  compassInit.stage = COMPASS_INIT_IDLE;
}

// Updates compass initialization.
void updateCompassInitialization() {
  if (!compassInit.active) return;

  unsigned long nowMs = millis();

  for (uint8_t guard = 0; guard < 4 && compassInit.active; guard++) {
    switch (compassInit.stage) {
      case COMPASS_INIT_IDLE:
        return;

      case COMPASS_INIT_SELECT_CANDIDATE:
        if (compassInit.candidateIndex >= compassInit.candidateCount) {
          finishCompassInitializationFailure();
          return;
        }
        compassInit.currentAddr = compassInit.candidateAddrs[compassInit.candidateIndex];
        compassInit.stage = COMPASS_INIT_TRY_HMC;
        break;

      case COMPASS_INIT_TRY_HMC: {
        uint8_t addr = compassInit.currentAddr;
        if (!i2cDevicePresent(addr)) {
          advanceCompassCandidate();
          break;
        }

        uint8_t id[3] = {0, 0, 0};
        if (i2cReadBytes(addr, 0x0A, id, 3) && id[0] == 'H' && id[1] == '4' && id[2] == '3') {
          i2cWrite8(addr, 0x00, 0x70);
          i2cWrite8(addr, 0x01, 0x20);
          i2cWrite8(addr, 0x02, 0x00);
          compassInit.waitStartMs = nowMs;
          compassInit.stage = COMPASS_INIT_WAIT_HMC_SETTLE;
          return;
        }

        compassInit.stage = COMPASS_INIT_TRY_QMC_RESET;
        break;
      }

      case COMPASS_INIT_WAIT_HMC_SETTLE:
        if ((nowMs - compassInit.waitStartMs) < COMPASS_HMC_INIT_SETTLE_MS) return;
        finishCompassInitializationSuccess(COMPASS_HMC5883, compassInit.currentAddr);
        return;

      case COMPASS_INIT_TRY_QMC_RESET:
        i2cWrite8(compassInit.currentAddr, 0x0B, 0x01);
        compassInit.waitStartMs = nowMs;
        compassInit.stage = COMPASS_INIT_WAIT_QMC_RESET;
        return;

      case COMPASS_INIT_WAIT_QMC_RESET:
        if ((nowMs - compassInit.waitStartMs) < COMPASS_QMC_RESET_SETTLE_MS) return;
        i2cWrite8(compassInit.currentAddr, 0x09, 0x1D);
        i2cWrite8(compassInit.currentAddr, 0x0A, 0x00);
        compassInit.waitStartMs = nowMs;
        compassInit.stage = COMPASS_INIT_WAIT_QMC_CONFIG;
        return;

      case COMPASS_INIT_WAIT_QMC_CONFIG: {
        if ((nowMs - compassInit.waitStartMs) < COMPASS_QMC_CONFIG_SETTLE_MS) return;
        uint8_t buf[6];
        if (i2cReadBytes(compassInit.currentAddr, 0x00, buf, 6)) {
          finishCompassInitializationSuccess(COMPASS_QMC5883, compassInit.currentAddr);
          return;
        }
        advanceCompassCandidate();
        break;
      }
    }
  }
}

// Detects and init compass.
bool detectAndInitCompass() {
  const unsigned long detectTimeoutMs = 350UL;
  startCompassInitialization();
  unsigned long startMs = millis();

  while (isCompassInitializationBusy() && (millis() - startMs) < detectTimeoutMs) {
    updateCompassInitialization();
  }

  if (isCompassInitializationBusy()) {
    finishCompassInitializationFailure();
  }

  compassInit.resultReported = true;
  return rt.compassReady;
}

// Reads compass raw.
bool readCompassRaw(int16_t &mx, int16_t &my, int16_t &mz) {
  mx = my = mz = 0;
  if (!rt.compassReady || compassType == COMPASS_NONE || compassAddr == 0) return false;

  if (compassType == COMPASS_HMC5883) {
    uint8_t buf[6];
    if (!i2cReadBytes(compassAddr, 0x03, buf, 6)) return false;
    mx = (int16_t)((buf[0] << 8) | buf[1]);
    mz = (int16_t)((buf[2] << 8) | buf[3]);
    my = (int16_t)((buf[4] << 8) | buf[5]);
    return true;
  }

  if (compassType == COMPASS_QMC5883) {
    uint8_t status = 0;
    if (!i2cReadBytes(compassAddr, 0x06, &status, 1)) return false;
    uint8_t buf[6];
    if (!i2cReadBytes(compassAddr, 0x00, buf, 6)) return false;
    mx = (int16_t)((buf[1] << 8) | buf[0]);
    my = (int16_t)((buf[3] << 8) | buf[2]);
    mz = (int16_t)((buf[5] << 8) | buf[4]);
    return true;
  }

  return false;
}

// Computes compass heading degrees.
float computeCompassHeadingDeg(int16_t mx, int16_t my) {
  float cx = ((float)mx - compassCal.offsetX) * compassCal.scaleX;
  float cy = ((float)my - compassCal.offsetY) * compassCal.scaleY;
  float heading = atan2(cy, cx) * 180.0f / PI;
  if (heading < 0.0f) heading += 360.0f;
  if (heading >= 360.0f) heading -= 360.0f;
  return heading;
}

// Utility: unwrap stream command.
// The runtime stream accepts only the current canonical envelope:
//   #CMD|ROBOT:<name>|SEQ:<n>|TS:<ms>|CMD:<BODY>
bool unwrapStreamCommand(char* cmd) {
  if (cmd == NULL) return false;
  if (strncmp(cmd, "#CMD|", 5) != 0) return false;

  char* savePtr = NULL;
  char* tok = strtok_r(cmd + 5, "|", &savePtr);
  char* robotTok = NULL;
  char* seqTok = NULL;
  char* tsTok = NULL;
  char* bodyTok = NULL;
  int tokenCount = 0;

  while (tok != NULL) {
    if (tokenCount == 0) robotTok = tok;
    else if (tokenCount == 1) seqTok = tok;
    else if (tokenCount == 2) tsTok = tok;
    else if (tokenCount == 3) bodyTok = tok;
    else return false;
    tokenCount++;
    tok = strtok_r(NULL, "|", &savePtr);
  }

  if (tokenCount != 4) return false;
  if (robotTok == NULL || seqTok == NULL || tsTok == NULL || bodyTok == NULL) return false;
  if (strncmp(robotTok, "ROBOT:", 6) != 0) return false;
  if (strncmp(seqTok, "SEQ:", 4) != 0) return false;
  if (strncmp(tsTok, "TS:", 3) != 0) return false;
  if (strncmp(bodyTok, "CMD:", 4) == 0) bodyTok += 4;
  if (bodyTok[0] == '\0') return false;

  const char* seqText = seqTok + 4;
  const char* tsText = tsTok + 3;
  if (*seqText == '\0' || *tsText == '\0') return false;
  for (const char* p = seqText; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
  }
  for (const char* p = tsText; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
  }

  lastRuntimeStreamSeq = strtoul(seqText, NULL, 10);
  lastRuntimeStreamRxMs = millis();
  runtimeStreamActive = true;
  runtimeStreamFailsafeLatched = false;
  clearRuntimeLinkHoldReturnState();

  strncpy(cmd, bodyTok, SERIAL_TEXT_BUFFER_SIZE - 1);
  cmd[SERIAL_TEXT_BUFFER_SIZE - 1] = '\0';
  return true;
}

// Resets compass calibration data.
void resetCompassCalibrationData() {
  compassCal.magic = COMPASS_CAL_MAGIC;
  compassCal.minX = -2048; compassCal.maxX = 2048;
  compassCal.minY = -2048; compassCal.maxY = 2048;
  compassCal.minZ = -2048; compassCal.maxZ = 2048;
  compassCal.offsetX = 0.0f; compassCal.offsetY = 0.0f; compassCal.offsetZ = 0.0f;
  compassCal.scaleX = 1.0f; compassCal.scaleY = 1.0f; compassCal.scaleZ = 1.0f;
}

// Loads compass calibration from EEPROM.
void loadCompassCalibrationFromEEPROM() {
  EEPROM.get(EEPROM_ADDR_COMPASS_CAL, compassCal);
  if (compassCal.magic != COMPASS_CAL_MAGIC) {
    resetCompassCalibrationData();
    return;
  }
  if (compassCal.scaleX == 0.0f) compassCal.scaleX = 1.0f;
  if (compassCal.scaleY == 0.0f) compassCal.scaleY = 1.0f;
  if (compassCal.scaleZ == 0.0f) compassCal.scaleZ = 1.0f;
}

// Saves compass calibration to EEPROM.
void saveCompassCalibrationToEEPROM() {
  compassCal.magic = COMPASS_CAL_MAGIC;
  EEPROM.put(EEPROM_ADDR_COMPASS_CAL, compassCal);
}

// Parses compass calibration config value.
bool parseCompassCalibrationConfigValue(const char* text, CompassCalibrationData& out) {
  if (text == NULL) return false;
  char buf[192];
  strncpy(buf, text, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  float values[12] = {0};
  uint8_t count = 0;
  char* tok = strtok(buf, ",");
  while (tok != NULL && count < 12) {
    values[count++] = (float)atof(tok);
    tok = strtok(NULL, ",");
  }
  if (tok != NULL || count != 12) return false;

  out.magic = COMPASS_CAL_MAGIC;
  out.minX = (int16_t)roundf(values[0]);
  out.maxX = (int16_t)roundf(values[1]);
  out.minY = (int16_t)roundf(values[2]);
  out.maxY = (int16_t)roundf(values[3]);
  out.minZ = (int16_t)roundf(values[4]);
  out.maxZ = (int16_t)roundf(values[5]);
  out.offsetX = values[6];
  out.offsetY = values[7];
  out.offsetZ = values[8];
  out.scaleX = (fabsf(values[9]) < 0.0001f) ? 1.0f : values[9];
  out.scaleY = (fabsf(values[10]) < 0.0001f) ? 1.0f : values[10];
  out.scaleZ = (fabsf(values[11]) < 0.0001f) ? 1.0f : values[11];
  if (out.minX > out.maxX || out.minY > out.maxY || out.minZ > out.maxZ) return false;
  return true;
}

// Starts compass calibration.
void startCompassCalibration() {
  if (!rt.compassReady) {
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("MAGCAL:NO_COMPASS"));
    return;
  }
  compassCalRuntime.active = true;
  compassCalRuntime.saved = false;
  compassCalRuntime.manualMode = (robotControlMode != ROBOT_MODE_MANIPULATOR);
  compassCalRuntime.stepIndex = 0;
  compassCalRuntime.subStep = 0;
  compassCalRuntime.stepStartedMs = millis();
  compassCalRuntime.startedMs = millis();
  compassCalRuntime.lastSampleMs = 0;
  compassCalRuntime.progress = 0.0f;
  compassCalRuntime.minX = 32767; compassCalRuntime.maxX = -32768;
  compassCalRuntime.minY = 32767; compassCalRuntime.maxY = -32768;
  compassCalRuntime.minZ = 32767; compassCalRuntime.maxZ = -32768;
  runtimeStreamFailsafeLatched = false;
  if (allowPrimaryAsciiStatusMessages()) Serial.println(F("MAGCAL:START"));
}

// Utility: cancel compass calibration.
void cancelCompassCalibration(bool saveResult) {
  if (!compassCalRuntime.active) return;
  compassCalRuntime.active = false;
  if (saveResult) {
    float ox = (compassCalRuntime.maxX + compassCalRuntime.minX) * 0.5f;
    float oy = (compassCalRuntime.maxY + compassCalRuntime.minY) * 0.5f;
    float oz = (compassCalRuntime.maxZ + compassCalRuntime.minZ) * 0.5f;
    float rx = max(1.0f, (compassCalRuntime.maxX - compassCalRuntime.minX) * 0.5f);
    float ry = max(1.0f, (compassCalRuntime.maxY - compassCalRuntime.minY) * 0.5f);
    float rz = max(1.0f, (compassCalRuntime.maxZ - compassCalRuntime.minZ) * 0.5f);
    float avgR = (rx + ry + rz) / 3.0f;
    compassCal.minX = compassCalRuntime.minX; compassCal.maxX = compassCalRuntime.maxX;
    compassCal.minY = compassCalRuntime.minY; compassCal.maxY = compassCalRuntime.maxY;
    compassCal.minZ = compassCalRuntime.minZ; compassCal.maxZ = compassCalRuntime.maxZ;
    compassCal.offsetX = ox; compassCal.offsetY = oy; compassCal.offsetZ = oz;
    compassCal.scaleX = avgR / rx; compassCal.scaleY = avgR / ry; compassCal.scaleZ = avgR / rz;
    saveCompassCalibrationToEEPROM();
    compassCalRuntime.saved = true;
    if (allowPrimaryAsciiStatusMessages()) Serial.println(F("MAGCAL:SAVED"));
  } else if (allowPrimaryAsciiStatusMessages()) {
    Serial.println(F("MAGCAL:CANCELLED"));
  }
}

// Sets manipulator compass calibration pose.
void setManipulatorCompassCalibrationPose(uint8_t index) {
  static const float PROGMEM baseAngles[8] = {0, 45, 90, 135, 180, 225, 270, 315};
  static const float PROGMEM wristRot[8] = {30, 150, 30, 150, 30, 150, 30, 150};
  static const float PROGMEM wristPitch[8] = {70, 110, 70, 110, 70, 110, 70, 110};
  uint8_t i = index % 8;

  uint8_t baseCh = getManipulatorMemberChannel(MANIP_MEMBER_BASE);
  float baseTarget = clampManipulatorMemberTargetToLimits(MANIP_MEMBER_BASE, pgm_read_float(&baseAngles[i]));
  if (baseCh == 0) {
    motor.targetAngle = baseTarget;
    servos.target[0] = baseTarget;
  } else if (isServoChannel(baseCh)) {
    servos.target[baseCh] = clampServoTargetToLimits(baseCh, baseTarget);
  }

  uint8_t wristPitchCh = getManipulatorMemberChannel(MANIP_MEMBER_WRIST_PITCH);
  uint8_t wristRotCh = getManipulatorMemberChannel(MANIP_MEMBER_WRIST_ROLL);
  if (isServoChannel(wristPitchCh)) {
    servos.target[wristPitchCh] = clampManipulatorMemberTargetToLimits(MANIP_MEMBER_WRIST_PITCH, pgm_read_float(&wristPitch[i]));
  }
  if (isServoChannel(wristRotCh)) {
    servos.target[wristRotCh] = clampManipulatorMemberTargetToLimits(MANIP_MEMBER_WRIST_ROLL, pgm_read_float(&wristRot[i]));
  }
}

// Updates compass calibration.
void updateCompassCalibration() {
  if (!compassCalRuntime.active) return;
  unsigned long now = millis();

  int16_t mx = 0, my = 0, mz = 0;
  if ((now - compassCalRuntime.lastSampleMs) >= 60 && readCompassRaw(mx, my, mz)) {
    compassCalRuntime.lastSampleMs = now;
    if (mx < compassCalRuntime.minX) compassCalRuntime.minX = mx;
    if (mx > compassCalRuntime.maxX) compassCalRuntime.maxX = mx;
    if (my < compassCalRuntime.minY) compassCalRuntime.minY = my;
    if (my > compassCalRuntime.maxY) compassCalRuntime.maxY = my;
    if (mz < compassCalRuntime.minZ) compassCalRuntime.minZ = mz;
    if (mz > compassCalRuntime.maxZ) compassCalRuntime.maxZ = mz;
  }

  if (robotControlMode == ROBOT_MODE_MANIPULATOR) {
    if (compassCalRuntime.subStep == 0) {
      setManipulatorCompassCalibrationPose(compassCalRuntime.stepIndex);
      compassCalRuntime.stepStartedMs = now;
      compassCalRuntime.subStep = 1;
    } else if ((now - compassCalRuntime.stepStartedMs) >= 900) {
      compassCalRuntime.stepIndex++;
      compassCalRuntime.subStep = 0;
    }
    compassCalRuntime.progress = min(100.0f, (compassCalRuntime.stepIndex / 8.0f) * 100.0f);
    if (compassCalRuntime.stepIndex >= 8) cancelCompassCalibration(true);
  } else if (robotControlMode == ROBOT_MODE_VEHICLE) {
    // Vehicle compass calibration now samples while the operator drives.
    // This keeps the diagnostic button aligned with the firmware state
    // without overriding the active track command.
    compassCalRuntime.progress = min(100.0f, ((now - compassCalRuntime.startedMs) / 12000.0f) * 100.0f);
    if ((now - compassCalRuntime.startedMs) >= 12000UL) cancelCompassCalibration(true);
  } else {
    // Drone compass calibration now samples while the vehicle is being flown.
    // The firmware owns the calibration session and progress, but it does not
    // inject or cancel flight commands during the sweep.
    compassCalRuntime.progress = min(100.0f, ((now - compassCalRuntime.startedMs) / 15000.0f) * 100.0f);
    if ((now - compassCalRuntime.startedMs) >= 15000UL) cancelCompassCalibration(true);
  }
}


// Records the latest active vehicle motion that can be inverted on runtime link loss.
void recordVehicleLinkHoldMotion() {
  int dead = max(1, (int)roundf(vehicleControlProfile.motorDeadbandPct));
  if (abs(vehicleLeftTrackPct) <= dead && abs(vehicleRightTrackPct) <= dead) {
    vehicleLinkHoldLastMotionValid = false;
    return;
  }
  vehicleLinkHoldLastLeftPct = constrain(vehicleLeftTrackPct, -100, 100);
  vehicleLinkHoldLastRightPct = constrain(vehicleRightTrackPct, -100, 100);
  vehicleLinkHoldLastMotionValid = true;
}

// Records the latest active drone motion that can be inverted on runtime link loss.
void recordDroneLinkHoldMotion() {
  int dead = max(1, (int)roundf(droneControlProfile.commandDeadbandPct));
  if (abs(droneThrottlePct) <= dead && abs(droneYawPct) <= dead &&
      abs(dronePitchPct) <= dead && abs(droneRollPct) <= dead &&
      abs(droneStrafePct) <= dead && abs(droneForwardPct) <= dead) {
    droneLinkHoldLastMotionValid = false;
    return;
  }
  droneLinkHoldLastThrottlePct = constrain(droneThrottlePct, -100, 100);
  droneLinkHoldLastYawPct = constrain(droneYawPct, -100, 100);
  droneLinkHoldLastPitchPct = constrain(dronePitchPct, -100, 100);
  droneLinkHoldLastRollPct = constrain(droneRollPct, -100, 100);
  droneLinkHoldLastStrafePct = constrain(droneStrafePct, -100, 100);
  droneLinkHoldLastForwardPct = constrain(droneForwardPct, -100, 100);
  droneLinkHoldLastMotionValid = true;
}

// Clears the runtime-only link-hold return latch when command link traffic resumes.
void clearRuntimeLinkHoldReturnState() {
  vehicleLinkHoldReturnActive = false;
  droneLinkHoldReturnActive = false;
}

// Applies inverse vehicle movement so the tracked vehicle walks back from the last command.
static bool applyVehicleLinkHoldReturnMotion() {
  int left = vehicleLinkHoldLastMotionValid ? vehicleLinkHoldLastLeftPct : vehicleLeftTrackPct;
  int right = vehicleLinkHoldLastMotionValid ? vehicleLinkHoldLastRightPct : vehicleRightTrackPct;
  if (left == 0 && right == 0) return false;
  vehicleLeftTrackPct = constrain(-left, -100, 100);
  vehicleRightTrackPct = constrain(-right, -100, 100);
  vehicleSafeReturnActive = false;
  vehicleLinkHoldReturnActive = true;
  return true;
}

// Applies inverse drone movement so the drone backs out of the last commanded vector.
static bool applyDroneLinkHoldReturnMotion() {
  int throttle = droneLinkHoldLastMotionValid ? droneLinkHoldLastThrottlePct : droneThrottlePct;
  int yaw = droneLinkHoldLastMotionValid ? droneLinkHoldLastYawPct : droneYawPct;
  int pitch = droneLinkHoldLastMotionValid ? droneLinkHoldLastPitchPct : dronePitchPct;
  int roll = droneLinkHoldLastMotionValid ? droneLinkHoldLastRollPct : droneRollPct;
  int strafe = droneLinkHoldLastMotionValid ? droneLinkHoldLastStrafePct : droneStrafePct;
  int forward = droneLinkHoldLastMotionValid ? droneLinkHoldLastForwardPct : droneForwardPct;
  if (throttle == 0 && yaw == 0 && pitch == 0 && roll == 0 && strafe == 0 && forward == 0) return false;
  droneThrottlePct = constrain(-throttle, -100, 100);
  droneYawPct = constrain(-yaw, -100, 100);
  dronePitchPct = constrain(-pitch, -100, 100);
  droneRollPct = constrain(-roll, -100, 100);
  droneStrafePct = constrain(-strafe, -100, 100);
  droneForwardPct = constrain(-forward, -100, 100);
  droneSafeReturnActive = false;
  droneAutoTakeoffActive = false;
  droneAutoLandActive = false;
  droneLinkHoldReturnActive = true;
  return true;
}

// Notes valid runtime link activity from any host command or handshake.
void noteRuntimeLinkActivity(uint32_t seq) {
  if (seq != 0u) lastRuntimeStreamSeq = seq;
  lastRuntimeStreamRxMs = millis();
  runtimeStreamActive = true;
  runtimeStreamFailsafeLatched = false;
  clearRuntimeLinkHoldReturnState();
}

// Disarms every runtime-controlled output after host link timeout.
void disarmRuntimeOutputsAfterLinkTimeout() {
  directMotorTest.active = false;
  initAutoMotorState();
  initMotorCalState();

  releaseAllManipulatorActuators();
  rt.servoOutputsArmed = false;
  rt.motorArmed = false;
  wristGimbalEnabled = false;
  wristGimbalRefValid = false;

  resetGenericRobotRuntimeState();

  vehicleLeftTrackPct = 0;
  vehicleRightTrackPct = 0;
  vehicleLightsCommand = false;
  vehicleLidarScanCommand = false;
  vehicleLinkHoldReturnActive = false;

  droneThrottlePct = 0;
  droneYawPct = 0;
  dronePitchPct = 0;
  droneRollPct = 0;
  droneStrafePct = 0;
  droneForwardPct = 0;
  droneCameraRecordCommand = false;
  droneSafeReturnActive = false;
  droneAutoTakeoffActive = false;
  droneAutoLandActive = false;
  droneLinkHoldReturnActive = false;

  updateDirectRobotHardwareOutputs();
}


// Utility: enforce runtime stream fail safe.
void enforceRuntimeStreamFailSafe() {
  if (!runtimeStreamActive) return;
  if (!rt.active) return;
  if (compassCalRuntime.active) return;
  unsigned long now = millis();
  if ((now - lastRuntimeStreamRxMs) < STREAM_FAILSAFE_TIMEOUT_MS) return;
  if (runtimeStreamFailsafeLatched) return;
  runtimeStreamFailsafeLatched = true;

  // On link timeout, fail closed: stop the manipulator and release all
  // runtime-controlled motor/servo outputs. A later valid host packet clears
  // runtimeStreamFailsafeLatched through noteRuntimeLinkActivity()/sequence RX.
  disarmRuntimeOutputsAfterLinkTimeout();

  if (allowPrimaryAsciiStatusMessages()) {
    Serial.println(F("STREAM:TIMEOUT DISARM"));
  }
}
