// =====================================================================
// SynROV Firmware V1 - Safety autonomy
// ---------------------------------------------------------------------
// Purpose:
//   Return-to-home using a GPS HOME captured once per active connection.
//   HOME remains in RAM for the whole connection. Drone HOME is copied to
//   EEPROM only below 10% battery as an emergency recovery snapshot.
//   Vehicle incline PWM assist is based on MPU inclination.
// =====================================================================

#ifndef SYNROV_SAFETY_AUTONOMY_H
#define SYNROV_SAFETY_AUTONOMY_H

static void sanitizeSafetyRuntimeParams() {
  droneSafeReturnBatteryThresholdPct = clampValue(droneSafeReturnBatteryThresholdPct, 1.0f, 80.0f);
  vehicleSafeReturnBatteryThresholdPct = clampValue(vehicleSafeReturnBatteryThresholdPct, 1.0f, 80.0f);
  vehicleInclineBasePowerPct = clampValue(vehicleInclineBasePowerPct, 0.0f, 100.0f);
  vehicleInclineMaxPowerPct = clampValue(vehicleInclineMaxPowerPct, vehicleInclineBasePowerPct, 100.0f);
  vehicleInclineDeadbandDeg = clampValue(vehicleInclineDeadbandDeg, 0.0f, 45.0f);
  vehicleInclineFullScaleDeg = clampValue(vehicleInclineFullScaleDeg, vehicleInclineDeadbandDeg + 1.0f, 80.0f);
  vehicleInclinePowerAlpha = clampValue(vehicleInclinePowerAlpha, 0.01f, 1.0f);
}

void fillDefaultSafetyConfig(PersistentSafetyConfig& cfg) {
  cfg.magic = SAFETY_CONFIG_MAGIC;
  cfg.schemaId = SAFETY_CONFIG_SCHEMA_ID;
  cfg.payload.droneSafeReturnEnabled = false;
  cfg.payload.vehicleSafeReturnEnabled = false;
  cfg.payload.vehicleInclinePowerEnabled = false;
  cfg.payload.droneSafeReturnBatteryThresholdPct = SAFE_RETURN_BATTERY_THRESHOLD_DEFAULT_PCT;
  cfg.payload.vehicleSafeReturnBatteryThresholdPct = SAFE_RETURN_BATTERY_THRESHOLD_DEFAULT_PCT;
  cfg.payload.vehicleInclineBasePowerPct = VEHICLE_INCLINE_BASE_POWER_DEFAULT_PCT;
  cfg.payload.vehicleInclineMaxPowerPct = VEHICLE_INCLINE_MAX_POWER_DEFAULT_PCT;
  cfg.payload.vehicleInclineFullScaleDeg = VEHICLE_INCLINE_FULL_SCALE_DEFAULT_DEG;
  cfg.payload.vehicleInclineDeadbandDeg = VEHICLE_INCLINE_DEADBAND_DEFAULT_DEG;
  cfg.payload.vehicleInclinePowerAlpha = VEHICLE_INCLINE_ALPHA_DEFAULT;
  cfg.crc = calculateSafetyConfigCRC(cfg);
}

void applySafetyConfigToRuntime(const PersistentSafetyConfig& cfg) {
  droneSafeReturnEnabled = cfg.payload.droneSafeReturnEnabled;
  vehicleSafeReturnEnabled = cfg.payload.vehicleSafeReturnEnabled;
  vehicleInclinePowerEnabled = cfg.payload.vehicleInclinePowerEnabled;
  droneSafeReturnBatteryThresholdPct = cfg.payload.droneSafeReturnBatteryThresholdPct;
  vehicleSafeReturnBatteryThresholdPct = cfg.payload.vehicleSafeReturnBatteryThresholdPct;
  vehicleInclineBasePowerPct = cfg.payload.vehicleInclineBasePowerPct;
  vehicleInclineMaxPowerPct = cfg.payload.vehicleInclineMaxPowerPct;
  vehicleInclineFullScaleDeg = cfg.payload.vehicleInclineFullScaleDeg;
  vehicleInclineDeadbandDeg = cfg.payload.vehicleInclineDeadbandDeg;
  vehicleInclinePowerAlpha = cfg.payload.vehicleInclinePowerAlpha;
  sanitizeSafetyRuntimeParams();
  vehicleInclineAutoPowerPct = vehicleInclineBasePowerPct;
}

void syncRuntimeToSafetyConfig(PersistentSafetyConfig& cfg) {
  sanitizeSafetyRuntimeParams();
  cfg.magic = SAFETY_CONFIG_MAGIC;
  cfg.schemaId = SAFETY_CONFIG_SCHEMA_ID;
  cfg.payload.droneSafeReturnEnabled = droneSafeReturnEnabled;
  cfg.payload.vehicleSafeReturnEnabled = vehicleSafeReturnEnabled;
  cfg.payload.vehicleInclinePowerEnabled = vehicleInclinePowerEnabled;
  cfg.payload.droneSafeReturnBatteryThresholdPct = droneSafeReturnBatteryThresholdPct;
  cfg.payload.vehicleSafeReturnBatteryThresholdPct = vehicleSafeReturnBatteryThresholdPct;
  cfg.payload.vehicleInclineBasePowerPct = vehicleInclineBasePowerPct;
  cfg.payload.vehicleInclineMaxPowerPct = vehicleInclineMaxPowerPct;
  cfg.payload.vehicleInclineFullScaleDeg = vehicleInclineFullScaleDeg;
  cfg.payload.vehicleInclineDeadbandDeg = vehicleInclineDeadbandDeg;
  cfg.payload.vehicleInclinePowerAlpha = vehicleInclinePowerAlpha;
  cfg.crc = calculateSafetyConfigCRC(cfg);
}

bool loadSafetyConfigFromEEPROM() {
  PersistentSafetyConfig cfg;
  EEPROM.get(EEPROM_ADDR_SAFETY_CONFIG, cfg);
  if (persistentBlockValid(cfg, SAFETY_CONFIG_MAGIC, SAFETY_CONFIG_SCHEMA_ID, calculateSafetyConfigCRC(cfg))) {
    applySafetyConfigToRuntime(cfg);
    return true;
  }
  fillDefaultSafetyConfig(cfg);
  applySafetyConfigToRuntime(cfg);
  EEPROM.put(EEPROM_ADDR_SAFETY_CONFIG, cfg);
  return false;
}

void saveSafetyConfigToEEPROM() {
  PersistentSafetyConfig cfg;
  syncRuntimeToSafetyConfig(cfg);
  EEPROM.put(EEPROM_ADDR_SAFETY_CONFIG, cfg);
}

bool loadDroneEmergencyHomeFromEEPROM() {
  PersistentDroneEmergencyHome home;
  EEPROM.get(EEPROM_ADDR_DRONE_EMERGENCY_HOME, home);
  bool valid = persistentBlockValid(home, DRONE_EMERGENCY_HOME_MAGIC, DRONE_EMERGENCY_HOME_SCHEMA_ID, calculateDroneEmergencyHomeCRC(home));
  droneEmergencyHomeBackupValid = valid;
  if (valid) {
    droneEmergencyHomeLatitudeE7 = home.latitudeE7;
    droneEmergencyHomeLongitudeE7 = home.longitudeE7;
    droneEmergencyHomeAltitudeCm = home.altitudeCm;
  } else {
    droneEmergencyHomeLatitudeE7 = 0;
    droneEmergencyHomeLongitudeE7 = 0;
    droneEmergencyHomeAltitudeCm = 0;
  }
  return valid;
}

void saveDroneEmergencyHomeToEEPROM() {
  if (!droneHomeValid) return;
  PersistentDroneEmergencyHome home;
  home.magic = DRONE_EMERGENCY_HOME_MAGIC;
  home.schemaId = DRONE_EMERGENCY_HOME_SCHEMA_ID;
  home.latitudeE7 = droneHomeLatitudeE7;
  home.longitudeE7 = droneHomeLongitudeE7;
  home.altitudeCm = droneHomeAltitudeCm;
  home.crc = calculateDroneEmergencyHomeCRC(home);
  EEPROM.put(EEPROM_ADDR_DRONE_EMERGENCY_HOME, home);
  droneEmergencyHomeBackupValid = true;
  droneEmergencyHomeLatitudeE7 = home.latitudeE7;
  droneEmergencyHomeLongitudeE7 = home.longitudeE7;
  droneEmergencyHomeAltitudeCm = home.altitudeCm;
  droneEmergencyHomePersistedThisSession = true;
}

void clearDroneEmergencyHomeEEPROM() {
  // Invalidating the magic is sufficient; normal EEPROM.put() later rewrites
  // the complete emergency block only when battery becomes critical.
  EEPROM.update(EEPROM_ADDR_DRONE_EMERGENCY_HOME, 0xFF);
  droneEmergencyHomeBackupValid = false;
  droneEmergencyHomeLatitudeE7 = 0;
  droneEmergencyHomeLongitudeE7 = 0;
  droneEmergencyHomeAltitudeCm = 0;
  droneEmergencyHomePersistedThisSession = false;
}

float currentBatteryPercent() {
  return batteryPercentFromAdc(readBatteryAdcRaw());
}

static bool gpsHomeSourceFresh() {
  return gpsRuntime.portReady && gpsFixFresh();
}

void clearDroneHomeRuntime() {
  droneHomeLatitudeE7 = 0;
  droneHomeLongitudeE7 = 0;
  droneHomeAltitudeCm = 0;
  droneHomeValid = false;
}

void clearVehicleHomeRuntime() {
  vehicleHomeLatitudeE7 = 0;
  vehicleHomeLongitudeE7 = 0;
  vehicleHomeAltitudeCm = 0;
  vehicleHomeValid = false;
}

bool captureDroneHomeFromGps() {
  if (!gpsHomeSourceFresh()) return false;
  droneHomeLatitudeE7 = gpsRuntime.latitudeE7;
  droneHomeLongitudeE7 = gpsRuntime.longitudeE7;
  droneHomeAltitudeCm = gpsRuntime.altitudeCm;
  droneHomeValid = true;
  return true;
}

bool captureVehicleHomeFromGps() {
  if (!gpsHomeSourceFresh()) return false;
  vehicleHomeLatitudeE7 = gpsRuntime.latitudeE7;
  vehicleHomeLongitudeE7 = gpsRuntime.longitudeE7;
  vehicleHomeAltitudeCm = gpsRuntime.altitudeCm;
  vehicleHomeValid = true;
  return true;
}

static float gpsDistanceMeters(int32_t lat1E7, int32_t lon1E7, int32_t lat2E7, int32_t lon2E7) {
  const float earthRadiusM = 6371000.0f;
  float lat1 = (float)lat1E7 * 0.0000001f;
  float lat2 = (float)lat2E7 * 0.0000001f;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(((float)lon2E7 - (float)lon1E7) * 0.0000001f);
  float meanLat = radians((lat1 + lat2) * 0.5f);
  float x = dLon * cosf(meanLat);
  float y = dLat;
  return sqrtf(x * x + y * y) * earthRadiusM;
}

static float gpsBearingDeg(int32_t lat1E7, int32_t lon1E7, int32_t lat2E7, int32_t lon2E7) {
  float lat1 = radians((float)lat1E7 * 0.0000001f);
  float lat2 = radians((float)lat2E7 * 0.0000001f);
  float dLon = radians(((float)lon2E7 - (float)lon1E7) * 0.0000001f);
  float y = sinf(dLon) * cosf(lat2);
  float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dLon);
  float brg = atan2f(y, x) * 57.2957795f;
  if (brg < 0.0f) brg += 360.0f;
  if (brg >= 360.0f) brg -= 360.0f;
  return brg;
}

static float getNavigationHeadingDeg(float fallbackYawDeg) {
  if (rt.compassReady) {
    int16_t mx = 0, my = 0, mz = 0;
    if (readCompassRaw(mx, my, mz)) {
      compassX = mx; compassY = my; compassZ = mz;
      compassHeadingDeg = computeCompassHeadingDeg(mx, my);
    }
    return compassHeadingDeg;
  }
  if (gpsFixFresh() && gpsRuntime.speedCms > 25) return gpsRuntime.courseDegX100 * 0.01f;
  return fallbackYawDeg;
}

void resetSafetyAutonomyRuntimeState() {
  // Robot-mode changes reset active autonomy but MUST NOT redefine HOME.
  // HOME is owned by the active connection session.
  droneSafeReturnActive = false;
  vehicleSafeReturnActive = false;
  vehicleInclinePitchDeg = 0.0f;
  vehicleInclineRollDeg = 0.0f;
  vehicleInclineMagnitudeDeg = 0.0f;
  vehicleInclineAutoPowerPct = vehicleInclineBasePowerPct;
  vehicleInclineLastSampleMs = 0;
}

void beginSafeHomeSession() {
  if (safeHomeSessionActive) return;
  safeHomeSessionActive = true;
  droneSafeReturnActive = false;
  vehicleSafeReturnActive = false;
  droneHomeCapturedThisSession = false;
  vehicleHomeCapturedThisSession = false;
  droneEmergencyHomePersistedThisSession = false;
  clearDroneHomeRuntime();
  clearVehicleHomeRuntime();

  // Emergency restoration is intentionally exceptional: only a Drone that
  // boots while already below 10% may reuse the EEPROM snapshot. A healthy
  // connection always waits for its own first fresh GPS fix.
  if (droneEmergencyHomeBackupValid && currentBatteryPercent() < DRONE_HOME_PERSIST_BATTERY_THRESHOLD_PCT) {
    droneHomeLatitudeE7 = droneEmergencyHomeLatitudeE7;
    droneHomeLongitudeE7 = droneEmergencyHomeLongitudeE7;
    droneHomeAltitudeCm = droneEmergencyHomeAltitudeCm;
    droneHomeValid = true;
    droneHomeCapturedThisSession = true;
    droneEmergencyHomePersistedThisSession = true;
  }
}

void endSafeHomeSession() {
  safeHomeSessionActive = false;
  droneSafeReturnActive = false;
  vehicleSafeReturnActive = false;
  droneHomeCapturedThisSession = false;
  vehicleHomeCapturedThisSession = false;
  droneEmergencyHomePersistedThisSession = false;
  clearDroneHomeRuntime();
  clearVehicleHomeRuntime();
}

void updateVehicleInclinePowerRuntime() {
  if (!vehicleInclinePowerEnabled || robotControlMode != ROBOT_MODE_VEHICLE || !rt.active) {
    vehicleInclineAutoPowerPct = vehicleInclineBasePowerPct;
    return;
  }
  unsigned long nowMs = millis();
  if (vehicleInclineLastSampleMs != 0 && (nowMs - vehicleInclineLastSampleMs) < VEHICLE_INCLINE_SAMPLE_MS) return;
  vehicleInclineLastSampleMs = nowMs;

  float measuredPitch = vehicleInclinePitchDeg;
  float measuredRoll = vehicleInclineRollDeg;
  if (rt.mpu1Ready) {
    int16_t ax = 0, ay = 0, az = 16384, gx = 0, gy = 0, gz = 0;
    mpu6050_1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    measuredPitch = estimatePitchDeg(ax, ay, az);
    measuredRoll = estimateRollDeg(ax, ay, az);
  }

  vehicleInclinePitchDeg = blendFloat(vehicleInclinePitchDeg, measuredPitch, 0.25f);
  vehicleInclineRollDeg = blendFloat(vehicleInclineRollDeg, measuredRoll, 0.25f);
  float mag = sqrtf(vehicleInclinePitchDeg * vehicleInclinePitchDeg + vehicleInclineRollDeg * vehicleInclineRollDeg);
  vehicleInclineMagnitudeDeg = blendFloat(vehicleInclineMagnitudeDeg, mag, 0.25f);

  float usableSpan = max(1.0f, vehicleInclineFullScaleDeg - vehicleInclineDeadbandDeg);
  float ratio = clampValue((vehicleInclineMagnitudeDeg - vehicleInclineDeadbandDeg) / usableSpan, 0.0f, 1.0f);
  float targetPower = vehicleInclineBasePowerPct + ratio * (vehicleInclineMaxPowerPct - vehicleInclineBasePowerPct);
  vehicleInclineAutoPowerPct = blendFloat(vehicleInclineAutoPowerPct, targetPower, vehicleInclinePowerAlpha);
}

void applyVehicleInclinePowerAssist(int &leftTargetPct, int &rightTargetPct) {
  if (!vehicleInclinePowerEnabled || robotControlMode != ROBOT_MODE_VEHICLE) return;
  updateVehicleInclinePowerRuntime();
  int minPct = (int)roundf(clampValue(vehicleInclineAutoPowerPct, 0.0f, 100.0f));
  if (leftTargetPct > 0) leftTargetPct = max(leftTargetPct, minPct);
  else if (leftTargetPct < 0) leftTargetPct = -max(abs(leftTargetPct), minPct);
  if (rightTargetPct > 0) rightTargetPct = max(rightTargetPct, minPct);
  else if (rightTargetPct < 0) rightTargetPct = -max(abs(rightTargetPct), minPct);
}

static void updateDroneSafeReturnHomeCapture() {
  if (!safeHomeSessionActive || !rt.active || droneHomeCapturedThisSession) return;
  if (captureDroneHomeFromGps()) {
    droneHomeCapturedThisSession = true;
    // A healthy session has a fresh RAM HOME, so a persisted emergency
    // snapshot must never become its normal reference.
    if (droneEmergencyHomeBackupValid && currentBatteryPercent() >= DRONE_HOME_PERSIST_BATTERY_THRESHOLD_PCT) {
      clearDroneEmergencyHomeEEPROM();
    }
  }
}

static void updateDroneEmergencyHomePersistence() {
  if (!safeHomeSessionActive || !rt.active || !droneHomeValid) return;
  if (droneEmergencyHomePersistedThisSession) return;
  if (currentBatteryPercent() < DRONE_HOME_PERSIST_BATTERY_THRESHOLD_PCT) {
    saveDroneEmergencyHomeToEEPROM();
  }
}

static void updateVehicleSafeReturnHomeCapture() {
  if (!safeHomeSessionActive || !rt.active || vehicleHomeCapturedThisSession) return;
  if (captureVehicleHomeFromGps()) vehicleHomeCapturedThisSession = true;
}

static void updateDroneSafeReturn() {
  if (!droneSafeReturnEnabled || robotControlMode != ROBOT_MODE_DRONE || !rt.active) {
    droneSafeReturnActive = false;
    return;
  }

  float batteryPct = currentBatteryPercent();
  bool linkLossReturn = runtimeStreamFailsafeLatched;
  if (!droneSafeReturnActive && (batteryPct <= droneSafeReturnBatteryThresholdPct || linkLossReturn)) droneSafeReturnActive = true;
  if (droneSafeReturnActive && !linkLossReturn && batteryPct >= droneSafeReturnBatteryThresholdPct + SAFE_RETURN_BATTERY_HYSTERESIS_PCT) droneSafeReturnActive = false;
  if (!droneSafeReturnActive) return;

  droneAutoTakeoffActive = false;
  if (!droneHomeValid || !gpsFixFresh()) {
    // No safe GPS path: controlled descent is safer than open-loop navigation.
    zeroDroneFlightCommand();
    droneAutoLandActive = true;
    return;
  }

  float distanceM = gpsDistanceMeters(gpsRuntime.latitudeE7, gpsRuntime.longitudeE7, droneHomeLatitudeE7, droneHomeLongitudeE7);
  if (distanceM <= SAFE_RETURN_HOME_REACHED_M) {
    zeroDroneFlightCommand();
    droneAutoLandActive = true;
    return;
  }

  droneAutoLandActive = false;
  float bearingHome = gpsBearingDeg(gpsRuntime.latitudeE7, gpsRuntime.longitudeE7, droneHomeLatitudeE7, droneHomeLongitudeE7);
  float heading = getNavigationHeadingDeg(droneTelemetryYawDeg);
  float yawError = wrapSignedDeg(bearingHome - heading);
  float alignment = 1.0f - clampValue<float>((float)(fabsf(yawError) / 90.0f), 0.0f, 0.85f);

  droneYawPct = constrain((int)roundf(yawError * 0.65f), -35, 35);
  dronePitchPct = constrain((int)roundf(22.0f * alignment), 4, 22);
  droneRollPct = 0;
  droneStrafePct = 0;
  droneForwardPct = dronePitchPct;
  droneThrottlePct = 35;
}

static void updateVehicleSafeReturn() {
  if (!vehicleSafeReturnEnabled || robotControlMode != ROBOT_MODE_VEHICLE || !rt.active) {
    vehicleSafeReturnActive = false;
    return;
  }

  float batteryPct = currentBatteryPercent();
  bool linkLossReturn = runtimeStreamFailsafeLatched;
  if (!vehicleSafeReturnActive && (batteryPct <= vehicleSafeReturnBatteryThresholdPct || linkLossReturn)) vehicleSafeReturnActive = true;
  if (vehicleSafeReturnActive && !linkLossReturn && batteryPct >= vehicleSafeReturnBatteryThresholdPct + SAFE_RETURN_BATTERY_HYSTERESIS_PCT) vehicleSafeReturnActive = false;
  if (!vehicleSafeReturnActive) return;

  if (!vehicleHomeValid || !gpsFixFresh()) {
    zeroVehicleMotionCommand();
    return;
  }

  float distanceM = gpsDistanceMeters(gpsRuntime.latitudeE7, gpsRuntime.longitudeE7, vehicleHomeLatitudeE7, vehicleHomeLongitudeE7);
  if (distanceM <= SAFE_RETURN_HOME_REACHED_M) {
    zeroVehicleMotionCommand();
    return;
  }

  updateVehicleInclinePowerRuntime();
  float bearingHome = gpsBearingDeg(gpsRuntime.latitudeE7, gpsRuntime.longitudeE7, vehicleHomeLatitudeE7, vehicleHomeLongitudeE7);
  float heading = getNavigationHeadingDeg(vehicleTelemetryYawDeg);
  float yawError = wrapSignedDeg(bearingHome - heading);
  int basePower = (int)roundf(vehicleInclinePowerEnabled ? (float)vehicleInclineAutoPowerPct : 45.0f);
  basePower = constrain(basePower, 30, 85);

  if (fabsf(yawError) > 28.0f) {
    int turnPower = constrain((int)roundf(30.0f + min(35.0f, fabsf(yawError) * 0.18f)), 30, 65);
    if (yawError > 0.0f) {
      vehicleLeftTrackPct = turnPower;
      vehicleRightTrackPct = -turnPower;
    } else {
      vehicleLeftTrackPct = -turnPower;
      vehicleRightTrackPct = turnPower;
    }
  } else {
    int steer = constrain((int)roundf(yawError * 0.75f), -30, 30);
    vehicleLeftTrackPct = constrain(basePower + steer, -100, 100);
    vehicleRightTrackPct = constrain(basePower - steer, -100, 100);
  }
}

void updateSafetyAutonomy() {
  sanitizeSafetyRuntimeParams();
  if (safeHomeSessionActive && rt.active) {
    // Capture each robot HOME from the first fresh GPS fix of the connection,
    // independent of takeoff/movement/RTH enable state. This allows mode
    // changes without redefining the session origin.
    updateDroneSafeReturnHomeCapture();
    updateVehicleSafeReturnHomeCapture();
    updateDroneEmergencyHomePersistence();
  }

  if (robotControlMode == ROBOT_MODE_DRONE) {
    updateDroneSafeReturn();
  } else if (robotControlMode == ROBOT_MODE_VEHICLE) {
    updateVehicleInclinePowerRuntime();
    updateVehicleSafeReturn();
  } else {
    droneSafeReturnActive = false;
    vehicleSafeReturnActive = false;
  }
}

#endif
