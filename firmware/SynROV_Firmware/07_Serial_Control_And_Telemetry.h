// =====================================================================
// SynROV Firmware - Serial control and telemetry
// ---------------------------------------------------------------------
// Purpose:
//   Single-serial protocol: ASCII during boot/config, HEX-framed during
//   runtime. All control and telemetry share Serial0.
//
// Protocol modes
// --------------
//   BOOT / CFG  (serialHexMode == false)
//     All I/O is plain ASCII for setup and configuration.
//     The Serial Monitor and the desktop config panel work normally.
//
//   RUNTIME     (serialHexMode == true, set on READY! / enterOperationMode)
//     TX telemetry  ->  HEX-framed lines on Serial0
//     RX commands   ->  HEX-framed lines on Serial0
//
//   HEX frame format (one line, terminated with \n):
//     A55A TT LLLL [payload bytes as HEX pairs] CC \n
//     |     |  |                                 |
//     |     |  |                                 +-- XOR checksum of payload bytes (2 hex chars)
//     |     |  +-- payload length in bytes, 4 hex chars big-endian
//     |     +---- frame type, 2 hex chars (current type table)
//     +---------- sync magic, always "A55A"
//
//   Each binary byte is encoded as exactly 2 uppercase hex chars.
//   The frame has no embedded spaces or separators, only the trailing \n.
//   Example (telemetry frame type 0x20, 4-byte payload 01 02 03 04, CS=04):
//     A55A200004010203040004\n   <- note: CS is XOR of payload bytes
//
// Receiving HEX frames from the host:
//   The host sends frames in the same format on Serial0.
//   processSerial() detects lines that start with "A55A" and routes them
//   to parseHexFrame() instead of handleCommand().
//   Lines that do NOT start with "A55A" are still treated as plain ASCII
//   commands so CFG=1234, SAVE, EXIT, BOOTRESET etc. always work regardless of mode.
// =====================================================================


// ─────────────────────────────────────────────────────────────────────
// HEX encoding helpers
// ─────────────────────────────────────────────────────────────────────

// Converts a nibble (0-15) to its uppercase hex character.
static inline char nibbleToHex(uint8_t n) {
  return (char)(n < 10 ? '0' + n : 'A' + n - 10);
}

// Converts an uppercase hex character to its nibble value (0-15).
// Returns 0xFF on invalid input.
static inline uint8_t hexToNibble(char c) {
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
  return 0xFF;
}

// Writes a byte as 2 uppercase hex chars into buf[pos] and buf[pos+1].
static inline void writeHexByte(char* buf, uint16_t& pos, uint8_t b) {
  buf[pos++] = nibbleToHex((b >> 4) & 0x0F);
  buf[pos++] = nibbleToHex(b & 0x0F);
}

// Reads 2 hex chars from src[pos] into a byte. Returns false if invalid.
static inline bool readHexByte(const char* src, uint16_t pos, uint8_t& out) {
  uint8_t hi = hexToNibble(src[pos]);
  uint8_t lo = hexToNibble(src[pos + 1]);
  if (hi == 0xFF || lo == 0xFF) return false;
  out = (uint8_t)((hi << 4) | lo);
  return true;
}


// Tracks the sequence number of the latest runtime stream frame.
static void noteRuntimeStreamRxSequence(uint32_t seq);

// Reads a signed 16-bit little-endian value from a byte buffer.
static inline int16_t readI16LE(const uint8_t* src, uint16_t pos) {
  return (int16_t)((uint16_t)src[pos] | ((uint16_t)src[pos + 1] << 8));
}


// ─────────────────────────────────────────────────────────────────────
// HEX frame TX  (runtime telemetry output)
// ─────────────────────────────────────────────────────────────────────

// Sends one HEX-framed line on Serial0.
// The frame is:  A55A TT LLLL [payload HEX] CC \n
// Maximum payload that fits in a 256-char line: (256 - 12) / 2 = 122 bytes.
// All existing telemetry payloads are under 112 bytes so we are safe.
void sendHexFrame(uint8_t type, const uint8_t* payload, uint16_t len) {
  // Build into a local char buffer to emit in one write burst.
  // Max frame chars: 4 (magic) + 2 (type) + 4 (len) + len*2 (payload) + 2 (cs) + 1 (\n) + 1 (\0)
  // We cap len at 120 bytes (240 payload chars + overhead = 252 chars, safe).
  if (len > 120) len = 120;

  const uint16_t bufSize = 4 + 2 + 4 + (len * 2) + 2 + 2; // +2 for \n and \0
  char buf[256]; // generous static buffer, stack-allocated

  uint16_t pos = 0;

  // Magic
  buf[pos++] = 'A'; buf[pos++] = '5'; buf[pos++] = '5'; buf[pos++] = 'A';

  // Frame type
  writeHexByte(buf, pos, type);

  // Length (big-endian, 2 bytes → 4 hex chars)
  writeHexByte(buf, pos, (uint8_t)((len >> 8) & 0xFF));
  writeHexByte(buf, pos, (uint8_t)(len & 0xFF));

  // Payload + running checksum
  uint8_t cs = 0;
  for (uint16_t i = 0; i < len; i++) {
    cs ^= payload[i];
    writeHexByte(buf, pos, payload[i]);
  }

  // Checksum
  writeHexByte(buf, pos, cs);

  // Terminator
  buf[pos++] = '\n';
  buf[pos] = '\0';

  Serial.write((const uint8_t*)buf, pos);
}


// ─────────────────────────────────────────────────────────────────────
// HEX frame RX  (runtime command input)
// ─────────────────────────────────────────────────────────────────────

// Parses and dispatches a received HEX-framed line from Serial0.
// The line (null-terminated, without the trailing \n) must start with "A55A".
// Returns true if the frame was well-formed; false on parse / checksum error.
static bool parseHexFrame(const char* line) {
  // Minimum length: 4 (magic) + 2 (type) + 4 (len) + 2 (cs) = 12 chars
  uint16_t lineLen = (uint16_t)strlen(line);
  if (lineLen < 12) return false;

  // Verify magic (already checked by caller but double-check)
  if (line[0] != 'A' || line[1] != '5' || line[2] != '5' || line[3] != 'A') return false;

  // Type
  uint8_t type = 0;
  if (!readHexByte(line, 4, type)) return false;

  // Length (big-endian)
  uint8_t lenHi = 0, lenLo = 0;
  if (!readHexByte(line, 6, lenHi)) return false;
  if (!readHexByte(line, 8, lenLo)) return false;
  uint16_t payloadLen = ((uint16_t)lenHi << 8) | lenLo;

  // Expected total chars: 4+2+4 + payloadLen*2 + 2 = 12 + payloadLen*2
  if ((uint16_t)(12 + payloadLen * 2) > lineLen) return false;

  // Decode payload into the runtime frame buffer.
  if (payloadLen > sizeof(runtimeFrameBuf)) return false;

  uint8_t cs = 0;
  for (uint16_t i = 0; i < payloadLen; i++) {
    uint8_t b = 0;
    if (!readHexByte(line, (uint16_t)(10 + i * 2), b)) return false;
    cs ^= b;
    runtimeFrameBuf[i] = b;
  }

  // Verify checksum
  uint8_t rxCs = 0;
  if (!readHexByte(line, (uint16_t)(10 + payloadLen * 2), rxCs)) return false;
  if (rxCs != cs) return false;

  // Dispatch using the runtime frame type table
  handleSerial0HexFrame(type, runtimeFrameBuf, payloadLen);
  return true;
}


// ─────────────────────────────────────────────────────────────────────
// HEX frame dispatcher
// ─────────────────────────────────────────────────────────────────────

// Handles one decoded HEX frame received on Serial0 during runtime.
// Type codes follow the current host protocol so the
// host-side driver only needs to swap the transport layer.
void handleSerial0HexFrame(uint8_t type, const uint8_t* payload, uint16_t len) {
  (void)payload;
  (void)len;

  // 0x00 — logical boot reset / reopen boot screen (Bluetooth-friendly pre-handshake)
  if (type == 0x00) {
    restartBootHandshakeScreen();
    return;
  }

  // 0x01 — handshake / enter operation
  if (type == 0x01) {
    controlOwner = CONTROL_OWNER_PRIMARY_ASCII;
    if (!rt.active) enterOperationMode();
    // Acknowledge with a HEX boot snapshot
    static const uint8_t ack[] = { 'O', 'K' };
    sendHexFrame(0x81, ack, sizeof(ack));
    sendMinimalHexRuntimeSnapshot();
    sendTelemetryHex();
    sendGpsTelemetryHex(true);
    return;
  }

  // All remaining types require the runtime to be active
  if (!rt.active) return;

  // 0x10 — set servo target  [ch(1) raw_angle_x100(2 LE)]
  if (type == 0x10 && len >= 3) {
    uint8_t ch = payload[0];
    uint16_t raw = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
    float value = ((float)raw) * 0.01f;
    if (ch <= LAST_SERVO_CH) applyRuntimeServoTarget(ch, roundf(value));
    return;
  }

  // 0x11 — set PWM percent  [ch(1) pct(1)]
  if (type == 0x11 && len >= 2) {
    applyRuntimePwmValue(payload[0], payload[1]);
    return;
  }

  // 0x12 — wrist gimbal toggle  [enabled(1)]
  if (type == 0x12 && len >= 1) {
    wristGimbalEnabled = (payload[0] != 0);
    wristGimbalRefValid = false;
    return;
  }

  // 0x13 — neutral state  [ch(1) on(1)]
  if (type == 0x13 && len >= 2) {
    applyRuntimeNeutralState(payload[0], payload[1] != 0);
    return;
  }

  // 0x14 — switch robot mode  [mode(1)]
  if (type == 0x14 && len >= 1) {
    applyRobotControlMode((RobotControlMode)sanitizeRobotControlMode(payload[0]), false);
    telemetryFrameCounter = 0;
    sendMinimalHexRuntimeSnapshot();
    sendTelemetryHex();
    sendGpsTelemetryHex(true);
    return;
  }

  // 0x15 — vehicle track command  [seq(4 LE optional) left(i8) right(i8) pan(i8) tilt(i8) flags(1)]
  if (type == 0x15 && len >= 5) {
    uint16_t base = 0;
    if (len >= 9) {
      noteRuntimeStreamRxSequence((uint32_t)payload[0] |
                                  ((uint32_t)payload[1] << 8) |
                                  ((uint32_t)payload[2] << 16) |
                                  ((uint32_t)payload[3] << 24));
      base = 4;
  }
    if (len < (uint16_t)(base + 5)) return;
    vehicleLeftTrackPct    = constrain((int8_t)payload[base + 0], -100, 100);
    vehicleRightTrackPct   = constrain((int8_t)payload[base + 1], -100, 100);
    vehicleCameraPanDeg    = constrain((int8_t)payload[base + 2], -90, 90);
    vehicleCameraTiltDeg   = constrain((int8_t)payload[base + 3], -45, 45);
    uint8_t flags = payload[base + 4];
    vehicleLightsCommand   = (flags & 0x01u) != 0;
    vehicleLidarScanCommand = (flags & 0x02u) != 0;
    recordVehicleLinkHoldMotion();
    return;
  }

  // 0x16 — drone command  [seq(4 LE optional) throttle yaw pitch roll strafe fwd camflags camPan camTilt]
  if (type == 0x16 && len >= 7) {
    uint16_t base = 0;
    if (len >= 11) {
      noteRuntimeStreamRxSequence((uint32_t)payload[0] |
                                  ((uint32_t)payload[1] << 8) |
                                  ((uint32_t)payload[2] << 16) |
                                  ((uint32_t)payload[3] << 24));
      base = 4;
  }
    if (len < (uint16_t)(base + 7)) return;
    droneThrottlePct      = constrain((int8_t)payload[base + 0], -100, 100);
    droneYawPct           = constrain((int8_t)payload[base + 1], -100, 100);
    dronePitchPct         = constrain((int8_t)payload[base + 2], -100, 100);
    droneRollPct          = constrain((int8_t)payload[base + 3], -100, 100);
    droneStrafePct        = constrain((int8_t)payload[base + 4], -100, 100);
    droneForwardPct       = constrain((int8_t)payload[base + 5], -100, 100);
    droneCameraRecordCommand = (payload[base + 6] & 0x01u) != 0;
    if (len >= (uint16_t)(base + 9)) {
      droneCameraPanDeg = constrain((int8_t)payload[base + 7], -90, 90);
      droneCameraTiltDeg = constrain((int8_t)payload[base + 8], -45, 35);
  }
    recordDroneLinkHoldMotion();
    return;
  }

  // 0x17 — pass-through ASCII command wrapped in a HEX frame  [text bytes]
  if (type == 0x17 && len > 0) {
    char cmd[RUNTIME_TEXT_BUFFER_SIZE];
    uint16_t copyLen = min((uint16_t)(RUNTIME_TEXT_BUFFER_SIZE - 1), len);
    memcpy(cmd, payload, copyLen);
    cmd[copyLen] = '\0';
    currentTextCommandSource = 1;   // treated as primary serial
    handleCommand(cmd);
    return;
  }

  // Firmware collision persistence is controlled only by COLL=0/1 in CFG mode.

  // 0x18 — manipulator pose stream  [seq(4 LE optional) 7×i16 LE joints + 4 pwm bytes]
  if (type == 0x18 && len >= 18) {
    applyRobotControlMode(ROBOT_MODE_MANIPULATOR, false);
    uint16_t base = 0;
    if (len >= 22) {
      noteRuntimeStreamRxSequence((uint32_t)payload[0] |
                                  ((uint32_t)payload[1] << 8) |
                                  ((uint32_t)payload[2] << 16) |
                                  ((uint32_t)payload[3] << 24));
      base = 4;
  }
    if (len < (uint16_t)(base + 18)) return;
    int values[7];
    for (uint8_t i = 0; i < 7; i++) {
      values[i] = (int)readI16LE(payload, (uint16_t)(base + i * 2));
  }
    // Apply the binary ARMJ packet directly. This avoids rebuilding a text
    // command and parsing it again on every frame, which keeps telemetry
    // responsive during dense manipulator streams and BASECH=1 motion.
    if (!applyManipulatorPoseRuntimeMemberValues(values)) return;
    for (uint8_t i = 0; i < 4; i++) {
      applyRuntimePwmValue((uint8_t)(FIRST_PWM_CH + i), (float)payload[base + 14 + i]);
  }
    return;
  }
}


// ─────────────────────────────────────────────────────────────────────
// HEX telemetry TX  (replaces sendTelemetry during runtime)
// ─────────────────────────────────────────────────────────────────────

// Push helpers (identical to former binary helpers, local to this TU)
static void pushI8h(uint8_t* out, uint16_t& idx, int8_t v)   { out[idx++] = (uint8_t)v; }
static void pushU16h(uint8_t* out, uint16_t& idx, uint16_t v){ out[idx++] = (uint8_t)(v & 0xFF); out[idx++] = (uint8_t)((v>>8)&0xFF); }
static void pushI16h(uint8_t* out, uint16_t& idx, int16_t v) { pushU16h(out, idx, (uint16_t)v); }
static void pushU32h(uint8_t* out, uint16_t& idx, uint32_t v){ out[idx++]=(uint8_t)(v&0xFF); out[idx++]=(uint8_t)((v>>8)&0xFF); out[idx++]=(uint8_t)((v>>16)&0xFF); out[idx++]=(uint8_t)((v>>24)&0xFF); }
static void pushI32h(uint8_t* out, uint16_t& idx, int32_t v) { pushU32h(out, idx, (uint32_t)v); }

// Sends the isolated GPS telemetry block as its own frame.
// This preserves the existing 0x20 control/telemetry layout and keeps GPS optional.
void sendGpsTelemetryHex(boolean force) {
  unsigned long now = millis();
  if (!force && (now - gpsRuntime.lastTelemetryMs) < GPS_TELEMETRY_INTERVAL_MS) return;
  gpsRuntime.lastTelemetryMs = now;

  uint8_t payload[32];
  uint16_t idx = 0;
  uint8_t flags = 0;
  if (gpsRuntime.portReady) flags |= 0x01;
  if (gpsRuntime.fixValid) flags |= 0x02;
  if (gpsFixFresh()) flags |= 0x04;
  if (gpsRuntime.sentenceValid) flags |= 0x08;

  payload[idx++] = (uint8_t)robotControlMode;
  pushU32h(payload, idx, now);
  payload[idx++] = flags;
  payload[idx++] = gpsRuntime.satellites;
  pushU16h(payload, idx, gpsRuntime.hdopX100);
  pushI32h(payload, idx, gpsRuntime.latitudeE7);
  pushI32h(payload, idx, gpsRuntime.longitudeE7);
  pushI32h(payload, idx, gpsRuntime.altitudeCm);
  pushU16h(payload, idx, gpsRuntime.speedCms);
  pushU16h(payload, idx, gpsRuntime.courseDegX100);

  sendHexFrame(0x21, payload, idx);
}

// Rate-limited GPS telemetry helper used by the runtime loop.
void maybeSendGpsTelemetryHex() {
  if (!rt.active || autoStepper.active) return;
  sendGpsTelemetryHex(false);
}

// Sends binary telemetry encoded as a HEX frame on Serial0.
// Payload layout follows the current binary telemetry stream.
void sendTelemetryHex() {
  telemetryFrameCounter++;

  uint8_t payload[120];
  uint16_t idx = 0;
  const bool sendSonarNow  = (TELEMETRY_SONAR_DIV  <= 1) || ((telemetryFrameCounter % TELEMETRY_SONAR_DIV)  == 0);
  const bool sendAnalogNow = (TELEMETRY_ANALOG_DIV <= 1) || ((telemetryFrameCounter % TELEMETRY_ANALOG_DIV) == 0);
  const bool sendImuNow    = (TELEMETRY_IMU_DIV    <= 1) || ((telemetryFrameCounter % TELEMETRY_IMU_DIV)    == 0);

  static int sonarCm = -1;
  static uint16_t analogs[6] = {0, 0, 0, 0, 0, 0};
  static int16_t ax1=-1, ay1=-1, az1=16384, gx1=0, gy1=0, gz1=0;
  static int16_t ax2=-1, ay2=-1, az2=16384, gx2=0, gy2=0, gz2=0;

  if (sendSonarNow) {
    sonarCm = readSonarDistanceCM();
    if (robotControlMode == ROBOT_MODE_DRONE) updateDroneHeightEstimateFromSonar(sonarCm, droneTelemetryPitchDeg, droneTelemetryRollDeg);
  }
  if (sendImuNow) {
    if (rt.mpu1Ready) mpu6050_1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    if (rt.mpu2Ready) mpu6050_2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  }
  if (sendAnalogNow) {
    analogs[0] = (uint16_t)analogRead(A0);
    analogs[1] = (uint16_t)analogRead(A1);
    analogs[2] = (uint16_t)analogRead(A2);
    analogs[3] = (uint16_t)analogRead(A3);
    analogs[4] = (uint16_t)analogRead(A4);
    analogs[5] = (uint16_t)analogRead(A5);
  }

  int ina1Current = rt.ina1Ready ? (int)ina219_1.getCurrent_mA() : -1;
  int ina2Current = rt.ina2Ready ? (int)ina219_2.getCurrent_mA() : -1;

  int16_t mx = compassX, my = compassY, mz = compassZ;
  float heading = rt.compassReady ? compassHeadingDeg : -1.0f;
  if (rt.compassReady && readCompassRaw(mx, my, mz)) {
    compassX = mx; compassY = my; compassZ = mz;
    compassHeadingDeg = computeCompassHeadingDeg(mx, my);
    heading = compassHeadingDeg;
  }

  // Common header
  payload[idx++] = (uint8_t)robotControlMode;
  pushU32h(payload, idx, telemetryFrameCounter);
  pushU32h(payload, idx, millis());
  uint16_t linkMs = runtimeStreamActive
    ? (uint16_t)min(65534UL, millis() - lastRuntimeStreamRxMs)
    : 0xFFFFu;
  pushU16h(payload, idx, linkMs);
  pushU32h(payload, idx, lastRuntimeStreamSeq);

  if (robotControlMode == ROBOT_MODE_MANIPULATOR) {
    uint8_t pwmPerc[4];
    getCurrentPwmPercents(pwmPerc);

    uint8_t flags = 0;
    if (stepperFeedbackInsideWindow) flags |= 0x01;
    if (wristGimbalEnabled)          flags |= 0x02;
    if (armStabilizeEnable)          flags |= 0x04;
    if (collisionRuntimeEnabled)     flags |= 0x08;
    if (collisionFlag)               flags |= 0x10;
    payload[idx++] = flags;

    pushI16h(payload, idx, (int16_t)gripIdleCurrentAvg);
    pushI16h(payload, idx, (int16_t)gripPressureDeltaMa);
    pushI16h(payload, idx, (int16_t)roundf(gripPressureEstimate * 100.0f));
    pushU16h(payload, idx, (uint16_t)roundf(getManipulatorMemberActualDeg(MANIP_MEMBER_BASE)          * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(getManipulatorMemberActualDeg(MANIP_MEMBER_UPPER)         * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(getManipulatorMemberActualDeg(MANIP_MEMBER_FORE)          * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(getManipulatorMemberActualDeg(MANIP_MEMBER_FOREARM_ROLL)  * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(getManipulatorMemberActualDeg(MANIP_MEMBER_WRIST_PITCH)   * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(getManipulatorMemberActualDeg(MANIP_MEMBER_WRIST_ROLL)    * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(getManipulatorMemberActualDeg(MANIP_MEMBER_GRIPPER)       * 100.0f));
    // Downward sonar range in centimeters. The host maps this value to drone_scan_cm and sonar_down_cm.
    pushI16h(payload, idx, (int16_t)sonarCm);
    pushI16h(payload, idx, (int16_t)ina1Current);
    pushI16h(payload, idx, (int16_t)ina2Current);
    pushU16h(payload, idx, (uint16_t)roundf(stepper.currentAngle * 100.0f));
    pushU16h(payload, idx, (uint16_t)roundf(stepper.targetAngle  * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(stepper.filteredSpeed * 100.0f));
    pushI16h(payload, idx, (int16_t)stepper.pwmValue);
    pushI16h(payload, idx, mx); pushI16h(payload, idx, my); pushI16h(payload, idx, mz);
    pushI16h(payload, idx, (int16_t)roundf(heading * 100.0f));
    payload[idx++] = pwmPerc[0]; payload[idx++] = pwmPerc[1];
    payload[idx++] = pwmPerc[2]; payload[idx++] = pwmPerc[3];
    for (uint8_t i = 0; i < 6; i++) pushU16h(payload, idx, analogs[i]);

  } else if (robotControlMode == ROBOT_MODE_VEHICLE) {
    uint8_t flags = 0;
    if (vehicleLightsCommand)       flags |= 0x01;
    if (vehicleLidarScanCommand)    flags |= 0x02;
    if (vehicleSafeReturnEnabled)   flags |= 0x04;
    if (vehicleSafeReturnActive)    flags |= 0x08;
    if (vehicleHomeValid)           flags |= 0x10;
    if (vehicleInclinePowerEnabled)       flags |= 0x20;
    if (vehicleHoldLastMotionOnLinkLoss) flags |= 0x40;
    if (vehicleLinkHoldReturnActive)     flags |= 0x80;
    payload[idx++] = flags;
    pushI8h(payload, idx, (int8_t)constrain(vehicleLeftTrackPct,  -100, 100));
    pushI8h(payload, idx, (int8_t)constrain(vehicleRightTrackPct, -100, 100));
    pushI8h(payload, idx, (int8_t)constrain(vehicleCameraPanDeg,  -90,  90));
    pushI8h(payload, idx, (int8_t)constrain(vehicleCameraTiltDeg, -45,  45));
    // Downward sonar range in centimeters. The host maps this value to drone_scan_cm and sonar_down_cm.
    pushI16h(payload, idx, (int16_t)sonarCm);
    pushI16h(payload, idx, (int16_t)ina1Current);
    pushI16h(payload, idx, (int16_t)ina2Current);
    pushI16h(payload, idx, (int16_t)roundf(vehicleTelemetryX      * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(vehicleTelemetryZ      * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(vehicleTelemetryYawDeg * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(heading                * 100.0f));

  } else { // ROBOT_MODE_DRONE
    uint8_t flags = droneCameraRecordCommand ? 0x01 : 0x00;
    if (droneSafeReturnEnabled)        flags |= 0x02;
    if (droneSafeReturnActive)         flags |= 0x04;
    if (droneHomeValid)                flags |= 0x08;
    if (droneHoldLastMotionOnLinkLoss) flags |= 0x10;
    if (droneLinkHoldReturnActive)     flags |= 0x20;
    payload[idx++] = flags;
    pushI8h(payload, idx, (int8_t)constrain(droneThrottlePct,  -100, 100));
    pushI8h(payload, idx, (int8_t)constrain(droneYawPct,       -100, 100));
    pushI8h(payload, idx, (int8_t)constrain(dronePitchPct,     -100, 100));
    pushI8h(payload, idx, (int8_t)constrain(droneRollPct,      -100, 100));
    pushI8h(payload, idx, (int8_t)constrain(droneStrafePct,    -100, 100));
    pushI8h(payload, idx, (int8_t)constrain(droneForwardPct,   -100, 100));
    // Downward sonar range in centimeters. The host maps this value to drone_scan_cm and sonar_down_cm.
    pushI16h(payload, idx, (int16_t)sonarCm);
    pushI16h(payload, idx, (int16_t)ina1Current);
    pushI16h(payload, idx, (int16_t)ina2Current);
    pushI16h(payload, idx, (int16_t)roundf(droneTelemetryAltitudeCm * 10.0f));
    pushI16h(payload, idx, (int16_t)roundf(droneTelemetryX          * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(droneTelemetryY          * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(droneTelemetryZ          * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(droneTelemetryYawDeg     * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(droneTelemetryPitchDeg   * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(droneTelemetryRollDeg    * 100.0f));
    pushI8h(payload, idx, (int8_t)constrain(droneCameraPanDeg,      -90,  90));
    pushI8h(payload, idx, (int8_t)constrain(droneCameraTiltDeg,     -45,  35));
  }

  if ((idx + 28u) <= sizeof(payload)) {
    pushI16h(payload, idx, ax1); pushI16h(payload, idx, ay1); pushI16h(payload, idx, az1);
    pushI16h(payload, idx, gx1); pushI16h(payload, idx, gy1); pushI16h(payload, idx, gz1);
    pushI16h(payload, idx, ax2); pushI16h(payload, idx, ay2); pushI16h(payload, idx, az2);
    pushI16h(payload, idx, gx2); pushI16h(payload, idx, gy2); pushI16h(payload, idx, gz2);
    uint16_t batteryRaw = analogs[2];
    pushU16h(payload, idx, batteryRaw);
    pushU16h(payload, idx, (uint16_t)roundf(batteryPercentFromAdc(batteryRaw) * 100.0f));
  }

  // Extra drone-only sonar/MPU estimator fields are appended after the base
  // payload so the working stream/snapshot layout stays compatible.
  if (robotControlMode == ROBOT_MODE_DRONE && (idx + 6u) <= sizeof(payload)) {
    pushI16h(payload, idx, (int16_t)roundf(droneSonarVerticalCm * 10.0f));
    pushI16h(payload, idx, (int16_t)roundf((droneSonarGroundReferenceValid ? droneSonarGroundReferenceCm : -1.0f) * 10.0f));
    pushI16h(payload, idx, (int16_t)roundf(droneAltitudeFromGroundCm * 10.0f));
  }

  sendHexFrame(0x20, payload, idx);
}


// ─────────────────────────────────────────────────────────────────────
// HEX boot snapshot
// ─────────────────────────────────────────────────────────────────────

// Sends a line of text wrapped in a HEX frame (type 0x30).
// Allows the existing ASCII boot-info strings to be delivered over the
// HEX transport so the host receives them in the same framed channel.
static void sendHexTextLine(const String& line) {
  uint8_t payload[RUNTIME_TEXT_BUFFER_SIZE];
  uint16_t len = (uint16_t)line.length();
  if (len >= (uint16_t)(RUNTIME_TEXT_BUFFER_SIZE - 1)) len = (uint16_t)(RUNTIME_TEXT_BUFFER_SIZE - 1);
  for (uint16_t i = 0; i < len; i++) payload[i] = (uint8_t)line.charAt(i);
  sendHexFrame(0x30, payload, len);
}

// Sends the minimal runtime identity snapshot as HEX text-line frames.
// The full configuration snapshot is intentionally not sent during automatic
// runtime handshakes because it is long and String-heavy on AVR. Keeping the
// handshake minimal prevents Serial0 stalls and heap fragmentation after the
// first telemetry frame. Full snapshots remain available through explicit
// config/status commands.
void sendMinimalHexRuntimeSnapshot() {
  sendHexTextLine(String(F("READY!")));
  sendHexTextLine(String(F("ROBOT=")) + String(getRobotControlModeName(robotControlMode)));
  sendHexTextLine(String(F("ROBOT_INDEX=")) + String((int)robotControlMode));
  sendHexTextLine(String(F("gps_serial1=")) + String(gpsRuntime.portReady ? 1 : 0));
  sendHexTextLine(String(F("gps_frame=0x21")));
}

// Sends the full boot/config snapshot as HEX text-line frames (type 0x30).
// Content follows the current boot snapshot stream contract.
void sendHexBootSnapshot() {
  syncProcessingRobotProfiles();
  sendHexTextLine(String(F("READY!")));
  sendHexTextLine(String(F("ROBOT=")) + String(getRobotControlModeName(robotControlMode)));
  sendHexTextLine(String(F("ROBOT_INDEX=")) + String((int)robotControlMode));
  sendHexTextLine(String(F("gps_serial1=")) + String(gpsRuntime.portReady ? 1 : 0));
  sendHexTextLine(String(F("gps_baud=")) + String(GPS_SERIAL_BAUD_RATE));
  sendHexTextLine(String(F("gps_frame=0x21")));

  if (robotControlMode == ROBOT_MODE_VEHICLE) {
    sendHexTextLine(String(F("[VEHICLE STATUS]")));
    sendHexTextLine(String(F("geometry_units=cm")));
    sendHexTextLine(String(F("vehicle_body_length="))           + String(vehicleProfile.bodyLength, 2));
    sendHexTextLine(String(F("vehicle_body_width="))            + String(vehicleProfile.bodyWidth, 2));
    sendHexTextLine(String(F("vehicle_body_height="))           + String(vehicleProfile.bodyHeight, 2));
    sendHexTextLine(String(F("vehicle_track_gauge="))           + String(vehicleProfile.trackGauge, 2));
    sendHexTextLine(String(F("vehicle_track_run="))             + String(vehicleProfile.trackRun, 2));
    sendHexTextLine(String(F("vehicle_drive_radius="))          + String(vehicleProfile.driveRadius, 2));
    sendHexTextLine(String(F("vehicle_support_radius="))        + String(vehicleProfile.supportRadius, 2));
    sendHexTextLine(String(F("vehicle_top_roller_radius="))     + String(vehicleProfile.topRollerRadius, 2));
    sendHexTextLine(String(F("vehicle_track_width="))           + String(vehicleProfile.trackWidth, 2));
    sendHexTextLine(String(F("vehicle_track_link_length="))     + String(vehicleProfile.trackLinkLength, 2));
    sendHexTextLine(String(F("vehicle_track_link_thickness="))  + String(vehicleProfile.trackLinkThickness, 2));
    sendHexTextLine(String(F("vehicle_track_height="))          + String(vehicleProfile.trackHeight, 2));
    sendHexTextLine(String(F("vehicle_track_links="))           + String(vehicleProfile.trackLinks));
    sendHexTextLine(String(F("vehicle_track_sag="))             + String(vehicleProfile.trackSag, 2));
    sendHexTextLine(String(F("vehicle_body_center_y="))         + String(vehicleProfile.bodyCenterYOffset, 2));
    sendHexTextLine(String(F("vehicle_camera_default_pan_deg="))  + String(vehicleProfile.cameraPanDefaultDeg, 2));
    sendHexTextLine(String(F("vehicle_camera_default_tilt_deg=")) + String(vehicleProfile.cameraTiltDefaultDeg, 2));
    sendHexTextLine(String(F("vehicle_lidar_mast_y="))          + String(vehicleProfile.lidarMastYOffset, 2));
    sendHexTextLine(String(F("vehicle_lidar_mast_z="))          + String(vehicleProfile.lidarMastZOffset, 2));
    sendHexTextLine(String(F("vehicle_camera_head_y="))         + String(vehicleProfile.cameraHeadYOffset, 2));
    sendHexTextLine(String(F("vehicle_camera_head_z="))         + String(vehicleProfile.cameraHeadZOffset, 2));
    sendHexTextLine(String(F("vehicle_motor_deadband_pct="))   + String(vehicleControlProfile.motorDeadbandPct, 2));
    sendHexTextLine(String(F("vehicle_motor_pwm_min="))        + String(vehicleControlProfile.motorPwmMin));
    sendHexTextLine(String(F("vehicle_motor_pwm_max="))        + String(vehicleControlProfile.motorPwmMax));
    sendHexTextLine(String(F("vehicle_motor_slew_pct_per_update=")) + String(vehicleControlProfile.motorSlewPctPerUpdate, 2));
    sendHexTextLine(String(F("vehicle_left_motor_invert="))    + String(vehicleControlProfile.leftMotorInvert ? 1 : 0));
    sendHexTextLine(String(F("vehicle_right_motor_invert="))   + String(vehicleControlProfile.rightMotorInvert ? 1 : 0));
    sendHexTextLine(String(F("vehicle_camera_pan_min_deg="))   + String(vehicleControlProfile.cameraPanMinDeg, 2));
    sendHexTextLine(String(F("vehicle_camera_pan_max_deg="))   + String(vehicleControlProfile.cameraPanMaxDeg, 2));
    sendHexTextLine(String(F("vehicle_camera_tilt_min_deg="))  + String(vehicleControlProfile.cameraTiltMinDeg, 2));
    sendHexTextLine(String(F("vehicle_camera_tilt_max_deg="))  + String(vehicleControlProfile.cameraTiltMaxDeg, 2));
    sendHexTextLine(String(F("vehicle_lidar_sweep_min_deg="))  + String(vehicleControlProfile.lidarSweepMinDeg));
    sendHexTextLine(String(F("vehicle_lidar_sweep_max_deg="))  + String(vehicleControlProfile.lidarSweepMaxDeg));
    sendHexTextLine(String(F("vehicle_lidar_sweep_cycle_ms=")) + String(vehicleControlProfile.lidarSweepCycleMs));
    sendHexTextLine(String(F("vehicle_safe_return_enabled=")) + String(vehicleSafeReturnEnabled ? 1 : 0));
    sendHexTextLine(String(F("vehicle_link_hold_enabled=")) + String(vehicleHoldLastMotionOnLinkLoss ? 1 : 0));
    sendHexTextLine(String(F("vehicle_safe_return_battery_threshold_pct=")) + String(vehicleSafeReturnBatteryThresholdPct, 1));
    sendHexTextLine(String(F("vehicle_home_valid=")) + String(vehicleHomeValid ? 1 : 0));
    sendHexTextLine(String(F("vehicle_home_lat_e7=")) + String(vehicleHomeLatitudeE7));
    sendHexTextLine(String(F("vehicle_home_lon_e7=")) + String(vehicleHomeLongitudeE7));
    sendHexTextLine(String(F("vehicle_incline_power_enabled=")) + String(vehicleInclinePowerEnabled ? 1 : 0));
    sendHexTextLine(String(F("vehicle_incline_base_power_pct=")) + String(vehicleInclineBasePowerPct, 1));
    sendHexTextLine(String(F("vehicle_incline_max_power_pct=")) + String(vehicleInclineMaxPowerPct, 1));
    sendHexTextLine(String(F("vehicle_incline_full_scale_deg=")) + String(vehicleInclineFullScaleDeg, 1));
    sendHexTextLine(String(F("vehicle_incline_deadband_deg=")) + String(vehicleInclineDeadbandDeg, 1));
    sendHexTextLine(String(F("vehicle_incline_alpha=")) + String(vehicleInclinePowerAlpha, 3));
    sendHexTextLine(String(F("============================================================")));
    return;
  }

  if (robotControlMode == ROBOT_MODE_DRONE) {
    sendHexTextLine(String(F("[DRONE STATUS]")));
    sendHexTextLine(String(F("geometry_units=cm")));
    sendHexTextLine(String(F("drone_body_length="))       + String(droneProfile.bodyLength, 2));
    sendHexTextLine(String(F("drone_body_width="))        + String(droneProfile.bodyWidth, 2));
    sendHexTextLine(String(F("drone_body_height="))       + String(droneProfile.bodyHeight, 2));
    sendHexTextLine(String(F("drone_arm_length="))        + String(droneProfile.armLength, 2));
    sendHexTextLine(String(F("drone_arm_thickness="))     + String(droneProfile.armThickness, 2));
    sendHexTextLine(String(F("drone_motor_radius="))      + String(droneProfile.motorRadius, 2));
    sendHexTextLine(String(F("drone_motor_height="))      + String(droneProfile.motorHeight, 2));
    sendHexTextLine(String(F("drone_prop_radius="))       + String(droneProfile.propRadius, 2));
    sendHexTextLine(String(F("drone_prop_thickness="))    + String(droneProfile.propThickness, 2));
    sendHexTextLine(String(F("drone_leg_height="))        + String(droneProfile.legHeight, 2));
    sendHexTextLine(String(F("drone_leg_span="))          + String(droneProfile.legSpan, 2));
    sendHexTextLine(String(F("drone_rest_y="))            + String(droneProfile.restYOffset, 2));
    sendHexTextLine(String(F("drone_visual_yaw_offset_deg=")) + String(droneProfile.visualYawOffsetDeg, 2));
    sendHexTextLine(String(F("drone_sonar_mount=bottom")));
    sendHexTextLine(String(F("drone_sonar_x="))           + String(droneProfile.sonarXOffset, 2));
    sendHexTextLine(String(F("drone_sonar_y="))           + String(droneProfile.sonarYOffset, 2));
    sendHexTextLine(String(F("drone_sonar_z="))           + String(droneProfile.sonarZOffset, 2));
    sendHexTextLine(String(F("drone_sonar_direction=local_down")));
    sendHexTextLine(String(F("drone_sonar_primary_key=DRONE_SONAR_DOWN_CM")));
    sendHexTextLine(String(F("drone_sonar_height_source=bottom_sonar_mpu_tilt_compensated")));
    sendHexTextLine(String(F("drone_sonar_aliases=SONAR_DOWN_CM,SONAR_VERTICAL_CM,SONAR_GROUND_REF_CM,HEIGHT_CM,ALT_CM,SONAR")));
    sendHexTextLine(String(F("drone_camera_y="))          + String(droneProfile.cameraYOffset, 2));
    sendHexTextLine(String(F("drone_camera_z="))          + String(droneProfile.cameraZOffset, 2));
    sendHexTextLine(String(F("drone_lamp_y="))            + String(droneProfile.lampYOffset, 2));
    sendHexTextLine(String(F("drone_lamp_z="))            + String(droneProfile.lampZOffset, 2));
    sendHexTextLine(String(F("drone_esc_min_us="))       + String(droneControlProfile.escMinUs));
    sendHexTextLine(String(F("drone_esc_max_us="))       + String(droneControlProfile.escMaxUs));
    sendHexTextLine(String(F("drone_esc_idle_us="))      + String(droneControlProfile.escIdleUs));
    sendHexTextLine(String(F("drone_throttle_range_us="))+ String(droneControlProfile.throttleRangeUs));
    sendHexTextLine(String(F("drone_pitch_mix_us="))     + String(droneControlProfile.pitchMixUs));
    sendHexTextLine(String(F("drone_roll_mix_us="))      + String(droneControlProfile.rollMixUs));
    sendHexTextLine(String(F("drone_yaw_mix_us="))       + String(droneControlProfile.yawMixUs));
    sendHexTextLine(String(F("drone_command_deadband_pct=")) + String(droneControlProfile.commandDeadbandPct, 2));
    sendHexTextLine(String(F("drone_spool_step_us_per_update=")) + String(droneControlProfile.spoolStepUsPerUpdate));
    sendHexTextLine(String(F("drone_safe_return_enabled=")) + String(droneSafeReturnEnabled ? 1 : 0));
    sendHexTextLine(String(F("drone_link_hold_enabled=")) + String(droneHoldLastMotionOnLinkLoss ? 1 : 0));
    sendHexTextLine(String(F("drone_safe_return_battery_threshold_pct=")) + String(droneSafeReturnBatteryThresholdPct, 1));
    sendHexTextLine(String(F("drone_home_valid=")) + String(droneHomeValid ? 1 : 0));
    sendHexTextLine(String(F("drone_home_lat_e7=")) + String(droneHomeLatitudeE7));
    sendHexTextLine(String(F("drone_home_lon_e7=")) + String(droneHomeLongitudeE7));
    sendHexTextLine(String(F("============================================================")));
    return;
  }

  // ROBOT_MODE_MANIPULATOR
  sendHexTextLine(String(F("[MANIPULATOR STATUS]")));
  sendHexTextLine(String(F("geometry_units=cm")));
  sendHexTextLine(String(F("arm_base_cylinder_y="))      + String(armSceneToCm(armBaseCylinderYOffset), 3));
  sendHexTextLine(String(F("arm_base_block_y="))         + String(armSceneToCm(armBaseBlockYOffset), 3));
  sendHexTextLine(String(F("arm_gripper_y="))            + String(armSceneToCm(armGripperYOffset), 3));
  sendHexTextLine(String(F("arm_base_radius="))          + String(armSceneToCm(armBaseRadius), 3));
  sendHexTextLine(String(F("arm_base_height="))          + String(armSceneToCm(armBaseHeight), 3));
  sendHexTextLine(String(F("arm_base_block_w="))         + String(armSceneToCm(armBaseBlockW), 3));
  sendHexTextLine(String(F("arm_base_block_h="))         + String(armSceneToCm(armBaseBlockH), 3));
  sendHexTextLine(String(F("arm_base_block_d="))         + String(armSceneToCm(armBaseBlockD), 3));
  sendHexTextLine(String(F("arm_upper_w="))              + String(armSceneToCm(armUpperW), 3));
  sendHexTextLine(String(F("arm_upper_h="))              + String(armSceneToCm(armUpperH), 3));
  sendHexTextLine(String(F("arm_upper_d="))              + String(armSceneToCm(armUpperD), 3));
  sendHexTextLine(String(F("arm_fore_w="))               + String(armSceneToCm(armForeW), 3));
  sendHexTextLine(String(F("arm_fore_h="))               + String(armSceneToCm(armForeH), 3));
  sendHexTextLine(String(F("arm_fore_d="))               + String(armSceneToCm(armForeD), 3));
  sendHexTextLine(String(F("arm_wrist_w="))              + String(armSceneToCm(armWristW), 3));
  sendHexTextLine(String(F("arm_wrist_h="))              + String(armSceneToCm(armWristH), 3));
  sendHexTextLine(String(F("arm_wrist_d="))              + String(armSceneToCm(armWristD), 3));
  sendHexTextLine(String(F("arm_finger_w="))             + String(armSceneToCm(armFingerW), 3));
  sendHexTextLine(String(F("arm_finger_h="))             + String(armSceneToCm(armFingerH), 3));
  sendHexTextLine(String(F("arm_finger_d="))             + String(armSceneToCm(armFingerD), 3));
  sendHexTextLine(String(F("arm_map_base_channel="))         + String(manipProfile.baseChannel));
  sendHexTextLine(String(F("arm_map_upper_channel="))        + String(manipProfile.upperChannel));
  sendHexTextLine(String(F("arm_map_fore_channel="))         + String(manipProfile.foreChannel));
  sendHexTextLine(String(F("arm_map_forearm_roll_channel=")) + String(manipProfile.forearmRollChannel));
  sendHexTextLine(String(F("arm_map_wrist_pitch_channel="))  + String(manipProfile.wristPitchChannel));
  sendHexTextLine(String(F("arm_map_wrist_rot_channel="))    + String(manipProfile.wristRotChannel));
  sendHexTextLine(String(F("arm_map_gripper_channel="))      + String(manipProfile.gripperChannel));
  sendHexTextLine(String(F("armoff=")) +
    String(armBaseServoZeroDeg, 3)       + String(',') + String(armBaseMemberZeroDeg, 3)       + String(',') + String(armBaseServoSign, 3)       + String(',') +
    String(armUpperServoZeroDeg, 3)      + String(',') + String(armUpperMemberZeroDeg, 3)      + String(',') + String(armUpperServoSign, 3)      + String(',') +
    String(armForeServoZeroDeg, 3)       + String(',') + String(armForeMemberZeroDeg, 3)       + String(',') + String(armForeServoSign, 3)       + String(',') +
    String(armForearmRollServoZeroDeg, 3)+ String(',') + String(armForearmRollMemberZeroDeg,3) + String(',') + String(armForearmRollServoSign, 3)+ String(',') +
    String(armWristPitchServoZeroDeg, 3) + String(',') + String(armWristPitchMemberZeroDeg, 3) + String(',') + String(armWristPitchServoSign, 3) + String(',') +
    String(armWristRollServoZeroDeg, 3)  + String(',') + String(armWristRollMemberZeroDeg, 3)  + String(',') + String(armWristRollServoSign, 3)  + String(',') +
    String(armGripServoZeroDeg, 3)       + String(',') + String(armGripMemberZeroDeg, 3)       + String(',') + String(armGripServoSign, 3));
  sendHexTextLine(String(F("base_source_mode=")) + String((manipProfile.baseChannel == 0) ? F("POT") : F("CHANNEL")));
  sendHexTextLine(String(F("collision_config_enabled=")) + String(collisionRuntimeEnabled ? 1 : 0));

  String armlim = String(F("armlim="));
  for (uint8_t i = 0; i < MANIP_MEMBER_COUNT; i++) {
    if (i > 0) armlim += ',';
    armlim += String(getManipulatorMemberMinLimit(i), 1);
    armlim += ',';
    armlim += String(getManipulatorMemberMaxLimit(i), 1);
  }
  sendHexTextLine(armlim);
  sendHexTextLine(String(F("============================================================")));
}


// ─────────────────────────────────────────────────────────────────────
// ASCII telemetry used during the boot and configuration phases.
// ─────────────────────────────────────────────────────────────────────

// Sends ASCII telemetry on Serial0.
// Called only while serialHexMode == false (boot / cfg phase).
void sendTelemetry() {
  // During HEX runtime mode telemetry is sent via sendTelemetryHex().
  // If somehow called in hex mode, redirect silently.
  if (serialHexMode) {
    sendTelemetryHex();
    return;
  }

  telemetryFrameCounter++;
  syncProcessingRobotProfiles();

  if (robotControlMode == ROBOT_MODE_MANIPULATOR) {
    const bool sendImuNow     = ((TELEMETRY_IMU_DIV     <= 1) || ((telemetryFrameCounter % TELEMETRY_IMU_DIV)     == 0));
    const bool sendPowerNow   = ((TELEMETRY_POWER_DIV   <= 1) || ((telemetryFrameCounter % TELEMETRY_POWER_DIV)   == 0));
    const bool sendAnalogNow  = ((TELEMETRY_ANALOG_DIV  <= 1) || ((telemetryFrameCounter % TELEMETRY_ANALOG_DIV)  == 0));
    const bool sendCompassNow = ((TELEMETRY_COMPASS_DIV <= 1) || ((telemetryFrameCounter % TELEMETRY_COMPASS_DIV) == 0));
    const bool sendSonarNow   = ((TELEMETRY_SONAR_DIV   <= 1) || ((telemetryFrameCounter % TELEMETRY_SONAR_DIV)   == 0));

    static int16_t ax1=-1,ay1=-1,az1=-1,gx1=-1,gy1=-1,gz1=-1;
    static int16_t ax2=-1,ay2=-1,az2=-1,gx2=-1,gy2=-1,gz2=-1;
    static int ina1Current=-1, ina2Current=-1, sonarCm=-1;
    static int16_t mx=0,my=0,mz=0;
    static float heading=-1.0f;
    static int analogs[6]={0,0,0,0,0,0};
    uint8_t pwmPerc[4];
    getCurrentPwmPercents(pwmPerc);

    if (sendImuNow) {
      if (rt.mpu1Ready) mpu6050_1.getMotion6(&ax1,&ay1,&az1,&gx1,&gy1,&gz1);
      if (rt.mpu2Ready) mpu6050_2.getMotion6(&ax2,&ay2,&az2,&gx2,&gy2,&gz2);
  }
    if (sendPowerNow) {
      if (rt.ina1Ready) ina1Current = (int)ina219_1.getCurrent_mA();
      if (rt.ina2Ready) ina2Current = (int)ina219_2.getCurrent_mA();
  }
    if (sendAnalogNow) {
      analogs[0]=analogRead(A0); analogs[1]=analogRead(A1); analogs[2]=analogRead(A2);
      analogs[3]=analogRead(A3); analogs[4]=analogRead(A4); analogs[5]=analogRead(A5);
  }
    if (sendSonarNow) {
    sonarCm = readSonarDistanceCM();
    if (robotControlMode == ROBOT_MODE_DRONE) updateDroneHeightEstimateFromSonar(sonarCm, droneTelemetryPitchDeg, droneTelemetryRollDeg);
  }
    if (sendCompassNow) {
      if (rt.compassReady && readCompassRaw(mx,my,mz)) {
        compassX=mx; compassY=my; compassZ=mz;
        compassHeadingDeg = computeCompassHeadingDeg(mx,my);
        heading = compassHeadingDeg;
      } else { mx=compassX; my=compassY; mz=compassZ; heading = rt.compassReady ? compassHeadingDeg : -1.0f; }
  }

    float baseActual      = getManipulatorMemberActualServoDeg(MANIP_MEMBER_BASE);
    float upperActual     = getManipulatorMemberActualServoDeg(MANIP_MEMBER_UPPER);
    float foreActual      = getManipulatorMemberActualServoDeg(MANIP_MEMBER_FORE);
    float foreRollActual  = getManipulatorMemberActualServoDeg(MANIP_MEMBER_FOREARM_ROLL);
    float wristPitchActual= getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_PITCH);
    float wristRotActual  = getManipulatorMemberActualServoDeg(MANIP_MEMBER_WRIST_ROLL);
    float gripActual      = getManipulatorMemberActualServoDeg(MANIP_MEMBER_GRIPPER);

    Serial.print(F("#TEL|SEQ:")); Serial.print(telemetryFrameCounter);
    Serial.print(F("|STAMP:"));   Serial.print(millis());
    Serial.print(F("|PRESS:"));
    Serial.print((int)gripIdleCurrentAvg); Serial.print(',');
    Serial.print(gripPressureDeltaMa);     Serial.print(',');
    Serial.print(gripPressureEstimate, 2);

    (void)baseActual; (void)upperActual; (void)foreActual; (void)foreRollActual;
    (void)wristPitchActual; (void)wristRotActual; (void)gripActual;

    Serial.print(F("|MPU1:"));
    Serial.print(ax1); Serial.print(','); Serial.print(ay1); Serial.print(','); Serial.print(az1); Serial.print(',');
    Serial.print(gx1); Serial.print(','); Serial.print(gy1); Serial.print(','); Serial.print(gz1);
    Serial.print(F("|MPU2:"));
    Serial.print(ax2); Serial.print(','); Serial.print(ay2); Serial.print(','); Serial.print(az2); Serial.print(',');
    Serial.print(gx2); Serial.print(','); Serial.print(gy2); Serial.print(','); Serial.print(gz2);
    Serial.print(F("|GYR1:")); Serial.print(gx1); Serial.print(','); Serial.print(gy1); Serial.print(','); Serial.print(gz1);
    Serial.print(F("|GYR2:")); Serial.print(gx2); Serial.print(','); Serial.print(gy2); Serial.print(','); Serial.print(gz2);
    Serial.print(F("|SONAR:")); Serial.print(sonarCm);
    for (uint8_t i=0;i<6;i++){Serial.print(F("|AN")); Serial.print(i+1); Serial.print(F(":")); Serial.print(analogs[i]);}
    Serial.print(F("|INA1:")); Serial.print(ina1Current);
    Serial.print(F("|INA2:")); Serial.print(ina2Current);
    Serial.print(F("|MAG:"));
    Serial.print(mx); Serial.print(','); Serial.print(my); Serial.print(','); Serial.print(mz); Serial.print(',');
    Serial.print(heading,2); Serial.print(','); printI2CAddressHex(compassAddr); Serial.print(','); Serial.print((uint8_t)compassType);
    Serial.print(F("|PWM:"));
    Serial.print(pwmPerc[0]); Serial.print(','); Serial.print(pwmPerc[1]); Serial.print(',');
    Serial.print(pwmPerc[2]); Serial.print(','); Serial.print(pwmPerc[3]);
    Serial.print(F("|STEP:"));    Serial.print(stepper.currentAngle, 2);
    Serial.print(F("|TGT:"));     Serial.print(stepper.targetAngle, 2);
    Serial.print(F("|SPD:"));     Serial.print(stepper.filteredSpeed, 2);
    Serial.print(F("|PWMRAW:"));  Serial.print(stepper.pwmValue);
    Serial.print(F("|WIN:"));     Serial.print(stepperFeedbackInsideWindow ? 1 : 0);
    Serial.print(F("|GSTAB:"));   Serial.print(wristGimbalEnabled ? 1 : 0);
    Serial.print(F("|ASTAB:"));   Serial.print(armStabilizeEnable ? 1 : 0);
    Serial.print(F("|COLL_EN:")); Serial.print(collisionRuntimeEnabled ? 1 : 0);
    Serial.print(F("|COLL:"));    Serial.print(collisionFlag ? 1 : 0);
    Serial.print(F("|RXSEQ:"));   Serial.print(lastRuntimeStreamSeq);
    Serial.print(F("|LINK_MS:")); Serial.print(runtimeStreamActive ? (long)(millis()-lastRuntimeStreamRxMs) : -1L);
    Serial.print(F("|MAGCAL_ACTIVE:"));   Serial.print(compassCalRuntime.active ? 1 : 0);
    Serial.print(F("|MAGCAL_PROGRESS:")); Serial.print(compassCalRuntime.progress, 1);
    Serial.println();
    return;
  }

  // Vehicle / Drone ASCII telemetry (boot/cfg phase only)
  const bool sendImuNow     = ((TELEMETRY_IMU_DIV     <= 1) || ((telemetryFrameCounter % TELEMETRY_IMU_DIV)     == 0));
  const bool sendPowerNow   = ((TELEMETRY_POWER_DIV   <= 1) || ((telemetryFrameCounter % TELEMETRY_POWER_DIV)   == 0));
  const bool sendAnalogNow  = ((TELEMETRY_ANALOG_DIV  <= 1) || ((telemetryFrameCounter % TELEMETRY_ANALOG_DIV)  == 0));
  const bool sendCompassNow = ((TELEMETRY_COMPASS_DIV <= 1) || ((telemetryFrameCounter % TELEMETRY_COMPASS_DIV) == 0));
  const bool sendSonarNow   = ((TELEMETRY_SONAR_DIV   <= 1) || ((telemetryFrameCounter % TELEMETRY_SONAR_DIV)   == 0));

  static int16_t ax1=-1,ay1=-1,az1=-1,gx1=-1,gy1=-1,gz1=-1;
  static int16_t ax2=-1,ay2=-1,az2=-1,gx2=-1,gy2=-1,gz2=-1;
  static int ina1Current=-1, ina2Current=-1, sonarCm=-1;
  static int16_t mx=0,my=0,mz=0;
  static float heading=-1.0f;
  static int analogs[6]={0,0,0,0,0,0};
  uint8_t pwmPerc[4]; getCurrentPwmPercents(pwmPerc);

  if (sendImuNow) {
    if (rt.mpu1Ready) mpu6050_1.getMotion6(&ax1,&ay1,&az1,&gx1,&gy1,&gz1);
    if (rt.mpu2Ready) mpu6050_2.getMotion6(&ax2,&ay2,&az2,&gx2,&gy2,&gz2);
  }
  if (sendPowerNow) {
    if (rt.ina1Ready) ina1Current = (int)ina219_1.getCurrent_mA();
    if (rt.ina2Ready) ina2Current = (int)ina219_2.getCurrent_mA();
  }
  if (sendAnalogNow) {
    analogs[0]=analogRead(A0); analogs[1]=analogRead(A1); analogs[2]=analogRead(A2);
    analogs[3]=analogRead(A3); analogs[4]=analogRead(A4); analogs[5]=analogRead(A5);
  }
  if (sendSonarNow) {
    sonarCm = readSonarDistanceCM();
    if (robotControlMode == ROBOT_MODE_DRONE) updateDroneHeightEstimateFromSonar(sonarCm, droneTelemetryPitchDeg, droneTelemetryRollDeg);
  }
  if (sendCompassNow) {
    if (rt.compassReady && readCompassRaw(mx,my,mz)) {
      compassX=mx; compassY=my; compassZ=mz;
      compassHeadingDeg = computeCompassHeadingDeg(mx,my);
      heading = compassHeadingDeg;
    } else { mx=compassX; my=compassY; mz=compassZ; heading = rt.compassReady ? compassHeadingDeg : heading; }
  } else { mx=compassX; my=compassY; mz=compassZ; heading = rt.compassReady ? compassHeadingDeg : heading; }

  Serial.print(F("#TEL|SEQ:")); Serial.print(telemetryFrameCounter);
  Serial.print(F("|STAMP:"));   Serial.print(millis());

  if (robotControlMode == ROBOT_MODE_VEHICLE) {
    uint16_t batteryRaw = readBatteryAdcRaw();
    Serial.print(F("|BATTERY:"));        Serial.print(batteryRaw);
    Serial.print(F("|BAT_PCT:"));        Serial.print(batteryPercentFromAdc(batteryRaw), 1);
    Serial.print(F("|LIDAR_CM:"));         Serial.print(sonarCm);
    Serial.print(F("|HEADING_DEG:"));      Serial.print(vehicleTelemetryYawDeg, 2);
    Serial.print(F("|LIGHT:"));            Serial.print(vehicleLightsCommand ? 1 : 0);
    Serial.print(F("|LSCAN:"));            Serial.print(vehicleLidarScanCommand ? 1 : 0);
    Serial.print(F("|VEHICLE_X:"));        Serial.print(vehicleTelemetryX, 2);
    Serial.print(F("|VEHICLE_Z:"));        Serial.print(vehicleTelemetryZ, 2);
    Serial.print(F("|VEHICLE_YAW_DEG:"));  Serial.print(vehicleTelemetryYawDeg, 2);
    Serial.print(F("|VEHICLE_CAM_PAN_DEG:")); Serial.print(vehicleCameraPanDeg);
    Serial.print(F("|VEHICLE_CAM_TILT_DEG:")); Serial.print(vehicleCameraTiltDeg);
    Serial.print(F("|RTH_EN:")); Serial.print(vehicleSafeReturnEnabled ? 1 : 0);
    Serial.print(F("|RTH_ACTIVE:")); Serial.print(vehicleSafeReturnActive ? 1 : 0);
    Serial.print(F("|HOME_OK:")); Serial.print(vehicleHomeValid ? 1 : 0);
    Serial.print(F("|INCL_EN:")); Serial.print(vehicleInclinePowerEnabled ? 1 : 0);
    Serial.print(F("|INCL_DEG:")); Serial.print(vehicleInclineMagnitudeDeg, 1);
    Serial.print(F("|INCL_PWR:")); Serial.print(vehicleInclineAutoPowerPct, 1);
  } else {
    uint16_t batteryRaw = readBatteryAdcRaw();
    Serial.print(F("|BATTERY:"));        Serial.print(batteryRaw);
    Serial.print(F("|BAT_PCT:"));        Serial.print(batteryPercentFromAdc(batteryRaw), 1);
    Serial.print(F("|ALT_CM:"));           Serial.print(droneTelemetryAltitudeCm, 1);
    Serial.print(F("|HEIGHT_CM:"));        Serial.print(droneTelemetryAltitudeCm, 1);
    Serial.print(F("|SONAR_VERTICAL_CM:")); Serial.print(droneSonarVerticalCm, 1);
    Serial.print(F("|SONAR_GROUND_REF_CM:")); Serial.print(droneSonarGroundReferenceValid ? droneSonarGroundReferenceCm : -1.0f, 1);
    Serial.print(F("|CAM:"));              Serial.print(droneCameraRecordCommand ? 1 : 0);
    Serial.print(F("|ATT_YAW_DEG:"));      Serial.print(droneTelemetryYawDeg, 2);
    Serial.print(F("|DRONE_SCAN_CM:"));    Serial.print(sonarCm);
    Serial.print(F("|DRONE_SONAR_DOWN_CM:")); Serial.print(sonarCm);
    Serial.print(F("|SONAR_DOWN_CM:"));    Serial.print(sonarCm);
    Serial.print(F("|DRONE_X:"));          Serial.print(droneTelemetryX, 2);
    Serial.print(F("|DRONE_Y:"));          Serial.print(droneTelemetryY, 2);
    Serial.print(F("|DRONE_Z:"));          Serial.print(droneTelemetryZ, 2);
    Serial.print(F("|DRONE_YAW_DEG:"));    Serial.print(droneTelemetryYawDeg, 2);
    Serial.print(F("|DRONE_PITCH_DEG:"));  Serial.print(droneTelemetryPitchDeg, 2);
    Serial.print(F("|DRONE_ROLL_DEG:"));   Serial.print(droneTelemetryRollDeg, 2);
    Serial.print(F("|DRONE_CAM_PAN_DEG:")); Serial.print(droneCameraPanDeg);
    Serial.print(F("|DRONE_CAM_TILT_DEG:")); Serial.print(droneCameraTiltDeg);
    Serial.print(F("|RTH_EN:")); Serial.print(droneSafeReturnEnabled ? 1 : 0);
    Serial.print(F("|RTH_ACTIVE:")); Serial.print(droneSafeReturnActive ? 1 : 0);
    Serial.print(F("|HOME_OK:")); Serial.print(droneHomeValid ? 1 : 0);
  }
  Serial.print(F("|MPU1:"));
  Serial.print(ax1); Serial.print(','); Serial.print(ay1); Serial.print(','); Serial.print(az1); Serial.print(',');
  Serial.print(gx1); Serial.print(','); Serial.print(gy1); Serial.print(','); Serial.print(gz1);
  Serial.print(F("|MPU2:"));
  Serial.print(ax2); Serial.print(','); Serial.print(ay2); Serial.print(','); Serial.print(az2); Serial.print(',');
  Serial.print(gx2); Serial.print(','); Serial.print(gy2); Serial.print(','); Serial.print(gz2);
  Serial.print(F("|GYR1:")); Serial.print(gx1); Serial.print(','); Serial.print(gy1); Serial.print(','); Serial.print(gz1);
  Serial.print(F("|GYR2:")); Serial.print(gx2); Serial.print(','); Serial.print(gy2); Serial.print(','); Serial.print(gz2);
  Serial.print(F("|SONAR:")); Serial.print(sonarCm);
  for (uint8_t i=0;i<6;i++){Serial.print(F("|AN")); Serial.print(i+1); Serial.print(F(":")); Serial.print(analogs[i]);}
  Serial.print(F("|INA1:")); Serial.print(ina1Current);
  Serial.print(F("|INA2:")); Serial.print(ina2Current);
  Serial.print(F("|MAG:"));
  Serial.print(mx); Serial.print(','); Serial.print(my); Serial.print(','); Serial.print(mz); Serial.print(',');
  Serial.print(heading,2); Serial.print(','); printI2CAddressHex(compassAddr); Serial.print(','); Serial.print((uint8_t)compassType);
  Serial.print(F("|PWM:"));
  Serial.print(pwmPerc[0]); Serial.print(','); Serial.print(pwmPerc[1]); Serial.print(',');
  Serial.print(pwmPerc[2]); Serial.print(','); Serial.print(pwmPerc[3]);
  Serial.print(F("|STEP:"));    Serial.print(stepper.currentAngle, 2);
  Serial.print(F("|TGT:"));     Serial.print(stepper.targetAngle, 2);
  Serial.print(F("|SPD:"));     Serial.print(stepper.filteredSpeed, 2);
  Serial.print(F("|PWMRAW:"));  Serial.print(stepper.pwmValue);
  Serial.print(F("|WIN:"));     Serial.print(stepperFeedbackInsideWindow ? 1 : 0);
  Serial.print(F("|GSTAB:"));   Serial.print(wristGimbalEnabled ? 1 : 0);
  Serial.print(F("|ASTAB:"));   Serial.print(armStabilizeEnable ? 1 : 0);
  Serial.print(F("|COLL_EN:")); Serial.print(collisionRuntimeEnabled ? 1 : 0);
  Serial.print(F("|COLL:"));    Serial.print(collisionFlag ? 1 : 0);
  Serial.print(F("|RXSEQ:"));   Serial.print(lastRuntimeStreamSeq);
  Serial.print(F("|LINK_MS:")); Serial.print(runtimeStreamActive ? (long)(millis()-lastRuntimeStreamRxMs) : -1L);
  Serial.print(F("|MAGCAL_ACTIVE:"));   Serial.print(compassCalRuntime.active ? 1 : 0);
  Serial.print(F("|MAGCAL_PROGRESS:")); Serial.print(compassCalRuntime.progress, 1);
  Serial.println();
}


// ─────────────────────────────────────────────────────────────────────
// Runtime stream failsafe
// ─────────────────────────────────────────────────────────────────────

// Updates the monotonic RX sequence from incoming HEX command frames.
static void noteRuntimeStreamRxSequence(uint32_t seq) {
  if (seq == 0u) return;
  lastRuntimeStreamSeq = seq;
  lastRuntimeStreamRxMs = millis();
  runtimeStreamActive = true;
  runtimeStreamFailsafeLatched = false;
  clearRuntimeLinkHoldReturnState();
}


// ─────────────────────────────────────────────────────────────────────
// Serial input processing  (single Serial0 path)
// ─────────────────────────────────────────────────────────────────────

// Utility: flush primary ASCII input.
static void flushPrimaryAsciiInput() {
  while (Serial.available() > 0) Serial.read();
  serialIndex = 0;
}

// Processes one complete line received on Serial0.
static void dispatchLine(char* line) {
  if (line == NULL || line[0] == '\0') return;

  if (line[0] == 'A' && line[1] == '5' && line[2] == '5' && line[3] == 'A') {
    // HEX-framed command from host. A55A frames are accepted during the
    // pre-runtime handshake as well as after READY?, matching the known-good
    // SynROV serial protocol.
    parseHexFrame(line);
  } else {
    // Plain ASCII command (always accepted for CFG=1234, READY?, EXIT, etc.)
    currentTextCommandSource = 1;
    handleCommand(line);
  }
}

// Reads bytes from Serial0 and dispatches complete lines.
void processSerial() {
  if (millis() < bootCommandIgnoreUntilMs) {
    while (Serial.available() > 0) Serial.read();
    serialIndex = 0;
    currentTextCommandSource = 0;
    return;
  }

  uint16_t bytesProcessed = 0;
  uint8_t linesDispatched = 0;
  while (Serial.available() > 0 && bytesProcessed < SERIAL_RX_BYTE_BUDGET && linesDispatched < SERIAL_RX_LINE_BUDGET) {
    char c = (char)Serial.read();
    bytesProcessed++;

    if (c == '\r') continue;   // ignore CR

    if (c == '\n') {
      // Terminate and dispatch
      serialBuffer[serialIndex] = '\0';
      if (serialIndex > 0) {
        dispatchLine(serialBuffer);
        linesDispatched++;
      }
      serialIndex = 0;
      continue;
  }

    if (serialIndex < (uint8_t)(SERIAL_TEXT_BUFFER_SIZE - 1)) {
      serialBuffer[serialIndex++] = c;
  }
    // If the buffer is full and no newline yet, keep overwriting the last slot
    // so at least the terminator will fit.
  }
}


// ─────────────────────────────────────────────────────────────────────
// Static telemetry config packet  (ASCII, boot/cfg phase only)
// ─────────────────────────────────────────────────────────────────────

// Forward-declared in FirmwareDeclarations.h; defined here.
// During runtime this becomes a HEX text-line frame via sendHexBootSnapshot().
void sendStaticTelemetryConfigPacket() {
  // In hex mode the full config snapshot is sent as part of sendHexBootSnapshot().
  if (serialHexMode) {
    sendHexBootSnapshot();
    return;
  }
  // ASCII boot/config path used before runtime HEX frames are active.
  syncProcessingRobotProfiles();
  Serial.println(F("READY!"));
  printRobotControlModeStatus();
  printRobotSpecificConfigSummary();
}


// ─────────────────────────────────────────────────────────────────────
// Stepper diagnostics are kept on Serial0 ASCII only.
// ─────────────────────────────────────────────────────────────────────

// Prints stepper diagnostic panel.
void printStepperDiagnosticPanel() {
  int rawAdc = analogRead(STEPPER_FEEDBACK_PIN);
  bool pwm46Ok = isStepperPwmTimerHealthy();
  Serial.println(F("[STEPPER DIAGNOSTIC]"));
  Serial.print(F("ROBOT="));    Serial.print((int)robotControlMode); Serial.print(F(" [")); Serial.print(getRobotControlModeName(robotControlMode)); Serial.println(F("]"));
  Serial.print(F("CH0MODE="));  Serial.print((int)ch0DriveMode);     Serial.print(F(" [")); Serial.print(getCh0DriveModeName(ch0DriveMode));         Serial.println(F("]"));
  Serial.print(F("STEPMODE=")); Serial.print((int)stepperDriveMode); Serial.print(F(" [")); Serial.print(getStepperDriveModeName(stepperDriveMode));  Serial.println(F("]"));
  Serial.print(F("ACTIVE="));   Serial.println(rt.active ? 1 : 0);
  Serial.print(F("ARMED="));    Serial.println(rt.stepperArmed ? 1 : 0);
  Serial.print(F("RAWADC="));   Serial.println(rawAdc);
  Serial.print(F("ZEROADC="));  Serial.println(stepperCalibrationZeroADC);
  Serial.print(F("SPANRAW="));  Serial.println(stepperOneTurnSpanRaw);
  Serial.print(F("STEPWIN="));  Serial.println(stepperFeedbackInsideWindow ? 1 : 0);
  Serial.print(F("CURRENT="));  Serial.println(stepper.currentAngle, 3);
  Serial.print(F("TARGET="));   Serial.println(stepper.targetAngle, 3);
  Serial.print(F("ERROR="));    Serial.println(stepper.targetAngle - stepper.currentAngle, 3);
  Serial.print(F("PWMRAW="));   Serial.println(stepper.pwmValue);
  Serial.print(F("PWM46_OK="));    Serial.println(pwm46Ok ? 1 : 0);
  Serial.print(F("PWM46_TOP="));   Serial.println(ICR5);
  Serial.print(F("PWM46_DUTY="));  Serial.println(OCR5A);
  Serial.print(F("PWM46_PCT="));   Serial.println(ICR5 ? (100.0f * (float)OCR5A / (float)ICR5) : 0.0f, 1);
  Serial.print(F("OFFSET0="));  Serial.println(servos.offset[0]);
  Serial.print(F("SPAN0="));    Serial.println(getServoSpan(0), 3);
  Serial.print(F("DIR0="));     Serial.println(servos.direction[0]);
  if (robotControlMode != ROBOT_MODE_MANIPULATOR) {
    Serial.println(F("WARNING: STEP PINS 42..45 ARE SHARED WITH VEHICLE/DRONE OUTPUTS"));
  }
  if (ch0DriveMode != CH0_MODE_STEPPER) {
    Serial.println(F("WARNING: CH0 IS CURRENTLY CONFIGURED AS DC MOTOR"));
  }
  Serial.println();
  Serial.println(F("  STEPDIAG                     -> print this diagnostic block"));
  Serial.println(F("  STEPTEST=F,200,80,2200      -> direct forward test"));
  Serial.println(F("  STEPTEST=R,200,80,2200      -> direct reverse test"));
  Serial.println(F("  STEPTEST=F,<steps>,<pct>,<delayus>"));
  Serial.println();
}

// Runs a direct stepper test.
void runDirectStepperTest(bool forward, uint16_t steps, uint16_t pwmPercent, unsigned long delayUs) {
  if (robotControlMode != ROBOT_MODE_MANIPULATOR) {
    Serial.println(F("DENIED: STEPTEST REQUIRES ROBOT=MANIPULATOR"));
    return;
  }
  if (ch0DriveMode != CH0_MODE_STEPPER) {
    Serial.println(F("DENIED: STEPTEST REQUIRES CH0MODE=STEPPER"));
    return;
  }

  pwmPercent = (uint16_t)clampValue((int)pwmPercent, 0, 100);
  if (steps < 1)    steps = 1;
  if (delayUs < 400) delayUs = 400;

  rt.stepperArmed = false;
  stepperMotor.setSpeed(0.0f);
  stepperMotor.disableOutputs();
  releaseStepperCoils();

  directStepperTest.active        = true;
  directStepperTest.forward       = forward;
  directStepperTest.totalSteps    = steps;
  directStepperTest.stepsRemaining = steps;
  directStepperTest.pwmRaw        = (uint16_t)map((long)pwmPercent, 0L, 100L, 0L, (long)ICR5);
  directStepperTest.stepDelayUs   = delayUs;
  directStepperTest.lastStepUs    = 0;

  stepperSetDriverPWM(directStepperTest.pwmRaw);
  applyStepOutput(stepper.seq, directStepperTest.pwmRaw);

  Serial.print(F("STEPTEST START DIR="));
  Serial.print(forward ? F("F") : F("R"));
  Serial.print(F(" STEPS="));    Serial.print(steps);
  Serial.print(F(" PWM%="));     Serial.print(pwmPercent);
  Serial.print(F(" DELAYUS="));  Serial.println(delayUs);
}

// Updates the direct stepper test tick.
void updateDirectStepperTest() {
  if (!directStepperTest.active) return;

  unsigned long nowUs = micros();
  if (directStepperTest.lastStepUs == 0) directStepperTest.lastStepUs = nowUs;

  uint8_t burstCount = 0;
  while (directStepperTest.stepsRemaining > 0 &&
         (unsigned long)(nowUs - directStepperTest.lastStepUs) >= directStepperTest.stepDelayUs &&
         burstCount < 8) {
    directStepperTest.lastStepUs += directStepperTest.stepDelayUs;
    if (directStepperTest.forward) accelStepForward();
    else                           accelStepBackward();
    applyStepOutput(stepper.seq, directStepperTest.pwmRaw);
    directStepperTest.stepsRemaining--;
    burstCount++;
  }

  unsigned long burstWindowUs = directStepperTest.stepDelayUs * 8UL;
  if ((unsigned long)(nowUs - directStepperTest.lastStepUs) > burstWindowUs) {
    directStepperTest.lastStepUs = nowUs;
  }

  if (directStepperTest.stepsRemaining == 0) {
    directStepperTest.active = false;
    stepperSetDriverPWM(0);
    releaseStepperCoils();
    readStepperAngle();
    stepper.targetAngle  = stepper.currentAngle;
    servos.target[0]     = stepper.currentAngle;
    servos.actual[0]     = stepper.currentAngle;
    Serial.println(F("STEPTEST DONE"));
    printStepperDiagnosticPanel();
  }
}

// Handles stepper diagnostic commands (STEPDIAG, STEPTEST=...).
bool handleStepperDiagnosticCommand(char* cmd) {
  if (cmd == NULL || cmd[0] == '\0') return false;

  if (strcmp(cmd, "STEPDIAG") == 0 || strcmp(cmd, "STEPSTATUS") == 0) {
    printStepperDiagnosticPanel();
    return true;
  }

  if (strncmp(cmd, "STEPTEST=", 9) == 0) {
    char buf[48];
    strncpy(buf, cmd + 9, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* tok = strtok(buf, ",");
    if (tok == NULL) { Serial.println(F("INVALID STEPTEST")); return true; }

    bool fwd = true;
    if (toupper((unsigned char)tok[0]) == 'R' ||
        toupper((unsigned char)tok[0]) == 'B' || tok[0] == '-') fwd = false;

    uint16_t steps = 200;
    tok = strtok(NULL, ",");
    if (tok != NULL) {
      long v = 0;
      if (!parseStrictLongValue(tok, v)) { Serial.println(F("INVALID STEPTEST")); return true; }
      steps = (uint16_t)max(1L, v);
  }

    uint16_t pwmPct = 80;
    tok = strtok(NULL, ",");
    if (tok != NULL) {
      long v = 0;
      if (!parseStrictLongValue(tok, v)) { Serial.println(F("INVALID STEPTEST")); return true; }
      pwmPct = (uint16_t)constrain(v, 0L, 100L);
  }

    unsigned long dUs = 2200UL;
    tok = strtok(NULL, ",");
    if (tok != NULL) {
      long v = 0;
      if (!parseStrictLongValue(tok, v)) { Serial.println(F("INVALID STEPTEST")); return true; }
      dUs = (unsigned long)max(400L, v);
  }

    if (strtok(NULL, ",") != NULL) { Serial.println(F("INVALID STEPTEST")); return true; }

    runDirectStepperTest(fwd, steps, pwmPct, dUs);
    return true;
  }

  return false;
}


// ─────────────────────────────────────────────────────────────────────
// Stepper motor PID update for the runtime stepper controller.
// ─────────────────────────────────────────────────────────────────────

// Reads sonar distance in cm.
int readSonarDistanceCM() {
  unsigned int cm = sonar.ping_cm();
  return (cm == 0) ? -1 : (int)cm;
}

// Reads vehicle/drone battery ADC raw value from AN2.
uint16_t readBatteryAdcRaw() {
  return (uint16_t)analogRead(A2);
}

// Converts battery ADC raw value to percentage.
float batteryPercentFromAdc(uint16_t raw) {
  return clampValue((raw / 1023.0f) * 100.0f, 0.0f, 100.0f);
}

// Computes the vertical component of the downward sonar beam using MPU attitude.
float computeDroneTiltCompensatedSonarCm(int sonarCm, float pitchDeg, float rollDeg) {
  if (sonarCm <= 0 || sonarCm >= DRONE_DOWNWARD_SONAR_VALID_MAX_CM) return -1.0f;
  float pitchFactor = cos(radians(pitchDeg));
  float rollFactor = cos(radians(rollDeg));
  return max(0.0f, (float)sonarCm * max(0.0f, pitchFactor * rollFactor));
}

// Updates the drone altitude estimate from the takeoff ground reference.
void updateDroneHeightEstimateFromSonar(int sonarCm, float pitchDeg, float rollDeg) {
  droneLastRawSonarCm = sonarCm;
  float verticalCm = computeDroneTiltCompensatedSonarCm(sonarCm, pitchDeg, rollDeg);
  if (verticalCm < 0.0f) return;

  droneAutoLastSonarCm = sonarCm;
  droneSonarVerticalCm = verticalCm;

  bool groundedForReference = !droneAutoTakeoffActive && !droneAutoLandActive &&
    droneTelemetryLift < 1.5f && droneTelemetryTargetLift < 1.5f &&
    abs(droneThrottlePct) <= (int)(droneControlProfile.commandDeadbandPct + 1.0f);

  if (!droneSonarGroundReferenceValid) {
    droneSonarGroundReferenceCm = groundedForReference ? verticalCm : 0.0f;
    droneSonarGroundReferenceValid = true;
  } else if (groundedForReference) {
    droneSonarGroundReferenceCm = blendFloat(droneSonarGroundReferenceCm, verticalCm, 0.05f);
  }

  droneAltitudeFromGroundCm = max(0.0f, verticalCm - max(0.0f, droneSonarGroundReferenceCm));
  droneTelemetryAltitudeCm = blendFloat(droneTelemetryAltitudeCm, droneAltitudeFromGroundCm, 0.35f);
}

// Reads cached auto-flight altitude from the sonar/MPU ground-reference estimator.
float readDroneAutoAltitudeCm() {
  unsigned long nowMs = millis();
  if ((nowMs - droneAutoLastSonarReadMs) >= 60UL || droneAutoLastSonarReadMs == 0) {
    int sonarCm = readSonarDistanceCM();
    updateDroneHeightEstimateFromSonar(sonarCm, droneTelemetryPitchDeg, droneTelemetryRollDeg);
    droneAutoLastSonarReadMs = nowMs;
  }
  return max(0.0f, max(droneAltitudeFromGroundCm, droneTelemetryAltitudeCm));
}

// Computes auto-flight throttle target for takeoff and landing.
float computeDroneAutoThrottlePct(float measuredAltCm) {
  if (droneAutoTakeoffActive) {
    float errorCm = max(0.0f, droneAutoTargetAltitudeCm - measuredAltCm);
    float throttlePct = 18.0f + errorCm * 0.32f;
    return clampValue(throttlePct, 18.0f, 65.0f);
  }
  if (droneAutoLandActive) {
    if (measuredAltCm > 80.0f) return 26.0f;
    if (measuredAltCm > 35.0f) return 22.0f;
    if (measuredAltCm > 12.0f) return 18.0f;
    if (measuredAltCm > 4.0f) return 14.0f;
    return 10.0f;
  }
  return constrain((float)droneThrottlePct, -100.0f, 100.0f);
}

// Utility: service active stepper motion.
void serviceActiveStepperMotion(unsigned long nowUs) {
  if (stepper.inHoldBand || ch0DriveMode == CH0_MODE_DC_MOTOR) return;
  stepperMotor.enableOutputs();
  stepperSetDriverPWM(stepper.pwmValue);

  float stepsPerSecond = 1000000.0f / max(1.0f, (float)stepper.stepDelayUs);
  if (!stepper.directionForward) stepsPerSecond = -stepsPerSecond;
  stepperMotor.setSpeed(stepsPerSecond);
  stepperMotor.runSpeed();
  stepper.lastStepUs = nowUs;
}

// Updates stepper motor PID.
void updateStepperMotorPID() {
  if (!rt.active) return;
  if (directStepperTest.active) return;

  readStepperAngle();

  if (!rt.stepperArmed) {
    stepper.targetAngle     = stepper.currentAngle;
    servos.target[0]        = stepper.currentAngle;
    servos.actual[0]        = stepper.currentAngle;
    stepper.integral        = 0.0f;
    stepper.filteredSpeed   = 0.0f;
    stepper.inHoldBand      = true;
    stepperMotor.setSpeed(0.0f);
    stepperMotor.disableOutputs();
    releaseStepperCoils();
    stepperSetDriverPWM(0);
    return;
  }

  unsigned long nowUs = micros();
  unsigned long nowMs = millis();
  float error    = wrapSignedDeg(stepper.targetAngle - stepper.currentAngle);
  float absError = fabsf(error);
  float degPerCount  = getStepperDegPerCount();
  float holdBand     = max(stepperStopEnter, degPerCount * 2.0f);
  float releaseBand  = max(stepperStopExit,  holdBand + max(1.0f, degPerCount * 2.0f));

  if (!stepperFeedbackInsideWindow) {
    holdBand    = max(holdBand,    1.0f);
    releaseBand = max(releaseBand, holdBand + 1.0f);
  }

  stepper.integral      = 0.0f;
  stepper.filteredSpeed = 0.0f;

  if (absError <= holdBand) {
    stepper.inHoldBand  = true;
    stepper.holdStartMs = nowMs;
    stepper.pwmValue    = holdPwm;
    if (ch0DriveMode == CH0_MODE_DC_MOTOR) {
      dcMotorStop();
    } else {
      stepperMotor.setSpeed(0.0f);
      stepperMotor.enableOutputs();
      applyStepOutput(stepper.seq, stepper.pwmValue);
  }
    return;
  }

  if (stepper.inHoldBand && absError <= releaseBand) {
    stepper.pwmValue = movePwmNear;
    if (ch0DriveMode == CH0_MODE_DC_MOTOR) {
      stepper.directionForward = (error > 0.0f);
      dcMotorDrive(stepper.directionForward, stepper.pwmValue);
    } else {
      stepperMotor.setSpeed(0.0f);
      stepperMotor.enableOutputs();
      applyStepOutput(stepper.seq, stepper.pwmValue);
  }
    return;
  }

  stepper.inHoldBand       = false;
  stepper.directionForward = (error > 0.0f);

  float errorSpan  = max(8.0f,  STEPPER_INPUT_MAX * 0.20f);
  float speedRatio = clampValue(absError / errorSpan, 0.0f, 1.0f);
  float pwmRatio   = clampValue(absError / max(12.0f, STEPPER_INPUT_MAX * 0.14f), 0.0f, 1.0f);

  float stepDelay = (float)maxStepDelayUs - speedRatio * (float)(maxStepDelayUs - minStepDelayUs);
  stepper.stepDelayUs = clampValue((unsigned long)roundf(stepDelay),
                                   (unsigned long)minStepDelayUs,
                                   (unsigned long)maxStepDelayUs);

  float pwmTarget  = (float)movePwmNear + pwmRatio * (float)(movePwmFar - movePwmNear);
  stepper.pwmValue = clampValue((uint16_t)roundf(pwmTarget), movePwmNear, movePwmFar);

  if (ch0DriveMode == CH0_MODE_DC_MOTOR) {
    dcMotorDrive(stepper.directionForward, stepper.pwmValue);
    return;
  }

  stepperMotor.enableOutputs();
  stepperSetDriverPWM(stepper.pwmValue);
  float stepsPerSecond = 1000000.0f / max(1.0f, (float)stepper.stepDelayUs);
  if (!stepper.directionForward) stepsPerSecond = -stepsPerSecond;
  stepperMotor.setSpeed(stepsPerSecond);
  stepperMotor.runSpeed();
  stepper.lastStepUs = nowUs;
}
