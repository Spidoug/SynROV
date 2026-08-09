// =====================================================================
// SynROV Firmware V1 - Serial control and telemetry
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
//     A55A2000040102030404\n     <- CS is XOR of payload bytes
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

// Builds the canonical V1 HEX wire frame for both blocking and queued TX.
// A single encoder guarantees identical magic/type/length/checksum handling.
static uint16_t buildHexFrame(char* dst, uint16_t capacity, uint8_t type, const uint8_t* payload, uint16_t len) {
  if (dst == NULL || capacity < 13) return 0;
  if (len > 0 && payload == NULL) return 0;
  if (len > 120) len = 120;

  const uint16_t required = (uint16_t)(4 + 2 + 4 + (len * 2) + 2 + 1);
  if (required >= capacity) return 0;

  uint16_t pos = 0;
  dst[pos++] = 'A';
  dst[pos++] = '5';
  dst[pos++] = '5';
  dst[pos++] = 'A';
  writeHexByte(dst, pos, type);
  writeHexByte(dst, pos, (uint8_t)((len >> 8) & 0xFF));
  writeHexByte(dst, pos, (uint8_t)(len & 0xFF));

  uint8_t checksum = 0;
  for (uint16_t i = 0; i < len; i++) {
    const uint8_t value = payload[i];
    checksum ^= value;
    writeHexByte(dst, pos, value);
  }

  writeHexByte(dst, pos, checksum);
  dst[pos++] = '\n';
  if (pos < capacity) dst[pos] = '\0';
  return pos;
}

// Sends one HEX-framed line on Serial0.
// The frame is:  A55A TT LLLL [payload HEX] CC \n
// The canonical encoder caps payloads at 120 bytes, leaving room in the
// 256-byte buffer for framing, newline and the local null terminator.
void sendHexFrame(uint8_t type, const uint8_t* payload, uint16_t len) {
  char buf[256];
  const uint16_t frameLen = buildHexFrame(buf, sizeof(buf), type, payload, len);
  if (frameLen == 0) return;

  // Telemetry is part of the control contract. Do not silently drop every
  // runtime frame when the UART TX ring is temporarily low.
  Serial.write((const uint8_t*)buf, frameLen);
}


// Non-blocking runtime telemetry TX.
// At 115200 baud a ~100-byte binary telemetry payload becomes >200 ASCII HEX
// characters. Writing that frame synchronously can occupy the AVR for many
// milliseconds once the 64-byte UART TX ring fills, which directly stretches
// the software PID period. Runtime 0x20/0x21 frames therefore use one small
// cooperative queue; control/config ACKs keep the ordinary blocking path.
static char runtimeHexTxBuffer[256];
static uint16_t runtimeHexTxLength = 0;
static uint16_t runtimeHexTxPosition = 0;
static uint32_t runtimeHexTxQueuedFrames = 0;
static uint32_t runtimeHexTxSkippedFrames = 0;

static bool runtimeHexTxBusy() {
  return runtimeHexTxPosition < runtimeHexTxLength;
}

void serviceRuntimeHexTx() {
  if (!runtimeHexTxBusy()) return;

  int writable = Serial.availableForWrite();
  if (writable <= 0) return;

  uint16_t remaining = runtimeHexTxLength - runtimeHexTxPosition;
  uint16_t chunk = min((uint16_t)writable, remaining);
  if (chunk == 0) return;

  size_t written = Serial.write(
    (const uint8_t*)&runtimeHexTxBuffer[runtimeHexTxPosition],
    chunk);
  runtimeHexTxPosition += (uint16_t)written;

  if (runtimeHexTxPosition >= runtimeHexTxLength) {
    runtimeHexTxLength = 0;
    runtimeHexTxPosition = 0;
  }
}

static bool queueRuntimeHexFrame(uint8_t type, const uint8_t* payload, uint16_t len) {
  if (runtimeHexTxBusy()) {
    runtimeHexTxSkippedFrames++;
    return false;
  }
  const uint16_t frameLen = buildHexFrame(runtimeHexTxBuffer, sizeof(runtimeHexTxBuffer), type, payload, len);
  if (frameLen == 0) {
    runtimeHexTxSkippedFrames++;
    return false;
  }

  runtimeHexTxLength = frameLen;
  runtimeHexTxPosition = 0;
  runtimeHexTxQueuedFrames++;
  serviceRuntimeHexTx();
  return true;
}

// Sends the standard HEX runtime ACK used by the Processing bridge to confirm
// that Serial0 is already in framed runtime mode.
void sendHexRuntimeAckOk() {
  static const uint8_t ack[] = { 'O', 'K' };
  sendHexFrame(0x81, ack, sizeof(ack));
}

// During the READY?/HELLO transition, keep the first runtime telemetry packet
// independent from String allocation, I2C devices, sonar and sensor reads.
// Processing marks the stream alive as soon as a valid 0x20 frame arrives,
// even if the payload only contains the robot mode byte.
static unsigned long runtimeHexHandshakeGuardUntilMs = 0;
static unsigned long runtimeHexLastHeartbeatMs = 0;
// Keep the HELLO/READY transition short so full telemetry resumes promptly.
const unsigned long RUNTIME_HEX_HANDSHAKE_GUARD_MS = 300UL;
const unsigned long RUNTIME_HEX_HEARTBEAT_INTERVAL_MS = 80UL;

void sendRuntimeTelemetryHeartbeatHex() {
  uint8_t payload[1];
  payload[0] = (uint8_t)robotControlMode;
  sendHexFrame(0x20, payload, 1);
  runtimeHexLastHeartbeatMs = millis();
}

void armRuntimeHexHandshakeGuard() {
  unsigned long now = millis();
  runtimeHexHandshakeGuardUntilMs = now + RUNTIME_HEX_HANDSHAKE_GUARD_MS;
  // Send one tiny heartbeat immediately so the host confirms HEX mode, but do
  // not suppress the full 0x20 arm packet for seconds.  Force the scheduler to
  // send a complete manipulator snapshot as soon as the short guard expires.
  sendRuntimeTelemetryHeartbeatHex();
  rt.lastSensorMs = now - SENSOR_INTERVAL_MS;
}

bool isRuntimeHexHandshakeGuardActive() {
  if (!rt.active) return false;
  return ((long)(runtimeHexHandshakeGuardUntilMs - millis()) > 0);
}

void maybeSendRuntimeHexHandshakeHeartbeat() {
  if (!isRuntimeHexHandshakeGuardActive()) return;
  unsigned long now = millis();
  if ((now - runtimeHexLastHeartbeatMs) >= RUNTIME_HEX_HEARTBEAT_INTERVAL_MS) {
    sendRuntimeTelemetryHeartbeatHex();
  }
}


// ─────────────────────────────────────────────────────────────────────
// HEX frame RX  (runtime command input)
// ─────────────────────────────────────────────────────────────────────

// Parses and dispatches a received HEX-framed line from Serial0.
// The line (null-terminated, without the trailing \n) must start with "A55A".
// Returns true if the frame was well-formed; false on parse / checksum error.
static bool parseHexFrame(char* line) {
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

  // Decode in-place into the beginning of the serial line buffer.
  // Source bytes live at offsets 10+2*i, always ahead of destination i,
  // so forward decoding cannot overwrite unread HEX characters. This removes
  // the dedicated runtimeFrameBuf from SRAM without reducing protocol capacity.
  uint8_t* payload = reinterpret_cast<uint8_t*>(line);
  uint8_t cs = 0;
  for (uint16_t i = 0; i < payloadLen; i++) {
    uint8_t b = 0;
    if (!readHexByte(line, (uint16_t)(10 + i * 2), b)) return false;
    cs ^= b;
    payload[i] = b;
  }

  // Verify checksum
  uint8_t rxCs = 0;
  if (!readHexByte(line, (uint16_t)(10 + payloadLen * 2), rxCs)) return false;
  if (rxCs != cs) return false;

  // Dispatch using the runtime frame type table.
  handleSerial0HexFrame(type, payload, payloadLen);
  return true;
}


// ─────────────────────────────────────────────────────────────────────
// HEX frame dispatcher
// ─────────────────────────────────────────────────────────────────────

// Handles one decoded HEX frame received on Serial0 during runtime.
// Type codes follow the current host protocol so the
// host-side driver only needs to swap the transport layer.
void handleSerial0HexFrame(uint8_t type, const uint8_t* payload, uint16_t len) {
  // 0x00 — logical boot reset / reopen boot screen (Bluetooth-friendly pre-handshake)
  if (type == 0x00) {
    restartBootHandshakeScreen();
    return;
  }

  // 0x01 — handshake / enter operation
  if (type == 0x01) {
    controlOwner = CONTROL_OWNER_PRIMARY_ASCII;
    if (!rt.active) enterOperationMode();
    else noteRuntimeLinkActivity(0);
    // Keep HELLO deterministic: ACK first, then a tiny 0x20 heartbeat.
    // Avoid String snapshots and sensor reads until Processing has accepted
    // the runtime stream.
    sendHexRuntimeAckOk();
    armRuntimeHexHandshakeGuard();
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
    switchRobotModeForRuntime((RobotControlMode)sanitizeRobotControlMode(payload[0]), false, true);
    return;
  }

  // 0x15 — Firmware V1 vehicle stream: [seq(4 LE) left(i8) right(i8) pan(i8) tilt(i8) flags(1)]
  if (type == 0x15 && len >= 9) {
    if (robotControlMode != ROBOT_MODE_VEHICLE) {
      switchRobotModeForRuntime(ROBOT_MODE_VEHICLE, false, true);
    }
    noteRuntimeStreamRxSequence((uint32_t)payload[0] |
                                ((uint32_t)payload[1] << 8) |
                                ((uint32_t)payload[2] << 16) |
                                ((uint32_t)payload[3] << 24));
    const uint16_t base = 4;
    vehicleLeftTrackPct  = constrain((int8_t)payload[base + 0], -100, 100);
    vehicleRightTrackPct = constrain((int8_t)payload[base + 1], -100, 100);
    vehicleCameraPanDeg = (int)roundf(clampValue((float)(int8_t)payload[base + 2],
                                                  (float)vehicleControlProfile.cameraPanMinDeg,
                                                  (float)vehicleControlProfile.cameraPanMaxDeg));
    vehicleCameraTiltDeg = (int)roundf(clampValue((float)(int8_t)payload[base + 3],
                                                   (float)vehicleControlProfile.cameraTiltMinDeg,
                                                   (float)vehicleControlProfile.cameraTiltMaxDeg));
    const uint8_t flags = payload[base + 4];
    vehicleLightsCommand = (flags & 0x01u) != 0;
    vehicleLidarScanCommand = (flags & 0x02u) != 0;
    recordVehicleLinkHoldMotion();
    return;
  }

  // 0x16 — Firmware V1 drone stream: [seq(4 LE) throttle yaw pitch roll strafe fwd flags pan tilt]
  if (type == 0x16 && len >= 13) {
    if (robotControlMode != ROBOT_MODE_DRONE) {
      switchRobotModeForRuntime(ROBOT_MODE_DRONE, false, true);
    }
    noteRuntimeStreamRxSequence((uint32_t)payload[0] |
                                ((uint32_t)payload[1] << 8) |
                                ((uint32_t)payload[2] << 16) |
                                ((uint32_t)payload[3] << 24));
    const uint16_t base = 4;
    droneThrottlePct = constrain((int8_t)payload[base + 0], -100, 100);
    droneYawPct      = constrain((int8_t)payload[base + 1], -100, 100);
    dronePitchPct    = constrain((int8_t)payload[base + 2], -100, 100);
    droneRollPct     = constrain((int8_t)payload[base + 3], -100, 100);
    droneStrafePct   = constrain((int8_t)payload[base + 4], -100, 100);
    droneForwardPct  = constrain((int8_t)payload[base + 5], -100, 100);
    droneCameraRecordCommand = (payload[base + 6] & 0x01u) != 0;
    droneCameraPanDeg = constrain((int8_t)payload[base + 7], -90, 90);
    droneCameraTiltDeg = constrain((int8_t)payload[base + 8], -45, 35);
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

  // 0x18 — manipulator pose stream
  // Protocol (seq always present):
  //   len == 18: [seq 4 LE] [7×i16 LE joints]          — joints only, PWM unchanged
  //   len == 22: [seq 4 LE] [7×i16 LE joints] [4 PWM]  — joints + CH12..15
  // The PWM bytes are only applied when explicitly included (len >= 22).
  // This decouples joint commands from PWM: moving any arm member never
  // touches CH12..15 unless the sender intentionally includes the PWM bytes.
  if (type == 0x18 && len >= 18) {
    // Do not reconfigure/release manipulator hardware on every streamed ARM frame.
    // Reconfiguring every packet can momentarily stop CH0 before the closed-loop
    // controller has a chance to keep a DC base motor driven.
    if (robotControlMode != ROBOT_MODE_MANIPULATOR) {
      switchRobotModeForRuntime(ROBOT_MODE_MANIPULATOR, false, true);
    }
    // Seq is always the first 4 bytes.
    noteRuntimeStreamRxSequence((uint32_t)payload[0] |
                                ((uint32_t)payload[1] << 8) |
                                ((uint32_t)payload[2] << 16) |
                                ((uint32_t)payload[3] << 24));
    const uint16_t base = 4;
    if (len < (uint16_t)(base + 14)) return; // need all 7 joints
    int values[7];
    for (uint8_t i = 0; i < 7; i++) {
      values[i] = (int)readI16LE(payload, (uint16_t)(base + i * 2));
    }
    // Apply the binary ARMJ packet directly. This avoids rebuilding a text
    // command and parsing it again on every frame, which keeps telemetry
    // responsive during dense manipulator streams and BASECH=1 motion.
    if (!applyManipulatorPoseRuntimeMemberValues(values)) return;
    // Apply PWM only when the sender explicitly included the 4 PWM bytes.
    if (len >= (uint16_t)(base + 18)) {
      for (uint8_t i = 0; i < 4; i++) {
        applyRuntimePwmValue((uint8_t)(FIRST_PWM_CH + i), (float)payload[base + 14 + i]);
      }
    }
    return;
  }
}


// ─────────────────────────────────────────────────────────────────────
// HEX telemetry TX
// ─────────────────────────────────────────────────────────────────────

// Payload encoding helpers.
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
  if (!force && runtimeHexTxBusy()) return;

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

  if (force) {
    sendHexFrame(0x21, payload, idx);
    gpsRuntime.lastTelemetryMs = now;
  } else if (queueRuntimeHexFrame(0x21, payload, idx)) {
    gpsRuntime.lastTelemetryMs = now;
  }
}

// Rate-limited GPS telemetry helper used by the runtime loop.
void maybeSendGpsTelemetryHex() {
  if (!rt.active || autoMotor.active) return;
  sendGpsTelemetryHex(false);
}

// Shared sensor snapshot for both HEX runtime and ASCII boot/config telemetry.
// Keeping one snapshot avoids three duplicate static caches in SRAM.
struct TelemetrySensorCache {
  int sonarCm;
  uint16_t analogs[6];
  int16_t ax1, ay1, az1, gx1, gy1, gz1;
  int16_t ax2, ay2, az2, gx2, gy2, gz2;
  int ina1Current;
  int ina2Current;
  int16_t mx, my, mz;
  float heading;
};
static TelemetrySensorCache telemetrySensorCache = {
  -1, {0,0,0,0,0,0},
  -1,-1,16384,0,0,0,
  -1,-1,16384,0,0,0,
  -1,-1, 0,0,0, -1.0f
};

// Sends binary telemetry encoded as a HEX frame on Serial0.
// Payload layout follows the current binary telemetry stream.
void sendTelemetryHex() {
  // Never block the control loop behind a previous UART frame. At 115200 baud
  // the previous frame should normally finish well before the next 33 ms slot;
  // if not, skip this snapshot and preserve actuator timing.
  if (runtimeHexTxBusy()) {
    runtimeHexTxSkippedFrames++;
    return;
  }

  TelemetrySensorCache& tc = telemetrySensorCache;
  telemetryFrameCounter++;

  uint8_t payload[120];
  uint16_t idx = 0;
  uint8_t sonarDiv   = TELEMETRY_SONAR_DIV;
  uint8_t analogDiv  = TELEMETRY_ANALOG_DIV;
  uint8_t imuDiv     = TELEMETRY_IMU_DIV;
  uint8_t powerDiv   = TELEMETRY_POWER_DIV;
  uint8_t compassDiv = TELEMETRY_COMPASS_DIV;

  // Runtime 0x20 packets remain periodic, but slow sensors do not have to be
  // sampled on every packet.  In manipulator mode the critical live data is the
  // arm/base pose; sonar/INA/compass/extra ADC are throttled to keep Serial0,
  // CH0 PID, servo refresh and collision validation responsive.
  if (robotControlMode == ROBOT_MODE_MANIPULATOR) {
    if (sonarDiv   < 8) sonarDiv   = 8;
    if (analogDiv  < 8) analogDiv  = 8;
    if (imuDiv     < 4) imuDiv     = 4;
    if (powerDiv   < 8) powerDiv   = 8;
    if (compassDiv < 8) compassDiv = 8;
  }

  const bool sendSonarNow   = (sonarDiv   <= 1) || ((telemetryFrameCounter % sonarDiv)   == 0);
  const bool sendAnalogNow  = (analogDiv  <= 1) || ((telemetryFrameCounter % analogDiv)  == 0);
  const bool sendImuNow     = (imuDiv     <= 1) || ((telemetryFrameCounter % imuDiv)     == 0);
  const bool sendPowerNow   = (powerDiv   <= 1) || ((telemetryFrameCounter % powerDiv)   == 0);
  const bool sendCompassNow = (compassDiv <= 1) || ((telemetryFrameCounter % compassDiv) == 0);

  if (sendSonarNow) {
    tc.sonarCm = readSonarDistanceCM();
    if (robotControlMode == ROBOT_MODE_DRONE) updateDroneHeightEstimateFromSonar(tc.sonarCm, droneTelemetryPitchDeg, droneTelemetryRollDeg);
  }
  if (sendImuNow) {
    if (rt.mpu1Ready) mpu6050_1.getMotion6(&tc.ax1, &tc.ay1, &tc.az1, &tc.gx1, &tc.gy1, &tc.gz1);
    if (rt.mpu2Ready) mpu6050_2.getMotion6(&tc.ax2, &tc.ay2, &tc.az2, &tc.gx2, &tc.gy2, &tc.gz2);
  }
  if (sendAnalogNow) {
    tc.analogs[0] = (uint16_t)analogRead(A0);
    tc.analogs[1] = (uint16_t)analogRead(A1);
    tc.analogs[2] = (uint16_t)analogRead(A2);
    tc.analogs[3] = (uint16_t)analogRead(A3);
    tc.analogs[4] = (uint16_t)analogRead(A4);
    tc.analogs[5] = (uint16_t)analogRead(A5);
  }
  if (sendPowerNow) {
    tc.ina1Current = rt.ina1Ready ? (int)ina219_1.getCurrent_mA() : -1;
    tc.ina2Current = rt.ina2Ready ? (int)ina219_2.getCurrent_mA() : -1;
  }
  if (sendCompassNow) {
    tc.mx = compassX;
    tc.my = compassY;
    tc.mz = compassZ;
    tc.heading = rt.compassReady ? compassHeadingDeg : -1.0f;
    if (rt.compassReady && readCompassRaw(tc.mx, tc.my, tc.mz)) {
      compassX = tc.mx; compassY = tc.my; compassZ = tc.mz;
      compassHeadingDeg = computeCompassHeadingDeg(tc.mx, tc.my);
      tc.heading = compassHeadingDeg;
    }
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
    if (motorFeedbackInsideWindow) flags |= 0x01;
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
    pushI16h(payload, idx, (int16_t)tc.sonarCm);
    pushI16h(payload, idx, (int16_t)tc.ina1Current);
    pushI16h(payload, idx, (int16_t)tc.ina2Current);
    pushU16h(payload, idx, (uint16_t)roundf(motor.currentAngle * 100.0f));
    pushU16h(payload, idx, (uint16_t)roundf(motor.targetAngle  * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(motor.filteredSpeed * 100.0f));
    pushI16h(payload, idx, (int16_t)motor.pwmValue);
    pushI16h(payload, idx, tc.mx); pushI16h(payload, idx, tc.my); pushI16h(payload, idx, tc.mz);
    pushI16h(payload, idx, (int16_t)roundf(tc.heading * 100.0f));
    payload[idx++] = pwmPerc[0]; payload[idx++] = pwmPerc[1];
    payload[idx++] = pwmPerc[2]; payload[idx++] = pwmPerc[3];
    for (uint8_t i = 0; i < 6; i++) pushU16h(payload, idx, tc.analogs[i]);

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
    pushI16h(payload, idx, (int16_t)tc.sonarCm);
    pushI16h(payload, idx, (int16_t)tc.ina1Current);
    pushI16h(payload, idx, (int16_t)tc.ina2Current);
    pushI16h(payload, idx, (int16_t)roundf(vehicleTelemetryX      * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(vehicleTelemetryZ      * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(vehicleTelemetryYawDeg * 100.0f));
    pushI16h(payload, idx, (int16_t)roundf(tc.heading                * 100.0f));

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
    pushI16h(payload, idx, (int16_t)tc.sonarCm);
    pushI16h(payload, idx, (int16_t)tc.ina1Current);
    pushI16h(payload, idx, (int16_t)tc.ina2Current);
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
    pushI16h(payload, idx, tc.ax1); pushI16h(payload, idx, tc.ay1); pushI16h(payload, idx, tc.az1);
    pushI16h(payload, idx, tc.gx1); pushI16h(payload, idx, tc.gy1); pushI16h(payload, idx, tc.gz1);
    pushI16h(payload, idx, tc.ax2); pushI16h(payload, idx, tc.ay2); pushI16h(payload, idx, tc.az2);
    pushI16h(payload, idx, tc.gx2); pushI16h(payload, idx, tc.gy2); pushI16h(payload, idx, tc.gz2);
    uint16_t batteryRaw = tc.analogs[2];
    pushU16h(payload, idx, batteryRaw);
    pushU16h(payload, idx, (uint16_t)roundf(batteryPercentFromAdc(batteryRaw) * 100.0f));
  }

  // Extra drone-only sonar/MPU estimator fields are appended after the base
  // payload so the working stream/snapshot layout stays stable.
  if (robotControlMode == ROBOT_MODE_DRONE && (idx + 6u) <= sizeof(payload)) {
    pushI16h(payload, idx, (int16_t)roundf(droneSonarVerticalCm * 10.0f));
    pushI16h(payload, idx, (int16_t)roundf((droneSonarGroundReferenceValid ? (float)droneSonarGroundReferenceCm : -1.0f) * 10.0f));
    pushI16h(payload, idx, (int16_t)roundf(droneAltitudeFromGroundCm * 10.0f));
  }

  queueRuntimeHexFrame(0x20, payload, idx);
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
  TelemetrySensorCache& tc = telemetrySensorCache;
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
    uint8_t pwmPerc[4];
    getCurrentPwmPercents(pwmPerc);

    if (sendImuNow) {
      if (rt.mpu1Ready) mpu6050_1.getMotion6(&tc.ax1,&tc.ay1,&tc.az1,&tc.gx1,&tc.gy1,&tc.gz1);
      if (rt.mpu2Ready) mpu6050_2.getMotion6(&tc.ax2,&tc.ay2,&tc.az2,&tc.gx2,&tc.gy2,&tc.gz2);
  }
    if (sendPowerNow) {
      if (rt.ina1Ready) tc.ina1Current = (int)ina219_1.getCurrent_mA();
      if (rt.ina2Ready) tc.ina2Current = (int)ina219_2.getCurrent_mA();
  }
    if (sendAnalogNow) {
      tc.analogs[0]=analogRead(A0); tc.analogs[1]=analogRead(A1); tc.analogs[2]=analogRead(A2);
      tc.analogs[3]=analogRead(A3); tc.analogs[4]=analogRead(A4); tc.analogs[5]=analogRead(A5);
  }
    if (sendSonarNow) {
    tc.sonarCm = readSonarDistanceCM();
    if (robotControlMode == ROBOT_MODE_DRONE) updateDroneHeightEstimateFromSonar(tc.sonarCm, droneTelemetryPitchDeg, droneTelemetryRollDeg);
  }
    if (sendCompassNow) {
      if (rt.compassReady && readCompassRaw(tc.mx,tc.my,tc.mz)) {
        compassX=tc.mx; compassY=tc.my; compassZ=tc.mz;
        compassHeadingDeg = computeCompassHeadingDeg(tc.mx,tc.my);
        tc.heading = compassHeadingDeg;
      } else { tc.mx=compassX; tc.my=compassY; tc.mz=compassZ; tc.heading = rt.compassReady ? compassHeadingDeg : -1.0f; }
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
    Serial.print(tc.ax1); Serial.print(','); Serial.print(tc.ay1); Serial.print(','); Serial.print(tc.az1); Serial.print(',');
    Serial.print(tc.gx1); Serial.print(','); Serial.print(tc.gy1); Serial.print(','); Serial.print(tc.gz1);
    Serial.print(F("|MPU2:"));
    Serial.print(tc.ax2); Serial.print(','); Serial.print(tc.ay2); Serial.print(','); Serial.print(tc.az2); Serial.print(',');
    Serial.print(tc.gx2); Serial.print(','); Serial.print(tc.gy2); Serial.print(','); Serial.print(tc.gz2);
    Serial.print(F("|GYR1:")); Serial.print(tc.gx1); Serial.print(','); Serial.print(tc.gy1); Serial.print(','); Serial.print(tc.gz1);
    Serial.print(F("|GYR2:")); Serial.print(tc.gx2); Serial.print(','); Serial.print(tc.gy2); Serial.print(','); Serial.print(tc.gz2);
    Serial.print(F("|SONAR:")); Serial.print(tc.sonarCm);
    for (uint8_t i=0;i<6;i++){Serial.print(F("|AN")); Serial.print(i+1); Serial.print(F(":")); Serial.print(tc.analogs[i]);}
    Serial.print(F("|INA1:")); Serial.print(tc.ina1Current);
    Serial.print(F("|INA2:")); Serial.print(tc.ina2Current);
    Serial.print(F("|MAG:"));
    Serial.print(tc.mx); Serial.print(','); Serial.print(tc.my); Serial.print(','); Serial.print(tc.mz); Serial.print(',');
    Serial.print(tc.heading,2); Serial.print(','); printI2CAddressHex(compassAddr); Serial.print(','); Serial.print((uint8_t)compassType);
    Serial.print(F("|PWM:"));
    Serial.print(pwmPerc[0]); Serial.print(','); Serial.print(pwmPerc[1]); Serial.print(',');
    Serial.print(pwmPerc[2]); Serial.print(','); Serial.print(pwmPerc[3]);
    Serial.print(F("|STEP:"));    Serial.print(motor.currentAngle, 2);
    Serial.print(F("|TGT:"));     Serial.print(motor.targetAngle, 2);
    Serial.print(F("|SPD:"));     Serial.print(motor.filteredSpeed, 2);
    Serial.print(F("|PWMRAW:"));  Serial.print(motor.pwmValue);
    Serial.print(F("|WIN:"));     Serial.print(motorFeedbackInsideWindow ? 1 : 0);
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
  uint8_t pwmPerc[4]; getCurrentPwmPercents(pwmPerc);

  if (sendImuNow) {
    if (rt.mpu1Ready) mpu6050_1.getMotion6(&tc.ax1,&tc.ay1,&tc.az1,&tc.gx1,&tc.gy1,&tc.gz1);
    if (rt.mpu2Ready) mpu6050_2.getMotion6(&tc.ax2,&tc.ay2,&tc.az2,&tc.gx2,&tc.gy2,&tc.gz2);
  }
  if (sendPowerNow) {
    if (rt.ina1Ready) tc.ina1Current = (int)ina219_1.getCurrent_mA();
    if (rt.ina2Ready) tc.ina2Current = (int)ina219_2.getCurrent_mA();
  }
  if (sendAnalogNow) {
    tc.analogs[0]=analogRead(A0); tc.analogs[1]=analogRead(A1); tc.analogs[2]=analogRead(A2);
    tc.analogs[3]=analogRead(A3); tc.analogs[4]=analogRead(A4); tc.analogs[5]=analogRead(A5);
  }
  if (sendSonarNow) {
    tc.sonarCm = readSonarDistanceCM();
    if (robotControlMode == ROBOT_MODE_DRONE) updateDroneHeightEstimateFromSonar(tc.sonarCm, droneTelemetryPitchDeg, droneTelemetryRollDeg);
  }
  if (sendCompassNow) {
    if (rt.compassReady && readCompassRaw(tc.mx,tc.my,tc.mz)) {
      compassX=tc.mx; compassY=tc.my; compassZ=tc.mz;
      compassHeadingDeg = computeCompassHeadingDeg(tc.mx,tc.my);
      tc.heading = compassHeadingDeg;
    } else { tc.mx=compassX; tc.my=compassY; tc.mz=compassZ; tc.heading = rt.compassReady ? compassHeadingDeg : tc.heading; }
  } else { tc.mx=compassX; tc.my=compassY; tc.mz=compassZ; tc.heading = rt.compassReady ? compassHeadingDeg : tc.heading; }

  Serial.print(F("#TEL|SEQ:")); Serial.print(telemetryFrameCounter);
  Serial.print(F("|STAMP:"));   Serial.print(millis());

  if (robotControlMode == ROBOT_MODE_VEHICLE) {
    uint16_t batteryRaw = readBatteryAdcRaw();
    Serial.print(F("|BATTERY:"));        Serial.print(batteryRaw);
    Serial.print(F("|BAT_PCT:"));        Serial.print(batteryPercentFromAdc(batteryRaw), 1);
    Serial.print(F("|LIDAR_CM:"));         Serial.print(tc.sonarCm);
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
    Serial.print(F("|SONAR_GROUND_REF_CM:")); Serial.print(droneSonarGroundReferenceValid ? (float)droneSonarGroundReferenceCm : -1.0f, 1);
    Serial.print(F("|CAM:"));              Serial.print(droneCameraRecordCommand ? 1 : 0);
    Serial.print(F("|ATT_YAW_DEG:"));      Serial.print(droneTelemetryYawDeg, 2);
    Serial.print(F("|DRONE_SCAN_CM:"));    Serial.print(tc.sonarCm);
    Serial.print(F("|DRONE_SONAR_DOWN_CM:")); Serial.print(tc.sonarCm);
    Serial.print(F("|SONAR_DOWN_CM:"));    Serial.print(tc.sonarCm);
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
  Serial.print(tc.ax1); Serial.print(','); Serial.print(tc.ay1); Serial.print(','); Serial.print(tc.az1); Serial.print(',');
  Serial.print(tc.gx1); Serial.print(','); Serial.print(tc.gy1); Serial.print(','); Serial.print(tc.gz1);
  Serial.print(F("|MPU2:"));
  Serial.print(tc.ax2); Serial.print(','); Serial.print(tc.ay2); Serial.print(','); Serial.print(tc.az2); Serial.print(',');
  Serial.print(tc.gx2); Serial.print(','); Serial.print(tc.gy2); Serial.print(','); Serial.print(tc.gz2);
  Serial.print(F("|GYR1:")); Serial.print(tc.gx1); Serial.print(','); Serial.print(tc.gy1); Serial.print(','); Serial.print(tc.gz1);
  Serial.print(F("|GYR2:")); Serial.print(tc.gx2); Serial.print(','); Serial.print(tc.gy2); Serial.print(','); Serial.print(tc.gz2);
  Serial.print(F("|SONAR:")); Serial.print(tc.sonarCm);
  for (uint8_t i=0;i<6;i++){Serial.print(F("|AN")); Serial.print(i+1); Serial.print(F(":")); Serial.print(tc.analogs[i]);}
  Serial.print(F("|INA1:")); Serial.print(tc.ina1Current);
  Serial.print(F("|INA2:")); Serial.print(tc.ina2Current);
  Serial.print(F("|MAG:"));
  Serial.print(tc.mx); Serial.print(','); Serial.print(tc.my); Serial.print(','); Serial.print(tc.mz); Serial.print(',');
  Serial.print(tc.heading,2); Serial.print(','); printI2CAddressHex(compassAddr); Serial.print(','); Serial.print((uint8_t)compassType);
  Serial.print(F("|PWM:"));
  Serial.print(pwmPerc[0]); Serial.print(','); Serial.print(pwmPerc[1]); Serial.print(',');
  Serial.print(pwmPerc[2]); Serial.print(','); Serial.print(pwmPerc[3]);
  Serial.print(F("|STEP:"));    Serial.print(motor.currentAngle, 2);
  Serial.print(F("|TGT:"));     Serial.print(motor.targetAngle, 2);
  Serial.print(F("|SPD:"));     Serial.print(motor.filteredSpeed, 2);
  Serial.print(F("|PWMRAW:"));  Serial.print(motor.pwmValue);
  Serial.print(F("|WIN:"));     Serial.print(motorFeedbackInsideWindow ? 1 : 0);
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
  noteRuntimeLinkActivity(seq);
}


// ─────────────────────────────────────────────────────────────────────
// Serial input processing  (single Serial0 path)
// ─────────────────────────────────────────────────────────────────────

// Utility: flush primary ASCII input.
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
  const uint8_t lineBudget =
    (rt.active &&
     robotControlMode == ROBOT_MODE_MANIPULATOR &&
     collisionRuntimeEnabled)
      ? SERIAL_RX_LINE_BUDGET_COLLISION
      : SERIAL_RX_LINE_BUDGET;

  while (Serial.available() > 0 &&
         bytesProcessed < SERIAL_RX_BYTE_BUDGET &&
         linesDispatched < lineBudget) {
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
// Motor diagnostics are kept on Serial0 ASCII only.
// ─────────────────────────────────────────────────────────────────────

// Prints motor diagnostic panel.
void printMotorDiagnosticPanel() {
  int rawAdc = readMotorRawADC();
  bool pwm46Ok = isMotorPwmTimerHealthy();
  Serial.println(F("[MOTOR DIAGNOSTIC]"));
  Serial.print(F("ROBOT="));    Serial.print((int)robotControlMode); Serial.print(F(" [")); Serial.print(getRobotControlModeName(robotControlMode)); Serial.println(F("]"));
  Serial.print(F("MOTORTYPE="));  Serial.print((int)motorType);     Serial.print(F(" [")); Serial.print(getMotorTypeName(motorType));         Serial.println(F("]"));
  Serial.print(F("STEPPERMODE=")); Serial.print((int)motorDriveMode); Serial.print(F(" [")); Serial.print(getMotorDriveModeName(motorDriveMode));  Serial.println(F("]"));
  Serial.print(F("ACTIVE="));   Serial.println(rt.active ? 1 : 0);
  Serial.print(F("ARMED="));    Serial.println(rt.motorArmed ? 1 : 0);
  Serial.print(F("RAWADC="));   Serial.println(rawAdc);
  Serial.print(F("ZEROADC="));  Serial.println(motorCalibrationZeroADC);
  Serial.print(F("SPANRAW="));  Serial.println(motorOneTurnSpanRaw);
  Serial.print(F("WINDOWADC=")); Serial.print(motorCalibrationZeroADC); Serial.print(F("..")); Serial.println((int32_t)motorCalibrationZeroADC + motorOneTurnSpanRaw);
  Serial.print(F("ADC_WINDOW_USAGE_PCT=")); Serial.println(100.0f * (float)motorOneTurnSpanRaw / 1023.0f, 2);
  Serial.print(F("DEGPERCOUNT=")); Serial.println(getMotorDegPerCount(), 6);
  if (motorOneTurnSpanRaw < MOTOR_CAL_RECOMMENDED_SPAN_RAW) {
    Serial.println(F("ADC_RESOLUTION_WARNING=ONE_TURN_WINDOW_BELOW_360_RAW_COUNTS"));
  }
  Serial.print(F("STEPWIN="));  Serial.println(motorFeedbackInsideWindow ? 1 : 0);
  Serial.print(F("CURRENT="));       Serial.println(motor.currentAngle, 3);
  Serial.print(F("CONTROL_ANGLE=")); Serial.println(motor.filteredAngle, 3);
  Serial.print(F("CONTROL_FEEDBACK=")); Serial.println(motor.controlFeedbackAngle, 3);
  Serial.print(F("TARGET="));        Serial.println(motor.targetAngle, 3);
  Serial.print(F("CONTROL_TARGET=")); Serial.println(motor.controlTargetAngle, 3);
  Serial.print(F("ERROR="));         Serial.println(motor.targetAngle - motor.currentAngle, 3);
  Serial.print(F("CONTROL_ERROR=")); Serial.println(motor.controlTargetAngle - motor.controlFeedbackAngle, 3);
  Serial.print(F("SPEED_DPS="));     Serial.println(motor.filteredSpeed, 3);
  uint32_t commandAgeMs = (uint32_t)(millis() - motor.lastCommandChangeMs);
  bool commandStreamingDiag =
    commandAgeMs <= (uint32_t)MOTOR_SERVO_COMMAND_STREAM_MS &&
    fabsf(motor.commandVelocityDps) >= MOTOR_SERVO_COMMAND_STREAM_RATE_DPS;
  Serial.print(F("COMMAND_VEL_DPS=")); Serial.println(motor.commandVelocityDps, 3);
  Serial.print(F("COMMAND_AGE_MS=")); Serial.println(commandAgeMs);
  Serial.print(F("COMMAND_STREAMING=")); Serial.println(commandStreamingDiag ? 1 : 0);
  Serial.print(F("A0_RAW_ALPHA_MOVING=")); Serial.println(MOTOR_FEEDBACK_RAW_ALPHA_MOVING, 3);
  Serial.print(F("A0_RAW_ALPHA_HOLD=")); Serial.println(MOTOR_FEEDBACK_RAW_ALPHA_HOLD, 3);
  Serial.print(F("KP=")); Serial.println(motor.kp, 4);
  Serial.print(F("KI=")); Serial.println(motor.ki, 4);
  Serial.print(F("KD=")); Serial.println(motor.kd, 4);
  Serial.print(F("PIDENTER=")); Serial.println((float)motorDcPidEnterBandDeg, 3);
  Serial.print(F("PIDEXIT=")); Serial.println((float)motorDcPidExitBandDeg, 3);
  Serial.print(F("PID_INTEGRAL=")); Serial.println(motor.integral, 4);
  Serial.print(F("SETTLE_SPEED_DPS=")); Serial.println(MOTOR_DC_SETTLE_SPEED_DPS, 2);
  Serial.print(F("SETTLE_DWELL_MS=")); Serial.println(MOTOR_DC_SETTLE_DWELL_MS);
  Serial.println(F("DC_TARGET_MODE=IMMEDIATE_ABSOLUTE"));
  Serial.print(F("PWM_SLEW_TICKS_PER_CONTROL=")); Serial.println(MOTOR_DC_PWM_SLEW_TICKS_PER_CONTROL);
  Serial.print(F("STICTION_START_RATIO=")); Serial.println(MOTOR_DC_STICTION_START_RATIO, 3);
  Serial.println(F("DC_CONTROL_POLICY=UNIFIED_POSITION_SERVO"));
  Serial.print(F("STICTION_RAMP_MS=")); Serial.println(MOTOR_DC_STICTION_RAMP_MS);
  Serial.print(F("STICTION_ACTIVE=")); Serial.println(motor.stictionStartMs != 0UL ? 1 : 0);
  Serial.print(F("COLL_ENABLED=")); Serial.println(collisionRuntimeEnabled ? 1 : 0);
  Serial.print(F("COLL_FLAG=")); Serial.println(collisionFlag ? 1 : 0);
  Serial.print(F("COLL_SOLVER_LAST_US=")); Serial.println(collisionSolverLastUs);
  Serial.print(F("COLL_SOLVER_MAX_US=")); Serial.println(collisionSolverMaxUs);
  Serial.print(F("COLL_SOLVER_EVALS=")); Serial.println(collisionSolverEvalCount);
  Serial.print(F("MOTOR_CTRL_GAP_LAST_US=")); Serial.println(motorControlGapLastUs);
  Serial.print(F("MOTOR_CTRL_GAP_MAX_US=")); Serial.println(motorControlGapMaxUs);
  Serial.print(F("MOTOR_CTRL_LATE_COUNT=")); Serial.println(motorControlLateCount);
  Serial.print(F("RUNTIME_TX_BUSY=")); Serial.println(runtimeHexTxBusy() ? 1 : 0);
  Serial.print(F("RUNTIME_TX_QUEUED=")); Serial.println(runtimeHexTxQueuedFrames);
  Serial.print(F("RUNTIME_TX_SKIPPED=")); Serial.println(runtimeHexTxSkippedFrames);
  Serial.println(F("AXISMODE=ONE_TURN_WINDOW_0_359_NO_WRAP"));
  Serial.print(F("PWMRAW="));   Serial.println(motor.pwmValue);
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
  if (motorType != MOTOR_TYPE_STEPPER) {
    Serial.println(F("WARNING: MOTORTYPE IS CURRENTLY DC_MOTOR"));
  }
  Serial.println();
  Serial.println(F("  MOTORDIAG                    -> print motor/collision timing diagnostics"));
  Serial.println(F("  STEPTEST=F,200,80,2200      -> direct forward test"));
  Serial.println(F("  STEPTEST=R,200,80,2200      -> direct reverse test"));
  Serial.println(F("  STEPTEST=F,<steps>,<pct>,<delayus>"));
  Serial.println();
}

// Runs a direct motor test.
void runDirectMotorTest(bool forward, uint16_t steps, uint16_t pwmPercent, unsigned long delayUs) {
  if (robotControlMode != ROBOT_MODE_MANIPULATOR) {
    Serial.println(F("DENIED: STEPTEST REQUIRES ROBOT=MANIPULATOR"));
    return;
  }
  if (motorType != MOTOR_TYPE_STEPPER) {
    Serial.println(F("DENIED: STEPTEST REQUIRES MOTORTYPE=STEPPER"));
    return;
  }

  pwmPercent = (uint16_t)clampValue((int)pwmPercent, 0, 100);
  if (steps < 1)    steps = 1;
  if (delayUs < 400) delayUs = 400;

  rt.motorArmed = false;
  motorPulse.setSpeed(0.0f);
  motorPulse.disableOutputs();
  releaseMotorCoils();

  directMotorTest.active        = true;
  directMotorTest.forward       = forward;
  directMotorTest.totalSteps    = steps;
  directMotorTest.stepsRemaining = steps;
  directMotorTest.pwmRaw        = (uint16_t)map((long)pwmPercent, 0L, 100L, 0L, (long)ICR5);
  directMotorTest.stepDelayUs   = delayUs;
  directMotorTest.lastStepUs    = 0;

  motorSetDriverPWM(directMotorTest.pwmRaw);
  applyStepOutput(motor.seq, directMotorTest.pwmRaw);

  Serial.print(F("STEPTEST START DIR="));
  Serial.print(forward ? F("F") : F("R"));
  Serial.print(F(" STEPS="));    Serial.print(steps);
  Serial.print(F(" PWM%="));     Serial.print(pwmPercent);
  Serial.print(F(" DELAYUS="));  Serial.println(delayUs);
}

// Updates the direct motor test tick.
void updateDirectMotorTest() {
  if (!directMotorTest.active) return;

  unsigned long nowUs = micros();
  if (directMotorTest.lastStepUs == 0) directMotorTest.lastStepUs = nowUs;

  uint8_t burstCount = 0;
  while (directMotorTest.stepsRemaining > 0 &&
         (unsigned long)(nowUs - directMotorTest.lastStepUs) >= directMotorTest.stepDelayUs &&
         burstCount < 8) {
    directMotorTest.lastStepUs += directMotorTest.stepDelayUs;
    if (directMotorTest.forward) accelStepForward();
    else                           accelStepBackward();
    applyStepOutput(motor.seq, directMotorTest.pwmRaw);
    directMotorTest.stepsRemaining--;
    burstCount++;
  }

  unsigned long burstWindowUs = directMotorTest.stepDelayUs * 8UL;
  if ((unsigned long)(nowUs - directMotorTest.lastStepUs) > burstWindowUs) {
    directMotorTest.lastStepUs = nowUs;
  }

  if (directMotorTest.stepsRemaining == 0) {
    directMotorTest.active = false;
    motorSetDriverPWM(0);
    releaseMotorCoils();
    readMotorAngle();
    motor.targetAngle  = motor.currentAngle;
    servos.target[0]     = motor.currentAngle;
    servos.actual[0]     = motor.currentAngle;
    Serial.println(F("STEPTEST DONE"));
    printMotorDiagnosticPanel();
  }
}

// Handles motor diagnostic commands (STEPDIAG, STEPTEST=...).
bool handleMotorDiagnosticCommand(char* cmd) {
  if (cmd == NULL || cmd[0] == '\0') return false;

  if (strcmp(cmd, "MOTORDIAG") == 0 ||
      strcmp(cmd, "STEPDIAG") == 0 ||
      strcmp(cmd, "STEPSTATUS") == 0) {
    printMotorDiagnosticPanel();
    return true;
  }

  if (strcmp(cmd, "MOTORDIAGRESET") == 0) {
    collisionSolverLastUs = 0;
    collisionSolverMaxUs = 0;
    collisionSolverEvalCount = 0;
    motorControlGapLastUs = 0;
    motorControlGapMaxUs = 0;
    motorControlLateCount = 0;
    runtimeHexTxQueuedFrames = 0;
    runtimeHexTxSkippedFrames = 0;
    Serial.println(F("MOTOR/COLLISION TIMING DIAGNOSTICS RESET"));
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

    runDirectMotorTest(fwd, steps, pwmPct, dUs);
    return true;
  }

  return false;
}


// ─────────────────────────────────────────────────────────────────────
// Runtime motor controller update.
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

  droneAltitudeFromGroundCm = max(0.0f, verticalCm - max(0.0f, (float)droneSonarGroundReferenceCm));
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
  return max(0.0f, max((float)droneAltitudeFromGroundCm, (float)droneTelemetryAltitudeCm));
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

// Utility: service active motor motion.
void serviceActiveMotorMotion(unsigned long nowUs) {
  if (motor.inHoldBand || motorType == MOTOR_TYPE_DC_MOTOR) return;
  motorPulse.enableOutputs();
  motorSetDriverPWM(motor.pwmValue);

  float stepsPerSecond = 1000000.0f / max(1.0f, (float)motor.stepDelayUs);
  if (!motor.directionForward) stepsPerSecond = -stepsPerSecond;
  motorPulse.setSpeed(stepsPerSecond);
  unsigned long svcBudgetStart = micros();
  do {
    motorPulse.runSpeed();
  } while ((uint32_t)(micros() - svcBudgetStart) < MOTOR_STEP_SERVICE_BUDGET_US);
  motor.lastStepUs = micros();
}

// Updates motor control.
void updateMotorControl() {
  if (!rt.active) return;
  if (directMotorTest.active) return;

  unsigned long nowUs = micros();

  // Stepper pulses need fast service between control calculations. The same
  // absolute 0..359 degree target/trajectory logic below is shared by both
  // motor technologies; only the final actuator stage is different.
  if (motorType == MOTOR_TYPE_STEPPER && rt.motorArmed && !motor.inHoldBand) {
    serviceActiveMotorMotion(nowUs);
  }

  unsigned long previousControlUs = motor.lastControlUs;
  if (previousControlUs != 0UL &&
      (uint32_t)(nowUs - previousControlUs) < (uint32_t)controlIntervalUs) {
    return;
  }

  if (previousControlUs != 0UL) {
    uint32_t controlGapUs = (uint32_t)(nowUs - previousControlUs);
    motorControlGapLastUs = controlGapUs;
    if (controlGapUs > motorControlGapMaxUs) motorControlGapMaxUs = controlGapUs;
    // Count a late control pass only when it exceeds two nominal periods.
    if (controlGapUs > (uint32_t)(controlIntervalUs * 2UL)) {
      motorControlLateCount++;
    }
  }
  motor.lastControlUs = nowUs;

  readMotorAngle();
  unsigned long nowMs = millis();

  // The controller uses the endpoint-aware feedback coordinate. Public feedback
  // remains strictly 0..359, but the PID is allowed a small bounded overrange
  // derived from the same filtered A0 signal. This removes the endpoint blind
  // zone that made small commands near 0/359 behave like large, imprecise moves.
  float feedbackAngle = motor.controlFeedbackAngle;

  if (motorType == MOTOR_TYPE_DC_MOTOR) {
    motorPulse.setSpeed(0.0f);
    motorPulse.disableOutputs();
  }

  if (!rt.motorArmed) {
    motor.targetAngle = motor.currentAngle;
    motor.controlTargetAngle = feedbackAngle;
    motor.controlTargetInitialized = false;
    motor.commandTrackerInitialized = false;
    motor.lastCommandAngle = motor.currentAngle;
    motor.commandVelocityDps = 0.0f;
    motor.lastCommandChangeMs = nowMs;
    servos.target[0] = motor.currentAngle;
    servos.actual[0] = motor.currentAngle;
    motor.integral = 0.0f;
    motor.filteredSpeed = 0.0f;
    motor.lastAngle = feedbackAngle;
    motor.lastAngleUs = nowUs;
    motor.stictionStartMs = 0;
    motor.stictionSign = 0;
    motor.inHoldBand = true;
    motorPulse.setSpeed(0.0f);
    motorPulse.disableOutputs();
    releaseMotorCoils();
    if (motorType == MOTOR_TYPE_DC_MOTOR) {
      mirrorPwm46FromCh12ForDcMotor();
    } else {
      motorSetDriverPWM(0);
    }
    return;
  }

  // CH0 target is an absolute bounded 0..359 reference. Endpoint behavior must
  // not depend on current position; only saturate malformed out-of-range input.
  float protectedTarget = resolveLinearBaseTargetDeg(motor.targetAngle, feedbackAngle);
  if (fabsf(protectedTarget - motor.targetAngle) > MOTOR_CONTROL_TARGET_EPS_DEG) {
    motor.targetAngle = protectedTarget;
    servos.target[0] = protectedTarget;
    motor.integral = 0.0f;
    motor.inHoldBand = false;
    motor.stictionStartMs = 0;
    motor.stictionSign = 0;
  }

  float dt = (previousControlUs != 0UL)
             ? (float)(uint32_t)(nowUs - previousControlUs) * 0.000001f
             : (float)controlIntervalUs * 0.000001f;
  dt = clampValue(dt, MOTOR_CONTROL_DT_MIN_S, MOTOR_CONTROL_DT_MAX_S);

  // Velocity is derivative-on-measurement. For DC, estimate it over a longer
  // window than the position loop so one ADC transition cannot create a huge
  // KD spike. Stepper mode keeps the ordinary control-period estimate.
  if (motorType == MOTOR_TYPE_DC_MOTOR) {
    if (motor.lastAngleUs == 0UL) {
      motor.lastAngleUs = nowUs;
      motor.lastAngle = feedbackAngle;
      motor.filteredSpeed = 0.0f;
    } else if ((uint32_t)(nowUs - motor.lastAngleUs) >= MOTOR_DC_SPEED_ESTIMATE_INTERVAL_US) {
      float speedDt = (float)(uint32_t)(nowUs - motor.lastAngleUs) * 0.000001f;
      speedDt = max(speedDt, MOTOR_SPEED_DT_MIN_S);
      float measuredSpeed = (feedbackAngle - motor.lastAngle) / speedDt;
      measuredSpeed = clampValue(measuredSpeed,
                                 -MOTOR_DC_SPEED_SANITY_LIMIT_DPS,
                                  MOTOR_DC_SPEED_SANITY_LIMIT_DPS);
      motor.filteredSpeed = blendFloat(motor.filteredSpeed,
                                       measuredSpeed,
                                       speedFilterAlpha);
      motor.lastAngle = feedbackAngle;
      motor.lastAngleUs = nowUs;
    }
  } else {
    float measuredSpeed = (feedbackAngle - motor.lastAngle) / dt;
    motor.lastAngle = feedbackAngle;
    motor.filteredSpeed = blendFloat(motor.filteredSpeed,
                                     measuredSpeed,
                                     speedFilterAlpha);
  }

  float degPerCount = getMotorDegPerCount();
  // Precision settle window: the DC controller is allowed to use roughly
  // half of one raw ADC code after filtering.  The old 0.85-code rule could
  // accept nearly a full raw count of final error (about 1.2 deg at SPANRAW=300).
  float dcHoldBand = max((float)motorDcPidEnterBandDeg,
                         degPerCount * MOTOR_DC_SETTLE_ADC_FRACTION);
  float dcReleaseBand = max((float)motorDcPidExitBandDeg,
                            dcHoldBand + degPerCount * MOTOR_DC_RELEASE_ADC_FRACTION);
  float stepperHoldBand = max((float)motorStopEnter,
                              degPerCount * MOTOR_STEPPER_SETTLE_ADC_FRACTION);
  float stepperReleaseBand = max((float)motorStopExit,
                                 stepperHoldBand + max(MOTOR_STEPPER_RELEASE_MIN_DEG,
                                      degPerCount * MOTOR_STEPPER_RELEASE_ADC_FRACTION));

  if (!motorFeedbackInsideWindow) {
    float endpointResolutionBand = max(dcHoldBand, degPerCount);
    dcHoldBand = endpointResolutionBand;
    dcReleaseBand = max(dcReleaseBand, endpointResolutionBand + degPerCount);
    stepperHoldBand = max(stepperHoldBand, endpointResolutionBand);
    stepperReleaseBand = max(stepperReleaseBand, stepperHoldBand + degPerCount);
  }

  // STOPENTER/STOPEXIT remain stepper tuning. DC has independent
  // PIDENTER/PIDEXIT tuning, but both technologies share the target tracker.
  float holdBand = (motorType == MOTOR_TYPE_STEPPER)
    ? stepperHoldBand : dcHoldBand;
  float releaseBand = (motorType == MOTOR_TYPE_STEPPER)
    ? stepperReleaseBand : dcReleaseBand;

  // Absolute-servo reference tracker shared by DC and stepper. Every command
  // source writes motor.targetAngle in 0..359. The internal target then advances
  // smoothly, preventing a large command jump from becoming an actuator shock.
  float commandTarget = clampValue(motor.targetAngle,
                                   MOTOR_INPUT_MIN,
                                   MOTOR_INPUT_MAX);
  if (!motor.controlTargetInitialized) {
    motor.controlTargetAngle = feedbackAngle;
    motor.controlTargetInitialized = true;
  }
  if (!motor.commandTrackerInitialized) {
    motor.lastCommandAngle = commandTarget;
    motor.commandVelocityDps = 0.0f;
    motor.lastCommandChangeMs = nowMs;
    motor.commandTrackerInitialized = true;
  }

  float commandDelta = commandTarget - motor.lastCommandAngle;
  if (fabsf(commandDelta) > MOTOR_SERVO_COMMAND_EPS_DEG) {
    float dtCmd = (float)(uint32_t)(nowMs - motor.lastCommandChangeMs) * 0.001f;
    dtCmd = clampValue(dtCmd, MOTOR_SERVO_COMMAND_DT_MIN_S, MOTOR_SERVO_COMMAND_DT_MAX_S);
    float instantCommandVelocity = commandDelta / dtCmd;
    motor.commandVelocityDps = blendFloat(motor.commandVelocityDps,
                                          instantCommandVelocity,
                                          MOTOR_SERVO_COMMAND_VELOCITY_ALPHA);
    motor.lastCommandAngle = commandTarget;
    motor.lastCommandChangeMs = nowMs;
  } else if ((uint32_t)(nowMs - motor.lastCommandChangeMs) >
             (uint32_t)MOTOR_SERVO_COMMAND_STREAM_MS) {
    // Do not decay reference velocity every 3 ms between two normal desktop
    // frames. That repeatedly toggled commandStreaming on/off and changed the
    // DC control law while the operator was moving the base continuously.
    // Hold the last measured reference velocity for one short stream window;
    // once the command is genuinely stale, clear it in one deterministic step.
    motor.commandVelocityDps = 0.0f;
  }

  float commandError = commandTarget - feedbackAngle;
  float commandAbsError = fabsf(commandError);
  bool commandStreaming =
    ((uint32_t)(nowMs - motor.lastCommandChangeMs) <=
      (uint32_t)MOTOR_SERVO_COMMAND_STREAM_MS) &&
    (fabsf(motor.commandVelocityDps) >= MOTOR_SERVO_COMMAND_STREAM_RATE_DPS);

  // V1 response model:
  // - DC motor behaves like an ordinary positional servo: a received absolute
  //   angle becomes the PID reference immediately. A reference slew would
  //   add tens or even hundreds of milliseconds on large one-shot commands,
  //   which was perceived as a brief hesitation compared with PCA servos.
  // - Stepper keeps the bounded reference trajectory because its actuator layer
  //   must convert position demand into an acceleration-limited step rate.
  if (motorType == MOTOR_TYPE_DC_MOTOR) {
    motor.controlTargetAngle = commandTarget;
  } else {
    float desiredControlTarget = commandTarget;
    bool directServoFollow = commandStreaming ||
                             (commandAbsError >= MOTOR_SERVO_DIRECT_ERROR_DEG);
    if (!directServoFollow &&
        commandAbsError > MOTOR_SERVO_STATIC_TARGET_LEAD_DEG) {
      desiredControlTarget = feedbackAngle +
        ((commandError > 0.0f)
          ? MOTOR_SERVO_STATIC_TARGET_LEAD_DEG
          : -MOTOR_SERVO_STATIC_TARGET_LEAD_DEG);
    }
    desiredControlTarget = clampValue(desiredControlTarget,
                                      MOTOR_INPUT_MIN,
                                      MOTOR_INPUT_MAX);

    float targetSlewDps = directServoFollow
      ? MOTOR_SERVO_TRACK_TARGET_SLEW_DPS
      : MOTOR_SERVO_FINE_TARGET_SLEW_DPS;
    float maxTargetStep = max(MOTOR_STEPPER_MIN_TARGET_STEP_DEG, targetSlewDps * dt);
    float targetDelta = desiredControlTarget - motor.controlTargetAngle;
    if (fabsf(targetDelta) <= maxTargetStep) {
      motor.controlTargetAngle = desiredControlTarget;
    } else {
      motor.controlTargetAngle += (targetDelta > 0.0f)
        ? maxTargetStep : -maxTargetStep;
    }
    motor.controlTargetAngle = clampValue(motor.controlTargetAngle,
                                          MOTOR_INPUT_MIN,
                                          MOTOR_INPUT_MAX);
  }

  float error = motor.controlTargetAngle - feedbackAngle;
  float absError = fabsf(error);

  // Action threshold derives from configured HOLD tolerance, A0 deadband and
  // actual ADC angular resolution. The same equation is used for every target.
  float commandIntentBand = holdBand;
  if (motorType == MOTOR_TYPE_DC_MOTOR) {
    float resolutionIntent = max((float)motorBaseA0DeadbandDeg,
                                 degPerCount * MOTOR_DC_COMMAND_INTENT_ADC_FRACTION);
    commandIntentBand = min(holdBand,
                            max(resolutionIntent,
                                holdBand * MOTOR_DC_COMMAND_INTENT_HOLD_RATIO));
  }
  bool commandPending = commandAbsError > commandIntentBand;

  // DC arrival is declared only after BOTH position and speed are settled for
  // a short dwell. Position alone is not enough while the base still has
  // base inertia can otherwise carry the final angle past the command after
  // PWM is removed. Stepper keeps its previous stop semantics.
  bool positionSettled = absError <= holdBand && !commandPending;
  bool speedSettled = fabsf(motor.filteredSpeed) <= MOTOR_DC_SETTLE_SPEED_DPS;
  bool dcDwellSettled = false;
  if (motorType == MOTOR_TYPE_DC_MOTOR && positionSettled && speedSettled) {
    if (motor.holdStartMs == 0UL) motor.holdStartMs = nowMs;
    dcDwellSettled = (uint32_t)(nowMs - motor.holdStartMs) >= MOTOR_DC_SETTLE_DWELL_MS;
  } else if (motorType == MOTOR_TYPE_DC_MOTOR) {
    motor.holdStartMs = 0UL;
  }

  if ((motorType == MOTOR_TYPE_DC_MOTOR && dcDwellSettled) ||
      (motorType != MOTOR_TYPE_DC_MOTOR && positionSettled)) {
    // Keep a small amount of learned integral bias instead of erasing it at
    // every arrival. This helps the axis reject gravity/cable/static-friction
    // bias when it is asked to hold the same absolute angle.
    motor.integral *= (motorType == MOTOR_TYPE_DC_MOTOR)
      ? MOTOR_DC_HOLD_INTEGRAL_RETENTION : 0.0f;
    motor.inHoldBand = true;
    motor.stictionStartMs = 0;
    motor.stictionSign = 0;
    motor.controlTargetAngle = commandTarget;
    motor.pwmValue = holdPwm;
    if (motorType == MOTOR_TYPE_DC_MOTOR) {
      dcMotorStop();
    } else {
      motorPulse.setSpeed(0.0f);
      motorPulse.enableOutputs();
      applyStepOutput(motor.seq, motor.pwmValue);
    }
    return;
  }

  if (motor.inHoldBand && absError <= releaseBand && !commandPending) {
    motor.integral *= MOTOR_DC_RELEASE_INTEGRAL_RETENTION;
    motor.stictionStartMs = 0;
    motor.stictionSign = 0;
    motor.pwmValue = movePwmNear;
    if (motorType == MOTOR_TYPE_DC_MOTOR) {
      dcMotorStop();
    } else {
      motorPulse.setSpeed(0.0f);
      motorPulse.enableOutputs();
      applyStepOutput(motor.seq, motor.pwmValue);
    }
    return;
  }

  motor.inHoldBand = false;
  motor.directionForward = (error > 0.0f);

  if (motorType == MOTOR_TYPE_DC_MOTOR) {
    // One continuous positional-servo controller. Integral authority varies
    // smoothly with error, D damps measured motion, and feed-forward follows a
    // moving reference. No target-size-specific mode is selected.
    float integralWeight = 1.0f - clampValue(
      absError / MOTOR_DC_INTEGRAL_FADE_ERROR_DEG, 0.0f, 1.0f);
    if (!commandStreaming) {
      motor.integral += error * dt * integralWeight;
    }
    float integralLeakWeight = commandStreaming ? 1.0f : (1.0f - integralWeight);
    float integralRetention = clampValue(
      1.0f - MOTOR_DC_INTEGRAL_LEAK_PER_SECOND * integralLeakWeight * dt,
      0.0f, 1.0f);
    motor.integral *= integralRetention;
    motor.integral = clampValue(motor.integral,
                                -MOTOR_DC_INTEGRAL_LIMIT,
                                 MOTOR_DC_INTEGRAL_LIMIT);

    float errorSign = (error >= 0.0f) ? 1.0f : -1.0f;
    float pTerm = motor.kp * error;
    float iTerm = motor.ki * motor.integral;
    float dScale = MOTOR_DC_D_MIN_SCALE +
      (1.0f - MOTOR_DC_D_MIN_SCALE) *
      clampValue(absError / MOTOR_DC_D_FULL_ERROR_DEG, 0.0f, 1.0f);
    float dTerm = motor.kd * (-motor.filteredSpeed) * dScale;
    float dLimit = max(MOTOR_DC_D_MIN_LIMIT,
                       fabsf(pTerm) * MOTOR_DC_D_RELATIVE_LIMIT);
    dTerm = clampValue(dTerm, -dLimit, dLimit);

    float referenceFeedForward = 0.0f;
    if (commandStreaming) {
      referenceFeedForward = clampValue(
        motor.commandVelocityDps * MOTOR_DC_REFERENCE_FF_GAIN,
        -MOTOR_DC_REFERENCE_FF_LIMIT,
         MOTOR_DC_REFERENCE_FF_LIMIT);
    }

    float pidEffort = pTerm + iTerm + dTerm + referenceFeedForward;

    // Measurement damping may reduce demanded torque, but it cannot reverse a
    // still-pending position demand. This is a generic servo invariant.
    if ((pidEffort * error) <= 0.0f && commandPending) {
      float minimumTowardTargetEffort = max(
        MOTOR_DC_OPPOSING_EFFORT_MIN_ABS,
        fabsf(pTerm) * MOTOR_DC_OPPOSING_EFFORT_MIN_RATIO);
      pidEffort = errorSign * minimumTowardTargetEffort;
    }

    uint16_t controlDuty = getDcMotorMirrorPwmDutyFromCh12();
    if (controlDuty == 0) {
      motor.stictionStartMs = 0;
      dcMotorStop();
      return;
    }

    float absEffort = fabsf(pidEffort);
    motor.directionForward = (pidEffort > 0.0f);
    int8_t directionSign = motor.directionForward ? 1 : -1;

    float pwmRatio = clampValue(absEffort / MOTOR_DC_PID_EFFORT_FULL_SCALE,
                                0.0f, 1.0f);
    float powerError = max(absError, commandAbsError);
    float errorPowerRatio = clampValue(
      powerError / MOTOR_DC_FULL_POWER_ERROR_DEG, 0.0f, 1.0f);
    float dutyCeilingRatio = MOTOR_DC_POWER_CEILING_MIN_RATIO +
      (1.0f - MOTOR_DC_POWER_CEILING_MIN_RATIO) * errorPowerRatio;
    uint16_t effectiveControlDuty = (uint16_t)roundf(
      (float)controlDuty * dutyCeilingRatio);
    effectiveControlDuty = (uint16_t)clampValue(
      (int)effectiveControlDuty, 1, (int)controlDuty);

    float minRunRatio = MOTOR_DC_MIN_RUN_START_RATIO +
      MOTOR_DC_MIN_RUN_EXTRA_RATIO *
      clampValue(powerError / MOTOR_DC_MIN_RUN_FULL_ERROR_DEG, 0.0f, 1.0f);
    uint16_t minRunDuty = (uint16_t)max(
      1, (int)roundf((float)controlDuty * minRunRatio));

    float speedAbs = fabsf(motor.filteredSpeed);
    bool stalled = speedAbs <= MOTOR_DC_STALL_SPEED_DPS;
    bool needsStictionAssist = stalled && commandPending;

    if (directionSign != motor.stictionSign) {
      motor.stictionSign = directionSign;
      motor.stictionStartMs = 0;
    }

    if (needsStictionAssist) {
      if (motor.stictionStartMs == 0UL) motor.stictionStartMs = nowMs;
      uint32_t stallAgeMs = (uint32_t)(nowMs - motor.stictionStartMs);
      float stallRamp = clampValue(
        (float)stallAgeMs / (float)MOTOR_DC_STICTION_RAMP_MS,
        0.0f, 1.0f);
      float errorAssist = clampValue(
        powerError / MOTOR_DC_STICTION_FULL_ERROR_DEG,
        0.0f, 1.0f);
      float stictionMaxRatio = MOTOR_DC_STICTION_START_RATIO +
        (MOTOR_DC_STICTION_MAX_RATIO - MOTOR_DC_STICTION_START_RATIO) * errorAssist;
      float stictionRatio = MOTOR_DC_STICTION_START_RATIO +
        (stictionMaxRatio - MOTOR_DC_STICTION_START_RATIO) * stallRamp;

      if (stallAgeMs > MOTOR_DC_STICTION_RAMP_MS) {
        float fullRamp = clampValue(
          (float)(stallAgeMs - MOTOR_DC_STICTION_RAMP_MS) /
          (float)(MOTOR_DC_STICTION_FULL_RAMP_MS - MOTOR_DC_STICTION_RAMP_MS),
          0.0f, 1.0f);
        stictionRatio += (1.0f - stictionRatio) * fullRamp;
      }

      uint16_t stictionDuty = (uint16_t)roundf(
        (float)controlDuty * stictionRatio);
      minRunDuty = max(minRunDuty, stictionDuty);
      effectiveControlDuty = max(effectiveControlDuty, stictionDuty);
    } else {
      motor.stictionStartMs = 0;
    }

    if (minRunDuty > effectiveControlDuty) minRunDuty = effectiveControlDuty;
    uint16_t pidDuty = (uint16_t)roundf(
      (float)minRunDuty +
      pwmRatio * (float)(effectiveControlDuty - minRunDuty));

    dcMotorDrive(motor.directionForward, pidDuty);
    return;
  }

  // Stepper actuator: same absolute target and internal trajectory as DC, but
  // output is step frequency/direction rather than PWM torque PID.
  motor.integral = 0.0f;
  motor.stictionStartMs = 0;
  motor.stictionSign = 0;

  float errorSpan = max(8.0f, MOTOR_INPUT_MAX * 0.20f);
  float speedRatio = clampValue(absError / errorSpan, 0.0f, 1.0f);
  float pwmRatio = clampValue(
    absError / max(12.0f, MOTOR_INPUT_MAX * 0.14f), 0.0f, 1.0f);

  float desiredStepDelay = (float)maxStepDelayUs -
    speedRatio * (float)(maxStepDelayUs - minStepDelayUs);
  unsigned long desiredDelayUs = clampValue(
    (unsigned long)roundf(desiredStepDelay),
    (unsigned long)minStepDelayUs,
    (unsigned long)maxStepDelayUs);

  // Smooth changes in step frequency so a new absolute angle resembles a servo
  // trajectory instead of an instantaneous step-rate jump.
  const unsigned long STEP_DELAY_SLEW_UP_US = 850UL;    // accelerate
  const unsigned long STEP_DELAY_SLEW_DOWN_US = 1300UL; // decelerate faster
  if (motor.stepDelayUs < (unsigned long)minStepDelayUs ||
      motor.stepDelayUs > (unsigned long)maxStepDelayUs) {
    motor.stepDelayUs = maxStepDelayUs;
  }
  if (desiredDelayUs < motor.stepDelayUs) {
    unsigned long delta = motor.stepDelayUs - desiredDelayUs;
    motor.stepDelayUs -= min(delta, STEP_DELAY_SLEW_UP_US);
  } else if (desiredDelayUs > motor.stepDelayUs) {
    unsigned long delta = desiredDelayUs - motor.stepDelayUs;
    motor.stepDelayUs += min(delta, STEP_DELAY_SLEW_DOWN_US);
  }
  motor.stepDelayUs = clampValue(
    motor.stepDelayUs,
    (unsigned long)minStepDelayUs,
    (unsigned long)maxStepDelayUs);

  float pwmTarget = (float)movePwmNear +
    pwmRatio * (float)(movePwmFar - movePwmNear);
  motor.pwmValue = clampValue(
    (uint16_t)roundf(pwmTarget), movePwmNear, movePwmFar);

  motorPulse.enableOutputs();
  motorSetDriverPWM(motor.pwmValue);
  float stepsPerSecond = 1000000.0f /
    max(1.0f, (float)motor.stepDelayUs);
  if (!motor.directionForward) stepsPerSecond = -stepsPerSecond;
  motorPulse.setSpeed(stepsPerSecond);

  unsigned long stepBudgetStart = micros();
  do {
    motorPulse.runSpeed();
  } while ((uint32_t)(micros() - stepBudgetStart) <
           MOTOR_STEP_SERVICE_BUDGET_US);
  motor.lastStepUs = micros();
}
