// =====================================================================
// SynROV Firmware - GPS on Serial1
// ---------------------------------------------------------------------
// Purpose:
//   Keep GPS input isolated from Serial0 so the Processing control stream
//   remains deterministic. The parser is non-blocking and has a small byte
//   budget per loop cycle.
// =====================================================================

#ifndef SYNROV_GPS_SERIAL1_H
#define SYNROV_GPS_SERIAL1_H

#if defined(HAVE_HWSERIAL1) || defined(UBRR1H) || defined(SERIAL_PORT_HARDWARE1) || defined(Serial1)
  #define SYNROV_HAS_GPS_SERIAL1 1
#else
  #define SYNROV_HAS_GPS_SERIAL1 0
#endif

static const uint32_t GPS_SERIAL_BAUD_RATE = 9600UL;
static const uint8_t  GPS_RX_BYTE_BUDGET = 48;
static const uint16_t GPS_TELEMETRY_INTERVAL_MS = 250;
static const uint32_t GPS_FIX_FRESH_MS = 2500UL;

struct GpsRuntimeSnapshot {
  bool portReady;
  bool fixValid;
  bool sentenceValid;
  uint8_t satellites;
  uint16_t hdopX100;
  int32_t latitudeE7;
  int32_t longitudeE7;
  int32_t altitudeCm;
  uint16_t speedCms;
  uint16_t courseDegX100;
  uint32_t lastSentenceMs;
  uint32_t lastFixMs;
  uint32_t lastTelemetryMs;
  char rxLine[96];
  uint8_t rxLen;
};

GpsRuntimeSnapshot gpsRuntime = { false, false, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "", 0 };

static int gpsHexToNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return -1;
}

static bool gpsCheckNmeaChecksum(const char* line) {
  if (line == NULL || line[0] != '$') return false;
  const char* star = strchr(line, '*');
  if (star == NULL || star[1] == '\0' || star[2] == '\0') return false;
  uint8_t calc = 0;
  for (const char* p = line + 1; p < star; ++p) calc ^= (uint8_t)(*p);
  int hi = gpsHexToNibble(star[1]);
  int lo = gpsHexToNibble(star[2]);
  if (hi < 0 || lo < 0) return false;
  return calc == (uint8_t)((hi << 4) | lo);
}

static uint8_t gpsTokenize(char* text, char** fields, uint8_t maxFields) {
  uint8_t count = 0;
  char* p = text;
  while (p != NULL && count < maxFields) {
    fields[count++] = p;
    char* comma = strchr(p, ',');
    if (comma == NULL) break;
    *comma = '\0';
    p = comma + 1;
  }
  return count;
}

static int32_t gpsParseLatLonE7(const char* value, const char* hemi) {
  if (value == NULL || hemi == NULL || value[0] == '\0') return 0;
  float raw = atof(value);
  int degrees = (int)(raw / 100.0f);
  float minutes = raw - (degrees * 100.0f);
  double decimal = (double)degrees + ((double)minutes / 60.0);
  if (hemi[0] == 'S' || hemi[0] == 'W') decimal = -decimal;
  return (int32_t)(decimal * 10000000.0 + (decimal >= 0.0 ? 0.5 : -0.5));
}

static uint16_t gpsParseScaledU16(const char* value, float scale) {
  if (value == NULL || value[0] == '\0') return 0;
  float v = atof(value) * scale;
  if (v < 0.0f) v = 0.0f;
  if (v > 65535.0f) v = 65535.0f;
  return (uint16_t)(v + 0.5f);
}

static int32_t gpsParseScaledI32(const char* value, float scale) {
  if (value == NULL || value[0] == '\0') return 0;
  float v = atof(value) * scale;
  return (int32_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
}

static void gpsApplyFixValidity(bool valid) {
  gpsRuntime.fixValid = valid;
  if (valid) gpsRuntime.lastFixMs = millis();
}

static void gpsParseGga(char* sentence) {
  char* star = strchr(sentence, '*');
  if (star != NULL) *star = '\0';
  char* fields[16];
  uint8_t n = gpsTokenize(sentence, fields, 16);
  if (n < 10) return;
  int fixQuality = atoi(fields[6]);
  gpsRuntime.satellites = (uint8_t)constrain(atoi(fields[7]), 0, 255);
  gpsRuntime.hdopX100 = gpsParseScaledU16(fields[8], 100.0f);
  gpsRuntime.altitudeCm = gpsParseScaledI32(fields[9], 100.0f);
  if (fixQuality > 0) {
    gpsRuntime.latitudeE7 = gpsParseLatLonE7(fields[2], fields[3]);
    gpsRuntime.longitudeE7 = gpsParseLatLonE7(fields[4], fields[5]);
    gpsApplyFixValidity(true);
  } else {
    gpsApplyFixValidity(false);
  }
}

static void gpsParseRmc(char* sentence) {
  char* star = strchr(sentence, '*');
  if (star != NULL) *star = '\0';
  char* fields[16];
  uint8_t n = gpsTokenize(sentence, fields, 16);
  if (n < 9) return;
  bool valid = fields[2] != NULL && fields[2][0] == 'A';
  if (valid) {
    gpsRuntime.latitudeE7 = gpsParseLatLonE7(fields[3], fields[4]);
    gpsRuntime.longitudeE7 = gpsParseLatLonE7(fields[5], fields[6]);
    gpsRuntime.speedCms = gpsParseScaledU16(fields[7], 51.4444f); // knots to cm/s
    gpsRuntime.courseDegX100 = gpsParseScaledU16(fields[8], 100.0f);
  }
  gpsApplyFixValidity(valid);
}

static bool gpsFixFresh() {
  if (!gpsRuntime.fixValid) return false;
  return (millis() - gpsRuntime.lastFixMs) <= GPS_FIX_FRESH_MS;
}

static void gpsParseLine(char* line) {
  if (line == NULL || line[0] == '\0') return;
  gpsRuntime.lastSentenceMs = millis();
  gpsRuntime.sentenceValid = gpsCheckNmeaChecksum(line);
  if (!gpsRuntime.sentenceValid) return;
  if (strstr(line, "GGA,") != NULL || strstr(line, "GNGGA,") != NULL || strstr(line, "GPGGA,") != NULL) {
    gpsParseGga(line);
  } else if (strstr(line, "RMC,") != NULL || strstr(line, "GNRMC,") != NULL || strstr(line, "GPRMC,") != NULL) {
    gpsParseRmc(line);
  }
}

void initGpsSerial1() {
#if SYNROV_HAS_GPS_SERIAL1
  Serial1.begin(GPS_SERIAL_BAUD_RATE);
  gpsRuntime.portReady = true;
#else
  gpsRuntime.portReady = false;
#endif
}

void pollGpsSerial1() {
#if SYNROV_HAS_GPS_SERIAL1
  if (!gpsRuntime.portReady) return;
  uint8_t budget = GPS_RX_BYTE_BUDGET;
  while (budget-- > 0 && Serial1.available() > 0) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;
    if (c == '\n') {
      gpsRuntime.rxLine[gpsRuntime.rxLen] = '\0';
      gpsParseLine(gpsRuntime.rxLine);
      gpsRuntime.rxLen = 0;
    } else if (gpsRuntime.rxLen < sizeof(gpsRuntime.rxLine) - 1) {
      gpsRuntime.rxLine[gpsRuntime.rxLen++] = c;
    } else {
      gpsRuntime.rxLen = 0;
    }
  }
#endif
}

#endif
