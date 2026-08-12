// =====================================================================
// SynROV Processing - Browser visual stream service
// ---------------------------------------------------------------------
// Purpose:
//   Keeps optional browser JPEG publishing separate from control/state and
//   from the dedicated AiBot perception channel. The browser must opt in, so
//   no image encoding work is performed when only the AiBot is connected.
// =====================================================================

long lastWebSocketFrameSentMillis = 0;
int webSocketFrameIntervalMs = 200;
long lastAiBotClientHeartbeatMillis = 0;
final int AIBOT_CONTROL_AUTHORITY_TIMEOUT_MS = 2200;
boolean aiBotTorqueAuthorityWasConnected = false;

// AiBot owns host-side Auto Torque only while its dedicated heartbeat is fresh.
boolean isAiBotControlAuthorityConnected() {
  return webClientConnected && lastAiBotClientHeartbeatMillis > 0 &&
    (millis() - lastAiBotClientHeartbeatMillis) <= AIBOT_CONTROL_AUTHORITY_TIMEOUT_MS;
}
long webVisualStreamRequestedUntilMs = 0;
final int WEB_VISUAL_STREAM_REQUEST_TTL_MS = 2200;
int wsStreamJpegWidth = 640;
int wsStreamJpegHeight = 360;
int wsStreamCaptureMargin = 8;

boolean isWebVisualStreamRequested() {
  return millis() <= webVisualStreamRequestedUntilMs;
}

void noteWebClientState(JSONObject command) {
  if (command == null) return;
  String action = getJsonString(command, "client", "ping");
  String normalized = trim(action).toLowerCase();
  String runtimeName = getJsonString(command, "runtime", "").toLowerCase();
  String sourceName = normalizeProtocolSource(getJsonString(command, "source", SYNROV_SOURCE_WEB));
  boolean dedicatedAi = sourceName.equals(SYNROV_SOURCE_AIBOT);
  boolean requested = command.hasKey("visualStream")
    ? getJsonBoolean(command, "visualStream", false)
    : !dedicatedAi;

  if (requested && (normalized.equals("connect") || normalized.equals("ping"))) {
    webVisualStreamRequestedUntilMs = millis() + WEB_VISUAL_STREAM_REQUEST_TTL_MS;
  }

  if (normalized.equals("connect") || normalized.equals("ping")) {
    markProtocolHeartbeat(sourceName);
    if (normalized.equals("connect")) sendFullWebState();
    else sendSystemStatus();
  } else if (normalized.equals("disconnect")) {
    disconnectProtocolClient(sourceName);
    sendSystemStatus();
  }
}

void sendFrameImage(PImage img) {
  if (!webClientConnected || wsServer == null || img == null || !isWebVisualStreamRequested()) return;

  String base64 = encodePImageJpegBase64(img, wsStreamJpegWidth, wsStreamJpegHeight, 700000);
  if (base64.length() == 0) return;

  JSONObject frameData = new JSONObject();
  frameData.setString("type", "frame");
  frameData.setString("frame", base64);
  sendWebSocketJson(frameData);
  lastWebSocketFrameSentMillis = millis();
}
