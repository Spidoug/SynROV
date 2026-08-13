// =====================================================================
// SynROV Processing - Shared application communication protocol
// ---------------------------------------------------------------------
// Transport is deliberately separate from protocol. Browser/AiBot use the
// control WebSocket, ROS uses rosbridge, and high-bandwidth streams use their
// dedicated sockets. Every SynROV application message uses this same envelope.
// =====================================================================

final String SYNROV_PROTOCOL_NAME = "synrov";
final int SYNROV_SOFTWARE_VERSION = 1;

final String SYNROV_SOURCE_PROCESSING = "processing";
final String SYNROV_SOURCE_WEB = "web";
final String SYNROV_SOURCE_AIBOT = "aibot";
final String SYNROV_SOURCE_ROS = "ros";

final String SYNROV_MESSAGE_CLIENT = "client";
final String SYNROV_MESSAGE_SYNC = "sync";
final String SYNROV_MESSAGE_CONTROL = "control";
final String SYNROV_MESSAGE_CAMERA = "camera";
final String SYNROV_MESSAGE_MODE = "mode";
final String SYNROV_MESSAGE_JOYSTICK = "joystick";
final String SYNROV_MESSAGE_TOGGLE = "toggle";
final String SYNROV_MESSAGE_ACTION = "action";
final String SYNROV_MESSAGE_CONNECT = "connect";
final String SYNROV_MESSAGE_STATE = "state";
final String SYNROV_MESSAGE_TELEMETRY = "telemetry";
final String SYNROV_MESSAGE_PERCEPTION_SUBSCRIPTION = "perception.subscription";
final String SYNROV_MESSAGE_PERCEPTION_FRAME = "perception.frame";
final String SYNROV_MESSAGE_ROBOT_CAMERA_CONTROL = "robot_camera.control";
final String SYNROV_MESSAGE_ROBOT_CAMERA_FRAME = "robot_camera.frame";
final String SYNROV_MESSAGE_HEARTBEAT = "heartbeat";

long synRovMessageSequence = 0;
int activeProtocolControlSource = CONTROL_SOURCE_WEB;
String activeProtocolSourceName = SYNROV_SOURCE_WEB;

String normalizeProtocolSource(String source) {
  String normalized = trim(source == null ? "" : source).toLowerCase();
  if (normalized.equals(SYNROV_SOURCE_ROS)) return SYNROV_SOURCE_ROS;
  if (normalized.equals(SYNROV_SOURCE_AIBOT)) return SYNROV_SOURCE_AIBOT;
  if (normalized.equals(SYNROV_SOURCE_WEB)) return SYNROV_SOURCE_WEB;
  if (normalized.equals(SYNROV_SOURCE_PROCESSING)) return SYNROV_SOURCE_PROCESSING;
  return normalized;
}

boolean isKnownProtocolSource(String source) {
  String normalized = normalizeProtocolSource(source);
  return normalized.equals(SYNROV_SOURCE_PROCESSING) ||
    normalized.equals(SYNROV_SOURCE_WEB) ||
    normalized.equals(SYNROV_SOURCE_AIBOT) ||
    normalized.equals(SYNROV_SOURCE_ROS);
}

JSONObject buildSynRovMessage(String messageType, String source, JSONObject payload) {
  JSONObject message = new JSONObject();
  message.setString("protocol", SYNROV_PROTOCOL_NAME);
  message.setInt("softwareVersion", SYNROV_SOFTWARE_VERSION);
  message.setString("messageType", messageType == null ? "" : trim(messageType));
  message.setString("source", normalizeProtocolSource(source));
  message.setLong("timestampMs", System.currentTimeMillis());
  message.setLong("seq", ++synRovMessageSequence);
  message.setJSONObject("payload", payload == null ? new JSONObject() : payload);
  return message;
}

boolean isSynRovMessage(JSONObject message) {
  if (message == null) return false;
  if (!SYNROV_PROTOCOL_NAME.equals(getJsonString(message, "protocol", ""))) return false;
  if (getJsonInt(message, "softwareVersion", -1) != SYNROV_SOFTWARE_VERSION) return false;
  if (trim(getJsonString(message, "messageType", "")).length() == 0) return false;
  String source = getJsonString(message, "source", "");
  if (!isKnownProtocolSource(source)) return false;
  if (!jsonHasValue(message, "timestampMs") || !jsonHasValue(message, "seq")) return false;
  Object timestamp = getJsonValueSafe(message, "timestampMs");
  Object sequence = getJsonValueSafe(message, "seq");
  if (!(timestamp instanceof Number) || !(sequence instanceof Number)) return false;
  return getJsonObjectSafe(message, "payload") != null;
}

boolean isSynRovExternalCommandType(String messageType) {
  if (messageType == null) return false;
  return messageType.equals(SYNROV_MESSAGE_SYNC) ||
    messageType.equals(SYNROV_MESSAGE_CONTROL) ||
    messageType.equals(SYNROV_MESSAGE_CAMERA) ||
    messageType.equals(SYNROV_MESSAGE_MODE) ||
    messageType.equals(SYNROV_MESSAGE_JOYSTICK) ||
    messageType.equals(SYNROV_MESSAGE_TOGGLE) ||
    messageType.equals(SYNROV_MESSAGE_ACTION) ||
    messageType.equals(SYNROV_MESSAGE_CONNECT);
}

boolean isSynRovClientLifecycleAction(String action) {
  String normalized = trim(action == null ? "" : action).toLowerCase();
  return normalized.equals("connect") || normalized.equals("ping") || normalized.equals("disconnect");
}

boolean synRovCommandPayloadMatchesType(String messageType, JSONObject payload) {
  if (messageType == null || payload == null) return false;
  if (messageType.equals(SYNROV_MESSAGE_CLIENT)) return isSynRovClientLifecycleAction(getJsonString(payload, "client", ""));
  if (messageType.equals(SYNROV_MESSAGE_SYNC)) return payload.hasKey("sync") && getJsonBoolean(payload, "sync", false);
  if (messageType.equals(SYNROV_MESSAGE_CONTROL)) return getJsonObjectSafe(payload, "control") != null;
  if (messageType.equals(SYNROV_MESSAGE_CAMERA)) return getJsonObjectSafe(payload, "camera") != null;
  if (messageType.equals(SYNROV_MESSAGE_MODE)) return trim(getJsonString(payload, "mode", "")).length() > 0;
  if (messageType.equals(SYNROV_MESSAGE_JOYSTICK)) return payload.hasKey("joystickType");
  if (messageType.equals(SYNROV_MESSAGE_TOGGLE)) return trim(getJsonString(payload, "toggle", "")).length() > 0;
  if (messageType.equals(SYNROV_MESSAGE_ACTION)) return payload.hasKey("action");
  if (messageType.equals(SYNROV_MESSAGE_CONNECT)) return payload.hasKey("connect") && getJsonBoolean(payload, "connect", false);
  return false;
}

void replyRuntimeToCurrentProtocolSource() {
  if (activeProtocolSourceName.equals(SYNROV_SOURCE_ROS)) publishRosRuntime();
  else sendFullWebState();
}

boolean isSynRovMessageType(JSONObject message, String expectedType) {
  return isSynRovMessage(message) && expectedType != null &&
    expectedType.equals(getJsonString(message, "messageType", ""));
}

JSONObject synRovMessagePayload(JSONObject message) {
  JSONObject payload = getJsonObjectSafe(message, "payload");
  return payload == null ? new JSONObject() : payload;
}

int protocolControlSourceForMessage(JSONObject message) {
  String source = normalizeProtocolSource(getJsonString(message, "source", ""));
  if (source.equals(SYNROV_SOURCE_ROS)) return CONTROL_SOURCE_ROS;
  if (source.equals(SYNROV_SOURCE_AIBOT)) return CONTROL_SOURCE_AIBOT;
  return CONTROL_SOURCE_WEB;
}

int currentProtocolControlSource() {
  return activeProtocolControlSource;
}

boolean isRemoteControlSource(int source) {
  return source == CONTROL_SOURCE_WEB || source == CONTROL_SOURCE_AIBOT || source == CONTROL_SOURCE_ROS;
}

void withCommandSourceContext(int source, Runnable action) {
  if (action == null) return;
  int previous = commandContextSource;
  setCommandContext(source);
  try {
    action.run();
  }
  finally {
    commandContextSource = previous;
  }
}

void withProtocolCommandContext(Runnable action) {
  withCommandSourceContext(currentProtocolControlSource(), action);
}

void dispatchSynRovCommand(JSONObject message) {
  if (!isSynRovMessage(message)) {
    updateMessage(tr("Invalid or incompatible SynROV message."));
    return;
  }

  JSONObject command = synRovMessagePayload(message);
  String sourceName = normalizeProtocolSource(getJsonString(message, "source", ""));
  String messageType = getJsonString(message, "messageType", "");
  if (sourceName.equals(SYNROV_SOURCE_PROCESSING)) {
    updateMessage(tr("Processing source cannot issue external commands."));
    return;
  }

  boolean clientMessage = messageType.equals(SYNROV_MESSAGE_CLIENT);
  boolean externalCommand = isSynRovExternalCommandType(messageType);
  if ((!clientMessage && !externalCommand) || !synRovCommandPayloadMatchesType(messageType, command)) {
    updateMessage(tr("SynROV message rejected: incompatible type/payload: ") + messageType);
    return;
  }
  if (clientMessage && sourceName.equals(SYNROV_SOURCE_ROS)) {
    updateMessage(tr("ROS does not use client lifecycle messages."));
    return;
  }

  int previousSource = activeProtocolControlSource;
  String previousSourceName = activeProtocolSourceName;
  activeProtocolControlSource = protocolControlSourceForMessage(message);
  activeProtocolSourceName = sourceName;

  // Existing command handlers stay focused on robot behavior. Transport and
  // source metadata are normalized here before any ownership/safety decision.
  command.setString("source", sourceName);
  command.setString("origin", sourceName);

  try {
    if (!sourceName.equals(SYNROV_SOURCE_ROS)) markProtocolHeartbeat(sourceName);
    markRemoteControlOrigin(command);

    if (clientMessage) {
      handleClientCommand(command);
    } else if (externalCommand) {
      if (messageType.equals(SYNROV_MESSAGE_SYNC)) replyRuntimeToCurrentProtocolSource();
      else if (messageType.equals(SYNROV_MESSAGE_CONTROL)) handleUnifiedControlCommand(command);
      else if (messageType.equals(SYNROV_MESSAGE_CAMERA)) handleCameraCommand(command);
      else if (messageType.equals(SYNROV_MESSAGE_MODE)) handleModeCommand(command);
      else if (messageType.equals(SYNROV_MESSAGE_JOYSTICK)) handleJoystickTypeCommand(command);
      else if (messageType.equals(SYNROV_MESSAGE_TOGGLE)) handleToggleCommand(command);
      else if (messageType.equals(SYNROV_MESSAGE_ACTION)) handleActionCommand(command);
      else if (messageType.equals(SYNROV_MESSAGE_CONNECT)) handleHardwareConnectCommand();
    } else {
      updateMessage(tr("Unknown SynROV message type: ") + messageType);
    }
  }
  finally {
    activeProtocolControlSource = previousSource;
    activeProtocolSourceName = previousSourceName;
  }
}
