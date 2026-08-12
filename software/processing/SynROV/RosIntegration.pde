// =====================================================================
// SynROV Processing - ROS 2 integration through rosbridge
// ---------------------------------------------------------------------
// ROS is a transport adapter, not a second robot-control protocol. Commands on
// /synrov/control carry the exact same SynROV envelope used by Web and AiBot.
// Native joint_states remains available as an outbound ROS interoperability
// view, but all SynROV command/state JSON uses the shared protocol.
// =====================================================================

WebsocketClient rosBridgeClient;
ArrayDeque<String> pendingRosMessages = new ArrayDeque<String>();
final int ROS_MESSAGE_QUEUE_MAX = 128;
final int ROS_MESSAGES_PER_FRAME = 12;

boolean rosEnabled = true;
boolean rosAcceptCommands = true;
volatile boolean rosBridgeConnected = false;
volatile boolean rosConnectInProgress = false;
String rosEndpoint = "ws://127.0.0.1:9090";
String rosNamespace = "/synrov";
int rosPublishIntervalMs = 100;
int rosReconnectIntervalMs = 3000;
int rosCommandTimeoutMs = 1000;
long rosLastPublishMillis = 0;
long rosLastHeartbeatPublishMillis = 0;
long rosLastHeartbeatRxMillis = 0;
long rosLastConnectAttemptMillis = 0;
long rosLastControlMillis = 0;

// Callback required by the Processing Websockets library for WebsocketClient.
// Parsing and robot control remain on the Processing runtime thread.
void webSocketEvent(String message) {
  queueRosMessage(message);
}

String sanitizeRosNamespace(String value) {
  String ns = trim(value == null ? "" : value);
  if (ns.length() == 0 || ns.equals("/")) return "/synrov";
  if (!ns.startsWith("/")) ns = "/" + ns;
  while (ns.length() > 1 && ns.endsWith("/")) ns = ns.substring(0, ns.length() - 1);
  return ns;
}

String rosTopic(String leaf) {
  String clean = trim(leaf == null ? "" : leaf);
  while (clean.startsWith("/")) clean = clean.substring(1);
  return sanitizeRosNamespace(rosNamespace) + "/" + clean;
}

String controlSourceName(int source) {
  if (source == CONTROL_SOURCE_LOCAL) return "local";
  if (source == CONTROL_SOURCE_WEB) return "web";
  if (source == CONTROL_SOURCE_AIBOT) return "aibot";
  if (source == CONTROL_SOURCE_ROS) return "ros";
  return "none";
}

void initializeRosIntegration() {
  rosNamespace = sanitizeRosNamespace(rosNamespace);
  if (rosEnabled) requestRosConnection();
}

boolean rosEndpointReachable() {
  try {
    java.net.URI uri = new java.net.URI(rosEndpoint);
    String host = uri.getHost();
    int port = uri.getPort();
    if (host == null || host.length() == 0) return false;
    if (port <= 0) port = uri.getScheme() != null && uri.getScheme().equalsIgnoreCase("wss") ? 443 : 80;
    java.net.Socket socket = new java.net.Socket();
    try {
      socket.connect(new java.net.InetSocketAddress(host, port), 500);
      return true;
    }
    finally {
      try { socket.close(); } catch (Exception ignore) {}
    }
  }
  catch (Exception e) {
    return false;
  }
}

void requestRosConnection() {
  if (!rosEnabled || rosBridgeConnected || rosConnectInProgress) return;
  rosLastConnectAttemptMillis = millis();
  rosConnectInProgress = true;

  Thread connector = new Thread(new Runnable() {
    public void run() {
      try {
        if (!rosEndpointReachable()) return;
        WebsocketClient client = new WebsocketClient(sketchApplet, rosEndpoint);
        rosBridgeClient = client;
        rosBridgeConnected = true;
        rosLastHeartbeatRxMillis = millis();
        advertiseAndSubscribeRosTopics();
      }
      catch (Throwable error) {
        rosBridgeClient = null;
        rosBridgeConnected = false;
      }
      finally {
        rosConnectInProgress = false;
      }
    }
  }, "SynROV-ROS");
  connector.setDaemon(true);
  connector.start();
}

void markRosDisconnected() {
  rosBridgeConnected = false;
  rosBridgeClient = null;
  if (currentControlOwner == CONTROL_SOURCE_ROS) currentControlOwner = CONTROL_SOURCE_NONE;
}

void queueRosMessage(String message) {
  if (message == null || message.length() == 0) return;
  synchronized (pendingRosMessages) {
    while (pendingRosMessages.size() >= ROS_MESSAGE_QUEUE_MAX) pendingRosMessages.pollFirst();
    pendingRosMessages.addLast(message);
  }
}

String pollRosMessage() {
  synchronized (pendingRosMessages) {
    return pendingRosMessages.pollFirst();
  }
}

void rosSend(JSONObject packet) {
  if (!rosBridgeConnected || rosBridgeClient == null || packet == null) return;
  try {
    rosBridgeClient.sendMessage(packet.toString());
  }
  catch (Throwable error) {
    markRosDisconnected();
  }
}

void rosAdvertise(String topic, String type) {
  JSONObject packet = new JSONObject();
  packet.setString("op", "advertise");
  packet.setString("topic", topic);
  packet.setString("type", type);
  rosSend(packet);
}

void rosSubscribe(String topic, String type) {
  JSONObject packet = new JSONObject();
  packet.setString("op", "subscribe");
  packet.setString("topic", topic);
  packet.setString("type", type);
  rosSend(packet);
}

void advertiseAndSubscribeRosTopics() {
  rosAdvertise(rosTopic("state"), "std_msgs/msg/String");
  rosAdvertise(rosTopic("telemetry"), "std_msgs/msg/String");
  rosAdvertise(rosTopic("joint_states"), "sensor_msgs/msg/JointState");
  rosAdvertise(rosTopic("bridge_heartbeat"), "std_msgs/msg/String");

  rosSubscribe(rosTopic("bridge_heartbeat"), "std_msgs/msg/String");
  if (rosAcceptCommands) rosSubscribe(rosTopic("control"), "std_msgs/msg/String");
}

void rosPublishMessage(String topic, JSONObject msg) {
  JSONObject packet = new JSONObject();
  packet.setString("op", "publish");
  packet.setString("topic", topic);
  packet.setJSONObject("msg", msg);
  rosSend(packet);
}

void rosPublishString(String topic, String data) {
  JSONObject msg = new JSONObject();
  msg.setString("data", data == null ? "" : data);
  rosPublishMessage(topic, msg);
}

JSONObject rosTime() {
  long nowMs = System.currentTimeMillis();
  JSONObject stamp = new JSONObject();
  stamp.setInt("sec", (int)(nowMs / 1000L));
  stamp.setInt("nanosec", (int)((nowMs % 1000L) * 1000000L));
  return stamp;
}

JSONArray rosJointNames() {
  JSONArray names = new JSONArray();
  String[] jointNames = {
    "base_yaw", "upper_arm", "forearm", "forearm_roll",
    "wrist_pitch", "wrist_roll", "gripper"
  };
  for (int i = 0; i < jointNames.length; i++) names.setString(i, jointNames[i]);
  return names;
}

JSONArray rosManipulatorPositionRad() {
  JSONArray pos = new JSONArray();
  for (int i = 0; i <= GRIPPER_IDX; i++) {
    float memberDeg = armUiServoToMemberDeg(i, getManipulatorVisualServoAngle(i));
    float value = i == GRIPPER_IDX ? constrain(memberDeg / 100.0f, 0.0f, 1.0f) : radians(memberDeg);
    pos.setFloat(i, value);
  }
  return pos;
}

void publishRosJointState() {
  JSONObject msg = new JSONObject();
  JSONObject header = new JSONObject();
  header.setJSONObject("stamp", rosTime());
  header.setString("frame_id", "synrov_base");
  msg.setJSONObject("header", header);
  msg.setJSONArray("name", rosJointNames());
  msg.setJSONArray("position", rosManipulatorPositionRad());
  msg.setJSONArray("velocity", new JSONArray());
  msg.setJSONArray("effort", new JSONArray());
  rosPublishMessage(rosTopic("joint_states"), msg);
}

void publishRosRuntime() {
  if (!rosBridgeConnected) return;
  syncLatestSensorsForActiveTelemetrySource();
  JSONObject sensors = latestSensors == null ? new JSONObject() : latestSensors;

  JSONObject statePayload = buildUnifiedRuntimeSnapshot(currentModeName(), sensors);
  JSONObject stateMessage = buildSynRovMessage(SYNROV_MESSAGE_STATE, SYNROV_SOURCE_PROCESSING, statePayload);
  rosPublishString(rosTopic("state"), stateMessage.toString());

  JSONObject telemetryPayload = new JSONObject();
  telemetryPayload.setString("robot", currentModeName());
  telemetryPayload.setInt("runtimeTimestampMs", millis());
  telemetryPayload.setJSONObject("sensors", buildAiAugmentedSensorsForRobot(currentModeName(), sensors));
  JSONObject telemetryMessage = buildSynRovMessage(SYNROV_MESSAGE_TELEMETRY, SYNROV_SOURCE_PROCESSING, telemetryPayload);
  rosPublishString(rosTopic("telemetry"), telemetryMessage.toString());

  publishRosJointState();
}

JSONObject parseRosStringMessage(JSONObject rosMsg) {
  if (rosMsg == null) return null;
  String data = getJsonString(rosMsg, "data", "");
  if (data.length() == 0) return null;
  return safeParseJsonObject(data);
}

void handleRosControlMessage(JSONObject rosMsg) {
  if (!rosAcceptCommands) return;
  JSONObject message = parseRosStringMessage(rosMsg);
  if (!isSynRovMessage(message)) return;
  if (!SYNROV_SOURCE_ROS.equals(normalizeProtocolSource(getJsonString(message, "source", "")))) return;
  if (!isSynRovExternalCommandType(getJsonString(message, "messageType", ""))) return;
  rosLastControlMillis = millis();
  dispatchSynRovCommand(message);
}

void processRosBridgePacket(String raw) {
  JSONObject packet = safeParseJsonObject(raw);
  if (packet == null) return;
  if (!"publish".equals(getJsonString(packet, "op", ""))) return;

  String topic = getJsonString(packet, "topic", "");
  JSONObject msg = getJsonObjectSafe(packet, "msg");
  if (msg == null) return;

  if (topic.equals(rosTopic("bridge_heartbeat"))) {
    rosLastHeartbeatRxMillis = millis();
    return;
  }

  if (topic.equals(rosTopic("control"))) handleRosControlMessage(msg);
}

void processPendingRosMessages() {
  int processed = 0;
  while (processed < ROS_MESSAGES_PER_FRAME) {
    String raw = pollRosMessage();
    if (raw == null) return;
    processRosBridgePacket(raw);
    processed++;
  }
}

void updateRosIntegration() {
  if (!rosEnabled) {
    if (rosBridgeConnected) markRosDisconnected();
    return;
  }

  long now = millis();
  if (!rosBridgeConnected && !rosConnectInProgress && (now - rosLastConnectAttemptMillis) >= rosReconnectIntervalMs) {
    requestRosConnection();
  }
  if (!rosBridgeConnected) return;

  processPendingRosMessages();

  if ((now - rosLastHeartbeatPublishMillis) >= 1000) {
    JSONObject payload = new JSONObject();
    payload.setString("status", "alive");
    JSONObject heartbeat = buildSynRovMessage(SYNROV_MESSAGE_HEARTBEAT, SYNROV_SOURCE_PROCESSING, payload);
    rosPublishString(rosTopic("bridge_heartbeat"), heartbeat.toString());
    rosLastHeartbeatPublishMillis = now;
  }

  if (rosLastHeartbeatRxMillis > 0 && (now - rosLastHeartbeatRxMillis) > max(5000, rosReconnectIntervalMs * 2)) {
    markRosDisconnected();
    return;
  }

  if ((now - rosLastPublishMillis) >= rosPublishIntervalMs) {
    publishRosRuntime();
    rosLastPublishMillis = now;
  }
}
