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
int rosCommandTimeoutMs = 2500;
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

void requestRosConnection() {
  if (!rosEnabled || rosBridgeConnected || rosConnectInProgress) return;
  rosLastConnectAttemptMillis = millis();
  rosConnectInProgress = true;

  Thread connector = new Thread(new Runnable() {
    public void run() {
      try {
        WebsocketClient client = new WebsocketClient(sketchApplet, rosEndpoint);
        rosBridgeClient = client;
        rosBridgeConnected = true;
        rosLastHeartbeatRxMillis = millis();
        advertiseAndSubscribeRosTopics();
      }
      catch (Exception error) {
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
  catch (Exception error) {
    markRosDisconnected();
  }
}

JSONObject rosRealtimeQos(int depth) {
  JSONObject qos = new JSONObject();
  qos.setString("history", "keep_last");
  qos.setInt("depth", max(1, depth));
  qos.setString("reliability", "reliable");
  qos.setString("durability", "volatile");
  return qos;
}

String rosOperationId(String direction, String leaf) {
  String clean = trim(leaf == null ? "" : leaf).replace('/', '-');
  while (clean.startsWith("-")) clean = clean.substring(1);
  return "synrov-" + direction + "-" + clean;
}

void rosAdvertise(String leaf, String type, int depth) {
  JSONObject packet = new JSONObject();
  packet.setString("op", "advertise");
  packet.setString("id", rosOperationId("pub", leaf));
  packet.setString("topic", rosTopic(leaf));
  packet.setString("type", type);
  packet.setJSONObject("qos", rosRealtimeQos(depth));
  rosSend(packet);
}

void rosSubscribe(String leaf, String type, int depth) {
  JSONObject packet = new JSONObject();
  packet.setString("op", "subscribe");
  packet.setString("id", rosOperationId("sub", leaf));
  packet.setString("topic", rosTopic(leaf));
  packet.setString("type", type);
  packet.setJSONObject("qos", rosRealtimeQos(depth));
  rosSend(packet);
}

void advertiseAndSubscribeRosTopics() {
  rosAdvertise("state", "std_msgs/msg/String", 10);
  rosAdvertise("telemetry", "std_msgs/msg/String", 10);
  rosAdvertise("joint_states", "sensor_msgs/msg/JointState", 10);
  rosAdvertise("bridge_heartbeat", "std_msgs/msg/String", 1);

  rosSubscribe("bridge_heartbeat", "std_msgs/msg/String", 1);
  if (rosAcceptCommands) rosSubscribe("control", "std_msgs/msg/String", 10);
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
    "wrist_pitch", "wrist_roll", "gripper_finger"
  };
  for (int i = 0; i < jointNames.length; i++) names.setString(i, jointNames[i]);
  return names;
}

JSONArray rosManipulatorPositionRad() {
  JSONArray pos = new JSONArray();
  for (int i = 0; i <= GRIPPER_IDX; i++) {
    float value;
    if (i == GRIPPER_IDX) {
      value = radians(getGripperFingerModelZDeg());
    } else {
      value = radians(armUiServoToMemberDeg(i, getManipulatorVisualServoAngle(i)));
    }
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
