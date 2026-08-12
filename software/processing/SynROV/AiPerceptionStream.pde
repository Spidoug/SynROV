// =====================================================================
// SynROV Processing - Dedicated AiBot perception stream
// ---------------------------------------------------------------------
// Purpose:
//   A second WebSocket channel dedicated to AI perception. Port 9000 remains
//   responsible for control/state. Port 9001 publishes a temporally coherent
//   package containing the clean 3D viewport, robot pose, world transform and
//   nearby collision/sensor samples. This avoids Base64 image traffic sharing
//   the command/telemetry transport.
// =====================================================================

WebsocketServer aiPerceptionServer;
final int AI_PERCEPTION_PORT = 9001;
final String AI_PERCEPTION_PATH = "/";
final int AI_PERCEPTION_INTERVAL_MS = 160;
final int AI_PERCEPTION_CLIENT_TIMEOUT_MS = 3200;
final int AI_PERCEPTION_JPEG_W = 480;
final int AI_PERCEPTION_JPEG_H = 270;
final int AI_PERCEPTION_MAX_BASE64_CHARS = 900000;
final int AI_PERCEPTION_MAX_NEAREST_OBSTACLES = 24;
final float AI_PERCEPTION_NEARBY_RADIUS_M_MOBILE = 120.0f;
final float AI_PERCEPTION_NEARBY_RADIUS_M_MANIPULATOR = 20.0f;

volatile long aiPerceptionHeartbeatMillis = 0;
volatile boolean aiPerceptionSubscribed = false;
long lastAiPerceptionFrameSentMillis = 0;
boolean aiPerceptionSendWarningLatched = false;

final int AI_PERCEPTION_OBSTACLE_CACHE_MS = 280;
final float AI_PERCEPTION_OBSTACLE_CACHE_MOVE_M = 0.35f;
final float AI_PERCEPTION_OBSTACLE_CACHE_YAW_DEG = 3.0f;
JSONArray aiPerceptionObstacleCache = new JSONArray();
long aiPerceptionObstacleCacheMillis = 0;
int aiPerceptionObstacleCacheRobotMode = -1;
PVector aiPerceptionObstacleCacheRobotWorld = new PVector();
float aiPerceptionObstacleCacheYawRad = 0.0f;

class AiObstacleCandidate {
  PVector world;
  float distanceScene;
  String source;

  AiObstacleCandidate(PVector world, float distanceScene, String source) {
    this.world = world.copy();
    this.distanceScene = distanceScene;
    this.source = source;
  }
}

void initializeAiPerceptionServer() {
  try {
    aiPerceptionServer = new WebsocketServer(this, AI_PERCEPTION_PORT, AI_PERCEPTION_PATH);
    println("[SynROV][AiPerception] stream online on port " + AI_PERCEPTION_PORT);
  }
  catch (Exception e) {
    aiPerceptionServer = null;
    println("[SynROV][AiPerception] stream failed on port " + AI_PERCEPTION_PORT + ": " + e.getMessage());
  }
}

boolean isAiPerceptionClientAlive() {
  if (!aiPerceptionSubscribed || aiPerceptionServer == null) return false;
  long age = millis() - aiPerceptionHeartbeatMillis;
  if (age > AI_PERCEPTION_CLIENT_TIMEOUT_MS) {
    aiPerceptionSubscribed = false;
    return false;
  }
  return true;
}

// Handles perception subscriptions using the same application envelope used by
// every producer. The dedicated socket remains only a transport separation.
boolean handleAiPerceptionSocketEvent(String raw) {
  if (raw == null) return false;
  String normalized = trim(raw);
  if (normalized.length() == 0) return false;

  JSONObject message = safeParseJsonObject(normalized);
  if (!isSynRovMessageType(message, SYNROV_MESSAGE_PERCEPTION_SUBSCRIPTION)) return false;
  if (!SYNROV_SOURCE_AIBOT.equals(normalizeProtocolSource(getJsonString(message, "source", "")))) return true;

  JSONObject payload = synRovMessagePayload(message);
  String subscriptionAction = getJsonString(payload, "action", "").toLowerCase();
  String streamName = getJsonString(payload, "stream", "ai_perception").toLowerCase();
  if (!streamName.equals("ai_perception")) return true;

  if (subscriptionAction.equals("disconnect") || subscriptionAction.equals("unsubscribe")) {
    aiPerceptionSubscribed = false;
    return true;
  }

  if (!(subscriptionAction.equals("subscribe") || subscriptionAction.equals("ping"))) return true;
  aiPerceptionSubscribed = true;
  aiPerceptionHeartbeatMillis = millis();
  return true;
}

boolean shouldSendAiPerceptionFrameNow() {
  if (!isAiPerceptionClientAlive()) return false;
  return (millis() - lastAiPerceptionFrameSentMillis) >= AI_PERCEPTION_INTERVAL_MS;
}

PVector aiPerceptionRobotWorldPositionScene(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return new PVector(vehicleNavX, GROUND_Y, vehicleNavZ);
  if (robotMode == ROBOT_MODE_DRONE) return new PVector(droneNavX, getDroneSceneY() + droneY, droneNavZ);
  return new PVector(0, GROUND_Y, 0);
}

float aiPerceptionRobotYawRad(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleNavYaw;
  if (robotMode == ROBOT_MODE_DRONE) return droneNavYaw;
  return radians(getBaseModelYawDeg());
}

JSONObject buildAiRobotPoseSnapshot(int robotMode) {
  JSONObject pose = new JSONObject();
  float scenePerMeter = max(0.0001f, sceneUnitsPerMeterForRobotMode(robotMode));

  if (robotMode == ROBOT_MODE_VEHICLE) {
    pose.setFloat("x_m", vehicleNavX / scenePerMeter);
    pose.setFloat("y_m", 0.0f);
    pose.setFloat("z_m", vehicleNavZ / scenePerMeter);
    pose.setFloat("yaw_deg", degrees(vehicleNavYaw));
    pose.setFloat("pitch_deg", vehicleBaseAttitudePitchDeg + vehicleVisualPitchDeg);
    pose.setFloat("roll_deg", vehicleBaseAttitudeRollDeg + vehicleVisualRollDeg);
    pose.setFloat("linear_speed_scene_s", vehicleLinearSpeed);
    pose.setFloat("angular_speed", vehicleAngularSpeed);
  } else if (robotMode == ROBOT_MODE_DRONE) {
    pose.setFloat("x_m", droneNavX / scenePerMeter);
    pose.setFloat("y_m", max(0.0f, droneAltitudeCm) / 100.0f);
    pose.setFloat("z_m", droneNavZ / scenePerMeter);
    pose.setFloat("yaw_deg", degrees(droneNavYaw));
    pose.setFloat("pitch_deg", degrees(dronePitch));
    pose.setFloat("roll_deg", degrees(droneRoll));
    pose.setFloat("altitude_m", max(0.0f, droneAltitudeCm) / 100.0f);
  } else {
    JSONArray joints = new JSONArray();
    for (int i = 0; i <= GRIPPER_IDX; i++) joints.append(round(getManipulatorVisualServoAngle(i)));
    pose.setJSONArray("joint_deg", joints);
    pose.setFloat("base_yaw_deg", getBasePoseYawDeg());
    pose.setFloat("base_pitch_deg", manipulatorBaseAttitudePitchDeg);
    pose.setFloat("base_roll_deg", manipulatorBaseAttitudeRollDeg);
    pose.setString("attitude_source", robotUsesFirmwareAttitude(ROBOT_MODE_MANIPULATOR) ? "firmware_imu" : "local_model");
  }
  return pose;
}

JSONObject buildAiWorldSnapshot(int robotMode) {
  JSONObject world = new JSONObject();
  boolean imported = hasImportedWorldForRobotMode(robotMode);
  boolean demo = shouldUseDemoWorldForRobotMode(robotMode);
  String type = imported ? "imported" : (demo ? "demo" : "scan");
  world.setString("type", type);
  world.setString("source", worldSourceLabelForRobotMode(robotMode));
  world.setBoolean("imported", imported);
  world.setBoolean("demo", demo);
  world.setBoolean("collision_enabled", environmentCollisionActiveForRobotMode(robotMode));
  world.setFloat("operating_radius_m", operatingAreaRadiusMetersForRobotMode(robotMode));
  world.setFloat("max_altitude_m", robotMode == ROBOT_MODE_DRONE ? DRONE_OPERATING_MAX_ALTITUDE_M : 0.0f);
  world.setInt("scan_points", environmentPointCountForRobotMode(robotMode));
  world.setInt("imported_collision_points", importedWorldPointListForRobotMode(robotMode).size());
  world.setInt("imported_geometry_count", importedWorldGeometryCountForRobotMode(robotMode));

  JSONObject transform = new JSONObject();
  transform.setFloat("scale", worldUserScaleForRobotMode(robotMode));
  transform.setFloat("pitch_deg", worldUserPitchDegForRobotMode(robotMode));
  transform.setFloat("yaw_deg", worldUserRotationDegForRobotMode(robotMode));
  transform.setFloat("roll_deg", worldUserRollDegForRobotMode(robotMode));
  PVector offset = worldUserOffsetForRobotMode(robotMode);
  float scenePerMeter = max(0.0001f, sceneUnitsPerMeterForRobotMode(robotMode));
  transform.setFloat("offset_x_m", offset.x / scenePerMeter);
  transform.setFloat("offset_y_m", offset.y / scenePerMeter);
  transform.setFloat("offset_z_m", offset.z / scenePerMeter);
  world.setJSONObject("transform", transform);

  JSONObject compass = new JSONObject();
  compass.setBoolean("locked", worldCompassNorthLockedForRobotMode(robotMode));
  compass.setFloat("north_offset_deg", worldCompassNorthOffsetDegForRobotMode(robotMode));
  world.setJSONObject("compass", compass);
  return world;
}

JSONObject buildAiCameraSnapshot(int robotMode) {
  JSONObject camera = new JSONObject();
  camera.setFloat("orbit_x_rad", cameraRotationX);
  camera.setFloat("orbit_y_rad", cameraRotationY);
  camera.setFloat("zoom", zoomLevel);
  camera.setFloat("min_zoom", sceneMinZoomForRobotMode(robotMode));
  camera.setFloat("far_clip", sceneFarClipForRobotMode(robotMode));
  camera.setInt("viewport_width", max(1, width - int(leftSidebarWidth())));
  camera.setInt("viewport_height", max(1, height));
  return camera;
}

void aiInsertObstacleCandidate(ArrayList<AiObstacleCandidate> nearest, AiObstacleCandidate candidate) {
  if (candidate == null) return;
  int insertAt = nearest.size();
  for (int i = 0; i < nearest.size(); i++) {
    if (candidate.distanceScene < nearest.get(i).distanceScene) {
      insertAt = i;
      break;
    }
  }
  nearest.add(insertAt, candidate);
  while (nearest.size() > AI_PERCEPTION_MAX_NEAREST_OBSTACLES) nearest.remove(nearest.size() - 1);
}

void aiCollectObstacleCandidates(
  ArrayList<AiObstacleCandidate> nearest,
  ArrayList<PVector> points,
  String source,
  PVector robotWorld,
  float maxDistanceScene
) {
  if (points == null || points.isEmpty()) return;
  float maxDistSq = maxDistanceScene * maxDistanceScene;
  int stride = max(1, points.size() / 5000);
  for (int i = 0; i < points.size(); i += stride) {
    PVector p = points.get(i);
    if (p == null) continue;
    float dx = p.x - robotWorld.x;
    float dy = p.y - robotWorld.y;
    float dz = p.z - robotWorld.z;
    float d2 = dx * dx + dy * dy + dz * dz;
    if (d2 > maxDistSq) continue;
    aiInsertObstacleCandidate(nearest, new AiObstacleCandidate(p, sqrt(d2), source));
  }
}

boolean aiPerceptionObstacleCacheReusable(int robotMode, PVector robotWorld, float yawRad, float scenePerMeter) {
  if (aiPerceptionObstacleCacheRobotMode != robotMode) return false;
  if ((millis() - aiPerceptionObstacleCacheMillis) > AI_PERCEPTION_OBSTACLE_CACHE_MS) return false;
  float moveThresholdScene = AI_PERCEPTION_OBSTACLE_CACHE_MOVE_M * max(0.0001f, scenePerMeter);
  if (PVector.dist(robotWorld, aiPerceptionObstacleCacheRobotWorld) > moveThresholdScene) return false;
  float yawDeltaDeg = abs(normalizeSignedAngleDeg(degrees(yawRad - aiPerceptionObstacleCacheYawRad)));
  return yawDeltaDeg <= AI_PERCEPTION_OBSTACLE_CACHE_YAW_DEG;
}

JSONArray buildAiNearestObstacleSnapshot(int robotMode) {
  PVector robotWorld = aiPerceptionRobotWorldPositionScene(robotMode);
  float yaw = aiPerceptionRobotYawRad(robotMode);
  float scenePerMeter = max(0.0001f, sceneUnitsPerMeterForRobotMode(robotMode));
  if (aiPerceptionObstacleCacheReusable(robotMode, robotWorld, yaw, scenePerMeter)) {
    return aiPerceptionObstacleCache;
  }

  ArrayList<AiObstacleCandidate> nearest = new ArrayList<AiObstacleCandidate>();
  float nearbyRadiusM = robotMode == ROBOT_MODE_MANIPULATOR
    ? AI_PERCEPTION_NEARBY_RADIUS_M_MANIPULATOR
    : AI_PERCEPTION_NEARBY_RADIUS_M_MOBILE;
  float maxDistanceScene = nearbyRadiusM * scenePerMeter;

  aiCollectObstacleCandidates(nearest, environmentPointListForRobotMode(robotMode), "scan", robotWorld, maxDistanceScene);
  aiCollectObstacleCandidates(nearest, importedWorldPointListForRobotMode(robotMode), "imported", robotWorld, maxDistanceScene);
  if (shouldUseDemoWorldForRobotMode(robotMode)) {
    aiCollectObstacleCandidates(nearest, demoWorldPointListForRobotMode(robotMode), "demo", robotWorld, maxDistanceScene);
  }

  JSONArray out = new JSONArray();
  for (int i = 0; i < nearest.size(); i++) {
    AiObstacleCandidate c = nearest.get(i);
    PVector delta = PVector.sub(c.world, robotWorld);
    PVector local = robotMode == ROBOT_MODE_MANIPULATOR ? delta : rotateWorldToLocal(delta, yaw);
    JSONObject item = new JSONObject();
    item.setString("source", c.source);
    item.setFloat("distance_m", c.distanceScene / scenePerMeter);
    item.setFloat("x_m", local.x / scenePerMeter);
    item.setFloat("y_m", local.y / scenePerMeter);
    item.setFloat("z_m", local.z / scenePerMeter);
    out.append(item);
  }

  aiPerceptionObstacleCache = out;
  aiPerceptionObstacleCacheMillis = millis();
  aiPerceptionObstacleCacheRobotMode = robotMode;
  aiPerceptionObstacleCacheRobotWorld = robotWorld.copy();
  aiPerceptionObstacleCacheYawRad = yaw;
  return out;
}

JSONObject buildAiPerceptionSceneSnapshot() {
  int robotMode = currentRobotMode();
  String robotName = currentModeName();
  JSONObject scene = new JSONObject();
  scene.setString("robot", robotName);
  scene.setJSONObject("pose", buildAiRobotPoseSnapshot(robotMode));
  scene.setJSONObject("world", buildAiWorldSnapshot(robotMode));
  scene.setJSONObject("camera", buildAiCameraSnapshot(robotMode));
  scene.setJSONArray("nearest_obstacles", buildAiNearestObstacleSnapshot(robotMode));
  scene.setJSONObject("sensorManifest", buildAiRobotSensorManifest(robotMode));
  scene.setString("telemetry_source", isLocalTelemetrySelected() ? "local" : "firmware");
  scene.setBoolean("simulation", simulationMode);
  return scene;
}

void sendAiPerceptionFrame(PImage sceneFrame) {
  if (!isAiPerceptionClientAlive() || aiPerceptionServer == null || sceneFrame == null) return;
  try {
    String encoded = encodePImageJpegBase64(
      sceneFrame,
      AI_PERCEPTION_JPEG_W,
      AI_PERCEPTION_JPEG_H,
      AI_PERCEPTION_MAX_BASE64_CHARS
    );
    if (encoded.length() == 0) return;

    JSONObject frame = new JSONObject();
    frame.setString("encoding", "jpeg_base64");
    frame.setInt("width", AI_PERCEPTION_JPEG_W);
    frame.setInt("height", AI_PERCEPTION_JPEG_H);
    frame.setString("data", encoded);

    JSONObject payload = new JSONObject();
    payload.setString("type", "perception");
    payload.setString("robot", currentModeName());
    payload.setJSONObject("frame", frame);
    payload.setJSONObject("scene", buildAiPerceptionSceneSnapshot());

    syncLatestSensorsForActiveTelemetrySource();
    if (latestSensors != null && latestSensors.size() > 0) {
      payload.setJSONObject("runtime", buildUnifiedRuntimeSnapshot(currentModeName(), latestSensors));
    }

    JSONObject message = buildSynRovMessage(
      SYNROV_MESSAGE_PERCEPTION_FRAME,
      SYNROV_SOURCE_PROCESSING,
      payload
    );
    aiPerceptionServer.sendMessage(message.toString());
    lastAiPerceptionFrameSentMillis = millis();
    aiPerceptionSendWarningLatched = false;
  }
  catch (Throwable t) {
    if (!aiPerceptionSendWarningLatched) {
      aiPerceptionSendWarningLatched = true;
      println("[SynROV][AiPerception] send failed: " + (t.getMessage() == null ? t.toString() : t.getMessage()));
    }
  }
}
