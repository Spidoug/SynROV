// =====================================================================
// SynROV Processing - Robot front-camera stream
// ---------------------------------------------------------------------
// Purpose:
//   Dedicated AiBot camera transport and in-world monitor for Vehicle and Drone.
//   Port 9002 remains a separate high-bandwidth transport, but camera control
//   and frame messages use the same SynROV application envelope as all other
//   communication paths.
// =====================================================================

WebsocketServer robotCameraServer;
final int ROBOT_CAMERA_PORT = 9002;
final String ROBOT_CAMERA_PATH = "/";
final int ROBOT_CAMERA_FRAME_STALE_MS = 2500;
final int ROBOT_CAMERA_CONTROL_REFRESH_MS = 1200;
final int ROBOT_CAMERA_MAX_BASE64_CHARS = 1800000;
// Optical monitor geometry. The monitor is positioned along the actual
// camera ray; pan/tilt are directions, not copied screen offsets.
final float ROBOT_CAMERA_PANEL_SIZE_MULTIPLIER = 2.40f;
final float ROBOT_CAMERA_CASE_STYLE_MULTIPLIER = 0.48f;
final float ROBOT_CAMERA_DISPLAY_DISTANCE = 175.0f;
final float ROBOT_CAMERA_THIRD_PERSON_ZOOM = 1.15f;
final float ROBOT_CAMERA_REAR_VIEW_YAW = PI;

boolean vehicleFrontCameraViewEnabled = false;
boolean droneFrontCameraViewEnabled = false;

PImage vehicleFrontCameraFrame = null;
PImage droneFrontCameraFrame = null;
long vehicleFrontCameraFrameMillis = 0;
long droneFrontCameraFrameMillis = 0;
long vehicleFrontCameraSequence = -1;
long droneFrontCameraSequence = -1;
String vehicleFrontCameraSource = "none";
String droneFrontCameraSource = "none";
boolean vehicleFrontCameraFallback = true;
boolean droneFrontCameraFallback = true;

final Object robotCameraPendingLock = new Object();
String pendingVehicleCameraBase64 = null;
String pendingDroneCameraBase64 = null;
long pendingVehicleCameraSequence = -1;
long pendingDroneCameraSequence = -1;
String pendingVehicleCameraSource = "none";
String pendingDroneCameraSource = "none";
boolean pendingVehicleCameraFallback = true;
boolean pendingDroneCameraFallback = true;

long lastRobotCameraControlBroadcastMs = 0;

boolean robotCameraThirdPersonApplied = false;
int robotCameraThirdPersonMode = -1;
float robotCameraSavedRotationX = 0.0f;
float robotCameraSavedRotationY = 0.0f;
float robotCameraSavedZoom = 1.0f;
PVector robotCameraSavedZoomTarget = new PVector();

void initializeRobotCameraServer() {
  try {
    robotCameraServer = new WebsocketServer(this, ROBOT_CAMERA_PORT, ROBOT_CAMERA_PATH);
    println("[SynROV][RobotCamera] stream online on port " + ROBOT_CAMERA_PORT);
  }
  catch (Exception e) {
    robotCameraServer = null;
    println("[SynROV][RobotCamera] stream failed on port " + ROBOT_CAMERA_PORT + ": " + e.getMessage());
  }
}

boolean isRobotCameraSupported(int robotMode) {
  return robotMode == ROBOT_MODE_VEHICLE || robotMode == ROBOT_MODE_DRONE;
}

boolean isRobotCameraViewEnabled(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleFrontCameraViewEnabled;
  if (robotMode == ROBOT_MODE_DRONE) return droneFrontCameraViewEnabled;
  return false;
}

boolean isRobotCameraThirdPersonActiveForMode(int robotMode) {
  return robotCameraThirdPersonApplied &&
    robotCameraThirdPersonMode == robotMode &&
    isRobotCameraViewEnabled(robotMode);
}

void setRobotCameraViewEnabled(int robotMode, boolean enabled) {
  if (!isRobotCameraSupported(robotMode)) return;

  if (robotMode == ROBOT_MODE_VEHICLE) {
    if (enabled && droneFrontCameraViewEnabled) {
      droneFrontCameraViewEnabled = false;
      broadcastRobotCameraControl(ROBOT_MODE_DRONE, "stop");
    }
    vehicleFrontCameraViewEnabled = enabled;
  } else {
    if (enabled && vehicleFrontCameraViewEnabled) {
      vehicleFrontCameraViewEnabled = false;
      broadcastRobotCameraControl(ROBOT_MODE_VEHICLE, "stop");
    }
    droneFrontCameraViewEnabled = enabled;
    // Drone camera mode and the firmware camera flag are one V1 state.
    if (droneCameraStreamingEnabled != enabled) {
      droneCameraStreamingEnabled = enabled;
      sendDroneRuntimeCommand(true);
    }
  }

  broadcastRobotCameraControl(robotMode, enabled ? "start" : "stop");
  lastRobotCameraControlBroadcastMs = millis();
  updateMessage(robotMode == ROBOT_MODE_VEHICLE
    ? tr("Vehicle camera: ") + (enabled ? "ON" : "OFF")
    : tr("Drone camera: ") + (enabled ? "ON" : "OFF"));
  sendSystemStatus();
}

void toggleRobotCameraView(int robotMode) {
  setRobotCameraViewEnabled(robotMode, !isRobotCameraViewEnabled(robotMode));
}

void toggleVehicleCamera() {
  toggleRobotCameraView(ROBOT_MODE_VEHICLE);
}

void toggleSelectedDroneCameraView() {
  toggleRobotCameraView(ROBOT_MODE_DRONE);
}

String robotCameraNameForMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return "vehicle";
  if (robotMode == ROBOT_MODE_DRONE) return "drone";
  return "unsupported";
}

int robotModeForCameraName(String name) {
  String normalized = name == null ? "" : trim(name).toLowerCase();
  if (normalized.equals("vehicle")) return ROBOT_MODE_VEHICLE;
  if (normalized.equals("drone")) return ROBOT_MODE_DRONE;
  return -1;
}

void broadcastRobotCameraControl(int robotMode, String action) {
  if (robotCameraServer == null || !isRobotCameraSupported(robotMode)) return;
  try {
    JSONObject payload = new JSONObject();
    payload.setString("robot", robotCameraNameForMode(robotMode));
    payload.setString("action", action == null ? "status" : action);
    payload.setBoolean("enabled", isRobotCameraViewEnabled(robotMode));
    payload.setInt("target_fps", 12);
    payload.setString("encoding", "jpeg_base64");
    JSONObject message = buildSynRovMessage(
      SYNROV_MESSAGE_ROBOT_CAMERA_CONTROL,
      SYNROV_SOURCE_PROCESSING,
      payload
    );
    robotCameraServer.sendMessage(message.toString());
  }
  catch (Exception e) {
    println("[SynROV][RobotCamera] control send failed: " + e.getMessage());
  }
}

String extractRobotCameraBase64(JSONObject payload) {
  if (payload == null) return "";
  return getJsonString(payload, "image", "");
}

// Handles messages arriving on the shared websocket callback. messageType
// identifies camera traffic because the Processing websocket library exposes
// one callback for all server instances.
boolean handleRobotCameraSocketEvent(String raw) {
  if (raw == null) return false;
  String normalized = trim(raw);
  if (normalized.length() == 0) return false;

  JSONObject message = safeParseJsonObject(normalized);
  if (!isSynRovMessageType(message, SYNROV_MESSAGE_ROBOT_CAMERA_FRAME)) return false;
  if (!SYNROV_SOURCE_AIBOT.equals(normalizeProtocolSource(getJsonString(message, "source", "")))) return true;

  JSONObject payload = synRovMessagePayload(message);
  int robotMode = robotModeForCameraName(getJsonString(payload, "robot", ""));
  if (!isRobotCameraSupported(robotMode)) return true;
  if (!isRobotCameraViewEnabled(robotMode)) return true;

  String encoding = getJsonString(payload, "encoding", "").toLowerCase();
  if (!encoding.equals("jpeg_base64")) return true;
  if (getJsonInt(payload, "width", 0) <= 0 || getJsonInt(payload, "height", 0) <= 0) return true;
  if (!jsonHasValue(payload, "fallback")) return true;

  String encoded = extractRobotCameraBase64(payload);
  if (encoded.length() == 0 || encoded.length() > ROBOT_CAMERA_MAX_BASE64_CHARS) return true;
  long seq = getJsonLong(message, "seq", -1);
  String source = trim(getJsonString(payload, "cameraSource", ""));
  if (source.length() == 0) return true;
  boolean fallback = getJsonBoolean(payload, "fallback", false);

  synchronized(robotCameraPendingLock) {
    if (robotMode == ROBOT_MODE_VEHICLE) {
      pendingVehicleCameraBase64 = encoded;
      pendingVehicleCameraSequence = seq;
      pendingVehicleCameraSource = source;
      pendingVehicleCameraFallback = fallback;
    } else {
      pendingDroneCameraBase64 = encoded;
      pendingDroneCameraSequence = seq;
      pendingDroneCameraSource = source;
      pendingDroneCameraFallback = fallback;
    }
  }
  return true;
}

void consumePendingRobotCameraFrames() {
  String vehicleEncoded = null;
  String droneEncoded = null;
  long vehicleSeq = -1;
  long droneSeq = -1;
  String vehicleSource = "none";
  String droneSource = "none";
  boolean vehicleFallback = true;
  boolean droneFallback = true;

  synchronized(robotCameraPendingLock) {
    if (pendingVehicleCameraBase64 != null) {
      vehicleEncoded = pendingVehicleCameraBase64;
      vehicleSeq = pendingVehicleCameraSequence;
      vehicleSource = pendingVehicleCameraSource;
      vehicleFallback = pendingVehicleCameraFallback;
      pendingVehicleCameraBase64 = null;
    }
    if (pendingDroneCameraBase64 != null) {
      droneEncoded = pendingDroneCameraBase64;
      droneSeq = pendingDroneCameraSequence;
      droneSource = pendingDroneCameraSource;
      droneFallback = pendingDroneCameraFallback;
      pendingDroneCameraBase64 = null;
    }
  }

  if (vehicleEncoded != null) {
    PImage decoded = decodeJpegBase64ToPImage(vehicleEncoded);
    if (decoded != null) {
      vehicleFrontCameraFrame = decoded;
      vehicleFrontCameraFrameMillis = millis();
      vehicleFrontCameraSequence = vehicleSeq;
      vehicleFrontCameraSource = vehicleSource;
      vehicleFrontCameraFallback = vehicleFallback;
    }
  }
  if (droneEncoded != null) {
    PImage decoded = decodeJpegBase64ToPImage(droneEncoded);
    if (decoded != null) {
      droneFrontCameraFrame = decoded;
      droneFrontCameraFrameMillis = millis();
      droneFrontCameraSequence = droneSeq;
      droneFrontCameraSource = droneSource;
      droneFrontCameraFallback = droneFallback;
    }
  }
}

PImage robotCameraFrameForMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleFrontCameraFrame;
  if (robotMode == ROBOT_MODE_DRONE) return droneFrontCameraFrame;
  return null;
}

long robotCameraFrameMillisForMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleFrontCameraFrameMillis;
  if (robotMode == ROBOT_MODE_DRONE) return droneFrontCameraFrameMillis;
  return 0;
}

boolean robotCameraFrameIsLive(int robotMode) {
  long stamp = robotCameraFrameMillisForMode(robotMode);
  return stamp > 0 && (millis() - stamp) <= ROBOT_CAMERA_FRAME_STALE_MS;
}

String robotCameraSourceForMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleFrontCameraSource;
  if (robotMode == ROBOT_MODE_DRONE) return droneFrontCameraSource;
  return "none";
}

boolean robotCameraFallbackForMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleFrontCameraFallback;
  if (robotMode == ROBOT_MODE_DRONE) return droneFrontCameraFallback;
  return true;
}

void applyRobotCameraThirdPersonPreset(int robotMode) {
  cameraRotationXIncrement = 0;
  cameraRotationYIncrement = 0;
  cameraRotationX = robotMode == ROBOT_MODE_DRONE ? -PI / 8.5f : -PI / 7.0f;
  // Camera mode is a rear chase view: the operator looks over the back of the
  // selected robot toward its forward direction.
  cameraRotationY = ROBOT_CAMERA_REAR_VIEW_YAW;
  // Vehicle and Drone intentionally share the same camera-mode zoom reference.
  zoomLevel = ROBOT_CAMERA_THIRD_PERSON_ZOOM;
  zoomLevel = constrain(zoomLevel, currentMinZoomLevel(), MAX_ZOOM);
  zoomTarget.set(width / 2 + 120, height / 2 + 80, 0);
}

void updateRobotCameraThirdPersonState() {
  int mode = currentRobotMode();
  boolean shouldFollow = isRobotCameraSupported(mode) && isRobotCameraViewEnabled(mode);

  if (shouldFollow) {
    if (!robotCameraThirdPersonApplied) {
      robotCameraSavedRotationX = cameraRotationX;
      robotCameraSavedRotationY = cameraRotationY;
      robotCameraSavedZoom = zoomLevel;
      robotCameraSavedZoomTarget.set(zoomTarget);
      robotCameraThirdPersonApplied = true;
    }
    if (robotCameraThirdPersonMode != mode) {
      robotCameraThirdPersonMode = mode;
      applyRobotCameraThirdPersonPreset(mode);
    }
  } else if (robotCameraThirdPersonApplied) {
    cameraRotationX = robotCameraSavedRotationX;
    cameraRotationY = robotCameraSavedRotationY;
    zoomLevel = constrain(robotCameraSavedZoom, currentMinZoomLevel(), MAX_ZOOM);
    zoomTarget.set(robotCameraSavedZoomTarget);
    robotCameraThirdPersonApplied = false;
    robotCameraThirdPersonMode = -1;
  }
}

// Applies camera-mode-only scene offsets. The Drone's position, including
// altitude, is already centered by applyRobotSceneReferenceOffset() for every
// Drone view so enabling the physical-camera mode must not translate it twice.
void applyRobotCameraThirdPersonSceneOffset(int robotMode) {
  if (!robotCameraThirdPersonApplied || robotCameraThirdPersonMode != robotMode) return;
  if (!isRobotCameraViewEnabled(robotMode)) return;
  if (robotMode == ROBOT_MODE_VEHICLE) {
    translate(-getVehicleSceneX(), 0, -getVehicleSceneZ());
  }
}

void updateRobotCameraStreamService() {
  consumePendingRobotCameraFrames();
  updateRobotCameraThirdPersonState();

  long now = millis();
  if ((now - lastRobotCameraControlBroadcastMs) >= ROBOT_CAMERA_CONTROL_REFRESH_MS) {
    lastRobotCameraControlBroadcastMs = now;
    broadcastRobotCameraControl(ROBOT_MODE_VEHICLE, vehicleFrontCameraViewEnabled ? "start" : "stop");
    broadcastRobotCameraControl(ROBOT_MODE_DRONE, droneFrontCameraViewEnabled ? "start" : "stop");
  }
}

void applyRobotFrontCameraPanelPose(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) {
    float robotScale = VEHICLE_VISUAL_RENDER_SCALE * largeWorldRobotPresentationScale(ROBOT_MODE_VEHICLE);
    float renderYaw = VEHICLE_RENDER_ANCHORED ? 0.0f : vehicleNavYaw;
    float bodyPitch = radians(vehicleBaseAttitudePitchDeg + vehicleVisualPitchDeg);
    float bodyRoll = radians(vehicleBaseAttitudeRollDeg + vehicleVisualRollDeg);
    float cameraPan = radians(constrain(vehicleCameraPanDeg, vehicleCameraPanMinDegLimit(), vehicleCameraPanMaxDegLimit()));
    float cameraTilt = radians(constrain(vehicleCameraTiltDeg, vehicleCameraTiltMinDegLimit(), vehicleCameraTiltMaxDegLimit()));

    // Reproduce the physical camera transform up to the lens origin.
    translate(getVehicleSceneX(), GROUND_Y + VEH_BODY_CENTER_Y_OFFSET + vehicleBodyBob, getVehicleSceneZ());
    scale(robotScale);
    rotateY(renderYaw + radians(vehicleVisualYawDeg));
    rotateZ(bodyRoll);
    rotateX(bodyPitch);
    translate(0, VEH_CAMERA_HEAD_Y - 3.0f, VEH_CAMERA_HEAD_Z - 3.0f);

    // Keep camera-origin placement scaled with the robot, then return to world
    // units so display distance and dimensions are real scene distances.
    if (abs(robotScale) > 0.0001f) scale(1.0f / robotScale);

    // Pan/tilt define the optical ray. Advancing only after these rotations
    // projects the monitor to the point reached at the configured distance.
    rotateY(cameraPan);
    rotateX(cameraTilt);
    translate(0, 0, ROBOT_CAMERA_DISPLAY_DISTANCE);

    // The textured front face looks back along the ray toward the camera.
    rotateY(PI);
    return;
  }

  if (robotMode == ROBOT_MODE_DRONE) {
    float robotScale = DRONE_VISUAL_RENDER_SCALE * largeWorldRobotPresentationScale(ROBOT_MODE_DRONE);
    float renderYaw = DRONE_RENDER_ANCHORED ? 0.0f : droneNavYaw;
    float cameraPan = radians(constrain(droneCameraPanDeg, droneCameraPanMinDegLimit(), droneCameraPanMaxDegLimit()));
    float cameraTilt = radians(constrain(droneCameraTiltDeg, droneCameraTiltMinDegLimit(), droneCameraTiltMaxDegLimit()));

    // Match the Drone body and gimbal transform all the way to the lens.
    translate(getDroneSceneX(), getDroneSceneY() + droneY, getDroneSceneZ());
    scale(robotScale);
    rotateY(renderYaw + droneYaw + DRONE_VISUAL_YAW_OFFSET);
    rotateX(dronePitch);
    rotateZ(droneRoll);
    translate(0, DRONE_CAMERA_Y, DRONE_CAMERA_Z + 3.0f);
    if (abs(robotScale) > 0.0001f) scale(1.0f / robotScale);

    rotateY(cameraPan);
    rotateX(cameraTilt);
    translate(0, 0, ROBOT_CAMERA_DISPLAY_DISTANCE);

    // Same visible-side convention as Vehicle.
    rotateY(PI);
  }
}

void drawRobotFrontCameraPanel(int robotMode) {
  if (!isRobotCameraSupported(robotMode) || !isRobotCameraViewEnabled(robotMode)) return;

  float panelW = (robotMode == ROBOT_MODE_VEHICLE ? 135.0f : 125.0f) * ROBOT_CAMERA_PANEL_SIZE_MULTIPLIER;
  float panelH = panelW * 9.0f / 16.0f;
  float casePadding = 8.0f * ROBOT_CAMERA_CASE_STYLE_MULTIPLIER;
  float caseDepth = 4.0f * ROBOT_CAMERA_CASE_STYLE_MULTIPLIER;
  float frontSurfaceZ = caseDepth * 0.5f + 0.15f;
  PImage frame = robotCameraFrameForMode(robotMode);
  boolean live = robotCameraFrameIsLive(robotMode);

  // This monitor is a real world-space object, not a camera-facing billboard.
  // It inherits the selected robot's pose; the common scene camera then gives
  // it the same perspective, foreshortening and depth as every other 3D object.
  pushMatrix();
  applyRobotFrontCameraPanelPose(robotMode);

  pushStyle();
  noLights();
  hint(ENABLE_DEPTH_TEST);
  noStroke();

  // A shallow physical case gives the monitor a visible back/edge when the
  // operator orbits around it. With depth testing enabled, the front video is
  // naturally hidden by the case when viewed from behind.
  fill(10, 12, 16, 238);
  box(panelW + casePadding, panelH + casePadding, caseDepth);

  translate(0, 0, frontSurfaceZ);
  if (frame != null) {
    textureMode(NORMAL);
    beginShape(QUADS);
    texture(frame);
    vertex(-panelW * 0.5f, -panelH * 0.5f, 0, 0, 0);
    vertex(panelW * 0.5f, -panelH * 0.5f, 0, 1, 0);
    vertex(panelW * 0.5f, panelH * 0.5f, 0, 1, 1);
    vertex(-panelW * 0.5f, panelH * 0.5f, 0, 0, 1);
    endShape();
  } else {
    fill(20, 24, 30);
    rectMode(CENTER);
    rect(0, 0, panelW, panelH);
  }

  noFill();
  stroke(live ? color(80, 235, 120) : color(245, 170, 60));
  strokeWeight(2.5f);
  rectMode(CENTER);
  rect(0, 0, panelW + 2.0f, panelH + 2.0f);

  popStyle();
  safePopMatrix("RobotCameraStream.pde:panel");
  applySceneLighting();
}
