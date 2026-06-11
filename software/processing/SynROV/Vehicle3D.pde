// =====================================================================
// SynROV Processing - Vehicle 3D model
// ---------------------------------------------------------------------
// Purpose:
//   Vehicle geometry, scene representation and 3D rendering helpers.
// =====================================================================

float VEH_BODY_LENGTH = 95.70f; // centimeters
float VEH_BODY_WIDTH = 59.16f;
float VEH_BODY_HEIGHT = 18.56f;
float VEH_TRACK_GAUGE = 71.92f;
float VEH_TRACK_RUN = 69.60f;
float VEH_DRIVE_R = 13.92f;
float VEH_SUPPORT_R = 6.96f;
float VEH_TOP_ROLLER_R = 4.06f;
float VEH_TRACK_WIDTH = 17.40f;
float VEH_TRACK_LINK_LEN = 5.80f;
float VEH_TRACK_LINK_THICK = 2.03f;
float VEH_TRACK_HEIGHT = 5.80f;
int VEH_TRACK_LINKS = 64;
float VEH_TRACK_SAG = 2.32f;

final float VEH_DEFAULT_BODY_CENTER_Y_OFFSET = -24.13f;
final float VEH_DEFAULT_CAMERA_PAN_DEG = 0.0f;
final float VEH_DEFAULT_CAMERA_TILT_DEG = -10.0f;
final float VEH_DEFAULT_LIDAR_MAST_Y = -17.63f;
final float VEH_DEFAULT_LIDAR_MAST_Z = -7.66f;
final float VEH_DEFAULT_CAMERA_HEAD_Y = -12.06f;
final float VEH_DEFAULT_CAMERA_HEAD_Z = 19.14f;

final float VEH_NAV_LINEAR_SPEED = 1.85f;
final float VEH_NAV_YAW_SPEED = radians(0.92f);
final float VEH_TRACK_RESPONSE = 0.10f;
final float VEH_TRACK_ANIM_GAIN = 0.13f;
final float VEH_CURVE_MIN_THROTTLE = 0.16f;
final float VEH_CURVE_STEER_MARGIN = 0.12f;
final boolean VEHICLE_RENDER_ANCHORED = true;

final float VEHICLE_VISUAL_RENDER_SCALE = 1.0f;

final int VEH_FWD_IDX = 9;
final int VEH_TURN_IDX = 10;
final int VEH_PIVOT_IDX = 11;
final int VEH_CAM_PAN_IDX = 12;
final int VEH_CAM_TILT_IDX = 13;

float vehicleNavX = 0;
float vehicleNavZ = 0;
float vehicleNavYaw = 0;

float vehicleVisualYawDeg = 0;
float vehicleVisualPitchDeg = 0;
float vehicleVisualRollDeg = 0;
float vehicleBodyBob = 0;

float vehicleThrottleCmd = 0;
float vehicleSteerCmd = 0;
float vehicleLeftTrackCmd = 0;
float vehicleRightTrackCmd = 0;
float vehicleLeftTrackActual = 0;
float vehicleRightTrackActual = 0;
float VEH_BODY_CENTER_Y_OFFSET = VEH_DEFAULT_BODY_CENTER_Y_OFFSET;
float VEH_CAMERA_DEFAULT_PAN_DEG = VEH_DEFAULT_CAMERA_PAN_DEG;
float VEH_CAMERA_DEFAULT_TILT_DEG = VEH_DEFAULT_CAMERA_TILT_DEG;
float VEH_LIDAR_MAST_Y = VEH_DEFAULT_LIDAR_MAST_Y;
float VEH_LIDAR_MAST_Z = VEH_DEFAULT_LIDAR_MAST_Z;
float VEH_CAMERA_HEAD_Y = VEH_DEFAULT_CAMERA_HEAD_Y;
float VEH_CAMERA_HEAD_Z = VEH_DEFAULT_CAMERA_HEAD_Z;
final float VEH_CAMERA_PAN_LIMIT = 68.0f;
final float VEH_CAMERA_TILT_MIN = -35.0f;
final float VEH_CAMERA_TILT_MAX = 30.0f;

float vehicleCameraPanDeg = VEH_DEFAULT_CAMERA_PAN_DEG;
float vehicleCameraTiltDeg = VEH_DEFAULT_CAMERA_TILT_DEG;
long vehicleEmergencyStopUntilMs = 0;
float vehicleLinearSpeed = 0;
float vehicleAngularSpeed = 0;
float vehicleTrackPhaseLeft = 0;
float vehicleTrackPhaseRight = 0;
float vehicleLidarSpinDeg = 0;
boolean vehicleLightsEnabled = false;
boolean vehicleLidarScanEnabled = true;
boolean vehicleLastLightsCommandSent = false;
boolean vehicleLastLidarScanCommandSent = true;
long vehicleLastAuxRuntimeSendMs = 0;

long vehicleLastTickMs = 0;
long vehicleLastRuntimeSendMs = 0;
String vehicleLastRuntimeCommand = "";
boolean vehicleRuntimeCollisionFiltered = false;
long vehicleLastCollisionBlockNoticeMs = 0;

// Vehicle helper for connected camera pan min deg.
float vehicleCameraPanMinDegLimit() {
  return getSensorFloat("vehicle_camera_pan_min_deg", -VEH_CAMERA_PAN_LIMIT);
}

// Vehicle helper for connected camera pan max deg.
float vehicleCameraPanMaxDegLimit() {
  return getSensorFloat("vehicle_camera_pan_max_deg", VEH_CAMERA_PAN_LIMIT);
}

// Vehicle helper for connected camera tilt min deg.
float vehicleCameraTiltMinDegLimit() {
  return getSensorFloat("vehicle_camera_tilt_min_deg", VEH_CAMERA_TILT_MIN);
}

// Vehicle helper for connected camera tilt max deg.
float vehicleCameraTiltMaxDegLimit() {
  return getSensorFloat("vehicle_camera_tilt_max_deg", VEH_CAMERA_TILT_MAX);
}

// Builds vehicle dimensions 3 d object.
JSONObject buildVehicleDimensions3DObject() {
  JSONObject dims = new JSONObject();

  JSONObject body = new JSONObject();
  body.setFloat("length", VEH_BODY_LENGTH);
  body.setFloat("width", VEH_BODY_WIDTH);
  body.setFloat("height", VEH_BODY_HEIGHT);
  dims.setJSONObject("body", body);

  JSONObject track = new JSONObject();
  track.setFloat("gauge", VEH_TRACK_GAUGE);
  track.setFloat("run", VEH_TRACK_RUN);
  track.setFloat("width", VEH_TRACK_WIDTH);
  track.setFloat("linkLength", VEH_TRACK_LINK_LEN);
  track.setFloat("linkThickness", VEH_TRACK_LINK_THICK);
  track.setFloat("height", VEH_TRACK_HEIGHT);
  track.setInt("links", VEH_TRACK_LINKS);
  track.setFloat("sag", VEH_TRACK_SAG);
  dims.setJSONObject("track", track);

  JSONObject rollers = new JSONObject();
  rollers.setFloat("driveRadius", VEH_DRIVE_R);
  rollers.setFloat("supportRadius", VEH_SUPPORT_R);
  rollers.setFloat("topRadius", VEH_TOP_ROLLER_R);
  dims.setJSONObject("rollers", rollers);

  return dims;
}

// Builds vehicle visual offsets 3 d object.
JSONObject buildVehicleVisualOffsets3DObject() {
  JSONObject offsets = new JSONObject();
  offsets.setFloat("bodyCenterY", VEH_BODY_CENTER_Y_OFFSET);
  offsets.setFloat("cameraDefaultPanDeg", VEH_CAMERA_DEFAULT_PAN_DEG);
  offsets.setFloat("cameraDefaultTiltDeg", VEH_CAMERA_DEFAULT_TILT_DEG);
  offsets.setFloat("lidarMastY", VEH_LIDAR_MAST_Y);
  offsets.setFloat("lidarMastZ", VEH_LIDAR_MAST_Z);
  offsets.setFloat("cameraHeadY", VEH_CAMERA_HEAD_Y);
  offsets.setFloat("cameraHeadZ", VEH_CAMERA_HEAD_Z);
  return offsets;
}

// Builds vehicle config 3 d object.
JSONObject buildVehicleConfig3DObject() {
  JSONObject config3D = new JSONObject();
  config3D.setJSONObject("dimensions", buildVehicleDimensions3DObject());
  config3D.setJSONObject("visualOffsets", buildVehicleVisualOffsets3DObject());
  return config3D;
}

// Applies default vehicle dimensions.
void applyDefaultVehicleDimensions() {
  VEH_BODY_LENGTH = 95.70f;
  VEH_BODY_WIDTH = 59.16f;
  VEH_BODY_HEIGHT = 18.56f;
  VEH_TRACK_GAUGE = 71.92f;
  VEH_TRACK_RUN = 69.60f;
  VEH_DRIVE_R = 13.92f;
  VEH_SUPPORT_R = 6.96f;
  VEH_TOP_ROLLER_R = 4.06f;
  VEH_TRACK_WIDTH = 17.40f;
  VEH_TRACK_LINK_LEN = 5.80f;
  VEH_TRACK_LINK_THICK = 2.03f;
  VEH_TRACK_HEIGHT = 5.80f;
  VEH_TRACK_LINKS = 64;
  VEH_TRACK_SAG = 2.32f;
}

// Applies default vehicle visual offsets.
void applyDefaultVehicleVisualOffsets() {
  VEH_BODY_CENTER_Y_OFFSET = VEH_DEFAULT_BODY_CENTER_Y_OFFSET;
  VEH_CAMERA_DEFAULT_PAN_DEG = VEH_DEFAULT_CAMERA_PAN_DEG;
  VEH_CAMERA_DEFAULT_TILT_DEG = VEH_DEFAULT_CAMERA_TILT_DEG;
  VEH_LIDAR_MAST_Y = VEH_DEFAULT_LIDAR_MAST_Y;
  VEH_LIDAR_MAST_Z = VEH_DEFAULT_LIDAR_MAST_Z;
  VEH_CAMERA_HEAD_Y = VEH_DEFAULT_CAMERA_HEAD_Y;
  VEH_CAMERA_HEAD_Z = VEH_DEFAULT_CAMERA_HEAD_Z;
}

// Applies default vehicle config state.
void applyDefaultVehicleConfigState() {
  applyDefaultVehicleDimensions();
  applyDefaultVehicleVisualOffsets();
  vehicleNavX = 0.0f;
  vehicleNavZ = 0.0f;
  vehicleNavYaw = 0.0f;
  vehicleCameraPanDeg = VEH_CAMERA_DEFAULT_PAN_DEG;
  vehicleCameraTiltDeg = VEH_CAMERA_DEFAULT_TILT_DEG;
}

// Utility: refresh vehicle derived offsets defaults.
void refreshVehicleDerivedOffsetsDefaults() {
  applyDefaultVehicleVisualOffsets();
}

// Returns vehicle scene x.
float getVehicleSceneX() {
  return VEHICLE_RENDER_ANCHORED ? 0 : vehicleNavX;
}

// Returns vehicle scene z.
float getVehicleSceneZ() {
  return VEHICLE_RENDER_ANCHORED ? 0 : vehicleNavZ;
}

// Builds vehicle config object.
JSONObject buildVehicleConfigObject() {
  JSONObject json = new JSONObject();
  json.setString("schema", "synrov.robot_config.v2");
  json.setString("robot", "Vehicle");
  json.setString("geometryUnits", ROBOT_GEOMETRY_UNITS);
  json.setFloat("renderScale", VEHICLE_CM_TO_SCENE);
  json.setJSONObject("config3D", buildVehicleConfig3DObject());

  JSONObject runtime = new JSONObject();
  runtime.setFloat("x", vehicleNavX);
  runtime.setFloat("z", vehicleNavZ);
  runtime.setFloat("yawDeg", degrees(vehicleNavYaw));
  runtime.setFloat("cameraPanDeg", vehicleCameraPanDeg);
  runtime.setFloat("cameraTiltDeg", vehicleCameraTiltDeg);
  json.setJSONObject("runtime", runtime);

  return json;
}

// Creates default vehicle config.
void createDefaultVehicleConfig(String filename) {
  applyDefaultVehicleConfigState();
  saveJSONObjectEnsured(buildVehicleConfigObject(), filename);
}

// Applies vehicle dimensions JSON.
void applyVehicleDimensionsJson(JSONObject dimsRoot) {
  if (dimsRoot == null) return;

  JSONObject body = getJsonObjectSafe(dimsRoot, "body");
  VEH_BODY_LENGTH = getJsonFloat(body, "length", VEH_BODY_LENGTH);
  VEH_BODY_WIDTH = getJsonFloat(body, "width", VEH_BODY_WIDTH);
  VEH_BODY_HEIGHT = getJsonFloat(body, "height", VEH_BODY_HEIGHT);

  JSONObject track = getJsonObjectSafe(dimsRoot, "track");
  VEH_TRACK_GAUGE = getJsonFloat(track, "gauge", VEH_TRACK_GAUGE);
  VEH_TRACK_RUN = getJsonFloat(track, "run", VEH_TRACK_RUN);
  VEH_TRACK_WIDTH = getJsonFloat(track, "width", VEH_TRACK_WIDTH);
  VEH_TRACK_LINK_LEN = getJsonFloat(track, "linkLength", VEH_TRACK_LINK_LEN);
  VEH_TRACK_LINK_THICK = getJsonFloat(track, "linkThickness", VEH_TRACK_LINK_THICK);
  VEH_TRACK_HEIGHT = getJsonFloat(track, "height", VEH_TRACK_HEIGHT);
  VEH_TRACK_LINKS = max(16, getJsonInt(track, "links", VEH_TRACK_LINKS));
  VEH_TRACK_SAG = getJsonFloat(track, "sag", VEH_TRACK_SAG);

  JSONObject rollers = getJsonObjectSafe(dimsRoot, "rollers");
  VEH_DRIVE_R = getJsonFloat(rollers, "driveRadius", VEH_DRIVE_R);
  VEH_SUPPORT_R = getJsonFloat(rollers, "supportRadius", VEH_SUPPORT_R);
  VEH_TOP_ROLLER_R = getJsonFloat(rollers, "topRadius", VEH_TOP_ROLLER_R);
}

// Applies vehicle visual offsets JSON.
void applyVehicleVisualOffsetsJson(JSONObject offsets) {
  if (offsets == null) {
    refreshVehicleDerivedOffsetsDefaults();
    return;
  }

  VEH_BODY_CENTER_Y_OFFSET = getJsonFloat(offsets, "bodyCenterY", VEH_BODY_CENTER_Y_OFFSET);
  VEH_CAMERA_DEFAULT_PAN_DEG = getJsonFloat(offsets, "cameraDefaultPanDeg", VEH_CAMERA_DEFAULT_PAN_DEG);
  VEH_CAMERA_DEFAULT_TILT_DEG = getJsonFloat(offsets, "cameraDefaultTiltDeg", VEH_CAMERA_DEFAULT_TILT_DEG);
  VEH_LIDAR_MAST_Y = getJsonFloat(offsets, "lidarMastY", VEH_LIDAR_MAST_Y);
  VEH_LIDAR_MAST_Z = getJsonFloat(offsets, "lidarMastZ", VEH_LIDAR_MAST_Z);
  VEH_CAMERA_HEAD_Y = getJsonFloat(offsets, "cameraHeadY", VEH_CAMERA_HEAD_Y);
  VEH_CAMERA_HEAD_Z = getJsonFloat(offsets, "cameraHeadZ", VEH_CAMERA_HEAD_Z);
}

// Checks whether vehicle config schema valid.
boolean isVehicleConfigSchemaValid(JSONObject json) {
  if (json == null) return false;
  if (!"synrov.robot_config.v2".equals(getJsonString(json, "schema", ""))) return false;
  JSONObject config3D = getJsonObjectSafe(json, "config3D");
  if (config3D == null) return false;

  JSONObject dims = getJsonObjectSafe(config3D, "dimensions");
  JSONObject visualOffsets = getJsonObjectSafe(config3D, "visualOffsets");
  if (dims == null || visualOffsets == null) return false;

  return
    getJsonObjectSafe(dims, "body") != null &&
    getJsonObjectSafe(dims, "track") != null &&
    getJsonObjectSafe(dims, "rollers") != null;
}

// Loads vehicle config.
void loadVehicleConfig(String filename) {
  JSONObject json = loadJSONObjectWithRepair(filename, buildVehicleConfigObject());
  if (json == null) return;

  JSONObject config3D = getJsonObjectSafe(json, "config3D");
  JSONObject dims = getJsonObjectSafe(config3D, "dimensions");
  JSONObject visualOffsets = getJsonObjectSafe(config3D, "visualOffsets");

  applyVehicleDimensionsJson(dims);
  applyVehicleVisualOffsetsJson(visualOffsets);

  JSONObject runtime = getJsonObjectSafe(json, "runtime");
  if (runtime != null) {
    vehicleNavX = getJsonFloat(runtime, "x", vehicleNavX);
    vehicleNavZ = getJsonFloat(runtime, "z", vehicleNavZ);
    vehicleNavYaw = radians(getJsonFloat(runtime, "yawDeg", degrees(vehicleNavYaw)));
    vehicleCameraPanDeg = getJsonFloat(runtime, "cameraPanDeg", vehicleCameraPanDeg);
    vehicleCameraTiltDeg = getJsonFloat(runtime, "cameraTiltDeg", vehicleCameraTiltDeg);
  }
}

// Loads or create vehicle config.
void loadOrCreateVehicleConfig() {
  File dataDir = new File(sketchPath("data"));
  if (!dataDir.exists()) dataDir.mkdirs();

  if (!fileExists(VEHICLE3D_CONFIG_FILE)) {
    createDefaultVehicleConfig(VEHICLE3D_CONFIG_FILE);
  }

  // The code is the default model. The file may override any subset of
  // parameters, but missing sections must not force a reset or NullPointer.
  loadJSONObjectWithRepair(VEHICLE3D_CONFIG_FILE, buildVehicleConfigObject());
  loadVehicleConfig(VEHICLE3D_CONFIG_FILE);
}

// Checks whether vehicle telemetry geometry.
boolean hasVehicleTelemetryGeometry() {
  return latestSensors != null && (
    latestSensors.hasKey("vehicle_body_length") ||
    latestSensors.hasKey("vehicle_track_gauge") ||
    latestSensors.hasKey("vehicle_track_run") ||
    latestSensors.hasKey("vehicle_drive_radius"));
}

// Applies vehicle telemetry geometry from sensors.
void applyVehicleTelemetryGeometryFromSensors() {
  if (!hasVehicleTelemetryGeometry()) return;

  VEH_BODY_LENGTH = getSensorFloat("vehicle_body_length", VEH_BODY_LENGTH);
  VEH_BODY_WIDTH = getSensorFloat("vehicle_body_width", VEH_BODY_WIDTH);
  VEH_BODY_HEIGHT = getSensorFloat("vehicle_body_height", VEH_BODY_HEIGHT);
  VEH_TRACK_GAUGE = getSensorFloat("vehicle_track_gauge", VEH_TRACK_GAUGE);
  VEH_TRACK_RUN = getSensorFloat("vehicle_track_run", VEH_TRACK_RUN);
  VEH_DRIVE_R = getSensorFloat("vehicle_drive_radius", VEH_DRIVE_R);
  VEH_SUPPORT_R = getSensorFloat("vehicle_support_radius", VEH_SUPPORT_R);
  VEH_TOP_ROLLER_R = getSensorFloat("vehicle_top_roller_radius", VEH_TOP_ROLLER_R);
  VEH_TRACK_WIDTH = getSensorFloat("vehicle_track_width", VEH_TRACK_WIDTH);
  VEH_TRACK_LINK_LEN = getSensorFloat("vehicle_track_link_length", VEH_TRACK_LINK_LEN);
  VEH_TRACK_LINK_THICK = getSensorFloat("vehicle_track_link_thickness", VEH_TRACK_LINK_THICK);
  VEH_TRACK_HEIGHT = getSensorFloat("vehicle_track_height", VEH_TRACK_HEIGHT);
  VEH_TRACK_LINKS = max(16, round(getSensorFloat("vehicle_track_links", VEH_TRACK_LINKS)));
  VEH_TRACK_SAG = getSensorFloat("vehicle_track_sag", VEH_TRACK_SAG);
  VEH_BODY_CENTER_Y_OFFSET = getSensorFloat("vehicle_body_center_y", VEH_BODY_CENTER_Y_OFFSET);
  VEH_CAMERA_DEFAULT_PAN_DEG = getSensorFloat("vehicle_camera_default_pan_deg", VEH_CAMERA_DEFAULT_PAN_DEG);
  VEH_CAMERA_DEFAULT_TILT_DEG = getSensorFloat("vehicle_camera_default_tilt_deg", VEH_CAMERA_DEFAULT_TILT_DEG);
  VEH_LIDAR_MAST_Y = getSensorFloat("vehicle_lidar_mast_y", VEH_LIDAR_MAST_Y);
  VEH_LIDAR_MAST_Z = getSensorFloat("vehicle_lidar_mast_z", VEH_LIDAR_MAST_Z);
  VEH_CAMERA_HEAD_Y = getSensorFloat("vehicle_camera_head_y", VEH_CAMERA_HEAD_Y);
  VEH_CAMERA_HEAD_Z = getSensorFloat("vehicle_camera_head_z", VEH_CAMERA_HEAD_Z);
}

class VehicleTrackLinkPose {
  PVector p;
  float pitch;
  VehicleTrackLinkPose(PVector p, float pitch) {
    this.p = p;
    this.pitch = pitch;
  }
}

// Resets vehicle input state.
void resetVehicleInputState() {
  vehicleThrottleCmd = 0;
  vehicleSteerCmd = 0;
  vehicleLeftTrackCmd = 0;
  vehicleRightTrackCmd = 0;
}

// Vehicle helper for emergency stop latch.
boolean vehicleEmergencyStopLatched() {
  return millis() < vehicleEmergencyStopUntilMs;
}

// Returns the throttle used by smooth steering commands.
float vehicleCurveThrottle(float throttle, float steer) {
  float semanticThrottle = constrain(throttle, -1.0f, 1.0f);
  float semanticSteer = constrain(steer, -1.0f, 1.0f);
  float steerMag = abs(semanticSteer);
  if (steerMag <= 0.0001f) return semanticThrottle;

  float throttleMag = abs(semanticThrottle);
  float direction = semanticThrottle < -0.0001f ? -1.0f : 1.0f;
  float requiredMag = constrain(max(VEH_CURVE_MIN_THROTTLE, steerMag + VEH_CURVE_STEER_MARGIN), 0.0f, 1.0f);
  if (throttleMag < requiredMag) throttleMag = requiredMag;
  return constrain(direction * throttleMag, -1.0f, 1.0f);
}

// Sets vehicle drive normalized with arc steering.
void setVehicleDriveNormalized(float throttle, float steer) {
  float semanticThrottle = vehicleEmergencyStopLatched() ? 0.0f : constrain(throttle, -1.0f, 1.0f);
  float semanticSteer = vehicleEmergencyStopLatched() ? 0.0f : constrain(steer, -1.0f, 1.0f);
  float curveThrottle = vehicleEmergencyStopLatched() ? 0.0f : vehicleCurveThrottle(semanticThrottle, semanticSteer);

  vehicleThrottleCmd = semanticThrottle;
  vehicleSteerCmd = semanticSteer;

  float leftSemantic = constrain(curveThrottle + semanticSteer, -1.0f, 1.0f);
  float rightSemantic = constrain(curveThrottle - semanticSteer, -1.0f, 1.0f);
  vehicleLeftTrackCmd = constrain(-rightSemantic, -1.0f, 1.0f);
  vehicleRightTrackCmd = constrain(-leftSemantic, -1.0f, 1.0f);
}

// Sets vehicle drive normalized as an in-place pivot.
void setVehiclePivotNormalized(float pivot) {
  float semanticPivot = vehicleEmergencyStopLatched() ? 0.0f : constrain(pivot, -1.0f, 1.0f);
  vehicleThrottleCmd = 0.0f;
  vehicleSteerCmd = semanticPivot;
  vehicleLeftTrackCmd = constrain(semanticPivot, -1.0f, 1.0f);
  vehicleRightTrackCmd = constrain(-semanticPivot, -1.0f, 1.0f);
}


// Sets vehicle track normalized.
void setVehicleTrackNormalized(float left, float right) {
  if (vehicleEmergencyStopLatched()) {
    vehicleLeftTrackCmd = 0;
    vehicleRightTrackCmd = 0;
  } else {
    vehicleLeftTrackCmd = constrain(left, -1.0f, 1.0f);
    vehicleRightTrackCmd = constrain(right, -1.0f, 1.0f);
  }
  vehicleThrottleCmd = -((vehicleLeftTrackCmd + vehicleRightTrackCmd) * 0.5f);
  vehicleSteerCmd = (vehicleLeftTrackCmd - vehicleRightTrackCmd) * 0.5f;
}

// Stops vehicle motion.
void stopVehicleMotion(boolean forceSend) {
  vehicleEmergencyStopUntilMs = millis() + 1200;
  resetVehicleInputState();
  setVehicleTrackNormalized(0, 0);
  sendVehicleRuntimeCommand(forceSend);
}

// Utility: center vehicle camera.
void centerVehicleCamera() {
  vehicleCameraPanDeg = constrain(VEH_CAMERA_DEFAULT_PAN_DEG, vehicleCameraPanMinDegLimit(), vehicleCameraPanMaxDegLimit());
  vehicleCameraTiltDeg = constrain(VEH_CAMERA_DEFAULT_TILT_DEG, vehicleCameraTiltMinDegLimit(), vehicleCameraTiltMaxDegLimit());
  sendVehicleRuntimeCommand(true);
}

// Utility: semantic vehicle track percentage.
int vehicleSemanticTrackPct(float trackValue) {
  return constrain(round((-trackValue) * 100.0f), -100, 100);
}

// Returns the operator-facing left track command.
float vehicleReportedLeftTrackCmdValue() {
  return vehicleRightTrackCmd;
}

// Returns the operator-facing right track command.
float vehicleReportedRightTrackCmdValue() {
  return vehicleLeftTrackCmd;
}

// Returns the operator-facing left track feedback.
float vehicleReportedLeftTrackActualValue() {
  return vehicleRightTrackActual;
}

// Returns the operator-facing right track feedback.
float vehicleReportedRightTrackActualValue() {
  return vehicleLeftTrackActual;
}

class VehicleRuntimeCommandState {
  int leftPct;
  int rightPct;
  int camPanDeg;
  int camTiltDeg;
  int flags;
}

// Returns whether the vehicle command contains track motion.
boolean vehicleRuntimeHasTrackMotion() {
  return abs(vehicleLeftTrackCmd) > 0.0001f || abs(vehicleRightTrackCmd) > 0.0001f;
}

// Checks the commanded vehicle motion against the shared environment guard.
boolean vehicleRuntimeMotionBlockedByCollision() {
  if (!environmentCollisionEnabled || vehicleEmergencyStopLatched()) return false;
  if (!vehicleRuntimeHasTrackMotion()) return false;

  float avgHardware = (vehicleLeftTrackCmd + vehicleRightTrackCmd) * 0.5f;
  float avg = -avgHardware;
  float diff = (vehicleRightTrackCmd - vehicleLeftTrackCmd) * 0.5f;
  if (abs(avg) < 0.0001f && abs(diff) < 0.0001f) return false;

  float turnBlend = constrain(abs(diff), 0.0f, 1.0f);
  float tractionFactor = 1.0f - 0.22f * turnBlend;
  float navYawStep = diff * VEH_NAV_YAW_SPEED;
  float navForwardStep = avg * VEH_NAV_LINEAR_SPEED * tractionFactor;
  if (abs(avg) < 0.02f && abs(diff) > 0.02f) navForwardStep = 0.0f;

  float proposedYaw = vehicleNavYaw + navYawStep;
  float motionYaw = vehicleNavYaw + navYawStep * 0.5f;
  float proposedX = vehicleNavX + sin(motionYaw) * navForwardStep;
  float proposedZ = vehicleNavZ + cos(motionYaw) * navForwardStep;
  return wouldVehicleEnvironmentCollide(proposedX, proposedZ, proposedYaw);
}

// Returns the collision-filtered left track command used by physics and hardware.
float vehicleEffectiveLeftTrackCommand() {
  return vehicleRuntimeMotionBlockedByCollision() ? 0.0f : vehicleLeftTrackCmd;
}

// Returns the collision-filtered right track command used by physics and hardware.
float vehicleEffectiveRightTrackCommand() {
  return vehicleRuntimeMotionBlockedByCollision() ? 0.0f : vehicleRightTrackCmd;
}

// Emits a bounded notice when the vehicle command is held by the map guard.
void notifyVehicleRuntimeCollisionBlock() {
  long now = millis();
  if ((now - vehicleLastCollisionBlockNoticeMs) < 700) return;
  vehicleLastCollisionBlockNoticeMs = now;
  updateMessage(tr("Movimento do veículo bloqueado pela proteção de colisão.", "Vehicle motion blocked by the collision guard."));
}

// Returns the canonical vehicle runtime state shared by animation, ASCII and HEX commands.
VehicleRuntimeCommandState buildVehicleRuntimeCommandState() {
  return buildVehicleRuntimeCommandState(true);
}

// Returns the vehicle runtime state, optionally applying the Processing guard before transport.
VehicleRuntimeCommandState buildVehicleRuntimeCommandState(boolean applyCollisionFilter) {
  VehicleRuntimeCommandState state = new VehicleRuntimeCommandState();
  boolean blockedByCollision = applyCollisionFilter && vehicleRuntimeMotionBlockedByCollision();
  vehicleRuntimeCollisionFiltered = blockedByCollision;
  if (vehicleEmergencyStopLatched() || blockedByCollision) {
    state.leftPct = 0;
    state.rightPct = 0;
  } else {
    state.leftPct = vehicleSemanticTrackPct(vehicleReportedLeftTrackCmdValue());
    state.rightPct = vehicleSemanticTrackPct(vehicleReportedRightTrackCmdValue());
  }
  state.camPanDeg = round(constrain(vehicleCameraPanDeg, vehicleCameraPanMinDegLimit(), vehicleCameraPanMaxDegLimit()));
  state.camTiltDeg = round(constrain(vehicleCameraTiltDeg, vehicleCameraTiltMinDegLimit(), vehicleCameraTiltMaxDegLimit()));
  state.flags = (vehicleLightsEnabled ? 0x01 : 0x00) | (vehicleLidarScanEnabled ? 0x02 : 0x00);
  return state;
}

// Builds vehicle runtime command.
String buildVehicleRuntimeCommand() {
  VehicleRuntimeCommandState state = buildVehicleRuntimeCommandState();
  return "TRACK=" + state.leftPct + "," + state.rightPct + ",CAM=" + state.camPanDeg + "," + state.camTiltDeg;
}

// Resets pending vehicle runtime command.
void resetPendingVehicleRuntimeCommand() {
  vehicleRuntimeCommandSource = CONTROL_SOURCE_LOCAL;
  vehicleLastRuntimeSendMs = 0;
  vehicleLastRuntimeCommand = "";
  vehicleLastLightsCommandSent = !vehicleLightsEnabled;
  vehicleLastLidarScanCommandSent = !vehicleLidarScanEnabled;
  vehicleLastAuxRuntimeSendMs = 0;
}

// Flushes queued vehicle auxiliary intents without using a WebSocket serial shortcut.
void flushPendingVehicleAuxRuntimeCommands() {
  if (!isVehicleSelected) return;
  if (!(systemReady && !simulationMode) || myPort == null) return;
  if (serialMonitorSessionActive || hardwareStreamStoppedByExit) return;

  long now = millis();
  if (vehicleLastAuxRuntimeSendMs > 0 && (now - vehicleLastAuxRuntimeSendMs) < 40) return;

  String cmd = "";
  String context = "";
  if (vehicleLightsEnabled != vehicleLastLightsCommandSent) {
    cmd = "LIGHT=" + (vehicleLightsEnabled ? 1 : 0);
    context = "vehicle lights";
  } else if (vehicleLidarScanEnabled != vehicleLastLidarScanCommandSent) {
    cmd = "SCAN=" + (vehicleLidarScanEnabled ? 1 : 0);
    context = "vehicle lidar";
  }
  if (cmd.length() == 0) return;

  int previousSource = commandContextSource;
  setCommandContext(vehicleRuntimeCommandSource);
  try {
    if (sendHardwareStreamCommand(cmd, context, false)) {
      if (cmd.startsWith("LIGHT=")) vehicleLastLightsCommandSent = vehicleLightsEnabled;
      if (cmd.startsWith("SCAN=")) vehicleLastLidarScanCommandSent = vehicleLidarScanEnabled;
      vehicleLastAuxRuntimeSendMs = now;
    }
  }
  finally {
    commandContextSource = previousSource;
  }
}

// Utility: flush pending vehicle runtime command.
void flushPendingVehicleRuntimeCommand(boolean force) {
  if (!isVehicleSelected) return;
  if (!(systemReady && !simulationMode) || myPort == null) return;
  if (serialMonitorSessionActive || hardwareStreamStoppedByExit) return;

  long now = millis();
  boolean blockedByCollision = vehicleRuntimeMotionBlockedByCollision();
  if (blockedByCollision) notifyVehicleRuntimeCollisionBlock();

  String cmd = buildVehicleRuntimeCommand();
  boolean changed = !cmd.equals(vehicleLastRuntimeCommand);
  boolean resendPending = vehicleRuntimeResendIntervalMs > 0 &&
    vehicleLastRuntimeSendMs > 0 &&
    (now - vehicleLastRuntimeSendMs) >= vehicleRuntimeResendIntervalMs;
  boolean keepAliveDue = vehicleLastRuntimeSendMs == 0 ||
    (now - vehicleLastRuntimeSendMs) >= HARDWARE_STREAM_KEEPALIVE_MS;

  if (!force && !changed && !resendPending && !keepAliveDue) return;

  if (!force) {
    int minInterval = changed ? vehicleRuntimeSendIntervalMs : min(vehicleRuntimeResendIntervalMs, HARDWARE_STREAM_KEEPALIVE_MS);
    if (minInterval > 0 && vehicleLastRuntimeSendMs > 0 && (now - vehicleLastRuntimeSendMs) < minInterval) return;
  }

  int previousSource = commandContextSource;
  setCommandContext(vehicleRuntimeCommandSource);
  try {
    if (sendHardwareStreamCommand(cmd, "vehicle runtime", false)) {
      vehicleLastRuntimeCommand = cmd;
      vehicleLastRuntimeSendMs = now;
    }
    flushPendingVehicleAuxRuntimeCommands();
  }
  finally {
    commandContextSource = previousSource;
  }
}

// Queues a vehicle runtime intent for the shared Processing control pipeline.
void sendVehicleRuntimeCommand(boolean force) {
  vehicleRuntimeCommandSource = commandContextSource;
  vehicleRuntimeForcePending = vehicleRuntimeForcePending || force;
}

// Vehicle helper for track path length.
float vehicleTrackPathLength() {
  return 2.0f * VEH_TRACK_RUN + TWO_PI * VEH_DRIVE_R;
}

// Vehicle helper for track sag.
float vehicleTrackSag(float t) {
  return -4.0f * (t - 0.5f) * (t - 0.5f) + 1.0f;
}

// Returns vehicle track link pose.
VehicleTrackLinkPose getVehicleTrackLinkPose(float u) {
  float zRear = -VEH_TRACK_RUN * 0.5f;
  float zFront = VEH_TRACK_RUN * 0.5f;
  float straight = VEH_TRACK_RUN;
  float arc = PI * VEH_DRIVE_R;
  float total = 2 * straight + 2 * arc;
  float s = (u - floor(u)) * total;
  PVector pos = new PVector();
  float pitchLocal = 0;

  if (s < straight) {
    float t = s / straight;
    pos.z = lerp(zRear, zFront, t);
    pos.y = -VEH_DRIVE_R;
    return new VehicleTrackLinkPose(pos, 0);
  }
  s -= straight;

  if (s < arc) {
    float ang = s / VEH_DRIVE_R;
    pos.z = zFront + VEH_DRIVE_R * sin(ang);
    pos.y = -VEH_DRIVE_R * cos(ang);
    pitchLocal = atan2(sin(ang), cos(ang));
    return new VehicleTrackLinkPose(pos, pitchLocal);
  }
  s -= arc;

  if (s < straight) {
    float t = s / straight;
    pos.z = lerp(zFront, zRear, t);
    float sag = VEH_TRACK_SAG * vehicleTrackSag(t);
    pos.y = VEH_DRIVE_R - sag;
    float eps = 0.002f;
    float z2 = lerp(zFront, zRear, min(1.0f, t + eps));
    float y2 = VEH_DRIVE_R - VEH_TRACK_SAG * vehicleTrackSag(min(1.0f, t + eps));
    pitchLocal = atan2(y2 - pos.y, z2 - pos.z);
    return new VehicleTrackLinkPose(pos, pitchLocal);
  }
  s -= straight;

  float ang = s / VEH_DRIVE_R;
  pos.z = zRear - VEH_DRIVE_R * sin(ang);
  pos.y = VEH_DRIVE_R * cos(ang);
  pitchLocal = atan2(-sin(ang), -cos(ang));
  return new VehicleTrackLinkPose(pos, pitchLocal);
}

// Utility: draw vehicle drive wheel.
void drawVehicleDriveWheel(float r, float width, float spinRad) {
  pushMatrix();
  rotateZ(HALF_PI);
  rotateY(spinRad);
  noStroke();
  fill(22, 22, 22);
  drawCylinder(r, width);

  fill(40, 40, 40);
  for (int i = 0; i < 4; i++) {
    pushMatrix();
    rotateY(TWO_PI * i / 4.0f);
    translate(r * 0.38f, 0, 0);
    box(r * 0.14f, width * 0.18f, r * 0.48f);
    safePopMatrix("Vehicle3D.pde:530");
  }

  fill(18, 18, 18);
  sphere(r * 0.18f);
  safePopMatrix("Vehicle3D.pde:535");
}

// Utility: draw vehicle track.
void drawVehicleTrack(float trackPhase) {
  noStroke();
  float total = vehicleTrackPathLength();
  float offset = (trackPhase * VEH_DRIVE_R) / total;

  for (int i = 0; i < VEH_TRACK_LINKS; i++) {
    float u = ((float)i / (float)VEH_TRACK_LINKS + offset) % 1.0f;
    if (u < 0) u += 1.0f;
    VehicleTrackLinkPose pose = getVehicleTrackLinkPose(u);
    pushMatrix();
    translate(0, pose.p.y, pose.p.z);
    rotateX(pose.pitch + HALF_PI);
    fill(18, 18, 18);
    box(VEH_TRACK_LINK_THICK, VEH_TRACK_HEIGHT, VEH_TRACK_LINK_LEN);
    pushMatrix();
    translate(0, VEH_TRACK_HEIGHT * 0.16f, 0);
    fill(32, 32, 32);
    box(VEH_TRACK_LINK_THICK * 1.15f, 2.6f, VEH_TRACK_LINK_LEN * 0.74f);
    safePopMatrix("Vehicle3D.pde:557");
    safePopMatrix("Vehicle3D.pde:558");
  }
}

// Updates vehicle physics.
void updateVehiclePhysics() {
  long now = millis();
  if (vehicleLastTickMs == 0) vehicleLastTickMs = now;
  float dt = max(0.001f, (now - vehicleLastTickMs) / 1000.0f);
  vehicleLastTickMs = now;

  vehicleLeftTrackActual = lerp(vehicleLeftTrackActual, vehicleEffectiveLeftTrackCommand(), VEH_TRACK_RESPONSE);
  vehicleRightTrackActual = lerp(vehicleRightTrackActual, vehicleEffectiveRightTrackCommand(), VEH_TRACK_RESPONSE);

  float avgHardware = (vehicleLeftTrackActual + vehicleRightTrackActual) * 0.5f;
  float avg = -avgHardware;
  float diff = (vehicleRightTrackActual - vehicleLeftTrackActual) * 0.5f;

  // Differential-drive integration with midpoint heading so the vehicle
  // follows an arc instead of visually "sliding" sideways while turning.
  float turnBlend = constrain(abs(diff), 0.0f, 1.0f);
  float tractionFactor = 1.0f - 0.22f * turnBlend;
  float navYawStep = diff * VEH_NAV_YAW_SPEED * dt * 60.0f;
  float navForwardStep = avg * VEH_NAV_LINEAR_SPEED * dt * 60.0f * tractionFactor;
  if (abs(avg) < 0.02f && abs(diff) > 0.02f) {
    navForwardStep = 0.0f;
  }

  vehicleLinearSpeed = navForwardStep;
  vehicleAngularSpeed = navYawStep;

  float proposedYaw = vehicleNavYaw + navYawStep;
  float motionYaw = vehicleNavYaw + navYawStep * 0.5f;
  float proposedX = vehicleNavX + sin(motionYaw) * navForwardStep;
  float proposedZ = vehicleNavZ + cos(motionYaw) * navForwardStep;

  if (!wouldVehicleEnvironmentCollide(proposedX, proposedZ, proposedYaw)) {
    vehicleNavYaw = proposedYaw;
    vehicleNavX = proposedX;
    vehicleNavZ = proposedZ;
  } else if (!wouldVehicleEnvironmentCollide(vehicleNavX, vehicleNavZ, proposedYaw)) {
    vehicleNavYaw = proposedYaw;
  }

  vehicleTrackPhaseLeft += (-vehicleLeftTrackActual) * VEH_TRACK_ANIM_GAIN;
  vehicleTrackPhaseRight += (-vehicleRightTrackActual) * VEH_TRACK_ANIM_GAIN;

  float targetYawDeg = constrain(diff * 13.0f, -11.0f, 11.0f);
  float targetPitchDeg = constrain(-avg * 4.2f, -5.0f, 5.0f);
  float targetRollDeg = constrain(diff * 4.6f, -4.5f, 4.5f);
  float motion = max(abs(avg), abs(diff));

  vehicleVisualYawDeg = lerp(vehicleVisualYawDeg, targetYawDeg, 0.07f);
  vehicleVisualPitchDeg = lerp(vehicleVisualPitchDeg, targetPitchDeg, 0.09f);
  vehicleVisualRollDeg = lerp(vehicleVisualRollDeg, targetRollDeg, 0.09f);
  vehicleBodyBob = lerp(vehicleBodyBob, sin(frameCount * 0.16f) * motion * 1.2f, 0.06f);

  vehicleLidarSpinDeg = (vehicleLidarSpinDeg + (vehicleLidarScanEnabled ? 4.4f : 0.35f) + motion * 6.0f) % 360.0f;

  if (!systemReady || simulationMode) {
    vehicleHeadingTelemetryDeg = degrees(vehicleNavYaw);
  }

}

// Utility: draw vehicle ground.
void drawVehicleGround() {
  float headingDeg = systemReady ? vehicleHeadingTelemetryDeg : degrees(vehicleNavYaw);
  drawSharedGroundPlane(visualGroundSizeForCurrentRobot(), headingDeg, true);
}

// Utility: draw vehicle 3 d.
void drawVehicle3D(boolean advanceCamera) {
  pushMatrix();
  translate(width / 2 + 120, height / 2 + 80);
  rotateX(cameraRotationX);
  rotateY(cameraRotationY);
  if (advanceCamera) {
    cameraRotationY += cameraRotationYIncrement;
    cameraRotationX += cameraRotationXIncrement;
  }

  if (zoomLevel != 1) {
    PVector center = new PVector(width / 2 + 120, height / 2 + 80);
    PVector offset = PVector.sub(zoomTarget, center);
    translate(offset.x, offset.y);
    scale(zoomLevel);
    translate(-offset.x, -offset.y);
  }

  drawVehicleGround();
  applySceneLighting();
  drawCurrentEnvironmentMap();
  drawVehicleRadarProbe();
  noStroke();

  float axleYWorld = GROUND_Y - VEH_DRIVE_R;
  float bodyCenterY = GROUND_Y + VEH_BODY_CENTER_Y_OFFSET + vehicleBodyBob;

  pushMatrix();
  translate(getVehicleSceneX(), bodyCenterY, getVehicleSceneZ());
  scale(VEHICLE_VISUAL_RENDER_SCALE);
  float vehicleRenderYaw = VEHICLE_RENDER_ANCHORED ? 0.0f : vehicleNavYaw;
  rotateY(vehicleRenderYaw + radians(vehicleVisualYawDeg));
  rotateZ(radians(vehicleVisualRollDeg));
  rotateX(radians(vehicleVisualPitchDeg));

  // Lower hull
  fill(173, 55, 247);
  box(VEH_BODY_WIDTH, VEH_BODY_HEIGHT, VEH_BODY_LENGTH);

  // Upper armor shell
  pushMatrix();
  translate(0, -15, -8);
  fill(200, 100, 100);
  box(VEH_BODY_WIDTH * 0.78f, VEH_BODY_HEIGHT * 0.78f, VEH_BODY_LENGTH * 0.50f);
  safePopMatrix("Vehicle3D.pde:673");

  // Side armor plates
  for (int side = -1; side <= 1; side += 2) {
    pushMatrix();
    translate(side * (VEH_TRACK_GAUGE * 0.5f - 8), -2, 0);
    fill(100, 200, 100);
    box(8, VEH_BODY_HEIGHT * 0.92f, VEH_BODY_LENGTH * 0.94f);
    safePopMatrix("Vehicle3D.pde:681");
  }

  // Front wedge
  pushMatrix();
  translate(0, 0, VEH_BODY_LENGTH * 0.46f);
  rotateX(radians(22));
  fill(100, 100, 200);
  box(VEH_BODY_WIDTH * 0.62f, 1, 30);
  safePopMatrix("Vehicle3D.pde:690");

  pushMatrix();
  translate(0, 5, VEH_BODY_LENGTH * 0.50f);
  fill(255, 220, 70);
  box(18, 4, 6);
  safePopMatrix("Vehicle3D.pde:696");

  // Sensor mast + LiDAR
  pushMatrix();
  translate(0, VEH_LIDAR_MAST_Y, VEH_LIDAR_MAST_Z);
  fill(100, 100, 200);
  box(12, 30, 12);
  translate(0, -18, 0);
  rotateY(radians(vehicleLidarSpinDeg));
  fill(0, 220, 80);
  box(24, 8, 24);
  safePopMatrix("Vehicle3D.pde:707");

  // Camera head
  pushMatrix();
  translate(0, VEH_CAMERA_HEAD_Y - 3, VEH_CAMERA_HEAD_Z - 3);
  rotateY(radians(constrain(vehicleCameraPanDeg, vehicleCameraPanMinDegLimit(), vehicleCameraPanMaxDegLimit())));
  rotateX(radians(constrain(vehicleCameraTiltDeg, vehicleCameraTiltMinDegLimit(), vehicleCameraTiltMaxDegLimit()) * 0.65f));
  fill(100, 100, 200);
  sphere(5);
  pushMatrix();
  translate(0, 0, 4.5);
  fill(255, 220, 0);
  sphere(1);
  safePopMatrix("Vehicle3D.pde:720");
  safePopMatrix("Vehicle3D.pde:721");

  // Lamps
  for (int side = -1; side <= 1; side += 2) {
    pushMatrix();
    translate(side * 22, -6, VEH_BODY_LENGTH * 0.48f);
    fill(vehicleLightsEnabled ? color(255, 220, 70) : color(110, 96, 40));
    sphere(5);
    safePopMatrix("Vehicle3D.pde:729");
  }

  float zRear = -VEH_TRACK_RUN * 0.5f;
  float zFront = VEH_TRACK_RUN * 0.5f;
  float yWheelLocal = axleYWorld - bodyCenterY;

  for (int side = -1; side <= 1; side += 2) {
    float xSide = side * (VEH_TRACK_GAUGE * 0.5f);
    float trackPhase = side < 0 ? vehicleTrackPhaseLeft : vehicleTrackPhaseRight;
    float driveSpin = trackPhase * 4.5f;

    pushMatrix();
    translate(xSide, yWheelLocal, 0);

    pushMatrix();
    translate(0, 0, zFront);
    drawVehicleDriveWheel(VEH_DRIVE_R, VEH_TRACK_WIDTH, driveSpin);
    safePopMatrix("Vehicle3D.pde:747");

    pushMatrix();
    translate(0, 0, zRear);
    drawVehicleDriveWheel(VEH_DRIVE_R, VEH_TRACK_WIDTH, driveSpin);
    safePopMatrix("Vehicle3D.pde:752");

    pushMatrix();
    fill(36, 36, 36);
    translate(0, -6, 0);
    box(8, 5, VEH_TRACK_RUN * 0.72f);
    safePopMatrix("Vehicle3D.pde:758");

    drawVehicleTrack(trackPhase);
    safePopMatrix("Vehicle3D.pde:761");
  }

  safePopMatrix("Vehicle3D.pde:764");
  safePopMatrix("Vehicle3D.pde:765");
  noLights();
}

// Utility: capture vehicle frame.
int[] captureVehicleFrame() {
  return new int[] {
    round(vehicleLeftTrackCmd * 100.0f),
    round(vehicleRightTrackCmd * 100.0f),
    round(vehicleCameraPanDeg),
    round(vehicleCameraTiltDeg),
    vehicleLightsEnabled ? 1 : 0,
    vehicleLidarScanEnabled ? 1 : 0
  };
}

// Applies vehicle frame.
void applyVehicleFrame(int[] frame) {
  if (frame == null || frame.length < 6) return;
  setVehicleTrackNormalized(frame[0] / 100.0f, frame[1] / 100.0f);
  vehicleCameraPanDeg = constrain(frame[2], vehicleCameraPanMinDegLimit(), vehicleCameraPanMaxDegLimit());
  vehicleCameraTiltDeg = constrain(frame[3], vehicleCameraTiltMinDegLimit(), vehicleCameraTiltMaxDegLimit());
  vehicleLightsEnabled = frame[4] > 0;
  vehicleLidarScanEnabled = frame[5] > 0;
  sendVehicleRuntimeCommand(true);
}

// Utility: move vehicle safely toward frame.
boolean moveVehicleSafelyTowardFrame(int[] targetFrame, int maxStep, int tolerance) {
  if (targetFrame == null || targetFrame.length < 6) return true;

  float left = approachFloat(vehicleLeftTrackCmd * 100.0f, targetFrame[0], maxStep);
  float right = approachFloat(vehicleRightTrackCmd * 100.0f, targetFrame[1], maxStep);
  setVehicleTrackNormalized(left / 100.0f, right / 100.0f);
  vehicleCameraPanDeg = approachFloat(vehicleCameraPanDeg, targetFrame[2], maxStep);
  vehicleCameraTiltDeg = approachFloat(vehicleCameraTiltDeg, targetFrame[3], maxStep);
  vehicleLightsEnabled = targetFrame[4] > 0;
  vehicleLidarScanEnabled = targetFrame[5] > 0;
  sendVehicleRuntimeCommand(false);

  return abs(vehicleLeftTrackCmd * 100.0f - targetFrame[0]) <= tolerance &&
    abs(vehicleRightTrackCmd * 100.0f - targetFrame[1]) <= tolerance &&
    abs(vehicleCameraPanDeg - targetFrame[2]) <= tolerance &&
    abs(vehicleCameraTiltDeg - targetFrame[3]) <= tolerance;
}


void drawVehicle3D() {
  drawVehicle3D(true);
}


// =====================================================================
// Vehicle diagnostics module section
// =====================================================================

boolean handleVehicleDiagnosticsPanelMousePressed(int mx, int my, int panelX, int panelY, int contentX, int contentW, int gap, int toolsInnerX, int toolsCol2W) {
  if (pointInRect(mx, my, toolsInnerX, diagnosticsMapButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { toggleRemoteCollisionFromPanel(); return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsMapButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { clearTraceMap(); return true; }
  if (pointInRect(mx, my, toolsInnerX, diagnosticsMapButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { loadWorldMapFromWindowsDialog(); return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsMapButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { removeImportedWorldOnly(); return true; }

  if (pointInRect(mx, my, toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { stopVehicleMotion(true); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { toggleVehicleLights(); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { toggleVehicleLidarScan(); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { centerVehicleCamera(); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 2), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { triggerCompassCalibration(); return true; }
  return true;
}

void drawVehicleDiagnosticsPanelSections(int panelY, int contentX, int contentW, int gap, int toolsInnerX, int toolsCol2W, int sensorsContentX, int sensorsContentW, int vehicleRuntimeCardY, int vehicleSensorsCardY) {
  drawSectionCard2D(contentX, vehicleRuntimeCardY, contentW, DIAGNOSTICS_VEHICLE_RUNTIME_CARD_H, tr("Runtime do veículo", "Vehicle runtime"));
  drawMiniButton2D(toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Parada de emergência", "Emergency stop"), false);
  drawMiniButton2D(toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Luzes", "Lights"), vehicleLightsEnabled);
  drawMiniButton2D(toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Varredura LiDAR", "LiDAR scan"), vehicleLidarScanEnabled);
  drawMiniButton2D(toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Centralizar câmera", "Center camera"), false);
  drawMiniButton2D(toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 2), toolsCol2W, DIAGNOSTICS_BUTTON_H, compassCalibrationActive() ? tr("Cal. bússola *", "Compass cal *") : tr("Cal. bússola", "Compass cal"), compassCalibrationActive());

  drawSectionCard2D(contentX, vehicleSensorsCardY, contentW, DIAGNOSTICS_VEHICLE_SENSORS_CARD_H, tr("Sensores de estabilidade do veículo", "Vehicle stability sensors"));
  int vehicleStabilityWidgetH = DIAGNOSTICS_VEHICLE_SENSORS_CARD_H - 144;
  drawVehicleStabilityTargets2D(sensorsContentX, vehicleSensorsCardY + 32, sensorsContentW, vehicleStabilityWidgetH);
  fill(uiPrimaryTextColor());
  int vehicleInfoY = vehicleSensorsCardY + 32 + vehicleStabilityWidgetH + 12;
  drawDiagnosticsFittedTextLeft(diagnosticsCompassSummaryText(), sensorsContentX, vehicleInfoY, sensorsContentW);
  drawDiagnosticsFittedTextLeft(tr("Posição: firmware/odometria interna pela bússola", "Position: firmware/internal odometry by compass"), sensorsContentX, vehicleInfoY + 18, sensorsContentW);
  drawPressureBar2D(sensorsContentX, vehicleInfoY + 42, sensorsContentW, 18, vehicleBatteryPct, 100, tr("Bateria", "Battery"));
  drawDiagnosticsFittedTextLeft(tr("Bateria bruta / Mundo: ", "Battery raw / World: ") + nf(getSensorFloat("battery_raw", getSensorFloat("battery", 0)), 1, 0) + " / " + currentEnvironmentWorldSummary(), sensorsContentX, vehicleInfoY + 70, sensorsContentW);
}

// =====================================================================
// SynRovModule adapter: Vehicle3D
// =====================================================================
class Vehicle3DModule implements SynRovModule {
  public String id() { return "Vehicle"; }
  public String displayName() { return robotDisplayName("Vehicle"); }
  public boolean available() { return true; }

  public boolean supports(String capability) {
    return MODULE_CAP_DIAGNOSTICS.equals(capability) ||
      MODULE_CAP_ENVIRONMENT.equals(capability) ||
      MODULE_CAP_RUNTIME.equals(capability) ||
      MODULE_CAP_COLLISION.equals(capability) ||
      MODULE_CAP_TRAJECTORY.equals(capability) ||
      MODULE_CAP_CAMERA.equals(capability) ||
      MODULE_CAP_GPS.equals(capability) ||
      MODULE_CAP_GROUND_VEHICLE.equals(capability);
  }

  public void setupModule() { }

  public void loadConfigSafe() {
    try {
      loadOrCreateVehicleConfig();
    }
    catch (Exception e) {
      println("Vehicle config recovery: " + e.getMessage());
      backupBrokenConfigFile(VEHICLE3D_CONFIG_FILE);
      saveJSONObjectEnsured(buildVehicleConfigObject(), VEHICLE3D_CONFIG_FILE);
      try { loadVehicleConfig(VEHICLE3D_CONFIG_FILE); }
      catch (Exception ignore) { resetVehicleInputState(); }
    }
  }

  public void updateModule() { }

  public void updatePhysicsSafe() {
    updateVehiclePhysics();
  }

  public void drawModule3D(boolean advanceCamera) {
    drawVehicle3D(advanceCamera);
  }

  public void resetInputStateSafe() {
    resetVehicleInputState();
  }

  public int[] captureFrameSafe() {
    return captureVehicleFrame();
  }

  public void applyFrameSafe(int[] frame) {
    applyVehicleFrame(frame);
  }

  public boolean moveSafelyTowardFrameSafe(int[] targetFrame, int maxStepDeg, int toleranceDeg) {
    return moveVehicleSafelyTowardFrame(targetFrame, maxStepDeg, toleranceDeg);
  }

  public JSONObject buildConfigObjectSafe() {
    return buildVehicleConfigObject();
  }
}
