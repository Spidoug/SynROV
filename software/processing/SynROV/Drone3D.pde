// =====================================================================
// SynROV Processing - Drone 3D model
// ---------------------------------------------------------------------
// Purpose:
//   Drone geometry, scene representation and 3D rendering helpers.
// =====================================================================

final float GROUND_SIZE = 480;
final float MAX_ALTITUDE = GROUND_SIZE * 2.8f;
final float DRONE_LOCAL_MAX_ALTITUDE_CM = 4000.0f;
final float DRONE_LOCAL_MAX_LIFT = DRONE_LOCAL_MAX_ALTITUDE_CM * 0.5f;
final float DRONE_ROTOR_FLIGHT_SPEED = 43.0f;
final float DRONE_ROTOR_SPOOL_UP_LERP = 0.14f;
final float DRONE_ROTOR_SPOOL_DOWN_LERP = 0.09f;
final float DRONE_ROTOR_BLUR_START_SPEED = 12.0f;
final float DRONE_ROTOR_BLUR_FULL_SPEED = 34.0f;

final float DRONE_DEFAULT_BODY_DIM = 36.48f;
final float DRONE_DEFAULT_BODY_L = 36.48f;
final float DRONE_DEFAULT_BODY_W = 36.48f;
final float DRONE_DEFAULT_BODY_H = 10.64f;
final float DRONE_DEFAULT_ARM_LENGTH = 34.96f;
final float DRONE_DEFAULT_ARM_THICKNESS = 3.04f;
final float DRONE_DEFAULT_MOTOR_RADIUS = 4.56f;
final float DRONE_DEFAULT_MOTOR_HEIGHT = 6.08f;
final float DRONE_DEFAULT_PROP_RADIUS = 13.68f;
final float DRONE_DEFAULT_PROP_THICKNESS = 1.82f;
final float DRONE_DEFAULT_LEG_HEIGHT = 12.16f;
final float DRONE_DEFAULT_LEG_SPAN = 15.20f;
final float DRONE_DEFAULT_VISUAL_YAW_OFFSET_DEG = 0.0f;
final float DRONE_DEFAULT_CAMERA_Y = -7.66f;
final float DRONE_DEFAULT_CAMERA_Z = 12.40f;
final float DRONE_DEFAULT_LAMP_Y = -1.06f;
final float DRONE_DEFAULT_LAMP_Z = 17.15f;
final float DRONE_DEFAULT_SONAR_X = 0.0f;
final float DRONE_DEFAULT_SONAR_Y = 6.90f;
final float DRONE_DEFAULT_SONAR_Z = 0.0f;
final float DRONE_DEFAULT_REST_Y = -17.48f;

float DRONE_BODY_DIM = DRONE_DEFAULT_BODY_DIM;
float DRONE_BODY_L = DRONE_DEFAULT_BODY_L;
float DRONE_BODY_W = DRONE_DEFAULT_BODY_W;
float DRONE_BODY_H = DRONE_DEFAULT_BODY_H;
float DRONE_ARM_LENGTH = DRONE_DEFAULT_ARM_LENGTH;
float DRONE_ARM_THICKNESS = DRONE_DEFAULT_ARM_THICKNESS;
float DRONE_MOTOR_RADIUS = DRONE_DEFAULT_MOTOR_RADIUS;
float DRONE_MOTOR_HEIGHT = DRONE_DEFAULT_MOTOR_HEIGHT;
float DRONE_PROP_RADIUS = DRONE_DEFAULT_PROP_RADIUS;
float DRONE_PROP_THICKNESS = DRONE_DEFAULT_PROP_THICKNESS;
float DRONE_LEG_HEIGHT = DRONE_DEFAULT_LEG_HEIGHT;
float DRONE_LEG_SPAN = DRONE_DEFAULT_LEG_SPAN;
float DRONE_REST_Y = DRONE_DEFAULT_REST_Y;
float DRONE_VISUAL_YAW_OFFSET = radians(DRONE_DEFAULT_VISUAL_YAW_OFFSET_DEG);
float DRONE_CAMERA_Y = DRONE_DEFAULT_CAMERA_Y;
float DRONE_CAMERA_Z = DRONE_DEFAULT_CAMERA_Z;
float DRONE_LAMP_Y = DRONE_DEFAULT_LAMP_Y;
float DRONE_LAMP_Z = DRONE_DEFAULT_LAMP_Z;
float DRONE_SONAR_X = DRONE_DEFAULT_SONAR_X;
float DRONE_SONAR_Y = DRONE_DEFAULT_SONAR_Y;
float DRONE_SONAR_Z = DRONE_DEFAULT_SONAR_Z;
final boolean DRONE_RENDER_ANCHORED = true;

final float DRONE_VISUAL_RENDER_SCALE = 1.0f;

final int YAW_IDX = 14;
final int PITCH_IDX = 15;
final int ROLL_IDX = 16;
final int ALT_IDX = 17;
final int STRAFE_IDX = 18;
final int FWD_IDX = 19;
final int DRONE_CAM_PAN_IDX = 20;
final int DRONE_CAM_TILT_IDX = 21;

float droneNavX = 0;
float droneNavY = 0;
float droneNavZ = 0;
float droneNavYaw = 0;

float droneX = 0;
float droneZ = 0;
float droneY = DRONE_REST_Y;
float droneYaw = 0;
float dronePitch = 0;
float droneRoll = 0;

float droneThrottleCmd = 0;
float droneYawCmd = 0;
float dronePitchCmd = 0;
float droneRollCmd = 0;
float droneStrafeCmd = 0;
float droneForwardCmd = 0;

final float DRONE_ASCENT_COMMAND_GAIN = 1.28f;
final float DRONE_ASCENT_MIN_EFFECTIVE_CMD = 0.16f;

float droneYawCmdFiltered = 0;
float dronePitchCmdFiltered = 0;
float droneRollCmdFiltered = 0;
float droneThrottleCmdFiltered = 0;
float droneStrafeCmdFiltered = 0;
float droneForwardCmdFiltered = 0;

float droneFlightLift = 0;
float droneTargetLift = 0;
float dronePropPhase = 0;
float droneScannerSweepDeg = 0;
boolean droneStabilizeEnabled = true;
boolean droneCameraStreamingEnabled = false;
boolean droneLastCameraStreamingCommandSent = false;
long droneLastAuxRuntimeSendMs = 0;
boolean droneRuntimeCollisionFiltered = false;
long droneLastCollisionBlockNoticeMs = 0;
float droneCameraPanDeg = 0;
float droneCameraTiltDeg = -10;
final float DRONE_CAMERA_PAN_LIMIT = 90.0f;
final float DRONE_CAMERA_TILT_MIN = -45.0f;
final float DRONE_CAMERA_TILT_MAX = 35.0f;
float droneRotorSpeed = 0;

float droneCameraPanMinDegLimit() {
  return getSensorFloat("drone_camera_pan_min_deg", -DRONE_CAMERA_PAN_LIMIT);
}

float droneCameraPanMaxDegLimit() {
  return getSensorFloat("drone_camera_pan_max_deg", DRONE_CAMERA_PAN_LIMIT);
}

float droneCameraTiltMinDegLimit() {
  return getSensorFloat("drone_camera_tilt_min_deg", DRONE_CAMERA_TILT_MIN);
}

float droneCameraTiltMaxDegLimit() {
  return getSensorFloat("drone_camera_tilt_max_deg", DRONE_CAMERA_TILT_MAX);
}

long droneLastTickMs = 0;
long droneLastRuntimeSendMs = 0;
String droneLastRuntimeCommand = "";

// Builds drone dimensions 3 d object.
JSONObject buildDroneDimensions3DObject() {
  JSONObject dims = new JSONObject();

  JSONObject body = new JSONObject();
  body.setFloat("length", DRONE_BODY_L);
  body.setFloat("width", DRONE_BODY_W);
  body.setFloat("height", DRONE_BODY_H);
  dims.setJSONObject("body", body);

  JSONObject arm = new JSONObject();
  arm.setFloat("length", DRONE_ARM_LENGTH);
  arm.setFloat("thickness", DRONE_ARM_THICKNESS);
  dims.setJSONObject("arm", arm);

  JSONObject motor = new JSONObject();
  motor.setFloat("radius", DRONE_MOTOR_RADIUS);
  motor.setFloat("height", DRONE_MOTOR_HEIGHT);
  dims.setJSONObject("motor", motor);

  JSONObject propeller = new JSONObject();
  propeller.setFloat("radius", DRONE_PROP_RADIUS);
  propeller.setFloat("thickness", DRONE_PROP_THICKNESS);
  dims.setJSONObject("propeller", propeller);

  JSONObject landingGear = new JSONObject();
  landingGear.setFloat("height", DRONE_LEG_HEIGHT);
  landingGear.setFloat("span", DRONE_LEG_SPAN);
  dims.setJSONObject("landingGear", landingGear);

  return dims;
}

// Builds drone visual offsets 3 d object.
JSONObject buildDroneVisualOffsets3DObject() {
  JSONObject offsets = new JSONObject();
  offsets.setFloat("restY", DRONE_REST_Y);
  offsets.setFloat("visualYawOffsetDeg", degrees(DRONE_VISUAL_YAW_OFFSET));
  offsets.setFloat("cameraY", DRONE_CAMERA_Y);
  offsets.setFloat("cameraZ", DRONE_CAMERA_Z);
  offsets.setFloat("lampY", DRONE_LAMP_Y);
  offsets.setFloat("lampZ", DRONE_LAMP_Z);
  return offsets;
}

// Builds drone sensor offsets 3 d object.
JSONObject buildDroneSensorOffsets3DObject() {
  JSONObject offsets = new JSONObject();

  JSONObject sonar = new JSONObject();
  sonar.setFloat("x", DRONE_SONAR_X);
  sonar.setFloat("y", DRONE_SONAR_Y);
  sonar.setFloat("z", DRONE_SONAR_Z);
  sonar.setString("direction", "down");
  sonar.setString("source", "sonar");
  offsets.setJSONObject("downwardSonar", sonar);

  return offsets;
}

// Builds drone config 3 d object.
JSONObject buildDroneConfig3DObject() {
  JSONObject config3D = new JSONObject();
  config3D.setJSONObject("dimensions", buildDroneDimensions3DObject());
  config3D.setJSONObject("visualOffsets", buildDroneVisualOffsets3DObject());
  config3D.setJSONObject("sensorOffsets", buildDroneSensorOffsets3DObject());
  return config3D;
}

// Applies default drone dimensions.
void applyDefaultDroneDimensions() {
  DRONE_BODY_L = DRONE_DEFAULT_BODY_L;
  DRONE_BODY_W = DRONE_DEFAULT_BODY_W;
  DRONE_BODY_H = DRONE_DEFAULT_BODY_H;
  DRONE_ARM_LENGTH = DRONE_DEFAULT_ARM_LENGTH;
  DRONE_ARM_THICKNESS = DRONE_DEFAULT_ARM_THICKNESS;
  DRONE_MOTOR_RADIUS = DRONE_DEFAULT_MOTOR_RADIUS;
  DRONE_MOTOR_HEIGHT = DRONE_DEFAULT_MOTOR_HEIGHT;
  DRONE_PROP_RADIUS = DRONE_DEFAULT_PROP_RADIUS;
  DRONE_PROP_THICKNESS = DRONE_DEFAULT_PROP_THICKNESS;
  DRONE_LEG_HEIGHT = DRONE_DEFAULT_LEG_HEIGHT;
  DRONE_LEG_SPAN = DRONE_DEFAULT_LEG_SPAN;
}

// Applies default drone visual offsets.
void applyDefaultDroneVisualOffsets() {
  DRONE_REST_Y = DRONE_DEFAULT_REST_Y;
  DRONE_VISUAL_YAW_OFFSET = radians(DRONE_DEFAULT_VISUAL_YAW_OFFSET_DEG);
  DRONE_CAMERA_Y = DRONE_DEFAULT_CAMERA_Y;
  DRONE_CAMERA_Z = DRONE_DEFAULT_CAMERA_Z;
  DRONE_LAMP_Y = DRONE_DEFAULT_LAMP_Y;
  DRONE_LAMP_Z = DRONE_DEFAULT_LAMP_Z;
  DRONE_SONAR_X = DRONE_DEFAULT_SONAR_X;
  DRONE_SONAR_Y = DRONE_DEFAULT_SONAR_Y;
  DRONE_SONAR_Z = DRONE_DEFAULT_SONAR_Z;
}

// Applies default drone config state.
void applyDefaultDroneConfigState() {
  applyDefaultDroneDimensions();
  applyDefaultDroneVisualOffsets();
  droneNavX = 0.0f;
  droneNavY = 0.0f;
  droneNavZ = 0.0f;
  droneNavYaw = 0.0f;
  dronePitch = 0.0f;
  droneRoll = 0.0f;
  updateDroneDimensions();
}

// Utility: refresh drone derived offsets defaults.
void refreshDroneDerivedOffsetsDefaults() {
  applyDefaultDroneVisualOffsets();
}

// Returns drone scene x.
float getDroneSceneX() {
  return DRONE_RENDER_ANCHORED ? 0 : droneNavX;
}

// Returns drone scene y.
float getDroneSceneY() {
  return DRONE_RENDER_ANCHORED ? 0 : droneNavY;
}

// Returns drone scene z.
float getDroneSceneZ() {
  return DRONE_RENDER_ANCHORED ? 0 : droneNavZ;
}

// Updates drone dimensions.
void updateDroneDimensions() {
  DRONE_BODY_DIM = max(max(DRONE_BODY_L, DRONE_BODY_W), DRONE_BODY_H / 0.28f);
  if (!droneIsAirborne()) {
    droneY = DRONE_REST_Y;
  }
}

// Builds drone config object.
JSONObject buildDroneConfigObject() {
  JSONObject json = new JSONObject();
  json.setString("schema", "synrov.robot_config.v2");
  json.setString("robot", "Drone");
  json.setString("geometryUnits", ROBOT_GEOMETRY_UNITS);
  json.setFloat("renderScale", DRONE_CM_TO_SCENE);
  json.setJSONObject("config3D", buildDroneConfig3DObject());

  JSONObject runtime = new JSONObject();
  runtime.setFloat("x", droneNavX);
  runtime.setFloat("y", droneNavY);
  runtime.setFloat("z", droneNavZ);
  runtime.setFloat("yawDeg", degrees(droneNavYaw));
  runtime.setFloat("pitchDeg", degrees(dronePitch));
  runtime.setFloat("rollDeg", degrees(droneRoll));
  json.setJSONObject("runtime", runtime);

  return json;
}

// Creates default drone config.
void createDefaultDroneConfig(String filename) {
  applyDefaultDroneConfigState();
  saveJSONObjectEnsured(buildDroneConfigObject(), filename);
}

// Applies drone dimensions JSON.
void applyDroneDimensionsJson(JSONObject dimsRoot) {
  if (dimsRoot == null) return;

  JSONObject body = getJsonObjectSafe(dimsRoot, "body");
  DRONE_BODY_L = getJsonFloat(body, "length", DRONE_BODY_L);
  DRONE_BODY_W = getJsonFloat(body, "width", DRONE_BODY_W);
  DRONE_BODY_H = getJsonFloat(body, "height", DRONE_BODY_H);

  JSONObject arm = getJsonObjectSafe(dimsRoot, "arm");
  DRONE_ARM_LENGTH = getJsonFloat(arm, "length", DRONE_ARM_LENGTH);
  DRONE_ARM_THICKNESS = getJsonFloat(arm, "thickness", DRONE_ARM_THICKNESS);

  JSONObject motor = getJsonObjectSafe(dimsRoot, "motor");
  DRONE_MOTOR_RADIUS = getJsonFloat(motor, "radius", DRONE_MOTOR_RADIUS);
  DRONE_MOTOR_HEIGHT = getJsonFloat(motor, "height", DRONE_MOTOR_HEIGHT);

  JSONObject propeller = getJsonObjectSafe(dimsRoot, "propeller");
  DRONE_PROP_RADIUS = getJsonFloat(propeller, "radius", DRONE_PROP_RADIUS);
  DRONE_PROP_THICKNESS = getJsonFloat(propeller, "thickness", DRONE_PROP_THICKNESS);

  JSONObject landingGear = getJsonObjectSafe(dimsRoot, "landingGear");
  DRONE_LEG_HEIGHT = getJsonFloat(landingGear, "height", DRONE_LEG_HEIGHT);
  DRONE_LEG_SPAN = getJsonFloat(landingGear, "span", DRONE_LEG_SPAN);
}

// Applies drone visual offsets JSON.
void applyDroneVisualOffsetsJson(JSONObject offsets) {
  if (offsets == null) {
    refreshDroneDerivedOffsetsDefaults();
    return;
  }

  DRONE_REST_Y = getJsonFloat(offsets, "restY", DRONE_REST_Y);
  DRONE_VISUAL_YAW_OFFSET = radians(getJsonFloat(offsets, "visualYawOffsetDeg", degrees(DRONE_VISUAL_YAW_OFFSET)));
  DRONE_CAMERA_Y = getJsonFloat(offsets, "cameraY", DRONE_CAMERA_Y);
  DRONE_CAMERA_Z = getJsonFloat(offsets, "cameraZ", DRONE_CAMERA_Z);
  DRONE_LAMP_Y = getJsonFloat(offsets, "lampY", DRONE_LAMP_Y);
  DRONE_LAMP_Z = getJsonFloat(offsets, "lampZ", DRONE_LAMP_Z);
}

// Applies drone sensor offsets JSON.
void applyDroneSensorOffsetsJson(JSONObject offsets) {
  if (offsets == null) return;
  JSONObject sonar = getJsonObjectSafe(offsets, "downwardSonar");
  if (sonar == null) return;

  DRONE_SONAR_X = getJsonFloat(sonar, "x", DRONE_SONAR_X);
  DRONE_SONAR_Y = getJsonFloat(sonar, "y", DRONE_SONAR_Y);
  DRONE_SONAR_Z = getJsonFloat(sonar, "z", DRONE_SONAR_Z);
}

// Checks whether drone config schema valid.
boolean isDroneConfigSchemaValid(JSONObject json) {
  if (json == null) return false;
  if (!"synrov.robot_config.v2".equals(getJsonString(json, "schema", ""))) return false;
  JSONObject config3D = getJsonObjectSafe(json, "config3D");
  if (config3D == null) return false;

  JSONObject dims = getJsonObjectSafe(config3D, "dimensions");
  JSONObject visualOffsets = getJsonObjectSafe(config3D, "visualOffsets");
  JSONObject sensorOffsets = getJsonObjectSafe(config3D, "sensorOffsets");
  if (dims == null || visualOffsets == null || sensorOffsets == null) return false;

  return
    getJsonObjectSafe(dims, "body") != null &&
    getJsonObjectSafe(dims, "arm") != null &&
    getJsonObjectSafe(dims, "motor") != null &&
    getJsonObjectSafe(dims, "propeller") != null &&
    getJsonObjectSafe(dims, "landingGear") != null &&
    getJsonObjectSafe(sensorOffsets, "downwardSonar") != null;
}

// Loads drone config.
void loadDroneConfig(String filename) {
  JSONObject json = loadJSONObjectWithRepair(filename, buildDroneConfigObject());
  if (json == null) return;

  JSONObject config3D = getJsonObjectSafe(json, "config3D");
  JSONObject dims = getJsonObjectSafe(config3D, "dimensions");
  JSONObject visualOffsets = getJsonObjectSafe(config3D, "visualOffsets");
  JSONObject sensorOffsets = getJsonObjectSafe(config3D, "sensorOffsets");

  applyDroneDimensionsJson(dims);
  applyDroneVisualOffsetsJson(visualOffsets);
  applyDroneSensorOffsetsJson(sensorOffsets);
  updateDroneDimensions();

  JSONObject runtime = getJsonObjectSafe(json, "runtime");
  if (runtime != null) {
    droneNavX = getJsonFloat(runtime, "x", droneNavX);
    droneNavY = getJsonFloat(runtime, "y", droneNavY);
    droneNavZ = getJsonFloat(runtime, "z", droneNavZ);
    droneNavYaw = radians(getJsonFloat(runtime, "yawDeg", degrees(droneNavYaw)));
    dronePitch = radians(getJsonFloat(runtime, "pitchDeg", degrees(dronePitch)));
    droneRoll = radians(getJsonFloat(runtime, "rollDeg", degrees(droneRoll)));
  }
}

// Loads or create drone config.
void loadOrCreateDroneConfig() {
  File dataDir = new File(sketchPath("data"));
  if (!dataDir.exists()) dataDir.mkdirs();

  if (!fileExists(DRONE3D_CONFIG_FILE)) {
    createDefaultDroneConfig(DRONE3D_CONFIG_FILE);
  }

  // The code is the default model. The file may override any subset of
  // parameters, but missing sections must not force a reset or NullPointer.
  loadJSONObjectWithRepair(DRONE3D_CONFIG_FILE, buildDroneConfigObject());
  loadDroneConfig(DRONE3D_CONFIG_FILE);
}

// Checks whether drone telemetry geometry.
boolean hasDroneTelemetryGeometry() {
  return latestSensors != null && (
    latestSensors.hasKey("drone_body_length") ||
    latestSensors.hasKey("drone_arm_length") ||
    latestSensors.hasKey("drone_motor_radius") ||
    latestSensors.hasKey("drone_sonar_y"));
}

// Applies drone telemetry geometry from sensors.
void applyDroneTelemetryGeometryFromSensors() {
  if (!hasDroneTelemetryGeometry()) return;

  DRONE_BODY_L = getSensorFloat("drone_body_length", DRONE_BODY_L);
  DRONE_BODY_W = getSensorFloat("drone_body_width", DRONE_BODY_W);
  DRONE_BODY_H = getSensorFloat("drone_body_height", DRONE_BODY_H);
  DRONE_ARM_LENGTH = getSensorFloat("drone_arm_length", DRONE_ARM_LENGTH);
  DRONE_ARM_THICKNESS = getSensorFloat("drone_arm_thickness", DRONE_ARM_THICKNESS);
  DRONE_MOTOR_RADIUS = getSensorFloat("drone_motor_radius", DRONE_MOTOR_RADIUS);
  DRONE_MOTOR_HEIGHT = getSensorFloat("drone_motor_height", DRONE_MOTOR_HEIGHT);
  DRONE_PROP_RADIUS = getSensorFloat("drone_prop_radius", DRONE_PROP_RADIUS);
  DRONE_PROP_THICKNESS = getSensorFloat("drone_prop_thickness", DRONE_PROP_THICKNESS);
  DRONE_LEG_HEIGHT = getSensorFloat("drone_leg_height", DRONE_LEG_HEIGHT);
  DRONE_LEG_SPAN = getSensorFloat("drone_leg_span", DRONE_LEG_SPAN);
  DRONE_REST_Y = getSensorFloat("drone_rest_y", DRONE_REST_Y);
  DRONE_VISUAL_YAW_OFFSET = radians(getSensorFloat("drone_visual_yaw_offset_deg", degrees(DRONE_VISUAL_YAW_OFFSET)));
  DRONE_CAMERA_Y = getSensorFloat("drone_camera_y", DRONE_CAMERA_Y);
  DRONE_CAMERA_Z = getSensorFloat("drone_camera_z", DRONE_CAMERA_Z);
  DRONE_LAMP_Y = getSensorFloat("drone_lamp_y", DRONE_LAMP_Y);
  DRONE_LAMP_Z = getSensorFloat("drone_lamp_z", DRONE_LAMP_Z);
  DRONE_SONAR_X = getSensorFloat("drone_sonar_x", DRONE_SONAR_X);
  DRONE_SONAR_Y = getSensorFloat("drone_sonar_y", DRONE_SONAR_Y);
  DRONE_SONAR_Z = getSensorFloat("drone_sonar_z", DRONE_SONAR_Z);
  updateDroneDimensions();
}

// Resets drone input state.
void resetDroneInputState() {
  droneThrottleCmd = 0;
  droneYawCmd = 0;
  dronePitchCmd = 0;
  droneRollCmd = 0;
  droneStrafeCmd = 0;
  droneForwardCmd = 0;
}

// Returns whether drone should use connected telemetry altitude.
boolean droneUsesConnectedAltitudeTelemetry() {
  return systemReady && !simulationMode;
}

// Returns local animation altitude limit in scene units.
float getDroneLocalMaxLift() {
  return max(0.0f, DRONE_LOCAL_MAX_LIFT);
}

// Drone helper for is airborne.
boolean droneIsAirborne() {
  return droneFlightLift > 1.5f || droneTargetLift > 1.5f;
}

// Stops drone motion.
void stopDroneMotion(boolean forceSend) {
  resetDroneInputState();
  sendDroneRuntimeCommand(forceSend);
}

// Returns the throttle command sent to hardware and physics.
// Positive throttle is boosted so every control type gets faster ascent.
float effectiveDroneThrottleCommand() {
  float cmd = constrain(droneThrottleCmd, -1.0f, 1.0f);
  if (cmd > 0.001f) {
    cmd = max(DRONE_ASCENT_MIN_EFFECTIVE_CMD, cmd * DRONE_ASCENT_COMMAND_GAIN);
  }
  return constrain(cmd, -1.0f, 1.0f);
}

class DroneRuntimeCommandState {
  int throttlePct;
  int yawPct;
  int pitchPct;
  int rollPct;
  int strafePct;
  int forwardPct;
  int camPanDeg;
  int camTiltDeg;
  int flags;
  boolean airborne;
}

// Checks the commanded drone movement against the shared environment guard.
boolean droneRuntimeMotionBlockedByCollision() {
  if (!environmentCollisionEnabled) return false;
  if (!droneIsAirborne()) return false;

  float yawStepNav = droneYawCmd * radians(2.05f);
  float motionYaw = droneNavYaw - yawStepNav * 0.5f;
  float semanticForward = -constrain(droneForwardCmd, -1.0f, 1.0f);
  float forwardSpeed = -semanticForward * 2.9f;
  float strafeSpeed = constrain(droneStrafeCmd, -1.0f, 1.0f) * 2.5f;
  float planarTraction = 1.0f - 0.18f * constrain(abs(droneYawCmd), 0.0f, 1.0f);
  forwardSpeed *= planarTraction;
  strafeSpeed *= planarTraction;

  if (abs(forwardSpeed) < 0.0001f && abs(strafeSpeed) < 0.0001f) return false;

  float proposedX = droneNavX + sin(motionYaw) * forwardSpeed + sin(motionYaw + HALF_PI) * strafeSpeed;
  float proposedZ = droneNavZ + cos(motionYaw) * forwardSpeed + cos(motionYaw + HALF_PI) * strafeSpeed;
  return wouldDroneEnvironmentCollide(proposedX, droneNavY, proposedZ);
}

// Emits a bounded notice when the drone command is held by the map guard.
void notifyDroneRuntimeCollisionBlock() {
  long now = millis();
  if ((now - droneLastCollisionBlockNoticeMs) < 700) return;
  droneLastCollisionBlockNoticeMs = now;
  updateMessage(tr("Movimento do drone bloqueado pela proteção de colisão.", "Drone motion blocked by the collision guard."));
}

// Returns the canonical drone runtime state shared by animation, ASCII and HEX commands.
DroneRuntimeCommandState buildDroneRuntimeCommandState() {
  return buildDroneRuntimeCommandState(true);
}

// Returns the drone runtime state, optionally applying the Processing guard before transport.
DroneRuntimeCommandState buildDroneRuntimeCommandState(boolean applyCollisionFilter) {
  DroneRuntimeCommandState state = new DroneRuntimeCommandState();
  state.airborne = droneIsAirborne();

  float throttle = effectiveDroneThrottleCommand();
  if (!state.airborne && throttle < 0.0f) throttle = 0.0f;

  boolean blockedByCollision = applyCollisionFilter && droneRuntimeMotionBlockedByCollision();
  droneRuntimeCollisionFiltered = blockedByCollision;

  state.throttlePct = constrain(round(throttle * 100.0f), -100, 100);
  state.yawPct = constrain(round((state.airborne ? droneYawCmd : 0.0f) * 100.0f), -100, 100);
  state.pitchPct = constrain(round((state.airborne && !blockedByCollision ? dronePitchCmd : 0.0f) * 100.0f), -100, 100);
  state.rollPct = constrain(round((state.airborne && !blockedByCollision ? droneRollCmd : 0.0f) * 100.0f), -100, 100);
  state.strafePct = constrain(round((state.airborne && !blockedByCollision ? droneStrafeCmd : 0.0f) * 100.0f), -100, 100);
  state.forwardPct = constrain(round((state.airborne && !blockedByCollision ? -droneForwardCmd : 0.0f) * 100.0f), -100, 100);
  state.camPanDeg = constrain(round(droneCameraPanDeg), round(droneCameraPanMinDegLimit()), round(droneCameraPanMaxDegLimit()));
  state.camTiltDeg = constrain(round(droneCameraTiltDeg), round(droneCameraTiltMinDegLimit()), round(droneCameraTiltMaxDegLimit()));
  state.flags = droneCameraStreamingEnabled ? 0x01 : 0x00;
  return state;
}

// Builds drone runtime command.
String buildDroneRuntimeCommand() {
  DroneRuntimeCommandState state = buildDroneRuntimeCommandState();
  return "FLY=" + state.throttlePct + "," + state.yawPct + "," + state.pitchPct + "," + state.rollPct + "," + state.strafePct + "," + state.forwardPct + ",CAM=" + state.camPanDeg + "," + state.camTiltDeg;
}

// Resets pending drone runtime command.
void resetPendingDroneRuntimeCommand() {
  droneRuntimeCommandSource = CONTROL_SOURCE_LOCAL;
  droneLastRuntimeSendMs = 0;
  droneLastRuntimeCommand = "";
  droneLastCameraStreamingCommandSent = !droneCameraStreamingEnabled;
  droneLastAuxRuntimeSendMs = 0;
}

// Flushes queued drone auxiliary intents without using a WebSocket serial shortcut.
void flushPendingDroneAuxRuntimeCommands() {
  if (!isDroneSelected) return;
  if (!(systemReady && !simulationMode) || myPort == null) return;
  if (serialMonitorSessionActive || hardwareStreamStoppedByExit) return;
  if (droneCameraStreamingEnabled == droneLastCameraStreamingCommandSent) return;

  long now = millis();
  if (droneLastAuxRuntimeSendMs > 0 && (now - droneLastAuxRuntimeSendMs) < 40) return;

  int previousSource = commandContextSource;
  setCommandContext(droneRuntimeCommandSource);
  try {
    if (sendHardwareStreamCommand("CAM=" + (droneCameraStreamingEnabled ? 1 : 0), "drone camera", false)) {
      droneLastCameraStreamingCommandSent = droneCameraStreamingEnabled;
      droneLastAuxRuntimeSendMs = now;
    }
  }
  finally {
    commandContextSource = previousSource;
  }
}

// Utility: flush pending drone runtime command.
void flushPendingDroneRuntimeCommand(boolean force) {
  if (!isDroneSelected) return;
  if (!(systemReady && !simulationMode) || myPort == null) return;
  if (serialMonitorSessionActive || hardwareStreamStoppedByExit) return;

  long now = millis();
  boolean blockedByCollision = droneRuntimeMotionBlockedByCollision();
  if (blockedByCollision) notifyDroneRuntimeCollisionBlock();

  String cmd = buildDroneRuntimeCommand();
  boolean changed = !cmd.equals(droneLastRuntimeCommand);
  boolean resendPending = droneRuntimeResendIntervalMs > 0 &&
    droneLastRuntimeSendMs > 0 &&
    (now - droneLastRuntimeSendMs) >= droneRuntimeResendIntervalMs;
  boolean keepAliveDue = droneLastRuntimeSendMs == 0 ||
    (now - droneLastRuntimeSendMs) >= HARDWARE_STREAM_KEEPALIVE_MS;

  if (!force && !changed && !resendPending && !keepAliveDue) return;

  if (!force) {
    int minInterval = changed ? droneRuntimeSendIntervalMs : min(droneRuntimeResendIntervalMs, HARDWARE_STREAM_KEEPALIVE_MS);
    if (minInterval > 0 && droneLastRuntimeSendMs > 0 && (now - droneLastRuntimeSendMs) < minInterval) return;
  }

  int previousSource = commandContextSource;
  setCommandContext(droneRuntimeCommandSource);
  try {
    if (sendHardwareStreamCommand(cmd, "drone runtime", false)) {
      droneLastRuntimeCommand = cmd;
      droneLastRuntimeSendMs = now;
    }
    flushPendingDroneAuxRuntimeCommands();
  }
  finally {
    commandContextSource = previousSource;
  }
}

// Queues a drone runtime intent for the shared Processing control pipeline.
void sendDroneRuntimeCommand(boolean force) {
  droneRuntimeCommandSource = commandContextSource;
  droneRuntimeForcePending = droneRuntimeForcePending || force;
}

// Utility: trigger drone take off.
void triggerDroneTakeOff() {
  resetDroneInputState();
  droneTargetLift = max(droneTargetLift, 60.0f);
  droneThrottleCmd = max(droneThrottleCmd, 0.35f);
  sendDroneRuntimeCommand(true);
  updateMessage(tr("Decolagem do drone solicitada.", "Drone takeoff requested."));
}


// Utility: trigger drone land.
void triggerDroneLand() {
  resetDroneInputState();
  droneTargetLift = 0;
  droneThrottleCmd = -0.32f;
  sendDroneRuntimeCommand(true);
  updateMessage(tr("Pouso automático do drone solicitado.", "Drone auto landing requested."));
}

// Updates drone physics.
void updateDronePhysics() {
  long now = millis();
  if (droneLastTickMs == 0) droneLastTickMs = now;
  float dt = max(0.001f, (now - droneLastTickMs) / 1000.0f);
  droneLastTickMs = now;

  DroneRuntimeCommandState runtimeState = buildDroneRuntimeCommandState();

  droneYawCmdFiltered = lerp(droneYawCmdFiltered, runtimeState.yawPct / 100.0f, 0.12f);
  dronePitchCmdFiltered = lerp(dronePitchCmdFiltered, runtimeState.pitchPct / 100.0f, 0.11f);
  droneRollCmdFiltered = lerp(droneRollCmdFiltered, runtimeState.rollPct / 100.0f, 0.11f);
  droneThrottleCmdFiltered = lerp(droneThrottleCmdFiltered, runtimeState.throttlePct / 100.0f, 0.13f);
  droneStrafeCmdFiltered = lerp(droneStrafeCmdFiltered, runtimeState.strafePct / 100.0f, 0.12f);
  droneForwardCmdFiltered = lerp(droneForwardCmdFiltered, -runtimeState.forwardPct / 100.0f, 0.12f);

  boolean airborne = runtimeState.airborne;
  float semanticForward = -droneForwardCmdFiltered;

  float yawStepNav = 0.0f;
  if (airborne) {
    yawStepNav = droneYawCmdFiltered * radians(2.05f) * dt * 60.0f;
    droneNavYaw += yawStepNav;
  }
  float targetVisualYaw = airborne ? droneYawCmdFiltered * radians(8.0f) : 0.0f;
  droneYaw = lerp(droneYaw, targetVisualYaw, airborne ? 0.10f : 0.08f);

  float targetPitchDeg = 0;
  float targetRollDeg = 0;
  if (airborne) {
    targetPitchDeg = constrain((dronePitchCmdFiltered + semanticForward * 1.18f) * 13.0f, -17.0f, 17.0f);
    targetRollDeg = constrain((droneRollCmdFiltered + droneStrafeCmdFiltered * 1.18f) * 13.0f, -17.0f, 17.0f);
    if (!droneStabilizeEnabled) {
      targetPitchDeg *= 1.30f;
      targetRollDeg *= 1.30f;
    }
  }

  dronePitch = lerp(dronePitch, radians(targetPitchDeg), droneStabilizeEnabled ? 0.11f : 0.07f);
  droneRoll = lerp(droneRoll, radians(targetRollDeg), droneStabilizeEnabled ? 0.11f : 0.07f);

  boolean useConnectedAltitudeTelemetry = droneUsesConnectedAltitudeTelemetry();
  float liftRate = 2.20f * dt * 60.0f;
  if (!useConnectedAltitudeTelemetry) {
    if (droneThrottleCmdFiltered > 0.02f) {
      droneTargetLift = min(getDroneLocalMaxLift(), droneTargetLift + droneThrottleCmdFiltered * liftRate);
    } else if (droneThrottleCmdFiltered < -0.02f) {
      droneTargetLift = max(0, droneTargetLift + droneThrottleCmdFiltered * liftRate);
    }

    droneFlightLift = lerp(droneFlightLift, droneTargetLift, droneTargetLift >= droneFlightLift ? 0.12f : 0.10f);
    if (droneFlightLift < 0.05f && droneTargetLift < 0.05f) {
      droneFlightLift = 0;
      droneTargetLift = 0;
    }
  } else {
    float connectedLift = max(0.0f, droneAltitudeCm * 0.5f);
    droneTargetLift = connectedLift;
    droneFlightLift = connectedLift;
  }
  droneY = DRONE_REST_Y - droneFlightLift;

  float navStepX = 0;
  float navStepZ = 0;
  float navStepY = 0;
  if (airborne) {
    float motionYaw = droneNavYaw - yawStepNav * 0.5f;
    float forwardSpeed = -semanticForward * 2.9f * dt * 60.0f;
    float strafeSpeed = droneStrafeCmdFiltered * 2.5f * dt * 60.0f;
    float planarTraction = 1.0f - 0.18f * constrain(abs(droneYawCmdFiltered), 0.0f, 1.0f);
    forwardSpeed *= planarTraction;
    strafeSpeed *= planarTraction;

    navStepX = sin(motionYaw) * forwardSpeed + sin(motionYaw + HALF_PI) * strafeSpeed;
    navStepZ = cos(motionYaw) * forwardSpeed + cos(motionYaw + HALF_PI) * strafeSpeed;
  }

  float proposedX = droneNavX + navStepX;
  float proposedY = droneNavY + navStepY;
  float proposedZ = droneNavZ + navStepZ;
  if (!wouldDroneEnvironmentCollide(proposedX, proposedY, proposedZ)) {
    droneNavX = proposedX;
    droneNavY = proposedY;
    droneNavZ = proposedZ;
  }

  boolean propsActive = airborne || abs(droneThrottleCmdFiltered) > 0.02f;
  float rotorTarget = propsActive ? DRONE_ROTOR_FLIGHT_SPEED : 0.0f;
  float rotorLerp = rotorTarget > droneRotorSpeed ? DRONE_ROTOR_SPOOL_UP_LERP : DRONE_ROTOR_SPOOL_DOWN_LERP;
  droneRotorSpeed = lerp(droneRotorSpeed, rotorTarget, rotorLerp);
  dronePropPhase = (dronePropPhase + droneRotorSpeed) % 360.0f;
  droneScannerSweepDeg = (droneScannerSweepDeg + (droneCameraStreamingEnabled ? 2.0f : 0.65f)) % 360.0f;

  if (!useConnectedAltitudeTelemetry) {
    droneHeadingTelemetryDeg = degrees(droneNavYaw);
    droneAltitudeCm = constrain(max(0, droneFlightLift * 2.0f), 0, DRONE_LOCAL_MAX_ALTITUDE_CM);
  }

}

// Utility: draw drone propeller.
void drawDronePropeller(float rotationAngle, float rotorSpeed) {
  float blurMix = constrain(
    map(rotorSpeed, DRONE_ROTOR_BLUR_START_SPEED, DRONE_ROTOR_BLUR_FULL_SPEED, 0.0f, 1.0f),
    0.0f,
    1.0f
  );
  float bladeAlpha = lerp(210.0f, 70.0f, blurMix);
  float blurAlpha = 120.0f * blurMix;

  pushMatrix();
  rotateY(rotationAngle);
  noStroke();
  fill(18, 18, 18, bladeAlpha);
  box(DRONE_PROP_RADIUS * 2.0f, DRONE_PROP_THICKNESS, DRONE_PROP_RADIUS * 0.36f);
  rotateY(HALF_PI);
  box(DRONE_PROP_RADIUS * 2.0f, DRONE_PROP_THICKNESS, DRONE_PROP_RADIUS * 0.36f);
  safePopMatrix("Drone3D.pde:599");

  if (blurMix > 0.001f) {
    pushMatrix();
    rotateX(HALF_PI);
    noStroke();
    fill(40, 40, 40, blurAlpha * 0.42f);
    ellipse(0, 0, DRONE_PROP_RADIUS * 2.22f, DRONE_PROP_RADIUS * 2.22f);
    fill(115, 115, 115, blurAlpha * 0.18f);
    ellipse(0, 0, DRONE_PROP_RADIUS * 1.50f, DRONE_PROP_RADIUS * 1.50f);
    safePopMatrix("Drone3D.pde:607");
  }
}

// Utility: draw drone motor.
void drawDroneMotor() {
  fill(28, 28, 28);
  noStroke();
  pushMatrix();
  rotateX(HALF_PI);
  drawCylinder(DRONE_MOTOR_RADIUS, DRONE_MOTOR_HEIGHT);
  safePopMatrix("Drone3D.pde:589");
}

// Utility: draw drone ground.
void drawDroneGround() {
  float headingDeg = systemReady ? droneHeadingTelemetryDeg : degrees(droneNavYaw);
  drawSharedGroundPlane(visualGroundSizeForCurrentRobot(), headingDeg, true);
}

// Utility: draw drone 3 d.
void drawDrone3D(boolean advanceCamera) {
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

  drawDroneGround();
  applySceneLighting();
  drawCurrentEnvironmentMap();
  drawDroneDownwardSonarProbe();
  noStroke();

  pushMatrix();
  translate(getDroneSceneX(), getDroneSceneY() + droneY, getDroneSceneZ());
  scale(DRONE_VISUAL_RENDER_SCALE);
  float droneRenderYaw = DRONE_RENDER_ANCHORED ? 0.0f : droneNavYaw;
  rotateY(droneRenderYaw + droneYaw + DRONE_VISUAL_YAW_OFFSET);
  rotateX(dronePitch);
  rotateZ(droneRoll);

  fill(173, 55, 247);
  box(DRONE_BODY_L, DRONE_BODY_H, DRONE_BODY_W);

  pushMatrix();
  translate(0, -8, 0);
  fill(200, 100, 100);
  box(DRONE_BODY_L * 0.62f, DRONE_BODY_H * 0.58f, DRONE_BODY_W * 0.82f);
  safePopMatrix("Drone3D.pde:637");

  for (int i = 0; i < 4; i++) {
    float angle = radians(45 + i * 90);
    float armX = cos(angle) * DRONE_ARM_LENGTH;
    float armZ = sin(angle) * DRONE_ARM_LENGTH;

    pushMatrix();
    rotateY(angle);
    translate(DRONE_ARM_LENGTH * 0.5f, 0, 0);
    fill(100, 200, 100);
    box(DRONE_ARM_LENGTH, DRONE_ARM_THICKNESS, DRONE_ARM_THICKNESS);
    safePopMatrix("Drone3D.pde:649");

    pushMatrix();
    translate(armX, 0, armZ);
    drawDroneMotor();

    pushMatrix();
    translate(0, -DRONE_MOTOR_HEIGHT * 0.5f - DRONE_PROP_THICKNESS * 0.5f, 0);
    float motorMix = radians(dronePropPhase) * ((i % 2 == 0) ? 1.0f : -1.0f);
    drawDronePropeller(motorMix, droneRotorSpeed);
    safePopMatrix("Drone3D.pde:679");
    safePopMatrix("Drone3D.pde:680");
  }

  pushMatrix();
  translate(0, DRONE_CAMERA_Y, DRONE_CAMERA_Z + 3);
  rotateY(radians(droneCameraPanDeg));
  rotateX(radians(-droneCameraTiltDeg));
  fill(100, 100, 200);
  sphere(2);
  pushMatrix();
  translate(0, 0, 1.8);
  fill(droneCameraStreamingEnabled ? color(255, 220, 0) : color(80, 80, 70));
  sphere(0.4);
  safePopMatrix("Drone3D.pde:671");
  safePopMatrix("Drone3D.pde:672");

  pushMatrix();
  translate(0, DRONE_LAMP_Y, DRONE_LAMP_Z);
  fill(255, 220, 70);
  box(12, 2, 6);
  safePopMatrix("Drone3D.pde:678");

  pushMatrix();
  translate(DRONE_SONAR_X, DRONE_SONAR_Y, DRONE_SONAR_Z);
  fill(70, 210, 255);
  box(7, 2, 7);
  stroke(70, 210, 255, 150);
  strokeWeight(1.2f);
  line(0, 1.2f, 0, 0, 16.0f, 0);
  noStroke();
  safePopMatrix("Drone3D.pde:687");

  for (int side = -1; side <= 1; side += 2) {
    pushMatrix();
    translate(side * DRONE_LEG_SPAN, DRONE_LEG_HEIGHT, 0);
    fill(100, 100, 200);
    box(3, 3, DRONE_BODY_W * 0.80f);
    safePopMatrix("Drone3D.pde:685");

    pushMatrix();
    translate(side * DRONE_LEG_SPAN, DRONE_LEG_HEIGHT * 0.72f, -DRONE_LEG_SPAN);
    box(3, DRONE_LEG_HEIGHT, 3);
    safePopMatrix("Drone3D.pde:690");

    pushMatrix();
    translate(side * DRONE_LEG_SPAN, DRONE_LEG_HEIGHT * 0.72f, DRONE_LEG_SPAN);
    box(3, DRONE_LEG_HEIGHT, 3);
    safePopMatrix("Drone3D.pde:695");
  }

  safePopMatrix("Drone3D.pde:698");
  safePopMatrix("Drone3D.pde:699");
  noLights();
}

// Utility: capture drone frame.
int[] captureDroneFrame() {
  return new int[] {
    round(droneYawCmd * 100.0f),
    0,
    0,
    round(droneThrottleCmd * 100.0f),
    round(droneStrafeCmd * 100.0f),
    round(droneForwardCmd * 100.0f)
  };
}

// Applies drone frame.
void applyDroneFrame(int[] frame) {
  if (frame == null || frame.length < 6) return;
  droneYawCmd = constrain(frame[0] / 100.0f, -1, 1);
  dronePitchCmd = 0;
  droneRollCmd = 0;
  droneThrottleCmd = constrain(frame[3] / 100.0f, -1, 1);
  droneStrafeCmd = constrain(frame[4] / 100.0f, -1, 1);
  droneForwardCmd = constrain(frame[5] / 100.0f, -1, 1);
  sendDroneRuntimeCommand(true);
}

// Utility: move drone safely toward frame.
boolean moveDroneSafelyTowardFrame(int[] targetFrame, int maxStep, int tolerance) {
  if (targetFrame == null || targetFrame.length < 6) return true;

  droneYawCmd = approachFloat(droneYawCmd * 100.0f, targetFrame[0], maxStep) / 100.0f;
  dronePitchCmd = 0;
  droneRollCmd = 0;
  droneThrottleCmd = approachFloat(droneThrottleCmd * 100.0f, targetFrame[3], maxStep) / 100.0f;
  droneStrafeCmd = approachFloat(droneStrafeCmd * 100.0f, targetFrame[4], maxStep) / 100.0f;
  droneForwardCmd = approachFloat(droneForwardCmd * 100.0f, targetFrame[5], maxStep) / 100.0f;
  sendDroneRuntimeCommand(false);

  return abs(droneYawCmd * 100.0f - targetFrame[0]) <= tolerance &&
    abs(droneThrottleCmd * 100.0f - targetFrame[3]) <= tolerance &&
    abs(droneStrafeCmd * 100.0f - targetFrame[4]) <= tolerance &&
    abs(droneForwardCmd * 100.0f - targetFrame[5]) <= tolerance;
}


void drawDrone3D() {
  drawDrone3D(true);
}


// =====================================================================
// Drone diagnostics module section
// =====================================================================

boolean handleDroneDiagnosticsPanelMousePressed(int mx, int my, int panelX, int panelY, int contentX, int contentW, int gap, int toolsInnerX, int toolsCol2W) {
  if (pointInRect(mx, my, toolsInnerX, diagnosticsMapButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { toggleRemoteCollisionFromPanel(); return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsMapButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { clearTraceMap(); return true; }
  if (pointInRect(mx, my, toolsInnerX, diagnosticsMapButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { loadWorldMapFromWindowsDialog(); return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsMapButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { removeImportedWorldOnly(); return true; }

  if (pointInRect(mx, my, toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { triggerDroneTakeOff(); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { triggerDroneLand(); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { toggleDroneCamera(); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { toggleDroneDownwardSonar(); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 2), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { triggerCompassCalibration(); return true; }
  return true;
}

void drawDroneDiagnosticsPanelSections(int panelY, int contentX, int contentW, int gap, int toolsInnerX, int toolsCol2W, int sensorsContentX, int sensorsContentW, int droneRuntimeCardY, int droneSensorsCardY) {
  drawSectionCard2D(contentX, droneRuntimeCardY, contentW, DIAGNOSTICS_DRONE_RUNTIME_CARD_H, tr("Runtime do drone", "Drone runtime"));
  drawMiniButton2D(toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Decolar", "Take off"), false);
  drawMiniButton2D(toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Pousar", "Land"), false);
  drawMiniButton2D(toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Câmera", "Camera"), droneCameraStreamingEnabled);
  drawMiniButton2D(toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Sonar inferior", "Downward sonar"), currentEnvironmentAutoScanEnabled());
  drawMiniButton2D(toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 2), toolsCol2W, DIAGNOSTICS_BUTTON_H, compassCalibrationActive() ? tr("Cal. bússola *", "Compass cal *") : tr("Cal. bússola", "Compass cal"), compassCalibrationActive());

  drawSectionCard2D(contentX, droneSensorsCardY, contentW, DIAGNOSTICS_DRONE_SENSORS_CARD_H, tr("Sensores de estabilidade do drone", "Drone stability sensors"));
  int droneStabilityWidgetH = DIAGNOSTICS_DRONE_SENSORS_CARD_H - 144;
  drawDroneStabilityTargets2D(sensorsContentX, droneSensorsCardY + 32, sensorsContentW, droneStabilityWidgetH);
  fill(uiPrimaryTextColor());
  int droneInfoY = droneSensorsCardY + 32 + droneStabilityWidgetH + 12;
  drawDiagnosticsFittedTextLeft(diagnosticsCompassSummaryText(), sensorsContentX, droneInfoY, sensorsContentW);
  drawDiagnosticsFittedTextLeft(tr("Posição: firmware/odometria interna pela bússola", "Position: firmware/internal odometry by compass"), sensorsContentX, droneInfoY + 18, sensorsContentW);
  drawPressureBar2D(sensorsContentX, droneInfoY + 42, sensorsContentW, 18, droneBatteryPct, 100, tr("Bateria", "Battery"));
  drawDiagnosticsFittedTextLeft(tr("Bateria bruta / Mundo: ", "Battery raw / World: ") + nf(getSensorFloat("battery_raw", getSensorFloat("battery", 0)), 1, 0) + " / " + currentEnvironmentWorldSummary(), sensorsContentX, droneInfoY + 70, sensorsContentW);
}

// =====================================================================
// SynRovModule adapter: Drone3D
// =====================================================================
class Drone3DModule implements SynRovModule {
  public String id() { return "Drone"; }
  public String displayName() { return robotDisplayName("Drone"); }
  public boolean available() { return true; }

  public boolean supports(String capability) {
    return MODULE_CAP_DIAGNOSTICS.equals(capability) ||
      MODULE_CAP_ENVIRONMENT.equals(capability) ||
      MODULE_CAP_RUNTIME.equals(capability) ||
      MODULE_CAP_COLLISION.equals(capability) ||
      MODULE_CAP_TRAJECTORY.equals(capability) ||
      MODULE_CAP_CAMERA.equals(capability) ||
      MODULE_CAP_GPS.equals(capability) ||
      MODULE_CAP_FLIGHT.equals(capability);
  }

  public void setupModule() { }

  public void loadConfigSafe() {
    try {
      loadOrCreateDroneConfig();
    }
    catch (Exception e) {
      println("Drone config recovery: " + e.getMessage());
      backupBrokenConfigFile(DRONE3D_CONFIG_FILE);
      saveJSONObjectEnsured(buildDroneConfigObject(), DRONE3D_CONFIG_FILE);
      try { loadDroneConfig(DRONE3D_CONFIG_FILE); }
      catch (Exception ignore) { resetDroneInputState(); updateDroneDimensions(); }
    }
  }

  public void updateModule() { }

  public void updatePhysicsSafe() {
    updateDronePhysics();
  }

  public void drawModule3D(boolean advanceCamera) {
    drawDrone3D(advanceCamera);
  }

  public void resetInputStateSafe() {
    resetDroneInputState();
  }

  public int[] captureFrameSafe() {
    return captureDroneFrame();
  }

  public void applyFrameSafe(int[] frame) {
    applyDroneFrame(frame);
  }

  public boolean moveSafelyTowardFrameSafe(int[] targetFrame, int maxStepDeg, int toleranceDeg) {
    return moveDroneSafelyTowardFrame(targetFrame, maxStepDeg, toleranceDeg);
  }

  public JSONObject buildConfigObjectSafe() {
    return buildDroneConfigObject();
  }
}
