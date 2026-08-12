// =====================================================================
// SynROV Processing - Drone 3D model
// ---------------------------------------------------------------------
// Purpose:
//   Drone geometry, scene representation and 3D rendering helpers.
// =====================================================================

final float DRONE_ROTOR_FLIGHT_SPEED = 43.0f;
final float DRONE_ROTOR_SPOOL_UP_LERP = 0.14f;
final float DRONE_ROTOR_SPOOL_DOWN_LERP = 0.09f;
final float DRONE_ROTOR_BLUR_START_SPEED = 12.0f;
final float DRONE_ROTOR_BLUR_FULL_SPEED = 34.0f;

// Commercial flight profile. Scene units are provided by the shared scene
// environment module so navigation, the 2 km operating area and 1 km ceiling
// use one canonical meter-to-scene conversion.
final float DRONE_COMMERCIAL_FORWARD_SPEED_MPS = 8.0f;
final float DRONE_COMMERCIAL_STRAFE_SPEED_MPS = 7.0f;
final float DRONE_COMMERCIAL_ASCENT_SPEED_MPS = 4.0f;
final float DRONE_COMMERCIAL_DESCENT_SPEED_MPS = 3.0f;
final float DRONE_DEFAULT_MAX_MOTOR_RPM = 12000.0f;
float droneMaxMotorRpm = DRONE_DEFAULT_MAX_MOTOR_RPM;
final float DRONE_FORWARD_COMMAND_GAIN = 1.55f;
final float DRONE_STRAFE_COMMAND_GAIN = 1.65f;
final float DRONE_DESCENT_COMMAND_GAIN = 1.35f;
final float DRONE_DESCENT_MIN_EFFECTIVE_CMD = 0.18f;
// These values depend on the shared scene module. Keep them as runtime helpers
// instead of global field initializers so Processing tab order cannot create
// Java illegal-forward-reference errors during preprocessing.
float droneMotorSpeedScale() {
  return constrain(droneMaxMotorRpm / DRONE_DEFAULT_MAX_MOTOR_RPM, 0.05f, 4.0f);
}

float droneForwardSceneStep60Hz() {
  return DRONE_COMMERCIAL_FORWARD_SPEED_MPS * droneMotorSpeedScale() * sceneUnitsPerMeterForRobotMode(ROBOT_MODE_DRONE) / 60.0f;
}

float droneStrafeSceneStep60Hz() {
  return DRONE_COMMERCIAL_STRAFE_SPEED_MPS * droneMotorSpeedScale() * sceneUnitsPerMeterForRobotMode(ROBOT_MODE_DRONE) / 60.0f;
}
float droneAscentLiftStep60Hz() {
  return DRONE_COMMERCIAL_ASCENT_SPEED_MPS * droneMotorSpeedScale() * 100.0f * 0.5f / 60.0f;
}
float droneDescentLiftStep60Hz() {
  return DRONE_COMMERCIAL_DESCENT_SPEED_MPS * droneMotorSpeedScale() * 100.0f * 0.5f / 60.0f;
}

void resetDroneCameraCenter() {
  droneCameraPanDeg = 0.0f;
  droneCameraTiltDeg = 0.0f;
}

float droneEstimatedAverageMotorPowerPct() {
  if (latestSensors != null && latestSensors.hasKey("drone_motor_fl_us")) {
    float sum = 0.0f;
    String[] keys = {"drone_motor_fl_us", "drone_motor_fr_us", "drone_motor_rl_us", "drone_motor_rr_us"};
    float escMin = getSensorFloat("drone_esc_min_us", 1000.0f);
    float escMax = max(escMin + 1.0f, getSensorFloat("drone_esc_max_us", 2000.0f));
    for (int i = 0; i < keys.length; i++) sum += constrain((getSensorFloat(keys[i], escMin) - escMin) / (escMax - escMin), 0.0f, 1.0f);
    return (sum / keys.length) * 100.0f;
  }
  return constrain(abs(droneThrottleCmdFiltered), 0.0f, 1.0f) * 100.0f;
}

float droneEstimatedAverageMotorRpm() {
  return droneMaxMotorRpm * droneEstimatedAverageMotorPowerPct() / 100.0f;
}

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
final int ALT_IDX = 17;
final int STRAFE_IDX = 18;
final int FWD_IDX = 19;
final int DRONE_CAM_PAN_IDX = 20;
final int DRONE_CAM_TILT_IDX = 21;

float droneNavX = 0;
float droneNavY = 0;
float droneNavZ = 0;
float droneNavYaw = 0;

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

final float DRONE_ASCENT_COMMAND_GAIN = 1.45f;
final float DRONE_ASCENT_MIN_EFFECTIVE_CMD = 0.20f;

float droneYawCmdFiltered = 0;
float dronePitchCmdFiltered = 0;
float droneRollCmdFiltered = 0;
float droneThrottleCmdFiltered = 0;
float droneStrafeCmdFiltered = 0;
float droneForwardCmdFiltered = 0;

float droneFlightLift = 0;
float droneTargetLift = 0;

final int DRONE_FLIGHT_GROUNDED = 0;
final int DRONE_FLIGHT_TAKING_OFF = 1;
final int DRONE_FLIGHT_AIRBORNE = 2;
final int DRONE_FLIGHT_LANDING = 3;
final float DRONE_PHYSICAL_AIRBORNE_LIFT = 1.5f;
final float DRONE_PLANAR_UNLOCK_LIFT = 12.0f;
final float DRONE_AUTO_TAKEOFF_LIFT = 60.0f;
final float DRONE_AUTO_TAKEOFF_COMPLETE_TOLERANCE = 3.0f;
final float DRONE_LANDED_LIFT_EPS = 0.75f;
final float DRONE_AUTO_TAKEOFF_THROTTLE = 0.42f;
final float DRONE_AUTO_LANDING_THROTTLE = -0.38f;
int droneFlightPhase = DRONE_FLIGHT_GROUNDED;
boolean droneAutoTakeoffActive = false;
boolean droneAutoLandingActive = false;
float droneAutoTakeoffTargetLift = DRONE_AUTO_TAKEOFF_LIFT;

float dronePropPhase = 0;
float droneScannerSweepDeg = 0;
boolean droneStabilizeEnabled = true;
boolean droneCameraStreamingEnabled = false;
boolean droneLastCameraStreamingCommandSent = false;
long droneLastAuxRuntimeSendMs = 0;
boolean droneRuntimeCollisionFiltered = false;
long droneLastCollisionBlockNoticeMs = 0;
float droneCameraPanDeg = 0;
float droneCameraTiltDeg = 0;
final float DRONE_CAMERA_PAN_LIMIT = 60.0f;
final float DRONE_CAMERA_TILT_MIN = -20.0f;
final float DRONE_CAMERA_TILT_MAX = 20.0f;
float droneRotorSpeed = 0;

float droneCameraPanMinDegLimit() { return -DRONE_CAMERA_PAN_LIMIT; }
float droneCameraPanMaxDegLimit() { return DRONE_CAMERA_PAN_LIMIT; }
float droneCameraTiltMinDegLimit() { return DRONE_CAMERA_TILT_MIN; }
float droneCameraTiltMaxDegLimit() { return DRONE_CAMERA_TILT_MAX; }

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
  resetDroneCameraCenter();
  droneMaxMotorRpm = DRONE_DEFAULT_MAX_MOTOR_RPM;
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
  json.setString("schema", "synrov.robot_config");
  json.setInt("softwareVersion", SYNROV_SOFTWARE_VERSION);
  json.setString("robot", "Drone");
  json.setString("geometryUnits", ROBOT_GEOMETRY_UNITS);
  json.setFloat("renderScale", DRONE_CM_TO_SCENE);
  json.setJSONObject("config3D", buildDroneConfig3DObject());

  JSONObject motion = new JSONObject();
  motion.setFloat("maxMotorRpm", droneMaxMotorRpm);
  json.setJSONObject("motion", motion);

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
  if (!isRobotConfigHeader(json, "Drone")) return false;
  JSONObject config3D = getJsonObjectSafe(json, "config3D");
  if (config3D == null) return false;

  JSONObject dims = getJsonObjectSafe(config3D, "dimensions");
  JSONObject visualOffsets = getJsonObjectSafe(config3D, "visualOffsets");
  JSONObject sensorOffsets = getJsonObjectSafe(config3D, "sensorOffsets");
  if (dims == null || visualOffsets == null || sensorOffsets == null) return false;
  JSONObject motion = getJsonObjectSafe(json, "motion");
  if (motion == null || !motion.hasKey("maxMotorRpm")) return false;

  return hasJsonObjects(dims, "body", "arm", "motor", "propeller", "landingGear") &&
    hasJsonObjects(sensorOffsets, "downwardSonar");
}

// Loads drone config.
void loadDroneConfig(String filename) {
  JSONObject defaults = buildDroneConfigObject();
  JSONObject json = loadJsonOrNull(filename);
  if (json == null || !isDroneConfigSchemaValid(json)) {
    json = defaults;
    saveJSONObjectEnsured(json, filename);
  }

  JSONObject config3D = getJsonObjectSafe(json, "config3D");
  JSONObject dims = getJsonObjectSafe(config3D, "dimensions");
  JSONObject visualOffsets = getJsonObjectSafe(config3D, "visualOffsets");
  JSONObject sensorOffsets = getJsonObjectSafe(config3D, "sensorOffsets");

  applyDroneDimensionsJson(dims);
  applyDroneVisualOffsetsJson(visualOffsets);
  applyDroneSensorOffsetsJson(sensorOffsets);
  JSONObject motion = getJsonObjectSafe(json, "motion");
  droneMaxMotorRpm = constrain(getJsonFloat(motion, "maxMotorRpm", DRONE_DEFAULT_MAX_MOTOR_RPM), 1000.0f, 100000.0f);
  resetDroneCameraCenter();
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

// Loads or creates the drone config.
void loadOrCreateDroneConfig() {
  if (!fileExists(DRONE_CONFIG_FILE)) {
    createDefaultDroneConfig(DRONE_CONFIG_FILE);
  }
  loadDroneConfig(DRONE_CONFIG_FILE);
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
  droneMaxMotorRpm = constrain(getSensorFloat("drone_esc_max_motor_rpm", droneMaxMotorRpm), 1000.0f, 100000.0f);
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

// Returns whether firmware telemetry should own the drone altitude.
// Local telemetry keeps the local flight model authoritative even while the
// serial link is connected, so the 3D drone follows locally issued commands.
boolean droneUsesConnectedAltitudeTelemetry() {
  return isFirmwareTelemetrySelected() && systemReady && !simulationMode;
}

// Returns local animation altitude limit in scene units.
float getDroneLocalMaxLift() {
  return max(0.0f, maxAltitudeSceneForRobotMode(ROBOT_MODE_DRONE));
}

// Returns measured/local physical lift; target altitude never counts as airborne.
float droneCurrentPhysicalLift() {
  if (droneUsesConnectedAltitudeTelemetry()) {
    return constrain(droneAltitudeCm * 0.5f, 0.0f, getDroneLocalMaxLift());
  }
  return constrain(droneFlightLift, 0.0f, getDroneLocalMaxLift());
}

// Drone helper for physical airborne state.
boolean droneIsAirborne() {
  return droneCurrentPhysicalLift() > DRONE_PHYSICAL_AIRBORNE_LIFT;
}

// Horizontal/yaw motion is unlocked only after a real minimum flight height.
boolean dronePlanarMotionReady() {
  return droneCurrentPhysicalLift() >= DRONE_PLANAR_UNLOCK_LIFT && !droneAutoLandingActive;
}

String droneFlightPhaseName() {
  if (droneFlightPhase == DRONE_FLIGHT_TAKING_OFF) return "taking_off";
  if (droneFlightPhase == DRONE_FLIGHT_AIRBORNE) return "airborne";
  if (droneFlightPhase == DRONE_FLIGHT_LANDING) return "landing";
  return "grounded";
}

// Every input source (keyboard, joystick, Leap, Web and AiBot) passes through
// this gate, so no producer can bypass the physical flight prerequisites.
void setDroneFlightNormalized(float throttle, float yaw, float pitch, float roll, float strafe, float forward) {
  float boundedThrottle = constrain(throttle, -1.0f, 1.0f);
  boolean flightReady = dronePlanarMotionReady();

  // Automatic takeoff/landing owns the vertical axis until its physical
  // completion condition is reached. Horizontal commands can arrive meanwhile
  // but cannot accidentally cancel the vertical sequence on real hardware.
  if (droneAutoTakeoffActive) {
    boundedThrottle = DRONE_AUTO_TAKEOFF_THROTTLE;
  } else if (droneAutoLandingActive) {
    boundedThrottle = DRONE_AUTO_LANDING_THROTTLE;
  } else if (!droneIsAirborne() && boundedThrottle < 0.0f) {
    boundedThrottle = 0.0f;
  }

  droneThrottleCmd = boundedThrottle;
  droneYawCmd = flightReady ? constrain(yaw, -1.0f, 1.0f) : 0.0f;
  dronePitchCmd = flightReady ? constrain(pitch, -1.0f, 1.0f) : 0.0f;
  droneRollCmd = flightReady ? constrain(roll, -1.0f, 1.0f) : 0.0f;
  droneStrafeCmd = flightReady ? constrain(strafe, -1.0f, 1.0f) : 0.0f;
  droneForwardCmd = flightReady ? constrain(forward, -1.0f, 1.0f) : 0.0f;
}

void cancelDroneAutomaticVerticalMotion() {
  droneAutoTakeoffActive = false;
  droneAutoLandingActive = false;
  droneFlightPhase = droneIsAirborne() ? DRONE_FLIGHT_AIRBORNE : DRONE_FLIGHT_GROUNDED;
}

// Stops drone motion and cancels automatic vertical sequencing.
void stopDroneMotion(boolean forceSend) {
  cancelDroneAutomaticVerticalMotion();
  resetDroneInputState();
  sendDroneRuntimeCommand(forceSend);
}

// Returns the throttle command sent to hardware and physics.
// The commercial profile boosts both climb and descent while keeping the
// command bounded to the firmware's normalized [-1, 1] contract.
float effectiveDroneThrottleCommand() {
  float cmd = constrain(droneThrottleCmd, -1.0f, 1.0f);
  if (cmd > 0.001f) {
    cmd = max(DRONE_ASCENT_MIN_EFFECTIVE_CMD, cmd * DRONE_ASCENT_COMMAND_GAIN);
  } else if (cmd < -0.001f) {
    cmd = min(-DRONE_DESCENT_MIN_EFFECTIVE_CMD, cmd * DRONE_DESCENT_COMMAND_GAIN);
  }
  return constrain(cmd, -1.0f, 1.0f);
}

// Returns the commercial forward command used by transport and local physics.
float effectiveDroneForwardCommand() {
  return constrain(droneForwardCmd * DRONE_FORWARD_COMMAND_GAIN, -1.0f, 1.0f);
}

// Returns the commercial lateral command used by transport and local physics.
float effectiveDroneStrafeCommand() {
  return constrain(droneStrafeCmd * DRONE_STRAFE_COMMAND_GAIN, -1.0f, 1.0f);
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
  boolean flightReady;
  String phase;
}

// Checks the commanded drone movement against the shared environment guard.
boolean droneRuntimeMotionBlockedByCollision() {
  if (!environmentCollisionActiveForRobotMode(ROBOT_MODE_DRONE)) return false;
  if (!droneIsAirborne()) return false;

  float yawStepNav = droneYawCmd * radians(2.05f);
  float motionYaw = droneNavYaw - yawStepNav * 0.5f;
  float semanticForward = -effectiveDroneForwardCommand();
  float forwardSpeed = -semanticForward * droneForwardSceneStep60Hz();
  float strafeSpeed = effectiveDroneStrafeCommand() * droneStrafeSceneStep60Hz();
  float planarTraction = 1.0f - 0.18f * constrain(abs(droneYawCmd), 0.0f, 1.0f);
  forwardSpeed *= planarTraction;
  strafeSpeed *= planarTraction;

  if (abs(forwardSpeed) < 0.0001f && abs(strafeSpeed) < 0.0001f) return false;

  float proposedX = droneNavX + sin(motionYaw) * forwardSpeed + sin(motionYaw + HALF_PI) * strafeSpeed;
  float proposedZ = droneNavZ + cos(motionYaw) * forwardSpeed + cos(motionYaw + HALF_PI) * strafeSpeed;
  return !droneEnvironmentMoveAllowed(
    droneNavX, droneNavY, droneNavZ,
    proposedX, droneNavY, proposedZ
  );
}

// Emits a bounded notice when the drone command is held by the map guard.
void notifyDroneRuntimeCollisionBlock() {
  long now = millis();
  if ((now - droneLastCollisionBlockNoticeMs) < 700) return;
  droneLastCollisionBlockNoticeMs = now;
  updateMessage(tr("Drone motion blocked by the collision guard."));
}

// Returns the canonical drone runtime state shared by animation, ASCII and HEX commands.
DroneRuntimeCommandState buildDroneRuntimeCommandState() {
  return buildDroneRuntimeCommandState(true);
}

// Returns the drone runtime state, optionally applying the Processing guard before transport.
DroneRuntimeCommandState buildDroneRuntimeCommandState(boolean applyCollisionFilter) {
  DroneRuntimeCommandState state = new DroneRuntimeCommandState();
  state.airborne = droneIsAirborne();
  state.flightReady = dronePlanarMotionReady();
  state.phase = droneFlightPhaseName();

  float throttle = effectiveDroneThrottleCommand();
  if (!state.airborne && throttle < 0.0f) throttle = 0.0f;

  boolean blockedByCollision = applyCollisionFilter && droneRuntimeMotionBlockedByCollision();
  droneRuntimeCollisionFiltered = blockedByCollision;

  state.throttlePct = constrain(round(throttle * 100.0f), -100, 100);
  state.yawPct = constrain(round((state.flightReady ? droneYawCmd : 0.0f) * 100.0f), -100, 100);
  state.pitchPct = constrain(round((state.flightReady && !blockedByCollision ? dronePitchCmd : 0.0f) * 100.0f), -100, 100);
  state.rollPct = constrain(round((state.flightReady && !blockedByCollision ? droneRollCmd : 0.0f) * 100.0f), -100, 100);
  state.strafePct = constrain(round((state.flightReady && !blockedByCollision ? effectiveDroneStrafeCommand() : 0.0f) * 100.0f), -100, 100);
  state.forwardPct = constrain(round((state.flightReady && !blockedByCollision ? -effectiveDroneForwardCommand() : 0.0f) * 100.0f), -100, 100);
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
  if (dronePlanarMotionReady()) {
    droneAutoTakeoffActive = false;
    droneAutoLandingActive = false;
    droneFlightPhase = DRONE_FLIGHT_AIRBORNE;
    resetDroneInputState();
    sendDroneRuntimeCommand(true);
    updateMessage(tr("Drone is already airborne."));
    return;
  }

  resetDroneInputState();
  droneAutoLandingActive = false;
  droneAutoTakeoffActive = true;
  droneFlightPhase = DRONE_FLIGHT_TAKING_OFF;
  droneAutoTakeoffTargetLift = constrain(max(DRONE_AUTO_TAKEOFF_LIFT, droneCurrentPhysicalLift() + 8.0f), 0.0f, getDroneLocalMaxLift());
  if (!droneUsesConnectedAltitudeTelemetry()) droneTargetLift = droneAutoTakeoffTargetLift;
  droneThrottleCmd = DRONE_AUTO_TAKEOFF_THROTTLE;
  sendDroneRuntimeCommand(true);
  updateMessage(tr("Drone takeoff requested."));
}


// Utility: trigger drone land.
void triggerDroneLand() {
  resetDroneInputState();
  droneAutoTakeoffActive = false;
  if (!droneIsAirborne()) {
    droneAutoLandingActive = false;
    droneFlightPhase = DRONE_FLIGHT_GROUNDED;
    droneTargetLift = 0;
    droneThrottleCmd = 0;
    sendDroneRuntimeCommand(true);
    updateMessage(tr("Drone is already on the ground."));
    return;
  }
  droneAutoLandingActive = true;
  droneFlightPhase = DRONE_FLIGHT_LANDING;
  if (!droneUsesConnectedAltitudeTelemetry()) droneTargetLift = 0;
  droneThrottleCmd = DRONE_AUTO_LANDING_THROTTLE;
  sendDroneRuntimeCommand(true);
  updateMessage(tr("Drone auto landing requested."));
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
  if (runtimeState.flightReady) {
    yawStepNav = droneYawCmdFiltered * radians(2.05f) * dt * 60.0f;
    droneNavYaw += yawStepNav;
  }
  float targetVisualYaw = runtimeState.flightReady ? droneYawCmdFiltered * radians(8.0f) : 0.0f;
  droneYaw = lerp(droneYaw, targetVisualYaw, airborne ? 0.10f : 0.08f);

  float targetPitchDeg = 0;
  float targetRollDeg = 0;
  if (runtimeState.flightReady) {
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
  float ascentLiftRate = droneAscentLiftStep60Hz() * dt * 60.0f;
  float descentLiftRate = droneDescentLiftStep60Hz() * dt * 60.0f;
  if (!useConnectedAltitudeTelemetry) {
    if (droneAutoTakeoffActive) {
      droneTargetLift = constrain(droneAutoTakeoffTargetLift, 0.0f, getDroneLocalMaxLift());
    } else if (droneAutoLandingActive) {
      droneTargetLift = 0.0f;
    } else if (droneThrottleCmdFiltered > 0.02f) {
      droneTargetLift = min(getDroneLocalMaxLift(), droneTargetLift + droneThrottleCmdFiltered * ascentLiftRate);
    } else if (droneThrottleCmdFiltered < -0.02f) {
      droneTargetLift = max(0, droneTargetLift + droneThrottleCmdFiltered * descentLiftRate);
    }

    droneFlightLift = lerp(droneFlightLift, droneTargetLift, droneTargetLift >= droneFlightLift ? 0.12f : 0.10f);
    if (droneFlightLift < 0.05f && droneTargetLift < 0.05f) {
      droneFlightLift = 0;
      droneTargetLift = 0;
    }
  } else {
    float connectedLift = constrain(droneAltitudeCm * 0.5f, 0.0f, getDroneLocalMaxLift());
    droneTargetLift = connectedLift;
    droneFlightLift = connectedLift;
  }

  float physicalLiftNow = droneCurrentPhysicalLift();
  if (droneAutoTakeoffActive && physicalLiftNow >= max(DRONE_PLANAR_UNLOCK_LIFT, droneAutoTakeoffTargetLift - DRONE_AUTO_TAKEOFF_COMPLETE_TOLERANCE)) {
    droneAutoTakeoffActive = false;
    droneFlightPhase = DRONE_FLIGHT_AIRBORNE;
    droneThrottleCmd = 0.0f;
    sendDroneRuntimeCommand(true);
    updateMessage(tr("Drone takeoff complete."));
  } else if (droneAutoLandingActive && physicalLiftNow <= DRONE_LANDED_LIFT_EPS) {
    droneAutoLandingActive = false;
    droneFlightPhase = DRONE_FLIGHT_GROUNDED;
    droneThrottleCmd = 0.0f;
    droneTargetLift = 0.0f;
    droneFlightLift = 0.0f;
    sendDroneRuntimeCommand(true);
    updateMessage(tr("Drone landing complete."));
  } else if (!droneAutoTakeoffActive && !droneAutoLandingActive) {
    if (dronePlanarMotionReady()) {
      droneFlightPhase = DRONE_FLIGHT_AIRBORNE;
    } else if (physicalLiftNow > DRONE_PHYSICAL_AIRBORNE_LIFT) {
      droneFlightPhase = droneThrottleCmd < -0.02f ? DRONE_FLIGHT_LANDING : DRONE_FLIGHT_TAKING_OFF;
    } else if (droneThrottleCmd > 0.02f) {
      droneFlightPhase = DRONE_FLIGHT_TAKING_OFF;
    } else {
      droneFlightPhase = DRONE_FLIGHT_GROUNDED;
    }
  }

  droneY = DRONE_REST_Y - droneFlightLift;

  // Rebuild after altitude transitions so planar motion unlocks only from the
  // physical state reached in this frame, never from a requested target.
  runtimeState = buildDroneRuntimeCommandState();
  boolean flightReady = runtimeState.flightReady;

  float navStepX = 0;
  float navStepZ = 0;
  float navStepY = 0;
  if (flightReady) {
    float motionYaw = droneNavYaw - yawStepNav * 0.5f;
    float forwardSpeed = -semanticForward * droneForwardSceneStep60Hz() * dt * 60.0f;
    float strafeSpeed = droneStrafeCmdFiltered * droneStrafeSceneStep60Hz() * dt * 60.0f;
    float planarTraction = 1.0f - 0.18f * constrain(abs(droneYawCmdFiltered), 0.0f, 1.0f);
    forwardSpeed *= planarTraction;
    strafeSpeed *= planarTraction;

    navStepX = sin(motionYaw) * forwardSpeed + sin(motionYaw + HALF_PI) * strafeSpeed;
    navStepZ = cos(motionYaw) * forwardSpeed + cos(motionYaw + HALF_PI) * strafeSpeed;
  }

  float proposedX = droneNavX + navStepX;
  float proposedY = droneNavY + navStepY;
  float proposedZ = droneNavZ + navStepZ;
  if (droneEnvironmentMoveAllowed(
    droneNavX, droneNavY, droneNavZ,
    proposedX, proposedY, proposedZ
  )) {
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
    droneAltitudeCm = constrain(max(0, droneFlightLift * 2.0f), 0, maxAltitudeCmForRobotMode(ROBOT_MODE_DRONE));
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
  float headingDeg = (isFirmwareTelemetrySelected() && systemReady)
    ? droneHeadingTelemetryDeg
    : degrees(droneNavYaw);
  drawRobotOperatingEnvironment(
    ROBOT_MODE_DRONE,
    droneNavX, droneNavZ, droneNavYaw,
    headingDeg,
    true
  );
}

// Utility: draw drone 3 d.
void drawDrone3D(boolean advanceCamera) {
  beginRobotScene3D(ROBOT_MODE_DRONE, advanceCamera);

  drawDroneGround();
  applySceneLighting();
  drawCurrentEnvironmentMap();
  drawDroneDownwardSonarProbe();
  drawRobotFrontCameraPanel(ROBOT_MODE_DRONE);
  noStroke();

  pushMatrix();
  translate(getDroneSceneX(), getDroneSceneY() + droneY, getDroneSceneZ());
  scale(DRONE_VISUAL_RENDER_SCALE * largeWorldRobotPresentationScale(ROBOT_MODE_DRONE));
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
  rotateX(radians(droneCameraTiltDeg));
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
  noLights();
  endRobotScene3D("Drone3D.pde:scene");
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
  setDroneFlightNormalized(
    constrain(frame[3] / 100.0f, -1, 1),
    constrain(frame[0] / 100.0f, -1, 1),
    0,
    0,
    constrain(frame[4] / 100.0f, -1, 1),
    constrain(frame[5] / 100.0f, -1, 1)
  );
  sendDroneRuntimeCommand(true);
}

// Utility: move drone safely toward frame.
boolean moveDroneSafelyTowardFrame(int[] targetFrame, int maxStep, int tolerance) {
  if (targetFrame == null || targetFrame.length < 6) return true;

  float targetYaw = approachFloat(droneYawCmd * 100.0f, targetFrame[0], maxStep) / 100.0f;
  float targetThrottle = approachFloat(droneThrottleCmd * 100.0f, targetFrame[3], maxStep) / 100.0f;
  float targetStrafe = approachFloat(droneStrafeCmd * 100.0f, targetFrame[4], maxStep) / 100.0f;
  float targetForward = approachFloat(droneForwardCmd * 100.0f, targetFrame[5], maxStep) / 100.0f;
  setDroneFlightNormalized(targetThrottle, targetYaw, 0, 0, targetStrafe, targetForward);
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
  if (pointInRect(mx, my, toolsInnerX, diagnosticsMapButtonRowY(panelY, 2), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { adjustCurrentImportedWorldTransform(); return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsMapButtonRowY(panelY, 2), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { saveCollisionWorldFromWindowsDialog(); return true; }

  if (pointInRect(mx, my, toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { triggerDroneTakeOff(); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { triggerDroneLand(); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { toggleSelectedDroneCameraView(); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { diagnosticsTakeLocalControl(); try { toggleDroneDownwardSonar(); } finally { diagnosticsEndLocalControl(); } return true; }
  if (pointInRect(mx, my, toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 2), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { triggerCompassCalibration(); return true; }
  return true;
}

void drawDroneDiagnosticsPanelSections(int panelY, int contentX, int contentW, int gap, int toolsInnerX, int toolsCol2W, int sensorsContentX, int sensorsContentW, int droneRuntimeCardY, int droneSensorsCardY) {
  drawSectionCard2D(contentX, droneRuntimeCardY, contentW, DIAGNOSTICS_DRONE_RUNTIME_CARD_H, tr("Drone runtime"));
  drawMiniButton2D(toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Take off"), false);
  drawMiniButton2D(toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Land"), false);
  drawMiniButton2D(toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Camera"), isRobotCameraViewEnabled(ROBOT_MODE_DRONE));
  drawMiniButton2D(toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Downward sonar"), currentEnvironmentAutoScanEnabled());
  drawMiniButton2D(toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 2), toolsCol2W, DIAGNOSTICS_BUTTON_H, compassCalibrationActive() ? tr("Compass cal *") : tr("Compass cal"), compassCalibrationActive());

  drawSectionCard2D(contentX, droneSensorsCardY, contentW, DIAGNOSTICS_DRONE_SENSORS_CARD_H, tr("Drone stability sensors"));
  int droneStabilityWidgetH = DIAGNOSTICS_DRONE_SENSORS_CARD_H - 144;
  drawDroneStabilityTargets2D(sensorsContentX, droneSensorsCardY + 32, sensorsContentW, droneStabilityWidgetH);
  fill(uiPrimaryTextColor());
  int droneInfoY = droneSensorsCardY + 32 + droneStabilityWidgetH + 12;
  drawDiagnosticsFittedTextLeft(diagnosticsCompassSummaryText(), sensorsContentX, droneInfoY, sensorsContentW);
  drawDiagnosticsFittedTextLeft(tr("Position: firmware/internal odometry by compass"), sensorsContentX, droneInfoY + 18, sensorsContentW);
  drawPressureBar2D(sensorsContentX, droneInfoY + 42, sensorsContentW, 18, droneBatteryPct, 100, tr("Battery"));
  drawDiagnosticsFittedTextLeft(tr("Battery raw / World: ") + nf(getSensorFloat("battery_raw", getSensorFloat("battery", 0)), 1, 0) + " / " + currentEnvironmentWorldSummary(), sensorsContentX, droneInfoY + 70, sensorsContentW);
}

// =====================================================================
// SynRovModule adapter: Drone3D
// =====================================================================
