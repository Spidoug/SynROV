// =====================================================================
// SynROV Processing - Robot intelligence and sensor orchestration
// ---------------------------------------------------------------------
// Purpose:
//   Centralizes robot-specific AI sensor roles, human-input ownership and
//   attitude visualization. The global world grid remains level; Manipulator
//   and Vehicle may tilt together with a small local support surface according
//   to their base/chassis IMU, while Drone tilts only its airborne body.
// =====================================================================

String lastLocalInputSource = "none";
long lastKeyboardInputMillis = 0;
long lastJoystickInputMillis = 0;
long lastLeapInputMillis = 0;
String lastRemoteControlOrigin = "web";


// Returns the first telemetry key present in the selected sensor object.
String aiFirstSensorKey(JSONObject sensors, String[] keys) {
  if (sensors == null || keys == null) return "";
  for (int i = 0; i < keys.length; i++) {
    String key = keys[i];
    if (key != null && sensors.hasKey(key)) return key;
  }
  return "";
}

float aiSensorFloat(JSONObject sensors, String[] keys, float fallback) {
  String key = aiFirstSensorKey(sensors, keys);
  return key.length() > 0 ? getJsonFloat(sensors, key, fallback) : fallback;
}

boolean aiHasSensor(JSONObject sensors, String[] keys) {
  return aiFirstSensorKey(sensors, keys).length() > 0;
}

JSONObject aiImuSnapshot(int index, String role) {
  JSONObject imu = new JSONObject();
  String prefix = index == 0 ? "mpu1_" : "mpu2_";
  imu.setString("role", role);
  imu.setBoolean("available", latestSensors != null && latestSensors.hasKey(prefix + "ax"));
  imu.setFloat("ax_raw", getSensorFloat(prefix + "ax", 0));
  imu.setFloat("ay_raw", getSensorFloat(prefix + "ay", 0));
  imu.setFloat("az_raw", getSensorFloat(prefix + "az", 0));
  imu.setFloat("gx_raw", getSensorFloat(prefix + "gx", 0));
  imu.setFloat("gy_raw", getSensorFloat(prefix + "gy", 0));
  imu.setFloat("gz_raw", getSensorFloat(prefix + "gz", 0));
  float roll = index == 0 ? mpu1RollDeg : mpu2RollDeg;
  float pitch = index == 0 ? mpu1PitchDeg : mpu2PitchDeg;
  float yaw = index == 0 ? mpu1YawDeg : mpu2YawDeg;
  imu.setFloat("roll_deg", roll);
  imu.setFloat("pitch_deg", pitch);
  imu.setFloat("yaw_deg", yaw);
  return imu;
}

String[] manipulatorJointTorqueKeys(String joint) {
  if (joint.equals("base")) return new String[] {"base_torque", "base_torque_raw", "base_current_ma", "an2", "an_2"};
  if (joint.equals("upper")) return new String[] {"upper_torque", "upper_arm_torque_raw", "upper_current_ma", "an3", "an_3"};
  if (joint.equals("fore")) return new String[] {"forearm_torque", "forearm_torque_raw", "fore_current_ma", "an4", "an_4"};
  if (joint.equals("forearm_roll")) return new String[] {"forearm_roll_torque", "forearm_roll_torque_raw", "forearm_roll_current_ma", "an5", "an_5"};
  if (joint.equals("wrist_pitch")) return new String[] {"wrist_pitch_current_ma", "wrist_pitch_torque", "wrist_pitch_torque_raw", "ina1_ma"};
  if (joint.equals("wrist_rot")) return new String[] {"wrist_rot_current_ma", "wrist_rotation_current_ma", "wrist_rot_torque", "wrist_rot_torque_raw", "ina2_ma"};
  return new String[] {"grip_current_ma", "gripper_current_ma", "grip_torque", "grip_torque_raw", "grip_pressure_ma", "ina1_ma"};
}

String aiManipulatorTorqueDisplayName(String source, String joint) {
  if (source.equals("an2") || source.equals("an_2")) return "Base torque sensor";
  if (source.equals("an3") || source.equals("an_3")) return "Upper arm torque sensor";
  if (source.equals("an4") || source.equals("an_4")) return "Forearm vertical torque sensor";
  if (source.equals("an5") || source.equals("an_5")) return "Forearm rotational torque sensor";
  if (source.equals("ina1_ma")) return "Wrist group current 1";
  if (source.equals("ina2_ma")) return "Wrist group current 2";
  if (source.equals("grip_pressure_ma")) return "Gripper pressure/current reference";
  return joint + " torque reference";
}

JSONObject buildManipulatorJointTorqueReferences(JSONObject sensors) {
  JSONObject refs = new JSONObject();
  String[] joints = {"base", "upper", "fore", "forearm_roll", "wrist_pitch", "wrist_rot", "grip"};
  for (int i = 0; i < joints.length; i++) {
    String[] keys = manipulatorJointTorqueKeys(joints[i]);
    String source = aiFirstSensorKey(sensors, keys);
    JSONObject item = new JSONObject();
    item.setBoolean("available", source.length() > 0);
    item.setString("source_key", source);
    item.setString("sensor_name", aiManipulatorTorqueDisplayName(source, joints[i]));
    item.setString("joint", joints[i]);
    item.setFloat("reference", source.length() > 0 ? getJsonFloat(sensors, source, 0.0f) : 0.0f);
    item.setString("meaning", "proportional_joint_torque_reference");
    item.setString("signal_type", source.indexOf("ma") >= 0 ? "current_ma" : "raw_torque_sensor");
    refs.setJSONObject(joints[i], item);
  }
  return refs;
}

float aiBatteryPercent(JSONObject sensors) {
  if (simulationMode && !aiHasSensor(sensors, new String[] {"battery_pct", "bat_pct"})) return 100.0f;
  return constrain(aiSensorFloat(sensors, new String[] {"battery_pct", "bat_pct"}, 0.0f), 0.0f, 100.0f);
}

float aiCommunicationQualityPct(JSONObject sensors) {
  if (simulationMode) return 100.0f;
  float direct = aiSensorFloat(sensors, new String[] {"communication_quality_pct", "link_quality_pct", "signal_pct", "rssi_pct"}, -1.0f);
  if (direct >= 0.0f) return constrain(direct, 0.0f, 100.0f);
  float rssi = aiSensorFloat(sensors, new String[] {"rssi_dbm", "signal_dbm"}, 999.0f);
  if (rssi < 500.0f) return constrain(map(rssi, -100.0f, -50.0f, 0.0f, 100.0f), 0.0f, 100.0f);
  float linkMs = aiSensorFloat(sensors, new String[] {"link_ms"}, -1.0f);
  if (linkMs >= 0.0f) return constrain(map(linkMs, 40.0f, 800.0f, 100.0f, 0.0f), 0.0f, 100.0f);
  return 0.0f;
}

String aiCommunicationQualitySource(JSONObject sensors) {
  if (simulationMode) return "simulation";
  String key = aiFirstSensorKey(sensors, new String[] {"communication_quality_pct", "link_quality_pct", "signal_pct", "rssi_pct"});
  if (key.length() > 0) return key;
  key = aiFirstSensorKey(sensors, new String[] {"rssi_dbm", "signal_dbm"});
  if (key.length() > 0) return key;
  return aiHasSensor(sensors, new String[] {"link_ms"}) ? "link_ms_derived" : "unavailable";
}

JSONObject buildVehicleDriveTorqueReferences(JSONObject sensors) {
  JSONObject refs = new JSONObject();
  String[] leftKeys = {"left_track_torque", "left_torque", "left_motor_torque", "an2", "an_2"};
  String[] rightKeys = {"right_track_torque", "right_torque", "right_motor_torque", "an3", "an_3"};
  String leftSource = aiFirstSensorKey(sensors, leftKeys);
  String rightSource = aiFirstSensorKey(sensors, rightKeys);
  refs.setFloat("left_reference", leftSource.length() > 0 ? getJsonFloat(sensors, leftSource, 0.0f) : 0.0f);
  refs.setFloat("right_reference", rightSource.length() > 0 ? getJsonFloat(sensors, rightSource, 0.0f) : 0.0f);
  refs.setString("left_source_key", leftSource);
  refs.setString("right_source_key", rightSource);
  refs.setBoolean("available", leftSource.length() > 0 || rightSource.length() > 0);
  refs.setString("meaning", "proportional_drive_torque_reference");
  refs.setString("unit", "raw_reference");
  return refs;
}

float manipulatorBaseAttitudePitchDeg = 0.0f;
float manipulatorBaseAttitudeRollDeg = 0.0f;
float vehicleBaseAttitudePitchDeg = 0.0f;
float vehicleBaseAttitudeRollDeg = 0.0f;

final int AI_INPUT_ACTIVE_WINDOW_MS = 360;
final float ROBOT_ATTITUDE_LERP = 0.16f;

void markRobotInputActivity(String source) {
  String normalized = source == null ? "none" : trim(source).toLowerCase();
  long now = millis();
  if (normalized.equals("keyboard")) lastKeyboardInputMillis = now;
  else if (normalized.equals("joystick")) lastJoystickInputMillis = now;
  else if (normalized.equals("leap")) lastLeapInputMillis = now;
  lastLocalInputSource = normalized;
}

void markRemoteControlOrigin(JSONObject command) {
  if (command == null) return;
  String origin = getJsonString(command, "origin", "");
  if (origin.length() == 0) origin = getJsonString(command, "source", "");
  if (origin.length() == 0) origin = getJsonString(command, "controlSource", "web");
  if (origin.length() == 0) origin = "web";
  lastRemoteControlOrigin = origin.toLowerCase();
}


boolean joystickMeaningfulInputForCurrentRobot() {
  if (!enableJoystick || !isJoystickConnected()) return false;
  String[] axes = joystickAxisLogicalNamesForRobot(currentModeName());
  if (axes != null) {
    for (int i = 0; i < axes.length; i++) {
      if (abs(joystickRuntimeState().getAxisValue(axes[i])) > max(0.04f, joystickDeadband)) return true;
    }
  }
  String[] buttons = joystickButtonLogicalNamesForRobot(currentModeName());
  if (buttons != null) {
    for (int i = 0; i < buttons.length; i++) {
      if (joystickRuntimeState().getButtonValue(buttons[i])) return true;
    }
  }
  return false;
}

String activeRobotInputSource() {
  long now = millis();
  if (isRemoteControlActiveForCurrentRobot()) {
    if (currentControlOwner == CONTROL_SOURCE_AIBOT) return "aibot";
    if (currentControlOwner == CONTROL_SOURCE_ROS) return "ros";
    return "web";
  }
  if (keyboardInputActiveForCurrentRobot() || (now - lastKeyboardInputMillis) <= AI_INPUT_ACTIVE_WINDOW_MS) return "keyboard";
  if (enableLeap && leapAvailable && (now - lastLeapInputMillis) <= AI_INPUT_ACTIVE_WINDOW_MS) return "leap";
  if (joystickMeaningfulInputForCurrentRobot() || (now - lastJoystickInputMillis) <= AI_INPUT_ACTIVE_WINDOW_MS) return "joystick";
  return "none";
}

boolean robotUsesFirmwareAttitude(int robotMode) {
  if (!isFirmwareTelemetrySelected() || !systemReady || simulationMode || myPort == null) return false;
  if (latestSensors == null || latestSensors.size() == 0) return false;
  if (robotMode == ROBOT_MODE_MANIPULATOR) {
    return hasAnySensorKey(new String[] {"base_pitch_deg", "base_roll_deg", "mpu2_ax"});
  }
  if (robotMode == ROBOT_MODE_VEHICLE) {
    return hasAnySensorKey(new String[] {"vehicle_pitch_deg", "vehicle_roll_deg", "mpu1_ax"});
  }
  return hasAnySensorKey(new String[] {"drone_pitch_deg", "drone_roll_deg", "mpu1_ax"});
}

float robotTelemetryPitchDeg(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) {
    if (latestSensors != null && latestSensors.hasKey("base_pitch_deg")) return getSensorFloat("base_pitch_deg", mpu2PitchDeg);
    if (latestSensors != null && latestSensors.hasKey("manipulator_pitch_deg")) return getSensorFloat("manipulator_pitch_deg", mpu2PitchDeg);
    if (latestSensors != null && latestSensors.hasKey("mpu2_ax")) return mpu2PitchDeg;
    return 0.0f;
  }
  if (robotMode == ROBOT_MODE_VEHICLE) {
    return getSensorFloat("vehicle_pitch_deg", mpu1PitchDeg);
  }
  return getSensorFloat("drone_pitch_deg", mpu1PitchDeg);
}

float robotTelemetryRollDeg(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) {
    if (latestSensors != null && latestSensors.hasKey("base_roll_deg")) return getSensorFloat("base_roll_deg", mpu2RollDeg);
    if (latestSensors != null && latestSensors.hasKey("manipulator_roll_deg")) return getSensorFloat("manipulator_roll_deg", mpu2RollDeg);
    if (latestSensors != null && latestSensors.hasKey("mpu2_ax")) return mpu2RollDeg;
    return 0.0f;
  }
  if (robotMode == ROBOT_MODE_VEHICLE) {
    return getSensorFloat("vehicle_roll_deg", mpu1RollDeg);
  }
  return getSensorFloat("drone_roll_deg", mpu1RollDeg);
}

float robotTelemetryYawDeg(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) return getSensorFloat("heading_deg", mpu2YawDeg);
  if (robotMode == ROBOT_MODE_VEHICLE) return getSensorFloatAny(new String[] {"vehicle_yaw_deg", "heading_deg"}, degrees(vehicleNavYaw));
  return getSensorFloatAny(new String[] {"drone_yaw_deg", "att_yaw_deg", "heading_deg"}, degrees(droneNavYaw));
}

void updateRobotAttitude3DState() {
  float manipPitchTarget = robotUsesFirmwareAttitude(ROBOT_MODE_MANIPULATOR)
    ? constrain(robotTelemetryPitchDeg(ROBOT_MODE_MANIPULATOR), -45.0f, 45.0f)
    : 0.0f;
  float manipRollTarget = robotUsesFirmwareAttitude(ROBOT_MODE_MANIPULATOR)
    ? constrain(robotTelemetryRollDeg(ROBOT_MODE_MANIPULATOR), -45.0f, 45.0f)
    : 0.0f;
  manipulatorBaseAttitudePitchDeg = lerp(manipulatorBaseAttitudePitchDeg, manipPitchTarget, ROBOT_ATTITUDE_LERP);
  manipulatorBaseAttitudeRollDeg = lerp(manipulatorBaseAttitudeRollDeg, manipRollTarget, ROBOT_ATTITUDE_LERP);

  float vehiclePitchTarget = robotUsesFirmwareAttitude(ROBOT_MODE_VEHICLE)
    ? constrain(robotTelemetryPitchDeg(ROBOT_MODE_VEHICLE), -45.0f, 45.0f)
    : 0.0f;
  float vehicleRollTarget = robotUsesFirmwareAttitude(ROBOT_MODE_VEHICLE)
    ? constrain(robotTelemetryRollDeg(ROBOT_MODE_VEHICLE), -45.0f, 45.0f)
    : 0.0f;
  vehicleBaseAttitudePitchDeg = lerp(vehicleBaseAttitudePitchDeg, vehiclePitchTarget, ROBOT_ATTITUDE_LERP);
  vehicleBaseAttitudeRollDeg = lerp(vehicleBaseAttitudeRollDeg, vehicleRollTarget, ROBOT_ATTITUDE_LERP);

  // Firmware IMU is authoritative for the Drone body attitude. In local
  // telemetry the flight model continues to own dronePitch/droneRoll.
  if (robotUsesFirmwareAttitude(ROBOT_MODE_DRONE)) {
    float pitchTarget = radians(constrain(robotTelemetryPitchDeg(ROBOT_MODE_DRONE), -70.0f, 70.0f));
    float rollTarget = radians(constrain(robotTelemetryRollDeg(ROBOT_MODE_DRONE), -70.0f, 70.0f));
    dronePitch = lerp(dronePitch, pitchTarget, 0.22f);
    droneRoll = lerp(droneRoll, rollTarget, 0.22f);
  }
}

float robotSupportPitchDeg(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) return manipulatorBaseAttitudePitchDeg;
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleBaseAttitudePitchDeg;
  return 0.0f;
}

float robotSupportRollDeg(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) return manipulatorBaseAttitudeRollDeg;
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleBaseAttitudeRollDeg;
  return 0.0f;
}

void applyRobotSupportAttitudeRotation(int robotMode) {
  rotateZ(radians(robotSupportRollDeg(robotMode)));
  rotateX(radians(robotSupportPitchDeg(robotMode)));
}

void beginManipulatorBaseAttitudeTransform() {
  pushMatrix();
  translate(0, GROUND_Y, 0);
  applyRobotSupportAttitudeRotation(ROBOT_MODE_MANIPULATOR);
  translate(0, -GROUND_Y, 0);
}

void endManipulatorBaseAttitudeTransform() {
  safePopMatrix("RobotIntelligenceOrchestration.pde:manipulatorBaseAttitude");
}

void drawRobotAttitudeSupportSurface(int robotMode) {
  if (robotMode == ROBOT_MODE_DRONE) return;
  float pitch = robotSupportPitchDeg(robotMode);
  float roll = robotSupportRollDeg(robotMode);
  if (abs(pitch) < 0.05f && abs(roll) < 0.05f && !robotUsesFirmwareAttitude(robotMode)) return;

  float widthScene = robotMode == ROBOT_MODE_MANIPULATOR ? 260.0f : max(220.0f, VEH_BODY_WIDTH * 2.3f);
  float lengthScene = robotMode == ROBOT_MODE_MANIPULATOR ? 260.0f : max(300.0f, VEH_BODY_LENGTH * 1.9f);
  float x = robotMode == ROBOT_MODE_VEHICLE ? getVehicleSceneX() : 0.0f;
  float z = robotMode == ROBOT_MODE_VEHICLE ? getVehicleSceneZ() : 0.0f;

  pushStyle();
  pushMatrix();
  translate(x, GROUND_Y - 0.6f, z);
  applyRobotSupportAttitudeRotation(robotMode);
  noStroke();
  fill(72, 78, 86, isDarkThemeEnabled() ? 92 : 52);
  box(widthScene, 1.8f, lengthScene);

  // A small local cross makes the IMU tilt readable against the fixed global
  // world grid without reintroducing the dense local grids removed earlier.
  stroke(230, 230, 230, isDarkThemeEnabled() ? 105 : 82);
  strokeWeight(1.3f);
  float halfW = widthScene * 0.42f;
  float halfL = lengthScene * 0.42f;
  line(-halfW, -1.0f, 0, halfW, -1.0f, 0);
  line(0, -1.0f, -halfL, 0, -1.0f, halfL);
  safePopMatrix("RobotIntelligenceOrchestration.pde:supportSurface");
  popStyle();
}

JSONObject buildAiInputActivitySnapshot() {
  JSONObject inputs = new JSONObject();
  String active = activeRobotInputSource();
  inputs.setString("active_source", active);
  inputs.setBoolean("keyboard", keyboardInputActiveForCurrentRobot() || (millis() - lastKeyboardInputMillis) <= AI_INPUT_ACTIVE_WINDOW_MS);
  inputs.setBoolean("joystick", joystickMeaningfulInputForCurrentRobot() || (millis() - lastJoystickInputMillis) <= AI_INPUT_ACTIVE_WINDOW_MS);
  inputs.setBoolean("leap_motion", enableLeap && leapAvailable && (millis() - lastLeapInputMillis) <= AI_INPUT_ACTIVE_WINDOW_MS);
  inputs.setBoolean("web", currentControlOwner == CONTROL_SOURCE_WEB);
  inputs.setBoolean("aibot", currentControlOwner == CONTROL_SOURCE_AIBOT);
  inputs.setBoolean("ros", currentControlOwner == CONTROL_SOURCE_ROS);
  inputs.setBoolean("human_override", active.equals("keyboard") || active.equals("joystick") || active.equals("leap") || active.equals("web"));
  inputs.setString("telemetry_authority", isLocalTelemetrySelected() ? "local_commanded" : "firmware_measured");
  inputs.setString("last_remote_origin", lastRemoteControlOrigin == null ? "" : lastRemoteControlOrigin);
  return inputs;
}

JSONObject buildAiAttitudeSnapshot(int robotMode) {
  JSONObject attitude = new JSONObject();
  boolean firmware = robotUsesFirmwareAttitude(robotMode);
  float pitch;
  float roll;
  float yaw;
  if (robotMode == ROBOT_MODE_MANIPULATOR) {
    pitch = manipulatorBaseAttitudePitchDeg;
    roll = manipulatorBaseAttitudeRollDeg;
    yaw = getBasePoseYawDeg();
  } else if (robotMode == ROBOT_MODE_VEHICLE) {
    pitch = vehicleBaseAttitudePitchDeg + vehicleVisualPitchDeg;
    roll = vehicleBaseAttitudeRollDeg + vehicleVisualRollDeg;
    yaw = degrees(vehicleNavYaw);
  } else {
    pitch = degrees(dronePitch);
    roll = degrees(droneRoll);
    yaw = degrees(droneNavYaw);
  }
  attitude.setFloat("pitch_deg", pitch);
  attitude.setFloat("roll_deg", roll);
  attitude.setFloat("yaw_deg", yaw);
  attitude.setString("source", firmware ? "firmware_imu" : "local_model");
  attitude.setBoolean("support_surface_tilted", robotMode != ROBOT_MODE_DRONE && firmware);
  attitude.setBoolean("world_grid_level", true);
  return attitude;
}

JSONObject buildAiCompassSnapshot(int robotMode) {
  JSONObject compass = new JSONObject();
  boolean localAvailable = simulationMode || isLocalTelemetrySelected();
  boolean firmwareAvailable = latestSensors != null && latestSensors.hasKey("heading_deg") && getSensorFloat("heading_deg", -1.0f) >= 0.0f;
  boolean available = firmwareAvailable || localAvailable;
  float heading = normalizeAbsoluteAngleDeg(compassHeadingDegForRobotMode(robotMode));

  compass.setBoolean("available", available);
  compass.setFloat("heading_deg", heading);
  compass.setFloat("mag_x_raw", getSensorFloat("mag_x", 0.0f));
  compass.setFloat("mag_y_raw", getSensorFloat("mag_y", 0.0f));
  compass.setFloat("mag_z_raw", getSensorFloat("mag_z", 0.0f));
  compass.setBoolean("magnetometer_raw_available", latestSensors != null && hasAnySensorKey(new String[] {"mag_x", "mag_y", "mag_z"}));
  compass.setString("source", firmwareAvailable ? "firmware_compass" : (localAvailable ? "processing_navigation_model" : "unavailable"));
  compass.setString("role", "absolute_heading_and_north_reference");
  return compass;
}

JSONObject aiSensorChannel(String name, String role, String authority, boolean expected, boolean available, String transport) {
  JSONObject item = new JSONObject();
  item.setString("name", name);
  item.setString("role", role);
  item.setString("authority", authority);
  item.setBoolean("expected", expected);
  item.setBoolean("available", available);
  item.setString("transport", transport);
  return item;
}

JSONArray buildAiSensorChannels(int robotMode) {
  JSONArray channels = new JSONArray();
  channels.append(aiSensorChannel("processing_world_3d", "world_context", "context", true, true, "ws:9001"));
  channels.append(aiSensorChannel("audio_microphone", "voice_sound_music", "interaction", true, false, "aibot_local"));

  boolean localStateAvailable = simulationMode || isLocalTelemetrySelected();
  boolean compassAvailable = (latestSensors != null && latestSensors.hasKey("heading_deg") && getSensorFloat("heading_deg", -1.0f) >= 0.0f) || localStateAvailable;
  String compassAuthority = robotMode == ROBOT_MODE_MANIPULATOR ? "primary_state" : "primary_navigation";
  channels.append(aiSensorChannel("compass", "absolute_heading_and_north_reference", compassAuthority, true, compassAvailable, isFirmwareTelemetrySelected() ? "ws:9000" : "processing_local_model"));
  if (robotMode == ROBOT_MODE_MANIPULATOR) {
    // Processing knows the camera role, but only AiBot can confirm that an
    // actual image source is delivering frames. Python upgrades availability.
    channels.append(aiSensorChannel("workspace_camera", "gripper_workspace_and_object_view", "primary_vision", true, false, "robot_camera"));
    channels.append(aiSensorChannel("joint_telemetry", "arm_pose", "primary_state", true, latestSensors != null && latestSensors.size() > 0, isFirmwareTelemetrySelected() ? "ws:9000" : "processing_local_model"));
    channels.append(aiSensorChannel("base_imu", "whole_structure_attitude_mpu2", "primary_state", true, latestSensors != null && (latestSensors.hasKey("mpu2_ax") || localStateAvailable), robotUsesFirmwareAttitude(robotMode) ? "ws:9000" : "processing_local_model"));
    channels.append(aiSensorChannel("gripper_imu", "gripper_wrist_attitude_mpu1", "primary_state", true, latestSensors != null && (latestSensors.hasKey("mpu1_ax") || localStateAvailable), isFirmwareTelemetrySelected() ? "ws:9000" : "processing_local_model"));
    channels.append(aiSensorChannel("joint_torque_references", "per_joint_load_and_torque_reference", "primary_state", true, latestSensors != null && hasAnySensorKey(new String[] {"an2", "an3", "an4", "an5", "ina1_ma", "ina2_ma", "grip_pressure_ma"}), "ws:9000"));
    channels.append(aiSensorChannel("sonar", "workspace_range", "primary_safety", true, latestSensors != null && latestSensors.hasKey("sonar_cm"), "ws:9000"));
    channels.append(aiSensorChannel("battery", "energy_state", "resource_state", true, simulationMode || (latestSensors != null && hasAnySensorKey(new String[] {"battery_pct", "bat_pct"})), isFirmwareTelemetrySelected() ? "ws:9000" : "processing_local_model"));
  } else if (robotMode == ROBOT_MODE_VEHICLE) {
    channels.append(aiSensorChannel("front_camera", "forward_navigation_vision", "primary_vision", true, false, "robot_camera"));
    channels.append(aiSensorChannel("chassis_imu", "attitude_and_heading", "primary_state", true, robotUsesFirmwareAttitude(robotMode) || localStateAvailable, robotUsesFirmwareAttitude(robotMode) ? "ws:9000" : "processing_local_model"));
    channels.append(aiSensorChannel("position_telemetry", "local_position_and_motion", "primary_state", true, true, isFirmwareTelemetrySelected() ? "ws:9000" : "processing_local_model"));
    channels.append(aiSensorChannel("lidar", "obstacle_range", "primary_safety", true, latestSensors != null && hasAnySensorKey(new String[] {"lidar_cm", "lidar", "range_cm"}), "ws:9000"));
    channels.append(aiSensorChannel("gps", "global_position", "primary_navigation", true, vehicleGpsFixValid || simulationMode, "ws:9000"));
    channels.append(aiSensorChannel("drive_torque", "left_right_track_torque_reference", "primary_state", true, latestSensors != null && hasAnySensorKey(new String[] {"an2", "an3", "left_track_torque", "right_track_torque"}), "ws:9000"));
    channels.append(aiSensorChannel("battery", "energy_state", "resource_state", true, simulationMode || (latestSensors != null && hasAnySensorKey(new String[] {"battery_pct", "bat_pct"})), isFirmwareTelemetrySelected() ? "ws:9000" : "processing_local_model"));
    channels.append(aiSensorChannel("communication_link", "command_and_telemetry_link_quality", "resource_state", true, simulationMode || (latestSensors != null && hasAnySensorKey(new String[] {"communication_quality_pct", "link_quality_pct", "signal_pct", "rssi_pct", "rssi_dbm", "link_ms"})), "ws:9000"));
  } else {
    channels.append(aiSensorChannel("front_camera", "forward_navigation_vision", "primary_vision", true, false, "robot_camera"));
    channels.append(aiSensorChannel("flight_imu", "attitude_and_heading", "primary_state", true, robotUsesFirmwareAttitude(robotMode) || localStateAvailable, robotUsesFirmwareAttitude(robotMode) ? "ws:9000" : "processing_local_model"));
    channels.append(aiSensorChannel("position_telemetry", "local_position_altitude_and_motion", "primary_state", true, true, isFirmwareTelemetrySelected() ? "ws:9000" : "processing_local_model"));
    channels.append(aiSensorChannel("downward_sonar", "altitude_and_ground_clearance", "primary_safety", true, latestSensors != null && hasAnySensorKey(new String[] {"drone_sonar_down_cm", "sonar_down_cm", "sonar_vertical_cm", "sonar_cm", "drone_scan_cm"}), "ws:9000"));
    channels.append(aiSensorChannel("gps", "global_position", "primary_navigation", true, droneGpsFixValid || simulationMode, "ws:9000"));
    channels.append(aiSensorChannel("battery", "energy_state", "resource_state", true, simulationMode || (latestSensors != null && hasAnySensorKey(new String[] {"battery_pct", "bat_pct"})), isFirmwareTelemetrySelected() ? "ws:9000" : "processing_local_model"));
    channels.append(aiSensorChannel("communication_link", "command_and_telemetry_link_quality", "resource_state", true, simulationMode || (latestSensors != null && hasAnySensorKey(new String[] {"communication_quality_pct", "link_quality_pct", "signal_pct", "rssi_pct", "rssi_dbm", "link_ms"})), "ws:9000"));
  }
  return channels;
}

JSONObject buildAiRobotSensorValues(int robotMode) {
  JSONObject out = new JSONObject();
  out.setJSONObject("attitude", buildAiAttitudeSnapshot(robotMode));
  out.setJSONObject("compass", buildAiCompassSnapshot(robotMode));
  if (robotMode == ROBOT_MODE_MANIPULATOR) {
    JSONArray joints = new JSONArray();
    for (int i = 0; i <= GRIPPER_IDX; i++) joints.append(getManipulatorVisualServoAngle(i));
    out.setJSONArray("joint_deg", joints);
    out.setFloat("sonar_cm", getSensorFloat("sonar_cm", sonarDistanceCm));
    out.setJSONObject("base_imu", aiImuSnapshot(1, "base_structure"));
    out.setJSONObject("gripper_imu", aiImuSnapshot(0, "gripper_wrist"));
    out.setJSONObject("joint_torque_references", buildManipulatorJointTorqueReferences(latestSensors));
    out.setFloat("battery_pct", aiBatteryPercent(latestSensors));
  } else if (robotMode == ROBOT_MODE_VEHICLE) {
    out.setFloat("lidar_cm", getSensorFloatAny(new String[] {"lidar_cm", "lidar", "range_cm"}, vehicleLidarDistanceCm));
    out.setFloat("x_m", vehicleNavX / max(0.0001f, sceneUnitsPerMeterForRobotMode(robotMode)));
    out.setFloat("z_m", vehicleNavZ / max(0.0001f, sceneUnitsPerMeterForRobotMode(robotMode)));
    out.setBoolean("gps_fix", vehicleGpsFixValid);
    out.setFloat("gps_lat_deg", (float) vehicleGpsLatitudeDeg);
    out.setFloat("gps_lon_deg", (float) vehicleGpsLongitudeDeg);
    out.setFloat("gps_speed_kph", vehicleGpsSpeedKph);
    out.setFloat("camera_pan_deg", vehicleCameraPanDeg);
    out.setFloat("camera_tilt_deg", vehicleCameraTiltDeg);
    out.setJSONObject("chassis_imu", aiImuSnapshot(0, "vehicle_chassis"));
    out.setJSONObject("drive_torque", buildVehicleDriveTorqueReferences(latestSensors));
    out.setFloat("battery_pct", aiBatteryPercent(latestSensors));
    out.setFloat("communication_quality_pct", aiCommunicationQualityPct(latestSensors));
    out.setString("communication_quality_source", aiCommunicationQualitySource(latestSensors));
  } else {
    out.setFloat("sonar_down_cm", getSensorFloatAny(new String[] {"drone_sonar_down_cm", "sonar_down_cm", "sonar_cm", "drone_scan_cm"}, droneScannerDistanceCm));
    out.setFloat("altitude_m", max(0.0f, droneAltitudeCm) / 100.0f);
    out.setBoolean("gps_fix", droneGpsFixValid);
    out.setFloat("gps_lat_deg", (float) droneGpsLatitudeDeg);
    out.setFloat("gps_lon_deg", (float) droneGpsLongitudeDeg);
    out.setFloat("gps_speed_kph", droneGpsSpeedKph);
    out.setFloat("camera_pan_deg", droneCameraPanDeg);
    out.setFloat("camera_tilt_deg", droneCameraTiltDeg);
    out.setJSONObject("flight_imu", aiImuSnapshot(0, "drone_body"));
    out.setFloat("battery_pct", aiBatteryPercent(latestSensors));
    out.setFloat("communication_quality_pct", aiCommunicationQualityPct(latestSensors));
    out.setString("communication_quality_source", aiCommunicationQualitySource(latestSensors));
  }
  return out;
}

JSONObject buildAiAugmentedSensorsForRobot(String robotName, JSONObject sensors) {
  JSONObject out = cloneJsonObjectShallow(sensors);
  String normalized = normalizeRobotTypeName(robotName);
  int robotMode = normalized.equals("Vehicle") ? ROBOT_MODE_VEHICLE : (normalized.equals("Drone") ? ROBOT_MODE_DRONE : ROBOT_MODE_MANIPULATOR);
  JSONObject attitude = buildAiAttitudeSnapshot(robotMode);
  float pitch = getJsonFloat(attitude, "pitch_deg", 0.0f);
  float roll = getJsonFloat(attitude, "roll_deg", 0.0f);
  float yaw = getJsonFloat(attitude, "yaw_deg", 0.0f);
  JSONObject compass = buildAiCompassSnapshot(robotMode);
  out.setBoolean("compass_available", getJsonBoolean(compass, "available", false));
  out.setFloat("compass_heading_deg", getJsonFloat(compass, "heading_deg", yaw));
  out.setString("compass_source", getJsonString(compass, "source", "unavailable"));
  if (!out.hasKey("heading_deg")) out.setFloat("heading_deg", getJsonFloat(compass, "heading_deg", yaw));

  if (robotMode == ROBOT_MODE_MANIPULATOR) {
    if (!out.hasKey("base_pitch_deg")) out.setFloat("base_pitch_deg", pitch);
    if (!out.hasKey("base_roll_deg")) out.setFloat("base_roll_deg", roll);
    if (!out.hasKey("base_yaw_deg")) out.setFloat("base_yaw_deg", yaw);
    String[] torqueJointNames = {"base", "upper", "fore", "forearm_roll", "wrist_pitch", "wrist_rot", "grip"};
    for (int i = 0; i < torqueJointNames.length; i++) {
      String joint = torqueJointNames[i];
      String source = aiFirstSensorKey(out, manipulatorJointTorqueKeys(joint));
      if (source.length() > 0) {
        out.setFloat(joint + "_torque_ref", getJsonFloat(out, source, 0.0f));
        out.setString(joint + "_torque_source", source);
      }
    }
    if (!out.hasKey("battery_pct")) out.setFloat("battery_pct", aiBatteryPercent(out));
    if (!out.hasKey("bat_pct")) out.setFloat("bat_pct", aiBatteryPercent(out));
  } else if (robotMode == ROBOT_MODE_VEHICLE) {
    if (!out.hasKey("vehicle_pitch_deg")) out.setFloat("vehicle_pitch_deg", pitch);
    if (!out.hasKey("vehicle_roll_deg")) out.setFloat("vehicle_roll_deg", roll);
    if (!out.hasKey("vehicle_yaw_deg")) out.setFloat("vehicle_yaw_deg", yaw);
    if (!out.hasKey("vehicle_x")) out.setFloat("vehicle_x", vehicleNavX);
    if (!out.hasKey("vehicle_z")) out.setFloat("vehicle_z", vehicleNavZ);
    JSONObject driveTorque = buildVehicleDriveTorqueReferences(out);
    out.setFloat("vehicle_left_torque_ref", getJsonFloat(driveTorque, "left_reference", 0.0f));
    out.setFloat("vehicle_right_torque_ref", getJsonFloat(driveTorque, "right_reference", 0.0f));
    if (!out.hasKey("battery_pct")) out.setFloat("battery_pct", aiBatteryPercent(out));
    if (!out.hasKey("bat_pct")) out.setFloat("bat_pct", aiBatteryPercent(out));
    out.setFloat("communication_quality_pct", aiCommunicationQualityPct(out));
    out.setString("communication_quality_source", aiCommunicationQualitySource(out));
  } else {
    if (!out.hasKey("drone_pitch_deg")) out.setFloat("drone_pitch_deg", pitch);
    if (!out.hasKey("drone_roll_deg")) out.setFloat("drone_roll_deg", roll);
    if (!out.hasKey("drone_yaw_deg")) out.setFloat("drone_yaw_deg", yaw);
    if (!out.hasKey("drone_x")) out.setFloat("drone_x", droneNavX);
    if (!out.hasKey("drone_y")) out.setFloat("drone_y", max(0.0f, droneAltitudeCm));
    if (!out.hasKey("drone_z")) out.setFloat("drone_z", droneNavZ);
    if (!out.hasKey("alt_cm")) out.setFloat("alt_cm", max(0.0f, droneAltitudeCm));
    if (!out.hasKey("battery_pct")) out.setFloat("battery_pct", aiBatteryPercent(out));
    if (!out.hasKey("bat_pct")) out.setFloat("bat_pct", aiBatteryPercent(out));
    out.setFloat("communication_quality_pct", aiCommunicationQualityPct(out));
    out.setString("communication_quality_source", aiCommunicationQualitySource(out));
  }
  return out;
}

JSONObject buildAiRobotSensorManifest(int robotMode) {
  JSONObject manifest = new JSONObject();
  manifest.setString("schema", SYNROV_ROBOT_SENSOR_MANIFEST_SCHEMA);
  manifest.setInt("softwareVersion", SYNROV_SOFTWARE_VERSION);
  manifest.setString("robot", robotMode == ROBOT_MODE_VEHICLE ? "Vehicle" : (robotMode == ROBOT_MODE_DRONE ? "Drone" : "Manipulator"));
  manifest.setString("camera_role", robotMode == ROBOT_MODE_MANIPULATOR ? "workspace" : "front_navigation");
  manifest.setString("music_policy", robotMode == ROBOT_MODE_MANIPULATOR ? "actuation_allowed_with_operator_enable" : "context_only");
  manifest.setString("voice_policy", "command_and_mission_input");
  manifest.setString("world_3d_policy", "context_and_collision_complement");
  manifest.setJSONArray("channels", buildAiSensorChannels(robotMode));
  manifest.setJSONObject("values", buildAiRobotSensorValues(robotMode));
  manifest.setJSONObject("inputs", buildAiInputActivitySnapshot());
  return manifest;
}
