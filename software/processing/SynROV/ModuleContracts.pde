// =====================================================================
// SynROV Processing - Robot module contract
// ---------------------------------------------------------------------
// All three robot modules are part of the current build. Registration is direct:
// there is no reflective loader, optional-module shim or placeholder module.
// =====================================================================

interface SynRovModule {
  public void loadConfigSafe();
  public void updatePhysicsSafe();
  public void drawModule3D(boolean advanceCamera);
  public void resetInputStateSafe();
  public int[] captureFrameSafe();
  public void applyFrameSafe(int[] frame);
  public boolean moveSafelyTowardFrameSafe(int[] targetFrame, int maxStepDeg, int toleranceDeg);
  public JSONObject buildConfigObjectSafe();
}

abstract class SynRovModuleBase implements SynRovModule {
  protected abstract String moduleName();
  protected abstract String configFilePath();
  protected abstract void loadConfig();

  protected void recoverRuntimeAfterConfigFailure() {
  }

  public final void loadConfigSafe() {
    try {
      loadConfig();
      return;
    }
    catch (Exception firstFailure) {
      println(moduleName() + " config reset: " + firstFailure.getMessage());
    }

    try {
      saveJSONObjectEnsured(buildConfigObjectSafe(), configFilePath());
      loadConfig();
    }
    catch (Exception recoveryFailure) {
      println(moduleName() + " config recovery failed: " + recoveryFailure.getMessage());
      recoverRuntimeAfterConfigFailure();
    }
  }
}

boolean isRobotConfigHeader(JSONObject json, String robotName) {
  if (json == null) return false;
  if (!"synrov.robot_config".equals(getJsonString(json, "schema", ""))) return false;
  if (!hasCurrentSoftwareVersion(json)) return false;
  return robotName != null && robotName.equals(normalizeRobotTypeName(getJsonString(json, "robot", "")));
}

boolean hasJsonObjects(JSONObject parent, String... keys) {
  if (parent == null || keys == null) return false;
  for (int i = 0; i < keys.length; i++) {
    if (getJsonObjectSafe(parent, keys[i]) == null) return false;
  }
  return true;
}

final String[] SYNROV_MODULE_NAMES = {"Manipulator", "Vehicle", "Drone"};
HashMap<String, SynRovModule> synRovModules = new HashMap<String, SynRovModule>();
boolean synRovModulesRegistered = false;

// Registers the three robot modules explicitly.
void registerSynRovModules() {
  synRovModules.clear();
  synRovModules.put("Manipulator", new Manipulator3DModule());
  synRovModules.put("Vehicle", new Vehicle3DModule());
  synRovModules.put("Drone", new Drone3DModule());
  synRovModulesRegistered = true;
}

boolean synRovModuleRegistryComplete() {
  if (!synRovModulesRegistered || synRovModules.size() != SYNROV_MODULE_NAMES.length) return false;
  for (int i = 0; i < SYNROV_MODULE_NAMES.length; i++) {
    if (synRovModules.get(SYNROV_MODULE_NAMES[i]) == null) return false;
  }
  return true;
}

void ensureSynRovModulesRegistered() {
  if (!synRovModuleRegistryComplete()) registerSynRovModules();
}

SynRovModule moduleForRobot(String robotName) {
  ensureSynRovModulesRegistered();
  String normalized = normalizeRobotTypeName(robotName);
  if (normalized.length() == 0) normalized = "Manipulator";
  SynRovModule module = synRovModules.get(normalized);
  return module == null ? synRovModules.get("Manipulator") : module;
}

SynRovModule activeModule() {
  return moduleForRobot(currentModeName());
}

void loadAllModuleConfigsSafely() {
  ensureSynRovModulesRegistered();
  for (int i = 0; i < SYNROV_MODULE_NAMES.length; i++) {
    moduleForRobot(SYNROV_MODULE_NAMES[i]).loadConfigSafe();
  }
  ensureRobotTelemetryDefaults();
}

// =====================================================================
// Robot module adapters
// ---------------------------------------------------------------------
// All adapter implementations live together so lifecycle/config recovery,
// playback and rendering contracts can be reviewed in one place.
// =====================================================================

class Manipulator3DModule extends SynRovModuleBase {
  protected String moduleName() {
    return "Manipulator";
  }

  protected String configFilePath() {
    return MANIPULATOR_CONFIG_FILE;
  }

  protected void loadConfig() {
    loadOrCreateManipulatorConfig();
  }

  protected void recoverRuntimeAfterConfigFailure() {
    updateDimensions();
  }

  public void updatePhysicsSafe() { }

  public void drawModule3D(boolean advanceCamera) {
    drawManipulator3D(advanceCamera);
  }

  public void resetInputStateSafe() {
    resetPendingManipulatorPoseCommand();
  }

  public int[] captureFrameSafe() {
    return new int[] {
      angles[BASE_IDX], angles[UPPERARM_IDX], angles[FOREARM_IDX],
      angles[FOREARM_ROLL_IDX], angles[WRIST_VERT_IDX], angles[WRIST_ROT_IDX], angles[GRIPPER_IDX]
    };
  }

  public void applyFrameSafe(int[] frame) {
    if (frame == null) return;
    int manipCount = min(min(frame.length, GRIPPER_IDX + 1), servoLimits.length);
    int[] requestedAngles = copyManipulatorCommandPose();
    boolean[] activeMask = new boolean[GRIPPER_IDX + 1];
    for (int i = 0; i < manipCount; i++) {
      requestedAngles[i] = normalizeManipulatorTargetValue(i, frame[i]);
      activeMask[i] = true;
    }

    setCommandContext(playbackCommandSource);
    try {
      applyManipulatorPoseTargets(requestedAngles, activeMask);
    }
    finally {
      restoreLocalCommandContext();
    }
  }

  public boolean moveSafelyTowardFrameSafe(int[] targetFrame, int maxStepDeg, int toleranceDeg) {
    if (targetFrame == null) return true;
    int manipCount = min(min(targetFrame.length, manipulatorCommandAnglesDeg.length), servoLimits.length);
    int[] requestedAngles = copyManipulatorCommandPose();
    boolean[] activeMask = new boolean[GRIPPER_IDX + 1];

    for (int i = 0; i < manipCount; i++) {
      int currentCommand = getManipulatorCommandTargetInt(i);
      int target = normalizeManipulatorTargetValue(i, targetFrame[i]);
      int delta = target - currentCommand;
      if (delta != 0) {
        int step = constrain(delta, -maxStepDeg, maxStepDeg);
        requestedAngles[i] = normalizeManipulatorTargetValue(i, currentCommand + step);
        activeMask[i] = true;
      }
    }

    setCommandContext(playbackCommandSource);
    try {
      applyManipulatorPoseTargets(requestedAngles, activeMask);
    }
    finally {
      restoreLocalCommandContext();
    }

    // Command interpolation and physical arrival are deliberately separate:
    // command reference keeps advancing; playback starts only after feedback is
    // actually within tolerance of the recorded first frame.
    return isManipulatorFrameWithinTolerance(targetFrame, toleranceDeg);
  }

  public JSONObject buildConfigObjectSafe() {
    return buildManipulatorConfigObject();
  }
}

class Vehicle3DModule extends SynRovModuleBase {
  protected String moduleName() {
    return "Vehicle";
  }

  protected String configFilePath() {
    return VEHICLE_CONFIG_FILE;
  }

  protected void loadConfig() {
    loadOrCreateVehicleConfig();
  }

  protected void recoverRuntimeAfterConfigFailure() {
    resetVehicleInputState();
  }


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

class Drone3DModule extends SynRovModuleBase {
  protected String moduleName() {
    return "Drone";
  }

  protected String configFilePath() {
    return DRONE_CONFIG_FILE;
  }

  protected void loadConfig() {
    loadOrCreateDroneConfig();
  }

  protected void recoverRuntimeAfterConfigFailure() {
    cancelDroneAutomaticVerticalMotion();
    resetDroneInputState();
    updateDroneDimensions();
  }


  public void updatePhysicsSafe() {
    updateDronePhysics();
  }

  public void drawModule3D(boolean advanceCamera) {
    drawDrone3D(advanceCamera);
  }

  public void resetInputStateSafe() {
    cancelDroneAutomaticVerticalMotion();
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
