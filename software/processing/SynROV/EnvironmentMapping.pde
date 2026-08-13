// =====================================================================
// SynROV Processing - Environment mapping
// ---------------------------------------------------------------------
// Purpose:
//   Scene environment drawing, overlays and mapping utilities.
// =====================================================================

ArrayList<PVector> manipulatorEnvironmentPoints = new ArrayList<PVector>();
ArrayList<PVector> vehicleEnvironmentPoints = new ArrayList<PVector>();
ArrayList<PVector> droneEnvironmentPoints = new ArrayList<PVector>();
PVector manipulatorSonarProbeWorld = null;
PVector vehicleRadarProbeWorld = null;
PVector droneDownwardSonarProbeWorld = null;

ArrayList<PVector> manipulatorImportedWorldPoints = new ArrayList<PVector>();
ArrayList<PVector> vehicleImportedWorldPoints = new ArrayList<PVector>();
ArrayList<PVector> droneImportedWorldPoints = new ArrayList<PVector>();

// Imported worlds keep their original triangle geometry for direct rendering.
// The point lists above remain collision/sensor samples only and are never used
// as the visual representation of an imported mesh.
MeshData manipulatorImportedWorldMesh = new MeshData();
MeshData vehicleImportedWorldMesh = new MeshData();
MeshData droneImportedWorldMesh = new MeshData();
MeshData manipulatorImportedWorldBaseMesh = new MeshData();
MeshData vehicleImportedWorldBaseMesh = new MeshData();
MeshData droneImportedWorldBaseMesh = new MeshData();
ArrayList<PVector> manipulatorImportedWorldBasePoints = new ArrayList<PVector>();
ArrayList<PVector> vehicleImportedWorldBasePoints = new ArrayList<PVector>();
ArrayList<PVector> droneImportedWorldBasePoints = new ArrayList<PVector>();

ArrayList<PVector> manipulatorDemoWorldPoints = new ArrayList<PVector>();
ArrayList<PVector> vehicleDemoWorldPoints = new ArrayList<PVector>();
ArrayList<PVector> droneDemoWorldPoints = new ArrayList<PVector>();
boolean manipulatorDemoWorldReady = false;
boolean vehicleDemoWorldReady = false;
boolean droneDemoWorldReady = false;
boolean manipulatorDemoWorldEnabled = true;
boolean vehicleDemoWorldEnabled = true;
boolean droneDemoWorldEnabled = true;
boolean bootDemoWorldEnabled = true;

String manipulatorWorldSourceLabel = "";
String vehicleWorldSourceLabel = "";
String droneWorldSourceLabel = "";
float manipulatorImportedWorldScale = 1.0f;
float vehicleImportedWorldScale = 1.0f;
float droneImportedWorldScale = 1.0f;
float manipulatorImportedWorldUnitsToCm = 1.0f;
float vehicleImportedWorldUnitsToCm = 1.0f;
float droneImportedWorldUnitsToCm = 1.0f;
float manipulatorWorldCompassNorthOffsetDeg = 0.0f;
float vehicleWorldCompassNorthOffsetDeg = 0.0f;
float droneWorldCompassNorthOffsetDeg = 0.0f;
boolean manipulatorWorldCompassNorthLocked = false;
boolean vehicleWorldCompassNorthLocked = false;
boolean droneWorldCompassNorthLocked = false;

boolean environmentCollisionEnabled = true;
// A sensor-saved .synrovworld carries its own solid-collision property for
// exactly the robot type that created it. This is independent from the
// operator's global collision toggle and never leaks across robot profiles.
boolean manipulatorCollisionWorldSolid = false;
boolean vehicleCollisionWorldSolid = false;
boolean droneCollisionWorldSolid = false;

boolean collisionWorldSolidForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleCollisionWorldSolid;
  if (robotMode == ROBOT_MODE_DRONE) return droneCollisionWorldSolid;
  return manipulatorCollisionWorldSolid;
}

void setCollisionWorldSolidForRobotMode(int robotMode, boolean solid) {
  if (robotMode == ROBOT_MODE_VEHICLE) vehicleCollisionWorldSolid = solid;
  else if (robotMode == ROBOT_MODE_DRONE) droneCollisionWorldSolid = solid;
  else manipulatorCollisionWorldSolid = solid;
  invalidateEnvCollisionCache();
}

boolean environmentCollisionActiveForRobotMode(int robotMode) {
  return environmentCollisionEnabled || collisionWorldSolidForRobotMode(robotMode);
}
float environmentPointRadius = 1.8f;
float environmentScanMinCm = 3.0f;
float environmentScanMaxCm = 260.0f;
float vehicleRadarCollisionMaxCm = 200.0f;
float manipulatorSonarDetectionMaxCm = 400.0f;
float droneDownwardSonarDetectionMaxCm = 400.0f;
int lastEnvironmentScanMillis = 0;
int environmentScanIntervalMs = 60;
final int ENVIRONMENT_MAX_POINTS_PER_MAP = 9000;
final int IMPORTED_WORLD_MAX_POINTS = 3200;
final int IMPORTED_WORLD_SIMPLE_RENDER_THRESHOLD = 720;
final int ENVIRONMENT_RENDER_POINT_BUDGET = 1800;

boolean manipulatorAutoScan = true;
boolean vehicleAutoScan = true;
boolean droneAutoScan = true;

final int ROBOT_MODE_MANIPULATOR = 0;
final int ROBOT_MODE_VEHICLE = 1;
final int ROBOT_MODE_DRONE = 2;

volatile boolean worldImportDialogOpen = false;
volatile boolean worldImportBusy = false;
int queuedWorldImportRobotMode = ROBOT_MODE_MANIPULATOR;
volatile boolean collisionWorldSaveDialogOpen = false;
int queuedCollisionWorldSaveRobotMode = ROBOT_MODE_MANIPULATOR;
final String COLLISION_WORLD_SCHEMA = "synrov.collision-world";
final String COLLISION_WORLD_EXTENSION = "synrovworld";
Object worldImportLock = new Object();
ImportMeshResult pendingWorldImportResult = null;

boolean worldTransformOverlayActive = false;
boolean worldTransformOverlayDragging = false;
boolean worldTransformOverlayPanelPointerDown = false;
boolean worldTransformMouseAdjustEnabled = false;
int worldTransformOverlayRobotMode = ROBOT_MODE_MANIPULATOR;
float manipulatorWorldTransformReferenceYaw = 0.0f;
float vehicleWorldTransformReferenceYaw = 0.0f;
float droneWorldTransformReferenceYaw = 0.0f;
float manipulatorWorldUserScale = 1.0f;
float vehicleWorldUserScale = 1.0f;
float droneWorldUserScale = 1.0f;
PVector manipulatorWorldUserOffset = new PVector(0, 0, 0);
PVector vehicleWorldUserOffset = new PVector(0, 0, 0);
PVector droneWorldUserOffset = new PVector(0, 0, 0);
float manipulatorWorldUserRotationDeg = 0.0f;
float vehicleWorldUserRotationDeg = 0.0f;
float droneWorldUserRotationDeg = 0.0f;
float manipulatorWorldUserPitchDeg = 0.0f;
float vehicleWorldUserPitchDeg = 0.0f;
float droneWorldUserPitchDeg = 0.0f;
float manipulatorWorldUserRollDeg = 0.0f;
float vehicleWorldUserRollDeg = 0.0f;
float droneWorldUserRollDeg = 0.0f;

// Per-world transform profiles persist the exact operator adjustment sequence.
// Profiles live under data/world_profiles/<robot>/<world-key>/profile.json.
final String WORLD_PROFILE_SCHEMA = "synrov.world-profile";
final String WORLD_PROFILE_ROOT = "data/world_profiles";
File manipulatorWorldSourceFile = null;
File vehicleWorldSourceFile = null;
File droneWorldSourceFile = null;
ArrayList<WorldTransformOperation> manipulatorWorldTransformHistory = new ArrayList<WorldTransformOperation>();
ArrayList<WorldTransformOperation> vehicleWorldTransformHistory = new ArrayList<WorldTransformOperation>();
ArrayList<WorldTransformOperation> droneWorldTransformHistory = new ArrayList<WorldTransformOperation>();
boolean worldTransformHistoryReplayActive = false;

final float WORLD_TRANSFORM_SCALE_STEP = 1.06f;
final float WORLD_TRANSFORM_NUDGE_STEP = 12.0f;
final float WORLD_TRANSFORM_ROTATION_STEP_DEG = 7.5f;

// Utility: robot mode for environment point list.
int robotModeForEnvironmentPointList(ArrayList<PVector> list) {
  if (list == vehicleEnvironmentPoints || list == vehicleImportedWorldPoints || list == vehicleDemoWorldPoints) return ROBOT_MODE_VEHICLE;
  if (list == droneEnvironmentPoints || list == droneImportedWorldPoints || list == droneDemoWorldPoints) return ROBOT_MODE_DRONE;
  return ROBOT_MODE_MANIPULATOR;
}

// Utility: environment point radius for robot mode.
float environmentPointRadiusForRobotMode(int robotMode) {
  return environmentPointRadius * robotPresentationScaleForMode(robotMode);
}

// Utility: current environment point radius.
float currentEnvironmentPointRadius() {
  return environmentPointRadiusForRobotMode(currentRobotMode());
}


// Utility: demo world ground half size for robot mode.
float demoWorldGroundHalfSizeForRobotMode(int robotMode) {
  return visualGroundSizeForRobotMode(robotMode) * 0.46f;
}

// Utility: demo world grid spacing for robot mode.
float demoWorldGridSpacingForRobotMode(int robotMode) {
  return max(12.0f, demoWorldGroundHalfSizeForRobotMode(robotMode) * 0.16f);
}

// Utility: demo world reference span for robot mode.
float demoWorldReferenceSpanForRobotMode(int robotMode) {
  float robotSpan = robotReferenceSpanScene(robotMode);
  float groundHalf = demoWorldGroundHalfSizeForRobotMode(robotMode);
  float groundDrivenSpan = groundHalf * 0.92f;
  return max(robotSpan, groundDrivenSpan);
}

// Utility: current robot mode.
int currentRobotMode() {
  if (isVehicleSelected) return ROBOT_MODE_VEHICLE;
  if (isDroneSelected) return ROBOT_MODE_DRONE;
  return ROBOT_MODE_MANIPULATOR;
}

// Utility: current environment auto scan enabled.
boolean currentEnvironmentAutoScanEnabled() {
  if (isVehicleSelected) return vehicleAutoScan;
  if (isDroneSelected) return droneAutoScan;
  return manipulatorAutoScan;
}

// Sets current environment auto scan.
void setCurrentEnvironmentAutoScan(boolean enabled) {
  if (isVehicleSelected) {
    vehicleAutoScan = enabled;
  } else if (isDroneSelected) {
    droneAutoScan = enabled;
  } else {
    manipulatorAutoScan = enabled;
  }
  environmentAutoScan = currentEnvironmentAutoScanEnabled();
}

// Utility: environment point list for robot mode.
ArrayList<PVector> environmentPointListForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleEnvironmentPoints;
  if (robotMode == ROBOT_MODE_DRONE) return droneEnvironmentPoints;
  return manipulatorEnvironmentPoints;
}

// Utility: imported world point list for robot mode.
ArrayList<PVector> importedWorldPointListForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleImportedWorldPoints;
  if (robotMode == ROBOT_MODE_DRONE) return droneImportedWorldPoints;
  return manipulatorImportedWorldPoints;
}

// Utility: direct imported world mesh for robot mode.
MeshData importedWorldMeshForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleImportedWorldMesh;
  if (robotMode == ROBOT_MODE_DRONE) return droneImportedWorldMesh;
  return manipulatorImportedWorldMesh;
}

// Utility: immutable import baseline mesh for robot mode.
MeshData importedWorldBaseMeshForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleImportedWorldBaseMesh;
  if (robotMode == ROBOT_MODE_DRONE) return droneImportedWorldBaseMesh;
  return manipulatorImportedWorldBaseMesh;
}

// Utility: immutable import baseline collision samples for robot mode.
ArrayList<PVector> importedWorldBasePointListForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleImportedWorldBasePoints;
  if (robotMode == ROBOT_MODE_DRONE) return droneImportedWorldBasePoints;
  return manipulatorImportedWorldBasePoints;
}

// Utility: demo world point list for robot mode.
ArrayList<PVector> demoWorldPointListForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleDemoWorldPoints;
  if (robotMode == ROBOT_MODE_DRONE) return droneDemoWorldPoints;
  return manipulatorDemoWorldPoints;
}

// Utility: current environment point list.
ArrayList<PVector> currentEnvironmentPointList() {
  return environmentPointListForRobotMode(currentRobotMode());
}

// Utility: current imported world point list.
ArrayList<PVector> currentImportedWorldPointList() {
  return importedWorldPointListForRobotMode(currentRobotMode());
}

// Utility: current demo world point list.
ArrayList<PVector> currentDemoWorldPointList() {
  return demoWorldPointListForRobotMode(currentRobotMode());
}

// Utility: demo world enabled for robot mode.
boolean demoWorldEnabledForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleDemoWorldEnabled;
  if (robotMode == ROBOT_MODE_DRONE) return droneDemoWorldEnabled;
  return manipulatorDemoWorldEnabled;
}

// Sets demo world enabled for robot mode.
void setDemoWorldEnabledForRobotMode(int robotMode, boolean enabled) {
  if (robotMode == ROBOT_MODE_VEHICLE) {
    vehicleDemoWorldEnabled = enabled;
  } else if (robotMode == ROBOT_MODE_DRONE) {
    droneDemoWorldEnabled = enabled;
  } else {
    manipulatorDemoWorldEnabled = enabled;
  }
}



// Utility: world user scale for robot mode.
float worldUserScaleForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleWorldUserScale;
  if (robotMode == ROBOT_MODE_DRONE) return droneWorldUserScale;
  return manipulatorWorldUserScale;
}

// Utility: world user offset for robot mode.
PVector worldUserOffsetForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleWorldUserOffset;
  if (robotMode == ROBOT_MODE_DRONE) return droneWorldUserOffset;
  return manipulatorWorldUserOffset;
}

float worldUserRotationDegForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleWorldUserRotationDeg;
  if (robotMode == ROBOT_MODE_DRONE) return droneWorldUserRotationDeg;
  return manipulatorWorldUserRotationDeg;
}

float worldUserPitchDegForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleWorldUserPitchDeg;
  if (robotMode == ROBOT_MODE_DRONE) return droneWorldUserPitchDeg;
  return manipulatorWorldUserPitchDeg;
}

float worldUserRollDegForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleWorldUserRollDeg;
  if (robotMode == ROBOT_MODE_DRONE) return droneWorldUserRollDeg;
  return manipulatorWorldUserRollDeg;
}

void addWorldUserRotationDegForRobotMode(int robotMode, float deltaDeg) {
  if (robotMode == ROBOT_MODE_VEHICLE) vehicleWorldUserRotationDeg += deltaDeg;
  else if (robotMode == ROBOT_MODE_DRONE) droneWorldUserRotationDeg += deltaDeg;
  else manipulatorWorldUserRotationDeg += deltaDeg;
}

void addWorldUserPitchDegForRobotMode(int robotMode, float deltaDeg) {
  if (robotMode == ROBOT_MODE_VEHICLE) vehicleWorldUserPitchDeg += deltaDeg;
  else if (robotMode == ROBOT_MODE_DRONE) droneWorldUserPitchDeg += deltaDeg;
  else manipulatorWorldUserPitchDeg += deltaDeg;
}

void addWorldUserRollDegForRobotMode(int robotMode, float deltaDeg) {
  if (robotMode == ROBOT_MODE_VEHICLE) vehicleWorldUserRollDeg += deltaDeg;
  else if (robotMode == ROBOT_MODE_DRONE) droneWorldUserRollDeg += deltaDeg;
  else manipulatorWorldUserRollDeg += deltaDeg;
}

// Utility: world transform reference yaw for robot mode.
float worldTransformReferenceYawForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleWorldTransformReferenceYaw;
  if (robotMode == ROBOT_MODE_DRONE) return droneWorldTransformReferenceYaw;
  return manipulatorWorldTransformReferenceYaw;
}

// Sets world transform reference yaw for robot mode.
void setWorldTransformReferenceYawForRobotMode(int robotMode, float yaw) {
  if (robotMode == ROBOT_MODE_VEHICLE) vehicleWorldTransformReferenceYaw = yaw;
  else if (robotMode == ROBOT_MODE_DRONE) droneWorldTransformReferenceYaw = yaw;
  else manipulatorWorldTransformReferenceYaw = yaw;
}

// Utility: source file for the imported world in one robot mode.
File worldSourceFileForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleWorldSourceFile;
  if (robotMode == ROBOT_MODE_DRONE) return droneWorldSourceFile;
  return manipulatorWorldSourceFile;
}

// Sets the imported world source file for one robot mode.
void setWorldSourceFileForRobotMode(int robotMode, File sourceFile) {
  if (robotMode == ROBOT_MODE_VEHICLE) vehicleWorldSourceFile = sourceFile;
  else if (robotMode == ROBOT_MODE_DRONE) droneWorldSourceFile = sourceFile;
  else manipulatorWorldSourceFile = sourceFile;
}

// Utility: exact transform operation history for one robot mode.
ArrayList<WorldTransformOperation> worldTransformHistoryForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleWorldTransformHistory;
  if (robotMode == ROBOT_MODE_DRONE) return droneWorldTransformHistory;
  return manipulatorWorldTransformHistory;
}

// Clears the exact transform history without changing the geometry.
void clearWorldTransformHistoryForRobotMode(int robotMode) {
  worldTransformHistoryForRobotMode(robotMode).clear();
}

// Records one transform operation. Consecutive operations of the same kind are
// compacted so mouse dragging does not create an unnecessarily large profile.
void recordWorldTransformOperation(int robotMode, String type, char axis, float a, float b) {
  if (worldTransformHistoryReplayActive) return;
  ArrayList<WorldTransformOperation> history = worldTransformHistoryForRobotMode(robotMode);
  if (history == null) return;
  String safeType = type == null ? "" : trim(type).toLowerCase();
  if (safeType.length() == 0) return;

  if (!history.isEmpty()) {
    WorldTransformOperation last = history.get(history.size() - 1);
    if (safeType.equals("translate") && last.type.equals("translate")) {
      last.a += a;
      last.b += b;
      return;
    }
    if (safeType.equals("scale") && last.type.equals("scale")) {
      last.a *= a;
      return;
    }
    if (safeType.equals("rotate") && last.type.equals("rotate") && last.axis == axis) {
      last.a += a;
      return;
    }
  }
  history.add(new WorldTransformOperation(safeType, axis, a, b));
}

// Utility: stable robot directory name used by world profiles.
String worldProfileRobotSlug(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return "vehicle";
  if (robotMode == ROBOT_MODE_DRONE) return "drone";
  return "manipulator";
}

// Utility: safe local file component.
String sanitizeWorldProfileComponent(String value) {
  String safe = value == null ? "world" : trim(value).toLowerCase();
  if (safe.length() == 0) safe = "world";
  safe = safe.replaceAll("[^a-z0-9._-]+", "-");
  safe = safe.replaceAll("^-+|-+$", "");
  if (safe.length() == 0) safe = "world";
  if (safe.length() > 52) safe = safe.substring(0, 52);
  return safe;
}

// Utility: canonical source path with a safe fallback.
String canonicalWorldSourcePath(File sourceFile) {
  if (sourceFile == null) return "";
  try {
    return sourceFile.getCanonicalPath();
  }
  catch (Exception e) {
    return sourceFile.getAbsolutePath();
  }
}

// Utility: compact path hash. File size and timestamp are validated separately.
String worldProfilePathHash(File sourceFile) {
  String path = canonicalWorldSourcePath(sourceFile).toLowerCase();
  java.util.zip.CRC32 crc = new java.util.zip.CRC32();
  try {
    byte[] data = path.getBytes("UTF-8");
    crc.update(data, 0, data.length);
  }
  catch (Exception e) {
    byte[] data = path.getBytes();
    crc.update(data, 0, data.length);
  }
  String hex = Long.toHexString(crc.getValue());
  while (hex.length() < 8) hex = "0" + hex;
  return hex;
}

// Utility: directory dedicated to one source world and one robot mode.
File worldProfileDirectoryForSource(File sourceFile, int robotMode) {
  if (sourceFile == null) return null;
  String fileName = sourceFile.getName();
  int dot = fileName.lastIndexOf('.');
  String stem = dot > 0 ? fileName.substring(0, dot) : fileName;
  String key = sanitizeWorldProfileComponent(stem) + "-" + worldProfilePathHash(sourceFile);
  return new File(sketchPath(WORLD_PROFILE_ROOT + "/" + worldProfileRobotSlug(robotMode) + "/" + key));
}

// Utility: profile JSON file for one imported source.
File worldProfileFileForSource(File sourceFile, int robotMode) {
  File dir = worldProfileDirectoryForSource(sourceFile, robotMode);
  return dir == null ? null : new File(dir, "profile.json");
}

// Utility: exact file metadata represented as strings to avoid JSON integer precision issues.
String worldSourceLengthText(File sourceFile) {
  return sourceFile == null ? "0" : Long.toString(sourceFile.length());
}

String worldSourceModifiedText(File sourceFile) {
  return sourceFile == null ? "0" : Long.toString(sourceFile.lastModified());
}

// Utility: loads a profile only when it belongs to this exact source file and current software version.
JSONObject loadWorldProfileForSource(File sourceFile, int robotMode) {
  File profileFile = worldProfileFileForSource(sourceFile, robotMode);
  if (sourceFile == null || profileFile == null || !profileFile.exists()) return null;
  try {
    JSONObject profile = loadJSONObject(profileFile.getAbsolutePath());
    if (profile == null) return null;
    if (!WORLD_PROFILE_SCHEMA.equals(getJsonString(profile, "schema", ""))) return null;
    if (getJsonInt(profile, "softwareVersion", -1) != SYNROV_SOFTWARE_VERSION) return null;
    if (!worldProfileRobotSlug(robotMode).equals(getJsonString(profile, "robot", ""))) return null;

    JSONObject source = getJsonObjectSafe(profile, "source");
    if (source == null) return null;
    if (!canonicalWorldSourcePath(sourceFile).equals(getJsonString(source, "path", ""))) return null;
    if (!worldSourceLengthText(sourceFile).equals(getJsonString(source, "size_bytes", ""))) return null;
    if (!worldSourceModifiedText(sourceFile).equals(getJsonString(source, "last_modified_ms", ""))) return null;
    return profile;
  }
  catch (Exception e) {
    println("World profile rejected: " + e.getMessage());
    return null;
  }
}

// Restores an exact saved adjustment sequence on top of the freshly imported base mesh.
boolean applyWorldProfileTransformForRobotMode(int robotMode, JSONObject profile) {
  if (profile == null) return false;
  JSONObject transform = getJsonObjectSafe(profile, "transform");
  JSONArray operations = transform == null ? null : getJsonArraySafe(transform, "operations");
  if (operations == null) return false;

  restoreImportedWorldForRobotMode(robotMode);
  clearWorldTransformHistoryForRobotMode(robotMode);
  ArrayList<WorldTransformOperation> history = worldTransformHistoryForRobotMode(robotMode);
  worldTransformHistoryReplayActive = true;
  try {
    for (int i = 0; i < operations.size(); i++) {
      JSONObject op = null;
      try { op = operations.getJSONObject(i); } catch (Exception e) { return false; }
      if (op == null) continue;
      String type = trim(getJsonString(op, "type", "")).toLowerCase();
      if (type.equals("translate")) {
        float dx = getJsonFloat(op, "x", 0.0f);
        float dz = getJsonFloat(op, "z", 0.0f);
        translateImportedWorldForRobotMode(robotMode, dx, dz);
        history.add(new WorldTransformOperation("translate", 'Y', dx, dz));
      } else if (type.equals("scale")) {
        float factor = getJsonFloat(op, "factor", 1.0f);
        if (factor > 0.0001f) {
          scaleImportedWorldForRobotMode(robotMode, factor);
          history.add(new WorldTransformOperation("scale", 'Y', factor, 0.0f));
        }
      } else if (type.equals("rotate")) {
        String axisText = trim(getJsonString(op, "axis", "Y")).toUpperCase();
        char axis = axisText.length() > 0 ? axisText.charAt(0) : 'Y';
        if (axis != 'X' && axis != 'Y' && axis != 'Z') axis = 'Y';
        float deg = getJsonFloat(op, "degrees", 0.0f);
        rotateImportedWorldForRobotMode(robotMode, axis, deg);
        history.add(new WorldTransformOperation("rotate", axis, deg, 0.0f));
      }
    }
  }
  finally {
    worldTransformHistoryReplayActive = false;
  }

  JSONObject compass = getJsonObjectSafe(profile, "compass");
  if (compass != null && getJsonBoolean(compass, "locked", false)) {
    setWorldCompassNorthReferenceForRobotMode(robotMode, getJsonFloat(compass, "north_offset_deg", 0.0f));
  } else {
    clearWorldCompassNorthReferenceForRobotMode(robotMode);
  }
  return true;
}

// Saves the current locked world setup. The readable aggregate parameters are
// stored together with the exact operation history used for lossless replay.
boolean saveWorldProfileForRobotMode(int robotMode, boolean floorIgnored) {
  File sourceFile = worldSourceFileForRobotMode(robotMode);
  File dir = worldProfileDirectoryForSource(sourceFile, robotMode);
  File profileFile = worldProfileFileForSource(sourceFile, robotMode);
  if (sourceFile == null || dir == null || profileFile == null) return false;
  try {
    if (!dir.exists() && !dir.mkdirs()) return false;

    JSONObject profile = new JSONObject();
    profile.setString("schema", WORLD_PROFILE_SCHEMA);
    profile.setInt("softwareVersion", SYNROV_SOFTWARE_VERSION);
    profile.setString("robot", worldProfileRobotSlug(robotMode));

    JSONObject source = new JSONObject();
    source.setString("name", sourceFile.getName());
    source.setString("path", canonicalWorldSourcePath(sourceFile));
    source.setString("extension", fileExtensionLower(sourceFile));
    source.setString("size_bytes", worldSourceLengthText(sourceFile));
    source.setString("last_modified_ms", worldSourceModifiedText(sourceFile));
    profile.setJSONObject("source", source);

    JSONObject importInfo = new JSONObject();
    float unitsToCm = robotMode == ROBOT_MODE_VEHICLE ? vehicleImportedWorldUnitsToCm : (robotMode == ROBOT_MODE_DRONE ? droneImportedWorldUnitsToCm : manipulatorImportedWorldUnitsToCm);
    float sceneScale = robotMode == ROBOT_MODE_VEHICLE ? vehicleImportedWorldScale : (robotMode == ROBOT_MODE_DRONE ? droneImportedWorldScale : manipulatorImportedWorldScale);
    importInfo.setFloat("units_to_cm", unitsToCm);
    importInfo.setFloat("scene_scale", sceneScale);
    profile.setJSONObject("import", importInfo);

    JSONObject transform = new JSONObject();
    transform.setFloat("scale", worldUserScaleForRobotMode(robotMode));
    transform.setFloat("pitch_deg", worldUserPitchDegForRobotMode(robotMode));
    transform.setFloat("yaw_deg", worldUserRotationDegForRobotMode(robotMode));
    transform.setFloat("roll_deg", worldUserRollDegForRobotMode(robotMode));
    PVector offset = worldUserOffsetForRobotMode(robotMode);
    transform.setFloat("offset_x", offset.x);
    transform.setFloat("offset_z", offset.z);
    JSONArray operations = new JSONArray();
    ArrayList<WorldTransformOperation> history = worldTransformHistoryForRobotMode(robotMode);
    for (int i = 0; i < history.size(); i++) {
      WorldTransformOperation item = history.get(i);
      JSONObject op = new JSONObject();
      op.setString("type", item.type);
      if (item.type.equals("translate")) {
        op.setFloat("x", item.a);
        op.setFloat("z", item.b);
      } else if (item.type.equals("scale")) {
        op.setFloat("factor", item.a);
      } else if (item.type.equals("rotate")) {
        op.setString("axis", String.valueOf(item.axis));
        op.setFloat("degrees", item.a);
      }
      operations.append(op);
    }
    transform.setJSONArray("operations", operations);
    profile.setJSONObject("transform", transform);

    JSONObject compass = new JSONObject();
    compass.setBoolean("locked", worldCompassNorthLockedForRobotMode(robotMode));
    compass.setFloat("north_offset_deg", worldCompassNorthOffsetDegForRobotMode(robotMode));
    profile.setJSONObject("compass", compass);

    JSONObject collision = new JSONObject();
    collision.setBoolean("imported_floor_excluded", floorIgnored);
    profile.setJSONObject("collision", collision);
    profile.setBoolean("locked", true);

    saveJSONObject(profile, profileFile.getAbsolutePath());
    return profileFile.exists();
  }
  catch (Exception e) {
    println("World profile save failed: " + e.getMessage());
    return false;
  }
}

// Utility: opens the transform controls for a world that is already loaded.
void adjustCurrentImportedWorldTransform() {
  int robotMode = currentRobotMode();
  if (!hasImportedWorldForRobotMode(robotMode)) {
    updateMessage(tr("No imported world to adjust."));
    return;
  }

  // A locked world has already received its final vertical ground snap. Reopen
  // the saved pre-lock transform from the immutable base before further edits
  // so a later Lock replays exactly the same operation sequence.
  File sourceFile = worldSourceFileForRobotMode(robotMode);
  JSONObject savedProfile = loadWorldProfileForSource(sourceFile, robotMode);
  if (savedProfile != null) applyWorldProfileTransformForRobotMode(robotMode, savedProfile);

  activateWorldTransformOverlay(robotMode);
  updateMessage(tr("Adjust the world and press Lock to update its saved profile."));
}

// Resets world transform for robot mode.
void resetWorldTransformForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) {
    vehicleWorldUserScale = 1.0f;
    vehicleWorldUserOffset.set(0, 0, 0);
    vehicleWorldUserRotationDeg = 0.0f;
    vehicleWorldUserPitchDeg = 0.0f;
    vehicleWorldUserRollDeg = 0.0f;
  } else if (robotMode == ROBOT_MODE_DRONE) {
    droneWorldUserScale = 1.0f;
    droneWorldUserOffset.set(0, 0, 0);
    droneWorldUserRotationDeg = 0.0f;
    droneWorldUserPitchDeg = 0.0f;
    droneWorldUserRollDeg = 0.0f;
  } else {
    manipulatorWorldUserScale = 1.0f;
    manipulatorWorldUserOffset.set(0, 0, 0);
    manipulatorWorldUserRotationDeg = 0.0f;
    manipulatorWorldUserPitchDeg = 0.0f;
    manipulatorWorldUserRollDeg = 0.0f;
  }
}

// Utility: multiply world user scale for robot mode.
void multiplyWorldUserScaleForRobotMode(int robotMode, float factor) {
  if (robotMode == ROBOT_MODE_VEHICLE) vehicleWorldUserScale *= factor;
  else if (robotMode == ROBOT_MODE_DRONE) droneWorldUserScale *= factor;
  else manipulatorWorldUserScale *= factor;
}

// Utility: add world user offset for robot mode.
void addWorldUserOffsetForRobotMode(int robotMode, float dx, float dz) {
  PVector off = worldUserOffsetForRobotMode(robotMode);
  off.x += dx;
  off.z += dz;
}

// Checks whether world transform overlay active.
boolean isWorldTransformOverlayActive() {
  return worldTransformOverlayActive
    && worldTransformOverlayRobotMode == currentRobotMode()
    && (!importedWorldPointListForRobotMode(worldTransformOverlayRobotMode).isEmpty()
      || !importedWorldMeshForRobotMode(worldTransformOverlayRobotMode).isEmpty());
}

// Utility: activate world transform overlay.
void activateWorldTransformOverlay(int robotMode) {
  worldTransformOverlayActive = true;
  worldTransformOverlayDragging = false;
  worldTransformOverlayPanelPointerDown = false;
  worldTransformMouseAdjustEnabled = false;
  worldTransformOverlayRobotMode = robotMode;
  setWorldTransformReferenceYawForRobotMode(robotMode, cameraRotationY);
}

// Utility: dismiss world transform overlay.
void dismissWorldTransformOverlay() {
  worldTransformOverlayActive = false;
  worldTransformOverlayDragging = false;
  worldTransformOverlayPanelPointerDown = false;
  worldTransformMouseAdjustEnabled = false;
}

// Utility: imported world centroid for robot mode.
PVector importedWorldCentroidForRobotMode(int robotMode) {
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  if (pts != null && !pts.isEmpty()) {
    PVector c = new PVector();
    for (int i = 0; i < pts.size(); i++) c.add(pts.get(i));
    c.div(max(1, pts.size()));
    return c;
  }
  MeshData mesh = importedWorldMeshForRobotMode(robotMode);
  if (mesh != null && !mesh.isEmpty()) {
    mesh.recalculateBounds();
    if (mesh.minBound != null && mesh.maxBound != null) return PVector.mult(PVector.add(mesh.minBound, mesh.maxBound), 0.5f);
  }
  return robotWorldAnchor(robotMode).copy();
}

// Utility: apply a translation to a mesh in world coordinates.
void translateMeshData(MeshData mesh, float dx, float dy, float dz) {
  if (mesh == null || mesh.isEmpty()) return;
  for (int i = 0; i < mesh.loosePoints.size(); i++) mesh.loosePoints.get(i).add(dx, dy, dz);
  for (int i = 0; i < mesh.triangles.size(); i++) {
    MeshTriangle tri = mesh.triangles.get(i);
    tri.a.add(dx, dy, dz);
    tri.b.add(dx, dy, dz);
    tri.c.add(dx, dy, dz);
  }
  mesh.recalculateBounds();
}

// Utility: translate imported world for robot mode.
void translateImportedWorldForRobotMode(int robotMode, float dx, float dz) {
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  MeshData mesh = importedWorldMeshForRobotMode(robotMode);
  if ((pts == null || pts.isEmpty()) && (mesh == null || mesh.isEmpty())) return;
  if (pts != null) {
    for (int i = 0; i < pts.size(); i++) {
      PVector p = pts.get(i);
      p.x += dx;
      p.z += dz;
    }
  }
  translateMeshData(mesh, dx, 0, dz);
  addWorldUserOffsetForRobotMode(robotMode, dx, dz);
  recordWorldTransformOperation(robotMode, "translate", 'Y', dx, dz);
  invalidateEnvCollisionCache();
}

// Utility: scale imported world collision density when the world grows.
void densifyImportedWorldForRobotMode(int robotMode, float factor) {
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  if (pts == null || pts.size() < 2 || factor <= 1.01f) return;
  int maxPoints = 18000;
  if (pts.size() >= maxPoints) return;
  int originalSize = pts.size();
  int addBudget = min(maxPoints - originalSize, max(64, int(originalSize * min(0.65f, (factor - 1.0f) * 0.55f))));
  float maxJoinDist = max(6.0f, robotReferenceSpanScene(robotMode) * 0.055f);
  ArrayList<PVector> extra = new ArrayList<PVector>();
  for (int i = 0; i < originalSize - 1 && extra.size() < addBudget; i += 2) {
    PVector a = pts.get(i);
    PVector b = pts.get(i + 1);
    if (PVector.dist(a, b) > maxJoinDist) continue;
    extra.add(PVector.lerp(a, b, 0.5f));
  }
  pts.addAll(extra);
}

// Utility: rotate one point around a pivot and one scene axis.
void rotatePointAroundWorldAxis(PVector p, PVector pivot, char axis, float rad) {
  if (p == null || pivot == null) return;
  float c = cos(rad);
  float s = sin(rad);
  float dx = p.x - pivot.x;
  float dy = p.y - pivot.y;
  float dz = p.z - pivot.z;
  if (axis == 'X') {
    p.y = pivot.y + dy * c - dz * s;
    p.z = pivot.z + dy * s + dz * c;
  } else if (axis == 'Z') {
    p.x = pivot.x + dx * c - dy * s;
    p.y = pivot.y + dx * s + dy * c;
  } else {
    p.x = pivot.x + dx * c + dz * s;
    p.z = pivot.z - dx * s + dz * c;
  }
}

// Utility: rotate direct mesh geometry around a pivot.
void rotateMeshDataAroundWorldAxis(MeshData mesh, PVector pivot, char axis, float rad) {
  if (mesh == null || mesh.isEmpty()) return;
  for (int i = 0; i < mesh.loosePoints.size(); i++) rotatePointAroundWorldAxis(mesh.loosePoints.get(i), pivot, axis, rad);
  for (int i = 0; i < mesh.triangles.size(); i++) {
    MeshTriangle tri = mesh.triangles.get(i);
    rotatePointAroundWorldAxis(tri.a, pivot, axis, rad);
    rotatePointAroundWorldAxis(tri.b, pivot, axis, rad);
    rotatePointAroundWorldAxis(tri.c, pivot, axis, rad);
  }
  mesh.recalculateBounds();
}

// Rotates the imported world around X (pitch), Y (yaw), or Z (roll).
void rotateImportedWorldForRobotMode(int robotMode, char axis, float deltaDeg) {
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  MeshData mesh = importedWorldMeshForRobotMode(robotMode);
  if ((pts == null || pts.isEmpty()) && (mesh == null || mesh.isEmpty())) return;
  PVector pivot = importedWorldCentroidForRobotMode(robotMode);
  float rad = radians(deltaDeg);
  if (pts != null) {
    for (int i = 0; i < pts.size(); i++) rotatePointAroundWorldAxis(pts.get(i), pivot, axis, rad);
  }
  rotateMeshDataAroundWorldAxis(mesh, pivot, axis, rad);
  if (axis == 'X') addWorldUserPitchDegForRobotMode(robotMode, deltaDeg);
  else if (axis == 'Z') addWorldUserRollDegForRobotMode(robotMode, deltaDeg);
  else addWorldUserRotationDegForRobotMode(robotMode, deltaDeg);
  recordWorldTransformOperation(robotMode, "rotate", axis, deltaDeg, 0.0f);
  invalidateEnvCollisionCache();
}

// Utility: scale direct mesh geometry using the same pivot convention as collision samples.
void scaleMeshDataForImportedWorld(MeshData mesh, PVector pivot, float factor) {
  if (mesh == null || mesh.isEmpty() || pivot == null) return;
  for (int i = 0; i < mesh.loosePoints.size(); i++) {
    PVector p = mesh.loosePoints.get(i);
    p.x = pivot.x + (p.x - pivot.x) * factor;
    p.y = GROUND_Y + (p.y - GROUND_Y) * factor;
    p.z = pivot.z + (p.z - pivot.z) * factor;
  }
  for (int i = 0; i < mesh.triangles.size(); i++) {
    MeshTriangle tri = mesh.triangles.get(i);
    PVector[] vertices = { tri.a, tri.b, tri.c };
    for (int k = 0; k < vertices.length; k++) {
      PVector p = vertices[k];
      p.x = pivot.x + (p.x - pivot.x) * factor;
      p.y = GROUND_Y + (p.y - GROUND_Y) * factor;
      p.z = pivot.z + (p.z - pivot.z) * factor;
    }
  }
  mesh.recalculateBounds();
}

void scaleImportedWorldForRobotMode(int robotMode, float factor) {
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  MeshData mesh = importedWorldMeshForRobotMode(robotMode);
  if ((pts == null || pts.isEmpty()) && (mesh == null || mesh.isEmpty())) return;
  float safeFactor = constrain(factor, 0.2f, 6.0f);
  PVector pivot = importedWorldCentroidForRobotMode(robotMode);
  if (pts != null) {
    for (int i = 0; i < pts.size(); i++) {
      PVector p = pts.get(i);
      p.x = pivot.x + (p.x - pivot.x) * safeFactor;
      p.y = GROUND_Y + (p.y - GROUND_Y) * safeFactor;
      p.z = pivot.z + (p.z - pivot.z) * safeFactor;
    }
  }
  scaleMeshDataForImportedWorld(mesh, pivot, safeFactor);
  multiplyWorldUserScaleForRobotMode(robotMode, safeFactor);
  recordWorldTransformOperation(robotMode, "scale", 'Y', safeFactor, 0.0f);
  densifyImportedWorldForRobotMode(robotMode, safeFactor);
  invalidateEnvCollisionCache();
}

// Restores the imported world to its initial aligned transform.
void restoreImportedWorldForRobotMode(int robotMode) {
  copyMeshDataInto(importedWorldMeshForRobotMode(robotMode), importedWorldBaseMeshForRobotMode(robotMode));
  copyPointListInto(importedWorldPointListForRobotMode(robotMode), importedWorldBasePointListForRobotMode(robotMode));
  resetWorldTransformForRobotMode(robotMode);
  clearWorldTransformHistoryForRobotMode(robotMode);
  invalidateEnvCollisionCache();
}

// Utility: recenter imported world for robot mode.
void recenterImportedWorldForRobotMode(int robotMode) {
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  MeshData mesh = importedWorldMeshForRobotMode(robotMode);
  if ((pts == null || pts.isEmpty()) && (mesh == null || mesh.isEmpty())) return;
  PVector center = importedWorldCentroidForRobotMode(robotMode);
  PVector anchor = robotWorldAnchor(robotMode);
  translateImportedWorldForRobotMode(robotMode, anchor.x - center.x, anchor.z - center.z);
}

// Utility: world transform drag delta.
PVector worldTransformDragDelta(float dx, float dy) {
  float gain = 0.85f / max(0.55f, zoomLevel);
  float referenceYaw = worldTransformReferenceYawForRobotMode(currentRobotMode());
  PVector right = new PVector(cos(referenceYaw), 0, -sin(referenceYaw));
  PVector forward = new PVector(sin(referenceYaw), 0, cos(referenceYaw));
  PVector delta = PVector.mult(right, dx * gain);
  delta.add(PVector.mult(forward, dy * gain));
  return delta;
}

// Resets demo world for robot mode.
void resetDemoWorldForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) {
    vehicleDemoWorldPoints.clear();
    vehicleDemoWorldReady = false;
  } else if (robotMode == ROBOT_MODE_DRONE) {
    droneDemoWorldPoints.clear();
    droneDemoWorldReady = false;
  } else {
    manipulatorDemoWorldPoints.clear();
    manipulatorDemoWorldReady = false;
  }
}


// Checks whether imported world for robot mode.
boolean hasImportedWorldForRobotMode(int robotMode) {
  ArrayList<PVector> points = importedWorldPointListForRobotMode(robotMode);
  MeshData mesh = importedWorldMeshForRobotMode(robotMode);
  return (points != null && !points.isEmpty()) || (mesh != null && !mesh.isEmpty());
}

// Compass helper for heading degrees for robot mode.
float compassHeadingDegForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) {
    return (isFirmwareTelemetrySelected() && systemReady)
      ? vehicleHeadingTelemetryDeg
      : degrees(vehicleNavYaw);
  }
  if (robotMode == ROBOT_MODE_DRONE) {
    return (isFirmwareTelemetrySelected() && systemReady)
      ? droneHeadingTelemetryDeg
      : degrees(droneNavYaw);
  }
  return systemReady ? mpu1YawDeg : 0.0f;
}

// Utility: world compass north offset degrees for robot mode.
float worldCompassNorthOffsetDegForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleWorldCompassNorthOffsetDeg;
  if (robotMode == ROBOT_MODE_DRONE) return droneWorldCompassNorthOffsetDeg;
  return manipulatorWorldCompassNorthOffsetDeg;
}

// Utility: world compass north locked for robot mode.
boolean worldCompassNorthLockedForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleWorldCompassNorthLocked;
  if (robotMode == ROBOT_MODE_DRONE) return droneWorldCompassNorthLocked;
  return manipulatorWorldCompassNorthLocked;
}

// Sets world compass north reference for robot mode.
void setWorldCompassNorthReferenceForRobotMode(int robotMode, float headingDeg) {
  float normalized = normalizeAbsoluteAngleDeg(headingDeg);
  if (robotMode == ROBOT_MODE_VEHICLE) {
    vehicleWorldCompassNorthOffsetDeg = normalized;
    vehicleWorldCompassNorthLocked = true;
  } else if (robotMode == ROBOT_MODE_DRONE) {
    droneWorldCompassNorthOffsetDeg = normalized;
    droneWorldCompassNorthLocked = true;
  } else {
    manipulatorWorldCompassNorthOffsetDeg = normalized;
    manipulatorWorldCompassNorthLocked = true;
  }
}

// Clears world compass north reference for robot mode.
void clearWorldCompassNorthReferenceForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) {
    vehicleWorldCompassNorthOffsetDeg = 0.0f;
    vehicleWorldCompassNorthLocked = false;
  } else if (robotMode == ROBOT_MODE_DRONE) {
    droneWorldCompassNorthOffsetDeg = 0.0f;
    droneWorldCompassNorthLocked = false;
  } else {
    manipulatorWorldCompassNorthOffsetDeg = 0.0f;
    manipulatorWorldCompassNorthLocked = false;
  }
}

// Utility: current world aligned heading degrees.
float currentWorldAlignedHeadingDeg(float headingDeg) {
  int robotMode = currentRobotMode();
  if (!worldCompassNorthLockedForRobotMode(robotMode)) return headingDeg;
  return normalizeAbsoluteAngleDeg(headingDeg - worldCompassNorthOffsetDegForRobotMode(robotMode));
}


// Utility: should use demo world for robot mode.
boolean shouldUseDemoWorldForRobotMode(int robotMode) {
  if (!demoWorldEnabledForRobotMode(robotMode)) return false;
  if (hasImportedWorldForRobotMode(robotMode)) return false;
  if (simulationMode) return true;
  return bootDemoWorldEnabled && !systemReady;
}

// Utility: should use demo world for current robot.
boolean shouldUseDemoWorldForCurrentRobot() {
  return shouldUseDemoWorldForRobotMode(currentRobotMode());
}

// Utility: disable boot demo worlds.
void disableBootDemoWorlds() {
  bootDemoWorldEnabled = false;
  manipulatorDemoWorldPoints.clear();
  vehicleDemoWorldPoints.clear();
  droneDemoWorldPoints.clear();
  manipulatorDemoWorldReady = false;
  vehicleDemoWorldReady = false;
  droneDemoWorldReady = false;
}

// Clears environment map.
void clearEnvironmentMap() {
  currentEnvironmentPointList().clear();
  if (isVehicleSelected) vehicleRadarProbeWorld = null;
  if (isDroneSelected) droneDownwardSonarProbeWorld = null;
}

// Clears demo world for robot mode and disables automatic regeneration.
void clearDemoWorldForRobotMode(int robotMode) {
  resetDemoWorldForRobotMode(robotMode);
  setDemoWorldEnabledForRobotMode(robotMode, false);
}

// Clears current demo world and disables automatic regeneration.
void clearCurrentDemoWorldAndDisable() {
  clearDemoWorldForRobotMode(currentRobotMode());
}

// Clears imported world for robot mode.
void clearImportedWorldForRobotMode(int robotMode) {
  setCollisionWorldSolidForRobotMode(robotMode, false);
  importedWorldPointListForRobotMode(robotMode).clear();
  importedWorldBasePointListForRobotMode(robotMode).clear();
  importedWorldMeshForRobotMode(robotMode).clear();
  importedWorldBaseMeshForRobotMode(robotMode).clear();
  setWorldSourceLabelForRobotMode(robotMode, "");
  setWorldSourceFileForRobotMode(robotMode, null);
  clearWorldTransformHistoryForRobotMode(robotMode);
  setImportedWorldUnitsToCmForRobotMode(robotMode, 1.0f);
  setImportedWorldScaleFactorForRobotMode(robotMode, 1.0f);
  clearWorldCompassNorthReferenceForRobotMode(robotMode);
  resetWorldTransformForRobotMode(robotMode);
  if (worldTransformOverlayRobotMode == robotMode) dismissWorldTransformOverlay();
}

// Clears current imported world.
void clearCurrentImportedWorld() {
  clearImportedWorldForRobotMode(currentRobotMode());
}

// Clears all environment maps.
void clearAllEnvironmentMaps() {
  manipulatorEnvironmentPoints.clear();
  vehicleEnvironmentPoints.clear();
  droneEnvironmentPoints.clear();
  vehicleRadarProbeWorld = null;
  droneDownwardSonarProbeWorld = null;
  manipulatorImportedWorldPoints.clear();
  vehicleImportedWorldPoints.clear();
  droneImportedWorldPoints.clear();
  manipulatorImportedWorldBasePoints.clear();
  vehicleImportedWorldBasePoints.clear();
  droneImportedWorldBasePoints.clear();
  manipulatorImportedWorldMesh.clear();
  vehicleImportedWorldMesh.clear();
  droneImportedWorldMesh.clear();
  manipulatorImportedWorldBaseMesh.clear();
  vehicleImportedWorldBaseMesh.clear();
  droneImportedWorldBaseMesh.clear();
  manipulatorDemoWorldPoints.clear();
  vehicleDemoWorldPoints.clear();
  droneDemoWorldPoints.clear();
  manipulatorDemoWorldReady = false;
  vehicleDemoWorldReady = false;
  droneDemoWorldReady = false;
  manipulatorDemoWorldEnabled = true;
  vehicleDemoWorldEnabled = true;
  droneDemoWorldEnabled = true;
  manipulatorWorldSourceLabel = "";
  vehicleWorldSourceLabel = "";
  droneWorldSourceLabel = "";
  manipulatorWorldSourceFile = null;
  vehicleWorldSourceFile = null;
  droneWorldSourceFile = null;
  manipulatorWorldTransformHistory.clear();
  vehicleWorldTransformHistory.clear();
  droneWorldTransformHistory.clear();
  manipulatorImportedWorldScale = 1.0f;
  vehicleImportedWorldScale = 1.0f;
  droneImportedWorldScale = 1.0f;
  manipulatorImportedWorldUnitsToCm = 1.0f;
  vehicleImportedWorldUnitsToCm = 1.0f;
  droneImportedWorldUnitsToCm = 1.0f;
}

// Utility: world source label for robot mode.
String worldSourceLabelForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleWorldSourceLabel;
  if (robotMode == ROBOT_MODE_DRONE) return droneWorldSourceLabel;
  return manipulatorWorldSourceLabel;
}

// Utility: current world source label.
String currentWorldSourceLabel() {
  return worldSourceLabelForRobotMode(currentRobotMode());
}

// Sets world source label for robot mode.
void setWorldSourceLabelForRobotMode(int robotMode, String label) {
  String safe = label == null ? "" : trim(label);
  if (robotMode == ROBOT_MODE_VEHICLE) {
    vehicleWorldSourceLabel = safe;
  } else if (robotMode == ROBOT_MODE_DRONE) {
    droneWorldSourceLabel = safe;
  } else {
    manipulatorWorldSourceLabel = safe;
  }
}




// Sets imported world units to cm for robot mode.
void setImportedWorldUnitsToCmForRobotMode(int robotMode, float value) {
  float safe = max(0.0001f, value);
  if (robotMode == ROBOT_MODE_VEHICLE) {
    vehicleImportedWorldUnitsToCm = safe;
  } else if (robotMode == ROBOT_MODE_DRONE) {
    droneImportedWorldUnitsToCm = safe;
  } else {
    manipulatorImportedWorldUnitsToCm = safe;
  }
}




// Sets imported world scale factor for robot mode.
void setImportedWorldScaleFactorForRobotMode(int robotMode, float value) {
  float safe = max(0.0001f, value);
  if (robotMode == ROBOT_MODE_VEHICLE) {
    vehicleImportedWorldScale = safe;
  } else if (robotMode == ROBOT_MODE_DRONE) {
    droneImportedWorldScale = safe;
  } else {
    manipulatorImportedWorldScale = safe;
  }
}


// Utility: robot scene units per cm.
float robotSceneUnitsPerCm(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) return MODEL_SCALE_3D;
  return 1.0f;
}


// Utility: robot reference span scene.
float robotReferenceSpanScene(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) {
    return max(baseCylinderRadius * 2.0f,
      baseCylinderHeight + upperArmH + forearmH + wristVerticalH + gripperFingerH);
  }
  if (robotMode == ROBOT_MODE_VEHICLE) {
    return max(max(VEH_BODY_LENGTH, VEH_BODY_WIDTH), VEH_TRACK_GAUGE);
  }
  return max(max(DRONE_BODY_L + DRONE_PROP_RADIUS * 2.0f, DRONE_BODY_W + DRONE_PROP_RADIUS * 2.0f), DRONE_ARM_LENGTH * 2.0f);
}


// Utility: robot world anchor.
PVector robotWorldAnchor(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return new PVector(vehicleNavX, GROUND_Y, vehicleNavZ);
  if (robotMode == ROBOT_MODE_DRONE) return new PVector(droneNavX, GROUND_Y, droneNavZ);
  return new PVector(0, GROUND_Y, 0);
}


// Utility: shorten world label.
String shortenWorldLabel(String value, int maxChars) {
  String safe = value == null ? "" : trim(value);
  if (safe.length() <= maxChars) return safe;
  if (maxChars < 4) return safe.substring(0, maxChars);
  return safe.substring(0, maxChars - 3) + "...";
}

// Utility: environment point count for robot mode.
int environmentPointCountForRobotMode(int robotMode) {
  return environmentPointListForRobotMode(robotMode).size();
}

// Utility: imported world geometry count for robot mode.
int importedWorldGeometryCountForRobotMode(int robotMode) {
  MeshData mesh = importedWorldMeshForRobotMode(robotMode);
  if (mesh == null) return 0;
  return !mesh.triangles.isEmpty() ? mesh.triangles.size() : mesh.loosePoints.size();
}



// Utility: current environment world summary.
String currentEnvironmentWorldSummary() {
  int robotMode = currentRobotMode();
  String label = worldSourceLabelForRobotMode(robotMode);
  int scanCount = environmentPointCountForRobotMode(robotMode);
  int worldCount = importedWorldGeometryCountForRobotMode(robotMode);

  if (label != null && label.length() > 0) {
    String northSuffix = worldCompassNorthLockedForRobotMode(robotMode)
      ? " N:" + nfc(worldCompassNorthOffsetDegForRobotMode(robotMode), 1) + "°"
      : "";
    if (scanCount > 0) {
      return shortenWorldLabel(label, 14) + " W:" + worldCount + " S:" + scanCount + northSuffix;
    }
    return shortenWorldLabel(label, 19) + " (" + worldCount + ")" + northSuffix;
  }
  if (shouldUseDemoWorldForCurrentRobot()) return "demo world";
  if (scanCount > 0) return "scan map (" + scanCount + ")";
  return "none";
}

class WorldTransformOperation {
  String type;
  char axis;
  float a;
  float b;
  WorldTransformOperation(String type, char axis, float a, float b) {
    this.type = type == null ? "" : type;
    this.axis = axis;
    this.a = a;
    this.b = b;
  }
}

class ImportMeshResult {
  int robotMode = ROBOT_MODE_MANIPULATOR;
  File sourceFile = null;
  String sourceLabel = "";
  String ext = "";
  MeshData mesh = null;
  String errorMessage = "";
}

class MeshTriangle {
  PVector a;
  PVector b;
  PVector c;
  MeshTriangle(PVector a, PVector b, PVector c) {
    this.a = a;
    this.b = b;
    this.c = c;
  }
}

class MeshData {
  ArrayList<MeshTriangle> triangles = new ArrayList<MeshTriangle>();
  ArrayList<PVector> loosePoints = new ArrayList<PVector>();
  PVector minBound = null;
  PVector maxBound = null;

// Utility: include point.
  void includePoint(PVector p) {
    if (p == null) return;
    if (minBound == null) {
      minBound = p.copy();
      maxBound = p.copy();
      return;
    }
    minBound.x = min(minBound.x, p.x);
    minBound.y = min(minBound.y, p.y);
    minBound.z = min(minBound.z, p.z);
    maxBound.x = max(maxBound.x, p.x);
    maxBound.y = max(maxBound.y, p.y);
    maxBound.z = max(maxBound.z, p.z);
  }

// Utility: add loose point.
  void addLoosePoint(PVector p) {
    if (p == null) return;
    loosePoints.add(p.copy());
    includePoint(p);
  }

// Utility: add triangle.
  void addTriangle(PVector a, PVector b, PVector c) {
    if (a == null || b == null || c == null) return;
    triangles.add(new MeshTriangle(a.copy(), b.copy(), c.copy()));
    includePoint(a);
    includePoint(b);
    includePoint(c);
  }

// Checks whether empty.
  boolean isEmpty() {
    return triangles.isEmpty() && loosePoints.isEmpty();
  }

  void clear() {
    triangles.clear();
    loosePoints.clear();
    minBound = null;
    maxBound = null;
  }

  void recalculateBounds() {
    minBound = null;
    maxBound = null;
    for (int i = 0; i < loosePoints.size(); i++) includePoint(loosePoints.get(i));
    for (int i = 0; i < triangles.size(); i++) {
      MeshTriangle tri = triangles.get(i);
      includePoint(tri.a);
      includePoint(tri.b);
      includePoint(tri.c);
    }
  }
}

class ImportedGroundLevel {
  float y = 0.0f;
  float projectedArea = 0.0f;
  int triangleCount = 0;
}

class ImportedGroundProfile {
  boolean detected = false;
  float alignmentY = 0.0f;
  float levelTolerance = 1.0f;
  ArrayList<Float> levels = new ArrayList<Float>();
}

// Utility: deep-copy mesh geometry without sharing mutable vertices.
void copyMeshDataInto(MeshData target, MeshData source) {
  if (target == null) return;
  target.clear();
  if (source == null) return;
  for (int i = 0; i < source.loosePoints.size(); i++) target.addLoosePoint(source.loosePoints.get(i));
  for (int i = 0; i < source.triangles.size(); i++) {
    MeshTriangle tri = source.triangles.get(i);
    target.addTriangle(tri.a, tri.b, tri.c);
  }
}

// Utility: deep-copy point samples.
void copyPointListInto(ArrayList<PVector> target, ArrayList<PVector> source) {
  if (target == null) return;
  target.clear();
  if (source == null) return;
  for (int i = 0; i < source.size(); i++) target.add(source.get(i).copy());
}

// Utility: remap imported source to scene.
PVector remapImportedSourceToScene(PVector p) {
  if (p == null) return null;
  return new PVector(p.x, -p.z, p.y);
}

// Utility: rotate source point xyz.
PVector rotateSourcePointXYZ(PVector p, PVector rpy) {
  if (p == null) return null;
  PVector out = p.copy();
  float cr = cos(rpy.x), sr = sin(rpy.x);
  float cp = cos(rpy.y), sp = sin(rpy.y);
  float cy = cos(rpy.z), sy = sin(rpy.z);

  float y1 = out.y * cr - out.z * sr;
  float z1 = out.y * sr + out.z * cr;
  out.y = y1;
  out.z = z1;

  float x2 = out.x * cp + out.z * sp;
  float z2 = -out.x * sp + out.z * cp;
  out.x = x2;
  out.z = z2;

  float x3 = out.x * cy - out.y * sy;
  float y3 = out.x * sy + out.y * cy;
  out.x = x3;
  out.y = y3;
  return out;
}

// Utility: transform imported source point.
PVector transformImportedSourcePoint(PVector sourcePoint, PVector translation, PVector rpy, PVector localScale) {
  if (sourcePoint == null) return null;
  PVector p = sourcePoint.copy();
  p.x *= localScale.x;
  p.y *= localScale.y;
  p.z *= localScale.z;
  p = rotateSourcePointXYZ(p, rpy);
  p.add(translation);
  return p;
}

// Utility: accumulate mesh data.
void accumulateMeshData(MeshData target, MeshData source, PVector translation, PVector rpy, PVector localScale) {
  if (target == null || source == null || source.isEmpty()) return;
  for (int i = 0; i < source.loosePoints.size(); i++) {
    target.addLoosePoint(transformImportedSourcePoint(source.loosePoints.get(i), translation, rpy, localScale));
  }
  for (int i = 0; i < source.triangles.size(); i++) {
    MeshTriangle tri = source.triangles.get(i);
    target.addTriangle(
      transformImportedSourcePoint(tri.a, translation, rpy, localScale),
      transformImportedSourcePoint(tri.b, translation, rpy, localScale),
      transformImportedSourcePoint(tri.c, translation, rpy, localScale)
      );
  }
}

// Utility: sample triangle to environment.
void sampleTriangleToEnvironment(ArrayList<PVector> dst, PVector a, PVector b, PVector c, float spacing) {
  if (a == null || b == null || c == null) return;

  addEnvironmentPoint(dst, a);
  addEnvironmentPoint(dst, b);
  addEnvironmentPoint(dst, c);
  addEnvironmentPoint(dst, PVector.mult(PVector.add(PVector.add(a, b), c), 1.0f / 3.0f));

  float longest = max(PVector.dist(a, b), max(PVector.dist(b, c), PVector.dist(c, a)));
  int segments = constrain(ceil(longest / max(4.0f, spacing)), 1, 8);

  for (int i = 1; i < segments; i++) {
    float t = i / float(segments);
    addEnvironmentPoint(dst, PVector.lerp(a, b, t));
    addEnvironmentPoint(dst, PVector.lerp(b, c, t));
    addEnvironmentPoint(dst, PVector.lerp(c, a, t));
  }

  if (segments >= 3) {
    int rows = min(6, segments);
    for (int row = 1; row < rows; row++) {
      float u = row / float(rows);
      PVector edge1 = PVector.lerp(a, c, u);
      PVector edge2 = PVector.lerp(b, c, u);
      int cols = max(1, rows - row);
      for (int col = 0; col <= cols; col++) {
        float v = col / float(max(1, cols));
        addEnvironmentPoint(dst, PVector.lerp(edge1, edge2, v));
      }
    }
  }
}

// Utility: projected X/Z area of one imported triangle.
float importedTriangleProjectedAreaXZ(MeshTriangle tri) {
  if (tri == null || tri.a == null || tri.b == null || tri.c == null) return 0.0f;
  float abx = tri.b.x - tri.a.x;
  float abz = tri.b.z - tri.a.z;
  float acx = tri.c.x - tri.a.x;
  float acz = tri.c.z - tri.a.z;
  return abs(abx * acz - abz * acx) * 0.5f;
}

// Utility: absolute Y component of the normalized triangle normal.
float importedTriangleHorizontalRatio(MeshTriangle tri) {
  if (tri == null || tri.a == null || tri.b == null || tri.c == null) return 0.0f;
  PVector ab = PVector.sub(tri.b, tri.a);
  PVector ac = PVector.sub(tri.c, tri.a);
  PVector n = ab.cross(ac);
  float mag = n.mag();
  if (mag <= 0.000001f) return 0.0f;
  return abs(n.y) / mag;
}

// Utility: centroid Y of an imported triangle.
float importedTriangleCentroidY(MeshTriangle tri) {
  if (tri == null || tri.a == null || tri.b == null || tri.c == null) return 0.0f;
  return (tri.a.y + tri.b.y + tri.c.y) / 3.0f;
}

// Detects broad near-horizontal surfaces at the bottom of an imported mesh.
// These surfaces are treated as the imported world's own floor. The visual
// mesh remains untouched; only collision sampling can ignore these levels.
ImportedGroundProfile detectImportedGroundProfile(MeshData mesh, int robotMode) {
  ImportedGroundProfile profile = new ImportedGroundProfile();
  if (mesh == null || mesh.triangles.isEmpty()) return profile;
  mesh.recalculateBounds();
  if (mesh.minBound == null || mesh.maxBound == null) return profile;

  float scenePerCm = max(0.0001f, robotSceneUnitsPerCm(robotMode));
  float verticalSpan = max(0.0f, mesh.maxBound.y - mesh.minBound.y);
  float searchDepth = max(12.0f * scenePerCm, min(35.0f * scenePerCm, verticalSpan * 0.10f));
  profile.levelTolerance = max(1.5f * scenePerCm, environmentPointRadiusForRobotMode(robotMode) * 1.15f);

  float footprintW = max(scenePerCm, mesh.maxBound.x - mesh.minBound.x);
  float footprintD = max(scenePerCm, mesh.maxBound.z - mesh.minBound.z);
  float robotSpan = max(scenePerCm, robotReferenceSpanScene(robotMode));
  // Do not mistake a single imported crate/table for a world floor. A floor
  // must span at least slightly more than the robot in both horizontal axes.
  if (footprintW < robotSpan * 1.10f || footprintD < robotSpan * 1.10f) return profile;
  float footprintArea = footprintW * footprintD;
  ArrayList<ImportedGroundLevel> candidates = new ArrayList<ImportedGroundLevel>();

  for (int i = 0; i < mesh.triangles.size(); i++) {
    MeshTriangle tri = mesh.triangles.get(i);
    if (importedTriangleHorizontalRatio(tri) < 0.92f) continue;
    float y = importedTriangleCentroidY(tri);
    if (y < mesh.maxBound.y - searchDepth) continue;
    float area = importedTriangleProjectedAreaXZ(tri);
    if (area <= 0.0001f) continue;

    ImportedGroundLevel level = null;
    for (int j = 0; j < candidates.size(); j++) {
      if (abs(candidates.get(j).y - y) <= profile.levelTolerance) {
        level = candidates.get(j);
        break;
      }
    }
    if (level == null) {
      level = new ImportedGroundLevel();
      level.y = y;
      candidates.add(level);
    }
    float previousArea = level.projectedArea;
    level.projectedArea += area;
    level.y = (level.y * previousArea + y * area) / max(0.0001f, level.projectedArea);
    level.triangleCount++;
  }

  float largestArea = 0.0f;
  for (int i = 0; i < candidates.size(); i++) largestArea = max(largestArea, candidates.get(i).projectedArea);
  float minimumFloorArea = footprintArea * 0.06f;
  if (largestArea < minimumFloorArea) return profile;

  float strongArea = max(minimumFloorArea, largestArea * 0.45f);
  float alignmentY = Float.MAX_VALUE;
  for (int i = 0; i < candidates.size(); i++) {
    ImportedGroundLevel level = candidates.get(i);
    if (level.projectedArea < strongArea) continue;
    profile.levels.add(level.y);
    alignmentY = min(alignmentY, level.y);
  }

  if (profile.levels.isEmpty() || alignmentY == Float.MAX_VALUE) return profile;
  profile.detected = true;
  // With Processing's Y-down scene convention, the smallest strong floor level
  // is the top surface of a thick floor slab. Align that usable surface to Y=0.
  profile.alignmentY = alignmentY;
  return profile;
}

// Checks whether a triangle belongs to the detected imported floor.
boolean importedTriangleIsDetectedGround(MeshTriangle tri, ImportedGroundProfile profile) {
  if (tri == null || profile == null || !profile.detected) return false;
  if (importedTriangleHorizontalRatio(tri) < 0.92f) return false;
  float y = importedTriangleCentroidY(tri);
  for (int i = 0; i < profile.levels.size(); i++) {
    if (abs(y - profile.levels.get(i)) <= profile.levelTolerance * 1.35f) return true;
  }
  return false;
}

// Checks whether a loose imported point belongs to a detected floor level.
boolean importedPointIsDetectedGround(PVector p, ImportedGroundProfile profile) {
  if (p == null || profile == null || !profile.detected) return false;
  for (int i = 0; i < profile.levels.size(); i++) {
    if (abs(p.y - profile.levels.get(i)) <= profile.levelTolerance * 1.35f) return true;
  }
  return false;
}

// Rebuilds invisible imported-world collision samples from the direct mesh.
// When ignoreDetectedGround is true, broad floor surfaces remain visible but
// are not inserted into the collision point cloud.
boolean rebuildImportedWorldCollisionSamplesForRobotMode(int robotMode, boolean ignoreDetectedGround) {
  MeshData mesh = importedWorldMeshForRobotMode(robotMode);
  ArrayList<PVector> dst = importedWorldPointListForRobotMode(robotMode);
  if (dst == null) return false;
  dst.clear();
  if (mesh == null || mesh.isEmpty()) return false;

  ImportedGroundProfile ground = ignoreDetectedGround
    ? detectImportedGroundProfile(mesh, robotMode)
    : new ImportedGroundProfile();

  float sampleSpacing = max(environmentPointRadiusForRobotMode(robotMode) * 0.90f,
    max(2.4f, robotReferenceSpanScene(robotMode) * 0.018f));

  for (int i = 0; i < mesh.loosePoints.size(); i++) {
    PVector p = mesh.loosePoints.get(i);
    if (ignoreDetectedGround && importedPointIsDetectedGround(p, ground)) continue;
    addEnvironmentPoint(dst, p);
  }

  int eligibleTriangles = 0;
  for (int i = 0; i < mesh.triangles.size(); i++) {
    if (ignoreDetectedGround && importedTriangleIsDetectedGround(mesh.triangles.get(i), ground)) continue;
    eligibleTriangles++;
  }
  int triangleStride = max(1, ceil(eligibleTriangles / float(max(1, IMPORTED_WORLD_MAX_POINTS))));
  int eligibleIndex = 0;
  for (int i = 0; i < mesh.triangles.size(); i++) {
    MeshTriangle tri = mesh.triangles.get(i);
    if (ignoreDetectedGround && importedTriangleIsDetectedGround(tri, ground)) continue;
    if ((eligibleIndex++ % triangleStride) != 0) continue;
    sampleTriangleToEnvironment(dst, tri.a, tri.b, tri.c, sampleSpacing);
  }
  trimEnvironmentPointListUniformly(dst, IMPORTED_WORLD_MAX_POINTS);
  invalidateEnvCollisionCache();
  return ground.detected;
}

// Snaps the imported world to the SynROV ground plane after the operator has
// finished pitch/yaw/roll and horizontal placement. If a broad imported floor
// is detected, its top surface is aligned to GROUND_Y; otherwise the lowest
// mesh extent is placed on GROUND_Y.
float snapImportedWorldToGroundForRobotMode(int robotMode) {
  MeshData mesh = importedWorldMeshForRobotMode(robotMode);
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  if ((mesh == null || mesh.isEmpty()) && (pts == null || pts.isEmpty())) return 0.0f;

  float supportY = -Float.MAX_VALUE;
  if (mesh != null && !mesh.isEmpty()) {
    mesh.recalculateBounds();
    ImportedGroundProfile ground = detectImportedGroundProfile(mesh, robotMode);
    if (ground.detected) supportY = ground.alignmentY;
    else if (mesh.maxBound != null) supportY = mesh.maxBound.y;
  }
  if (supportY == -Float.MAX_VALUE && pts != null) {
    for (int i = 0; i < pts.size(); i++) supportY = max(supportY, pts.get(i).y);
  }
  if (supportY == -Float.MAX_VALUE) return 0.0f;

  float dy = GROUND_Y - supportY;
  if (abs(dy) <= 0.0001f) return 0.0f;
  if (pts != null) {
    for (int i = 0; i < pts.size(); i++) pts.get(i).y += dy;
  }
  translateMeshData(mesh, 0, dy, 0);
  invalidateEnvCollisionCache();
  return dy;
}

// Finalizes the imported-world placement. Lock always snaps the finished world
// onto the SynROV floor, then rebuilds collision samples while ignoring a
// detected floor that is already part of the imported geometry.
void lockImportedWorldTransformForRobotMode(int robotMode) {
  snapImportedWorldToGroundForRobotMode(robotMode);
  boolean floorIgnored = rebuildImportedWorldCollisionSamplesForRobotMode(robotMode, true);
  boolean profileSaved = saveWorldProfileForRobotMode(robotMode, floorIgnored);
  setWorldTransformReferenceYawForRobotMode(robotMode, cameraRotationY);
  dismissWorldTransformOverlay();
  if (floorIgnored) {
    updateMessage(tr(profileSaved
        ? "World locked; floor excluded from collision and profile saved."
        : "World locked to ground; imported floor excluded from collision."));
  } else {
    updateMessage(tr(profileSaved
        ? "World locked and transform profile saved."
        : "World locked and aligned to ground."));
  }
}

// Utility: commit imported mesh to robot environment.
void commitImportedMeshToRobotEnvironment(MeshData mesh, File sourceFile, String sourceLabel, String ext, int robotMode) {
  if (mesh == null || mesh.isEmpty() || mesh.minBound == null || mesh.maxBound == null) {
    updateMessage(tr("World import failed: geometry not found."));
    return;
  }

  PVector rawSize = PVector.sub(mesh.maxBound, mesh.minBound);
  float rawLargest = max(rawSize.x, max(rawSize.y, rawSize.z));
  if (rawLargest <= 0.0001f) {
    updateMessage(tr("World import failed: invalid bounds."));
    return;
  }

  // Normalize every imported world to centimeters first, then project those
  // centimeters into the active robot scene scale.
  JSONObject savedProfile = loadWorldProfileForSource(sourceFile, robotMode);
  JSONObject savedImport = savedProfile == null ? null : getJsonObjectSafe(savedProfile, "import");
  float unitsToCm = savedImport == null
    ? chooseImportedUnitsToCmScale(rawLargest, ext, robotMode)
    : max(0.0001f, getJsonFloat(savedImport, "units_to_cm", chooseImportedUnitsToCmScale(rawLargest, ext, robotMode)));
  float robotScale = robotSceneUnitsPerCm(robotMode);
  float finalSceneScale = unitsToCm * robotScale;
  setImportedWorldUnitsToCmForRobotMode(robotMode, unitsToCm);
  setImportedWorldScaleFactorForRobotMode(robotMode, finalSceneScale);

  PVector minScene = null;
  PVector maxScene = null;
  ArrayList<PVector> stagedLoose = new ArrayList<PVector>();
  ArrayList<MeshTriangle> stagedTriangles = new ArrayList<MeshTriangle>();

  for (int i = 0; i < mesh.loosePoints.size(); i++) {
    PVector pointCm = PVector.mult(mesh.loosePoints.get(i), unitsToCm);
    PVector scenePoint = remapImportedSourceToScene(PVector.mult(pointCm, robotScale));
    stagedLoose.add(scenePoint);
    if (minScene == null) {
      minScene = scenePoint.copy();
      maxScene = scenePoint.copy();
    } else {
      minScene.x = min(minScene.x, scenePoint.x);
      minScene.y = min(minScene.y, scenePoint.y);
      minScene.z = min(minScene.z, scenePoint.z);
      maxScene.x = max(maxScene.x, scenePoint.x);
      maxScene.y = max(maxScene.y, scenePoint.y);
      maxScene.z = max(maxScene.z, scenePoint.z);
    }
  }

  for (int i = 0; i < mesh.triangles.size(); i++) {
    MeshTriangle tri = mesh.triangles.get(i);
    PVector aCm = PVector.mult(tri.a, unitsToCm);
    PVector bCm = PVector.mult(tri.b, unitsToCm);
    PVector cCm = PVector.mult(tri.c, unitsToCm);
    PVector a = remapImportedSourceToScene(PVector.mult(aCm, robotScale));
    PVector b = remapImportedSourceToScene(PVector.mult(bCm, robotScale));
    PVector c = remapImportedSourceToScene(PVector.mult(cCm, robotScale));
    stagedTriangles.add(new MeshTriangle(a, b, c));
    PVector[] pts = {a, b, c};
    for (int k = 0; k < pts.length; k++) {
      PVector p = pts[k];
      if (minScene == null) {
        minScene = p.copy();
        maxScene = p.copy();
      } else {
        minScene.x = min(minScene.x, p.x);
        minScene.y = min(minScene.y, p.y);
        minScene.z = min(minScene.z, p.z);
        maxScene.x = max(maxScene.x, p.x);
        maxScene.y = max(maxScene.y, p.y);
        maxScene.z = max(maxScene.z, p.z);
      }
    }
  }

  if (minScene == null || maxScene == null) {
    updateMessage(tr("World import failed: empty scene data."));
    return;
  }

  PVector anchor = robotWorldAnchor(robotMode);
  float centerX = (minScene.x + maxScene.x) * 0.5f;
  float centerZ = (minScene.z + maxScene.z) * 0.5f;
  PVector offset = new PVector(anchor.x - centerX, GROUND_Y - maxScene.y, anchor.z - centerZ);

  ArrayList<PVector> dst = importedWorldPointListForRobotMode(robotMode);
  dst.clear();
  MeshData directMesh = importedWorldMeshForRobotMode(robotMode);
  directMesh.clear();
  resetDemoWorldForRobotMode(robotMode);
  disableBootDemoWorlds();

  // Keep the imported triangle geometry intact for direct 3D rendering. The
  // sampled point cloud below is generated only for collision and sensors.
  for (int i = 0; i < stagedLoose.size(); i++) directMesh.addLoosePoint(PVector.add(stagedLoose.get(i), offset));
  for (int i = 0; i < stagedTriangles.size(); i++) {
    MeshTriangle tri = stagedTriangles.get(i);
    directMesh.addTriangle(PVector.add(tri.a, offset), PVector.add(tri.b, offset), PVector.add(tri.c, offset));
  }

  // Build the initial invisible collision cloud from the direct mesh. Floor
  // filtering is intentionally deferred until Lock because the operator may
  // still rotate the world before defining its final up/down orientation.
  rebuildImportedWorldCollisionSamplesForRobotMode(robotMode, false);

  // Preserve the initial aligned import so Reset can restore the real mesh and
  // its collision samples after arbitrary scale/pitch/yaw/roll adjustments.
  copyMeshDataInto(importedWorldBaseMeshForRobotMode(robotMode), directMesh);
  copyPointListInto(importedWorldBasePointListForRobotMode(robotMode), dst);

  setWorldSourceLabelForRobotMode(robotMode, sourceLabel);
  setWorldSourceFileForRobotMode(robotMode, sourceFile);
  setWorldCompassNorthReferenceForRobotMode(robotMode, compassHeadingDegForRobotMode(robotMode));
  resetWorldTransformForRobotMode(robotMode);
  clearWorldTransformHistoryForRobotMode(robotMode);
  environmentCollisionEnabled = true;

  boolean profileRestored = savedProfile != null && applyWorldProfileTransformForRobotMode(robotMode, savedProfile);
  boolean restoredFloorIgnored = false;
  if (profileRestored) {
    snapImportedWorldToGroundForRobotMode(robotMode);
    restoredFloorIgnored = rebuildImportedWorldCollisionSamplesForRobotMode(robotMode, true);
    dismissWorldTransformOverlay();
  } else {
    activateWorldTransformOverlay(robotMode);
  }

  invalidateEnvCollisionCache(); // new world loaded: force a fresh collision check
  int scanCount = environmentPointCountForRobotMode(robotMode);
  if (profileRestored) {
    updateMessage(tr("World and saved profile restored: ") + sourceLabel
      + " | triangles " + directMesh.triangles.size()
      + " | collision pts " + dst.size()
      + (restoredFloorIgnored ? " | floor excluded" : "")
      + " | use Adjust world to edit");
  } else {
    updateMessage(tr("World loaded: ") + sourceLabel
      + " | triangles " + directMesh.triangles.size()
      + " | collision pts " + dst.size()
      + " | scan pts " + scanCount
      + " | units→cm x" + nf(unitsToCm, 1, 2)
      + " | robot x" + nf(robotScale, 1, 2)
      + " | final x" + nf(finalSceneScale, 1, 2)
      + " | adjust and lock");
  }
}


// Utility: file extension lower.
String fileExtensionLower(File file) {
  if (file == null) return "";
  String name = file.getName();
  int idx = name.lastIndexOf('.');
  if (idx < 0 || idx >= name.length() - 1) return "";
  return trim(name.substring(idx + 1).toLowerCase());
}

// Parses three floats.
PVector parseThreeFloats(String text, PVector fallback) {
  if (text == null) return fallback == null ? new PVector(0, 0, 0) : fallback.copy();
  String[] parts = splitTokens(text, " ,\t\r\n");
  if (parts == null || parts.length < 3) return fallback == null ? new PVector(0, 0, 0) : fallback.copy();
  return new PVector(parseFloat(parts[0]), parseFloat(parts[1]), parseFloat(parts[2]));
}

// Utility: multiply scale.
PVector multiplyScale(PVector a, PVector b) {
  if (a == null && b == null) return new PVector(1, 1, 1);
  if (a == null) return b.copy();
  if (b == null) return a.copy();
  return new PVector(a.x * b.x, a.y * b.y, a.z * b.z);
}

// Utility: safe scale vector.
PVector safeScaleVector(PVector s) {
  if (s == null) return new PVector(1, 1, 1);
  return new PVector(max(0.0001f, s.x), max(0.0001f, s.y), max(0.0001f, s.z));
}

// Utility: rotate point by rpy.
PVector rotatePointByRpy(PVector p, PVector rpy) {
  return rotateSourcePointXYZ(p, rpy == null ? new PVector(0, 0, 0) : rpy);
}

// Utility: compose world position.
PVector composeWorldPosition(PVector parentPos, PVector parentRpy, PVector parentScale, PVector localPos) {
  PVector scaledLocal = localPos == null ? new PVector(0, 0, 0) : localPos.copy();
  PVector safeParentScale = safeScaleVector(parentScale);
  scaledLocal.x *= safeParentScale.x;
  scaledLocal.y *= safeParentScale.y;
  scaledLocal.z *= safeParentScale.z;
  PVector rotated = rotatePointByRpy(scaledLocal, parentRpy);
  return PVector.add(parentPos == null ? new PVector(0, 0, 0) : parentPos, rotated);
}

// Utility: choose imported units to cm scale.
float chooseImportedUnitsToCmScale(float rawLargestDim, String ext, int robotMode) {
  String safeExt = ext == null ? "" : ext.trim().toLowerCase();

  if ("world".equals(safeExt) || "sdf".equals(safeExt) || "gazebo".equals(safeExt)) {
    // Gazebo world geometry is expressed in meters by default.
    return 100.0f;
  }

  float[] candidates = "stl".equals(safeExt)
    ? new float[] {0.1f, 1.0f, 100.0f, 0.01f}
    : new float[] {1.0f, 100.0f, 0.1f, 0.01f};

  float bestScale = candidates[0];
  float bestScore = Float.MAX_VALUE;

  for (int i = 0; i < candidates.length; i++) {
    float candidate = candidates[i];
    float candidateCm = rawLargestDim * candidate;
    float score = abs(log(max(1.0f, candidateCm) / 250.0f));

    if (candidateCm < 5.0f) score += 6.0f;
    else if (candidateCm < 20.0f) score += 2.5f;

    if (candidateCm > 500000.0f) score += 6.0f;
    else if (candidateCm > 50000.0f) score += 2.5f;

    if ("stl".equals(safeExt)) {
      if (candidate == 0.1f && rawLargestDim >= 80.0f) score -= 0.6f;   // STL often arrives in mm
      if (candidate == 100.0f && rawLargestDim <= 20.0f) score -= 0.6f; // or in meters
    } else {
      if (candidate == 100.0f && rawLargestDim <= 30.0f) score -= 0.4f;
      if (candidate == 1.0f && rawLargestDim >= 30.0f && rawLargestDim <= 5000.0f) score -= 0.2f;
    }

    if (score < bestScore) {
      bestScore = score;
      bestScale = candidate;
    }
  }

  if (rawLargestDim < 0.5f) return 100.0f;
  if (rawLargestDim > 20000.0f) return 0.1f;
  return bestScale;
}

// Utility: choose imported units to cm scale.
float chooseImportedUnitsToCmScale(float rawLargestDim, String ext) {

  return chooseImportedUnitsToCmScale(rawLargestDim, ext, currentRobotMode());
}

// Utility: little endian float.
float littleEndianFloat(byte[] data, int offset) {
  int bits = (data[offset] & 0xFF) |
             ((data[offset + 1] & 0xFF) << 8) |
             ((data[offset + 2] & 0xFF) << 16) |
             ((data[offset + 3] & 0xFF) << 24);
  return Float.intBitsToFloat(bits);
}

// Utility: little endian u int 32.
long littleEndianUInt32(byte[] data, int offset) {
  return ((long)(data[offset] & 0xFF)) |
         (((long)(data[offset + 1] & 0xFF)) << 8) |
         (((long)(data[offset + 2] & 0xFF)) << 16) |
         (((long)(data[offset + 3] & 0xFF)) << 24);
}

// Loads binary or ascii stl.
MeshData loadBinaryOrAsciiStl(File file) {
  MeshData mesh = new MeshData();
  byte[] bytes = loadBytes(file.getAbsolutePath());
  if (bytes == null || bytes.length < 84) return mesh;

  long triCount = littleEndianUInt32(bytes, 80);
  long expected = 84L + triCount * 50L;
  boolean looksBinary = expected == bytes.length && triCount >= 1;

  if (looksBinary) {
    int offset = 84;
    for (int i = 0; i < triCount && offset + 50 <= bytes.length; i++) {
      offset += 12;
      PVector a = new PVector(littleEndianFloat(bytes, offset), littleEndianFloat(bytes, offset + 4), littleEndianFloat(bytes, offset + 8)); offset += 12;
      PVector b = new PVector(littleEndianFloat(bytes, offset), littleEndianFloat(bytes, offset + 4), littleEndianFloat(bytes, offset + 8)); offset += 12;
      PVector c = new PVector(littleEndianFloat(bytes, offset), littleEndianFloat(bytes, offset + 4), littleEndianFloat(bytes, offset + 8)); offset += 12;
      mesh.addTriangle(a, b, c);
      offset += 2;
    }
    return mesh;
  }

  String[] lines = loadStrings(file.getAbsolutePath());
  if (lines == null) return mesh;
  ArrayList<PVector> triVerts = new ArrayList<PVector>();
  for (int i = 0; i < lines.length; i++) {
    String line = trim(lines[i]);
    if (line.startsWith("vertex")) {
      String[] parts = splitTokens(line.substring(6), " ");
      if (parts != null && parts.length >= 3) {
        triVerts.add(new PVector(parseFloat(parts[0]), parseFloat(parts[1]), parseFloat(parts[2])));
        if (triVerts.size() == 3) {
          mesh.addTriangle(triVerts.get(0), triVerts.get(1), triVerts.get(2));
          triVerts.clear();
        }
      }
    }
  }
  return mesh;
}

// Parses obj index token.
int parseObjIndexToken(String token, int vertexCount) {
  if (token == null || token.length() == 0) return -1;
  String[] parts = split(token, '/');
  if (parts == null || parts.length == 0 || parts[0].length() == 0) return -1;
  int idx = parseInt(parts[0]);
  if (idx > 0) return idx - 1;
  if (idx < 0) return vertexCount + idx;
  return -1;
}

// Loads obj mesh.
MeshData loadObjMesh(File file) {
  MeshData mesh = new MeshData();
  String[] lines = loadStrings(file.getAbsolutePath());
  if (lines == null) return mesh;

  ArrayList<PVector> vertices = new ArrayList<PVector>();
  for (int i = 0; i < lines.length; i++) {
    String line = trim(lines[i]);
    if (line.length() == 0 || line.startsWith("#")) continue;

    if (line.startsWith("v ")) {
      String[] parts = splitTokens(line.substring(2), " ");
      if (parts != null && parts.length >= 3) {
        vertices.add(new PVector(parseFloat(parts[0]), parseFloat(parts[1]), parseFloat(parts[2])));
      }
    } else if (line.startsWith("f ")) {
      String[] parts = splitTokens(line.substring(2), " ");
      if (parts == null || parts.length < 3) continue;
      int first = parseObjIndexToken(parts[0], vertices.size());
      for (int k = 1; k < parts.length - 1; k++) {
        int second = parseObjIndexToken(parts[k], vertices.size());
        int third = parseObjIndexToken(parts[k + 1], vertices.size());
        if (first >= 0 && second >= 0 && third >= 0 &&
            first < vertices.size() && second < vertices.size() && third < vertices.size()) {
          mesh.addTriangle(vertices.get(first), vertices.get(second), vertices.get(third));
        }
      }
    }
  }

  if (mesh.isEmpty()) {
    for (int i = 0; i < vertices.size(); i++) mesh.addLoosePoint(vertices.get(i));
  }
  return mesh;
}

// Utility: add box primitive.
void addBoxPrimitive(MeshData mesh, PVector size, PVector translation, PVector rpy) {
  float hx = size.x * 0.5f;
  float hy = size.y * 0.5f;
  float hz = size.z * 0.5f;
  PVector[] corners = new PVector[] {
    new PVector(-hx, -hy, -hz), new PVector(hx, -hy, -hz), new PVector(hx, hy, -hz), new PVector(-hx, hy, -hz),
    new PVector(-hx, -hy, hz),  new PVector(hx, -hy, hz),  new PVector(hx, hy, hz),  new PVector(-hx, hy, hz)
  };
  int[][] faces = {
    {0, 1, 2}, {0, 2, 3},
    {4, 5, 6}, {4, 6, 7},
    {0, 1, 5}, {0, 5, 4},
    {1, 2, 6}, {1, 6, 5},
    {2, 3, 7}, {2, 7, 6},
    {3, 0, 4}, {3, 4, 7}
  };
  for (int i = 0; i < faces.length; i++) {
    int[] f = faces[i];
    mesh.addTriangle(
      transformImportedSourcePoint(corners[f[0]], translation, rpy, new PVector(1, 1, 1)),
      transformImportedSourcePoint(corners[f[1]], translation, rpy, new PVector(1, 1, 1)),
      transformImportedSourcePoint(corners[f[2]], translation, rpy, new PVector(1, 1, 1))
      );
  }
}

// Utility: add cylinder primitive.
void addCylinderPrimitive(MeshData mesh, float radius, float length, PVector translation, PVector rpy) {
  int sides = 18;
  float half = length * 0.5f;
  for (int i = 0; i < sides; i++) {
    float a0 = TWO_PI * i / sides;
    float a1 = TWO_PI * (i + 1) / sides;
    PVector p0 = new PVector(radius * cos(a0), radius * sin(a0), -half);
    PVector p1 = new PVector(radius * cos(a1), radius * sin(a1), -half);
    PVector p2 = new PVector(radius * cos(a1), radius * sin(a1), half);
    PVector p3 = new PVector(radius * cos(a0), radius * sin(a0), half);
    mesh.addTriangle(transformImportedSourcePoint(p0, translation, rpy, new PVector(1, 1, 1)),
                     transformImportedSourcePoint(p1, translation, rpy, new PVector(1, 1, 1)),
                     transformImportedSourcePoint(p2, translation, rpy, new PVector(1, 1, 1)));
    mesh.addTriangle(transformImportedSourcePoint(p0, translation, rpy, new PVector(1, 1, 1)),
                     transformImportedSourcePoint(p2, translation, rpy, new PVector(1, 1, 1)),
                     transformImportedSourcePoint(p3, translation, rpy, new PVector(1, 1, 1)));

    PVector topC = new PVector(0, 0, half);
    PVector botC = new PVector(0, 0, -half);
    mesh.addTriangle(transformImportedSourcePoint(topC, translation, rpy, new PVector(1, 1, 1)),
                     transformImportedSourcePoint(p3, translation, rpy, new PVector(1, 1, 1)),
                     transformImportedSourcePoint(p2, translation, rpy, new PVector(1, 1, 1)));
    mesh.addTriangle(transformImportedSourcePoint(botC, translation, rpy, new PVector(1, 1, 1)),
                     transformImportedSourcePoint(p1, translation, rpy, new PVector(1, 1, 1)),
                     transformImportedSourcePoint(p0, translation, rpy, new PVector(1, 1, 1)));
  }
}

// Utility: add sphere primitive.
void addSpherePrimitive(MeshData mesh, float radius, PVector translation, PVector rpy) {
  int latSteps = 8;
  int lonSteps = 14;
  for (int lat = 0; lat < latSteps; lat++) {
    float t0 = map(lat, 0, latSteps, -HALF_PI, HALF_PI);
    float t1 = map(lat + 1, 0, latSteps, -HALF_PI, HALF_PI);
    for (int lon = 0; lon < lonSteps; lon++) {
      float p0 = TWO_PI * lon / lonSteps;
      float p1 = TWO_PI * (lon + 1) / lonSteps;
      PVector a = new PVector(radius * cos(t0) * cos(p0), radius * cos(t0) * sin(p0), radius * sin(t0));
      PVector b = new PVector(radius * cos(t0) * cos(p1), radius * cos(t0) * sin(p1), radius * sin(t0));
      PVector c = new PVector(radius * cos(t1) * cos(p1), radius * cos(t1) * sin(p1), radius * sin(t1));
      PVector d = new PVector(radius * cos(t1) * cos(p0), radius * cos(t1) * sin(p0), radius * sin(t1));
      mesh.addTriangle(transformImportedSourcePoint(a, translation, rpy, new PVector(1, 1, 1)),
                       transformImportedSourcePoint(b, translation, rpy, new PVector(1, 1, 1)),
                       transformImportedSourcePoint(c, translation, rpy, new PVector(1, 1, 1)));
      mesh.addTriangle(transformImportedSourcePoint(a, translation, rpy, new PVector(1, 1, 1)),
                       transformImportedSourcePoint(c, translation, rpy, new PVector(1, 1, 1)),
                       transformImportedSourcePoint(d, translation, rpy, new PVector(1, 1, 1)));
    }
  }
}

// Utility: normalize gazebo uri.
String normalizeGazeboUri(String uri) {
  if (uri == null) return "";
  String cleaned = trim(uri).replace('\\', '/');
  if (cleaned.startsWith("file://")) cleaned = cleaned.substring(7);
  return cleaned;
}

// Utility: canonical path safe.
String canonicalPathSafe(File file) {
  try {
    return file == null ? "" : file.getCanonicalPath();
  }
  catch (Exception e) {
    return file == null ? "" : file.getAbsolutePath();
  }
}

// Utility: add unique file.
void addUniqueFile(ArrayList<File> files, File candidate) {
  if (candidate == null) return;
  String canonical = canonicalPathSafe(candidate);
  for (int i = 0; i < files.size(); i++) {
    if (canonical.equals(canonicalPathSafe(files.get(i)))) return;
  }
  files.add(candidate);
}

// Utility: gazebo search roots.
ArrayList<File> gazeboSearchRoots(File rootFile) {
  ArrayList<File> roots = new ArrayList<File>();
  File parent = rootFile != null ? rootFile.getParentFile() : null;
  File walk = parent;
  while (walk != null) {
    addUniqueFile(roots, walk);
    addUniqueFile(roots, new File(walk, "models"));
    walk = walk.getParentFile();
  }

  String[] envVars = {"GAZEBO_MODEL_PATH", "IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH"};
  for (int i = 0; i < envVars.length; i++) {
    String envValue = System.getenv(envVars[i]);
    if (envValue == null || envValue.length() == 0) continue;
    String[] parts = split(envValue, File.pathSeparatorChar);
    if (parts == null) continue;
    for (int k = 0; k < parts.length; k++) {
      if (parts[k] == null || trim(parts[k]).length() == 0) continue;
      addUniqueFile(roots, new File(trim(parts[k])));
    }
  }

  String home = System.getProperty("user.home", "");
  if (home != null && home.length() > 0) {
    addUniqueFile(roots, new File(home, ".gazebo/models"));
    addUniqueFile(roots, new File(home, "gazebo_models"));
  }
  return roots;
}

// Utility: resolve gazebo model entry file.
File resolveGazeboModelEntryFile(File modelDir) {
  if (modelDir == null) return null;
  if (modelDir.isFile()) return modelDir;
  if (!modelDir.exists()) return null;

  File[] preferred = new File[] {
    new File(modelDir, "model.sdf"),
    new File(modelDir, "model.world"),
    new File(modelDir, "world.sdf"),
    new File(modelDir, "main.sdf")
  };
  for (int i = 0; i < preferred.length; i++) {
    if (preferred[i].exists()) return preferred[i];
  }

  File config = new File(modelDir, "model.config");
  if (config.exists()) {
    try {
      XML xml = loadXML(config.getAbsolutePath());
      if (xml != null) {
        XML[] sdfNodes = xml.getChildren("sdf");
        if (sdfNodes != null) {
          for (int i = 0; i < sdfNodes.length; i++) {
            String entry = trim(sdfNodes[i].getContent());
            if (entry != null && entry.length() > 0) {
              File candidate = new File(modelDir, entry);
              if (candidate.exists()) return candidate;
            }
          }
        }
      }
    }
    catch (Exception e) {
    }
  }

  File[] files = modelDir.listFiles();
  if (files != null) {
    for (int i = 0; i < files.length; i++) {
      String ext = fileExtensionLower(files[i]);
      if ("sdf".equals(ext) || "world".equals(ext) || "gazebo".equals(ext)) return files[i];
    }
  }
  return null;
}

// Utility: resolve gazebo uri.
File resolveGazeboUri(File rootFile, String uri) {
  String cleaned = normalizeGazeboUri(uri);
  if (cleaned.length() == 0) return null;

  File direct = new File(cleaned);
  if (direct.exists()) {
    return direct.isDirectory() ? resolveGazeboModelEntryFile(direct) : direct;
  }

  File parent = rootFile != null ? rootFile.getParentFile() : null;
  if (!cleaned.startsWith("model://") && !cleaned.startsWith("package://")) {
    if (parent != null) {
      File relative = new File(parent, cleaned);
      if (relative.exists()) return relative.isDirectory() ? resolveGazeboModelEntryFile(relative) : relative;
    }
    return null;
  }

  String relative = cleaned.startsWith("model://") ? cleaned.substring(8) : cleaned.substring(10);
  while (relative.startsWith("/")) relative = relative.substring(1);
  ArrayList<File> roots = gazeboSearchRoots(rootFile);

  for (int i = 0; i < roots.size(); i++) {
    File root = roots.get(i);
    if (root == null) continue;
    File exact = new File(root, relative);
    if (exact.exists()) return exact.isDirectory() ? resolveGazeboModelEntryFile(exact) : exact;

    File modelsRelative = new File(new File(root, "models"), relative);
    if (modelsRelative.exists()) return modelsRelative.isDirectory() ? resolveGazeboModelEntryFile(modelsRelative) : modelsRelative;

    int slash = relative.indexOf('/');
    if (slash > 0) {
      File modelRoot = new File(root, relative.substring(0, slash));
      if (modelRoot.exists()) {
        File inside = new File(modelRoot, relative.substring(slash + 1));
        if (inside.exists()) return inside.isDirectory() ? resolveGazeboModelEntryFile(inside) : inside;
      }
      File modelRoot2 = new File(new File(root, "models"), relative.substring(0, slash));
      if (modelRoot2.exists()) {
        File inside = new File(modelRoot2, relative.substring(slash + 1));
        if (inside.exists()) return inside.isDirectory() ? resolveGazeboModelEntryFile(inside) : inside;
      }
    }
  }
  return null;
}

// Parses pose vector.
PVector parsePoseVector(XML poseChild, boolean rotationPart) {
  if (poseChild == null) return new PVector(0, 0, 0);
  String content = trim(poseChild.getContent());
  String[] vals = splitTokens(content, " ");
  if (vals == null) return new PVector(0, 0, 0);
  if (!rotationPart && vals.length >= 3) {
    return new PVector(parseFloat(vals[0]), parseFloat(vals[1]), parseFloat(vals[2]));
  }
  if (rotationPart && vals.length >= 6) {
    return new PVector(parseFloat(vals[3]), parseFloat(vals[4]), parseFloat(vals[5]));
  }
  return new PVector(0, 0, 0);
}

// Utility: collect gazebo include.
void collectGazeboInclude(XML includeNode, MeshData out, File rootFile, PVector parentPos, PVector parentRpy, PVector parentScale, int depth, java.util.HashSet<String> activeStack) {
  if (includeNode == null) return;
  XML uriNode = includeNode.getChild("uri");
  if (uriNode == null) return;

  PVector localPos = parsePoseVector(includeNode.getChild("pose"), false);
  PVector localRpy = parsePoseVector(includeNode.getChild("pose"), true);
  PVector worldPos = composeWorldPosition(parentPos, parentRpy, parentScale, localPos);
  PVector worldRpy = PVector.add(parentRpy, localRpy);
  PVector includeScale = parseThreeFloats(includeNode.getChild("scale") != null ? includeNode.getChild("scale").getContent() : null, new PVector(1, 1, 1));
  PVector totalScale = multiplyScale(parentScale, includeScale);

  File includeTarget = resolveGazeboUri(rootFile, trim(uriNode.getContent()));
  if (includeTarget == null || !includeTarget.exists()) return;

  MeshData child = loadMeshDataFromFile(includeTarget, depth + 1, activeStack);
  if (child == null || child.isEmpty()) return;
  accumulateMeshData(out, child, worldPos, worldRpy, totalScale);
}

// Utility: collect gazebo node geometry.
void collectGazeboNodeGeometry(XML node, MeshData out, File rootFile, PVector parentPos, PVector parentRpy, PVector parentScale, int depth, java.util.HashSet<String> activeStack) {
  if (node == null || depth > 6) return;

  String nodeName = node.getName();
  if ("include".equals(nodeName)) {
    collectGazeboInclude(node, out, rootFile, parentPos, parentRpy, parentScale, depth, activeStack);
    return;
  }

  PVector localPos = parsePoseVector(node.getChild("pose"), false);
  PVector localRpy = parsePoseVector(node.getChild("pose"), true);
  PVector worldPos = composeWorldPosition(parentPos, parentRpy, parentScale, localPos);
  PVector worldRpy = PVector.add(parentRpy, localRpy);
  PVector worldScale = safeScaleVector(parentScale);

  XML geometry = node.getChild("geometry");
  if (geometry != null) {
    XML box = geometry.getChild("box");
    if (box != null) {
      XML sizeNode = box.getChild("size");
      if (sizeNode != null) addBoxPrimitive(out, multiplyScale(parseThreeFloats(sizeNode.getContent(), new PVector(1, 1, 1)), worldScale), worldPos, worldRpy);
    }

    XML cylinder = geometry.getChild("cylinder");
    if (cylinder != null) {
      float radius = 0.5f;
      float length = 1.0f;
      XML radiusNode = cylinder.getChild("radius");
      XML lengthNode = cylinder.getChild("length");
      if (radiusNode != null) radius = parseFloat(trim(radiusNode.getContent()));
      if (lengthNode != null) length = parseFloat(trim(lengthNode.getContent()));
      float radialScale = (abs(worldScale.x) + abs(worldScale.y)) * 0.5f;
      addCylinderPrimitive(out, radius * max(0.0001f, radialScale), length * abs(worldScale.z), worldPos, worldRpy);
    }

    XML sphere = geometry.getChild("sphere");
    if (sphere != null) {
      float radius = 0.5f;
      XML radiusNode = sphere.getChild("radius");
      if (radiusNode != null) radius = parseFloat(trim(radiusNode.getContent()));
      float avgScale = (abs(worldScale.x) + abs(worldScale.y) + abs(worldScale.z)) / 3.0f;
      addSpherePrimitive(out, radius * max(0.0001f, avgScale), worldPos, worldRpy);
    }

    XML meshNode = geometry.getChild("mesh");
    if (meshNode != null) {
      XML uriNode = meshNode.getChild("uri");
      if (uriNode != null) {
        File meshFile = resolveGazeboUri(rootFile, trim(uriNode.getContent()));
        if (meshFile != null && meshFile.exists()) {
          MeshData child = loadMeshDataFromFile(meshFile, depth + 1, activeStack);
          PVector scaleVec = new PVector(1, 1, 1);
          XML scaleNode = meshNode.getChild("scale");
          if (scaleNode != null) scaleVec = parseThreeFloats(scaleNode.getContent(), scaleVec);
          accumulateMeshData(out, child, worldPos, worldRpy, multiplyScale(worldScale, scaleVec));
        }
      }
    }
  }

  XML[] children = node.getChildren();
  for (int i = 0; i < children.length; i++) {
    String childName = children[i].getName();
    if ("geometry".equals(childName) || "pose".equals(childName)) continue;
    collectGazeboNodeGeometry(children[i], out, rootFile, worldPos, worldRpy, worldScale, depth + 1, activeStack);
  }
}

// Loads gazebo world.
MeshData loadGazeboWorld(File file, int depth, java.util.HashSet<String> activeStack) {
  MeshData mesh = new MeshData();
  if (file == null || !file.exists() || depth > 6) return mesh;

  String canonical = canonicalPathSafe(file);
  if (activeStack.contains(canonical)) return mesh;
  activeStack.add(canonical);

  try {
    XML xml = loadXML(file.getAbsolutePath());
    if (xml != null) {
      collectGazeboNodeGeometry(xml, mesh, file, new PVector(0, 0, 0), new PVector(0, 0, 0), new PVector(1, 1, 1), depth, activeStack);
    }
  }
  catch (Exception e) {
  }

  activeStack.remove(canonical);
  return mesh;
}

// Loads gazebo world.
MeshData loadGazeboWorld(File file) {
  return loadGazeboWorld(file, 0, new java.util.HashSet<String>());
}

// Loads mesh data from file.
MeshData loadMeshDataFromFile(File file, int depth, java.util.HashSet<String> activeStack) {
  MeshData mesh = new MeshData();
  if (file == null || !file.exists() || depth > 6) return mesh;

  File resolved = file.isDirectory() ? resolveGazeboModelEntryFile(file) : file;
  if (resolved == null || !resolved.exists()) return mesh;

  String ext = fileExtensionLower(resolved);
  if ("stl".equals(ext)) return loadBinaryOrAsciiStl(resolved);
  if ("obj".equals(ext)) return loadObjMesh(resolved);
  if ("world".equals(ext) || "sdf".equals(ext) || "gazebo".equals(ext)) return loadGazeboWorld(resolved, depth, activeStack);
  return mesh;
}

// Loads mesh data from file.
MeshData loadMeshDataFromFile(File file, int depth) {
  return loadMeshDataFromFile(file, depth, new java.util.HashSet<String>());
}

// Builds import mesh result.
ImportMeshResult buildImportMeshResult(File file, int robotMode) {
  ImportMeshResult result = new ImportMeshResult();
  result.robotMode = robotMode;
  result.sourceFile = file;
  result.sourceLabel = file != null ? file.getName() : "";
  result.ext = fileExtensionLower(file);

  if (file == null || !file.exists()) {
    result.errorMessage = "World import cancelled.";
    return result;
  }

  if (!("stl".equals(result.ext) || "obj".equals(result.ext) || "world".equals(result.ext) || "sdf".equals(result.ext) || "gazebo".equals(result.ext))) {
    result.errorMessage = "Unsupported world format: ." + result.ext;
    return result;
  }

  try {
    result.mesh = loadMeshDataFromFile(file, 0, new java.util.HashSet<String>());
  }
  catch (Exception e) {
    result.errorMessage = "World import failed: " + e.getClass().getSimpleName();
    return result;
  }

  if (result.mesh == null || result.mesh.isEmpty()) {
    result.errorMessage = "World import failed: no usable geometry in " + file.getName();
  }
  return result;
}


// Localizes worker-produced import errors without storing translated text in
// background-thread state. Dynamic details (file name, exception class, format)
// remain untouched while the operator-facing prefix follows the active pack.
String localizedWorldImportError(String rawMessage) {
  String raw = rawMessage == null ? "" : rawMessage;
  String noGeometryPrefix = "World import failed: no usable geometry in ";
  if (raw.startsWith(noGeometryPrefix)) {
    return tr(noGeometryPrefix) + raw.substring(noGeometryPrefix.length());
  }
  String unsupportedPrefix = "Unsupported world format: .";
  if (raw.startsWith(unsupportedPrefix)) {
    return tr(unsupportedPrefix) + raw.substring(unsupportedPrefix.length());
  }
  String failedPrefix = "World import failed: ";
  if (raw.startsWith(failedPrefix)) {
    return tr(failedPrefix) + raw.substring(failedPrefix.length());
  }
  return tr(raw);
}

// Utility: process pending world import.
void processPendingWorldImport() {
  ImportMeshResult result = null;
  synchronized(worldImportLock) {
    if (pendingWorldImportResult != null) {
      result = pendingWorldImportResult;
      pendingWorldImportResult = null;
    }
  }
  if (result == null) return;

  if (result.errorMessage != null && result.errorMessage.length() > 0) {
    updateMessage(localizedWorldImportError(result.errorMessage));
    return;
  }
  commitImportedMeshToRobotEnvironment(result.mesh, result.sourceFile, result.sourceLabel, result.ext, result.robotMode);
}

// Utility: queue async world import.
void queueAsyncWorldImport(final File file, final int robotMode) {
  if (worldImportBusy) {
    updateMessage(tr("World import already in progress."));
    return;
  }

  worldImportBusy = true;
  updateMessage(file == null ? tr("World import cancelled.") : tr("Importing world: ") + file.getName());

  Thread worker = new Thread(new Runnable() {
    public void run() {
      ImportMeshResult result = buildImportMeshResult(file, robotMode);
      synchronized(worldImportLock) {
        pendingWorldImportResult = result;
      }
      worldImportBusy = false;
    }
  }, "SynROV-WorldImport");
  worker.start();
}

// Canonical robot name stored in a sensor-created collision world.
String collisionWorldRobotName(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return "Vehicle";
  if (robotMode == ROBOT_MODE_DRONE) return "Drone";
  return "Manipulator";
}

// Ensures the dedicated SynROV collision-world extension is used.
File ensureCollisionWorldExtension(File selection) {
  if (selection == null) return null;
  String name = selection.getName();
  String suffix = "." + COLLISION_WORLD_EXTENSION;
  if (name.toLowerCase().endsWith(suffix)) return selection;
  int dot = name.lastIndexOf('.');
  String stem = dot > 0 ? name.substring(0, dot) : name;
  File parent = selection.getParentFile();
  return new File(parent, stem + suffix);
}

// Builds the portable world generated exclusively from the robot's sensor map.
JSONObject buildCollisionWorldArchive(int robotMode) {
  ArrayList<PVector> source = environmentPointListForRobotMode(robotMode);
  if (source == null || source.isEmpty()) return null;
  JSONObject archive = new JSONObject();
  archive.setString("schema", COLLISION_WORLD_SCHEMA);
  archive.setInt("softwareVersion", SYNROV_SOFTWARE_VERSION);
  archive.setString("robot", collisionWorldRobotName(robotMode));
  archive.setString("collisionRobot", collisionWorldRobotName(robotMode));
  archive.setString("collisionPolicy", "solid");
  archive.setBoolean("collisionEnabled", true);
  archive.setString("coordinateSpace", "synrov_scene_world");
  archive.setFloat("sceneUnitsPerMeter", sceneUnitsPerMeterForRobotMode(robotMode));
  archive.setFloat("pointRadiusScene", environmentPointRadiusForRobotMode(robotMode));
  archive.setBoolean("northLocked", worldCompassNorthLockedForRobotMode(robotMode));
  archive.setFloat("northOffsetDeg", worldCompassNorthOffsetDegForRobotMode(robotMode));
  archive.setInt("pointCount", min(source.size(), ENVIRONMENT_MAX_POINTS_PER_MAP));

  JSONArray points = new JSONArray();
  int start = max(0, source.size() - ENVIRONMENT_MAX_POINTS_PER_MAP);
  int outIndex = 0;
  for (int i = start; i < source.size(); i++) {
    PVector p = source.get(i);
    if (p == null) continue;
    JSONObject item = new JSONObject();
    item.setFloat("x", p.x);
    item.setFloat("y", p.y);
    item.setFloat("z", p.z);
    points.setJSONObject(outIndex++, item);
  }
  archive.setJSONArray("points", points);
  archive.setInt("pointCount", outIndex);
  return archive;
}

// Callback from Processing's native save dialog.
void onCollisionWorldFileSelected(File selection) {
  collisionWorldSaveDialogOpen = false;
  if (selection == null) {
    updateMessage(tr("Collision-world save cancelled."));
    return;
  }
  JSONObject archive = buildCollisionWorldArchive(queuedCollisionWorldSaveRobotMode);
  if (archive == null) {
    updateMessage(tr("No mapped obstacles to save."));
    return;
  }
  File target = ensureCollisionWorldExtension(selection);
  try {
    saveJSONObject(archive, target.getAbsolutePath());
    updateMessage(tr("Collision world saved: ") + target.getName()
      + " | " + archive.getInt("pointCount") + " pts");
  }
  catch (Exception e) {
    updateMessage(tr("Collision-world save failed: ") + e.getClass().getSimpleName());
  }
}

// Opens a native dialog for the sensor-map world archive.
void saveCollisionWorldFromWindowsDialog() {
  if (collisionWorldSaveDialogOpen) return;
  int robotMode = currentRobotMode();
  ArrayList<PVector> source = environmentPointListForRobotMode(robotMode);
  if (source == null || source.isEmpty()) {
    updateMessage(tr("Map obstacles with the robot sensors before saving."));
    return;
  }
  queuedCollisionWorldSaveRobotMode = robotMode;
  collisionWorldSaveDialogOpen = true;
  selectOutput("Save collision world (*.synrovworld)", "onCollisionWorldFileSelected");
}

// Loads a sensor-created .synrovworld directly as the active imported world.
boolean loadCollisionWorldFile(File file, int robotMode) {
  if (file == null || !file.exists()) return false;
  JSONObject archive = null;
  try {
    archive = loadJSONObject(file.getAbsolutePath());
  }
  catch (Exception e) {
    updateMessage(tr("Invalid collision-world file."));
    return false;
  }
  if (archive == null || !COLLISION_WORLD_SCHEMA.equals(getJsonString(archive, "schema", "")) || getJsonInt(archive, "softwareVersion", -1) != SYNROV_SOFTWARE_VERSION) {
    updateMessage(tr("Incompatible .synrovworld file."));
    return false;
  }

  String savedRobot = normalizeRobotTypeName(getJsonString(archive, "robot", ""));
  String collisionRobot = normalizeRobotTypeName(getJsonString(archive, "collisionRobot", ""));
  String collisionPolicy = getJsonString(archive, "collisionPolicy", "");
  boolean collisionEnabled = getJsonBoolean(archive, "collisionEnabled", false);
  String targetRobot = collisionWorldRobotName(robotMode);
  if (savedRobot.length() == 0 || !savedRobot.equals(targetRobot) ||
      collisionRobot.length() == 0 || !collisionRobot.equals(targetRobot) ||
      !"solid".equals(collisionPolicy) || !collisionEnabled) {
    updateMessage(tr("Collision world is incompatible with this robot."));
    return false;
  }

  JSONArray points = null;
  try { points = archive.getJSONArray("points"); } catch (Exception ignored) {}
  if (points == null || points.size() == 0) {
    updateMessage(tr("Collision world contains no obstacles."));
    return false;
  }

  clearImportedWorldForRobotMode(robotMode);
  environmentPointListForRobotMode(robotMode).clear();
  resetDemoWorldForRobotMode(robotMode);
  disableBootDemoWorlds();

  MeshData mesh = importedWorldMeshForRobotMode(robotMode);
  ArrayList<PVector> collisionPoints = importedWorldPointListForRobotMode(robotMode);
  int accepted = 0;
  for (int i = 0; i < points.size() && accepted < ENVIRONMENT_MAX_POINTS_PER_MAP; i++) {
    JSONObject item = null;
    try { item = points.getJSONObject(i); } catch (Exception ignored) {}
    if (item == null) continue;
    PVector point = new PVector(
      getJsonFloat(item, "x", 0.0f),
      getJsonFloat(item, "y", GROUND_Y),
      getJsonFloat(item, "z", 0.0f)
    );
    mesh.addLoosePoint(point.copy());
    collisionPoints.add(point);
    accepted++;
  }
  if (accepted == 0) {
    clearImportedWorldForRobotMode(robotMode);
    updateMessage(tr("No valid points in collision world."));
    return false;
  }
  mesh.recalculateBounds();
  copyMeshDataInto(importedWorldBaseMeshForRobotMode(robotMode), mesh);
  copyPointListInto(importedWorldBasePointListForRobotMode(robotMode), collisionPoints);
  setWorldSourceLabelForRobotMode(robotMode, file.getName());
  setWorldSourceFileForRobotMode(robotMode, file);
  setImportedWorldUnitsToCmForRobotMode(robotMode, 1.0f / max(0.0001f, robotSceneUnitsPerCm(robotMode)));
  setImportedWorldScaleFactorForRobotMode(robotMode, 1.0f);
  resetWorldTransformForRobotMode(robotMode);
  clearWorldTransformHistoryForRobotMode(robotMode);
  setWorldTransformReferenceYawForRobotMode(robotMode, cameraRotationY);
  if (getJsonBoolean(archive, "northLocked", false)) {
    setWorldCompassNorthReferenceForRobotMode(robotMode, getJsonFloat(archive, "northOffsetDeg", 0.0f));
  } else {
    clearWorldCompassNorthReferenceForRobotMode(robotMode);
  }
  setCollisionWorldSolidForRobotMode(robotMode, true);
  dismissWorldTransformOverlay();

  // If this saved world has an operator transform profile, restore it exactly.
  JSONObject savedProfile = loadWorldProfileForSource(file, robotMode);
  if (savedProfile != null) applyWorldProfileTransformForRobotMode(robotMode, savedProfile);

  invalidateEnvCollisionCache();
  updateMessage(tr("Collision world loaded: ") + file.getName()
    + " | " + accepted + " pts");
  return true;
}

// Utility: on world map file selected.
void onWorldMapFileSelected(File selection) {
  worldImportDialogOpen = false;
  if (selection == null) {
    updateMessage(tr("World import cancelled."));
    return;
  }
  if (COLLISION_WORLD_EXTENSION.equals(fileExtensionLower(selection))) {
    loadCollisionWorldFile(selection, queuedWorldImportRobotMode);
    return;
  }
  queueAsyncWorldImport(selection, queuedWorldImportRobotMode);
}

// Loads world map from windows dialog.
void loadWorldMapFromWindowsDialog() {
  if (worldImportDialogOpen || worldImportBusy) {
    updateMessage(tr("World import already in progress."));
    return;
  }
  queuedWorldImportRobotMode = currentRobotMode();
  worldImportDialogOpen = true;
  selectInput("Select 3D world / environment map (*.synrovworld supported)", "onWorldMapFileSelected");
}

// Utility: rotate local to world.
PVector rotateLocalToWorld(PVector local, float yaw) {
  float c = cos(yaw);
  float s = sin(yaw);
  return new PVector(local.x * c + local.z * s, local.y, -local.x * s + local.z * c);
}

// Utility: rotate world to local.
PVector rotateWorldToLocal(PVector worldDelta, float yaw) {
  float c = cos(yaw);
  float s = sin(yaw);
  return new PVector(worldDelta.x * c - worldDelta.z * s, worldDelta.y, worldDelta.x * s + worldDelta.z * c);
}

// Utility: rotate vector around X.
PVector rotateVectorAroundX(PVector v, float angle) {
  float c = cos(angle);
  float s = sin(angle);
  return new PVector(v.x, v.y * c - v.z * s, v.y * s + v.z * c);
}

// Utility: rotate vector around Y.
PVector rotateVectorAroundY(PVector v, float angle) {
  float c = cos(angle);
  float s = sin(angle);
  return new PVector(v.x * c + v.z * s, v.y, -v.x * s + v.z * c);
}

// Utility: rotate vector around Z.
PVector rotateVectorAroundZ(PVector v, float angle) {
  float c = cos(angle);
  float s = sin(angle);
  return new PVector(v.x * c - v.y * s, v.x * s + v.y * c, v.z);
}

// Utility: drone body local vector to world.
PVector rotateDroneBodyLocalToWorld(PVector local) {
  PVector out = local == null ? new PVector(0, 0, 0) : local.copy();
  out = rotateVectorAroundZ(out, droneRoll);
  out = rotateVectorAroundX(out, dronePitch);
  out = rotateVectorAroundY(out, droneNavYaw);
  return out;
}

// Utility: drone downward sonar world origin.
PVector droneDownwardSonarWorldOrigin() {
  PVector bodyWorld = new PVector(droneNavX, droneNavY + droneY, droneNavZ);
  PVector sensorLocal = new PVector(DRONE_SONAR_X, DRONE_SONAR_Y, DRONE_SONAR_Z);
  return PVector.add(bodyWorld, rotateDroneBodyLocalToWorld(sensorLocal));
}

// Utility: drone downward sonar direction.
PVector droneDownwardSonarWorldDirection() {
  PVector dir = rotateDroneBodyLocalToWorld(new PVector(0, 1, 0));
  if (dir.magSq() < 0.0001f) dir = new PVector(0, 1, 0);
  dir.normalize();
  return dir;
}

// Checks whether live telemetry includes a manipulator sonar range field.
boolean hasCurrentManipulatorSonarTelemetry() {
  return latestSensors != null && (
    latestSensors.hasKey("hand_sonar_cm") ||
    latestSensors.hasKey("manipulator_sonar_cm") ||
    latestSensors.hasKey("sonar_cm") ||
    latestSensors.hasKey("range_cm"));
}

// Utility: current manipulator sonar distance.
float currentManipulatorSonarDistanceCm() {
  return getSensorFloat("hand_sonar_cm",
    getSensorFloat("manipulator_sonar_cm",
    getSensorFloat("sonar_cm",
    getSensorFloat("range_cm", sonarDistanceCm))));
}

// Checks whether live telemetry includes a vehicle radar or LiDAR range field.
boolean hasCurrentVehicleRadarTelemetry() {
  return latestSensors != null && (
    latestSensors.hasKey("vehicle_radar_cm") ||
    latestSensors.hasKey("vehicle_lidar_cm") ||
    latestSensors.hasKey("radar_cm") ||
    latestSensors.hasKey("lidar_cm") ||
    latestSensors.hasKey("lidar") ||
    latestSensors.hasKey("range_cm") ||
    latestSensors.hasKey("sonar_cm"));
}

// Utility: current vehicle radar distance.
float currentVehicleRadarDistanceCm() {
  return getSensorFloat("vehicle_radar_cm",
    getSensorFloat("vehicle_lidar_cm",
    getSensorFloat("radar_cm",
    getSensorFloat("lidar_cm",
    getSensorFloat("lidar",
    getSensorFloat("range_cm",
    getSensorFloat("sonar_cm", vehicleLidarDistanceCm)))))));
}

// Checks whether live telemetry includes a raw drone sonar range field.
boolean hasCurrentDroneRawSonarTelemetry() {
  return latestSensors != null && (
    latestSensors.hasKey("drone_scan_cm") ||
    latestSensors.hasKey("drone_sonar_down_cm") ||
    latestSensors.hasKey("sonar_down_cm") ||
    latestSensors.hasKey("scan_cm") ||
    latestSensors.hasKey("range_cm") ||
    latestSensors.hasKey("sonar_cm"));
}


// Utility: downward sonar distance to the ground plane.
float droneDownwardGroundDistanceCm(PVector sensorWorld, PVector dirWorld) {
  if (sensorWorld == null || dirWorld == null || dirWorld.magSq() < 0.0001f) return -1.0f;
  if (dirWorld.y <= 0.05f) return -1.0f;
  float sceneUnitsPerCm = max(0.0001f, robotSceneUnitsPerCm(ROBOT_MODE_DRONE));
  float groundDeltaY = GROUND_Y - sensorWorld.y;
  if (groundDeltaY < 0.0f) return -1.0f;
  float sceneDist = groundDeltaY / dirWorld.y;
  return sceneDist / sceneUnitsPerCm;
}

// Utility: add environment point.
void addEnvironmentPoint(ArrayList<PVector> list, PVector point) {
  if (point == null) return;
  float dedupeRadius = environmentPointRadiusForRobotMode(robotModeForEnvironmentPointList(list)) * 1.35f;
  for (int i = 0; i < list.size(); i++) {
    PVector existing = list.get(i);
    if (PVector.dist(existing, point) <= dedupeRadius) {
      existing.lerp(point, 0.30f);
      return;
    }
  }
  // Overwrite the oldest point (index 0 acts as a circular cursor) instead of
  // shifting the entire list with remove(0), which is O(n) on an ArrayList.
  if (list.size() >= ENVIRONMENT_MAX_POINTS_PER_MAP) {
    list.set(0, point.copy());
    // Rotate the replaced slot to the end so the oldest entry is always at [0].
    list.add(list.remove(0));
  } else {
    list.add(point.copy());
  }
  // Invalidate the environment-collision result cache so the next check
  // uses the updated point cloud rather than a stale result.
  invalidateEnvCollisionCache();
}

// Utility: add demo point box.
void addDemoPointBox(ArrayList<PVector> list, float cx, float cy, float cz, float sx, float sy, float sz, float spacing) {
  float step = max(8.0f, spacing);
  for (float x = -sx * 0.5f; x <= sx * 0.5f + 0.1f; x += step) {
    for (float y = -sy * 0.5f; y <= sy * 0.5f + 0.1f; y += step) {
      addEnvironmentPoint(list, new PVector(cx + x, cy + y, cz - sz * 0.5f));
      addEnvironmentPoint(list, new PVector(cx + x, cy + y, cz + sz * 0.5f));
    }
  }
  for (float z = -sz * 0.5f; z <= sz * 0.5f + 0.1f; z += step) {
    for (float y = -sy * 0.5f; y <= sy * 0.5f + 0.1f; y += step) {
      addEnvironmentPoint(list, new PVector(cx - sx * 0.5f, cy + y, cz + z));
      addEnvironmentPoint(list, new PVector(cx + sx * 0.5f, cy + y, cz + z));
    }
  }
}

// Utility: add demo point wall.
void addDemoPointWall(ArrayList<PVector> list, float cx, float cy, float cz, float sx, float sy, float spacing) {
  float step = max(10.0f, spacing);
  for (float x = -sx * 0.5f; x <= sx * 0.5f + 0.1f; x += step) {
    for (float y = -sy * 0.5f; y <= sy * 0.5f + 0.1f; y += step) {
      addEnvironmentPoint(list, new PVector(cx + x, cy + y, cz));
    }
  }
}

// Utility: add demo floor grid.
void addDemoFloorGrid(ArrayList<PVector> list, float halfSize, float spacing) {
  float step = max(18.0f, spacing);
  for (float x = -halfSize; x <= halfSize + 0.1f; x += step) {
    for (float z = -halfSize; z <= halfSize + 0.1f; z += step) {
      if (((int)((x + z + halfSize) / step)) % 4 == 0) {
        addEnvironmentPoint(list, new PVector(x, GROUND_Y, z));
      }
    }
  }
}

// Utility: keep environment point visuals above the ground plane.
PVector adjustEnvironmentPointSceneForGroundVisual(PVector scenePoint) {
  return adjustEnvironmentPointSceneForGroundVisual(scenePoint, currentEnvironmentPointRadius());
}

// Utility: keep environment point visuals above the ground plane.
PVector adjustEnvironmentPointSceneForGroundVisual(PVector scenePoint, float sphereRadius) {
  if (scenePoint == null) return null;
  PVector adjusted = scenePoint.copy();
  float minVisualY = -max(0.6f, sphereRadius);
  if (adjusted.y > minVisualY) adjusted.y = minVisualY;
  return adjusted;
}

// Utility: clamp world point so it never falls below the ground plane.
PVector clampEnvironmentPointWorldAboveGround(PVector worldPoint) {
  if (worldPoint == null) return null;
  PVector clamped = worldPoint.copy();
  if (clamped.y > GROUND_Y) clamped.y = GROUND_Y;
  return clamped;
}


// Ensures manipulator demo world.
void ensureManipulatorDemoWorld() {
  if (manipulatorDemoWorldReady) return;
  manipulatorDemoWorldPoints.clear();

  float half = demoWorldGroundHalfSizeForRobotMode(ROBOT_MODE_MANIPULATOR);
  float span = demoWorldReferenceSpanForRobotMode(ROBOT_MODE_MANIPULATOR);
  float grid = demoWorldGridSpacingForRobotMode(ROBOT_MODE_MANIPULATOR);
  float boxW = span * 0.28f;
  float boxH = span * 0.40f;
  float boxD = span * 0.26f;

  addDemoFloorGrid(manipulatorDemoWorldPoints, half, grid);
  addDemoPointBox(manipulatorDemoWorldPoints, span * 0.42f, -boxH * 0.50f, span * 0.42f, boxW, boxH, boxD, 10);
  addDemoPointBox(manipulatorDemoWorldPoints, -span * 0.46f, -boxH * 0.42f, span * 0.62f, boxW * 1.10f, boxH * 0.86f, boxD * 1.05f, 10);
  addDemoPointWall(manipulatorDemoWorldPoints, 0, -span * 0.36f, half * 0.82f, half * 1.18f, span * 0.46f, 12);
  manipulatorDemoWorldReady = true;
}

// Ensures vehicle demo world.
void ensureVehicleDemoWorld() {
  if (vehicleDemoWorldReady) return;
  vehicleDemoWorldPoints.clear();

  float half = demoWorldGroundHalfSizeForRobotMode(ROBOT_MODE_VEHICLE);
  float span = demoWorldReferenceSpanForRobotMode(ROBOT_MODE_VEHICLE);
  float grid = demoWorldGridSpacingForRobotMode(ROBOT_MODE_VEHICLE);

  addDemoFloorGrid(vehicleDemoWorldPoints, half, grid);
  addDemoPointWall(vehicleDemoWorldPoints, 0, -span * 0.30f, half * 0.84f, half * 1.24f, span * 0.44f, 12);
  addDemoPointBox(vehicleDemoWorldPoints, -span * 0.82f, -span * 0.24f, span * 0.56f, span * 0.40f, span * 0.28f, span * 0.34f, 12);
  addDemoPointBox(vehicleDemoWorldPoints, span * 0.80f, -span * 0.24f, span * 0.12f, span * 0.34f, span * 0.28f, span * 0.34f, 12);
  addDemoPointBox(vehicleDemoWorldPoints, 0, -span * 0.22f, -span * 0.82f, span * 0.66f, span * 0.24f, span * 0.30f, 12);
  vehicleDemoWorldReady = true;
}

// Ensures drone demo world.
void ensureDroneDemoWorld() {
  if (droneDemoWorldReady) return;
  droneDemoWorldPoints.clear();

  float half = demoWorldGroundHalfSizeForRobotMode(ROBOT_MODE_DRONE);
  float span = demoWorldReferenceSpanForRobotMode(ROBOT_MODE_DRONE);
  float grid = demoWorldGridSpacingForRobotMode(ROBOT_MODE_DRONE);

  addDemoFloorGrid(droneDemoWorldPoints, half, grid);

  float wall1W = span * 0.42f;
  float wall1H = span * 0.88f;
  float wall2W = span * 0.54f;
  float wall2H = span * 1.18f;
  float box1W = span * 0.36f;
  float box1H = span * 0.36f;
  float box1D = span * 0.36f;
  float box2W = span * 0.28f;
  float box2H = span * 0.52f;
  float box2D = span * 0.28f;

  addDemoPointWall(droneDemoWorldPoints, -span * 0.96f, GROUND_Y - wall1H * 0.5f, span * 0.24f, wall1W, wall1H, 12);
  addDemoPointWall(droneDemoWorldPoints, span * 0.92f, GROUND_Y - wall2H * 0.5f, span * 0.86f, wall2W, wall2H, 12);
  addDemoPointBox(droneDemoWorldPoints, 0, GROUND_Y - box1H * 0.5f, span * 0.62f, box1W, box1H, box1D, 12);
  addDemoPointBox(droneDemoWorldPoints, -span * 0.52f, GROUND_Y - box2H * 0.5f, -span * 0.30f, box2W, box2H, box2D, 12);
  droneDemoWorldReady = true;
}

// Ensures current environment demo world.
void ensureCurrentEnvironmentDemoWorld() {
  if (!shouldUseDemoWorldForCurrentRobot()) return;
  if (isVehicleSelected) {
    ensureVehicleDemoWorld();
  } else if (isDroneSelected) {
    ensureDroneDemoWorld();
  } else {
    ensureManipulatorDemoWorld();
  }
}

// Utility: simulate distance to demo world.
float simulateDistanceToDemoWorld(PVector origin, PVector dir, ArrayList<PVector> worldPoints, float maxDist) {
  if (origin == null || dir == null || worldPoints == null || worldPoints.isEmpty()) return -1;
  PVector ray = dir.copy();
  if (ray.magSq() < 0.0001f) return -1;
  ray.normalize();

  float best = maxDist + 1.0f;
  float rayHitRadius = max(currentEnvironmentPointRadius() * 1.8f, 8.0f);
  for (int i = 0; i < worldPoints.size(); i++) {
    PVector toPoint = PVector.sub(worldPoints.get(i), origin);
    float along = PVector.dot(toPoint, ray);
    if (along < environmentScanMinCm || along > maxDist) continue;
    PVector projected = PVector.mult(ray, along);
    float radial = PVector.sub(toPoint, projected).mag();
    if (radial <= rayHitRadius && along < best) {
      best = along;
    }
  }
  return best <= maxDist ? best : -1;
}



// Utility: world transform overlay x.
int worldTransformOverlayX() {
  int w = worldTransformOverlayW();
  return diagnosticsPanelVisible() ? 306 : max(314, (width - w) / 2);
}

// Utility: world transform overlay y.
int worldTransformOverlayY() {
  return 20;
}

// Utility: world transform overlay w.
int worldTransformOverlayW() {
  return 420;
}

// Utility: world transform overlay h.
int worldTransformOverlayH() {
  return 286;
}

// Checks whether a pointer is over the world-transform control panel itself.
boolean pointInWorldTransformOverlayPanel(int mx, int my) {
  return pointInRect(mx, my, worldTransformOverlayX(), worldTransformOverlayY(), worldTransformOverlayW(), worldTransformOverlayH());
}

// Utility: draw world transform overlay.
void drawWorldTransformOverlay() {
  if (!isWorldTransformOverlayActive()) return;

  int x = worldTransformOverlayX();
  int y = worldTransformOverlayY();
  int w = worldTransformOverlayW();
  int h = worldTransformOverlayH();
  int btnW = 90;
  int smallGap = 8;
  int adjustRowY = y + 112;
  int row1Y = y + 144;
  int row2Y = y + 176;
  int row3Y = y + 208;
  int row4Y = y + 240;
  int robotMode = currentRobotMode();

  hint(DISABLE_DEPTH_TEST);
  pushMatrix();
  resetMatrix();
  camera();
  pushStyle();
  noLights();

  fill(255, 252);
  stroke(185);
  strokeWeight(1.2f);
  rect(x, y, w, h, 12);

  fill(24);
  textAlign(LEFT, TOP);
  textSize(14);
  text(tr("World transform"), x + 14, y + 12);
  textSize(11);
  fill(80);
  String label = currentWorldSourceLabel();
  if (label == null || label.length() == 0) label = tr("Imported world");
  text(shortenWorldLabel(label, 44), x + 14, y + 34);
  text(tr("Scale x") + nfc(worldUserScaleForRobotMode(robotMode), 2)
    + "   " + tr("Pitch") + " " + nfc(worldUserPitchDegForRobotMode(robotMode), 1) + "°"
    + "   " + tr("Yaw") + " " + nfc(worldUserRotationDegForRobotMode(robotMode), 1) + "°"
    + "   " + tr("Roll") + " " + nfc(worldUserRollDegForRobotMode(robotMode), 1) + "°", x + 14, y + 54);
  text(tr("Offset X/Z ") + nfc(worldUserOffsetForRobotMode(robotMode).x, 1) + " / " + nfc(worldUserOffsetForRobotMode(robotMode).z, 1), x + 14, y + 68);

  drawMiniButton2D(x + 14, adjustRowY, btnW * 2 + smallGap, 24, tr("Adjust"), worldTransformMouseAdjustEnabled);
  drawMiniButton2D(x + 14 + 2 * (btnW + smallGap), adjustRowY, btnW, 24, tr("Recenter"), false);
  drawMiniButton2D(x + 14 + 3 * (btnW + smallGap), adjustRowY, btnW, 24, tr("Lock"), true);

  drawMiniButton2D(x + 14, row1Y, btnW, 24, tr("Scale -"), false);
  drawMiniButton2D(x + 14 + (btnW + smallGap), row1Y, btnW, 24, tr("Scale +"), false);
  drawMiniButton2D(x + 14 + 2 * (btnW + smallGap), row1Y, btnW, 24, tr("Left"), false);
  drawMiniButton2D(x + 14 + 3 * (btnW + smallGap), row1Y, btnW, 24, tr("Right"), false);

  drawMiniButton2D(x + 14, row2Y, btnW, 24, tr("Front"), false);
  drawMiniButton2D(x + 14 + (btnW + smallGap), row2Y, btnW, 24, tr("Back"), false);
  drawMiniButton2D(x + 14 + 2 * (btnW + smallGap), row2Y, btnW, 24, "Pitch -", false);
  drawMiniButton2D(x + 14 + 3 * (btnW + smallGap), row2Y, btnW, 24, "Pitch +", false);

  drawMiniButton2D(x + 14, row3Y, btnW, 24, "Yaw -", false);
  drawMiniButton2D(x + 14 + (btnW + smallGap), row3Y, btnW, 24, "Yaw +", false);
  drawMiniButton2D(x + 14 + 2 * (btnW + smallGap), row3Y, btnW, 24, "Roll -", false);
  drawMiniButton2D(x + 14 + 3 * (btnW + smallGap), row3Y, btnW, 24, "Roll +", false);

  drawMiniButton2D(x + 14, row4Y, btnW, 24, "Yaw 90°", false);
  drawMiniButton2D(x + 14 + (btnW + smallGap), row4Y, btnW, 24, tr("Reset"), false);

  popStyle();
  safePopMatrix("EnvironmentMapping.pde:drawWorldTransformOverlay");
  hint(ENABLE_DEPTH_TEST);
}

// Handles world transform overlay mouse pressed.
boolean handleWorldTransformOverlayMousePressed(int mx, int my) {
  if (!isWorldTransformOverlayActive()) return false;

  int x = worldTransformOverlayX();
  int y = worldTransformOverlayY();
  int btnW = 90;
  int smallGap = 8;
  int adjustRowY = y + 112;
  int row1Y = y + 144;
  int row2Y = y + 176;
  int row3Y = y + 208;
  int row4Y = y + 240;
  int robotMode = currentRobotMode();

  if (pointInWorldTransformOverlayPanel(mx, my)) {
    worldTransformOverlayPanelPointerDown = true;

    if (pointInRect(mx, my, x + 14, adjustRowY, btnW * 2 + smallGap, 24)) {
      worldTransformMouseAdjustEnabled = !worldTransformMouseAdjustEnabled;
      worldTransformOverlayDragging = false;
      updateMessage(worldTransformMouseAdjustEnabled
        ? tr("Mouse Adjust enabled: drag to move the world and use the wheel for world scale.")
        : tr("Mouse Adjust disabled: drag to orbit the camera and use the wheel for scene zoom."));
      return true;
    }
    if (pointInRect(mx, my, x + 14 + 2 * (btnW + smallGap), adjustRowY, btnW, 24)) { recenterImportedWorldForRobotMode(robotMode); return true; }
    if (pointInRect(mx, my, x + 14 + 3 * (btnW + smallGap), adjustRowY, btnW, 24)) { lockImportedWorldTransformForRobotMode(robotMode); return true; }

    if (pointInRect(mx, my, x + 14, row1Y, btnW, 24)) { scaleImportedWorldForRobotMode(robotMode, 1.0f / WORLD_TRANSFORM_SCALE_STEP); return true; }
    if (pointInRect(mx, my, x + 14 + (btnW + smallGap), row1Y, btnW, 24)) { scaleImportedWorldForRobotMode(robotMode, WORLD_TRANSFORM_SCALE_STEP); return true; }
    if (pointInRect(mx, my, x + 14 + 2 * (btnW + smallGap), row1Y, btnW, 24)) { translateImportedWorldForRobotMode(robotMode, -WORLD_TRANSFORM_NUDGE_STEP, 0); return true; }
    if (pointInRect(mx, my, x + 14 + 3 * (btnW + smallGap), row1Y, btnW, 24)) { translateImportedWorldForRobotMode(robotMode, WORLD_TRANSFORM_NUDGE_STEP, 0); return true; }

    if (pointInRect(mx, my, x + 14, row2Y, btnW, 24)) { translateImportedWorldForRobotMode(robotMode, 0, -WORLD_TRANSFORM_NUDGE_STEP); return true; }
    if (pointInRect(mx, my, x + 14 + (btnW + smallGap), row2Y, btnW, 24)) { translateImportedWorldForRobotMode(robotMode, 0, WORLD_TRANSFORM_NUDGE_STEP); return true; }
    if (pointInRect(mx, my, x + 14 + 2 * (btnW + smallGap), row2Y, btnW, 24)) { rotateImportedWorldForRobotMode(robotMode, 'X', -WORLD_TRANSFORM_ROTATION_STEP_DEG); return true; }
    if (pointInRect(mx, my, x + 14 + 3 * (btnW + smallGap), row2Y, btnW, 24)) { rotateImportedWorldForRobotMode(robotMode, 'X', WORLD_TRANSFORM_ROTATION_STEP_DEG); return true; }

    if (pointInRect(mx, my, x + 14, row3Y, btnW, 24)) { rotateImportedWorldForRobotMode(robotMode, 'Y', -WORLD_TRANSFORM_ROTATION_STEP_DEG); return true; }
    if (pointInRect(mx, my, x + 14 + (btnW + smallGap), row3Y, btnW, 24)) { rotateImportedWorldForRobotMode(robotMode, 'Y', WORLD_TRANSFORM_ROTATION_STEP_DEG); return true; }
    if (pointInRect(mx, my, x + 14 + 2 * (btnW + smallGap), row3Y, btnW, 24)) { rotateImportedWorldForRobotMode(robotMode, 'Z', -WORLD_TRANSFORM_ROTATION_STEP_DEG); return true; }
    if (pointInRect(mx, my, x + 14 + 3 * (btnW + smallGap), row3Y, btnW, 24)) { rotateImportedWorldForRobotMode(robotMode, 'Z', WORLD_TRANSFORM_ROTATION_STEP_DEG); return true; }

    if (pointInRect(mx, my, x + 14, row4Y, btnW, 24)) { rotateImportedWorldForRobotMode(robotMode, 'Y', 90.0f); return true; }
    if (pointInRect(mx, my, x + 14 + (btnW + smallGap), row4Y, btnW, 24)) { restoreImportedWorldForRobotMode(robotMode); return true; }
    return true;
  }

  if (worldTransformMouseAdjustEnabled && mx >= 300 && !pointInDiagnosticsGraphsWindow(mx, my) && !pointInDiagnosticsPanel(mx, my)) {
    worldTransformOverlayDragging = true;
    return true;
  }

  // In view mode, let the normal Processing mouse handlers orbit the camera.
  return false;
}

// Handles world transform overlay mouse dragged.
boolean handleWorldTransformOverlayMouseDragged(int mx, int my, int pmx, int pmy) {
  if (!isWorldTransformOverlayActive()) return false;
  if (worldTransformOverlayPanelPointerDown) return true;
  if (!worldTransformMouseAdjustEnabled || !worldTransformOverlayDragging) return false;
  PVector delta = worldTransformDragDelta(mx - pmx, my - pmy);
  translateImportedWorldForRobotMode(currentRobotMode(), delta.x, delta.z);
  return true;
}

// Handles world transform overlay mouse released.
void handleWorldTransformOverlayMouseReleased() {
  worldTransformOverlayDragging = false;
  worldTransformOverlayPanelPointerDown = false;
}

// Handles world transform overlay mouse wheel.
boolean handleWorldTransformOverlayMouseWheel(float scroll) {
  if (!isWorldTransformOverlayActive()) return false;
  if (pointInWorldTransformOverlayPanel(mouseX, mouseY)) return true;
  if (!worldTransformMouseAdjustEnabled) return false;
  float factor = pow(1.05f, -scroll);
  scaleImportedWorldForRobotMode(currentRobotMode(), factor);
  return true;
}

// Handles world transform overlay key pressed.
boolean handleWorldTransformOverlayKeyPressed() {
  if (!isWorldTransformOverlayActive()) return false;
  if (key == ENTER || key == RETURN || key == ESC) {
    if (key == ESC) key = 0;
    lockImportedWorldTransformForRobotMode(currentRobotMode());
    return true;
  }
  if (key == '+' || key == '=') {
    scaleImportedWorldForRobotMode(currentRobotMode(), WORLD_TRANSFORM_SCALE_STEP);
    return true;
  }
  if (key == '-') {
    scaleImportedWorldForRobotMode(currentRobotMode(), 1.0f / WORLD_TRANSFORM_SCALE_STEP);
    return true;
  }
  if (key == CODED) {
    if (keyCode == LEFT) { translateImportedWorldForRobotMode(currentRobotMode(), -WORLD_TRANSFORM_NUDGE_STEP, 0); return true; }
    if (keyCode == RIGHT) { translateImportedWorldForRobotMode(currentRobotMode(), WORLD_TRANSFORM_NUDGE_STEP, 0); return true; }
    if (keyCode == UP) { translateImportedWorldForRobotMode(currentRobotMode(), 0, -WORLD_TRANSFORM_NUDGE_STEP); return true; }
    if (keyCode == DOWN) { translateImportedWorldForRobotMode(currentRobotMode(), 0, WORLD_TRANSFORM_NUDGE_STEP); return true; }
  }
  return false;
}

// Updates environment scan.
void updateEnvironmentScan() {
  processPendingWorldImport();
  ensureCurrentEnvironmentDemoWorld();
  environmentAutoScan = currentEnvironmentAutoScanEnabled();
  if (!isManipulatorSelected || !environmentAutoScan) manipulatorSonarProbeWorld = null;
  if (!isDroneSelected || !environmentAutoScan) droneDownwardSonarProbeWorld = null;
  if (!environmentAutoScan) return;
  if (millis() - lastEnvironmentScanMillis < environmentScanIntervalMs) return;
  lastEnvironmentScanMillis = millis();

  if (isManipulatorSelected) {
    updateManipulatorEnvironmentScan();
  } else if (isVehicleSelected) {
    updateVehicleEnvironmentScan();
  } else if (isDroneSelected) {
    updateDroneEnvironmentScan();
  }
}

// Updates manipulator environment scan.
void updateManipulatorEnvironmentScan() {
  manipulatorSonarProbeWorld = null;

  PVector origin = manipulatorSonarHandOrigin();
  if (origin == null) return;
  PVector dir = manipulatorEnvironmentScanDirection();
  if (dir == null || dir.magSq() < 0.0001f) return;
  dir.normalize();

  float detectionLimitCm = manipulatorSonarDetectionMaxCm;
  float dist = -1.0f;
  boolean hasTrackedHit = false;

  if (shouldUseDemoWorldForCurrentRobot()) {
    ensureManipulatorDemoWorld();
    float simulated = simulateDistanceToDemoWorld(origin, dir, manipulatorDemoWorldPoints, detectionLimitCm);
    if (simulated > 0.0f) {
      dist = simulated;
      hasTrackedHit = true;
    }
  } else if (hasCurrentManipulatorSonarTelemetry()) {
    dist = currentManipulatorSonarDistanceCm();
    hasTrackedHit = dist > 0.0f;
  }

  // A scan hit exists only while the demo ray or live sonar tracks a target.
  // Readings at 4 m or beyond are treated as out of range.
  if (!hasTrackedHit || dist < environmentScanMinCm || dist >= detectionLimitCm) return;

  float sceneDist = dist * robotSceneUnitsPerCm(ROBOT_MODE_MANIPULATOR);
  PVector hitPoint = PVector.add(origin, PVector.mult(dir, sceneDist));
  manipulatorSonarProbeWorld = hitPoint.copy();
  addEnvironmentPoint(manipulatorEnvironmentPoints, hitPoint);
}

// Utility: manipulator sonar hand origin.
PVector manipulatorSonarHandOrigin() {
  return getGripperTipWorldPos();
}

// Utility: manipulator gripper-base world position.
PVector manipulatorGripperBaseWorldPos() {
  pushMatrix();
  applyManipulatorChainToGripperBase();
  PVector p = modelToWorld(0, 0, 0);
  safePopMatrix("EnvironmentMapping.pde:manipulatorGripperBaseWorldPos");
  return p;
}

// Utility: manipulator environment scan direction.
PVector manipulatorEnvironmentScanDirection() {
  PVector origin = manipulatorSonarHandOrigin();
  PVector base = manipulatorGripperBaseWorldPos();
  if (origin != null && base != null) {
    PVector toolForward = PVector.sub(origin, base);
    if (toolForward.magSq() >= 0.0001f) {
      toolForward.normalize();
      return toolForward;
    }
  }

  // Fallback for unusual startup frames before the hand transform is available.
  float yaw = radians(mpu1YawDeg - 90.0f);
  float pitch = radians(-mpu1PitchDeg);
  float cp = cos(pitch);
  PVector dir = new PVector(cos(yaw) * cp, sin(pitch), sin(yaw) * cp);
  if (dir.magSq() < 0.0001f) return new PVector(0, 0, -1);
  return dir;
}

// Draws the live sonar hit marker for the manipulator hand.
void drawManipulatorSonarProbe() {
  if (manipulatorSonarProbeWorld == null) return;

  float probeRadius = max(1.8f, currentEnvironmentPointRadius() * 1.15f);
  PVector hitScene = adjustEnvironmentPointSceneForGroundVisual(environmentPointToScene(manipulatorSonarProbeWorld), probeRadius);
  if (hitScene == null) return;

  pushMatrix();
  translate(hitScene.x, hitScene.y, hitScene.z);
  noStroke();
  fill(255, 90, 70, 185);
  sphere(probeRadius);
  safePopMatrix("EnvironmentMapping.pde:manipulatorSonarProbe");
}

// Draws the live radar hit marker for the vehicle.
void drawVehicleRadarProbe() {
  if (vehicleRadarProbeWorld == null) return;

  float probeRadius = max(1.8f, currentEnvironmentPointRadius() * 1.15f);
  PVector hitScene = adjustEnvironmentPointSceneForGroundVisual(environmentPointToScene(vehicleRadarProbeWorld), probeRadius);
  if (hitScene == null) return;

  pushMatrix();
  translate(hitScene.x, hitScene.y, hitScene.z);
  noStroke();
  fill(255, 170, 60, 185);
  sphere(probeRadius);
  safePopMatrix("EnvironmentMapping.pde:vehicleRadarProbe");
}

// Draws the live downward-sonar hit marker for the drone.
void drawDroneDownwardSonarProbe() {
  if (droneDownwardSonarProbeWorld == null) return;

  float probeRadius = max(1.8f, currentEnvironmentPointRadius() * 1.15f);
  PVector hitScene = adjustEnvironmentPointSceneForGroundVisual(environmentPointToScene(droneDownwardSonarProbeWorld), probeRadius);
  if (hitScene == null) return;

  pushMatrix();
  translate(hitScene.x, hitScene.y, hitScene.z);
  noStroke();
  fill(70, 210, 255, 185);
  sphere(probeRadius);
  safePopMatrix("EnvironmentMapping.pde:droneDownwardSonarProbe");
}

// Updates vehicle environment scan.
void updateVehicleEnvironmentScan() {
  vehicleRadarProbeWorld = null;
  if (!vehicleLidarScanEnabled) return;

  PVector sensorLocal = new PVector(0, -34, 8);
  PVector dirLocal = new PVector(sin(radians(vehicleLidarSpinDeg)), 0, cos(radians(vehicleLidarSpinDeg)));
  PVector sensorWorld = PVector.add(new PVector(vehicleNavX, GROUND_Y, vehicleNavZ), rotateLocalToWorld(sensorLocal, vehicleNavYaw));
  PVector dirWorld = rotateLocalToWorld(dirLocal, vehicleNavYaw);
  dirWorld.normalize();

  float detectionLimitCm = vehicleRadarCollisionMaxCm;
  float dist = -1.0f;
  boolean hasTrackedHit = false;

  if (shouldUseDemoWorldForCurrentRobot()) {
    ensureVehicleDemoWorld();
    float simulated = simulateDistanceToDemoWorld(sensorWorld, dirWorld, vehicleDemoWorldPoints, detectionLimitCm);
    if (simulated > 0.0f) {
      dist = simulated;
      hasTrackedHit = true;
      vehicleLidarDistanceCm = dist;
    }
  } else if (hasCurrentVehicleRadarTelemetry()) {
    dist = currentVehicleRadarDistanceCm();
    hasTrackedHit = dist > 0.0f;
  }

  // Collision spheres are created only for confirmed targets closer than 2 m.
  // Readings at 2 m or beyond are displayed as clear range and do not extend the collision map.
  if (!hasTrackedHit || dist < environmentScanMinCm || dist >= detectionLimitCm) return;

  float sceneDist = dist * robotSceneUnitsPerCm(ROBOT_MODE_VEHICLE);
  PVector hitPoint = clampEnvironmentPointWorldAboveGround(PVector.add(sensorWorld, PVector.mult(dirWorld, sceneDist)));
  vehicleRadarProbeWorld = hitPoint.copy();
  addEnvironmentPoint(vehicleEnvironmentPoints, hitPoint);
}

// Updates drone environment scan.
void updateDroneEnvironmentScan() {
  droneDownwardSonarProbeWorld = null;

  PVector sensorWorld = droneDownwardSonarWorldOrigin();
  PVector dirWorld = droneDownwardSonarWorldDirection();
  float detectionLimitCm = droneDownwardSonarDetectionMaxCm;
  float dist = -1.0f;
  boolean hasTrackedHit = false;

  if (shouldUseDemoWorldForCurrentRobot()) {
    ensureDroneDemoWorld();
    float simulated = simulateDistanceToDemoWorld(sensorWorld, dirWorld, droneDemoWorldPoints, detectionLimitCm);
    if (simulated > 0.0f) {
      dist = simulated;
      hasTrackedHit = true;
    } else {
      float groundDist = droneDownwardGroundDistanceCm(sensorWorld, dirWorld);
      if (groundDist > 0.0f) {
        dist = groundDist;
        hasTrackedHit = true;
      }
    }
    if (hasTrackedHit) droneScannerDistanceCm = dist;
  } else if (hasCurrentDroneRawSonarTelemetry()) {
    dist = droneScannerDistanceCm;
    hasTrackedHit = dist > 0.0f;
  }

  // A scan hit exists only while the demo ground/world or live sonar tracks a target.
  // Readings at 4 m or beyond are treated as out of range.
  if (!hasTrackedHit || dist < environmentScanMinCm || dist >= detectionLimitCm) return;

  float sceneDist = dist * robotSceneUnitsPerCm(ROBOT_MODE_DRONE);
  PVector hitPoint = clampEnvironmentPointWorldAboveGround(PVector.add(sensorWorld, PVector.mult(dirWorld, sceneDist)));
  droneDownwardSonarProbeWorld = hitPoint.copy();
  addEnvironmentPoint(droneEnvironmentPoints, hitPoint);
}

// Utility: manipulator collision against.
boolean manipulatorCollisionAgainst(ArrayList<PVector> points, ArrayList<PVector> samples, float hitRadius) {
  if (points == null || points.isEmpty()) return false;
  float hitRadiusSq = hitRadius * hitRadius;
  for (int i = 0; i < samples.size(); i++) {
    PVector s = samples.get(i);
    for (int j = 0; j < points.size(); j++) {
      PVector p = points.get(j);
      float dx = s.x - p.x, dy = s.y - p.y, dz = s.z - p.z;
      if (dx * dx + dy * dy + dz * dz <= hitRadiusSq) return true;
    }
  }
  return false;
}

// Vehicle helper for collision against.
boolean vehicleCollisionAgainst(ArrayList<PVector> points, float proposedX, float proposedZ, float proposedYaw) {
  if (points == null || points.isEmpty()) return false;
  for (int i = 0; i < points.size(); i++) {
    PVector world = points.get(i);
    PVector delta = PVector.sub(world, new PVector(proposedX, GROUND_Y, proposedZ));
    PVector local = rotateWorldToLocal(delta, proposedYaw);
    if (abs(local.x) <= VEH_TRACK_GAUGE * 0.5f - 10 && abs(local.z) <= VEH_BODY_LENGTH * 0.5f + 8 && abs(local.y) <= 40) {
      return true;
    }
  }
  return false;
}

// Drone helper for collision against.
boolean droneCollisionAgainst(ArrayList<PVector> points, float proposedX, float proposedY, float proposedZ) {
  if (points == null || points.isEmpty()) return false;
  PVector center = new PVector(proposedX, proposedY + droneY, proposedZ);
  for (int i = 0; i < points.size(); i++) {
    if (PVector.dist(center, points.get(i)) <= DRONE_BODY_DIM * 0.70f) return true;
  }
  return false;
}

// Cache for the environment-collision result. The check is O(samples×points) and
// can be expensive with large imported worlds. We reuse the last result while the
// arm pose has not changed meaningfully and the cache is still fresh.
boolean  envCollisionCacheValid  = false;
boolean  envCollisionCacheResult = false;
long     envCollisionCacheMs     = 0;
float[]  envCollisionCacheAngles = new float[7];
static final long   ENV_COLLISION_CACHE_TTL_MS  = 60;
static final float  ENV_COLLISION_CACHE_DEG_TOL = 0.8f;

boolean envCollisionCacheStillValid() {
  if (!envCollisionCacheValid) return false;
  if ((millis() - envCollisionCacheMs) > ENV_COLLISION_CACHE_TTL_MS) return false;
  for (int i = 0; i < min(envCollisionCacheAngles.length, angles.length); i++) {
    if (abs(angles[i] - envCollisionCacheAngles[i]) > ENV_COLLISION_CACHE_DEG_TOL) return false;
  }
  return true;
}

void invalidateEnvCollisionCache() {
  envCollisionCacheValid = false;
}

// Utility: check environment collision for current pose.
boolean checkEnvironmentCollisionForCurrentPose() {
  if (!environmentCollisionActiveForRobotMode(currentRobotMode())) return false;
  if (isVehicleSelected) return wouldVehicleEnvironmentCollide(getVehicleSceneX(), getVehicleSceneZ(), vehicleNavYaw);
  if (isDroneSelected) return wouldDroneEnvironmentCollide(getDroneSceneX(), getDroneSceneY(), getDroneSceneZ());

  if (envCollisionCacheStillValid()) return envCollisionCacheResult;

  ArrayList<PVector> samples = getManipulatorCollisionSamples();
  float hitRadius = currentEnvironmentPointRadius() * 1.15f;
  boolean result = manipulatorCollisionAgainst(manipulatorEnvironmentPoints, samples, hitRadius) ||
                   manipulatorCollisionAgainst(manipulatorImportedWorldPoints, samples, hitRadius);

  envCollisionCacheResult = result;
  envCollisionCacheMs     = millis();
  envCollisionCacheValid  = true;
  for (int i = 0; i < min(envCollisionCacheAngles.length, angles.length); i++) {
    envCollisionCacheAngles[i] = angles[i];
  }
  return result;
}

// Utility: would vehicle environment collide.
boolean wouldVehicleEnvironmentCollide(float proposedX, float proposedZ, float proposedYaw) {
  if (!environmentCollisionActiveForRobotMode(ROBOT_MODE_VEHICLE)) return false;
  return vehicleCollisionAgainst(vehicleEnvironmentPoints, proposedX, proposedZ, proposedYaw) ||
         vehicleCollisionAgainst(vehicleImportedWorldPoints, proposedX, proposedZ, proposedYaw);
}

// Signed clearance from the vehicle collision rectangle to the nearest map
// point. Negative means the vehicle is already intersecting the environment.
float vehicleEnvironmentClearanceAgainst(ArrayList<PVector> points, float x, float z, float yaw) {
  if (points == null || points.isEmpty()) return Float.MAX_VALUE;
  float halfX = max(1.0f, VEH_TRACK_GAUGE * 0.5f - 10.0f);
  float halfZ = max(1.0f, VEH_BODY_LENGTH * 0.5f + 8.0f);
  float nearest = Float.MAX_VALUE;
  PVector center = new PVector(x, GROUND_Y, z);

  for (int i = 0; i < points.size(); i++) {
    PVector world = points.get(i);
    if (world == null) continue;
    PVector local = rotateWorldToLocal(PVector.sub(world, center), yaw);
    if (abs(local.y) > 40.0f) continue;

    float outsideX = max(abs(local.x) - halfX, 0.0f);
    float outsideZ = max(abs(local.z) - halfZ, 0.0f);
    float outsideDistance = sqrt(outsideX * outsideX + outsideZ * outsideZ);
    boolean inside = abs(local.x) <= halfX && abs(local.z) <= halfZ;
    float signedDistance = inside
      ? -min(halfX - abs(local.x), halfZ - abs(local.z))
      : outsideDistance;
    nearest = min(nearest, signedDistance);
  }
  return nearest;
}

float vehicleEnvironmentClearance(float x, float z, float yaw) {
  return min(
    vehicleEnvironmentClearanceAgainst(vehicleEnvironmentPoints, x, z, yaw),
    vehicleEnvironmentClearanceAgainst(vehicleImportedWorldPoints, x, z, yaw)
  );
}

final float COLLISION_ESCAPE_CLEARANCE_EPSILON = 0.01f;

// Shared collision-recovery rule: a body may never enter/deepen a collision,
// but an already-colliding body may move when the candidate pose increases
// signed clearance by a measurable amount.
boolean collisionRecoveryMoveAllowed(boolean currentlyColliding, float currentClearance, float proposedClearance) {
  return currentlyColliding && proposedClearance > currentClearance + COLLISION_ESCAPE_CLEARANCE_EPSILON;
}

// Blocks entry/deeper penetration, but if the vehicle is already colliding it
// explicitly permits reverse/turn/escape commands that increase clearance.
boolean vehicleEnvironmentMoveAllowed(
  float currentX, float currentZ, float currentYaw,
  float proposedX, float proposedZ, float proposedYaw
) {
  if (!environmentCollisionActiveForRobotMode(ROBOT_MODE_VEHICLE)) return true;
  if (!wouldVehicleEnvironmentCollide(proposedX, proposedZ, proposedYaw)) return true;
  boolean currentlyColliding = wouldVehicleEnvironmentCollide(currentX, currentZ, currentYaw);
  float currentClearance = vehicleEnvironmentClearance(currentX, currentZ, currentYaw);
  float proposedClearance = vehicleEnvironmentClearance(proposedX, proposedZ, proposedYaw);
  return collisionRecoveryMoveAllowed(currentlyColliding, currentClearance, proposedClearance);
}

// Utility: would drone environment collide.
boolean wouldDroneEnvironmentCollide(float proposedX, float proposedY, float proposedZ) {
  if (!environmentCollisionActiveForRobotMode(ROBOT_MODE_DRONE)) return false;
  return droneCollisionAgainst(droneEnvironmentPoints, proposedX, proposedY, proposedZ) ||
         droneCollisionAgainst(droneImportedWorldPoints, proposedX, proposedY, proposedZ);
}

// Returns the signed clearance from the drone body to the closest environment
// point. Negative values mean the drone is already intersecting the map.
float droneEnvironmentClearanceAgainst(ArrayList<PVector> points, float x, float y, float z) {
  if (points == null || points.isEmpty()) return Float.MAX_VALUE;
  PVector center = new PVector(x, y + droneY, z);
  float nearest = Float.MAX_VALUE;
  for (int i = 0; i < points.size(); i++) {
    nearest = min(nearest, PVector.dist(center, points.get(i)));
  }
  return nearest - DRONE_BODY_DIM * 0.70f;
}

float droneEnvironmentClearance(float x, float y, float z) {
  return min(
    droneEnvironmentClearanceAgainst(droneEnvironmentPoints, x, y, z),
    droneEnvironmentClearanceAgainst(droneImportedWorldPoints, x, y, z)
  );
}

// Prevent entering/deepening a collision, but when the drone is already inside
// one, allow commands that increase clearance. This gives the operator a valid
// escape path instead of zeroing every planar command forever.
boolean droneEnvironmentMoveAllowed(
  float currentX, float currentY, float currentZ,
  float proposedX, float proposedY, float proposedZ
) {
  if (!environmentCollisionActiveForRobotMode(ROBOT_MODE_DRONE)) return true;
  if (!wouldDroneEnvironmentCollide(proposedX, proposedY, proposedZ)) return true;
  boolean currentlyColliding = wouldDroneEnvironmentCollide(currentX, currentY, currentZ);
  float currentClearance = droneEnvironmentClearance(currentX, currentY, currentZ);
  float proposedClearance = droneEnvironmentClearance(proposedX, proposedY, proposedZ);
  return collisionRecoveryMoveAllowed(currentlyColliding, currentClearance, proposedClearance);
}

// Returns a render stride that keeps large worlds responsive.
int environmentRenderStrideFor(ArrayList<PVector> points) {
  if (points == null || points.size() <= ENVIRONMENT_RENDER_POINT_BUDGET) return 1;
  return max(1, ceil(points.size() / float(ENVIRONMENT_RENDER_POINT_BUDGET)));
}

// Draws a point cloud without per-point sphere geometry.
void drawEnvironmentAsPoints(ArrayList<PVector> points, float pointRadius, int stride) {
  strokeWeight(max(2.0f, pointRadius * 1.1f));
  beginShape(POINTS);
  for (int i = 0; i < points.size(); i += max(1, stride)) {
    PVector drawPos = adjustEnvironmentPointSceneForGroundVisual(environmentPointToScene(points.get(i)), pointRadius);
    vertex(drawPos.x, drawPos.y, drawPos.z);
  }
  endShape();
  noStroke();
}

// Utility: draw current demo world.
void drawCurrentDemoWorld() {
  if (!shouldUseDemoWorldForCurrentRobot()) return;
  ensureCurrentEnvironmentDemoWorld();
  ArrayList<PVector> demoPoints = currentDemoWorldPointList();
  if (demoPoints.isEmpty()) return;

  pushStyle();
  noStroke();
  float pointRadius = currentEnvironmentPointRadius() * 0.52f;
  if (isVehicleSelected) {
    stroke(255, 170, 80, 55);
    fill(255, 170, 80, 55);
  } else if (isDroneSelected) {
    stroke(80, 210, 255, 55);
    fill(80, 210, 255, 55);
  } else {
    stroke(255, 120, 90, 45);
    fill(255, 120, 90, 45);
  }
  drawEnvironmentAsPoints(demoPoints, pointRadius, environmentRenderStrideFor(demoPoints));
  popStyle();
}

// Utility: trim environment point list uniformly.
void trimEnvironmentPointListUniformly(ArrayList<PVector> points, int maxPoints) {
  if (points == null || maxPoints <= 0 || points.size() <= maxPoints) return;
  ArrayList<PVector> trimmed = new ArrayList<PVector>();
  float stride = points.size() / float(maxPoints);
  float cursor = 0.0f;
  while (trimmed.size() < maxPoints) {
    int idx = constrain(round(cursor), 0, points.size() - 1);
    trimmed.add(points.get(idx).copy());
    cursor += stride;
  }
  points.clear();
  points.addAll(trimmed);
}

// Utility: draw environment point set.
void drawEnvironmentPointSet(ArrayList<PVector> points, float alphaScale, float radiusScale) {
  if (points == null || points.isEmpty()) return;
  float pointRadius = currentEnvironmentPointRadius() * radiusScale;
  // Use the fast GL POINTS path for large scanned clouds. Small live scan sets
  // may still use spheres for visibility, but imported worlds are rendered by
  // drawImportedWorldMeshDirect() and never reach this point-cloud renderer.
  boolean simpleRender = points.size() >= IMPORTED_WORLD_SIMPLE_RENDER_THRESHOLD;

  if (isVehicleSelected) {
    stroke(255, 150, 40, 180 * alphaScale);
    fill(255, 150, 40, 180 * alphaScale);
  } else if (isDroneSelected) {
    stroke(70, 200, 255, 170 * alphaScale);
    fill(70, 200, 255, 170 * alphaScale);
  } else {
    stroke(255, 90, 70, 180 * alphaScale);
    fill(255, 90, 70, 180 * alphaScale);
  }

  if (simpleRender) {
    drawEnvironmentAsPoints(points, pointRadius, environmentRenderStrideFor(points));
    return;
  }

  noStroke();
  int drawStride = environmentRenderStrideFor(points);
  for (int i = 0; i < points.size(); i += drawStride) {
    PVector drawPos = adjustEnvironmentPointSceneForGroundVisual(environmentPointToScene(points.get(i)), pointRadius);
    pushMatrix();
    translate(drawPos.x, drawPos.y, drawPos.z);
    sphere(pointRadius);
    safePopMatrix("EnvironmentMapping.pde:drawEnvironmentPointSet");
  }
}

// Draws the imported world as its original triangle mesh. Imported geometry is
// never converted to spheres for display; collision samples remain invisible.
void drawImportedWorldMeshDirect() {
  MeshData mesh = importedWorldMeshForRobotMode(currentRobotMode());
  if (mesh == null || mesh.isEmpty()) return;

  pushStyle();
  noStroke();
  if (isVehicleSelected) fill(235, 150, 70, 185);
  else if (isDroneSelected) fill(80, 190, 230, 180);
  else fill(210, 105, 85, 180);

  if (!mesh.triangles.isEmpty()) {
    beginShape(TRIANGLES);
    for (int i = 0; i < mesh.triangles.size(); i++) {
      MeshTriangle tri = mesh.triangles.get(i);
      PVector a = environmentPointToScene(tri.a);
      PVector b = environmentPointToScene(tri.b);
      PVector c = environmentPointToScene(tri.c);
      PVector ab = PVector.sub(b, a);
      PVector ac = PVector.sub(c, a);
      PVector n = ab.cross(ac);
      if (n.magSq() > 0.000001f) {
        n.normalize();
        normal(n.x, n.y, n.z);
      }
      vertex(a.x, a.y, a.z);
      vertex(b.x, b.y, b.z);
      vertex(c.x, c.y, c.z);
    }
    endShape();
  }

  // Point-only source files are rendered as GL points, never as spheres.
  if (!mesh.loosePoints.isEmpty()) {
    strokeWeight(max(2.0f, currentEnvironmentPointRadius() * 0.85f));
    if (isVehicleSelected) stroke(235, 150, 70, 210);
    else if (isDroneSelected) stroke(80, 190, 230, 205);
    else stroke(210, 105, 85, 205);
    beginShape(POINTS);
    for (int i = 0; i < mesh.loosePoints.size(); i++) {
      PVector p = environmentPointToScene(mesh.loosePoints.get(i));
      vertex(p.x, p.y, p.z);
    }
    endShape();
  }
  popStyle();
}

// Utility: draw current environment map.
void drawCurrentEnvironmentMap() {
  drawCurrentDemoWorld();
  MeshData importedMesh = importedWorldMeshForRobotMode(currentRobotMode());
  ArrayList<PVector> scanned = currentEnvironmentPointList();
  if ((importedMesh == null || importedMesh.isEmpty()) && (scanned == null || scanned.isEmpty())) return;

  drawImportedWorldMeshDirect();
  pushStyle();
  noStroke();
  drawEnvironmentPointSet(scanned, 1.0f, 1.0f);
  popStyle();
}

// Utility: environment point to scene.
PVector environmentPointToScene(PVector worldPoint) {
  if (isVehicleSelected) {
    PVector delta = PVector.sub(worldPoint, new PVector(vehicleNavX, GROUND_Y, vehicleNavZ));
    return rotateWorldToLocal(delta, vehicleNavYaw);
  }
  if (isDroneSelected) {
    PVector delta = PVector.sub(worldPoint, new PVector(droneNavX, GROUND_Y, droneNavZ));
    return rotateWorldToLocal(delta, droneNavYaw);
  }
  return worldPoint.copy();
}

// Utility: draw environment collision points.
void drawEnvironmentCollisionPoints() {
  drawCurrentEnvironmentMap();
}

// Returns manipulator collision samples.
ArrayList<PVector> getManipulatorCollisionSamples() {
  ArrayList<PVector> pts = new ArrayList<PVector>();

  PVector base = new PVector(0, baseBlockYOffset, 0);
  pts.add(base);

  pushMatrix();
  translate(0, baseBlockYOffset, 0);
  rotateY(radians(getBaseModelYawDeg()));
  translate(0, -(baseBlockH / 2.0f), 0);
  PVector shoulder = modelToWorld(0, 0, 0);
  pts.add(shoulder);

  rotateZ(radians(getUpperArmModelZDeg()));
  PVector upperMid = modelToWorld(0, -upperArmH * 0.5f, 0);
  PVector elbow = modelToWorld(0, -upperArmH, 0);
  pts.add(upperMid);
  pts.add(elbow);

  rotateZ(radians(getForearmModelZDeg()));
  PVector foreMid = modelToWorld(0, -forearmH * 0.5f, 0);
  PVector wrist = modelToWorld(0, -forearmH, 0);
  pts.add(foreMid);
  pts.add(wrist);

  rotateZ(radians(getWristVerticalModelZDeg()));
  PVector wristMid = modelToWorld(0, -wristVerticalH * 0.5f, 0);
  PVector gripperBase = modelToWorld(0, -wristVerticalH, 0);
  pts.add(wristMid);
  pts.add(gripperBase);
  safePopMatrix("EnvironmentMapping.pde:2216");

  PVector gripTip = getGripperTipWorldPos();
  PVector fingerR = getFingerTipWorldPos(true);
  PVector fingerL = getFingerTipWorldPos(false);
  if (gripTip != null) pts.add(gripTip);
  if (fingerR != null) pts.add(fingerR);
  if (fingerL != null) pts.add(fingerL);

  return pts;
}
