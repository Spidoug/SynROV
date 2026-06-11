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
Object worldImportLock = new Object();
ImportMeshResult pendingWorldImportResult = null;

boolean worldTransformOverlayActive = false;
boolean worldTransformOverlayDragging = false;
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
final float WORLD_TRANSFORM_SCALE_STEP = 1.06f;
final float WORLD_TRANSFORM_NUDGE_STEP = 12.0f;

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

// Utility: demo world scale for robot mode.
float demoWorldScaleForRobotMode(int robotMode) {
  return 1.0f;
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

// Utility: current demo world enabled.
boolean currentDemoWorldEnabled() {
  return demoWorldEnabledForRobotMode(currentRobotMode());
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

void addWorldUserRotationDegForRobotMode(int robotMode, float deltaDeg) {
  if (robotMode == ROBOT_MODE_VEHICLE) vehicleWorldUserRotationDeg += deltaDeg;
  else if (robotMode == ROBOT_MODE_DRONE) droneWorldUserRotationDeg += deltaDeg;
  else manipulatorWorldUserRotationDeg += deltaDeg;
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

// Resets world transform for robot mode.
void resetWorldTransformForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) {
    vehicleWorldUserScale = 1.0f;
    vehicleWorldUserOffset.set(0, 0, 0);
    vehicleWorldUserRotationDeg = 0.0f;
  } else if (robotMode == ROBOT_MODE_DRONE) {
    droneWorldUserScale = 1.0f;
    droneWorldUserOffset.set(0, 0, 0);
    droneWorldUserRotationDeg = 0.0f;
  } else {
    manipulatorWorldUserScale = 1.0f;
    manipulatorWorldUserOffset.set(0, 0, 0);
    manipulatorWorldUserRotationDeg = 0.0f;
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
    && !importedWorldPointListForRobotMode(worldTransformOverlayRobotMode).isEmpty();
}

// Utility: activate world transform overlay.
void activateWorldTransformOverlay(int robotMode) {
  worldTransformOverlayActive = true;
  worldTransformOverlayDragging = false;
  worldTransformOverlayRobotMode = robotMode;
  setWorldTransformReferenceYawForRobotMode(robotMode, cameraRotationY);
}

// Utility: dismiss world transform overlay.
void dismissWorldTransformOverlay() {
  worldTransformOverlayActive = false;
  worldTransformOverlayDragging = false;
}

// Utility: imported world centroid for robot mode.
PVector importedWorldCentroidForRobotMode(int robotMode) {
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  if (pts == null || pts.isEmpty()) return robotWorldAnchor(robotMode).copy();
  PVector c = new PVector();
  for (int i = 0; i < pts.size(); i++) c.add(pts.get(i));
  c.div(max(1, pts.size()));
  return c;
}

// Utility: translate imported world for robot mode.
void translateImportedWorldForRobotMode(int robotMode, float dx, float dz) {
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  if (pts == null || pts.isEmpty()) return;
  for (int i = 0; i < pts.size(); i++) {
    PVector p = pts.get(i);
    p.x += dx;
    p.z += dz;
  }
  addWorldUserOffsetForRobotMode(robotMode, dx, dz);
}

// Utility: scale imported world for robot mode.
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

void rotateImportedWorldForRobotMode(int robotMode, float deltaDeg) {
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  if (pts == null || pts.isEmpty()) return;
  PVector pivot = importedWorldCentroidForRobotMode(robotMode);
  float rad = radians(deltaDeg);
  float c = cos(rad);
  float s = sin(rad);
  for (int i = 0; i < pts.size(); i++) {
    PVector p = pts.get(i);
    float dx = p.x - pivot.x;
    float dz = p.z - pivot.z;
    p.x = pivot.x + dx * c + dz * s;
    p.z = pivot.z - dx * s + dz * c;
  }
  addWorldUserRotationDegForRobotMode(robotMode, deltaDeg);
}

void scaleImportedWorldForRobotMode(int robotMode, float factor) {
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  if (pts == null || pts.isEmpty()) return;
  float safeFactor = constrain(factor, 0.2f, 6.0f);
  PVector pivot = importedWorldCentroidForRobotMode(robotMode);
  for (int i = 0; i < pts.size(); i++) {
    PVector p = pts.get(i);
    p.x = pivot.x + (p.x - pivot.x) * safeFactor;
    p.y = GROUND_Y + (p.y - GROUND_Y) * safeFactor;
    p.z = pivot.z + (p.z - pivot.z) * safeFactor;
  }
  multiplyWorldUserScaleForRobotMode(robotMode, safeFactor);
  densifyImportedWorldForRobotMode(robotMode, safeFactor);
}

// Utility: recenter imported world for robot mode.
void recenterImportedWorldForRobotMode(int robotMode) {
  ArrayList<PVector> pts = importedWorldPointListForRobotMode(robotMode);
  if (pts == null || pts.isEmpty()) return;
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

// Resets current demo world.
void resetCurrentDemoWorld() {
  resetDemoWorldForRobotMode(currentRobotMode());
}

// Checks whether imported world for robot mode.
boolean hasImportedWorldForRobotMode(int robotMode) {
  return !importedWorldPointListForRobotMode(robotMode).isEmpty();
}

// Compass helper for heading degrees for robot mode.
float compassHeadingDegForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) {
    return systemReady ? vehicleHeadingTelemetryDeg : degrees(vehicleNavYaw);
  }
  if (robotMode == ROBOT_MODE_DRONE) {
    return systemReady ? droneHeadingTelemetryDeg : degrees(droneNavYaw);
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
  importedWorldPointListForRobotMode(robotMode).clear();
  setWorldSourceLabelForRobotMode(robotMode, "");
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

// Sets current world source label.
void setCurrentWorldSourceLabel(String label) {
  setWorldSourceLabelForRobotMode(currentRobotMode(), label);
}

// Utility: imported world units to cm for robot mode.
float importedWorldUnitsToCmForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleImportedWorldUnitsToCm;
  if (robotMode == ROBOT_MODE_DRONE) return droneImportedWorldUnitsToCm;
  return manipulatorImportedWorldUnitsToCm;
}

// Utility: current imported world units to cm.
float currentImportedWorldUnitsToCm() {
  return importedWorldUnitsToCmForRobotMode(currentRobotMode());
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

// Sets current imported world units to cm.
void setCurrentImportedWorldUnitsToCm(float value) {
  setImportedWorldUnitsToCmForRobotMode(currentRobotMode(), value);
}

// Utility: imported world scale factor for robot mode.
float importedWorldScaleFactorForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return vehicleImportedWorldScale;
  if (robotMode == ROBOT_MODE_DRONE) return droneImportedWorldScale;
  return manipulatorImportedWorldScale;
}

// Utility: current imported world scale factor.
float currentImportedWorldScaleFactor() {
  return importedWorldScaleFactorForRobotMode(currentRobotMode());
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

// Sets current imported world scale factor.
void setCurrentImportedWorldScaleFactor(float value) {
  setImportedWorldScaleFactorForRobotMode(currentRobotMode(), value);
}

// Utility: robot scene units per cm.
float robotSceneUnitsPerCm(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) return MODEL_SCALE_3D;
  return 1.0f;
}

// Utility: current robot scene units per cm.
float currentRobotSceneUnitsPerCm() {
  return robotSceneUnitsPerCm(currentRobotMode());
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

// Utility: current robot reference span scene.
float currentRobotReferenceSpanScene() {
  return robotReferenceSpanScene(currentRobotMode());
}

// Utility: robot world anchor.
PVector robotWorldAnchor(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return new PVector(vehicleNavX, GROUND_Y, vehicleNavZ);
  if (robotMode == ROBOT_MODE_DRONE) return new PVector(droneNavX, GROUND_Y, droneNavZ);
  return new PVector(0, GROUND_Y, 0);
}

// Utility: current robot world anchor.
PVector currentRobotWorldAnchor() {
  return robotWorldAnchor(currentRobotMode());
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

// Utility: imported world point count for robot mode.
int importedWorldPointCountForRobotMode(int robotMode) {
  return importedWorldPointListForRobotMode(robotMode).size();
}

// Utility: total environment point count for robot mode.
int totalEnvironmentPointCountForRobotMode(int robotMode) {
  return environmentPointCountForRobotMode(robotMode) + importedWorldPointCountForRobotMode(robotMode);
}

// Utility: current environment point count.
int currentEnvironmentPointCount() {
  return totalEnvironmentPointCountForRobotMode(currentRobotMode());
}

// Utility: current environment world summary.
String currentEnvironmentWorldSummary() {
  int robotMode = currentRobotMode();
  String label = worldSourceLabelForRobotMode(robotMode);
  int scanCount = environmentPointCountForRobotMode(robotMode);
  int worldCount = importedWorldPointCountForRobotMode(robotMode);

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

// Utility: commit imported mesh to robot environment.
void commitImportedMeshToRobotEnvironment(MeshData mesh, String sourceLabel, String ext, int robotMode) {
  if (mesh == null || mesh.isEmpty() || mesh.minBound == null || mesh.maxBound == null) {
    updateMessage(tr("Falha ao importar mundo: geometria não encontrada.", "World import failed: geometry not found."));
    return;
  }

  PVector rawSize = PVector.sub(mesh.maxBound, mesh.minBound);
  float rawLargest = max(rawSize.x, max(rawSize.y, rawSize.z));
  if (rawLargest <= 0.0001f) {
    updateMessage(tr("Falha ao importar mundo: limites inválidos.", "World import failed: invalid bounds."));
    return;
  }

  // Normalize every imported world to centimeters first, then project those
  // centimeters into the active robot scene scale.
  float unitsToCm = chooseImportedUnitsToCmScale(rawLargest, ext, robotMode);
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
    updateMessage(tr("Falha ao importar mundo: dados de cena vazios.", "World import failed: empty scene data."));
    return;
  }

  PVector anchor = robotWorldAnchor(robotMode);
  float centerX = (minScene.x + maxScene.x) * 0.5f;
  float centerZ = (minScene.z + maxScene.z) * 0.5f;
  PVector offset = new PVector(anchor.x - centerX, GROUND_Y - maxScene.y, anchor.z - centerZ);

  ArrayList<PVector> dst = importedWorldPointListForRobotMode(robotMode);
  dst.clear();
  resetDemoWorldForRobotMode(robotMode);
  disableBootDemoWorlds();

  float sampleSpacing = max(environmentPointRadiusForRobotMode(robotMode) * 0.90f, max(2.4f, robotReferenceSpanScene(robotMode) * 0.018f));
  for (int i = 0; i < stagedLoose.size(); i++) {
    addEnvironmentPoint(dst, PVector.add(stagedLoose.get(i), offset));
  }
  int triangleStride = max(1, ceil(stagedTriangles.size() / float(max(1, IMPORTED_WORLD_MAX_POINTS))));
  for (int i = 0; i < stagedTriangles.size(); i += triangleStride) {
    MeshTriangle tri = stagedTriangles.get(i);
    sampleTriangleToEnvironment(dst, PVector.add(tri.a, offset), PVector.add(tri.b, offset), PVector.add(tri.c, offset), sampleSpacing);
  }
  trimEnvironmentPointListUniformly(dst, IMPORTED_WORLD_MAX_POINTS);

  setWorldSourceLabelForRobotMode(robotMode, sourceLabel);
  setWorldCompassNorthReferenceForRobotMode(robotMode, compassHeadingDegForRobotMode(robotMode));
  resetWorldTransformForRobotMode(robotMode);
  activateWorldTransformOverlay(robotMode);
  environmentCollisionEnabled = true;
  invalidateEnvCollisionCache(); // new world loaded: force a fresh collision check
  int scanCount = environmentPointCountForRobotMode(robotMode);
  updateMessage(tr("Mundo carregado: ", "World loaded: ") + sourceLabel
    + " | world pts " + dst.size()
    + " | scan pts " + scanCount
    + " | units→cm x" + nf(unitsToCm, 1, 2)
    + " | robot x" + nf(robotScale, 1, 2)
    + " | final x" + nf(finalSceneScale, 1, 2)
    + " | adjust and lock");
}

// Utility: commit imported mesh to current environment.
void commitImportedMeshToCurrentEnvironment(MeshData mesh, String sourceLabel, String ext) {
  commitImportedMeshToRobotEnvironment(mesh, sourceLabel, ext, currentRobotMode());
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

// Loads world map from file.
boolean loadWorldMapFromFile(File file) {
  ImportMeshResult result = buildImportMeshResult(file, currentRobotMode());
  if (result.errorMessage != null && result.errorMessage.length() > 0) {
    updateMessage(result.errorMessage);
    return false;
  }
  commitImportedMeshToRobotEnvironment(result.mesh, result.sourceLabel, result.ext, result.robotMode);
  return true;
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
    updateMessage(result.errorMessage);
    return;
  }
  commitImportedMeshToRobotEnvironment(result.mesh, result.sourceLabel, result.ext, result.robotMode);
}

// Utility: queue async world import.
void queueAsyncWorldImport(final File file, final int robotMode) {
  if (worldImportBusy) {
    updateMessage(tr("Importação de mundo já está em andamento.", "World import already in progress."));
    return;
  }

  worldImportBusy = true;
  updateMessage(file == null ? "World import cancelled." : "Importing world: " + file.getName());

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

// Utility: on world map file selected.
void onWorldMapFileSelected(File selection) {
  worldImportDialogOpen = false;
  if (selection == null) {
    updateMessage(tr("Importação de mundo cancelada.", "World import cancelled."));
    return;
  }
  queueAsyncWorldImport(selection, queuedWorldImportRobotMode);
}

// Loads world map from windows dialog.
void loadWorldMapFromWindowsDialog() {
  if (worldImportDialogOpen || worldImportBusy) {
    updateMessage(tr("Importação de mundo já está em andamento.", "World import already in progress."));
    return;
  }
  queuedWorldImportRobotMode = currentRobotMode();
  worldImportDialogOpen = true;
  selectInput("Select 3D world / environment map", "onWorldMapFileSelected");
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

// Utility: current downward sonar fallback distance.
float droneDownwardSonarFallbackCm(PVector sensorWorld) {
  float sceneUnitsPerCm = max(0.0001f, robotSceneUnitsPerCm(ROBOT_MODE_DRONE));
  float geometricHeightCm = max(0.0f, (GROUND_Y - sensorWorld.y) / sceneUnitsPerCm);
  if (droneScannerDistanceCm > 0.0f) return droneScannerDistanceCm;
  if (hasCurrentDroneRawSonarTelemetry()) return -1.0f;
  if (sonarDistanceCm > 0.0f) return sonarDistanceCm;
  return geometricHeightCm;
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
  return diagnosticsPanelVisible() ? 306 : max(314, (width - 340) / 2);
}

// Utility: world transform overlay y.
int worldTransformOverlayY() {
  return 20;
}

// Utility: world transform overlay w.
int worldTransformOverlayW() {
  return 340;
}

// Utility: world transform overlay h.
int worldTransformOverlayH() {
  return 214;
}

// Utility: draw world transform overlay.
void drawWorldTransformOverlay() {
  if (!isWorldTransformOverlayActive()) return;

  int x = worldTransformOverlayX();
  int y = worldTransformOverlayY();
  int w = worldTransformOverlayW();
  int h = worldTransformOverlayH();
  int btnW = 72;
  int smallGap = 8;
  int row1Y = y + 78;
  int row2Y = y + 110;
  int row3Y = y + 142;

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
  text(tr("Transformação do mundo", "World transform"), x + 14, y + 12);
  textSize(11);
  fill(80);
  String label = currentWorldSourceLabel();
  if (label == null || label.length() == 0) label = tr("Mundo importado", "Imported world");
  text(shortenWorldLabel(label, 34), x + 14, y + 34);
  text(tr("Arraste na visualização 3D para mover. A roda do mouse ou +/- ajusta a escala. Enter fixa a referência.", "Drag in the 3D view to move. Mouse wheel or +/- to scale. Enter locks."), x + 14, y + 50);
  text((isUiPortuguese() ? "Escala x" : "Scale x") + nfc(worldUserScaleForRobotMode(currentRobotMode()), 2)
    + (isUiPortuguese() ? "   Rot " : "   Rot ") + nfc(worldUserRotationDegForRobotMode(currentRobotMode()), 1) + "°   " + (isUiPortuguese() ? "Offset X/Z " : "Offset X/Z ") + nfc(worldUserOffsetForRobotMode(currentRobotMode()).x, 1) + " / " + nfc(worldUserOffsetForRobotMode(currentRobotMode()).z, 1), x + 14, y + 64);

  drawMiniButton2D(x + 14, row1Y, btnW, 24, tr("Escala -", "Scale -"), false);
  drawMiniButton2D(x + 14 + (btnW + smallGap), row1Y, btnW, 24, tr("Escala +", "Scale +"), false);
  drawMiniButton2D(x + 14 + 2 * (btnW + smallGap), row1Y, btnW, 24, tr("Recentralizar", "Recenter"), false);
  drawMiniButton2D(x + 14 + 3 * (btnW + smallGap), row1Y, btnW, 24, tr("Fixar", "Lock"), true);

  drawMiniButton2D(x + 14, row2Y, btnW, 24, tr("Esquerda", "Left"), false);
  drawMiniButton2D(x + 14 + (btnW + smallGap), row2Y, btnW, 24, tr("Direita", "Right"), false);
  drawMiniButton2D(x + 14 + 2 * (btnW + smallGap), row2Y, btnW, 24, tr("Frente", "Front"), false);
  drawMiniButton2D(x + 14 + 3 * (btnW + smallGap), row2Y, btnW, 24, tr("Trás", "Back"), false);

  drawMiniButton2D(x + 14, row3Y, btnW, 24, tr("Rot -", "Rot -"), false);
  drawMiniButton2D(x + 14 + (btnW + smallGap), row3Y, btnW, 24, tr("Rot +", "Rot +"), false);
  drawMiniButton2D(x + 14 + 2 * (btnW + smallGap), row3Y, btnW, 24, tr("Rot 90", "Rot 90"), false);
  drawMiniButton2D(x + 14 + 3 * (btnW + smallGap), row3Y, btnW, 24, tr("Resetar", "Reset"), false);

  popStyle();
  safePopMatrix("EnvironmentMapping.pde:1866");
  hint(ENABLE_DEPTH_TEST);
}

// Handles world transform overlay mouse pressed.
boolean handleWorldTransformOverlayMousePressed(int mx, int my) {
  if (!isWorldTransformOverlayActive()) return false;

  int x = worldTransformOverlayX();
  int y = worldTransformOverlayY();
  int w = worldTransformOverlayW();
  int btnW = 72;
  int smallGap = 8;
  int row1Y = y + 78;
  int row2Y = y + 110;
  int row3Y = y + 142;

  if (pointInRect(mx, my, x + 14, row1Y, btnW, 24)) { scaleImportedWorldForRobotMode(currentRobotMode(), 1.0f / WORLD_TRANSFORM_SCALE_STEP); return true; }
  if (pointInRect(mx, my, x + 14 + (btnW + smallGap), row1Y, btnW, 24)) { scaleImportedWorldForRobotMode(currentRobotMode(), WORLD_TRANSFORM_SCALE_STEP); return true; }
  if (pointInRect(mx, my, x + 14 + 2 * (btnW + smallGap), row1Y, btnW, 24)) { recenterImportedWorldForRobotMode(currentRobotMode()); return true; }
  if (pointInRect(mx, my, x + 14 + 3 * (btnW + smallGap), row1Y, btnW, 24)) { setWorldTransformReferenceYawForRobotMode(currentRobotMode(), cameraRotationY); dismissWorldTransformOverlay(); updateMessage(tr("Referência de transformação do mundo fixada.", "World transform reference locked.")); return true; }

  if (pointInRect(mx, my, x + 14, row2Y, btnW, 24)) { translateImportedWorldForRobotMode(currentRobotMode(), -WORLD_TRANSFORM_NUDGE_STEP, 0); return true; }
  if (pointInRect(mx, my, x + 14 + (btnW + smallGap), row2Y, btnW, 24)) { translateImportedWorldForRobotMode(currentRobotMode(), WORLD_TRANSFORM_NUDGE_STEP, 0); return true; }
  if (pointInRect(mx, my, x + 14 + 2 * (btnW + smallGap), row2Y, btnW, 24)) { translateImportedWorldForRobotMode(currentRobotMode(), 0, -WORLD_TRANSFORM_NUDGE_STEP); return true; }
  if (pointInRect(mx, my, x + 14 + 3 * (btnW + smallGap), row2Y, btnW, 24)) { translateImportedWorldForRobotMode(currentRobotMode(), 0, WORLD_TRANSFORM_NUDGE_STEP); return true; }

  if (pointInRect(mx, my, x + 14, row3Y, btnW, 24)) { rotateImportedWorldForRobotMode(currentRobotMode(), -7.5f); return true; }
  if (pointInRect(mx, my, x + 14 + (btnW + smallGap), row3Y, btnW, 24)) { rotateImportedWorldForRobotMode(currentRobotMode(), 7.5f); return true; }
  if (pointInRect(mx, my, x + 14 + 2 * (btnW + smallGap), row3Y, btnW, 24)) { rotateImportedWorldForRobotMode(currentRobotMode(), 90.0f); return true; }
  if (pointInRect(mx, my, x + 14 + 3 * (btnW + smallGap), row3Y, btnW, 24)) { resetWorldTransformForRobotMode(currentRobotMode()); recenterImportedWorldForRobotMode(currentRobotMode()); return true; }

  if (mx >= 300 && !pointInDiagnosticsGraphsWindow(mx, my) && !pointInDiagnosticsPanel(mx, my)) {
    worldTransformOverlayDragging = true;
    return true;
  }
  return true;
}

// Handles world transform overlay mouse dragged.
boolean handleWorldTransformOverlayMouseDragged(int mx, int my, int pmx, int pmy) {
  if (!isWorldTransformOverlayActive() || !worldTransformOverlayDragging) return false;
  PVector delta = worldTransformDragDelta(mx - pmx, my - pmy);
  translateImportedWorldForRobotMode(currentRobotMode(), delta.x, delta.z);
  return true;
}

// Handles world transform overlay mouse released.
void handleWorldTransformOverlayMouseReleased() {
  worldTransformOverlayDragging = false;
}

// Handles world transform overlay mouse wheel.
boolean handleWorldTransformOverlayMouseWheel(float scroll) {
  if (!isWorldTransformOverlayActive()) return false;
  float factor = pow(1.05f, -scroll);
  scaleImportedWorldForRobotMode(currentRobotMode(), factor);
  return true;
}

// Handles world transform overlay key pressed.
boolean handleWorldTransformOverlayKeyPressed() {
  if (!isWorldTransformOverlayActive()) return false;
  if (key == ENTER || key == RETURN || key == ESC) {
    if (key == ESC) key = 0;
    setWorldTransformReferenceYawForRobotMode(currentRobotMode(), cameraRotationY);
    dismissWorldTransformOverlay();
    updateMessage(tr("Referência de transformação do mundo fixada.", "World transform reference locked."));
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
  if (!environmentCollisionEnabled) return false;
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
  if (!environmentCollisionEnabled) return false;
  return vehicleCollisionAgainst(vehicleEnvironmentPoints, proposedX, proposedZ, proposedYaw) ||
         vehicleCollisionAgainst(vehicleImportedWorldPoints, proposedX, proposedZ, proposedYaw);
}

// Utility: would drone environment collide.
boolean wouldDroneEnvironmentCollide(float proposedX, float proposedY, float proposedZ) {
  if (!environmentCollisionEnabled) return false;
  return droneCollisionAgainst(droneEnvironmentPoints, proposedX, proposedY, proposedZ) ||
         droneCollisionAgainst(droneImportedWorldPoints, proposedX, proposedY, proposedZ);
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
  // Use the fast GL POINTS path for both imported and scanned clouds once they
  // exceed the simple-render threshold. sphere() costs one full GL draw call per
  // point and becomes the dominant frame cost well before 1 000 points.
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

// Utility: draw current environment map.
void drawCurrentEnvironmentMap() {
  drawCurrentDemoWorld();
  ArrayList<PVector> imported = currentImportedWorldPointList();
  ArrayList<PVector> scanned = currentEnvironmentPointList();
  if ((imported == null || imported.isEmpty()) && (scanned == null || scanned.isEmpty())) return;

  pushStyle();
  noStroke();
  drawEnvironmentPointSet(imported, 0.62f, 0.78f);
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
