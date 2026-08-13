// =====================================================================
// SynROV Processing - Shared 3D scene environment
// ---------------------------------------------------------------------
// Purpose:
//   Centralize robot operating-area dimensions, world-grid rendering,
//   overview zoom, presentation scaling, camera transforms and scene limits.
// =====================================================================

final float SCENE_UNITS_PER_METER_MANIPULATOR = 100.0f;
final float SCENE_UNITS_PER_METER_VEHICLE = 100.0f;
final float SCENE_UNITS_PER_METER_DRONE = 50.0f;

final float MANIPULATOR_OPERATING_RADIUS_M = 30.0f;
final float VEHICLE_OPERATING_RADIUS_M = 2000.0f;
final float DRONE_OPERATING_RADIUS_M = 2000.0f;
final float DRONE_OPERATING_MAX_ALTITUDE_M = 1000.0f;

final float MANIPULATOR_GRID_STEP_M = 5.0f;
final float MOBILE_GRID_STEP_M = 100.0f;
final float MOBILE_GRID_MAJOR_STEP_M = 500.0f;
final float DRONE_ALTITUDE_GUIDE_STEP_M = 10.0f;
final float DRONE_ALTITUDE_LABEL_STEP_M = 10.0f;

final float LARGE_WORLD_TRUE_SCALE_ZOOM = 0.24f;
final float LARGE_WORLD_TARGET_ROBOT_PIXELS = 24.0f;
final float LARGE_WORLD_MAX_PRESENTATION_SCALE = 180.0f;
final float LARGE_WORLD_OVERVIEW_SCREEN_FRACTION = 0.32f;
final float LARGE_WORLD_MIN_ABSOLUTE_ZOOM = 0.00075f;
final float LARGE_WORLD_EXTREME_CENTER_MULTIPLIER = 10.0f;
final float LARGE_WORLD_MIN_EXTREME_CENTER_THRESHOLD = 0.025f;
final float LARGE_WORLD_MAX_EXTREME_CENTER_THRESHOLD = 0.20f;

// Returns scene units represented by one meter for the selected robot.
float sceneUnitsPerMeterForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_DRONE) return SCENE_UNITS_PER_METER_DRONE;
  if (robotMode == ROBOT_MODE_VEHICLE) return SCENE_UNITS_PER_METER_VEHICLE;
  return SCENE_UNITS_PER_METER_MANIPULATOR;
}

// Returns the configured planar operating radius in meters.
float operatingAreaRadiusMetersForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_VEHICLE) return VEHICLE_OPERATING_RADIUS_M;
  if (robotMode == ROBOT_MODE_DRONE) return DRONE_OPERATING_RADIUS_M;
  return MANIPULATOR_OPERATING_RADIUS_M;
}

// Returns the configured planar operating radius in scene units.
float operatingAreaRadiusSceneForRobotMode(int robotMode) {
  return operatingAreaRadiusMetersForRobotMode(robotMode) * sceneUnitsPerMeterForRobotMode(robotMode);
}

// Returns the shared world-grid spacing in scene units.
float operatingAreaGridStepSceneForRobotMode(int robotMode) {
  float stepMeters = robotMode == ROBOT_MODE_MANIPULATOR
    ? MANIPULATOR_GRID_STEP_M
    : MOBILE_GRID_STEP_M;
  return stepMeters * sceneUnitsPerMeterForRobotMode(robotMode);
}

// Returns how many minor grid cells form one major cell.
int operatingAreaMajorEveryForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) return 2;
  return max(1, round(MOBILE_GRID_MAJOR_STEP_M / MOBILE_GRID_STEP_M));
}

// Returns the maximum configured altitude in centimeters.
float maxAltitudeCmForRobotMode(int robotMode) {
  if (robotMode != ROBOT_MODE_DRONE) return 0.0f;
  return DRONE_OPERATING_MAX_ALTITUDE_M * 100.0f;
}

// Returns the maximum configured altitude in scene units.
float maxAltitudeSceneForRobotMode(int robotMode) {
  if (robotMode != ROBOT_MODE_DRONE) return 0.0f;
  return DRONE_OPERATING_MAX_ALTITUDE_M * sceneUnitsPerMeterForRobotMode(robotMode);
}

// Returns a far clipping plane large enough for the current operating area.
float sceneFarClipForRobotMode(int robotMode) {
  float horizontal = operatingAreaRadiusSceneForRobotMode(robotMode) * 2.4f;
  float vertical = maxAltitudeSceneForRobotMode(robotMode) * 2.4f;
  return max(30000.0f, max(horizontal, vertical));
}

// Calculates the zoom needed to fit the operating radius in a wide overview.
float operatingAreaOverviewZoomForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) return MIN_ZOOM;
  float radiusScene = max(1.0f, operatingAreaRadiusSceneForRobotMode(robotMode));
  float viewport = max(240.0f, min(width, height));
  float targetRadiusPixels = viewport * LARGE_WORLD_OVERVIEW_SCREEN_FRACTION;
  return constrain(targetRadiusPixels / radiusScene, LARGE_WORLD_MIN_ABSOLUTE_ZOOM, 1.0f);
}

// Returns the minimum camera zoom for the selected robot.
float sceneMinZoomForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) return MIN_ZOOM;
  return operatingAreaOverviewZoomForRobotMode(robotMode);
}

// Returns the threshold below which the camera target is recentered.
float sceneExtremeZoomCenterThresholdForRobotMode(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) return MIN_ZOOM;
  return constrain(
    sceneMinZoomForRobotMode(robotMode) * LARGE_WORLD_EXTREME_CENTER_MULTIPLIER,
    LARGE_WORLD_MIN_EXTREME_CENTER_THRESHOLD,
    LARGE_WORLD_MAX_EXTREME_CENTER_THRESHOLD
  );
}

// Keeps the robot readable at kilometer-scale zoom without changing physics.
float largeWorldRobotPresentationScale(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) return 1.0f;
  float currentZoom = max(0.000001f, zoomLevel);
  if (currentZoom >= LARGE_WORLD_TRUE_SCALE_ZOOM) return 1.0f;
  float robotSpan = max(1.0f, robotReferenceSpanScene(robotMode));
  float requiredScale = LARGE_WORLD_TARGET_ROBOT_PIXELS / (robotSpan * currentZoom);
  return constrain(requiredScale, 1.0f, LARGE_WORLD_MAX_PRESENTATION_SCALE);
}

// Returns a screen-conscious clearance radius around the selected robot.
// The grid is interrupted inside this radius so ground lines never compete
// visually with the robot model, even at kilometer-scale overview zoom.
float operatingGridRobotClearRadiusScene(int robotMode) {
  float radius = operatingAreaRadiusSceneForRobotMode(robotMode);
  float robotSpan = max(1.0f, robotReferenceSpanScene(robotMode));
  float currentZoom = max(0.000001f, zoomLevel);
  float screenTargetScene = 46.0f / currentZoom;
  return constrain(screenTargetScene, robotSpan * 1.75f, radius * 0.16f);
}

// Returns the representative vertical height whose silhouette should be kept
// free of ground-grid lines. The value is deliberately visual rather than a
// collision dimension: it protects the tall Manipulator, the low Vehicle and
// the Drone at its current altitude using the same projection-aware algorithm.
float operatingGridVisualHeightScene(int robotMode) {
  float robotSpan = max(1.0f, robotReferenceSpanScene(robotMode));
  if (robotMode == ROBOT_MODE_DRONE) {
    return max(robotSpan * 0.30f, max(0.0f, GROUND_Y - (getDroneSceneY() + droneY)));
  }
  if (robotMode == ROBOT_MODE_MANIPULATOR) return robotSpan * 0.62f;
  return robotSpan * 0.18f;
}

// Returns the ground-space point whose current camera projection sits behind
// the visible robot silhouette. This general form protects all robot modes:
// the Drone uses its live altitude, the Manipulator uses its tall arm envelope,
// and the Vehicle uses a smaller body-height allowance.
PVector operatingGridVisualClearCenterWorld(
  int robotMode,
  float navX,
  float navZ,
  float navYaw
) {
  float verticalSeparation = operatingGridVisualHeightScene(robotMode);
  float sinPitch = sin(cameraRotationX);
  if (verticalSeparation < 1.0f || abs(sinPitch) < 0.08f) {
    return new PVector(navX, 0.0f, navZ);
  }

  // In camera-yaw coordinates, qZ is the ground displacement that projects
  // to the same screen Y as the protected robot height after camera rotation.
  float qZ = verticalSeparation * cos(cameraRotationX) / sinPitch;
  float cameraYawSin = sin(cameraRotationY);
  float cameraYawCos = cos(cameraRotationY);
  float anchoredX = -cameraYawSin * qZ;
  float anchoredZ = cameraYawCos * qZ;

  // Convert the anchored offset back into global X/Z coordinates because the
  // operating grid remains locked to world coordinates while mobile robots are
  // visually anchored near the center of their scene.
  float navCos = cos(navYaw);
  float navSin = sin(navYaw);
  float worldOffsetX = navCos * anchoredX + navSin * anchoredZ;
  float worldOffsetZ = -navSin * anchoredX + navCos * anchoredZ;
  PVector projected = new PVector(navX + worldOffsetX, 0.0f, navZ + worldOffsetZ);

  // If the projected point falls outside the displayed operating disk, no
  // visible grid segment can cross the protected silhouette at that angle.
  float radius = operatingAreaRadiusSceneForRobotMode(robotMode);
  if (dist(projected.x, projected.z, navX, navZ) > radius) {
    return new PVector(navX, 0.0f, navZ);
  }
  return projected;
}

// Reduces grid contrast when the camera is close to the horizon, where many
// long perspective lines otherwise collapse into the robot silhouette.
float operatingGridCameraVisibilityFactor() {
  float topDownFactor = abs(sin(cameraRotationX));
  return constrain(map(topDownFactor, 0.0f, 1.0f, 0.24f, 1.0f), 0.24f, 1.0f);
}

// Reduces secondary-grid contrast as the camera approaches a wide overview.
// The same policy is applied to every robot so the ground never dominates the
// selected model, even though the Manipulator uses a much smaller work area.
float operatingGridOverviewVisibilityFactor(int robotMode) {
  if (robotMode == ROBOT_MODE_MANIPULATOR) {
    return constrain(map(zoomLevel, MIN_ZOOM, MAX_ZOOM, 0.62f, 1.0f), 0.62f, 1.0f);
  }
  float minZoom = max(0.000001f, sceneMinZoomForRobotMode(robotMode));
  float ratio = zoomLevel / minZoom;
  return constrain(map(ratio, 1.0f, 7.0f, 0.38f, 1.0f), 0.38f, 1.0f);
}

// Detailed demo/imported worlds already provide strong spatial cues. In those
// environments the operating grid becomes a secondary reference so it does not
// visually fight with walls, floors, machinery or the selected robot.
float operatingGridEnvironmentVisibilityFactor(int robotMode) {
  if (hasImportedWorldForRobotMode(robotMode)) return 0.48f;
  if (shouldUseDemoWorldForRobotMode(robotMode)) return 0.68f;
  return 1.0f;
}

// Draws one X-aligned world-grid segment while leaving a projection-aware
// capsule corridor free between the robot anchor and its projected silhouette.
// A single circular hole fails at close, oblique camera angles because the
// visible body spans both points; the sampled capsule protects that full span.
void drawOperatingGridXLineWithRobotGap(
  float worldX,
  float minZ,
  float maxZ,
  float clearStartX,
  float clearStartZ,
  float clearEndX,
  float clearEndZ,
  float clearRadius
) {
  float corridorLength = dist(clearStartX, clearStartZ, clearEndX, clearEndZ);
  int samples = constrain(ceil(corridorLength / max(1.0f, clearRadius * 0.55f)) + 1, 2, 32);
  boolean intersects = false;
  float gapMin = Float.MAX_VALUE;
  float gapMax = -Float.MAX_VALUE;
  for (int sample = 0; sample < samples; sample++) {
    float t = sample / float(max(1, samples - 1));
    float centerX = lerp(clearStartX, clearEndX, t);
    float centerZ = lerp(clearStartZ, clearEndZ, t);
    float dx = worldX - centerX;
    if (abs(dx) >= clearRadius) continue;
    float halfGap = sqrt(max(0.0f, clearRadius * clearRadius - dx * dx));
    gapMin = min(gapMin, centerZ - halfGap);
    gapMax = max(gapMax, centerZ + halfGap);
    intersects = true;
  }
  if (!intersects) {
    line(worldX, 0, minZ, worldX, 0, maxZ);
    return;
  }
  if (minZ < gapMin) line(worldX, 0, minZ, worldX, 0, min(maxZ, gapMin));
  if (maxZ > gapMax) line(worldX, 0, max(minZ, gapMax), worldX, 0, maxZ);
}

// Z-aligned companion of the projection-aware capsule clearance above.
void drawOperatingGridZLineWithRobotGap(
  float worldZ,
  float minX,
  float maxX,
  float clearStartX,
  float clearStartZ,
  float clearEndX,
  float clearEndZ,
  float clearRadius
) {
  float corridorLength = dist(clearStartX, clearStartZ, clearEndX, clearEndZ);
  int samples = constrain(ceil(corridorLength / max(1.0f, clearRadius * 0.55f)) + 1, 2, 32);
  boolean intersects = false;
  float gapMin = Float.MAX_VALUE;
  float gapMax = -Float.MAX_VALUE;
  for (int sample = 0; sample < samples; sample++) {
    float t = sample / float(max(1, samples - 1));
    float centerX = lerp(clearStartX, clearEndX, t);
    float centerZ = lerp(clearStartZ, clearEndZ, t);
    float dz = worldZ - centerZ;
    if (abs(dz) >= clearRadius) continue;
    float halfGap = sqrt(max(0.0f, clearRadius * clearRadius - dz * dz));
    gapMin = min(gapMin, centerX - halfGap);
    gapMax = max(gapMax, centerX + halfGap);
    intersects = true;
  }
  if (!intersects) {
    line(minX, 0, worldZ, maxX, 0, worldZ);
    return;
  }
  if (minX < gapMin) line(minX, 0, worldZ, min(maxX, gapMin), 0, worldZ);
  if (maxX > gapMax) line(max(minX, gapMax), 0, worldZ, maxX, 0, worldZ);
}

// Draws a circular, world-aligned operating grid in the anchored robot frame.
void drawRobotOperatingAreaGrid(int robotMode, float navX, float navZ, float navYaw) {
  float radius = operatingAreaRadiusSceneForRobotMode(robotMode);
  float step = max(1.0f, operatingAreaGridStepSceneForRobotMode(robotMode));
  int majorEvery = operatingAreaMajorEveryForRobotMode(robotMode);
  float clearRadius = operatingGridRobotClearRadiusScene(robotMode);
  PVector clearCenter = operatingGridVisualClearCenterWorld(robotMode, navX, navZ, navYaw);

  // The displayed area follows the robot position, but every grid line remains
  // anchored to global X/Z coordinates. This gives a 2 km world window without
  // imposing an artificial navigation fence on real or simulated hardware.
  int minXIndex = floor((navX - radius) / step);
  int maxXIndex = ceil((navX + radius) / step);
  int minZIndex = floor((navZ - radius) / step);
  int maxZIndex = ceil((navZ + radius) / step);

  float cameraFactor = operatingGridCameraVisibilityFactor();
  float overviewFactor = operatingGridOverviewVisibilityFactor(robotMode);
  float environmentFactor = operatingGridEnvironmentVisibilityFactor(robotMode);
  float altitudeFactor = 1.0f;
  if (robotMode == ROBOT_MODE_DRONE) {
    float maxAltitudeScene = max(1.0f, maxAltitudeSceneForRobotMode(ROBOT_MODE_DRONE));
    float altitudeRatio = constrain(droneFlightLift / maxAltitudeScene, 0.0f, 1.0f);
    altitudeFactor = lerp(1.0f, 0.58f, altitudeRatio);
  }
  float minorAlpha = 58.0f * cameraFactor * overviewFactor * altitudeFactor * environmentFactor;
  float majorAlpha = 128.0f * cameraFactor * lerp(0.72f, 1.0f, overviewFactor) * lerp(0.82f, 1.0f, altitudeFactor) * environmentFactor;
  float axisAlpha = 165.0f * cameraFactor * lerp(0.86f, 1.0f, altitudeFactor) * lerp(0.72f, 1.0f, environmentFactor);
  float boundaryAlpha = 150.0f * cameraFactor * lerp(0.70f, 1.0f, overviewFactor) * lerp(0.84f, 1.0f, altitudeFactor) * lerp(0.60f, 1.0f, environmentFactor);

  pushStyle();
  pushMatrix();
  translate(0, GROUND_Y + 0.03f, 0);
  rotateY(-navYaw);
  translate(-navX, 0, -navZ);

  // Keep world-grid strokes visually light. The circular clearance below is
  // the primary protection against lines crossing the selected robot.
  strokeWeight(0.85f);

  for (int i = minXIndex; i <= maxXIndex; i++) {
    float worldX = i * step;
    float dx = worldX - navX;
    float remainingSq = radius * radius - dx * dx;
    if (remainingSq < 0.0f) continue;
    float zExtent = sqrt(max(0.0f, remainingSq));
    boolean major = abs(i % majorEvery) == 0;
    if (major) stroke(sceneGroundGridColor(), majorAlpha);
    else stroke(120, 120, 120, minorAlpha);
    drawOperatingGridXLineWithRobotGap(
      worldX,
      navZ - zExtent,
      navZ + zExtent,
      navX,
      navZ,
      clearCenter.x,
      clearCenter.z,
      clearRadius
    );
  }

  for (int i = minZIndex; i <= maxZIndex; i++) {
    float worldZ = i * step;
    float dz = worldZ - navZ;
    float remainingSq = radius * radius - dz * dz;
    if (remainingSq < 0.0f) continue;
    float xExtent = sqrt(max(0.0f, remainingSq));
    boolean major = abs(i % majorEvery) == 0;
    if (major) stroke(sceneGroundGridColor(), majorAlpha);
    else stroke(120, 120, 120, minorAlpha);
    drawOperatingGridZLineWithRobotGap(
      worldZ,
      navX - xExtent,
      navX + xExtent,
      navX,
      navZ,
      clearCenter.x,
      clearCenter.z,
      clearRadius
    );
  }

  // Keep global X/Z axes available as references, but apply the same clean
  // zone so even a global axis cannot cut through the selected robot.
  stroke(32, 32, 32, axisAlpha);
  strokeWeight(1.25f);
  if (abs(navZ) <= radius) {
    float xExtent = sqrt(max(0.0f, radius * radius - navZ * navZ));
    drawOperatingGridZLineWithRobotGap(
      0.0f,
      navX - xExtent,
      navX + xExtent,
      navX,
      navZ,
      clearCenter.x,
      clearCenter.z,
      clearRadius
    );
  }
  if (abs(navX) <= radius) {
    float zExtent = sqrt(max(0.0f, radius * radius - navX * navX));
    drawOperatingGridXLineWithRobotGap(
      0.0f,
      navZ - zExtent,
      navZ + zExtent,
      navX,
      navZ,
      clearCenter.x,
      clearCenter.z,
      clearRadius
    );
  }

  // The operating-radius boundary remains visible but deliberately quieter
  // than the robot and major grid references.
  stroke(30, 110, 190, boundaryAlpha);
  strokeWeight(1.35f);
  noFill();
  beginShape();
  int boundarySegments = 180;
  for (int i = 0; i <= boundarySegments; i++) {
    float angle = TWO_PI * i / float(boundarySegments);
    vertex(navX + cos(angle) * radius, 0, navZ + sin(angle) * radius);
  }
  endShape();

  safePopMatrix("SceneEnvironment3D.pde:operatingAreaGrid");
  popStyle();
}

// Draws the drone altitude reference from ground level to the configured ceiling.
void drawDroneOperatingAltitudeGuide() {
  float maxAltitudeMeters = DRONE_OPERATING_MAX_ALTITUDE_M;
  float sceneUnitsPerMeter = sceneUnitsPerMeterForRobotMode(ROBOT_MODE_DRONE);
  float topY = GROUND_Y - maxAltitudeSceneForRobotMode(ROBOT_MODE_DRONE);
  float currentZoom = max(0.000001f, zoomLevel);
  float guideRadius = operatingAreaRadiusSceneForRobotMode(ROBOT_MODE_DRONE);
  // Keep the ruler readable in screen space from normal view to kilometer overview.
  float guideX = constrain(130.0f / currentZoom, 220.0f, guideRadius * 0.35f);

  pushStyle();
  stroke(65, 65, 65, 185);
  strokeWeight(1.2f);
  line(guideX, GROUND_Y, 0, guideX, topY, 0);

  for (float altitudeMeters = 0.0f; altitudeMeters <= maxAltitudeMeters + 0.01f; altitudeMeters += DRONE_ALTITUDE_GUIDE_STEP_M) {
    float y = GROUND_Y - altitudeMeters * sceneUnitsPerMeter;
    boolean labeled = abs((altitudeMeters / DRONE_ALTITUDE_LABEL_STEP_M) - round(altitudeMeters / DRONE_ALTITUDE_LABEL_STEP_M)) < 0.01f;
    float tickHalf = (labeled ? 28.0f : 16.0f) / currentZoom;
    stroke(labeled ? color(38, 38, 38, 210) : color(95, 95, 95, 150));
    line(guideX - tickHalf, y, 0, guideX + tickHalf, y, 0);

    if (labeled) {
      pushMatrix();
      translate(guideX + 40.0f, y, 0);
      rotateY(-cameraRotationY);
      rotateX(-cameraRotationX);
      scale(1.0f / currentZoom);
      fill(uiPrimaryTextColor());
      noStroke();
      textAlign(LEFT, CENTER);
      textSize(11.0f);
      text(round(altitudeMeters) + " m", 0, 0);
      safePopMatrix("SceneEnvironment3D.pde:altitudeGuideLabel");
    }
  }
  popStyle();
}

// Draws the shared operating environment for a robot mode.
void drawRobotOperatingEnvironment(
  int robotMode,
  float navX, float navZ, float navYaw,
  float headingDeg,
  boolean alignCardinalOverlay
) {
  drawRobotOperatingAreaGrid(robotMode, navX, navZ, navYaw);

  // The operating-area grid is now the single ground grid for every robot.
  // Keep the dense local fallback disabled to avoid a second set of lines under
  // the Manipulator, Vehicle or Drone. Cardinal references retain a center gap.
  drawSharedGroundPlane(
    visualGroundSizeForRobotMode(robotMode),
    headingDeg,
    alignCardinalOverlay,
    false
  );
  if (robotMode == ROBOT_MODE_DRONE) drawDroneOperatingAltitudeGuide();
}

// Applies the robot-centric scene reference inside the camera zoom frame.
//
// Drone navigation is intentionally rendered in a robot-referenced frame. X/Z
// were already anchored by DRONE_RENDER_ANCHORED; altitude must follow the same
// rule. Keeping the drone body at the local origin makes climb/descent move the
// world and ground relative to the aircraft instead of letting the aircraft
// drift out of the viewport while the ground remains the visual reference.
void applyRobotSceneReferenceOffset(int robotMode) {
  if (robotMode == ROBOT_MODE_DRONE) {
    translate(-getDroneSceneX(), -(getDroneSceneY() + droneY), -getDroneSceneZ());
  }
}

PVector robotZoomFocus3D(int robotMode) {
  if (isRobotCameraThirdPersonActiveForMode(robotMode)) return new PVector(0, 0, 0);
  if (robotMode == ROBOT_MODE_VEHICLE) {
    return new PVector(getVehicleSceneX(), GROUND_Y + VEH_BODY_CENTER_Y_OFFSET + vehicleBodyBob, getVehicleSceneZ());
  }
  if (robotMode == ROBOT_MODE_DRONE) {
    // applyRobotSceneReferenceOffset() keeps the drone itself at the origin.
    // Zooming around a world/ground coordinate here would break the
    // ground-referenced behavior during altitude changes.
    return new PVector(0, 0, 0);
  }
  float manipFocusY = GROUND_Y - max(20.0f, robotReferenceSpanScene(ROBOT_MODE_MANIPULATOR) * 0.28f);
  return new PVector(0, manipFocusY, 0);
}

// Opens the common 3D camera transform for a robot scene.
void beginRobotScene3D(int robotMode, boolean advanceCamera) {
  perspective(
    PI / 3.0f,
    max(0.1f, width / (float)max(1, height)),
    1.0f,
    sceneFarClipForRobotMode(robotMode)
  );

  pushMatrix();
  translate(width / 2 + 120, height / 2 + 80);
  rotateX(cameraRotationX);
  rotateY(cameraRotationY);

  if (advanceCamera) {
    cameraRotationY += cameraRotationYIncrement;
    cameraRotationX += cameraRotationXIncrement;
  }

  if (zoomLevel != 1.0f) {
    PVector focus = robotZoomFocus3D(robotMode);
    translate(focus.x, focus.y, focus.z);
    scale(zoomLevel);
    translate(-focus.x, -focus.y, -focus.z);
  }

  // Apply robot-centering inside the scaled frame. If the Drone altitude offset
  // is applied before scale while the body translation is applied after scale,
  // a residual (zoom - 1) * altitude moves the aircraft away from its reference.
  applyRobotSceneReferenceOffset(robotMode);
  applyRobotCameraThirdPersonSceneOffset(robotMode);
}

// Closes the common 3D camera transform and restores Processing projection.
void endRobotScene3D(String context) {
  safePopMatrix(context);
  perspective();
}
