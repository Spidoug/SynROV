// =====================================================================
// SynROV Processing - Manipulator 3D model
// ---------------------------------------------------------------------
// Purpose:
//   Manipulator geometry, kinematics, collision helpers and 3D drawing.
// =====================================================================

final float GROUND_EPS = 0.1f;      // Small elevation to prevent z-fighting during rendering
final float BASE_GAP = 0.0f;        // Optional gap between the top of the cylinder and the purple base
final float ARM_FINGER_PIVOT_HEIGHT_RATIO = 0.22857143f;
final float ARM_FINGER_LATERAL_CLEARANCE_CM = 0.321f;
final float TRAJECTORY_POINT_RADIUS = 0.95f;
final float GRIPPER_TIP_MARKER_RADIUS = 2.1f;

final float DEFAULT_ARM_BASE_RADIUS = 15.0f;
final float DEFAULT_ARM_BASE_HEIGHT = 30.0f;
final float DEFAULT_ARM_BASE_BLOCK_W = 12.5f;
final float DEFAULT_ARM_BASE_BLOCK_H = 20.0f;
final float DEFAULT_ARM_BASE_BLOCK_D = 12.0f;
final float DEFAULT_ARM_UPPER_W = 7.65f;
final float DEFAULT_ARM_UPPER_H = 37.0f;
final float DEFAULT_ARM_UPPER_D = 7.5f;
final float DEFAULT_ARM_FORE_W = 5.544f;
final float DEFAULT_ARM_FORE_H = 29.0f;
final float DEFAULT_ARM_FORE_D = 5.412f;
final float DEFAULT_ARM_WRIST_W = 3.219f;
final float DEFAULT_ARM_WRIST_H = 6.0f;
final float DEFAULT_ARM_WRIST_D = 3.219f;
final float DEFAULT_ARM_FINGER_W = 0.709f;
final float DEFAULT_ARM_FINGER_H = 10.0f;
final float DEFAULT_ARM_FINGER_D = 0.945f;
final float DEFAULT_ARM_BASE_CYLINDER_Y = -15.5f;
final float DEFAULT_ARM_BASE_BLOCK_Y = -40.5f;
final float DEFAULT_ARM_GRIPPER_Y = -1.8f;

// Utility: default manipulator servo limits JSON.
JSONArray defaultManipulatorServoLimitsJson() {
  JSONArray jsonArray = new JSONArray();
  int[][] defaultLimits = {
    {0, 359},
    {0, 180},
    {0, 180},
    {0, 180},
    {0, 180},
    {0, 180},
    {0, 100}
  };
  for (int i = 0; i < defaultLimits.length; i++) {
    JSONObject obj = new JSONObject();
    obj.setInt("min", defaultLimits[i][0]);
    obj.setInt("max", defaultLimits[i][1]);
    jsonArray.append(obj);
  }
  return jsonArray;
}

// Utility: default manipulator homing JSON.
JSONArray defaultManipulatorHomingJson() {
  JSONArray jsonArray = new JSONArray();
  int[] defaultHomingAngles = {180, 150, 70, 90, 95, 130, 0};
  for (int i = 0; i < defaultHomingAngles.length; i++) {
    JSONObject obj = new JSONObject();
    obj.setInt("angle", defaultHomingAngles[i]);
    jsonArray.append(obj);
  }
  return jsonArray;
}

// Applies manipulator servo limits JSON.
void applyManipulatorServoLimitsJson(JSONArray jsonArray) {
  if (jsonArray == null) return;
  for (int i = 0; i < jsonArray.size() && i < servoLimits.length; i++) {
    JSONObject obj = jsonArray.getJSONObject(i);
    if (obj == null) continue;
    servoLimits[i][0] = getJsonInt(obj, "min", servoLimits[i][0]);
    servoLimits[i][1] = getJsonInt(obj, "max", servoLimits[i][1]);
  }
}

// Applies manipulator homing JSON.
void applyManipulatorHomingJson(JSONArray jsonArray) {
  if (jsonArray == null) return;
  for (int i = 0; i < jsonArray.size() && i < homingPositions.length; i++) {
    JSONObject obj = jsonArray.getJSONObject(i);
    if (obj == null) continue;
    homingPositions[i] = getJsonInt(obj, "angle", homingPositions[i]);
  }
}

// Builds manipulator config 3 d object.
JSONObject buildManipulatorConfig3DObject() {
  JSONObject config3D = new JSONObject();

  JSONObject dims = new JSONObject();

  JSONObject baseCyl = new JSONObject();
  baseCyl.setFloat("radius", DEFAULT_ARM_BASE_RADIUS);
  baseCyl.setFloat("height", DEFAULT_ARM_BASE_HEIGHT);
  dims.setJSONObject("baseCylinder", baseCyl);

  JSONObject baseBlock = new JSONObject();
  baseBlock.setFloat("width", DEFAULT_ARM_BASE_BLOCK_W);
  baseBlock.setFloat("height", DEFAULT_ARM_BASE_BLOCK_H);
  baseBlock.setFloat("depth", DEFAULT_ARM_BASE_BLOCK_D);
  dims.setJSONObject("baseBlock", baseBlock);

  JSONObject upperArm = new JSONObject();
  upperArm.setFloat("width", DEFAULT_ARM_UPPER_W);
  upperArm.setFloat("height", DEFAULT_ARM_UPPER_H);
  upperArm.setFloat("depth", DEFAULT_ARM_UPPER_D);
  dims.setJSONObject("upperArm", upperArm);

  JSONObject forearm = new JSONObject();
  forearm.setFloat("width", DEFAULT_ARM_FORE_W);
  forearm.setFloat("height", DEFAULT_ARM_FORE_H);
  forearm.setFloat("depth", DEFAULT_ARM_FORE_D);
  dims.setJSONObject("forearm", forearm);

  JSONObject wrist = new JSONObject();
  wrist.setFloat("width", DEFAULT_ARM_WRIST_W);
  wrist.setFloat("height", DEFAULT_ARM_WRIST_H);
  wrist.setFloat("depth", DEFAULT_ARM_WRIST_D);
  dims.setJSONObject("wrist", wrist);

  JSONObject gripper = new JSONObject();
  gripper.setFloat("fingerWidth", DEFAULT_ARM_FINGER_W);
  gripper.setFloat("fingerHeight", DEFAULT_ARM_FINGER_H);
  gripper.setFloat("fingerDepth", DEFAULT_ARM_FINGER_D);
  dims.setJSONObject("gripper", gripper);

  config3D.setJSONObject("dimensions", dims);

  JSONObject visualOffsets = new JSONObject();
  visualOffsets.setFloat("baseCylinderY", DEFAULT_ARM_BASE_CYLINDER_Y);
  visualOffsets.setFloat("baseBlockY", DEFAULT_ARM_BASE_BLOCK_Y);
  visualOffsets.setFloat("gripperY", DEFAULT_ARM_GRIPPER_Y);
  config3D.setJSONObject("visualOffsets", visualOffsets);

  JSONObject jointOffsets = new JSONObject();

  JSONObject jBase = new JSONObject();
  jBase.setFloat("servoZero", 180.0f);
  jBase.setFloat("memberZero", 0.0f);
  jBase.setFloat("sign", 1.0f);
  jointOffsets.setJSONObject("base", jBase);

  JSONObject jUpper = new JSONObject();
  jUpper.setFloat("servoZero", 45.0f);
  jUpper.setFloat("memberZero", 0.0f);
  jUpper.setFloat("sign", 1.0f);
  jointOffsets.setJSONObject("upper", jUpper);

  JSONObject jFore = new JSONObject();
  jFore.setFloat("servoZero", 180.0f);
  jFore.setFloat("memberZero", 0.0f);
  jFore.setFloat("sign", 1.0f);
  jointOffsets.setJSONObject("forearm", jFore);

  JSONObject jForeRoll = new JSONObject();
  jForeRoll.setFloat("servoZero", 90.0f);
  jForeRoll.setFloat("memberZero", 0.0f);
  jForeRoll.setFloat("sign", 1.0f);
  jointOffsets.setJSONObject("forearmRoll", jForeRoll);

  JSONObject jWristPitch = new JSONObject();
  jWristPitch.setFloat("servoZero", 90.0f);
  jWristPitch.setFloat("memberZero", 0.0f);
  jWristPitch.setFloat("sign", 1.0f);
  jointOffsets.setJSONObject("wristPitch", jWristPitch);

  JSONObject jWristRoll = new JSONObject();
  jWristRoll.setFloat("servoZero", 90.0f);
  jWristRoll.setFloat("memberZero", 0.0f);
  jWristRoll.setFloat("sign", 1.0f);
  jointOffsets.setJSONObject("wristRoll", jWristRoll);

  JSONObject jGrip = new JSONObject();
  jGrip.setFloat("servoZero", 0.0f);
  jGrip.setFloat("memberZero", 0.0f);
  jGrip.setFloat("sign", 1.0f);
  jointOffsets.setJSONObject("gripper", jGrip);

  config3D.setJSONObject("jointOffsets", jointOffsets);

  JSONObject colliderDims = new JSONObject();
  JSONObject groundCol = new JSONObject();
  groundCol.setFloat("width", MANIPULATOR_VISUAL_GROUND_SIZE);
  groundCol.setFloat("height", 1.0);
  groundCol.setFloat("depth", MANIPULATOR_VISUAL_GROUND_SIZE);
  colliderDims.setJSONObject("ground", groundCol);
  config3D.setJSONObject("colliderDimensions", colliderDims);

  return config3D;
}

// Builds manipulator config object.
JSONObject buildManipulatorConfigObject() {
  JSONObject json = new JSONObject();
  json.setString("schema", "synrov.robot_config.v2");
  json.setString("robot", "Manipulator");
  json.setString("geometryUnits", ROBOT_GEOMETRY_UNITS);
  json.setFloat("renderScale", MODEL_SCALE_3D);
  json.setJSONObject("config3D", buildManipulatorConfig3DObject());
  json.setJSONArray("limitServo", defaultManipulatorServoLimitsJson());
  json.setJSONArray("homingPosition", defaultManipulatorHomingJson());
  return json;
}

// Checks whether manipulator config schema valid.
boolean isManipulatorConfigSchemaValid(JSONObject json) {
  if (json == null) return false;
  if (!"synrov.robot_config.v2".equals(getJsonString(json, "schema", ""))) return false;
  JSONObject config3D = getJsonObjectSafe(json, "config3D");
  if (config3D == null) return false;

  JSONObject dims = getJsonObjectSafe(config3D, "dimensions");
  JSONObject visualOffsets = getJsonObjectSafe(config3D, "visualOffsets");
  JSONObject jointOffsets = getJsonObjectSafe(config3D, "jointOffsets");
  JSONObject colliderDimensions = getJsonObjectSafe(config3D, "colliderDimensions");
  if (dims == null || visualOffsets == null || jointOffsets == null || colliderDimensions == null) return false;

  return
    getJsonObjectSafe(dims, "baseCylinder") != null &&
    getJsonObjectSafe(dims, "baseBlock") != null &&
    getJsonObjectSafe(dims, "upperArm") != null &&
    getJsonObjectSafe(dims, "forearm") != null &&
    getJsonObjectSafe(dims, "wrist") != null &&
    getJsonObjectSafe(dims, "gripper") != null &&
    getJsonObjectSafe(jointOffsets, "base") != null &&
    getJsonObjectSafe(jointOffsets, "upper") != null &&
    getJsonObjectSafe(jointOffsets, "forearm") != null &&
    getJsonObjectSafe(jointOffsets, "forearmRoll") != null &&
    getJsonObjectSafe(jointOffsets, "wristPitch") != null &&
    getJsonObjectSafe(jointOffsets, "wristRoll") != null &&
    getJsonObjectSafe(jointOffsets, "gripper") != null &&
    getJsonObjectSafe(colliderDimensions, "ground") != null;
}

// Creates default config.
void createDefaultConfig(String filename) {
  JSONObject json = buildManipulatorConfigObject();
  saveJSONObject(json, filename);
}

// Loads config.
void loadConfig(String filename) {
  JSONObject json = loadJSONObjectWithRepair(filename, buildManipulatorConfigObject());
  if (json == null) return;

  JSONObject configRoot = getJsonObjectSafe(json, "config3D");
  JSONObject dims = getJsonObjectSafe(configRoot, "dimensions");
  if (configRoot == null || dims == null) {
    updateDimensions();
    return;
  }

  JSONObject baseCylinder = getJsonObjectSafe(dims, "baseCylinder");
  baseCylinderRadius = getJsonFloat(baseCylinder, "radius", baseCylinderRadius / MODEL_SCALE_3D) * MODEL_SCALE_3D;
  baseCylinderHeight = getJsonFloat(baseCylinder, "height", baseCylinderHeight / MODEL_SCALE_3D) * MODEL_SCALE_3D;

  JSONObject baseBlock = getJsonObjectSafe(dims, "baseBlock");
  baseBlockW = getJsonFloat(baseBlock, "width", baseBlockW / MODEL_SCALE_3D) * MODEL_SCALE_3D;
  baseBlockH = getJsonFloat(baseBlock, "height", baseBlockH / MODEL_SCALE_3D) * MODEL_SCALE_3D;
  baseBlockD = getJsonFloat(baseBlock, "depth", baseBlockD / MODEL_SCALE_3D) * MODEL_SCALE_3D;

  JSONObject upperArm = getJsonObjectSafe(dims, "upperArm");
  upperArmW = getJsonFloat(upperArm, "width", upperArmW / MODEL_SCALE_3D) * MODEL_SCALE_3D;
  upperArmH = getJsonFloat(upperArm, "height", upperArmH / MODEL_SCALE_3D) * MODEL_SCALE_3D;
  upperArmD = getJsonFloat(upperArm, "depth", upperArmD / MODEL_SCALE_3D) * MODEL_SCALE_3D;

  JSONObject forearm = getJsonObjectSafe(dims, "forearm");
  forearmW = getJsonFloat(forearm, "width", forearmW / MODEL_SCALE_3D) * MODEL_SCALE_3D;
  forearmH = getJsonFloat(forearm, "height", forearmH / MODEL_SCALE_3D) * MODEL_SCALE_3D;
  forearmD = getJsonFloat(forearm, "depth", forearmD / MODEL_SCALE_3D) * MODEL_SCALE_3D;

  JSONObject wrist = getJsonObjectSafe(dims, "wrist");
  wristVerticalW = getJsonFloat(wrist, "width", wristVerticalW / MODEL_SCALE_3D) * MODEL_SCALE_3D;
  wristVerticalH = getJsonFloat(wrist, "height", wristVerticalH / MODEL_SCALE_3D) * MODEL_SCALE_3D;
  wristVerticalD = getJsonFloat(wrist, "depth", wristVerticalD / MODEL_SCALE_3D) * MODEL_SCALE_3D;

  JSONObject gripper = getJsonObjectSafe(dims, "gripper");
  gripperFingerW = getJsonFloat(gripper, "fingerWidth", gripperFingerW / MODEL_SCALE_3D) * MODEL_SCALE_3D;
  gripperFingerH = getJsonFloat(gripper, "fingerHeight", gripperFingerH / MODEL_SCALE_3D) * MODEL_SCALE_3D;
  gripperFingerD = getJsonFloat(gripper, "fingerDepth", gripperFingerD / MODEL_SCALE_3D) * MODEL_SCALE_3D;

  applyManipulatorServoLimitsJson(getJsonArraySafe(json, "limitServo"));
  applyManipulatorHomingJson(getJsonArraySafe(json, "homingPosition"));

  updateDimensions();

  JSONObject visualOffsets = getJsonObjectSafe(configRoot, "visualOffsets");
  baseCylinderYOffset = getJsonFloat(visualOffsets, "baseCylinderY", DEFAULT_ARM_BASE_CYLINDER_Y) * MODEL_SCALE_3D;
  baseBlockYOffset = getJsonFloat(visualOffsets, "baseBlockY", DEFAULT_ARM_BASE_BLOCK_Y) * MODEL_SCALE_3D;
  gripperYOffset = getJsonFloat(visualOffsets, "gripperY", DEFAULT_ARM_GRIPPER_Y) * MODEL_SCALE_3D;
}

// Loads or create manipulator config.
void loadOrCreateManipulatorConfig() {
  File dataDir = new File(sketchPath("data"));
  if (!dataDir.exists()) dataDir.mkdirs();

  if (!fileExists(MANIPULATOR3D)) {
    saveJSONObjectEnsured(buildManipulatorConfigObject(), MANIPULATOR3D);
  }

  // The code is the default model. The file may override any subset of
  // parameters, but missing sections must not force a reset or NullPointer.
  loadJSONObjectWithRepair(MANIPULATOR3D, buildManipulatorConfigObject());
  loadConfig(MANIPULATOR3D);
}


// Returns finger pivot offset scene.
float getFingerPivotOffsetScene() {
  return gripperFingerH * ARM_FINGER_PIVOT_HEIGHT_RATIO;
}

// Returns finger lateral offset scene.
float getFingerLateralOffsetScene() {
  return (wristVerticalW * 0.5f) + (gripperFingerW * 0.5f) + (MODEL_SCALE_3D * ARM_FINGER_LATERAL_CLEARANCE_CM);
}

// Updates dimensions.
void updateDimensions() {
  // Ground dimensions aligned with the shared 1x scene scale.
  groundColliderW = visualGroundSizeForRobotMode(ROBOT_MODE_MANIPULATOR);
  groundColliderH = 1.0f;
  groundColliderD = visualGroundSizeForRobotMode(ROBOT_MODE_MANIPULATOR);

  // In P3D, Y+ points down. Keep these explicit defaults aligned with the firmware
  // instead of inferring them from other segment sizes.
  baseCylinderYOffset = DEFAULT_ARM_BASE_CYLINDER_Y * MODEL_SCALE_3D;
  baseBlockYOffset = DEFAULT_ARM_BASE_BLOCK_Y * MODEL_SCALE_3D;

  // Segment-local offsets (drawing at the geometric center)
  upperArmYOffset      = -upperArmH        / 2.0f;
  forearmYOffset       = -forearmH         / 2.0f;
  wristVerticalYOffset = -wristVerticalH   / 2.0f;
  gripperYOffset       = DEFAULT_ARM_GRIPPER_Y * MODEL_SCALE_3D;
  fingerXOffset        = -getFingerLateralOffsetScene();

  // 5) Cylinder collider: calculated from the cylinder center
  baseCylinderColliderR    = baseCylinderRadius;
  baseCylinderColliderYMin = baseCylinderYOffset - (baseCylinderHeight * 0.5f); // more negative (top)
  baseCylinderColliderYMax = baseCylinderYOffset + (baseCylinderHeight * 0.5f); // less negative (bottom)

  // Match the reduced collision profile used by the following arm segments.
  baseBlockColliderW = baseBlockW;
  baseBlockColliderH = baseBlockH;
  baseBlockColliderD = baseBlockD;

  upperArmColliderW = upperArmW;
  upperArmColliderH = upperArmH;
  upperArmColliderD = upperArmD;

  forearmColliderW = forearmW;
  forearmColliderH = forearmH;
  forearmColliderD = forearmD;

  wristVerticalColliderW = wristVerticalW;
  wristVerticalColliderH = wristVerticalH;
  wristVerticalColliderD = wristVerticalD;

  gripperFingerColliderW = gripperFingerW;
  gripperFingerColliderH = gripperFingerH;
  gripperFingerColliderD = gripperFingerD;
}

//===========================================================
// VISUALIZATION & DRAWING
//===========================================================


// Returns base model yaw degrees.
float getBaseModelYawDeg() {
  // Cardinal reference: base yaw/angle remain identical, but the model
  // front marker is mounted on the local +X face. Adding 90° makes
  // yaw 180° point to South on the stable cardinal floor.
  return normalizeAbsoluteAngleDeg(getBasePoseYawDeg() + 90.0f);
}

// Returns upper arm model z degrees.
float getUpperArmModelZDeg() {
  return 135.0f - getManipulatorVisualServoAngle(UPPERARM_IDX);
}

// Returns forearm model z degrees.
float getForearmModelZDeg() {
  return 180.0f - getManipulatorVisualServoAngle(FOREARM_IDX);
}

// Returns forearm roll model y degrees.
float getForearmRollModelYDeg() {
  return -(getManipulatorVisualServoAngle(FOREARM_ROLL_IDX) - 90.0f);
}

// Returns wrist vertical model z degrees.
float getWristVerticalModelZDeg() {
  return 90.0f - getManipulatorVisualServoAngle(WRIST_VERT_IDX);
}

// Returns wrist rotation model y degrees.
float getWristRotationModelYDeg() {
  return -(getManipulatorVisualServoAngle(WRIST_ROT_IDX) - 90.0f);
}

// Returns gripper finger model z degrees.
float getGripperFingerModelZDeg() {
  float gripperDeg = constrain(getManipulatorVisualServoAngle(GRIPPER_IDX), 0.0f, 100.0f);
  // Logical gripper convention:
  //   0°   = closed
  //   100° = fully open
  // Keep the same visual opening span as the previous 180→100 range,
  // but remap it so the closed pose now starts at 0°.
  return 40.0f - (0.8f * gripperDeg);
}

/**
 * Draws the 3D model of the robotic arm, including all its joints and links.
 */
// Utility: draw manipulator 3 d.
void drawManipulator3D(boolean advanceCamera) {

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

  noStroke();
  noFill();

  // ---------- Ground (shared floor) ----------
  // Keep the manipulator cardinal floor stable: base yaw 0/90/180/270
  // maps to North/East/South/West, without rotating the labels with telemetry.
  drawSharedGroundPlane(visualGroundSizeForCurrentRobot(), 0.0f, false);

  applySceneLighting();

  // ---------- Base cylinder (centered at baseCylinderYOffset) ----------
  pushMatrix();
  fill(80);
  translate(0, baseCylinderYOffset, 0);
  drawCylinder(baseCylinderRadius, baseCylinderHeight);

  // ---------- Purple base resting on top of the cylinder ----------
  safePopMatrix("Manipulator3D.pde:487"); // closes the cylinder's pushMatrix
  pushMatrix();
  translate(0, baseBlockYOffset, 0);
  rotateY(radians(getBaseModelYawDeg()));
  fill(173, 55, 247);
  box(baseBlockW, baseBlockH, baseBlockD);

  // Yellow marker glued to the front face of the purple arm/base member
  // Keep it proportional to the base block so it stays aligned after scale changes.
  pushMatrix();
  float markerThickness = constrain(baseBlockW * 0.08f, 0.8f, 2.2f);
  float markerHeight = max(8.0f, baseBlockH * 0.56f);
  float markerDepth = max(4.0f, baseBlockD * 0.66f);
  float markerFaceOffset = 0.12f;
  translate((baseBlockW * 0.5f) + (markerThickness * 0.5f) + markerFaceOffset, 0, 0);
  fill(255, 247, 0);
  box(markerThickness, markerHeight, markerDepth);
  safePopMatrix("Manipulator3D.pde:504");

  translate(0, -(baseBlockH / 2.0f), 0);
  pushMatrix();
  rotateZ(radians(getUpperArmModelZDeg()));
  translate(0, upperArmYOffset, 0);       // center of the upperArm
  fill(200, 100, 100);
  box(upperArmW, upperArmH, upperArmD);

  translate(0, -(upperArmH / 2.0f), 0);
  pushMatrix();
  rotateZ(radians(getForearmModelZDeg()));
  translate(0, forearmYOffset, 0);        // center of the forearm
  fill(100, 200, 100);
  box(forearmW, forearmH, forearmD);

  translate(0, -(forearmH / 2.0f), 0);
  rotateY(radians(getForearmRollModelYDeg()));
  pushMatrix();
  rotateZ(radians(getWristVerticalModelZDeg()));
  translate(0, wristVerticalYOffset, 0);   // center of the vertical wrist
  fill(100, 100, 200);
  box(wristVerticalW, wristVerticalH, wristVerticalD);

  translate(0, -(wristVerticalH / 2.0f), 0);

  // Wrist rotation
  rotateY(radians(getWristRotationModelYDeg()));

  // Gripper
  pushMatrix();
  translate(0, gripperYOffset, 0);

  // Right finger
  pushMatrix();
  translate(-fingerXOffset, 0, 0);
  rotateZ(radians(-getGripperFingerModelZDeg()));
  translate(0, -getFingerPivotOffsetScene(), 0);
  fill(200);
  box(gripperFingerW, gripperFingerH, gripperFingerD);
  safePopMatrix("Manipulator3D.pde:544");

  // Draws the left gripper finger.
  pushMatrix();
  translate(fingerXOffset, 0, 0);
  rotateZ(radians(getGripperFingerModelZDeg()));
  translate(0, -getFingerPivotOffsetScene(), 0);
  fill(200);
  box(gripperFingerW, gripperFingerH, gripperFingerD);
  safePopMatrix("Manipulator3D.pde:553");
  noLights();

  // Restores the manipulator joint transform stack.
  safePopMatrix("Manipulator3D.pde:557"); // Claw base
  safePopMatrix("Manipulator3D.pde:558"); // Vertical wrist
  safePopMatrix("Manipulator3D.pde:559"); // Forearm
  safePopMatrix("Manipulator3D.pde:560"); // Upper arm
  safePopMatrix("Manipulator3D.pde:561"); // Purple base

  noLights();

  if (manipulatorCollisionDebugVisible && isManipulatorSelfCollisionGuardActive()) drawCollidersDebug();
  drawEnvironmentCollisionPoints();
  drawManipulatorSonarProbe();
  if (trace)        drawTrajectoryGhosting();
  if (showCurrentTipMarker) drawCurrentGripperTipMarker();

  safePopMatrix("Manipulator3D.pde:570");

  if (frameCount % 10 == 0) recordTrajectoryPoint();
}

//===========================================================
// COLLISION DETECTION
//===========================================================

ArrayList<Collider> colliders = new ArrayList<Collider>();
float collisionTolerance = 1.0f;
boolean manipulatorCollisionDebugVisible = true;

// Draws the local collision volumes used by the manipulator guard.
void drawCollidersDebug() {
  for (Collider c : colliders) c.draw();
}

/**
 * Transforms a point from local model space to world space.
 */
// Utility: model to world.
PVector modelToWorld(float x, float y, float z) {
  return new PVector(modelX(x, y, z), modelY(x, y, z), modelZ(x, y, z));
}

/**
 * Builds an Oriented Bounding Box (OBB) collider from local dimensions.
 * (ex,ey,ez = half-extents calculated from w,h,d)
 */
// Builds OBB.
OBB buildOBB(float w, float h, float d, ColliderType type) {
  PVector c = modelToWorld(0, 0, 0);
  PVector ex = PVector.sub(modelToWorld(1, 0, 0), c).normalize();
  PVector ey = PVector.sub(modelToWorld(0, 1, 0), c).normalize();
  PVector ez = PVector.sub(modelToWorld(0, 0, 1), c).normalize();
  return new OBB(c, ex, ey, ez, w * 0.5f, h * 0.5f, d * 0.5f, type);
}

/**
 * Collects all colliders for collision detection based on the current joint positions.
 * — OBBs centered in the same frame as the meshes (use mesh H for positioning/joints)
 * — OBB length already reduced (0.7) in updateDimensions()
 */
// Utility: collect colliders.
void collectColliders() {
  colliders.clear();

  // --- 1) GROUND (OBB with half-extents) ---
  colliders.add(new OBB(
    new PVector(0, GROUND_Y + groundColliderH * 0.5f, 0),
    new PVector(1, 0, 0),
    new PVector(0, 1, 0),
    new PVector(0, 0, 1),
    groundColliderW * 0.5f,
    groundColliderH * 0.5f,
    groundColliderD * 0.5f,
    ColliderType.GROUND
    ));

  // --- 2) BASE CYLINDER (uses YMin/YMax already calculated) ---
  colliders.add(new CylinderY(
    0, // cx
    0, // cz
    baseCylinderColliderR,
    baseCylinderColliderYMin,
    baseCylinderColliderYMax,
    ColliderType.BASE_CYLINDER
    ));

  // --- 3) PURPLE BASE (centered at baseBlockYOffset) ---
  pushMatrix();
  translate(0, baseBlockYOffset, 0);
  rotateY(radians(getBaseModelYawDeg()));
  // Mesh center == current point -> create OBB already centered
  colliders.add(buildOBB(baseBlockColliderW, baseBlockColliderH, baseBlockColliderD, ColliderType.BASE_BLOCK));

  // Move to the top-of-base joint (same as draw)
  translate(0, -(baseBlockH * 0.5f), 0);

  // --- 4) UPPER ARM (center at upperArmYOffset) ---
  pushMatrix();
  rotateZ(radians(getUpperArmModelZDeg()));
  translate(0, upperArmYOffset, 0);  // == geometric center of the arm
  colliders.add(buildOBB(upperArmColliderW, upperArmColliderH, upperArmColliderD, ColliderType.UPPER_ARM));

  // Move to the top-of-arm joint
  translate(0, -(upperArmH * 0.5f), 0);

  // --- 5) FOREARM ---
  pushMatrix();
  rotateZ(radians(getForearmModelZDeg()));
  translate(0, forearmYOffset, 0);   // center of the forearm
  colliders.add(buildOBB(forearmColliderW, forearmColliderH, forearmColliderD, ColliderType.FOREARM));

  // Move to the top-of-forearm joint
  translate(0, -(forearmH * 0.5f), 0);
  rotateY(radians(getForearmRollModelYDeg()));

  // --- 6) WRIST VERTICAL ---
  pushMatrix();
  rotateZ(radians(getWristVerticalModelZDeg()));
  translate(0, wristVerticalYOffset, 0); // center of the vertical wrist
  colliders.add(buildOBB(wristVerticalColliderW, wristVerticalColliderH, wristVerticalColliderD, ColliderType.WRIST_VERTICAL));

  // Move to the top-of-vertical-wrist joint
  translate(0, -(wristVerticalH * 0.5f), 0);

  // --- 7) WRIST ROTATE + GRIPPER BASE ---
  rotateY(radians(getWristRotationModelYDeg())); // same sign as render
  pushMatrix();
  translate(0, gripperYOffset, 0); // gripper base (finger axis)

  // --- 8) RIGHT FINGER ---
  pushMatrix();
  translate(-fingerXOffset, 0, 0);
  rotateZ(radians(-getGripperFingerModelZDeg()));
  translate(0, -getFingerPivotOffsetScene(), 0); // center of the finger
  colliders.add(buildOBB(gripperFingerColliderW, gripperFingerColliderH, gripperFingerColliderD, ColliderType.FINGER_R));
  safePopMatrix("Manipulator3D.pde:688");

  // --- 9) LEFT FINGER ---
  pushMatrix();
  translate(fingerXOffset, 0, 0);
  rotateZ(radians(getGripperFingerModelZDeg()));
  translate(0, -getFingerPivotOffsetScene(), 0); // center of the finger
  colliders.add(buildOBB(gripperFingerColliderW, gripperFingerColliderH, gripperFingerColliderD, ColliderType.FINGER_L));
  safePopMatrix("Manipulator3D.pde:696");

  // Restores the manipulator joint transform stack in draw-order sequence
  safePopMatrix("Manipulator3D.pde:699"); // closes gripper base
  safePopMatrix("Manipulator3D.pde:700"); // closes vertical wrist
  safePopMatrix("Manipulator3D.pde:701"); // closes forearm
  safePopMatrix("Manipulator3D.pde:702"); // closes upper arm
  safePopMatrix("Manipulator3D.pde:703"); // closes purple base
}

/**
 * Checks for collisions between all pairs of colliders.
 * @return `true` if any collision is found, `false` otherwise.
 */
// Utility: are adjacent manipulator collider types.
boolean areAdjacentManipulatorColliderTypes(ColliderType a, ColliderType b) {
  return
    (a == ColliderType.GROUND && b == ColliderType.BASE_BLOCK) ||
    (a == ColliderType.BASE_BLOCK && b == ColliderType.GROUND) ||
    (a == ColliderType.BASE_CYLINDER && b == ColliderType.BASE_BLOCK) ||
    (a == ColliderType.BASE_BLOCK && b == ColliderType.BASE_CYLINDER) ||
    (a == ColliderType.BASE_BLOCK && b == ColliderType.UPPER_ARM) ||
    (a == ColliderType.UPPER_ARM && b == ColliderType.BASE_BLOCK) ||
    (a == ColliderType.UPPER_ARM && b == ColliderType.FOREARM) ||
    (a == ColliderType.FOREARM && b == ColliderType.UPPER_ARM) ||
    (a == ColliderType.FOREARM && b == ColliderType.WRIST_VERTICAL) ||
    (a == ColliderType.WRIST_VERTICAL && b == ColliderType.FOREARM) ||
    (a == ColliderType.WRIST_VERTICAL && b == ColliderType.FINGER_R) ||
    (a == ColliderType.FINGER_R && b == ColliderType.WRIST_VERTICAL) ||
    (a == ColliderType.WRIST_VERTICAL && b == ColliderType.FINGER_L) ||
    (a == ColliderType.FINGER_L && b == ColliderType.WRIST_VERTICAL);
}

// Utility: should ignore collision pair.
boolean shouldIgnoreCollisionPair(ColliderType a, ColliderType b) {
  if ((a == ColliderType.FINGER_R && b == ColliderType.FINGER_L) ||
    (a == ColliderType.FINGER_L && b == ColliderType.FINGER_R)) return true;

  if ((a == ColliderType.GROUND && b == ColliderType.BASE_CYLINDER) ||
    (a == ColliderType.BASE_CYLINDER && b == ColliderType.GROUND)) return true;

  return areAdjacentManipulatorColliderTypes(a, b);
}

// Utility: check collisions.
boolean checkCollisions() {
  for (int i = 0; i < colliders.size(); i++) {
    for (int j = i + 1; j < colliders.size(); j++) {
      Collider c1 = colliders.get(i);
      Collider c2 = colliders.get(j);

      if (shouldIgnoreCollisionPair(c1.type, c2.type)) continue;
      if (c1.intersects(c2, collisionTolerance)) return true;
    }
  }
  return false;
}

// --- Enum for collider types ---
enum ColliderType {
  GROUND,
    BASE_CYLINDER,
    BASE_BLOCK,
    UPPER_ARM,
    FOREARM,
    WRIST_VERTICAL,
    FINGER_R,
    FINGER_L
}

// Base class for runtime collision volumes.
abstract class Collider {
  ColliderType type;
  abstract boolean intersects(Collider other, float tol);
  abstract void draw();
}

// Cylinder collision volume aligned with the world Y axis.
class CylinderY extends Collider {
  float cx, cz;
  float radius;
  float yMin, yMax;
  CylinderY(float cx, float cz, float radius, float yMin, float yMax, ColliderType type) {
    this.cx = cx;
    this.cz = cz;
    this.radius = radius;
    this.yMin = min(yMin, yMax);
    this.yMax = max(yMin, yMax);
    this.type = type;
  }
// Utility: intersects.
  boolean intersects(Collider other, float tol) {
    if (other instanceof CylinderY) {
      CylinderY o = (CylinderY) other;
      boolean yOverlap = (this.yMin + tol <= o.yMax) && (this.yMax - tol >= o.yMin);
      if (!yOverlap) return false;
      float dx = this.cx - o.cx, dz = this.cz - o.cz;
      float r = this.radius + o.radius + tol;
      return dx * dx + dz * dz <= r * r;
    } else if (other instanceof OBB) {
      return ColliderUtil.cylinderOBBIntersects(this, (OBB) other, tol);
    }
    return false;
  }
// Utility: draw.
  void draw() {
    pushMatrix();
    pushStyle();
    translate(cx, (yMin + yMax) * 0.5f, cz);
    noFill();
    stroke(255, 80, 80, 185);
    strokeWeight(0.72f);
    int sides = 36;
    float h = (yMax - yMin);
    float y0 = -h / 2, y1 = h / 2;
    beginShape();
    for (int i = 0; i < sides; i++) {
      float a = TWO_PI * i / sides;
      vertex(radius * cos(a), y0, radius * sin(a));
    }
    endShape(CLOSE);
    beginShape();
    for (int i = 0; i < sides; i++) {
      float a = TWO_PI * i / sides;
      vertex(radius * cos(a), y1, radius * sin(a));
    }
    endShape(CLOSE);
    for (int i = 0; i < sides; i += 6) {
      float a = TWO_PI * i / sides;
      beginShape();
      vertex(radius * cos(a), y0, radius * sin(a));
      vertex(radius * cos(a), y1, radius * sin(a));
      endShape();
    }
    popStyle();
    safePopMatrix("Manipulator3D.pde:831");
  }
}

// Oriented bounding box used by the collision solver.
class OBB extends Collider {
  PVector c;
  PVector ux, uy, uz;
  float ex, ey, ez;
  OBB(PVector c, PVector ux, PVector uy, PVector uz, float ex, float ey, float ez, ColliderType type) {
    this.c = c.copy();
    this.ux = ux.copy();
    this.uy = uy.copy();
    this.uz = uz.copy();
    this.ex = ex;
    this.ey = ey;
    this.ez = ez;
    this.type = type;
  }
// Utility: project radius.
  float projectRadius(PVector axis) {
    axis.normalize();
    return ex * abs(ux.dot(axis)) + ey * abs(uy.dot(axis)) + ez * abs(uz.dot(axis));
  }
// Utility: overlaps on axis.
  boolean overlapsOnAxis(OBB o, PVector axis, float tol) {
    if (axis.magSq() < 1e-9f) return true;
    axis.normalize();
    float dist = abs(PVector.sub(o.c, this.c).dot(axis));
    float rA = this.projectRadius(axis);
    float rB = o.projectRadius(axis);
    return dist <= (rA + rB + tol);
  }
// Utility: intersects.
  boolean intersects(Collider other, float tol) {
    if (other instanceof OBB) {
      OBB o = (OBB) other;
      PVector[] axes = new PVector[] {
        ux, uy, uz,
        o.ux, o.uy, o.uz,
        ux.cross(o.ux), ux.cross(o.uy), ux.cross(o.uz),
        uy.cross(o.ux), uy.cross(o.uy), uy.cross(o.uz),
        uz.cross(o.ux), uz.cross(o.uy), uz.cross(o.uz)
      };
      for (PVector a : axes) {
        if (!overlapsOnAxis(o, a, tol)) return false;
      }
      return true;
    } else if (other instanceof CylinderY) {
      return ColliderUtil.cylinderOBBIntersects((CylinderY) other, this, tol);
    }
    return false;
  }
// Utility: draw.
  void draw() {
    pushStyle();
    stroke(255, 80, 80, 185);
    strokeWeight(0.72f);
    noFill();
    PVector[] v = new PVector[8];
    int idx = 0;
    for (int sx = -1; sx <= 1; sx += 2)
      for (int sy = -1; sy <= 1; sy += 2)
        for (int sz = -1; sz <= 1; sz += 2) {
          PVector p = PVector.add(c,
            PVector.add(PVector.mult(ux, ex * sx),
            PVector.add(PVector.mult(uy, ey * sy),
            PVector.mult(uz, ez * sz))));
          v[idx++] = p;
        }
    int[][] e = {{0, 1}, {0, 2}, {0, 4}, {1, 3}, {1, 5}, {2, 3}, {2, 6}, {3, 7}, {4, 5}, {4, 6}, {5, 7}, {6, 7}};
    beginShape(LINES);
    for (int[] ed : e) {
      vertex(v[ed[0]].x, v[ed[0]].y, v[ed[0]].z);
      vertex(v[ed[1]].x, v[ed[1]].y, v[ed[1]].z);
    }
    endShape();
    popStyle();
  }
}

// Static geometric intersection helpers.
static class ColliderUtil {
  // Tests a vertical cylinder against an oriented bounding box.
  static boolean cylinderOBBIntersects(CylinderY c, OBB b, float tol) {
    // Check vertical interval overlap.
    float centerY = b.c.y;
    PVector Y = new PVector(0, 1, 0);
    float rY = b.projectRadius(Y);
    float bMin = centerY - rY, bMax = centerY + rY;
    if (!intervalsOverlap(c.yMin, c.yMax, bMin, bMax, tol)) return false;

    // Check horizontal projected overlap.
    PVector ax = new PVector(b.ux.x, b.ux.z);
    PVector az = new PVector(b.uz.x, b.uz.z);
    float axLen = ax.mag();
    float azLen = az.mag();
    if (axLen < 1e-6f && azLen < 1e-6f) {
      float dx = b.c.x - c.cx, dz = b.c.z - c.cz;
      return (dx * dx + dz * dz) <= sq(c.radius + tol);
    }
    if (axLen > 1e-6f) ax.div(axLen);
    else ax.set(1, 0);
    if (azLen > 1e-6f) az.div(azLen);
    else az.set(0, 1);

    float hx = b.projectRadius(new PVector(ax.x, 0, ax.y));
    float hz = b.projectRadius(new PVector(az.x, 0, az.y));

    PVector cRect = new PVector(b.c.x, b.c.z);
    PVector cCircle = new PVector(c.cx, c.cz);
    PVector d = PVector.sub(cCircle, cRect);
    float localX = dot2(d, ax);
    float localZ = dot2(d, az);
    float clampedX = clamp(localX, -hx, hx);
    float clampedZ = clamp(localZ, -hz, hz);

    float dx = localX - clampedX, dz = localZ - clampedZ;
    return (dx * dx + dz * dz) <= sq(c.radius + tol);
  }

// Utility: intervals overlap.
  static boolean intervalsOverlap(float aMin, float aMax, float bMin, float bMax, float tol) {
    return (aMin + tol <= bMax) && (aMax - tol >= bMin);
  }

// Utility: dot 2.
  static float dot2(PVector a, PVector b) {
    return a.x * b.x + a.y * b.y;
  }

// Clamps the requested value.
  static float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
  }
}

//===========================================================
// TRAJECTORY & UTILITIES
//===========================================================

/**
 * Records the current world position of the gripper tip to draw a trajectory.
 */
// Utility: record trajectory point.
void recordTrajectoryPoint() {
  tip = getGripperTipWorldPos();
  if (!trace) {
    tip = null;
    trajectoryPoints = new ArrayList<PVector>();
  }
  if (tip != null) {
    trajectoryPoints.add(tip);
    if (trajectoryPoints.size() > MAX_TRAJECTORY) {
      trajectoryPoints.remove(0);
    }
  }
}

/**
 * Draws the recorded trajectory points as a series of fading spheres.
 */
// Utility: draw trajectory ghosting.
void drawTrajectoryGhosting() {
  noStroke();
  for (int i = 0; i < trajectoryPoints.size(); i++) {
    float alpha = map(i, 0, trajectoryPoints.size(), 30, 200);
    fill(222, 150, 24, alpha);
    PVector p = trajectoryPoints.get(i);
    pushMatrix();
    translate(p.x, p.y, p.z);
    sphere(TRAJECTORY_POINT_RADIUS);
    safePopMatrix("Manipulator3D.pde:1003");
  }
}

// Applies manipulator chain to gripper base.
void applyManipulatorChainToGripperBase() {
  translate(0, baseBlockYOffset, 0);
  rotateY(radians(getBaseModelYawDeg()));
  translate(0, -(baseBlockH / 2.0f), 0);
  rotateZ(radians(getUpperArmModelZDeg()));
  translate(0, -upperArmH, 0);
  rotateZ(radians(getForearmModelZDeg()));
  translate(0, -forearmH, 0);
  rotateY(radians(getForearmRollModelYDeg()));
  rotateZ(radians(getWristVerticalModelZDeg()));
  translate(0, -wristVerticalH, 0);
  rotateY(radians(getWristRotationModelYDeg()));
  translate(0, gripperYOffset, 0);
}

// Returns finger tip world pos.
PVector getFingerTipWorldPos(boolean rightFinger) {
  pushMatrix();
  applyManipulatorChainToGripperBase();
  translate(rightFinger ? -fingerXOffset : fingerXOffset, 0, 0);
  rotateZ(radians(rightFinger ? -getGripperFingerModelZDeg() : getGripperFingerModelZDeg()));
  translate(0, -getFingerPivotOffsetScene(), 0);
  PVector fingerTip = modelToWorld(0, -(gripperFingerH * 0.5f), 0);
  safePopMatrix("Manipulator3D.pde:1031");
  return fingerTip;
}

// Utility: draw current gripper tip marker.
void drawCurrentGripperTipMarker() {
  PVector currentTip = getGripperTipWorldPos();
  if (currentTip == null) return;

  pushMatrix();
  noStroke();
  fill(255, 220, 0);
  translate(currentTip.x, currentTip.y, currentTip.z);
  sphere(GRIPPER_TIP_MARKER_RADIUS);
  safePopMatrix("Manipulator3D.pde:1045");
}

/**
 * Calculates the world position of the gripper's tip.
 */
// Returns gripper tip world pos.
PVector getGripperTipWorldPos() {
  PVector rightTip = getFingerTipWorldPos(true);
  PVector leftTip = getFingerTipWorldPos(false);
  return PVector.mult(PVector.add(rightTip, leftTip), 0.5f);
}


void drawManipulator3D() {
  drawManipulator3D(true);
}


// =====================================================================
// Manipulator diagnostics module section
// =====================================================================

boolean handleManipulatorDiagnosticsPanelMousePressed(int mx, int my, int panelX, int panelY, int contentX, int contentW, int gap, int toolsInnerX, int toolsCol2W, int neutralInnerX, int neutralCol2W) {
  if (pointInRect(mx, my, toolsInnerX, diagnosticsMapButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { toggleManipulatorLocalCollisionGuard("diagnostics panel"); return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsMapButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { clearTraceMap(); return true; }
  if (pointInRect(mx, my, toolsInnerX, diagnosticsMapButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { loadWorldMapFromWindowsDialog(); return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsMapButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { removeImportedWorldOnly(); return true; }

  if (pointInRect(mx, my, toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { toggleEnvironmentScan(); return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { toggleGimbalStab(); return true; }
  if (pointInRect(mx, my, toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { toggleArmStab(); return true; }
  if (pointInRect(mx, my, toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H)) { triggerCompassCalibration(); return true; }

  for (int i = 0; i < 7; i++) {
    int col = i % 2;
    int bx = neutralInnerX + col * (neutralCol2W + gap);
    int by = diagnosticsManipulatorNeutralButtonY(panelY, i);
    if (pointInRect(mx, my, bx, by, neutralCol2W, DIAGNOSTICS_BUTTON_H)) {
      diagnosticsTakeLocalControl();
      try {
        toggleManipulatorMemberNeutral(i);
      }
      finally {
        diagnosticsEndLocalControl();
      }
      return true;
    }
  }

  int autoTorqueX = neutralInnerX + neutralCol2W + gap;
  int autoTorqueY = diagnosticsManipulatorNeutralButtonY(panelY, GRIPPER_IDX);
  if (pointInRect(mx, my, autoTorqueX, autoTorqueY, neutralCol2W, DIAGNOSTICS_BUTTON_H)) {
    diagnosticsTakeLocalControl();
    try {
      toggleManipulatorAutoPwmPowerControl();
    }
    finally {
      diagnosticsEndLocalControl();
    }
    return true;
  }

  if (handleManipulatorSliderInteraction(mx, my)) return true;
  return true;
}

void drawManipulatorDiagnosticsPanelSections(int panelY, int contentX, int contentW, int gap, int toolsInnerX, int toolsCol2W, int neutralInnerX, int neutralCol2W, int sensorsContentX, int sensorsContentW, int slidersContentX, int slidersContentW, int manipRuntimeCardY, int manipNeutralCardY, int manipSensorsCardY, int manipForceCardY) {
  drawSectionCard2D(contentX, manipRuntimeCardY, contentW, DIAGNOSTICS_MANIP_RUNTIME_CARD_H, tr("Ferramentas de runtime", "Runtime tools"));
  drawMiniButton2D(toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Varredura do ambiente", "Environment scan"), currentEnvironmentAutoScanEnabled());
  drawMiniButton2D(toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Estabilizador da garra", "Gripper stabilizer"), gimbalStabEnabled);
  drawMiniButton2D(toolsInnerX, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Estabilizador do braço", "Arm stabilizer"), armStabEnabled);
  drawMiniButton2D(toolsInnerX + toolsCol2W + gap, diagnosticsRuntimeButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H, compassCalibrationActive() ? tr("Cal. bússola *", "Compass cal *") : tr("Cal. bússola", "Compass cal"), compassCalibrationActive());

  drawSectionCard2D(contentX, manipNeutralCardY, contentW, DIAGNOSTICS_MANIP_NEUTRAL_CARD_H, tr("Neutro por junta", "Neutral buttons by joint"));
  for (int i = 0; i < 7; i++) {
    int col = i % 2;
    int bx = neutralInnerX + col * (neutralCol2W + gap);
    int by = diagnosticsManipulatorNeutralButtonY(panelY, i);
    drawMiniButton2D(bx, by, neutralCol2W, DIAGNOSTICS_BUTTON_H, manipulatorNeutralLabel(i), isManipulatorMemberNeutral(i));
  }
  drawMiniButton2D(neutralInnerX + neutralCol2W + gap, diagnosticsManipulatorNeutralButtonY(panelY, GRIPPER_IDX), neutralCol2W, DIAGNOSTICS_BUTTON_H, tr("Auto torque", "Auto torque"), manipulatorAutoPwmPowerControlEnabled());

  drawSectionCard2D(contentX, manipSensorsCardY, contentW, DIAGNOSTICS_MANIP_SENSORS_CARD_H, tr("Sensores de estabilidade do manipulador", "Manipulator stability sensors"));
  drawManipulatorStabilityTargets2D(sensorsContentX, manipSensorsCardY + 32, sensorsContentW, DIAGNOSTICS_MANIP_SENSORS_CARD_H - 44);

  drawSectionCard2D(contentX, manipForceCardY, contentW, DIAGNOSTICS_MANIP_FORCE_CARD_H, tr("Força auxiliar das juntas", "Joint auxiliary force"));
  drawDiagnosticsSlider2D(slidersContentX, diagnosticsManipulatorSliderDrawY(panelY, 0), slidersContentW, tr("Força da base", "Base force"), 12, dutyTargetPercentages[0], dutyPercentages[0]);
  drawDiagnosticsSlider2D(slidersContentX, diagnosticsManipulatorSliderDrawY(panelY, 1), slidersContentW, tr("Força do braço superior", "Upper arm force"), 13, dutyTargetPercentages[1], dutyPercentages[1]);
  drawDiagnosticsSlider2D(slidersContentX, diagnosticsManipulatorSliderDrawY(panelY, 2), slidersContentW, tr("Força do antebraço", "Forearm force"), 14, dutyTargetPercentages[2], dutyPercentages[2]);
  drawDiagnosticsSlider2D(slidersContentX, diagnosticsManipulatorSliderDrawY(panelY, 3), slidersContentW, tr("Força rotacional do antebraço", "Forearm rotational force"), 15, dutyTargetPercentages[3], dutyPercentages[3]);
  fill(uiPrimaryTextColor());
  drawDiagnosticsFittedTextLeft(diagnosticsHasActiveSensorTelemetry() ? (tr("Corrente do grupo do punho: ", "Wrist group current: ") + nf(ina1mA, 1, 1) + " / " + nf(ina2mA, 1, 1) + " mA") : tr("Corrente do grupo do punho: -- / -- mA", "Wrist group current: -- / -- mA"), slidersContentX, manipForceCardY + 180, slidersContentW);
}

// =====================================================================
// SynRovModule adapter: Manipulator3D
// =====================================================================
class Manipulator3DModule implements SynRovModule {
  public String id() { return "Manipulator"; }
  public String displayName() { return robotDisplayName("Manipulator"); }
  public boolean available() { return true; }

  public boolean supports(String capability) {
    return MODULE_CAP_DIAGNOSTICS.equals(capability) ||
      MODULE_CAP_ENVIRONMENT.equals(capability) ||
      MODULE_CAP_RUNTIME.equals(capability) ||
      MODULE_CAP_COLLISION.equals(capability) ||
      MODULE_CAP_TRAJECTORY.equals(capability) ||
      MODULE_CAP_GRIPPER.equals(capability);
  }

  public void setupModule() { }

  public void loadConfigSafe() {
    try {
      loadOrCreateManipulatorConfig();
    }
    catch (Exception e) {
      println("Manipulator config recovery: " + e.getMessage());
      backupBrokenConfigFile(MANIPULATOR3D);
      saveJSONObjectEnsured(buildManipulatorConfigObject(), MANIPULATOR3D);
      try { loadConfig(MANIPULATOR3D); }
      catch (Exception ignore) { updateDimensions(); }
    }
  }

  public void updateModule() { }
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
    setCommandContext(playbackCommandSource);
    int manipCount = min(frame.length, GRIPPER_IDX + 1);
    for (int i = 0; i < manipCount; i++) {
      setAngle(mapIndexToChannel(i), frame[i]);
    }
    restoreLocalCommandContext();
  }

  public boolean moveSafelyTowardFrameSafe(int[] targetFrame, int maxStepDeg, int toleranceDeg) {
    if (targetFrame == null) return true;
    setCommandContext(playbackCommandSource);
    int manipCount = min(min(targetFrame.length, angles.length), servoLimits.length);
    for (int i = 0; i < manipCount; i++) {
      int current = angles[i];
      int target = normalizeManipulatorTargetValue(i, targetFrame[i]);
      int delta = target - current;
      if (abs(delta) > toleranceDeg) {
        int step = constrain(delta, -maxStepDeg, maxStepDeg);
        setAngle(mapIndexToChannel(i), normalizeManipulatorTargetValue(i, current + step));
      }
    }
    restoreLocalCommandContext();
    return isManipulatorFrameWithinTolerance(targetFrame, toleranceDeg);
  }

  public JSONObject buildConfigObjectSafe() {
    return buildManipulatorConfigObject();
  }
}
