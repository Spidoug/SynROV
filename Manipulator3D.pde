//===========================================================
// CONFIGURATION & SETUP
//===========================================================

final float GROUND_EPS = 0.1f;      // Small elevation to prevent z-fighting during rendering
final float BASE_GAP = 0.0f;        // Optional gap between the top of the cylinder and the purple base

/**
 * Loads the configuration from "config3D.json". If the file doesn't exist,
 * it creates a new one with default values and then loads it.
 */
void loadOrCreateConfig() {

  if (fileExists(MANIPULATOR3D)) {
    println("Configuration file found. Loading data...");
    loadConfig(MANIPULATOR3D);
  } else {
    println("Configuration file not found. Creating with default values...");
    createDefaultConfig(MANIPULATOR3D);
    loadConfig(MANIPULATOR3D);
  }
}

/**
 * Checks if a file exists in the sketch's data folder.
 */
boolean fileExists(String filename) {
  File f = new File(sketchPath(filename));
  return f.exists() && f.isFile();
}

/**
 * Loads values from the JSON file into global variables and then updates
 * all related dimensions and offsets.
 */
void loadConfig(String filename) {
  JSONObject json = loadJSONObject(filename);

  float SCALE_3D = 2.8;

  JSONObject armDims = json.getJSONObject("armDimensions");
  baseCylinderRadius = armDims.getJSONObject("baseCylinder").getFloat("radius") * SCALE_3D;
  baseCylinderHeight = armDims.getJSONObject("baseCylinder").getFloat("height") * SCALE_3D;

  baseBlockW = armDims.getJSONObject("baseBlock").getFloat("width") * SCALE_3D;
  baseBlockH = armDims.getJSONObject("baseBlock").getFloat("height") * SCALE_3D;
  baseBlockD = armDims.getJSONObject("baseBlock").getFloat("depth") * SCALE_3D;

  upperArmW = armDims.getJSONObject("upperArm").getFloat("width") * SCALE_3D;
  upperArmH = armDims.getJSONObject("upperArm").getFloat("height") * SCALE_3D;
  upperArmD = armDims.getJSONObject("upperArm").getFloat("depth") * SCALE_3D;

  forearmW = armDims.getJSONObject("forearm").getFloat("width") * SCALE_3D;
  forearmH = armDims.getJSONObject("forearm").getFloat("height") * SCALE_3D;
  forearmD = armDims.getJSONObject("forearm").getFloat("depth") * SCALE_3D;

  wristVerticalW = armDims.getJSONObject("wristVertical").getFloat("width") * SCALE_3D;
  wristVerticalH = armDims.getJSONObject("wristVertical").getFloat("height") * SCALE_3D;
  wristVerticalD = armDims.getJSONObject("wristVertical").getFloat("depth") * SCALE_3D;

  gripperFingerW = armDims.getJSONObject("gripper").getFloat("fingerWidth") * SCALE_3D;
  gripperFingerH = armDims.getJSONObject("gripper").getFloat("fingerHeight") * SCALE_3D;
  gripperFingerD = armDims.getJSONObject("gripper").getFloat("fingerDepth") * SCALE_3D;

  // Collider dimensions/offsets are derived in updateDimensions()
  updateDimensions();
}

/**
 * Creates a new JSON file with the default configuration values.
 */
void createDefaultConfig(String filename) {
  JSONObject json = new JSONObject();

  // Arm dimensions
  JSONObject armDims = new JSONObject();

  JSONObject baseCyl = new JSONObject();
  baseCyl.setFloat("radius", 20);
  baseCyl.setFloat("height", 42);
  armDims.setJSONObject("baseCylinder", baseCyl);

  JSONObject baseBlock = new JSONObject();
  baseBlock.setFloat("width", 10);
  baseBlock.setFloat("height", 14.5);
  baseBlock.setFloat("depth", 10);
  armDims.setJSONObject("baseBlock", baseBlock);

  JSONObject upperArm = new JSONObject();
  upperArm.setFloat("width", 10);
  upperArm.setFloat("height", 22);
  upperArm.setFloat("depth", 6.5);
  armDims.setJSONObject("upperArm", upperArm);

  JSONObject forearm = new JSONObject();
  forearm.setFloat("width", 6);
  forearm.setFloat("height", 12);
  forearm.setFloat("depth", 5.5);
  armDims.setJSONObject("forearm", forearm);

  JSONObject wristVert = new JSONObject();
  wristVert.setFloat("width", 4);
  wristVert.setFloat("height", 12);
  wristVert.setFloat("depth", 5);
  armDims.setJSONObject("wristVertical", wristVert);

  JSONObject gripper = new JSONObject();
  gripper.setFloat("fingerWidth", 2);
  gripper.setFloat("fingerHeight", 10);
  gripper.setFloat("fingerDepth", 1.7);
  armDims.setJSONObject("gripper", gripper);

  json.setJSONObject("armDimensions", armDims);

  // Ground collider dimensions
  JSONObject colliderDims = new JSONObject();
  JSONObject groundCol = new JSONObject();
  groundCol.setFloat("width", 460.0);
  groundCol.setFloat("height", 1.0);
  groundCol.setFloat("depth", 460.0);
  colliderDims.setJSONObject("ground", groundCol);
  json.setJSONObject("colliderDimensions", colliderDims);

  saveJSONObject(json, filename);
}

void updateDimensions() {
  // Ground dimensions (unchanged)
  groundColliderW = 460.0f;
  groundColliderH = 1.0f;
  groundColliderD = 460.0f;

  // === IMPORTANT: In P3D, Y+ is DOWN. To be "above the ground," we use NEGATIVE Y values. ===

  // 1) Center the cylinder so it is supported on the ground:
  //    - The ground plane is at GROUND_Y (0)
  //    - We want the ENTIRE cylinder above the ground, so the center is UP (negative Y)
  //    - Center = GROUND_Y - (H_ground/2) - (H_cyl/2)
  baseCylinderYOffset = GROUND_Y - (groundColliderH * 0.5f) - (baseCylinderHeight * 0.5f);

  // 2) Top of the cylinder (in Y): since up is NEGATIVE, the top is even more negative
  float cylinderTopY = baseCylinderYOffset - (baseCylinderHeight * 0.5f);

  // 3) Purple base: center exactly above the top of the cylinder (+ optional gap)
  //    "Above" = more negative.
  baseBlockYOffset = cylinderTopY - (baseBlockH * 0.5f) - BASE_GAP;

  // 4) "Central" offsets of the segments (drawing at the geometric center)
  upperArmYOffset      = -upperArmH        / 2.0f;
  forearmYOffset       = -forearmH         / 2.0f;
  wristVerticalYOffset = -wristVerticalH   / 2.0f;
  gripperYOffset       = -6.4f;
  fingerXOffset        = -6.4f;

  // 5) Cylinder collider: calculated using the correct center
  baseCylinderColliderR    = baseCylinderRadius;
  baseCylinderColliderYMin = baseCylinderYOffset - (baseCylinderHeight * 0.5f); // more negative (top)
  baseCylinderColliderYMax = baseCylinderYOffset + (baseCylinderHeight * 0.5f); // less negative (bottom)

  baseBlockColliderW = baseBlockW;
  baseBlockColliderH = baseBlockH;
  baseBlockColliderD = baseBlockD;

  upperArmColliderW = upperArmW;
  upperArmColliderH = upperArmH * 0.5f;
  upperArmColliderD = upperArmD;

  forearmColliderW = forearmW;
  forearmColliderH = forearmH * 0.5f;
  forearmColliderD = forearmD;

  wristVerticalColliderW = wristVerticalW;
  wristVerticalColliderH = wristVerticalH * 0.8f;
  wristVerticalColliderD = wristVerticalD;

  gripperFingerColliderW = gripperFingerW;
  gripperFingerColliderH = gripperFingerH;
  gripperFingerColliderD = gripperFingerD;
}

//===========================================================
// VISUALIZATION & DRAWING
//===========================================================

/**
 * Draws a 3D cylinder centered at the origin, with a given radius and height.
 * @param r The radius of the cylinder.
 * @param h The height of the cylinder.
 */
void drawCylinder(float r, float h) {
  int sides = 30;
  float ang, x, z;
  float halfH = h / 2.0f;
  noStroke();
  // top cap
  beginShape(TRIANGLE_FAN);
  vertex(0, -halfH, 0);
  for (int i = 0; i <= sides; i++) {
    ang = TWO_PI * i / sides;
    x = cos(ang) * r;
    z = sin(ang) * r;
    vertex(x, -halfH, z);
  }
  endShape();
  // sides
  beginShape(QUAD_STRIP);
  for (int i = 0; i <= sides; i++) {
    ang = TWO_PI * i / sides;
    x = cos(ang) * r;
    z = sin(ang) * r;
    vertex(x, -halfH, z);
    vertex(x, halfH, z);
  }
  endShape();
  // bottom cap
  beginShape(TRIANGLE_FAN);
  vertex(0, halfH, 0);
  for (int i = 0; i <= sides; i++) {
    ang = TWO_PI * i / sides;
    x = cos(ang) * r;
    z = sin(ang) * r;
    vertex(x, halfH, z);
  }
  endShape();
}

/**
 * Draws the 3D model of the robotic arm, including all its joints and links.
 */
void drawManipulator3D() {

  pushMatrix();
  translate(width / 2 + 120, height / 2 + 80);
  rotateX(cameraRotationX);
  rotateY(cameraRotationY);
  cameraRotationY += cameraRotationYIncrement;
  cameraRotationX += cameraRotationXIncrement;

  if (width <= 1100 && height <= 650) zoomLevel = 1;
  if (zoomLevel != 1) {
    PVector center = new PVector(width / 2 + 120, height / 2 + 80);
    PVector offset = PVector.sub(zoomTarget, center);
    translate(offset.x, offset.y);
    scale(zoomLevel);
    translate(-offset.x, -offset.y);
  }

  noStroke();
  noFill();

  // ---------- Ground (y = GROUND_Y) ----------
  pushMatrix();
  translate(0, GROUND_Y + GROUND_EPS, 0); // elevates by 0.1 to avoid z-fighting
  if (groundLogo != null) {
    pushMatrix();
    rotateX(HALF_PI);
    imageMode(CENTER);
    image(groundLogo, 0, 0, groundColliderW, groundColliderD);
    popMatrix();
  }
  popMatrix();

  lights();

  // ---------- Base cylinder (centered at baseCylinderYOffset) ----------
  pushMatrix();
  fill(80);
  translate(0, baseCylinderYOffset, 0);
  drawCylinder(baseCylinderRadius, baseCylinderHeight);

  // Yellow marker
  pushMatrix();
  float baseRadius = baseCylinderRadius;
  float markOut = 0.6f;
  rotateY(radians(angles[BASE_IDX] - 90));
  translate(0, 0, baseRadius + markOut);
  fill(255, 247, 0);
  box(8f, 20f, 1f);
  popMatrix();

  // ---------- Purple base resting on top of the cylinder ----------
  popMatrix(); // closes the cylinder's pushMatrix
  pushMatrix();
  translate(0, baseBlockYOffset, 0);
  rotateY(radians(angles[BASE_IDX]));
  fill(173, 55, 247);
  box(baseBlockW, baseBlockH, baseBlockD);

  translate(0, -(baseBlockH / 2.0f), 0);
  pushMatrix();
  rotateZ(radians(angles[UPPERARM_IDX] - 90));
  translate(0, upperArmYOffset, 0);       // center of the upperArm
  fill(200, 100, 100);
  box(upperArmW, upperArmH, upperArmD);

  translate(0, -(upperArmH / 2.0f), 0);
  pushMatrix();
  rotateZ(radians(angles[FOREARM_IDX] - 180));
  translate(0, forearmYOffset, 0);        // center of the forearm
  fill(100, 200, 100);
  box(forearmW, forearmH, forearmD);

  translate(0, -(forearmH / 2.0f), 0);
  pushMatrix();
  rotateZ(radians(angles[WRIST_VERT_IDX] - 90));
  translate(0, wristVerticalYOffset, 0);   // center of the vertical wrist
  fill(100, 100, 200);
  box(wristVerticalW, wristVerticalH, wristVerticalD);

  translate(0, -(wristVerticalH / 2.0f), 0);

  // Wrist rotation
  rotateY(radians(angles[WRIST_ROT_IDX] - 90));

  // Gripper
  pushMatrix();
  translate(0, gripperYOffset, 0);

  // Right finger
  pushMatrix();
  translate(6.4f, 0, 0);
  rotateZ(radians(-(angles[GRIPPER_IDX] - 140)));
  translate(0, -6.4f, 0);
  fill(200);
  box(gripperFingerW, gripperFingerH, gripperFingerD);
  popMatrix();

  // Left finger
  pushMatrix();
  translate(-6.4f, 0, 0);
  rotateZ(radians(angles[GRIPPER_IDX] - 140));
  translate(0, -6.4f, 0);
  fill(200);
  box(gripperFingerW, gripperFingerH, gripperFingerD);
  popMatrix();
  noLights();

  // pops
  popMatrix(); // Claw base
  popMatrix(); // Vertical wrist
  popMatrix(); // Forearm
  popMatrix(); // Upper arm
  popMatrix(); // Purple base

  noLights();

  if (collisionSet) drawCollidersDebug();
  if (trace)        drawTrajectoryGhosting();

  popMatrix();

  drawEnvironmentPoints();
  if (frameCount % 10 == 0) recordTrajectoryPoint();
}

//===========================================================
// COLLISION DETECTION
//===========================================================

ArrayList<Collider> colliders = new ArrayList<Collider>();
float collisionTolerance = 1.0f; // Example global variable for tolerance

void drawCollidersDebug() {
  for (Collider c : colliders) c.draw();
}

/**
 * Transforms a point from local model space to world space.
 */
PVector modelToWorld(float x, float y, float z) {
  return new PVector(modelX(x, y, z), modelY(x, y, z), modelZ(x, y, z));
}

/**
 * Builds an Oriented Bounding Box (OBB) collider from local dimensions.
 * (ex,ey,ez = half-extents calculated from w,h,d)
 */
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
  rotateY(radians(angles[BASE_IDX]));
  // Mesh center == current point -> create OBB already centered
  colliders.add(buildOBB(baseBlockColliderW, baseBlockColliderH, baseBlockColliderD, ColliderType.BASE_BLOCK));

  // Move to the top-of-base joint (same as draw)
  translate(0, -(baseBlockH * 0.5f), 0);

  // --- 4) UPPER ARM (center at upperArmYOffset) ---
  pushMatrix();
  rotateZ(radians(angles[UPPERARM_IDX] - 90));
  translate(0, upperArmYOffset, 0);  // == geometric center of the arm
  colliders.add(buildOBB(upperArmColliderW, upperArmColliderH, upperArmColliderD, ColliderType.UPPER_ARM));

  // Move to the top-of-arm joint
  translate(0, -(upperArmH * 0.5f), 0);

  // --- 5) FOREARM ---
  pushMatrix();
  rotateZ(radians(angles[FOREARM_IDX] - 180));
  translate(0, forearmYOffset, 0);   // center of the forearm
  colliders.add(buildOBB(forearmColliderW, forearmColliderH, forearmColliderD, ColliderType.FOREARM));

  // Move to the top-of-forearm joint
  translate(0, -(forearmH * 0.5f), 0);

  // --- 6) WRIST VERTICAL ---
  pushMatrix();
  rotateZ(radians(angles[WRIST_VERT_IDX] - 90));
  translate(0, wristVerticalYOffset, 0); // center of the vertical wrist
  colliders.add(buildOBB(wristVerticalColliderW, wristVerticalColliderH, wristVerticalColliderD, ColliderType.WRIST_VERTICAL));

  // Move to the top-of-vertical-wrist joint
  translate(0, -(wristVerticalH * 0.5f), 0);

  // --- 7) WRIST ROTATE + GRIPPER BASE ---
  rotateY(radians(angles[WRIST_ROT_IDX] - 90)); // same sign as render
  pushMatrix();
  translate(0, gripperYOffset, 0); // gripper base (finger axis)

  // --- 8) RIGHT FINGER ---
  pushMatrix();
  translate( 6.4f, 0, 0);
  rotateZ(radians(-(angles[GRIPPER_IDX] - 140)));
  translate(0, -6.4f, 0); // center of the finger
  colliders.add(buildOBB(gripperFingerColliderW, gripperFingerColliderH, gripperFingerColliderD, ColliderType.FINGER_R));
  popMatrix();

  // --- 9) LEFT FINGER ---
  pushMatrix();
  translate(-6.4f, 0, 0);
  rotateZ(radians( (angles[GRIPPER_IDX] - 140)));
  translate(0, -6.4f, 0); // center of the finger
  colliders.add(buildOBB(gripperFingerColliderW, gripperFingerColliderH, gripperFingerColliderD, ColliderType.FINGER_L));
  popMatrix();

  // pops stacked in the same order as the drawing
  popMatrix(); // closes gripper base
  popMatrix(); // closes vertical wrist
  popMatrix(); // closes forearm
  popMatrix(); // closes upper arm
  popMatrix(); // closes purple base
}

/**
 * Checks for collisions between all pairs of colliders.
 * @return `true` if any collision is found, `false` otherwise.
 */
boolean checkCollisions() {
  for (int i = 0; i < colliders.size(); i++) {
    for (int j = i + 1; j < colliders.size(); j++) {
      Collider c1 = colliders.get(i);
      Collider c2 = colliders.get(j);

      // ignore finger/finger and ground/baseCylinder
      if ((c1.type == ColliderType.FINGER_R && c2.type == ColliderType.FINGER_L) ||
        (c1.type == ColliderType.FINGER_L && c2.type == ColliderType.FINGER_R)) continue;

      if ((c1.type == ColliderType.GROUND && c2.type == ColliderType.BASE_CYLINDER) ||
        (c1.type == ColliderType.BASE_CYLINDER && c2.type == ColliderType.GROUND)) continue;

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

// --- Abstract base class for all colliders ---
abstract class Collider {
  ColliderType type;
  abstract boolean intersects(Collider other, float tol);
  abstract void draw(); // Debug drawing
}

// --- CylinderY: A collider representing a cylinder aligned with the Y-axis ---
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
  void draw() {
    pushMatrix();
    translate(cx, (yMin + yMax) * 0.5f, cz);
    noFill();
    stroke(255, 0, 0); // Red
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
    popMatrix();
  }
}

// --- OBB: Oriented Bounding Box, used for collision detection ---
class OBB extends Collider {
  PVector c; // Center
  PVector ux, uy, uz; // Unit axes in world space
  float ex, ey, ez; // Half-extents
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
  float projectRadius(PVector axis) {
    axis.normalize();
    return ex * abs(ux.dot(axis)) + ey * abs(uy.dot(axis)) + ez * abs(uz.dot(axis));
  }
  boolean overlapsOnAxis(OBB o, PVector axis, float tol) {
    if (axis.magSq() < 1e-9f) return true;
    axis.normalize();
    float dist = abs(PVector.sub(o.c, this.c).dot(axis));
    float rA = this.projectRadius(axis);
    float rB = o.projectRadius(axis);
    return dist <= (rA + rB + tol);
  }
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
  void draw() {
    stroke(255, 0, 0);
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
  }
}

// --- Static utility class for collision checks ---
static class ColliderUtil {
  // Cylinder-Y vs OBB intersection test
  static boolean cylinderOBBIntersects(CylinderY c, OBB b, float tol) {
    // 1) Y overlap
    float centerY = b.c.y;
    PVector Y = new PVector(0, 1, 0);
    float rY = b.projectRadius(Y);
    float bMin = centerY - rY, bMax = centerY + rY;
    if (!intervalsOverlap(c.yMin, c.yMax, bMin, bMax, tol)) return false;

    // 2) Overlap on XZ
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

  static boolean intervalsOverlap(float aMin, float aMax, float bMin, float bMax, float tol) {
    return (aMin + tol <= bMax) && (aMax - tol >= bMin);
  }

  static float dot2(PVector a, PVector b) {
    return a.x * b.x + a.y * b.y;
  }

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
void drawTrajectoryGhosting() {
  noStroke();
  for (int i = 0; i < trajectoryPoints.size(); i++) {
    float alpha = map(i, 0, trajectoryPoints.size(), 30, 200);
    fill(222, 150, 24, alpha);
    PVector p = trajectoryPoints.get(i);
    pushMatrix();
    translate(p.x, p.y, p.z);
    sphere(2);
    popMatrix();
  }
}

/**
 * Calculates the world position of the gripper's tip.
 */
PVector getGripperTipWorldPos() {
  pushMatrix();
  translate(0, baseCylinderYOffset, 0);
  translate(0, baseBlockYOffset - baseCylinderYOffset, 0);
  rotateY(radians(angles[BASE_IDX]));
  translate(0, -(baseBlockH / 2.0f), 0);
  rotateZ(radians(angles[UPPERARM_IDX] - 90));
  translate(0, -upperArmH, 0);
  rotateZ(radians(angles[FOREARM_IDX] - 180));
  translate(0, -forearmH, 0);
  rotateZ(radians(angles[WRIST_VERT_IDX] - 90));
  translate(0, -wristVerticalH, 0);
  rotateY(radians(angles[WRIST_ROT_IDX] - 90));
  translate(0, -6.4f, 0);
  PVector tip = modelToWorld(0, 0, 0);
  popMatrix();
  return tip;
}
