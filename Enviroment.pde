ArrayList<PVector> environmentPoints = new ArrayList<PVector>();

void applyPalmTransformations() {
  translate(0, -36, 0);
  rotateY(radians(angles[0]));
  translate(0, -64, 0);
  rotateZ(radians(angles[1] - 90));
  translate(0, -38.4, 0);
  rotateZ(radians(angles[2] - 180));
  translate(0, -35, 0);
  rotateZ(radians(angles[3] - 90));
  translate(0, -16, 0);
  rotateY(radians(angles[4] - 90));
  translate(0, -6.4, 0);
}

PVector getPalmPositionWorld() {
  pushMatrix();
  applyPalmTransformations();
  PVector tip = modelToWorld(0, 0, -16);
  popMatrix();
  return tip;
}

PVector getPalmForwardDirection() {
  pushMatrix();
  applyPalmTransformations();
  PVector origin = modelToWorld(0, 0, -16);
  PVector forward = modelToWorld(0, 0, -17).sub(origin).normalize();
  popMatrix();
  return forward;
}

void updateEnvironmentFromSonar() {
  if (latestSensors == null) return;
  if (latestSensors.hasKey("ex1_0")) {
    int dist = latestSensors.getInt("ex1_0");
    if (dist <= 0 || dist > 400) return;
    PVector palmPos = getPalmPositionWorld();
    PVector dir = getPalmForwardDirection();
    PVector point = PVector.add(palmPos, PVector.mult(dir, dist));
    environmentPoints.add(point);
    if (environmentPoints.size() > 1000) {
      environmentPoints.remove(0);
    }
  }
}

void drawEnvironmentPoints() {
  stroke(0, 200, 255);
  strokeWeight(3);
  for (PVector p : environmentPoints) {
    point(p.x, p.y, p.z);
  }
}

void displaySonarDistanceCaption() {
  if (latestSensors == null) {
    fill(255, 150, 0);
    textSize(16);
    textAlign(LEFT, TOP);
    float displayX = 10;
    float displayY = 10;
    text("Waiting for sensor data...", displayX, displayY);
    return;
  }
  if (latestSensors.hasKey("ex1_0")) {
    int dist = latestSensors.getInt("ex1_0");
    fill(255);
    textSize(16);
    textAlign(LEFT, TOP);
    float displayX = 10;
    float displayY = 10;
    if (dist > 0 && dist <= 400) {
      text("Object Distance: " + dist + " cm", displayX, displayY);
    } else {
      text("Object Distance: Out of Range / No Object Detected", displayX, displayY);
    }
  }
}
