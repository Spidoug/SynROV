final int MPU_FILTER_WINDOW_SIZE = 15;
final float ACC_NORM = 16384.0;
final float ACC_SCALE = 80;

ArrayList<Float> axReadings = new ArrayList<Float>();
ArrayList<Float> ayReadings = new ArrayList<Float>();
ArrayList<Float> azReadings = new ArrayList<Float>();

float smoothedAX = 0, smoothedAY = 0, smoothedAZ = 0;
PGraphics mpuHUD;

float calculateMovingAverage(ArrayList<Float> buffer, float newVal) {
  buffer.add(newVal);
  if (buffer.size() > MPU_FILTER_WINDOW_SIZE) buffer.remove(0);
  float sum = 0;
  for (float v : buffer) sum += v;
  return sum / buffer.size();
}

void drawSmoothedVector(PGraphics pg, float ax, float ay, float az, float scale) {
  pg.stroke(255, 180, 0);
  pg.strokeWeight(4);
  float sx = ax * scale;
  float sy = -ay * scale;
  float sz = az * scale;
  pg.line(0, 0, 0, sx, sy, sz);

  pg.pushMatrix();
  pg.translate(sx, sy, sz);
  pg.noStroke();
  pg.fill(255, 180, 0);
  pg.sphere(6);
  pg.popMatrix();
}

void drawLabeledAxis(PGraphics pg, float len) {
  pg.strokeWeight(2);
  pg.textAlign(CENTER, CENTER);
  pg.textSize(12);
  pg.textFont(createFont("Arial", 12));

  // X - Red
  pg.stroke(255, 70, 70);
  pg.line(0, 0, 0, len, 0, 0);
  pg.pushMatrix();
  pg.translate(len + 10, 0, 0);
  pg.fill(255, 70, 70);
  pg.text("X", 0, 0);
  pg.popMatrix();

  // Y - Green
  pg.stroke(70, 255, 70);
  pg.line(0, 0, 0, 0, -len, 0);
  pg.pushMatrix();
  pg.translate(0, -len - 10, 0);
  pg.fill(70, 255, 70);
  pg.text("Y", 0, 0);
  pg.popMatrix();

  // Z - Blue
  pg.stroke(70, 70, 255);
  pg.line(0, 0, 0, 0, 0, len);
  pg.pushMatrix();
  pg.translate(0, 0, len + 10);
  pg.fill(70, 70, 255);
  pg.text("Z", 0, 0);
  pg.popMatrix();
}

void drawMpu3dHud(float x, float y, float w, float h) {

  if (latestSensors != null) {
    if (latestSensors.hasKey("ax")) ax = latestSensors.getInt("ax");
    if (latestSensors.hasKey("ay")) ay = latestSensors.getInt("ay");
    if (latestSensors.hasKey("az")) az = latestSensors.getInt("az");
  }

  if (mpuHUD == null || mpuHUD.width != w || mpuHUD.height != h) {
    mpuHUD = createGraphics(int(w), int(h), P3D);
  }

  mpuHUD.beginDraw();
  mpuHUD.clear();
  mpuHUD.hint(DISABLE_DEPTH_MASK);

  mpuHUD.translate(w / 2, h / 2 + 10, -90);
  mpuHUD.rotateX(radians(-25));
  mpuHUD.rotateY(radians(30));

  mpuHUD.noFill();
  mpuHUD.stroke(150, 150, 150, 180);
  mpuHUD.strokeWeight(1.5);
  mpuHUD.box(50);

  drawLabeledAxis(mpuHUD, 70);

  float nax = ax / ACC_NORM;
  float nay = ay / ACC_NORM;
  float naz = az / ACC_NORM;

  smoothedAX = calculateMovingAverage(axReadings, nax);
  smoothedAY = calculateMovingAverage(ayReadings, nay);
  smoothedAZ = calculateMovingAverage(azReadings, naz);

  drawSmoothedVector(mpuHUD, smoothedAX, smoothedAY, smoothedAZ, ACC_SCALE);

  mpuHUD.pushMatrix();
  mpuHUD.translate(0, -60, 0);
  mpuHUD.rotateX(radians(25));
  mpuHUD.rotateY(radians(-30));
  mpuHUD.textAlign(CENTER, CENTER);
  mpuHUD.textFont(createFont("Arial Bold", 14));
  mpuHUD.fill(150);
  mpuHUD.text("Gripper", 0, 0);
  mpuHUD.popMatrix();

  mpuHUD.endDraw();
  image(mpuHUD, x, y);
}
