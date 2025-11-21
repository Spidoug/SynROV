// ===========================
// VEHICLE – parâmetros & estado (drop-in)
// ===========================
final float VEH_BODY_L = 220;   // Comprimento da carroceria
final float VEH_BODY_W = 100;   // Largura da carroceria
final float VEH_BODY_H = 10;    // Altura da carroceria
final float AXLE_CLEAR = 18;    // Folga entre corpo e eixo
final float WHEEL_R    = 30;    // Raio das rodas motrizes
final float WHEEL_W    = 15;    // Largura das rodas
final float TRACK_GAUGE  = 115; // Distância entre centros das esteiras
final float WHEELBASE_Z  = 180; // Distância (eixo-a-eixo) Z
final float IDLER_WHEEL_R = 20; // Raio das rodas guia (menores)
final int   IDLER_COUNT   = 3;  // Qtde de rodas guia por lado
final float IDLER_SPACING = WHEELBASE_Z / (IDLER_COUNT + 1.0f);

final int   TRACK_LINKS   = 60;   // Mais links = curva mais suave
final float TRACK_LINK_L  = 8;
final float TRACK_LINK_W  = 15;
final float TRACK_THICK   = 3;

// Aparência / física da esteira
final int   SPROCKET_TEETH     = 12;   // dentes no pinhão
final float TOOTH_DEPTH        = 4;
final float TOOTH_THICK        = 5;
final float TRACK_SAG          = 6;    // "barriga" no topo
final int   TOP_ROLLERS        = 2;    // roletes superiores
final float TOP_ROLLER_R       = 10;   // raio dos roletes sup.
final float TRACK_LINK_GROUSER = 2.5f; // “taco” do link

float vehicleX = 0, vehicleZ = 0;
float vehicleYaw = 0;
float vehicleHeight = 0;
float vehicleSpeed = 0.0f; // px/s ao longo do chão
float wheelAngle = 0.0f;
long  _lastTickMs = 0;

// ===========================
// HELPERS (reutiliza seu drawCylinder(...))
// ===========================

// roda lisa simples (para idlers / roletes)
void drawWheel(float r, float w, float spinRad) {
  pushMatrix();
  rotateX(HALF_PI);       // roda no plano YZ
  rotateZ(spinRad);       // rotação (animação)
  fill(50);
  stroke(20);
  strokeWeight(1);
  drawCylinder(r, w);
  popMatrix();
}

// pinhão motriz com dentes visuais
void drawSprocket(float r, float w, float spinRad, int teeth, float toothDepth, float toothThick) {
  pushMatrix();
  rotateX(HALF_PI);
  rotateZ(spinRad);

  // disco base (sem dentes)
  noStroke();
  fill(45);
  drawCylinder(r - toothDepth, w);

  // dentes
  stroke(20);
  strokeWeight(1);
  fill(60);
  for (int i = 0; i < teeth; i++) {
    float a0 = TWO_PI * i / teeth;
    pushMatrix();
    rotateZ(a0);
    translate(0, r - toothDepth * 0.5f, 0);
    box(toothThick, toothDepth, w * 0.9f);
    popMatrix();
  }

  // flanges finas para acabamento
  noStroke();
  fill(35);
  pushMatrix(); translate(0, 0, -w * 0.55f); cylinderRing(r, w * 0.1f); popMatrix();
  pushMatrix(); translate(0, 0,  w * 0.55f); cylinderRing(r, w * 0.1f); popMatrix();

  popMatrix();
}

void cylinderRing(float r, float thickness) {
  drawCylinder(r, thickness);
}

// ===========================
// ESTEIRA – geometria e pose
// ===========================
class LinkPose {
  PVector p;  // posição local (x=0)
  float yaw;  // rotação no eixo X (alinha à tangente YZ)
  LinkPose(PVector p, float yaw) { this.p = p; this.yaw = yaw; }
}

// comprimento total do caminho (retos + dois semicírculos)
float trackPathLength(float driveR, float wheelbaseZ) {
  return 2 * wheelbaseZ + 2 * PI * driveR;
}

// perfil de "sag" (barriga) no trecho superior: parabólica invertida
float sagProfile(float t) { // t ∈ [0..1]
  return -4f * (t - 0.5f) * (t - 0.5f) + 1f; // 0..1 com máximo no centro
}

// retorna pose do link na fração u do caminho
LinkPose getTrackLinkPose(float u, float driveR, float wheelbaseZ) {
  float zRear  = -wheelbaseZ * 0.5f;
  float zFront =  wheelbaseZ * 0.5f;
  float straight = wheelbaseZ;
  float arc      = PI * driveR;      // cada semicírculo
  float L        = 2 * straight + 2 * arc;

  float s = (u - floor(u)) * L;      // distância ao longo do caminho
  PVector pos = new PVector(0, 0, 0);
  float yawX = 0;

  // 1) Reto inferior: rear -> front (y = -R)
  if (s < straight) {
    float t = s / straight;
    pos.z = lerp(zRear, zFront, t);
    pos.y = -driveR;
    yawX = 0;
    return new LinkPose(pos, yawX);
  }
  s -= straight;

  // 2) Semicírculo frontal (de baixo p/ cima): 0..PI
  if (s < arc) {
    float ang = s / driveR; // 0..PI
    pos.z = zFront + driveR * sin(ang);
    pos.y =           driveR * cos(ang);
    // tangente: dy/dz -> atan2(dy, dz), derivadas com sinal correto
    yawX = atan2(-sin(ang), cos(ang));
    return new LinkPose(pos, yawX);
  }
  s -= arc;

  // 3) Reto superior com SAG: front -> rear (y ≈ +R - sag)
  if (s < straight) {
    float t = s / straight;                 // 0..1 indo front->rear
    pos.z = lerp(zFront, zRear, t);
    float baseY = driveR;
    float sag   = TRACK_SAG * sagProfile(t);
    pos.y = baseY - sag;

    // yaw por diferenças finitas (estimativa da tangente)
    float eps = 0.001f;
    float z2 = lerp(zFront, zRear, min(1, t + eps));
    float y2 = driveR - TRACK_SAG * sagProfile(min(1, t + eps));
    float dz = z2 - pos.z;
    float dy = y2 - pos.y;
    yawX = atan2(dy, dz);
    return new LinkPose(pos, yawX);
  }
  s -= straight;

  // 4) Semicírculo traseiro (de cima p/ baixo): PI..0
  float ang = PI - (s / driveR); // decresce PI->0
  pos.z = zRear - driveR * sin(ang);
  pos.y =         driveR * cos(ang);
  // ajuste de sinal p/ sentido do caminho
  yawX = atan2(-sin(ang), -cos(ang));
  return new LinkPose(pos, yawX);
}

// desenha toda a correia/esteira (links)
void drawTrackBelt(float driveR, float wheelbaseZ, float spinRad) {
  noStroke();
  float L = trackPathLength(driveR, wheelbaseZ);

  // offset linear: quanto andou ao longo do caminho
  float path_offset = (spinRad * driveR) / L;

  for (int i = 0; i < TRACK_LINKS; i++) {
    float u = ((float)i / TRACK_LINKS + path_offset) % 1.0f;
    if (u < 0) u += 1.0f;

    LinkPose pose = getTrackLinkPose(u, driveR, wheelbaseZ);

    pushMatrix();
    translate(0, pose.p.y, pose.p.z);
    // link desenhado com eixo longo em Y -> rodamos em X para alinhar à tangente
    rotateX(pose.yaw + HALF_PI);

    // placa base do link
    pushMatrix();
    fill(30);
    box(TRACK_LINK_L, TRACK_LINK_W, TRACK_THICK);
    popMatrix();

    // “grouser” (taco) para tração
    pushMatrix();
    translate(0, TRACK_LINK_W * 0.25f, 0);
    fill(20);
    box(TRACK_LINK_L * 0.9f, TRACK_LINK_GROUSER, TRACK_THICK * 1.6f);
    popMatrix();

    popMatrix();
  }
}

// ===========================
// DESENHO DO VEÍCULO
// ===========================
void drawVehicle3D() {
  // delta-time e rotação da roda coerente com a velocidade
  long now = millis();
  if (_lastTickMs == 0) _lastTickMs = now;
  float dt = (now - _lastTickMs) / 1000.0f;
  _lastTickMs = now;

  float omega = (WHEEL_R > 1e-6f) ? (vehicleSpeed / WHEEL_R) : 0; // rad/s
  wheelAngle = (wheelAngle + omega * dt) % TWO_PI;

  pushMatrix();
  // === sua câmera/zoom (usa variáveis já existentes no seu sketch)
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

  // chão com logo (se existir)
  pushMatrix();
  translate(0, GROUND_Y + GROUND_EPS, 0);
  if (groundLogo != null) {
    pushMatrix();
    rotateX(HALF_PI);
    imageMode(CENTER);
    image(groundLogo, 0, 0, 460, 460);
    popMatrix();
  }
  popMatrix();

  lights();

  // alturas globais
  float axleYWorld = GROUND_Y - WHEEL_R - vehicleHeight;           // centro das rodas motrizes
  float bodyCenterY = axleYWorld - AXLE_CLEAR - (VEH_BODY_H * 0.5f);

  // corpo do veículo
  pushMatrix();
  translate(vehicleX, bodyCenterY, vehicleZ);
  rotateY(vehicleYaw);

  // 1) Chassi principal
  fill(80);
  stroke(50);
  strokeWeight(1);
  box(VEH_BODY_L, VEH_BODY_H, VEH_BODY_W);

  // 2) Cabine/estrutura superior
  pushMatrix();
  translate(VEH_BODY_L * 0.3f, -(VEH_BODY_H * 0.5f + 25), 0);
  fill(100, 100, 150);
  box(80, 50, 80);

  // sensor/câmera frontal
  pushMatrix();
  translate(40, 0, 0);
  fill(180, 0, 0);
  sphere(5);
  popMatrix();
  popMatrix();

  // 3) Saias laterais / chanfrado frontal
  // Saias protegem a esteira
  pushMatrix();
  noStroke();
  fill(70);
  for (int side = -1; side <= 1; side += 2) {
    pushMatrix();
    translate(side * (TRACK_GAUGE * 0.5f + WHEEL_W * 0.6f), -VEH_BODY_H * 0.1f, 0);
    box(6, VEH_BODY_H * 1.6f, WHEELBASE_Z * 1.05f);
    popMatrix();
  }
  popMatrix();

  // Chanfrado frontal (nariz)
  pushMatrix();
  translate(VEH_BODY_L * 0.5f - 18, -(VEH_BODY_H * 0.3f), 0);
  rotateZ(radians(-18));
  fill(85);
  box(36, 8, VEH_BODY_W * 0.7f);
  popMatrix();

  // 4) Rodas/Esteiras
  float yWheelLocal = (axleYWorld - bodyCenterY); // eixo das rodas no espaço local do chassi
  float zRear  = -WHEELBASE_Z * 0.5f;
  float zFront =  WHEELBASE_Z * 0.5f;

  for (int side = -1; side <= 1; side += 2) {
    float xSide = side * TRACK_GAUGE * 0.5f; // deslocamento lateral da esteira

    pushMatrix();
    translate(xSide, yWheelLocal, 0);

    // Roda motriz frontal (pinhão)
    pushMatrix();
    translate(0, 0, zFront);
    drawSprocket(WHEEL_R, WHEEL_W, wheelAngle, SPROCKET_TEETH, TOOTH_DEPTH, TOOTH_THICK);
    popMatrix();

    // Roda "motriz"/tensionadora traseira (pode usar o mesmo pinhão p/ visual coeso)
    pushMatrix();
    translate(0, 0, zRear);
    drawSprocket(WHEEL_R, WHEEL_W, -wheelAngle, SPROCKET_TEETH, TOOTH_DEPTH, TOOTH_THICK);
    popMatrix();

    // Rodas guia (road wheels) inferiores
    float idlerYOffset = -(WHEEL_R - IDLER_WHEEL_R);
    for (int i = 0; i < IDLER_COUNT; i++) {
      float zIdler = zRear + IDLER_SPACING * (i + 1);
      pushMatrix();
      translate(0, idlerYOffset, zIdler);
      drawWheel(IDLER_WHEEL_R, WHEEL_W * 0.8f, -wheelAngle);
      popMatrix();
    }

    // Roletes superiores de retorno (mantêm o topo no lugar)
    for (int r = 0; r < TOP_ROLLERS; r++) {
      float t = (r + 1f) / (TOP_ROLLERS + 1f);
      float zRoll = lerp(zFront, zRear, t);
      float yRoll = (WHEEL_R) - TRACK_SAG + 2; // levemente abaixo do topo
      pushMatrix();
      translate(0, yRoll - (WHEEL_R - TOP_ROLLER_R), zRoll);
      drawWheel(TOP_ROLLER_R, WHEEL_W * 0.6f, -wheelAngle * 0.3f);
      popMatrix();
    }

    // Esteira (links)
    pushMatrix();
    drawTrackBelt(WHEEL_R, WHEELBASE_Z, wheelAngle);
    popMatrix();

    popMatrix(); // fim de um lado
  }

  popMatrix(); // fim do chassi

  popMatrix(); // fim da cena
  noLights();
}
