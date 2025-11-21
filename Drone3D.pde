// ===========================
// CONFIGURAÇÃO GERAL
// ===========================
final float GROUND_SIZE = 480; // Largura/Profundidade do chão (baseado no grid de -240 a 240)
final float MAX_ALTITUDE = GROUND_SIZE * 3; // Altura máxima de voo (3x a largura do chão)

// ===========================
// DRONE – parameters & state
// ===========================

// Novo tamanho do drone: 10% do tamanho do chão
final float DRONE_SIZE_FACTOR = 0.10f;
final float DRONE_BODY_DIM = GROUND_SIZE * DRONE_SIZE_FACTOR;

final float DRONE_BODY_L = DRONE_BODY_DIM; // Profundidade
final float DRONE_BODY_W = DRONE_BODY_DIM; // Largura
final float DRONE_BODY_H = DRONE_BODY_DIM * 0.25f; // Altura

final float ARM_LENGTH = DRONE_BODY_DIM * 0.8f;
final float ARM_THICKNESS = DRONE_BODY_DIM * 0.08f;
final float ARM_ANGLE_DEG = 45;

final float PROP_RADIUS = DRONE_BODY_DIM * 0.3f;
final float PROP_HEIGHT = 3;
final float MOTOR_HEIGHT = DRONE_BODY_DIM * 0.15f;
final float MOTOR_RADIUS = DRONE_BODY_DIM * 0.1f;

// Variáveis de Posição e Orientação do Drone
float droneX = 0, droneZ = 0;
float droneY = GROUND_Y;
float droneYaw = 0;
float droneRoll = 0;
float dronePitch = 0;
float propSpeed = 0.0f;

// NOVAS VARIÁVEIS DE ESTADO (Física)
PVector velocity = new PVector(0, 0, 0); // Velocidade
PVector acceleration = new PVector(0, 0, 0); // Aceleração

float rollRate = 0;
float pitchRate = 0;

// CONSTANTES DE FÍSICA
final float GRAVITY = 0.5f;       // Simulação de gravidade
final float DRAG = 0.05f;         // Arrasto/Fricção simulada
final float RESTORE_FORCE = 0.05f; // Força para voltar à posição plana (Roll/Pitch)
final float MAX_THRUST_FORCE = 15.0f; // Força máxima do motor simulada

void drawPropeller(float rotationAngle) {
  pushMatrix();
  rotateY(rotationAngle);
  noStroke();
  fill(100);
  box(PROP_RADIUS * 2, PROP_HEIGHT, PROP_RADIUS * 0.4f);
  rotateY(HALF_PI);
  box(PROP_RADIUS * 2, PROP_HEIGHT, PROP_RADIUS * 0.4f);
  popMatrix();
}

void drawMotor() {
  fill(50);
  noStroke();
  pushMatrix();
  rotateX(HALF_PI);
  drawCylinder(MOTOR_RADIUS, MOTOR_HEIGHT); // Assumindo drawCylinder está definido
  popMatrix();
}

// ===========================
// DRONE PHYSICS UPDATE
// (Deve ser chamado antes de drawDrone3D() no seu loop draw())
// ===========================
void updateDronePhysics() {
  // === Inputs de Controle ===
  // Nota: Substitua esta lógica pela leitura real dos seus ângulos de controle (angles[])
  // Assumimos que angles[1] (Throttle) é usado para empuxo e angles[3] (Roll), angles[2] (Pitch) para inclinação.
  
  // Exemplo de mapeamento: Throttle de 0-180 para Empuxo de -1.0 a 1.0 (ajustado para hover)
  float controlThrustNorm = map(angles[1], 0, 180, -1.0f, 1.0f);
  // Exemplo de mapeamento: Roll/Pitch de 0-180 para Ângulo de inclinação desejado
  float targetRoll = map(angles[3], 0, 180, radians(-15), radians(15));
  float targetPitch = map(angles[2], 0, 180, radians(-15), radians(15));
  
  // === 1. Dinâmica da Atitude (Roll e Pitch) ===
  
  // Aplica a força de restauração (volta para 0 se não houver input)
  float rollError = targetRoll - droneRoll;
  float pitchError = targetPitch - dronePitch;
  
  rollRate += rollError * RESTORE_FORCE;
  pitchRate += pitchError * RESTORE_FORCE;
  
  // Aplica Arrasto Rotacional (amortecimento)
  rollRate *= 0.85; 
  pitchRate *= 0.85; 

  // Atualiza os ângulos
  droneRoll += rollRate;
  dronePitch += pitchRate;
  
  // === 2. Dinâmica de Posição ===
  
  acceleration.set(0, 0, 0);

  // Gravidade
  acceleration.y += GRAVITY; 

  // Empuxo (Lift)
  float effectiveThrust = (controlThrustNorm * MAX_THRUST_FORCE); 
  
  // O Empuxo combate a gravidade, mas é reduzido quando o drone está inclinado (Roll/Pitch)
  float liftComponent = effectiveThrust * cos(droneRoll) * cos(dronePitch);
  acceleration.y -= liftComponent;
  
  // Movimento Horizontal (resultante da inclinação)
  acceleration.x += sin(droneRoll) * effectiveThrust;
  acceleration.z -= sin(dronePitch) * effectiveThrust; 

  // Integração
  velocity.mult(1.0f - DRAG); // Aplica Arrasto Linear
  velocity.add(acceleration);
  
  droneX += velocity.x;
  droneY += velocity.y;
  droneZ += velocity.z;

  // === 3. Limites de Voo ===
  
  // Limites Horizontal (X e Z)
  float limitXY = GROUND_SIZE / 2.0f;
  
  if (droneX > limitXY) { droneX = limitXY; velocity.x = 0; }
  if (droneX < -limitXY) { droneX = -limitXY; velocity.x = 0; }
  
  if (droneZ > limitXY) { droneZ = limitXY; velocity.z = 0; }
  if (droneZ < -limitXY) { droneZ = -limitXY; velocity.z = 0; }

  // Limite Vertical (Y)
  if (droneY > GROUND_Y) { // Pouso/Chão
    droneY = GROUND_Y; 
    velocity.y = 0;
    droneRoll = 0; // Nivelar ao pousar
    dronePitch = 0;
  }
  if (droneY < GROUND_Y - MAX_ALTITUDE) { // Teto de voo (Altitude máxima)
    droneY = GROUND_Y - MAX_ALTITUDE;
    velocity.y = 0; 
  }

  // Atualização da Velocidade da Hélice (Visual)
  propSpeed += map(abs(controlThrustNorm), 0.0f, 1.0f, 0.5f, 2.0f);
}

// ===========================
// DRONE DRAWING (complete e atualizado)
// ===========================
void drawDrone3D() {
  // -------------------------------------------------
  // CAMERA / ZOOM (Código Existente)
  // -------------------------------------------------
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

  // -------------------------------------------------
  // GROUND (Código Existente)
  // -------------------------------------------------
  pushMatrix();
  translate(0, GROUND_Y + GROUND_EPS, 0);
  if (groundLogo != null) {
    pushMatrix();
    rotateX(HALF_PI);
    imageMode(CENTER);
    image(groundLogo, 0, 0, 460, 460);
    popMatrix();
  } else {
    pushMatrix();
    stroke(40);
    for (int i = -6; i <= 6; i++) {
      line(-240, 0, i * 40, 240, 0, i * 40);
      line(i * 40, 0, -240, i * 40, 0, 240);
    }
    popMatrix();
  }
  popMatrix();

  lights();

  // -------------------------------------------------
  // DRONE
  // -------------------------------------------------
  pushMatrix();
  translate(droneX, droneY, droneZ);
  rotateY(droneYaw);
  rotateX(dronePitch);
  rotateZ(droneRoll);

  fill(0, 100, 150);
  noStroke();
  box(DRONE_BODY_W, DRONE_BODY_H, DRONE_BODY_L);
  
  // --- MARCADOR FRONTAL ---
  pushMatrix();
  // Posiciona à frente do corpo do drone
  translate(0, 0, DRONE_BODY_L / 2.0f + 5); 
  fill(255, 50, 50); // Vermelho
  box(DRONE_BODY_W * 0.2f, DRONE_BODY_H * 0.5f, 10); // Pequeno marcador
  popMatrix();

  // ======================
  // Arms + Motors + Props
  // ======================
  final float ARM_TOTAL = (DRONE_BODY_L / 2.0f) + ARM_LENGTH;

  float[] anglesDeg = new float[] {
    -ARM_ANGLE_DEG,
    ARM_ANGLE_DEG,
    180 + ARM_ANGLE_DEG,
    180 - ARM_ANGLE_DEG
  };

  int[] propDir = new int[] { +1, -1, -1, +1 };

  for (int i = 0; i < 4; i++) {
    pushMatrix();
    rotateY(radians(anglesDeg[i]));

    // --- ARM ---
    fill(50, 50, 50);
    pushMatrix();
    translate(0, 0, ARM_TOTAL / 2.0f);
    box(ARM_THICKNESS, ARM_THICKNESS, ARM_TOTAL);
    popMatrix();

    // --- MOTOR ---
    pushMatrix();
    translate(0,
      -DRONE_BODY_H / 2.0f - MOTOR_HEIGHT / 2.0f,
      ARM_TOTAL);
    drawMotor();

    // --- PROPELLER ---
    translate(0, -MOTOR_HEIGHT / 2.0f - PROP_HEIGHT / 2.0f, 0);
    drawPropeller(propSpeed * propDir[i]);
    popMatrix();
    popMatrix();
  }

  popMatrix();
  popMatrix();
  noLights();

   propSpeed += 0.5f; // Removido daqui, agora é atualizado em updateDronePhysics()
}

// ==============================
// DRONE – canais de controle (NOVO)
// ==============================
// use estes 6 índices no lugar dos *_IDX do manipulador
final int YAW_IDX    = 0;  // giro (°)
final int PITCH_IDX  = 1;  // inclinação X (°)
final int ROLL_IDX   = 2;  // inclinação Z (°)
final int ALT_IDX    = 3;  // altitude (Y)
final int STRAFE_IDX = 4;  // deslocamento X (lateral)
final int FWD_IDX    = 5;  // deslocamento Z (frente/trás)

// limites e velocidades (ajuste fino)
final float YAW_SPEED_DEG   = 2.5f;
final float PITCH_SPEED_DEG = 2.0f;
final float ROLL_SPEED_DEG  = 2.0f;
final float ALT_SPEED_UNITS = 2.0f;  // Y no seu mundo
final float XY_SPEED_UNITS  = 2.0f;  // X/Z

// limites “safe”
final float YAW_MIN=-180, YAW_MAX=180;
final float PITCH_MIN=-45, PITCH_MAX=45;
final float ROLL_MIN=-45, ROLL_MAX=45;
final float ALT_MIN=-300, ALT_MAX=50;    // Y negativo é “alto”
final float X_MIN=-400, X_MAX=400;
final float Z_MIN=-400, Z_MAX=400;

// captura o estado atual do drone em um frame int[6]
int[] captureDroneFrame() {
  return new int[] {
    Math.round(degrees(droneYaw)),
    Math.round(degrees(dronePitch)),
    Math.round(degrees(droneRoll)),
    Math.round(droneY),
    Math.round(droneX),
    Math.round(droneZ)
  };
}

// aplica um frame int[6] no estado do drone
void applyDroneFrame(int[] f) {
  droneYaw   = radians(constrain(f[YAW_IDX],   (int)YAW_MIN,   (int)YAW_MAX));
  dronePitch = radians(constrain(f[PITCH_IDX], (int)PITCH_MIN, (int)PITCH_MAX));
  droneRoll  = radians(constrain(f[ROLL_IDX],  (int)ROLL_MIN,  (int)ROLL_MAX));
  droneY     = constrain(f[ALT_IDX],           (int)ALT_MIN,   (int)ALT_MAX);
  droneX     = constrain(f[STRAFE_IDX],        (int)X_MIN,     (int)X_MAX);
  droneZ     = constrain(f[FWD_IDX],           (int)Z_MIN,     (int)Z_MAX);
}
