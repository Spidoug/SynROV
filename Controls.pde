// ==============================
// Input Events
// ==============================

// ==============================
// Keyboard
// ==============================

void keyPressed() {

  char k = Character.toLowerCase(key);

  // ==============================
  // Keyboard → ARM
  // ==============================

  if (isManipulatorSelected) {
    if (k >= '1' && k <= '5') {
      handleActionButton(k - '1');
      return;
    }
    if (isPlaying()) {
      switch (k) {
      case 'c':
      case 'n':
      case 'o':
      case 'l':
      case 'j':
        break;
      default:
        return;
      }
    }
    switch (k) {
    case 'a':
      setKeyboardIncrement(BASE_IDX, 1);
      break;
    case 'd':
      setKeyboardIncrement(BASE_IDX, -1);
      break;
    case 's':
      setKeyboardIncrement(UPPERARM_IDX, 1);
      break;
    case 'w':
      setKeyboardIncrement(UPPERARM_IDX, -1);
      break;
    case 'f':
      setKeyboardIncrement(FOREARM_IDX, 1);
      break;
    case 'r':
      setKeyboardIncrement(FOREARM_IDX, -1);
      break;
    case 'g':
      setKeyboardIncrement(WRIST_VERT_IDX, 1);
      break;
    case 't':
      setKeyboardIncrement(WRIST_VERT_IDX, -1);
      break;
    case 'q':
      setKeyboardIncrement(WRIST_ROT_IDX, 1);
      break;
    case 'e':
      setKeyboardIncrement(WRIST_ROT_IDX, -1);
      break;
    case 'z':
      setKeyboardIncrement(GRIPPER_IDX, 1);
      break;
    case 'x':
      setKeyboardIncrement(GRIPPER_IDX, -1);
      break;
    case 'c':
      if (systemReady || simulationMode) {
        collisionSet = !collisionSet;
        updateMessage("Collision detection: " + (collisionSet ? "ON" : "OFF"));
      } else {
        updateMessage("Cannot toggle collision");
      }
      break;
    case 'n':
      connect();
      break;
    case 'o':
      trace = !trace;
      updateMessage("Trace: " + (trace ? "ON" : "OFF"));
      break;
    case 'l':
      toggleLeapMotion();
      break;
    case 'j':
      toggleJoystick();
      break;
    }
    if (key == CODED && mouseX >= 300) {
      if (keyCode == UP) cameraRotationXIncrement = -0.02;
      else if (keyCode == DOWN) cameraRotationXIncrement = 0.02;
      if (keyCode == LEFT) cameraRotationYIncrement = -0.02;
      else if (keyCode == RIGHT) cameraRotationYIncrement = 0.02;
    }
    if (width > 1100 && height > 650 && mouseX >= 300) {
      if (key == '+') {
        zoomLevel *= KEYBOARD_ZOOM_FACTOR;
        zoomLevel = constrain(zoomLevel, MIN_ZOOM, MAX_ZOOM);
        zoomTarget.set(width / 2 + 120, height / 2 + 80, 0);
      }
      if (key == '-') {
        zoomLevel /= KEYBOARD_ZOOM_FACTOR;
        zoomLevel = constrain(zoomLevel, MIN_ZOOM, MAX_ZOOM);
        zoomTarget.set(width / 2 + 120, height / 2 + 80, 0);
      }
    }
  }

  // ==============================
  // Keyboard → DRONE
  // ==============================

  if (isDroneSelected) {
    // 1..5: como já existe
    if (k >= '1' && k <= '5') {
      handleActionButton(k - '1');
      return;
    }

    if (isPlaying()) {
      switch (k) {
      case 'c':
      case 'n':
      case 'o':
      case 'l':
      case 'j':
        break;
      default:
        return;
      }
    }

    switch (k) {
      // YAW (giro): A/D
    case 'a':
      setKeyboardIncrement(YAW_IDX, +1);
      break;
    case 'd':
      setKeyboardIncrement(YAW_IDX, -1);
      break;

      // PITCH (nariz sobe/desce): W/S
    case 'w':
      setKeyboardIncrement(PITCH_IDX, -1);
      break; // nariz desce (pitch negativo)
    case 's':
      setKeyboardIncrement(PITCH_IDX, +1);
      break; // nariz sobe

      // ROLL (inclinação lateral): R/F
    case 'r':
      setKeyboardIncrement(ROLL_IDX, -1);
      break; // rola p/ esquerda
    case 'f':
      setKeyboardIncrement(ROLL_IDX, +1);
      break; // rola p/ direita

      // ALTITUDE (Y): T/G
    case 't':
      setKeyboardIncrement(ALT_IDX, -1);
      break; // sobe (Y mais negativo)
    case 'g':
      setKeyboardIncrement(ALT_IDX, +1);
      break; // desce

      // STRAFE X: Q/E
    case 'q':
      setKeyboardIncrement(STRAFE_IDX, -1);
      break; // esquerda
    case 'e':
      setKeyboardIncrement(STRAFE_IDX, +1);
      break; // direita

      // FORWARD/BACK Z: Z/X
    case 'z':
      setKeyboardIncrement(FWD_IDX, -1);
      break;  // frente (Z-)
    case 'x':
      setKeyboardIncrement(FWD_IDX, +1);
      break;  // trás (Z+)

      // Toggles e conexão (iguais ao seu)
    case 'c':
      if (systemReady || simulationMode) {
        collisionSet = !collisionSet;
        updateMessage("Collision: " + (collisionSet ? "ON" : "OFF"));
      } else updateMessage("Cannot toggle collision");
      break;
    case 'n':
      connect();
      break;
    case 'o':
      trace = !trace;
      updateMessage("Trace: " + (trace ? "ON" : "OFF"));
      break;
    case 'l':
      toggleLeapMotion();
      break;
    case 'j':
      toggleJoystick();
      break;
    }

    // Câmera (igual ao seu)
    if (key == CODED && mouseX >= 300) {
      if (keyCode == UP)    cameraRotationXIncrement = -0.02;
      if (keyCode == DOWN)  cameraRotationXIncrement =  0.02;
      if (keyCode == LEFT)  cameraRotationYIncrement = -0.02;
      if (keyCode == RIGHT) cameraRotationYIncrement =  0.02;
    }

    if (width > 1100 && height > 650 && mouseX >= 300) {
      if (key == '+') {
        zoomLevel *= KEYBOARD_ZOOM_FACTOR;
        zoomLevel = constrain(zoomLevel, MIN_ZOOM, MAX_ZOOM);
        zoomTarget.set(width/2+120, height/2+80, 0);
      }
      if (key == '-') {
        zoomLevel /= KEYBOARD_ZOOM_FACTOR;
        zoomLevel = constrain(zoomLevel, MIN_ZOOM, MAX_ZOOM);
        zoomTarget.set(width/2+120, height/2+80, 0);
      }
    }
  }
}

void keyReleased() {
  char k = Character.toLowerCase(key);

  // ==============================
  // Keyboard → ARM
  // ==============================

  if (isManipulatorSelected) {
    switch (k) {
    case 'a':
    case 'd':
      resetKeyboardState(BASE_IDX);
      break;
    case 'w':
    case 's':
      resetKeyboardState(UPPERARM_IDX);
      break;
    case 'r':
    case 'f':
      resetKeyboardState(FOREARM_IDX);
      break;
    case 't':
    case 'g':
      resetKeyboardState(WRIST_VERT_IDX);
      break;
    case 'q':
    case 'e':
      resetKeyboardState(WRIST_ROT_IDX);
      break;
    case 'z':
    case 'x':
      resetKeyboardState(GRIPPER_IDX);
      break;
    }
    if (key == CODED && mouseX >= 300) {
      if (keyCode == UP || keyCode == DOWN) cameraRotationXIncrement = 0;
      if (keyCode == LEFT || keyCode == RIGHT) cameraRotationYIncrement = 0;
      if (keyCode == '+' || keyCode == '-') cameraRotationXIncrement = 0;
    }
  }

  // ==============================
  // Keyboard → DRONE
  // ==============================

  if (isDroneSelected) {
    switch (k) {
    case 'a':
    case 'd':
      resetKeyboardState(YAW_IDX);
      break;
    case 'w':
    case 's':
      resetKeyboardState(PITCH_IDX);
      break;
    case 'r':
    case 'f':
      resetKeyboardState(ROLL_IDX);
      break;
    case 't':
    case 'g':
      resetKeyboardState(ALT_IDX);
      break;
    case 'q':
    case 'e':
      resetKeyboardState(STRAFE_IDX);
      break;
    case 'z':
    case 'x':
      resetKeyboardState(FWD_IDX);
      break;
    }
    if (key == CODED && mouseX >= 300) {
      if (keyCode == UP || keyCode == DOWN)    cameraRotationXIncrement = 0;
      if (keyCode == LEFT || keyCode == RIGHT) cameraRotationYIncrement = 0;
    }
  }
}

void updateWithKeyboard() {

  // ==============================
  // Keyboard → ARM
  // ==============================

  if (isManipulatorSelected) {
    if (frameCount % 1 != 0 || isPlaying()) {
      return;
    }
    final float SERVO_KEYBOARD_SPEED = 1.0f;
    for (int i = BASE_IDX; i <= GRIPPER_IDX; i++) {
      if (keyPressedStates[i]) {
        setAngle(mapIndexToChannel(i), angles[i] + (int)(keyboardIncrement[i] * SERVO_KEYBOARD_SPEED));
      }
    }
  }

  // ==============================
  // Keyboard → DRONE
  // ==============================

  if (isDroneSelected) {
    if (isPlaying()) return;

    if (keyPressedStates[YAW_IDX]) {
      float d = (keyboardIncrement[YAW_IDX] > 0 ? +YAW_SPEED_DEG : -YAW_SPEED_DEG);
      float deg = degrees(droneYaw) + d;
      deg = constrain(deg, YAW_MIN, YAW_MAX);
      droneYaw = radians(deg);
    }
    if (keyPressedStates[PITCH_IDX]) {
      float d = (keyboardIncrement[PITCH_IDX] > 0 ? +PITCH_SPEED_DEG : -PITCH_SPEED_DEG);
      float deg = degrees(dronePitch) + d;
      deg = constrain(deg, PITCH_MIN, PITCH_MAX);
      dronePitch = radians(deg);
    }
    if (keyPressedStates[ROLL_IDX]) {
      float d = (keyboardIncrement[ROLL_IDX] > 0 ? +ROLL_SPEED_DEG : -ROLL_SPEED_DEG);
      float deg = degrees(droneRoll) + d;
      deg = constrain(deg, ROLL_MIN, ROLL_MAX);
      droneRoll = radians(deg);
    }
    if (keyPressedStates[ALT_IDX]) {
      float d = (keyboardIncrement[ALT_IDX] > 0 ? +ALT_SPEED_UNITS : -ALT_SPEED_UNITS);
      droneY = constrain(droneY + d, ALT_MIN, ALT_MAX);
    }
    if (keyPressedStates[STRAFE_IDX]) {
      float d = (keyboardIncrement[STRAFE_IDX] > 0 ? +XY_SPEED_UNITS : -XY_SPEED_UNITS);
      droneX = constrain(droneX + d, X_MIN, X_MAX);
    }
    if (keyPressedStates[FWD_IDX]) {
      float d = (keyboardIncrement[FWD_IDX] > 0 ? +XY_SPEED_UNITS : -XY_SPEED_UNITS);
      droneZ = constrain(droneZ + d, Z_MIN, Z_MAX);
    }
  }
}

void setKeyboardIncrement(int jointIndex, int incrementValue) {

  // ==============================
  // Keyboard → ARM
  // ==============================

  if (isManipulatorSelected) {
    if (jointIndex >= 0 && jointIndex < keyPressedStates.length) {
      keyPressedStates[jointIndex] = true;
      keyboardIncrement[jointIndex] = incrementValue;
    } else {
    }
  }

  // ==============================
  // Keyboard → DRONE
  // ==============================

  if (isDroneSelected) {
    if (jointIndex >= 0 && jointIndex < keyPressedStates.length) {
      keyPressedStates[jointIndex] = true;
      keyboardIncrement[jointIndex] = incrementValue;
    }
  }
}

void resetKeyboardState(int jointIndex) {

  // ==============================
  // Keyboard → ARM
  // ==============================

  if (isManipulatorSelected) {
    if (jointIndex >= 0 && jointIndex < keyPressedStates.length) {
      keyPressedStates[jointIndex] = false;
      keyboardIncrement[jointIndex] = 0;
    }
  }

  // ==============================
  // Keyboard → DRONE
  // ==============================

  if (isDroneSelected) {

    if (jointIndex >= 0 && jointIndex < keyPressedStates.length) {
      keyPressedStates[jointIndex] = false;
      keyboardIncrement[jointIndex] = 0;
    }
  }
}

// ==============================
// Joystick
// ==============================

void toggleJoystick() {
  if (!systemReady && !simulationMode) {
    updateMessage("Joystick requires Hardware connection");
    return;
  }
  if (!enableJoystick) {
    enableJoystick = true;
    checkJoystick();
  } else {
    enableJoystick = false;
    updateMessage("Joystick control disabled");
  }
}

void checkJoystick() {
  String filePath = sketchPath("data/SynROV_joystick");
  File file = new File(filePath);
  if (!file.exists()) {

    String fileContent = "SynROV_joystick\n" +
      "BASE\tBASE\t3\tSLIDER\tEixo X\t0\t1.0\t0.0\n" +
      "UPPERARM\tUPPERARM\t3\tSLIDER\tEixo Y\t0\t1.0\t0.0\n" +
      "FOREARM\tFOREARM\t3\tSLIDER\tEixo Z\t0\t1.0\t0.0\n" +
      "WRIST_VERT\tWRIST_VERT\t3\tSLIDER\tRotação Y\t0\t1.0\t0.0\n" +
      "WRIST_ROT\tWRIST_ROT\t3\tSLIDER\tRotação X\t0\t1.0\t0.0\n" +
      "GRIPPER_OPEN\tGRIPPER_OPEN\t1\tBUTTON\tBotão 0\t0\t0.0\t0.0\n" +
      "GRIPPER_CLOSE\tGRIPPER_CLOSE\t1\tBUTTON\tBotão 1\t0\t0.0\t0.0";

    File dataFolder = new File(sketchPath("data"));
    if (!dataFolder.exists()) {
      dataFolder.mkdir();
    }
    try {
      PrintWriter writer = createWriter(filePath);
      writer.print(fileContent);
      writer.flush();
      writer.close();
      println("Successfully created SynROV_joystick file in the 'data' folder.");
    }
    catch (Exception e) {
      println("Failed to create file: " + e.getMessage());
    }
  } else {
    println("SynROV_joystick file already exists.");
  }
  try {
    control = ControlIO.getInstance(this);
    joystick = control.filter(GCP.GAMEPAD).getMatchedDevice("SynROV_joystick");
    if (joystick == null) {
      println("No suitable device configured");
    }
  }
  catch (Exception e) {
  }
}

float applyDeadband(float value, float deadband) {
  return abs(value) > deadband ? value : 0.0f;
}

void updateWithJoystick() {
  final float JOYSTICK_DEADBAND = 0.1f;
  final float SERVO_JOYSTICK_SPEED = 3.0f;
  final float JOYSTICK_SMOOTH_FACTOR = 0.4f;
  final int GRIPPER_BUTTON_SPEED = 2;

  if (isManipulatorSelected) {
    float baseValue = applyDeadband(joystick.getSlider("BASE").getValue(), JOYSTICK_DEADBAND);
    float upperArmValue = applyDeadband(joystick.getSlider("UPPERARM").getValue(), JOYSTICK_DEADBAND);
    float foreArmValue = applyDeadband(joystick.getSlider("FOREARM").getValue(), JOYSTICK_DEADBAND);
    float wristVertValue = applyDeadband(joystick.getSlider("WRIST_VERT").getValue(), JOYSTICK_DEADBAND);
    float wristRotValue = applyDeadband(joystick.getSlider("WRIST_ROT").getValue(), JOYSTICK_DEADBAND);
    int gripperDelta = 0;
    if (joystick.getButton("GRIPPER_CLOSE").pressed()) {
      gripperDelta = GRIPPER_BUTTON_SPEED;
    }
    if (joystick.getButton("GRIPPER_OPEN").pressed()) {
      gripperDelta = -GRIPPER_BUTTON_SPEED;
    }
    float baseDelta = baseValue * SERVO_JOYSTICK_SPEED;
    float upperArmDelta = upperArmValue * SERVO_JOYSTICK_SPEED;
    float foreArmDelta = foreArmValue * SERVO_JOYSTICK_SPEED;
    float wristVertDelta = wristVertValue * SERVO_JOYSTICK_SPEED;
    float wristRotDelta = wristRotValue * SERVO_JOYSTICK_SPEED;
    int newBaseAngle = (int)(angles[BASE_IDX] + baseDelta);
    int newUpperArmAngle = (int)(angles[UPPERARM_IDX] + upperArmDelta);
    int newForeArmAngle = (int)(angles[FOREARM_IDX] + foreArmDelta);
    int newWristVertAngle = (int)(angles[WRIST_VERT_IDX] + wristVertDelta);
    int newWristRotAngle = (int)(angles[WRIST_ROT_IDX] + wristRotDelta);
    int newGripperAngle = (int)(angles[GRIPPER_IDX] + gripperDelta);
    int smoothedBase = smooth(angles[BASE_IDX], newBaseAngle, JOYSTICK_SMOOTH_FACTOR);
    int smoothedUpperArm = smooth(angles[UPPERARM_IDX], newUpperArmAngle, JOYSTICK_SMOOTH_FACTOR);
    int smoothedForeArm = smooth(angles[FOREARM_IDX], newForeArmAngle, JOYSTICK_SMOOTH_FACTOR);
    int smoothedWristVert = smooth(angles[WRIST_VERT_IDX], newWristVertAngle, JOYSTICK_SMOOTH_FACTOR);
    int smoothedWristRot = smooth(angles[WRIST_ROT_IDX], newWristRotAngle, JOYSTICK_SMOOTH_FACTOR);
    int smoothedGripper = newGripperAngle;
    setAngle(mapIndexToChannel(BASE_IDX), smoothedBase);
    setAngle(mapIndexToChannel(UPPERARM_IDX), smoothedUpperArm);
    setAngle(mapIndexToChannel(FOREARM_IDX), smoothedForeArm);
    setAngle(mapIndexToChannel(WRIST_VERT_IDX), smoothedWristVert);
    setAngle(mapIndexToChannel(WRIST_ROT_IDX), smoothedWristRot);
    setAngle(mapIndexToChannel(GRIPPER_IDX), smoothedGripper);
  }
}

// ==============================
// Leap Motion
// ==============================

void toggleLeapMotion() {
  if (!systemReady && !simulationMode) {
    updateMessage("Leap Motion requires Hardware connection");
    return;
  }
  if (!enableLeap) {
    checkLeapMotion();
    if (leapAvailable) {
      enableLeap = true;
      updateMessage("Leap Motion enabled");
    }
  } else {
    enableLeap = false;
    updateMessage("Leap Motion disabled");
  }
}

void checkLeapMotion() {
  updateMessage("Checking Leap Motion...");
  if (leap != null) {
    try {
      leap = null;
    }
    catch (Exception e) {
    }
  }
  try {
    leap = new LeapMotion(this);
    if (leap.getDevices() != null && leap.getDevices().size() > 0) {
      leapAvailable = true;
      updateMessage("Leap Motion detected and activated");
    } else {
      leapAvailable = false;
      leap = null;
      updateMessage("No Leap Motion device found");
    }
  }
  catch (Exception e) {
    leapAvailable = false;
    leap = null;
    e.printStackTrace();
    updateMessage("Error initializing Leap Motion");
  }
}

void updateWithLeap() {
  if (!leapAvailable || isPlaying()) {
    return;
  }
  if (millis() - lastLeapUpdateTime < MIN_LEAP_UPDATE_INTERVAL) {
    return;
  }
  lastLeapUpdateTime = millis();
  for (Hand hand : leap.getHands()) {
    if (hand.isRight()) {
      updateRightHand(hand);
    } else if (hand.isLeft()) {
      updateLeftHand(hand);
    }
  }
}

private void updateRightHand(Hand hand) {

  if (isManipulatorSelected) {
    final float WRIST_SMOOTH_FACTOR = 0.30f;
    final float GRIPPER_SMOOTH_FACTOR = 0.15f;
    int wristV = (int) map(hand.getPitch(), -100, 100, 0, 180);
    int newWristV = smooth(angles[WRIST_VERT_IDX], wristV, WRIST_SMOOTH_FACTOR);
    if (Math.abs(newWristV - angles[WRIST_VERT_IDX]) >= ANGLE_DEADBAND) {
      setAngle(mapIndexToChannel(WRIST_VERT_IDX), newWristV);
    }
    int wristR = (int) map(hand.getRoll(), -100, 100, 0, 180);
    int newWristR = smooth(angles[WRIST_ROT_IDX], wristR, WRIST_SMOOTH_FACTOR);
    if (Math.abs(newWristR - angles[WRIST_ROT_IDX]) >= ANGLE_DEADBAND) {
      setAngle(mapIndexToChannel(WRIST_ROT_IDX), newWristR);
    }
    int gripper = (int) map(hand.getPinchStrength(), 0, 1, 100, 180);
    int newGripper = smooth(angles[GRIPPER_IDX], gripper, GRIPPER_SMOOTH_FACTOR);
    if (Math.abs(newGripper - angles[GRIPPER_IDX]) >= ANGLE_DEADBAND) {
      setAngle(mapIndexToChannel(GRIPPER_IDX), newGripper);
    }
  }
}

private void updateLeftHand(Hand hand) {

  if (isManipulatorSelected) {
    final float BASE_SMOOTH_FACTOR = 0.05f;
    final float ARM_SMOOTH_FACTOR = 0.03f;
    final float FOREARM_SMOOTH_FACTOR = 0.03f;
    int base = (int) map(hand.getYaw(), 80, -30, 0, 180);
    int newBase = smooth(angles[BASE_IDX], base, BASE_SMOOTH_FACTOR);
    if (Math.abs(newBase - angles[BASE_IDX]) >= ANGLE_DEADBAND) {
      setAngle(mapIndexToChannel(BASE_IDX), newBase);
    }
    int arm = (int) map(hand.getPitch(), -100, 100, 0, 180);
    int newArm = smooth(angles[UPPERARM_IDX], arm, ARM_SMOOTH_FACTOR);
    if (Math.abs(newArm - angles[UPPERARM_IDX]) >= ANGLE_DEADBAND) {
      setAngle(mapIndexToChannel(UPPERARM_IDX), newArm);
    }
    int forearm = (int) map(hand.getRoll(), 100, -100, 0, 180);
    int newForearm = smooth(angles[FOREARM_IDX], forearm, FOREARM_SMOOTH_FACTOR);
    if (Math.abs(newForearm - angles[FOREARM_IDX]) >= ANGLE_DEADBAND) {
      setAngle(mapIndexToChannel(FOREARM_IDX), newForearm);
    }
  }
}

int smooth(int current, int target, float factor) {
  factor = constrain(factor, 0.0f, 1.0f);
  return (int)(current * (1.0f - factor) + target * factor);
}
