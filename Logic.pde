void moveServo(int channel, int angle) {
  if (systemReady && myPort != null) {
    myPort.write(String.format("S%dP%d\n", channel, angle));
  }
  if (wsServer != null) {
    JSONObject jsonMessage = new JSONObject();
    jsonMessage.setInt("servo", channel);
    jsonMessage.setInt("angle", angle);
    jsonMessage.setInt("timestamp", millis());
    wsServer.sendMessage(jsonMessage.toString());
  }
}

void setAngle(int channel, int value) {
  int servoIndex = mapServoIndex(channel);
  if (servoIndex == -1) {
    updateMessage("Invalid servo channel " + channel);
    return;
  }
  int constrainedAngle = constrain(value, servoLimits[servoIndex][0], servoLimits[servoIndex][1]);
  if (angles[servoIndex] == constrainedAngle) {
    return;
  }
  int oldAngle = angles[servoIndex];
  angles[servoIndex] = constrainedAngle;
  if (collisionSet) {
    collectColliders();
    if (checkCollisions()) {
      angles[servoIndex] = oldAngle;
      return;
    }
  }
  moveServo(channel, constrainedAngle);
  lastServoCommandTime = millis();
}

int mapServoIndex(int channel) {
  switch (channel) {
  case 0:
    return BASE_IDX;
  case 1:
    return UPPERARM_IDX;
  case 2:
    return FOREARM_IDX;
  case 3:
    return WRIST_VERT_IDX;
  case 4:
    return WRIST_ROT_IDX;
  case 5:
    return GRIPPER_IDX;
  default:
    return -1;
  }
}

int mapIndexToChannel(int index) {
  switch (index) {
  case BASE_IDX:
    return 0;
  case UPPERARM_IDX:
    return 1;
  case FOREARM_IDX:
    return 2;
  case WRIST_VERT_IDX:
    return 3;
  case WRIST_ROT_IDX:
    return 4;
  case GRIPPER_IDX:
    return 5;
  default:
    return -1;
  }
}

boolean isPlaying() {
  for (boolean pb : playingBack) {
    if (pb) {
      return true;
    }
  }
  return false;
}

void loadServoLimits() {
  File dataDir = new File(sketchPath("data"));
  if (!dataDir.exists()) {
    dataDir.mkdirs();
  }
  File file = new File(sketchPath(SERVO_LIMITS_FILE));
  if (!file.exists()) {
    createDefaultLimitsFile();
  }
  try {
    loadLimitsFromFile(SERVO_LIMITS_FILE);
  } catch (Exception e) {
    e.printStackTrace();
  }
}

private void createDefaultLimitsFile() {
  JSONArray jsonArray = new JSONArray();
  int[][] defaultLimits = {
    {0, 180},
    {0, 140},
    {20, 180},
    {0, 180},
    {0, 180},
    {100, 180}
  };
  for (int i = 0; i < defaultLimits.length; i++) {
    JSONObject obj = new JSONObject();
    obj.setInt("min", defaultLimits[i][0]);
    obj.setInt("max", defaultLimits[i][1]);
    jsonArray.append(obj);
  }
  saveJSONArray(jsonArray, sketchPath(SERVO_LIMITS_FILE));
}

private void loadLimitsFromFile(String fileName) {
  JSONArray jsonArray = loadJSONArray(fileName);
  for (int i = 0; i < jsonArray.size() && i < servoLimits.length; i++) {
    JSONObject obj = jsonArray.getJSONObject(i);
    servoLimits[i][0] = obj.getInt("min");
    servoLimits[i][1] = obj.getInt("max");
  }
  updateMessage("Servo limits loaded successfully");
}

// ==============================
// Homing
// ==============================

void loadHoming() {
  File dataDir = new File(sketchPath("data"));
  if (!dataDir.exists()) {
    dataDir.mkdirs();
  }
  File file = new File(sketchPath(HOMING_FILE));
  if (!file.exists()) {
    createDefaultHoming();
  }
  try {
    loadHomingFromFile();
  } catch (Exception e) {
    println("Error loading homing file: " + e.getMessage());
    createDefaultHoming();
    loadHomingFromFile();
  }
}

private void createDefaultHoming() {
  JSONArray jsonArray = new JSONArray();
  int[] defaultHomingAngles = {
    90,
    90,
    90,
    90,
    90,
    140
  };
  for (int i = 0; i < defaultHomingAngles.length; i++) {
    JSONObject obj = new JSONObject();
    obj.setInt("angle", defaultHomingAngles[i]);
    jsonArray.append(obj);
  }
  saveJSONArray(jsonArray, sketchPath(HOMING_FILE));
  println("Default homing file created successfully.");
}

private void loadHomingFromFile() {
  JSONArray jsonArray = loadJSONArray(HOMING_FILE);
  if (jsonArray == null) {
    throw new RuntimeException("Homing file not found or is empty.");
  }
  for (int i = 0; i < jsonArray.size() && i < homingPositions.length; i++) {
    JSONObject obj = jsonArray.getJSONObject(i);
    homingPositions[i] = obj.getInt("angle");
  }
  updateMessage("Homing position loaded successfully.");
}

void goHome() {
  if (!goingHome) {
    return;
  }
  
  final int ANGLE_TOLERANCE = 20;
  
  boolean allJointsAtHome = true;

  for (int i = 0; i < homingPositions.length; i++) {
    int channel = mapIndexToChannel(i);
    int smoothedAngle = smooth(angles[i], homingPositions[i], HOMING_SMOOTH_FACTOR);
    
    setAngle(channel, smoothedAngle);
    
    if (abs(smoothedAngle - homingPositions[i]) >= ANGLE_TOLERANCE) {
      allJointsAtHome = false;
    }
  }

  if (allJointsAtHome) {
    goingHome = false;
    // Set all angles to their exact final position to ensure precision.
    for (int i = 0; i < homingPositions.length; i++) {
      int channel = mapIndexToChannel(i);
      setAngle(channel, homingPositions[i]);
    }
    updateMessage("Homing done.");
  }
}
