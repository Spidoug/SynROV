// drawSidebar: Handles the rendering of the control sidebar on the left side of the screen.
void drawSidebar() {
  fill(255);
  stroke(200);
  strokeWeight(1);
  textSize(12);
  rect(0, 0, 300, height);

  drawFrame(12, 15, 276, 230, 255, color(220), 10);

  fill(30);
  textAlign(CENTER, CENTER);

  if (isManipulatorSelected) {
    text(controlInstructionsManipulator, 150, 130);
  }

  if (isVehicleSelected) {
    text(controlInstructionsVehicle, 150, 130);
  }

  if (isDroneSelected) {
    text(controlInstructionsDrone, 150, 130);
  }

  drawFrame(12, 270, 276, 215, 255, color(220), 10);

  for (int i = 0; i < 5; i++) {
    boolean hasAction = actions[i] != null && actions[i].size() > 0;
    boolean isPlaying = playingBack[i];
    boolean isRecording = (recording && selectedAction == i);
    String label = isRecording ? "Stop action " + (i + 1) : "Action " + (i + 1);

    drawButtonAction(i, 20, 280 + i * 30, 260, 20, label, hasAction, isPlaying, isRecording, false);
  }

  if (width > 1100 && height > 650) {
    maximized = true;
  } else {
    maximized = false;
  }

  if (maximized) {
    fill(30);
    textAlign(CENTER, CENTER);
    textSize(16);
    drawFrame(12, 510, 276, 215, 255, color(220), 10);
    fill(0);
    drawButtonType(20, 520, 260, 35, "Manipulator", isManipulatorSelected);
    drawButtonType(20, 565, 260, 35, "Vehicle", isVehicleSelected);
    drawButtonType(20, 610, 260, 35, "Drone", isDroneSelected);
  }

  fill(80);
  textAlign(CENTER, CENTER);
  textSize(16);
  text(timeDisplay, 150, 450);

  drawFrame(12, height - 140, 276, 125, 255, color(220), 10);

  fill(0);
  drawButtonConnection(20, height - 130, 260, 35, "Connect");

  textAlign(CENTER, CENTER);
  textSize(12);
  for (int i = 0; i < logMessages.length; i++) {
    text(logMessages[i], 150, height - 80 + i * 16);
  }
}

// drawFrame: Renders a rounded rectangle frame with a border and background.
void drawFrame(float x, float y, float w, float h, int borderColor, int bgColor, float radius) {
  noStroke();
  fill(bgColor);
  rect(x, y, w, h, radius);
  stroke(borderColor);
  strokeWeight(1.5);
  rect(x, y, w, h, radius);
  noFill();
}

// drawButton: Renders a button with dynamic coloring based on its state.
void drawButtonType(int x, int y, int w, int h, String label, boolean isSelected) {

  if (isSelected) {
    fill(150, 200, 255); // Cor azul claro para indicar seleção
  } else {
    fill(220);
  }

  stroke(150);
  strokeWeight(2);
  rect(x, y, w, h, 8);
  noFill();

  fill(0);
  textAlign(CENTER, CENTER);
  text(label, x + w / 2, y + h / 2);
}

// drawButton: Renders a button with dynamic coloring based on its state.
void drawButtonAction(int index, int x, int y, int w, int h, String label, boolean hasAction, boolean isPlaying, boolean isRecording, boolean isSelected) {

  if (isRecording) {
    fill(139, 149, 255);
  } else if (isPlaying) {
    fill(255, 100, 100);
  } else if (index >= 0 && actionSlotLocked[index]) {
    fill(0, 200, 100);
  } else {
    fill(230);
  }

  stroke(150);
  strokeWeight(2);
  rect(x, y, w, h, 8);
  noFill();

  fill(0);
  textAlign(CENTER, CENTER);
  text(label, x + w / 2, y + h / 2);
}

// drawButton: Renders a button with dynamic coloring based on its state.
void drawButtonConnection(int x, int y, int w, int h, String label) {
  if (systemReady) {
    fill(0, 200, 100);
    label = "Connected";
  } else if (simulationMode) {
    fill(255, 121, 121);
    label = "Simulation";
  } else if (initializationStep > 0 && initializationStep < 6 && !simulationMode) {
    fill(255, 150, 0);
    label = "Connecting...";
  } else {
    fill(220);
  }

  stroke(150);
  strokeWeight(2);
  rect(x, y, w, h, 8);
  noFill();

  fill(0);
  textAlign(CENTER, CENTER);
  text(label, x + w / 2, y + h / 2);
}

// drawHUD: Renders the Heads-Up Display with various system statuses and information.
void drawHUD() {
  pushMatrix();
  hint(DISABLE_DEPTH_TEST);
  camera();
  textAlign(LEFT, TOP);
  textSize(12);
  fill(20);

  float camAngleDeg = (degrees(cameraRotationY) % 360 + 360) % 360;
  text("CAMERA Y ROTATION: " + nf(camAngleDeg, 1, 1) + "°", width / 2 + 80, 27);

  int statusColor = color(255);
  String statusLabel = "Disconnected";

  fill(30);
  textSize(12);
  textAlign(LEFT, TOP);

  text("SERVO ANGLES", 340, 27);
  text("-Base: " + angles[0], 320, 47);
  text("-Arm: " + angles[1], 320, 67);
  text("-Forearm: " + angles[2], 320, 87);
  text("-Wrist Vertical: " + angles[3], 320, 107);
  text("-Wrist Rotation: " + angles[4], 320, 127);
  text("-Gripper: " + angles[5], 320, 147);

  fill(30);
  textSize(12);
  textAlign(RIGHT, CENTER);
  text(leapAvailable ? "Hands detected: " + (leap != null ? leap.countHands() : 0) : "Leap Motion not available", width - 20, 140);

  text("Leap Motion:    " + (enableLeap ? "Enable" : "Disable"), width - 20, 170);
  text("Joystick:     " + (enableJoystick ? "Enable" : "Disable"), width - 20, 190);
  text("Collision detection:    " + (collisionSet ? "Enable" : "Disable"), width - 20, 210);

  if (systemReady) {
    boolean recentlyMoved = (millis() - lastServoCommandTime) < movementTimeout;
    statusColor = recentlyMoved ? color(255, 100, 100) : color(0, 200, 100);
    statusLabel = recentlyMoved ? "Moving" : "Idle";

    fill(30);
    textSize(12);
    textAlign(LEFT, TOP);

    text("Last move: " + int(((millis() - lastServoCommandTime)) / 1000) + " s ago", width - 140, 40);

    if (int(((millis() - lastServoCommandTime)) / 1000) > 86400) {
      connect();
    }
  }

  textAlign(RIGHT, CENTER);
  noStroke();
  fill(statusColor);
  ellipse(width - 30, 27, 16, 16);
  fill(0);
  text("Robot Status: " + statusLabel, width - 50, 27);
  noFill();
  hint(ENABLE_DEPTH_TEST);
  popMatrix();
}
