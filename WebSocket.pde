void webSocketServerEvent(String msg) {
  if (msg != null) {
    pendingWsCommands.add(msg.trim());
  } else {
  }
}

void processPendingWsCommands() {
  while (!pendingWsCommands.isEmpty()) {
    String cmd = pendingWsCommands.remove(0);

    try {
      JSONObject jsonCommand = JSONObject.parse(cmd);

      if (jsonCommand.hasKey("handshake")) {
        handleHandshakeCommand(jsonCommand);
      } else if (jsonCommand.hasKey("camera")) {
        handleCameraCommand(jsonCommand);
      } else if (jsonCommand.hasKey("servo") && jsonCommand.hasKey("angle")) {
        handleServoCommand(jsonCommand);
      } else if (jsonCommand.hasKey("serial")) {
        handleSerialCommand(jsonCommand);
      } else if (jsonCommand.hasKey("toggle")) {
        handleToggleCommand(jsonCommand);
      } else if (jsonCommand.hasKey("action")) {
        handleActionCommand(jsonCommand);
      } else if (jsonCommand.hasKey("connect") && jsonCommand.getBoolean("connect")) {
        handleHardwareConnectCommand();
      } else {
        updateMessage("Unknown WS command");
      }
    }
    catch (Exception e) {
      e.printStackTrace();
      updateMessage("WS command error: " + e.getMessage());
    }
  }
}

void handleHandshakeCommand(JSONObject jsonCommand) {
  for (int i = 0; i < 6; i++) {
    int servoChannel = (i < 3) ? i : i + 10;
    JSONObject response = new JSONObject();
    response.setInt("servo", servoChannel);
    response.setInt("angle", angles[i]);
    response.setInt("timestamp", millis());
    if (wsServer != null) {
      wsServer.sendMessage(response.toString());
    }
  }

  if (latestSensors != null && latestSensors.size() > 0) {
    JSONObject sensorMessage = new JSONObject();
    sensorMessage.setJSONObject("sensors", latestSensors);
    if (wsServer != null) {
      wsServer.sendMessage(sensorMessage.toString());
    }
  }
}

void handleCameraCommand(JSONObject jsonCommand) {
  String direction = jsonCommand.getString("camera");
  final float CAMERA_STEP = 0.05f;
  final float ZOOM_STEP_AMOUNT = 0.05f; // Smaller step for smooth zoom

  switch (direction) {
  case "left":
    cameraRotationY -= CAMERA_STEP;
    break;
  case "right":
    cameraRotationY += CAMERA_STEP;
    break;
  case "up":
    cameraRotationX -= CAMERA_STEP;
    break;
  case "down":
    cameraRotationX += CAMERA_STEP;
    break;
  case "zoom_in": // Added: Zoom in
    zoomLevel = constrain(zoomLevel * (1.0f - ZOOM_STEP_AMOUNT), 0.5f, 5.0f);
    break;
  case "zoom_out": // Added: Zoom out
    zoomLevel = constrain(zoomLevel * (1.0f + ZOOM_STEP_AMOUNT), 0.5f, 5.0f);
    break;
  default:
    break;
  }
}

void handleServoCommand(JSONObject jsonCommand) {
  int servoId = jsonCommand.getInt("servo");
  int targetAngle = jsonCommand.getInt("angle");
  setAngle(servoId, targetAngle);
}

void handleSerialCommand(JSONObject jsonCommand) {
  if ((systemReady && !simulationMode) && myPort != null) {
    String serialData = jsonCommand.getString("serial");
    myPort.write(serialData + "\n");
  } else {
    updateMessage("Hardware not ready or port closed");
  }
}

void handleToggleCommand(JSONObject jsonCommand) {
  String toggleFeature = jsonCommand.getString("toggle").toLowerCase();

  switch (toggleFeature) {
  case "leap":
    if (!enableLeap) {
      checkLeapMotion();
      if (leapAvailable) {
        enableLeap = true;
        updateMessage("Leap Motion enabled via WS");
      } else {
        updateMessage("Leap Motion not available");
      }
    } else {
      enableLeap = false;
      updateMessage("Leap Motion disabled via WS");
    }
    break;
  case "joystick":
    if (!enableJoystick) {
      checkJoystick();
    } else {
      enableJoystick = false;
      updateMessage("Joystick disabled via WS");
    }
    break;
  case "collision":
    if (systemReady || simulationMode) {
      collisionSet = !collisionSet;
      updateMessage("Collision detection " + (collisionSet ? "enabled" : "disabled") + " via WS");
    } else {
      updateMessage("Collision detection cannot be toggled");
    }
    break;
  case "trace":
    trace = !trace;
    updateMessage("Trace " + (trace ? "enabled" : "disabled") + " via WS");
    break;
  default:
    updateMessage("Unknown toggle command: " + toggleFeature);
    break;
  }
  sendSystemStatus();
}

void handleActionCommand(JSONObject jsonCommand) {
  int actionIndex = jsonCommand.getInt("action") - 1;
  final int MAX_ACTIONS = 5;

  if (actionIndex >= 0 && actionIndex < MAX_ACTIONS) {
    handleActionButton(actionIndex);
  } else {
    updateMessage("Invalid action index: " + (actionIndex + 1));
  }
}

void handleHardwareConnectCommand() {
  connect();
}
