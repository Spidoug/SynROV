void closeSerialPort(Serial port, String portName) {
  if (port != null) {
    try {
      port.stop();
      port.clear();
    }
    catch (Exception e) {
      e.printStackTrace();
    }
  }
}

void connect() {
  sentReadyRequest = false;
  readyRequestTime = 0;

  if (initializationStep != 0) {
    systemReady = false;
    simulationMode = false;
    initializationStep = 0;
    updateMessage("Disconnected. System reset.");
    closeSerialPort(testPort, "testPort");
    closeSerialPort(myPort, "myPort");
    myPort = null;
    testPort = null;
  } else {
    updateMessage("Starting connection...");

    closeSerialPort(testPort, "testPort");
    closeSerialPort(myPort, "myPort");
    testPort = null;
    myPort = null;

    portIndex = 0;
    hardwareStartTime = 0;
    waiting = false;
    waitDuration = 0;
    waitStartTime = 0;
    systemReady = false;
    simulationMode = false;
    initializationStep = 1;
    clearLog();

    startWait(1000);
  }
}

void checkLastPort() {
  if (!checkWait()) return;

  updateMessage("Checking last used port...");
  String lastPort = loadLastPort();
  ports = Serial.list();

  if (lastPort != null) {
    for (int i = 0; i < ports.length; i++) {
      if (ports[i].equals(lastPort)) {
        updateMessage("Attempting to reconnect to last port: " + lastPort);
        try {
          testPort = new Serial(this, lastPort, 115200);
          testPort.clear();
          while (testPort.available() > 0) {
            testPort.read();
          }
          testPort.bufferUntil('\n');
          hardwareStartTime = millis();
          portIndex = i;
          initializationStep = 4;
          return;
        }
        catch (Exception e) {
          e.printStackTrace();
          updateMessage("Auto-reconnect failed. Testing others");
        }
      }
    }
  } else {
    updateMessage("No last port saved or found");
  }

  startWait(1000);
  initializationStep = 2;
}

void listSerialPorts() {
  if (!checkWait()) return;

  updateMessage("Searching for serial ports...");
  ports = Serial.list();
  updateMessage(" " + ports.length + " ports found.");
  startWait(500);
  initializationStep = 3;
}

void testSerialPorts() {
  if (!checkWait()) return;

  if (portIndex < ports.length) {
    updateMessage("Testing port: " + ports[portIndex]);
    try {
      testPort = new Serial(this, ports[portIndex], 115200);
      testPort.clear();
      testPort.bufferUntil('\n');
      hardwareStartTime = millis();
      initializationStep = 4;
      startWait(500);
    }
    catch (Exception e) {
      e.printStackTrace();
      updateMessage("Failed to open port: " + ports[portIndex] + ". Trying next...");
      portIndex++;
      startWait(400);
    }
  } else {
    initializationStep = 6;
    updateMessage("No compatible SynROV port found");
  }
}

void connectHardware() {
  if (testPort != null && testPort.available() > 0) {
    String msg = testPort.readStringUntil('\n');
    if (msg != null) {
      msg = msg.trim();

      if (msg.equals("SynROV") && !sentReadyRequest) {
        testPort.write("READY?\n");
        sentReadyRequest = true;
        readyRequestTime = millis();
      } else if (msg.equals("READY!") && sentReadyRequest) {
        myPort = testPort;
        systemReady = true;
        saveLastPort(ports[portIndex]);
        updateMessage("Hardware connected: " + ports[portIndex]);
        goingHome = true;
        initializationStep = 5;
        sentReadyRequest = false;
        return;
      }
    }
  }

  if (sentReadyRequest && millis() - readyRequestTime > 2000) {
    closeSerialPort(testPort, "testPort");
    testPort = null;
    sentReadyRequest = false;
    portIndex++;
    initializationStep = 3;
    startWait(500);
  } else if (!sentReadyRequest && millis() - hardwareStartTime > 3000) {
    closeSerialPort(testPort, "testPort");
    testPort = null;
    portIndex++;
    initializationStep = 3;
    startWait(500);
  }
}

void serialEvent(Serial port) {
  if (port == null || port != myPort) return;

  String line = port.readStringUntil('\n');
  if (line == null) return;
  line = line.trim();

  if (line.isEmpty()) return;

  latestSerialLine = line;

  if (wsServer != null) {
    JSONObject j = new JSONObject();
    j.setString("serial", line);
    wsServer.sendMessage(j.toString());
  }

  if (line.startsWith("#SENS|")) {
    parseSensorPacket(line);
  }
}

void parseSensorPacket(String raw) {
  JSONObject sens = new JSONObject();
  String payload = raw.substring(6);
  String[] blocks = split(payload, '|');

  for (String b : blocks) {
    String[] kv = split(b, ':');
    if (kv.length == 2) {
      if (kv[0].equals("MPU") && kv[1].contains(",")) {
        String[] mpuData = split(kv[1], ',');
        if (mpuData.length >= 3) {
          try {
            sens.setInt("ax", int(mpuData[0]));
            sens.setInt("ay", int(mpuData[1]));
            sens.setInt("az", int(mpuData[2]));
            if (mpuData.length >= 6) {
              sens.setInt("gx", int(mpuData[3]));
              sens.setInt("gy", int(mpuData[4]));
              sens.setInt("gz", int(mpuData[5]));
            }
          }
          catch (NumberFormatException e) {
          }
        }
      } else if (kv[0].equals("AN") && kv[1].contains(",")) {
        String[] analogData = split(kv[1], ',');
        for (int i = 0; i < analogData.length; i++) {
          try {
            sens.setInt("AN" + i, int(analogData[i]));
          }
          catch (NumberFormatException e) {
          }
        }
      } else if ((kv[0].equals("EX1") || kv[0].equals("EX2")) && kv[1].contains(",")) {
        String[] values = split(kv[1], ',');
        for (int i = 0; i < values.length; i++) {
          try {
            sens.setInt(kv[0].toLowerCase() + "_" + i, int(values[i]));
          }
          catch (NumberFormatException e) {
          }
        }
      } else {
        sens.setString(kv[0], kv[1]);
      }
    }
  }

  latestSensors = sens;
}

String loadLastPort() {
  File dataDir = new File(sketchPath("data"));
  if (!dataDir.exists()) {
    dataDir.mkdirs();
  }

  String[] lines = loadStrings(LAST_PORT);
  if (lines != null && lines.length > 0) {
    return lines[0].trim();
  }
  return null;
}

void saveLastPort(String port) {
  File dataDir = new File(sketchPath("data"));
  if (!dataDir.exists()) {
    dataDir.mkdirs();
  }
  String[] data = {
    port
  };
  saveStrings(LAST_PORT, data);
}

void simulationMode() {
  simulationMode = true;
  systemReady = false;
  updateMessage("Simulation Mode Enabled");
  initializationStep = 5;
  goingHome = true;
}
