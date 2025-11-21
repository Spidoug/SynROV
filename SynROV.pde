// =====================================================================
// SynROV
// Robotic Arm Control – Leap Motion · Joystick · Keyboard · WebSockets
// ---------------------------------------------------------------------
// Douglas Santana (SPIDOUG)
// =====================================================================

import de.voidplus.leapmotion.*;
import processing.serial.*;
import org.gamecontrolplus.*;
import net.java.games.input.*;
import websockets.*;
import java.util.List;
import javax.swing.JFrame;
import javax.swing.ImageIcon;
import java.awt.Image;
import java.io.File;
import java.util.Base64;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.util.ArrayList;

WebsocketServer wsServer;
int WS_PORT = 9000;
String WS_PATH = "/";
ArrayList<String> pendingWsCommands = new ArrayList<String>();

String latestSerialLine = "";
JSONObject latestSensors = new JSONObject();

ControlIO control;
ControlDevice joystick;

boolean enableJoystick = false;

PImage groundLogo;

boolean systemReady = false;
boolean simulationMode = false;
boolean trace = true;

ArrayList<PVector> trajectoryPoints = new ArrayList<PVector>();
final int MAX_TRAJECTORY = 100;
PVector tip = null;

boolean isManipulatorSelected = true;
boolean isVehicleSelected = false;
boolean isDroneSelected = false;

boolean enableLeap = false;
boolean leapAvailable = false;
int ANGLE_DEADBAND = 1;
int lastLeapUpdateTime = 0;
LeapMotion leap;
long MIN_LEAP_UPDATE_INTERVAL = 30;

boolean collisionSet = true;
boolean isColliding = false;
boolean maximized = false;

final int BASE_IDX = 0;
final int UPPERARM_IDX = 1;
final int FOREARM_IDX = 2;
final int WRIST_VERT_IDX = 3;
final int WRIST_ROT_IDX = 4;
final int GRIPPER_IDX = 5;
final int ZOOM_IN_IDX = 6;
final int ZOOM_OUT_IDX = 7;

int[] angles = {90, 110, 100, 130, 90, 100};
int[][] servoLimits = {
  {0, 359}, {0, 359}, {0, 359}, {0, 359}, {0, 359}, {0, 359}};

boolean goingHome = false;

final float HOMING_SMOOTH_FACTOR = 0.05f;

final int BASE_HOME_POS = 90;
final int UPPERARM_HOME_POS = 90;
final int FOREARM_HOME_POS = 90;
final int WRIST_VERT_HOME_POS = 90;
final int WRIST_ROT_HOME_POS = 90;
final int GRIPPER_HOME_POS = 180;

final String MANIPULATOR3D = "data/config3D.json";
float baseCylinderRadius, baseCylinderHeight;
float baseBlockW, baseBlockH, baseBlockD;
float upperArmW, upperArmH, upperArmD;
float forearmW, forearmH, forearmD;
float wristVerticalW, wristVerticalH, wristVerticalD;
float gripperFingerW, gripperFingerH, gripperFingerD;

float groundColliderW, groundColliderH, groundColliderD;
float baseCylinderColliderR, baseCylinderColliderYMin, baseCylinderColliderYMax;
float baseBlockColliderW, baseBlockColliderH, baseBlockColliderD;
float upperArmColliderW, upperArmColliderH, upperArmColliderD;
float forearmColliderW, forearmColliderH, forearmColliderD;
float wristVerticalColliderW, wristVerticalColliderH, wristVerticalColliderD;
float gripperFingerColliderW, gripperFingerColliderH, gripperFingerColliderD;

float baseCylinderYOffset, baseBlockYOffset, upperArmYOffset, forearmYOffset, wristVerticalYOffset, gripperYOffset;
float fingerXOffset;

final String SERVO_LIMITS_FILE = "data/limitsServo.json";
long lastServoCommandTime = 0;
int movementTimeout = 500;

final String HOMING_FILE = "data/homingPosition.json";
int[] homingPositions = new int[6];

final String ACTION_FILE = "data/actions/action_";
ArrayList<int[]>[] actions = (ArrayList<int[]>[]) new ArrayList[5];
boolean[] playingBack = new boolean[5];
int[] playbackIndex = new int[5];
int[] lastFrameTime = new int[5];
int recordingInterval = 33;
int playbackInterval = 33;
int maxRecordingDuration = 3600 * 1000;
int selectedAction = -1;
boolean recording = false;
int recordingStartTime = 0;
String timeDisplay = "No action selected";
boolean[] actionSlotLocked = new boolean[5];
long lastRecordTime = 0;

PVector zoomTarget = new PVector();

final String LAST_PORT = "data/lastPt.txt";
Serial myPort;
Serial testPort;
String[] ports;
int portIndex = 0;
boolean sentReadyRequest = false;
long readyRequestTime = 0;
int hardwareStartTime = 0;
int ax, ay, az;

int initializationStep = 0;
boolean waiting = false;
int waitDuration = 0;
int waitStartTime = 0;
String[] logMessages = new String[4];
String currentMessage = "";

float cameraRotationXIncrement = 0;
float cameraRotationYIncrement = 0;
float cameraRotationX = -PI / 8;
float cameraRotationY = 0;
float cameraRotationSpeed = 0.01;
float zoomLevel = 1;

String controlInstructionsManipulator =
  "CONTROL (KEYBOARD)\n" +
  "\n" +
  "  A / D   : Base\n" +
  "  W / S   : Upperarm\n" +
  "  R / F   : Forearm\n" +
  "  T / G   : Wrist Vertical\n" +
  "  Q / E   : Wrist Rotate\n" +
  "  Z / X   : Gripper\n" +
  "        N   : Connect\n" +
  "        C   : Enable/Disable Collision\n" +
  "        O   : Enable/Disable Trace\n" +
  "        L   : Enable/Disable Leap Motion\n" +
  "        J   : Enable/Disable Joystick\n" +
  " 1 / 2 / 3 / 4 / 5 : Controls of Action\n" +
  "  ← / → / ↑ / ↓ / - / + : Rotate camera";

String controlInstructionsVehicle =
  "CONTROL (KEYBOARD)\n" +
  "\n" +
  "        N   : Connect\n" +
  "        C   : Enable/Disable Collision\n" +
  "        O   : Enable/Disable Trace\n" +
  "        L   : Enable/Disable Leap Motion\n" +
  "        J   : Enable/Disable Joystick\n" +
  " 1 / 2 / 3 / 4 / 5 : Controls of Action\n" +
  "  ← / → / ↑ / ↓ / - / + : Rotate camera";

String controlInstructionsDrone =
  "CONTROL (KEYBOARD)\n" +
  "\n" +
  "        N   : Connect\n" +
  "        C   : Enable/Disable Collision\n" +
  "        O   : Enable/Disable Trace\n" +
  "        L   : Enable/Disable Leap Motion\n" +
  "        J   : Enable/Disable Joystick\n" +
  " 1 / 2 / 3 / 4 / 5 : Controls of Action\n" +
  "  ← / → / ↑ / ↓ / - / + : Rotate camera";

boolean[] keyPressedStates = new boolean[8];
int[] keyboardIncrement = new int[8];
int NUM_KEYBOARD_CONTROLS = 8;
float KEYBOARD_ZOOM_FACTOR = 1.05f;
float MIN_ZOOM = 0.5f;
float MAX_ZOOM = 5.0f;

// --- Ground & base alignment ---
final float GROUND_Y = 0.0f;        // The ground plane's Y-coordinate

void setup() {
  size(1100, 650, P3D);
  frameRate(30);

  try {
    String base64Data = base64String.split(",")[1];
    byte[] imageBytes = Base64.getDecoder().decode(base64Data);
    java.awt.image.BufferedImage bimg = ImageIO.read(new ByteArrayInputStream(imageBytes));
    groundLogo = new PImage(bimg.getWidth(), bimg.getHeight(), ARGB);
    bimg.getRGB(0, 0, bimg.getWidth(), bimg.getHeight(), groundLogo.pixels, 0, bimg.getWidth());
    groundLogo.updatePixels();
  }
  catch (Exception e) {
  }

  try {
    wsServer = new WebsocketServer(this, WS_PORT, WS_PATH);
    updateMessage("WS server online (port " + WS_PORT + ")");
  }
  catch (Exception e) {
    updateMessage("WS server failed");
  }

  initializeLog();
  loadServoLimits();
  loadHoming();

  mpuHUD = (PGraphics3D) createGraphics(200, 200, P3D);

  for (int i = 0; i < actions.length; i++) {
    actions[i] = new ArrayList<int[]>();
  }
  loadAllActions();

  loadOrCreateConfig();

  updateMessage("");
}

void draw() {
  background(245);

  if (isManipulatorSelected) {
    drawManipulator3D();
  }

  if (isVehicleSelected) {
    drawVehicle3D();
  }

  if (isDroneSelected) {
    drawDrone3D();
  }

  hint(DISABLE_DEPTH_TEST);

  drawSidebar();
  drawHUD();
  updateTimeDisplay();
  stepsSystem();

  hint(ENABLE_DEPTH_TEST);

  try {
    if (frameCount % 6 == 0) {
      sendFrameImage();
    }
  }
  catch (Exception e) {
    println("Error sending frame via websocket" + e.getMessage());
  }
}

void stepsSystem() {
  switch (initializationStep) {
  case 0:
    break;
  case 1:
    checkLastPort();
    break;
  case 2:
    listSerialPorts();
    break;
  case 3:
    testSerialPorts();
    break;
  case 4:
    connectHardware();
    break;
  case 5:
    updateControls();
    break;
  case 6:
    simulationMode();
    break;
  }
}

void selectMode(String mode) {
  isManipulatorSelected = false;
  isVehicleSelected = false;
  isDroneSelected = false;

  if (mode.equals("Manipulator")) {
    isManipulatorSelected = true;
  } else if (mode.equals("Vehicle")) {
    isVehicleSelected = true;
  } else if (mode.equals("Drone")) {
    isDroneSelected = true;
  }
}

void updateControls() {

  goHome();

  if (systemReady) {
    updateEnvironmentFromSonar();
  }

  if (!goingHome) {
    if (systemReady || simulationMode) {
      handleRecording();
      handlePlayback();
      updateWithKeyboard();
      if (enableJoystick) updateWithJoystick();
      if (enableLeap) updateWithLeap();
    }
    processPendingWsCommands();
  }
}

void updateMessage(String newMsg) {
  textSize(14);
  fill(0);
  for (int i = logMessages.length - 1; i > 0; i--) {
    logMessages[i] = logMessages[i - 1];
  }
  logMessages[0] = newMsg;
  currentMessage = newMsg;

  if (wsServer != null) {
    JSONObject j = new JSONObject();
    j.setString("log", newMsg);
    wsServer.sendMessage(j.toString());
  }
}

void initializeLog() {
  textSize(14);
  fill(0);
  for (int i = 0; i < logMessages.length; i++) {
    logMessages[i] = "";
  }
}

void clearLog() {
  for (int i = 0; i < logMessages.length; i++) {
    logMessages[i] = "";
  }
}

void startWait(int duration) {
  waiting = true;
  waitDuration = duration;
  waitStartTime = millis();
}

boolean checkWait() {
  if (waiting && millis() - waitStartTime >= waitDuration) {
    waiting = false;
    return true;
  }
  return false;
}

void sendSystemStatus() {
  if (wsServer == null) return;

  JSONObject state = new JSONObject();
  state.setBoolean("leapEnabled", enableLeap);
  state.setBoolean("leapAvailable", leapAvailable);
  state.setBoolean("joystickEnabled", enableJoystick);
  state.setBoolean("collisionSetection", collisionSet);
  state.setString("status", systemReady ? "connected" : "disconnected");
  state.setBoolean("simulation", simulationMode);

  wsServer.sendMessage(state.toString());
}

static BufferedImage pimageToBuffered(PImage src) {
  src.loadPixels();
  int w = src.width, h = src.height;

  int[] rgb = new int[w * h];
  for (int i = 0; i < rgb.length; i++) {
    rgb[i] = src.pixels[i] & 0x00FFFFFF;
  }

  BufferedImage out = new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB);
  out.setRGB(0, 0, w, h, rgb, 0, w);
  return out;
}

void sendFrameImage() {
  try {
    PImage img = get();

    img.resize(600, 400);

    BufferedImage bimg = pimageToBuffered(img);

    ByteArrayOutputStream baos = new ByteArrayOutputStream(128 * 1024);
    ImageIO.write(bimg, "jpg", baos);
    byte[] jpgBytes = baos.toByteArray();
    baos.close();

    String base64 = Base64.getEncoder().encodeToString(jpgBytes);

    if (base64.length() > 2_000_000) {
      println("Frame too large, not sending (" + base64.length() + " chars)");
      return;
    }

    JSONObject frameData = new JSONObject();
    frameData.setString("frame", base64);
    String payload = frameData.toString();

    if (wsServer != null) {

      wsServer.sendMessage(payload);
    }
  }
  catch (Exception e) {
    println("Error sending frame image: " + e.getMessage());
  }
}

// ==============================
// Record and Play
// ==============================

void handleActionButton(int index) {
  if (!systemReady && !simulationMode) {
    return;
  }
  if (recording && selectedAction == index) {
    recording = false;
    File file = new File(sketchPath(ACTION_FILE + index + ".json"));
    if (!file.exists()) {
      JSONArray jsonArray = new JSONArray();
      for (int[] frame : actions[index]) {
        JSONArray frameArray = new JSONArray();
        for (int val : frame) frameArray.append(val);
        jsonArray.append(frameArray);
      }
      try {
        saveJSONArray(jsonArray, file.getAbsolutePath());
        actionSlotLocked[index] = true;
        updateMessage("Action " + (index + 1) + " saved successfully");
      }
      catch (Exception e) {
        e.printStackTrace();
        updateMessage("Failed to save action " + (index + 1));
      }
    } else {
      updateMessage("File already exists. Recording not overwritten");
    }
    selectedAction = -1;
    return;
  }
  if (playingBack[index]) {
    playingBack[index] = false;
    updateMessage("Stopped playback of action " + (index + 1));
    return;
  }
  if (!actionSlotLocked[index] && !recording && !isPlaying()) {
    recording = true;
    selectedAction = index;
    recordingStartTime = millis();
    actions[selectedAction].clear();
    actions[index].clear();
    updateMessage("Recording action " + (index + 1));
    return;
  }
  if (actionSlotLocked[index] && !recording && !isPlaying()) {
    playingBack[index] = true;
    tip = null;
    trajectoryPoints = new ArrayList<PVector>();
    playbackIndex[index] = 0;
    lastFrameTime[index] = millis();
    updateMessage("Playing action " + (index + 1));
    return;
  }
  updateMessage("Another action is already active or this slot is locked");
}

void loadAllActions() {
  for (int i = 0; i < actions.length; i++) {
    actions[i].clear();
    String path = sketchPath(ACTION_FILE + i + ".json");
    File file = new File(path);
    if (file.exists()) {
      actionSlotLocked[i] = true;
      try {
        JSONArray jsonArray = loadJSONArray(path);
        if (jsonArray != null) {
          for (int j = 0; j < jsonArray.size(); j++) {
            JSONArray frameArray = jsonArray.getJSONArray(j);
            int[] frame = new int[frameArray.size()];
            for (int k = 0; k < frame.length; k++) {
              frame[k] = frameArray.getInt(k);
            }
            actions[i].add(frame);
          }
        }
      }
      catch (Exception e) {
        println("Failed to load action_" + i + ".json: " + e.getMessage());
      }
    } else {
      actionSlotLocked[i] = false;
    }
  }
}

void updateTimeDisplay() {
  if (recording && selectedAction >= 0) {
    int elapsed = millis() - recordingStartTime;
    int remaining = maxRecordingDuration - elapsed;
    remaining = max(0, remaining);
    int remSec = remaining / 1000;
    timeDisplay = String.format("Recording: %02d:%02d remaining", remSec / 60, remSec % 60);
    return;
  }
  for (int i = 0; i < actions.length; i++) {
    if (playingBack[i]) {
      int totalFrames = actions[i].size();
      int currentFrame = playbackIndex[i];
      int totalSecs = (totalFrames * playbackInterval) / 1000;
      int currentSecs = (currentFrame * playbackInterval) / 1000;
      timeDisplay = String.format("Action %d: %02d:%02d / %02d:%02d",
        i + 1, currentSecs / 60, currentSecs % 60, totalSecs / 60, totalSecs % 60);
      return;
    }
  }
  timeDisplay = "No action selected";
}

void handleRecording() {
  if (recording && selectedAction >= 0 && millis() - lastRecordTime >= recordingInterval) {
    if (millis() - recordingStartTime >= maxRecordingDuration) {
      recording = false;
      updateMessage("Recording stopped: maximum time reached");
      return;
    }
    int[] frame = {
      angles[BASE_IDX], angles[UPPERARM_IDX], angles[FOREARM_IDX],
      angles[WRIST_VERT_IDX], angles[WRIST_ROT_IDX], angles[GRIPPER_IDX]
    };
    actions[selectedAction].add(frame);
    lastRecordTime = millis();
  }
}

void handlePlayback() {
  for (int i = 0; i < actions.length; i++) {
    if (playingBack[i] && millis() - lastFrameTime[i] > playbackInterval) {
      if (playbackIndex[i] < actions[i].size()) {
        int[] frame = actions[i].get(playbackIndex[i]);
        setAngle(mapIndexToChannel(BASE_IDX), frame[BASE_IDX]);
        setAngle(mapIndexToChannel(UPPERARM_IDX), frame[UPPERARM_IDX]);
        setAngle(mapIndexToChannel(FOREARM_IDX), frame[FOREARM_IDX]);
        setAngle(mapIndexToChannel(WRIST_VERT_IDX), frame[WRIST_VERT_IDX]);
        setAngle(mapIndexToChannel(WRIST_ROT_IDX), frame[WRIST_ROT_IDX]);
        setAngle(mapIndexToChannel(GRIPPER_IDX), frame[GRIPPER_IDX]);
        playbackIndex[i]++;
        lastFrameTime[i] = millis();
      } else {
        playingBack[i] = false;
        playbackIndex[i] = 0;
        updateMessage("Action " + (i + 1) + " playback finished");
      }
    }
  }
}

// ==============================
// Mouse
// ==============================

void mouseDragged() {
  if (mouseX < 300) return;
  float dx = (mouseX - pmouseX);
  float dy = (mouseY - pmouseY);
  cameraRotationY += dx * cameraRotationSpeed;
  cameraRotationX += dy * cameraRotationSpeed;
  cameraRotationX = constrain(cameraRotationX, -PI / 2.5f, PI / 2.5f);
}

void mousePressed() {
  final int CONNECT_BUTTON_X1 = 20;
  final int CONNECT_BUTTON_X2 = 280;
  final int CONNECT_BUTTON_Y1 = height - 130;
  final int CONNECT_BUTTON_Y2 = height - 95;
  if (mouseX >= CONNECT_BUTTON_X1 && mouseX <= CONNECT_BUTTON_X2 &&
    mouseY >= CONNECT_BUTTON_Y1 && mouseY <= CONNECT_BUTTON_Y2) {
    connect();
    return;
  }
  final int ACTION_BUTTON_START_Y = 280;
  final int ACTION_BUTTON_HEIGHT = 20;
  final int ACTION_BUTTON_SPACING = 30;
  for (int i = 0; i < 5; i++) {
    int y1 = ACTION_BUTTON_START_Y + i * ACTION_BUTTON_SPACING;
    if (mouseX >= 20 && mouseX <= 280 &&
      mouseY >= y1 && mouseY <= y1 + ACTION_BUTTON_HEIGHT) {
      handleActionButton(i);
      return;
    }
  }

  if (maximized) {
    if (mouseX > 20 && mouseX < 20 + 260 && mouseY > 520 && mouseY < 520 + 35) {
      selectMode("Manipulator");
    }
    if (mouseX > 20 && mouseX < 20 + 260 && mouseY > 565 && mouseY < 565 + 35) {
      selectMode("Vehicle");
    }
    if (mouseX > 20 && mouseX < 20 + 260 && mouseY > 610 && mouseY < 610 + 35) {
      selectMode("Drone");
    }
  }
}

void mouseWheel(MouseEvent event) {
  float scroll = event.getCount();
  if (width > 1100 && height > 650) {
    float zoomAmount = pow(1.05f, -scroll);
    zoomLevel *= zoomAmount;
    zoomLevel = constrain(zoomLevel, 0.5f, 5.0f);
    zoomTarget.set(mouseX, mouseY, 0);
  }
}
