// =====================================================================
// SynROV Processing - Runtime frame pipeline
// ---------------------------------------------------------------------
// Purpose:
//   Groups the per-frame update/render/stream order in one place. Robot modules
//   remain responsible for their own physics and drawing while this pipeline
//   coordinates serial I/O, collision refresh, clean 3D capture, UI and the independent network streams.
// =====================================================================

// Renders the currently selected robot scene.
void renderActiveRobotScene(boolean advanceCamera) {
  activeModule().drawModule3D(advanceCamera);
}

// Captures only the already-rendered 3D viewport. Call after robot rendering
// and before drawing sidebar/HUD/diagnostics overlays.
PImage captureActive3DViewport() {
  int captureX = int(leftSidebarWidth() + wsStreamCaptureMargin);
  int captureY = wsStreamCaptureMargin;
  int captureW = max(64, width - captureX - wsStreamCaptureMargin);
  int captureH = max(64, height - captureY - wsStreamCaptureMargin);

  captureX = constrain(captureX, 0, max(0, width - 1));
  captureY = constrain(captureY, 0, max(0, height - 1));
  captureW = min(captureW, max(1, width - captureX));
  captureH = min(captureH, max(1, height - captureY));
  return get(captureX, captureY, captureW, captureH);
}

// Returns true when the browser live-view frame can be sent without starving
// the serial loop. AiBot explicitly opts out of this channel and uses port 9001.
boolean shouldSendWebSocketFrameNow() {
  if (!webClientConnected || wsServer == null) return false;
  if (!isWebVisualStreamRequested()) return false;
  long now = millis();
  if ((now - lastWebSocketFrameSentMillis) < max(80, webSocketFrameIntervalMs)) return false;
  if (systemReady && !simulationMode && myPort != null && lastHardwareRxMillis > 0) {
    if ((now - lastHardwareRxMillis) > max(600, hardwareTelemetryTimeoutMs / 2)) return false;
  }
  return true;
}

void updateRuntimeBeforeRender() {
  syncViewportAfterResize();
  processSerialIO();
  processPendingWsCommands();
  updateRosIntegration();
  updateRobotCameraStreamService();
  flushPendingWebMotionIntentsEarly();
  processPendingRemoteManipulatorProgress();
  updateEnvironmentScan();

  if (collisionSet && isManipulatorSelected) {
    long nowMs = millis();
    if ((nowMs - lastCollisionRefreshMs) >= COLLISION_REFRESH_INTERVAL_MS) {
      lastCollisionRefreshMs = nowMs;
      refreshLocalCollisionState();
    }
  } else if (!collisionSet) {
    isColliding = false;
  }

  activeModule().updatePhysicsSafe();
  updateRobotAttitude3DState();
}

void drawRuntimeUiAfterScene() {
  hint(DISABLE_DEPTH_TEST);
  applySynUiFont();
  drawSidebar();
  drawHUD();
  updateTimeDisplay();
  stepsSystem();
  flushPendingControlIntents();
  drawDiagnosticsPanelOrToggle();
  drawWorldTransformOverlay();
  hint(ENABLE_DEPTH_TEST);
}

void publishRuntimeVisualStreams(PImage cleanSceneFrame, boolean sendWebFrame, boolean sendAiFrame) {
  if (cleanSceneFrame == null) return;
  if (sendWebFrame) sendFrameImage(cleanSceneFrame);
  if (sendAiFrame) sendAiPerceptionFrame(cleanSceneFrame);
}

void runRuntimeFrame() {
  updateRuntimeBeforeRender();

  boolean sendWebFrame = shouldSendWebSocketFrameNow();
  boolean sendAiFrame = shouldSendAiPerceptionFrameNow();
  boolean captureScene = sendWebFrame || sendAiFrame;
  PImage cleanSceneFrame = null;

  background(sceneBackgroundColor());
  updateActionButtonLongPress();
  renderActiveRobotScene(true);

  if (captureScene) {
    try {
      cleanSceneFrame = captureActive3DViewport();
    }
    catch (Exception e) {
      println("[SynROV][Stream] viewport capture failed: " + e.getMessage());
    }
  }

  drawRuntimeUiAfterScene();

  try {
    publishRuntimeVisualStreams(cleanSceneFrame, sendWebFrame, sendAiFrame);
  }
  catch (Exception e) {
    println("[SynROV][Stream] publish failed: " + e.getMessage());
  }
}
