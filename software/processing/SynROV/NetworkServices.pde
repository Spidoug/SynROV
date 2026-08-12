// =====================================================================
// SynROV Processing - Network services
// ---------------------------------------------------------------------
// Purpose:
//   Own control/state WebSocket transport configuration and startup. Image
//   traffic is delegated to VisualStreamServices, AiPerceptionStream and the
//   dedicated RobotCameraStream service so command latency is independent from
//   browser, AI-world or physical-camera JPEG traffic.
// =====================================================================

WebsocketServer wsServer;
final int WS_PORT = 9000;
final String WS_PATH = "/";
final int WS_COMMAND_QUEUE_MAX = 96;
final int WS_COMMANDS_PER_FRAME = 6;
final int WS_COMMAND_PROCESS_BUDGET_MS = 8;
ArrayDeque<String> pendingWsCommands = new ArrayDeque<String>();
boolean webSocketSendWarningLatched = false;
boolean webSocketQueueWarningLatched = false;

void initializeNetworkServices() {
  try {
    wsServer = new WebsocketServer(this, WS_PORT, WS_PATH);
    updateMessage(tr("WebSocket server online on port ") + WS_PORT);
  }
  catch (Exception e) {
    wsServer = null;
    updateMessage(tr("WebSocket server failed"));
  }

  initializeAiPerceptionServer();
  initializeRobotCameraServer();
}
