// =====================================================================
// SynROV Processing - Diagnostics panel
// ---------------------------------------------------------------------
// Purpose:
//   Visual diagnostics, operator readouts and debugging widgets.
// =====================================================================

PFont diagnosticsFont;

int diagnosticsPanelW = 500;
int diagnosticsPanelH = 940;
final int DIAGNOSTICS_PANEL_MIN_W = 430;
final int DIAGNOSTICS_PANEL_MAX_W = 500;
final int DIAGNOSTICS_PANEL_CONTENT_MARGIN = 18;
int diagnosticsPanelMarginRight = 20;
int diagnosticsPanelMarginBottom = 14;

boolean diagnosticsGraphsWindowOpen = false;
int diagnosticsGraphsPage = 0;
final int DIAGNOSTICS_GRAPHS_PER_PAGE = 4;
final int DIAGNOSTICS_SENSOR_HISTORY_LIMIT = 240;
HashMap<String, ArrayList<Float>> diagnosticsSensorHistory = new HashMap<String, ArrayList<Float>>();
ArrayList<String> diagnosticsSensorNumericKeys = new ArrayList<String>();

boolean diagnosticsSensorRecording = false;
PrintWriter diagnosticsSensorRecordWriter = null;
String diagnosticsSensorRecordFilename = "";
ArrayList<String> diagnosticsSensorRecordHeaders = new ArrayList<String>();

int diagnosticsActiveSlider = -1;

final String[] DIAGNOSTICS_ARM_LIVE_RECORD_KEYS = {
  "arm_live_base_deg", "arm_live_upper_deg", "arm_live_fore_deg", "arm_live_forearm_roll_deg", "arm_live_wrist_pitch_deg", "arm_live_wrist_rot_deg", "arm_live_grip_deg"
};
final String[] DIAGNOSTICS_ARM_TARGET_RECORD_KEYS = {
  "arm_target_base_deg", "arm_target_upper_deg", "arm_target_fore_deg", "arm_target_forearm_roll_deg", "arm_target_wrist_pitch_deg", "arm_target_wrist_rot_deg", "arm_target_grip_deg"
};

String manipulatorNeutralLabel(int memberIdx) {
  switch(memberIdx) {
  case 0: return tr("Base", "Base");
  case 1: return tr("Braço superior", "Upper arm");
  case 2: return tr("Antebraço", "Forearm");
  case 3: return tr("Rotação do antebraço", "Forearm roll");
  case 4: return tr("Inclinação do punho", "Wrist pitch");
  case 5: return tr("Rotação do punho", "Wrist roll");
  case 6: return tr("Garra", "Gripper");
  }
  return tr("Junta", "Joint");
}

final int DIAGNOSTICS_MODULE_INSET_X = 12;
final int DIAGNOSTICS_MODULE_GAP = 12;

final float DIAGNOSTICS_CARD_TEXT_SIZE = 11.0f;
final float DIAGNOSTICS_CARD_TEXT_MIN_SIZE = 9.5f;
final int DIAGNOSTICS_CARD_TEXT_INSET = 10;
final int DIAGNOSTICS_BUTTON_H = 24;
final int DIAGNOSTICS_BUTTON_ROW_STEP = 30;
final int DIAGNOSTICS_SLIDER_ROW_STEP = 36;
final int DIAGNOSTICS_SLIDER_BAR_H = 10;
final int DIAGNOSTICS_SLIDER_HIT_H = 38;
final int DIAGNOSTICS_SECTION_RADIUS = 10;
final int DIAGNOSTICS_SECTION_GAP = 8;

final int DIAGNOSTICS_MAP_CARD_H = 110;
final int DIAGNOSTICS_MANIP_RUNTIME_CARD_H = 110;
final int DIAGNOSTICS_MANIP_NEUTRAL_CARD_H = 170;
final int DIAGNOSTICS_MANIP_SENSORS_CARD_H = 210;
final int DIAGNOSTICS_MANIP_FORCE_CARD_H = 196;
final int DIAGNOSTICS_VEHICLE_RUNTIME_CARD_H = 140;
final int DIAGNOSTICS_VEHICLE_SENSORS_CARD_H = 433;
final int DIAGNOSTICS_DRONE_RUNTIME_CARD_H = 140;
final int DIAGNOSTICS_DRONE_SENSORS_CARD_H = 404;

// Utility: diagnostics is static sensor key.
boolean diagnosticsIsStaticSensorKey(String key) {
  if (key == null) return false;
  String k = trim(key).toLowerCase();
  if (k.length() == 0) return false;

  if (k.equals("geometry_units") || k.equals("static") || k.equals("arm_gripper_y") || k.equals("base_source_mode")) return true;
  if (k.startsWith("srvlim_") || k.startsWith("arm_map_")) return true;
  if (k.startsWith("arm_base_") || k.startsWith("arm_upper_") || k.startsWith("arm_fore_") || k.startsWith("arm_wrist_") || k.startsWith("arm_finger_")) return true;

  String[] staticExactKeys = {
    "vehicle_body_length", "vehicle_body_width", "vehicle_body_height",
    "vehicle_track_gauge", "vehicle_track_run", "vehicle_drive_radius",
    "vehicle_support_radius", "vehicle_top_roller_radius", "vehicle_track_width",
    "vehicle_track_link_length", "vehicle_track_link_thickness", "vehicle_track_height",
    "vehicle_track_links", "vehicle_track_sag", "vehicle_body_center_y",
    "vehicle_camera_default_pan_deg", "vehicle_camera_default_tilt_deg",
    "vehicle_lidar_mast_y", "vehicle_lidar_mast_z",
    "vehicle_camera_head_y", "vehicle_camera_head_z",
    "vehicle_motor_deadband_pct", "vehicle_motor_pwm_min", "vehicle_motor_pwm_max",
    "vehicle_motor_slew_pct_per_update", "vehicle_left_motor_invert", "vehicle_right_motor_invert",
    "vehicle_camera_pan_min_deg", "vehicle_camera_pan_max_deg", "vehicle_camera_tilt_min_deg", "vehicle_camera_tilt_max_deg",
    "vehicle_lidar_sweep_min_deg", "vehicle_lidar_sweep_max_deg", "vehicle_lidar_sweep_cycle_ms",
    "drone_body_length", "drone_body_width", "drone_body_height",
    "drone_arm_length", "drone_arm_thickness",
    "drone_motor_radius", "drone_motor_height",
    "drone_prop_radius", "drone_prop_thickness",
    "drone_leg_height", "drone_leg_span",
    "drone_rest_y", "drone_visual_yaw_offset_deg",
    "drone_camera_y", "drone_camera_z",
    "drone_lamp_y", "drone_lamp_z",
    "drone_esc_min_us", "drone_esc_max_us", "drone_esc_idle_us", "drone_throttle_range_us",
    "drone_pitch_mix_us", "drone_roll_mix_us", "drone_yaw_mix_us", "drone_command_deadband_pct",
    "drone_spool_step_us_per_update"
  };

  for (int i = 0; i < staticExactKeys.length; i++) {
    if (k.equals(staticExactKeys[i])) return true;
  }
  return false;
}


// Utility: diagnostics ellipsize.
String diagnosticsEllipsize(String value, float maxW) {
  String v = value == null ? "" : value;
  if (textWidth(v) <= maxW) return v;
  String suffix = "...";
  while (v.length() > 1 && textWidth(v + suffix) > maxW) {
    v = v.substring(0, v.length() - 1);
  }
  return v + suffix;
}

// Utility: draw diagnostics fitted text left.
void drawDiagnosticsFittedTextLeft(String value, float x, float y, float maxW) {
  String v = value == null ? "" : value;
  textAlign(LEFT, TOP);
  for (float sz = DIAGNOSTICS_CARD_TEXT_SIZE; sz >= DIAGNOSTICS_CARD_TEXT_MIN_SIZE; sz -= 0.5f) {
    textSize(sz);
    if (textWidth(v) <= maxW) {
      text(v, x, y);
      textSize(12);
      return;
    }
  }
  textSize(DIAGNOSTICS_CARD_TEXT_MIN_SIZE);
  text(diagnosticsEllipsize(v, maxW), x, y);
  textSize(12);
}

// Utility: draw diagnostics fitted text centered.
void drawDiagnosticsFittedTextCentered(String value, float cx, float cy, float maxW, float preferredSize, float minSize) {
  String v = value == null ? "" : value;
  textAlign(CENTER, CENTER);
  for (float sz = preferredSize; sz >= minSize; sz -= 0.5f) {
    textSize(sz);
    if (textWidth(v) <= maxW) {
      text(v, cx, cy);
      textSize(12);
      return;
    }
  }
  textSize(minSize);
  text(diagnosticsEllipsize(v, maxW), cx, cy);
  textSize(12);
}


// Utility: diagnostics module inner x.
int diagnosticsModuleInnerX(int moduleX) {
  return moduleX + DIAGNOSTICS_MODULE_INSET_X;
}

// Utility: diagnostics module inner w.
int diagnosticsModuleInnerW(int moduleW) {
  return max(40, moduleW - DIAGNOSTICS_MODULE_INSET_X * 2);
}

// Utility: diagnostics two col button w.
int diagnosticsTwoColButtonW(int moduleW) {
  return (diagnosticsModuleInnerW(moduleW) - DIAGNOSTICS_MODULE_GAP) / 2;
}

final int DIAGNOSTICS_MANIPULATOR_MIN_VISIBLE_H = 888;
final int DIAGNOSTICS_VEHICLE_MIN_VISIBLE_H = 760;
final int DIAGNOSTICS_DRONE_MIN_VISIBLE_H = 760;

// Utility: diagnostics panel visible.
boolean diagnosticsPanelVisible() {
  if (width < 1100) return false;
  if (isManipulatorSelected) return height >= DIAGNOSTICS_MANIPULATOR_MIN_VISIBLE_H;
  if (isVehicleSelected) return height >= DIAGNOSTICS_VEHICLE_MIN_VISIBLE_H;
  if (isDroneSelected) return height >= DIAGNOSTICS_DRONE_MIN_VISIBLE_H;
  return height >= 560;
}

// Utility: preferred diagnostics panel width.
int preferredDiagnosticsPanelWidth() {
  int available = round(width - leftSidebarWidth() - diagnosticsPanelMarginRight - 20);
  return constrain(available, DIAGNOSTICS_PANEL_MIN_W, DIAGNOSTICS_PANEL_MAX_W);
}

// Utility: preferred diagnostics panel height.
int preferredDiagnosticsPanelHeight() {
  if (isManipulatorSelected) return min(920, height);
  if (isVehicleSelected) return min(790, height);
  if (isDroneSelected) return min(760, height);
  return min(560, height);
}

// Utility: diagnostics panel x.
int diagnosticsPanelX() {
  diagnosticsPanelW = preferredDiagnosticsPanelWidth();
  return width - diagnosticsPanelW - diagnosticsPanelMarginRight;
}

// Utility: diagnostics panel y.
int diagnosticsPanelY() {
  return max(0, height - diagnosticsPanelH - diagnosticsPanelMarginBottom);
}

// Ensures diagnostics hud.
void ensureDiagnosticsHUD(int w, int h) {
  if (diagnosticsFont == null) {
    diagnosticsFont = createFont("Arial", 12);
  }
}

// Draws a compact diagnostics button using the shared UI palette.
void drawMiniButton2D(int x, int y, int w, int h, String label, boolean active) {
  drawMiniButton2D(x, y, w, h, label, active, true);
}

// Draws a compact diagnostics button using the shared UI palette.
void drawMiniButton2D(int x, int y, int w, int h, String label, boolean active, boolean enabled) {
  synDrawButtonSurface2D(x, y, w, h, active, enabled, 8, 1.5f);
  fill(!enabled ? uiSecondaryTextColor() : uiPrimaryTextColor());
  drawDiagnosticsFittedTextCentered(label, x + w / 2, y + h / 2 - 1, w - 14, 11, 9.5f);
}

// Draws a diagnostics card with the same surface palette as the left sidebar.
void drawSectionCard2D(int x, int y, int w, int h, String title) {
  synDrawCardShell2D(x, y, w, h, DIAGNOSTICS_SECTION_RADIUS);
  fill(uiPrimaryTextColor());
  drawDiagnosticsFittedTextLeft(title, x + 16, y + 12, w - 32);
}

int diagnosticsTargetSurfaceColor() {
  return isDarkThemeEnabled() ? color(226, 229, 234) : color(255);
}

int diagnosticsTargetGridColor() {
  return isDarkThemeEnabled() ? color(132, 138, 148) : color(205);
}

int diagnosticsTargetTextColor() {
  return color(28, 32, 38);
}

// Draws a diagnostics pressure/status bar.
void drawPressureBar2D(float x, float y, float w, float h, float value, float maxV, String label) {
  stroke(uiButtonStrokeColor());
  fill(uiButtonIdleColor());
  rect(x, y, w, h, 6);
  float safeMax = max(0.0001f, maxV);
  float fw = constrain(map(value, 0, safeMax, 0, w), 0, w);
  noStroke();
  fill(255, 190, 0);
  rect(x, y, fw, h, 6);
  fill(uiPrimaryTextColor());
  textAlign(LEFT, CENTER);
  text(label + " " + nf(value, 1, 1), x + 8, y + h / 2);
}

// Data container for one stability target shown in the diagnostics 3D attitude widget.
class DiagnosticsStabilityTarget3D {
  String label;
  float rollDeg;
  float pitchDeg;
  float yawDeg;
  float velocityCms;
  boolean active;
  int markerStyle;

  DiagnosticsStabilityTarget3D(String label, float rollDeg, float pitchDeg, float yawDeg, float velocityCms, boolean active, int markerStyle) {
    this.label = label;
    this.rollDeg = rollDeg;
    this.pitchDeg = pitchDeg;
    this.yawDeg = yawDeg;
    this.velocityCms = velocityCms;
    this.active = active;
    this.markerStyle = markerStyle;
  }
}

// Utility: active color for a stability target marker.
int diagnosticsStabilityTargetColor(int idx, boolean active) {
  if (!active) return color(132, 138, 148);
  if (idx == 0) return color(0, 126, 255);
  if (idx == 1) return color(255, 166, 0);
  return color(80, 190, 120);
}

// Utility: normalized attitude component in the visual target plane.
float diagnosticsAttitudeNorm(float valueDeg, float maxAbsDeg) {
  return constrain(valueDeg / max(0.0001f, maxAbsDeg), -1.0f, 1.0f);
}

// Utility: draw a compact roll/pitch/yaw component bar.
void drawStabilityComponentBar2D(float x, float y, float w, String axisLabel, float valueDeg, int c) {
  float centerX = x + w * 0.5f;
  stroke(diagnosticsTargetGridColor());
  line(x, y, x + w, y);
  line(centerX, y - 3, centerX, y + 3);
  float nx = diagnosticsAttitudeNorm(valueDeg, 45.0f);
  float ex = centerX + nx * w * 0.48f;
  stroke(c);
  strokeWeight(3);
  line(centerX, y, ex, y);
  noStroke();
  fill(diagnosticsTargetTextColor());
  textAlign(LEFT, CENTER);
  textSize(9.5f);
  text(axisLabel + " " + nf(valueDeg, 1, 0) + "°", x, y - 9);
  strokeWeight(1);
}

// Utility: draw one marker in the pseudo-3D stability target plane.
void drawStabilityTargetMarker2D(float cx, float cy, float rr, DiagnosticsStabilityTarget3D target, int idx) {
  if (target == null) return;
  int c = diagnosticsStabilityTargetColor(idx, target.active);
  float pitchNorm = diagnosticsAttitudeNorm(target.pitchDeg, 45.0f);
  float rollNorm = diagnosticsAttitudeNorm(target.rollDeg, 45.0f);
  float markerX = cx + pitchNorm * rr * 0.78f;
  float markerY = cy - rollNorm * rr * 0.78f;
  float yawRad = radians(target.yawDeg - 90.0f);
  float yawX = markerX + cos(yawRad) * rr * 0.24f;
  float yawY = markerY + sin(yawRad) * rr * 0.24f;

  stroke(c, target.active ? 210 : 135);
  strokeWeight(idx == 0 ? 3 : 2.4f);
  line(cx, cy, markerX, markerY);
  line(markerX, markerY, yawX, yawY);
  noStroke();
  fill(c, target.active ? 230 : 150);

  if (target.markerStyle == 1) {
    rectMode(CENTER);
    rect(markerX, markerY, 11, 11, 2);
    rectMode(CORNER);
  } else if (target.markerStyle == 2) {
    pushMatrix();
    translate(markerX, markerY);
    rotate(PI / 4.0f);
    rectMode(CENTER);
    rect(0, 0, 10, 10, 2);
    rectMode(CORNER);
    popMatrix();
  } else {
    ellipse(markerX, markerY, 12, 12);
  }
  strokeWeight(1);
}

// Utility: draw one compact target data block without repeating external IMU text.
void drawStabilityTargetSummaryBlock2D(float x, float y, float w, float h, DiagnosticsStabilityTarget3D target, int idx) {
  if (target == null) return;
  int c = diagnosticsStabilityTargetColor(idx, target.active);
  float pad = 9;
  boolean compact = h < 60;
  boolean medium = h >= 60 && h < 84;

  stroke(uiBorderColor());
  fill(isDarkThemeEnabled() ? color(242, 244, 248) : color(250, 250, 252));
  rect(x, y, w, h, 8);

  noStroke();
  fill(c, target.active ? 230 : 145);
  if (target.markerStyle == 1) {
    rect(x + pad, y + 10, 10, 10, 2);
  } else if (target.markerStyle == 2) {
    pushMatrix();
    translate(x + pad + 5, y + 15);
    rotate(PI / 4.0f);
    rectMode(CENTER);
    rect(0, 0, 10, 10, 2);
    rectMode(CORNER);
    popMatrix();
  } else {
    ellipse(x + pad + 5, y + 15, 11, 11);
  }

  float textX = x + pad + 18;
  float textW = max(20, w - (textX - x) - pad);
  fill(diagnosticsTargetTextColor());
  textAlign(LEFT, TOP);
  drawDiagnosticsFittedTextLeft(target.label, textX, y + 7, textW);

  textSize(9.5f);
  String attitudeLine = "R " + nf(target.rollDeg, 1, 0) + "°   P " + nf(target.pitchDeg, 1, 0) + "°   Y " + nf(target.yawDeg, 1, 0) + "°";
  if (compact) {
    text(diagnosticsEllipsize(attitudeLine, w - 18), x + pad, y + 28);
    return;
  }

  text(diagnosticsEllipsize(attitudeLine, w - 18), x + pad, y + 30);

  String statusLine = (target.active ? tr("Telemetria ativa", "Telemetry active") : tr("Sem referência ativa", "No active reference"));
  statusLine += "  ·  " + tr("Vel ", "Vel ") + nf(target.velocityCms, 1, 1) + " cm/s";
  if (!medium) {
    text(diagnosticsEllipsize(statusLine, w - 18), x + pad, y + 45);
  }
}

// Draws the per-robot 3D stability target widget.
void drawStabilityTargets3D2D(float x, float y, float w, float h, String label, DiagnosticsStabilityTarget3D[] targets) {
  pushMatrix();
  translate(x, y);
  fill(diagnosticsTargetSurfaceColor());
  stroke(uiBorderColor());
  rect(0, 0, w, h, 10);

  float pad = 10;
  float gap = 10;
  int maxTargets = targets == null ? 0 : min(targets.length, 2);
  int visibleTargets = 0;
  for (int i = 0; i < maxTargets; i++) {
    if (targets[i] != null) visibleTargets++;
  }
  if (visibleTargets <= 0) visibleTargets = 1;

  boolean stackedLayout = (w < 300 || h < 138);
  float plotX = pad;
  float plotY = pad;
  float plotW = stackedLayout ? (w - pad * 2) : max(96, (w - pad * 2 - gap) * 0.54f);
  float infoX = stackedLayout ? pad : (plotX + plotW + gap);
  float infoW = stackedLayout ? (w - pad * 2) : max(72, w - infoX - pad);
  float infoY;
  float infoH;

  if (stackedLayout) {
    float legendH = 16;
    float summaryGap = 8;
    float summaryTotalH = max(48, min(h * 0.42f, 140));
    infoH = summaryTotalH;
    infoY = h - pad - infoH;
    float maxPlotBottom = infoY - summaryGap;
    float availablePlotH = max(54, maxPlotBottom - plotY - legendH);
    plotW = w - pad * 2;
    float cx = plotX + plotW * 0.5f;
    float cy = plotY + availablePlotH * 0.46f;
    float rr = max(18, min(plotW * 0.21f, availablePlotH * 0.36f));

    stroke(diagnosticsTargetGridColor());
    noFill();
    ellipse(cx, cy, rr * 2.0f, rr * 2.0f);
    ellipse(cx, cy, rr * 1.36f, rr * 1.36f);
    line(cx - rr, cy, cx + rr, cy);
    line(cx, cy - rr, cx, cy + rr);
    line(cx - rr * 0.707f, cy + rr * 0.707f, cx + rr * 0.707f, cy - rr * 0.707f);
    line(cx - rr * 0.707f, cy - rr * 0.707f, cx + rr * 0.707f, cy + rr * 0.707f);

    if (targets != null) {
      for (int i = 0; i < maxTargets; i++) {
        drawStabilityTargetMarker2D(cx, cy, rr, targets[i], i);
      }
    }

    fill(diagnosticsTargetTextColor());
    textAlign(CENTER, TOP);
    textSize(9.5f);
    text(tr("Plano pitch × roll; seta indica yaw", "Pitch × roll plane; arrow shows yaw"), cx, max(plotY + 4, maxPlotBottom - 12));
  } else {
    infoY = pad;
    infoH = h - pad * 2;
    float legendH = 16;
    float availablePlotH = h - pad * 2 - legendH;
    float cx = plotX + plotW * 0.5f;
    float cy = plotY + availablePlotH * 0.50f;
    float rr = max(22, min(plotW * 0.28f, availablePlotH * 0.34f));

    stroke(diagnosticsTargetGridColor());
    noFill();
    ellipse(cx, cy, rr * 2.0f, rr * 2.0f);
    ellipse(cx, cy, rr * 1.36f, rr * 1.36f);
    line(cx - rr, cy, cx + rr, cy);
    line(cx, cy - rr, cx, cy + rr);
    line(cx - rr * 0.707f, cy + rr * 0.707f, cx + rr * 0.707f, cy - rr * 0.707f);
    line(cx - rr * 0.707f, cy - rr * 0.707f, cx + rr * 0.707f, cy + rr * 0.707f);

    if (targets != null) {
      for (int i = 0; i < maxTargets; i++) {
        drawStabilityTargetMarker2D(cx, cy, rr, targets[i], i);
      }
    }

    fill(diagnosticsTargetTextColor());
    textAlign(CENTER, TOP);
    textSize(9.5f);
    text(tr("Plano pitch × roll; seta indica yaw", "Pitch × roll plane; arrow shows yaw"), cx, h - pad - 12);
  }

  float blockGap = 8;
  float blockH = max(42, (infoH - blockGap * (visibleTargets - 1)) / visibleTargets);
  float drawY = infoY + max(0, (infoH - (blockH * visibleTargets + blockGap * (visibleTargets - 1))) * 0.5f);
  for (int i = 0; i < maxTargets; i++) {
    DiagnosticsStabilityTarget3D target = targets[i];
    if (target == null) continue;
    drawStabilityTargetSummaryBlock2D(infoX, drawY, infoW, blockH, target, i);
    drawY += blockH + blockGap;
  }

  safePopMatrix("DiagnosticsPanel_Base.pde:drawStabilityTargets3D2D");
}

// Backward-compatible default target widget. Prefer the robot-specific wrappers below.
void drawMpuRadar2D(float x, float y, float w, float h, String label) {
  DiagnosticsStabilityTarget3D[] targets = new DiagnosticsStabilityTarget3D[2];
  boolean active = diagnosticsHasActiveSensorTelemetry();
  targets[0] = new DiagnosticsStabilityTarget3D(tr("Principal", "Main"), mpu1RollDeg, mpu1PitchDeg, mpu1YawDeg, mpuVelocityMagCms[0], active, 0);
  targets[1] = new DiagnosticsStabilityTarget3D(tr("Auxiliar", "Auxiliary"), mpu2RollDeg, mpu2PitchDeg, mpu2YawDeg, mpuVelocityMagCms[1], active, 1);
  drawStabilityTargets3D2D(x, y, w, h, label, targets);
}

// Vehicle stability targets: chassis plus camera/LiDAR head when the robot has a second attitude reference.
void drawVehicleStabilityTargets2D(float x, float y, float w, float h) {
  boolean active = diagnosticsHasActiveSensorTelemetry();
  DiagnosticsStabilityTarget3D[] targets = new DiagnosticsStabilityTarget3D[2];
  targets[0] = new DiagnosticsStabilityTarget3D(tr("Chassi", "Chassis"), mpu1RollDeg, mpu1PitchDeg, getSensorFloatAny(new String[] {"vehicle_yaw_deg", "heading_deg"}, mpu1YawDeg), mpuVelocityMagCms[0], active, 0);
  targets[1] = new DiagnosticsStabilityTarget3D(tr("Câmera/LiDAR", "Camera/LiDAR"), getSensorFloat("vehicle_cam_roll_deg", mpu2RollDeg), getSensorFloat("vehicle_cam_tilt_deg", mpu2PitchDeg), getSensorFloat("vehicle_cam_pan_deg", mpu2YawDeg), mpuVelocityMagCms[1], active && hasAnySensorKey(new String[] {"mpu2_ax", "vehicle_cam_pan_deg", "vehicle_cam_tilt_deg"}), 1);
  drawStabilityTargets3D2D(x, y, w, h, tr("Alvos de estabilidade do veículo", "Vehicle stability targets"), targets);
}

// Drone stability targets: flight body plus camera/gimbal attitude.
void drawDroneStabilityTargets2D(float x, float y, float w, float h) {
  boolean active = diagnosticsHasActiveSensorTelemetry();
  DiagnosticsStabilityTarget3D[] targets = new DiagnosticsStabilityTarget3D[2];
  targets[0] = new DiagnosticsStabilityTarget3D(tr("Corpo de voo", "Flight body"), getSensorFloat("drone_roll_deg", mpu1RollDeg), getSensorFloat("drone_pitch_deg", mpu1PitchDeg), getSensorFloatAny(new String[] {"drone_yaw_deg", "att_yaw_deg", "heading_deg"}, mpu1YawDeg), mpuVelocityMagCms[0], active, 0);
  targets[1] = new DiagnosticsStabilityTarget3D(tr("Câmera/Gimbal", "Camera/Gimbal"), getSensorFloat("drone_cam_roll_deg", mpu2RollDeg), getSensorFloat("drone_cam_tilt_deg", mpu2PitchDeg), getSensorFloat("drone_cam_pan_deg", mpu2YawDeg), mpuVelocityMagCms[1], active && hasAnySensorKey(new String[] {"mpu2_ax", "drone_cam_pan_deg", "drone_cam_tilt_deg"}), 1);
  drawStabilityTargets3D2D(x, y, w, h, tr("Alvos de estabilidade do drone", "Drone stability targets"), targets);
}

// Manipulator stability targets: gripper/wrist stabilizer and arm/base stabilizer are independent.
void drawManipulatorStabilityTargets2D(float x, float y, float w, float h) {
  boolean active = diagnosticsHasActiveSensorTelemetry();
  DiagnosticsStabilityTarget3D[] targets = new DiagnosticsStabilityTarget3D[2];
  targets[0] = new DiagnosticsStabilityTarget3D(tr("Garra/Punho", "Gripper/Wrist"), mpu1RollDeg, mpu1PitchDeg, getSensorFloatAny(new String[] {"wrist_rot_deg", "gimbal_yaw_deg"}, mpu1YawDeg), mpuVelocityMagCms[0], active, 0);
  targets[1] = new DiagnosticsStabilityTarget3D(tr("Braço/Base", "Arm/Base"), mpu2RollDeg, mpu2PitchDeg, getSensorFloatAny(new String[] {"base_yaw_deg", "heading_deg"}, mpu2YawDeg), mpuVelocityMagCms[1], active && hasAnySensorKey(new String[] {"mpu2_ax", "base_yaw_deg", "heading_deg"}), 1);
  drawStabilityTargets3D2D(x, y, w, h, tr("Alvos de estabilidade garra/braço", "Grip/arm stability targets"), targets);
}

// Utility: diagnostics IMU vector line.
String diagnosticsImuVectorLine(String prefix, float[] values, int decimals) {
  return prefix + " " + nf(values[0], 1, decimals) + " / " + nf(values[1], 1, decimals) + " / " + nf(values[2], 1, decimals);
}

// Utility: diagnostics IMU accel line.
String diagnosticsImuAccelLine(String label, int sensorIndex) {
  return diagnosticsImuVectorLine(label + tr(" Acc[g]", " Acc[g]"), mpuAccelG[sensorIndex], 2);
}

// Utility: diagnostics IMU gyro line.
String diagnosticsImuGyroLine(String label, int sensorIndex) {
  return diagnosticsImuVectorLine(label + tr(" Gir[°/s]", " Gyr[dps]"), mpuGyroDps[sensorIndex], 1);
}

// Utility: diagnostics IMU velocity line.
String diagnosticsImuVelocityLine(String label, int sensorIndex) {
  return label + tr(" Vel. est ", " Vel est ") + nf(mpuVelocityMagCms[sensorIndex], 1, 1) + " cm/s";
}

// Utility: point in rect.
boolean pointInRect(int mx, int my, int x, int y, int w, int h) {
  return mx >= x && mx <= x + w && my >= y && my <= y + h;
}

// Utility: point in diagnostics panel.
boolean pointInDiagnosticsPanel(int mx, int my) {
  if (!diagnosticsPanelVisible()) return false;
  diagnosticsPanelW = preferredDiagnosticsPanelWidth();
  diagnosticsPanelH = preferredDiagnosticsPanelHeight();
  int panelX = diagnosticsPanelX();
  int panelY = diagnosticsPanelY();
  return mx >= panelX && mx <= panelX + diagnosticsPanelW &&
    my >= panelY && my <= panelY + diagnosticsPanelH;
}

// Utility: current diagnostics robot key.
String diagnosticsRobotKey() {
  if (isDroneSelected) return "drone";
  if (isVehicleSelected) return "vehicle";
  return "manipulator";
}

// Utility: diagnostics hidden analog channel.
boolean diagnosticsIsUnusedAnalogKey(String key) {
  if (key == null) return false;
  String k = trim(key).toLowerCase();
  if (k.equals("an6") || k.equals("an_6")) return true;
  if (!(k.matches("an_?[0-6]"))) return false;
  String robot = diagnosticsRobotKey();
  if (robot.equals("drone")) return !(k.equals("an4") || k.equals("an_4") || k.equals("an5") || k.equals("an_5"));
  if (robot.equals("vehicle")) return !(k.equals("an2") || k.equals("an_2") || k.equals("an3") || k.equals("an_3"));
  return !(k.equals("an2") || k.equals("an_2") || k.equals("an3") || k.equals("an_3") || k.equals("an4") || k.equals("an_4") || k.equals("an5") || k.equals("an_5"));
}

// Utility: format sensor key label.
String formatSensorKeyLabel(String key) {
  if (key == null) return tr("sensor", "sensor");
  String k = trim(key).toLowerCase();
  if (k.equals("mpu1_ax")) return tr("Acel. principal X bruto", "Main accel X raw");
  if (k.equals("mpu1_ay")) return tr("Acel. principal Y bruto", "Main accel Y raw");
  if (k.equals("mpu1_az")) return tr("Acel. principal Z bruto", "Main accel Z raw");
  if (k.equals("mpu1_gx")) return tr("Giro principal X bruto", "Main gyro X raw");
  if (k.equals("mpu1_gy")) return tr("Giro principal Y bruto", "Main gyro Y raw");
  if (k.equals("mpu1_gz")) return tr("Giro principal Z bruto", "Main gyro Z raw");
  if (k.equals("mpu2_ax")) return tr("Acel. auxiliar X bruto", "Aux accel X raw");
  if (k.equals("mpu2_ay")) return tr("Acel. auxiliar Y bruto", "Aux accel Y raw");
  if (k.equals("mpu2_az")) return tr("Acel. auxiliar Z bruto", "Aux accel Z raw");
  if (k.equals("mpu2_gx")) return tr("Giro auxiliar X bruto", "Aux gyro X raw");
  if (k.equals("mpu2_gy")) return tr("Giro auxiliar Y bruto", "Aux gyro Y raw");
  if (k.equals("mpu2_gz")) return tr("Giro auxiliar Z bruto", "Aux gyro Z raw");
  if (k.equals("heading_deg")) return tr("Rumo da bússola", "Compass heading");
  if (k.equals("mag_cal_active")) return tr("Calibração da bússola ativa", "Compass calibration active");
  if (k.equals("mag_cal_progress")) return tr("Progresso da calibração da bússola", "Compass calibration progress");
  if (k.equals("seq")) return tr("Seq", "Seq");
  if (k.equals("rxseq")) return tr("Seq Rx", "Rx seq");
  if (k.equals("stamp_ms")) return tr("Tempo ms", "Stamp ms");
  if (k.equals("link_ms")) return tr("Link ms", "Link ms");
  if (isDroneSelected) {
    if (k.equals("an4") || k.equals("an_4")) return tr("Sensor de distância esquerdo", "Left distance sensor");
    if (k.equals("an5") || k.equals("an_5")) return tr("Sensor de distância direito", "Right distance sensor");
    if (k.equals("drone_cam_pan_deg")) return tr("Pan do gimbal", "Gimbal pan");
    if (k.equals("drone_cam_tilt_deg")) return tr("Tilt do gimbal", "Gimbal tilt");
    if (k.equals("sonar_vertical_cm")) return tr("Sonar inferior vertical corrigido", "Corrected downward sonar vertical");
    if (k.equals("sonar_ground_ref_cm")) return tr("Referência do chão do sonar", "Sonar ground reference");
    if (k.equals("sonar_height_cm") || k.equals("drone_height_from_sonar_cm")) return tr("Altura corrigida sonar/MPU", "Corrected sonar/MPU height");
    if ((k.equals("sonar_cm") || k.equals("range_cm")) && isManipulatorSelected) return tr("Distância do sonar da mão (< 4 m)", "Hand sonar distance (< 4 m)");
    if (k.equals("sonar_cm") || k.equals("range_cm") || k.equals("drone_scan_cm") || k.equals("drone_sonar_down_cm") || k.equals("height_cm") || k.equals("alt_cm")) return tr("Altura do sonar inferior (< 4 m)", "Downward sonar height (< 4 m)");
    if (k.equals("battery_raw") || k.equals("battery")) return tr("Bateria bruta", "Battery raw");
    if (k.equals("battery_pct") || k.equals("bat_pct")) return tr("Percentual da bateria", "Battery percent");
    if (k.equals("ina1_ma")) return tr("Sensor de corrente 1", "Current sensor 1");
    if (k.equals("ina2_ma")) return tr("Sensor de corrente 2", "Current sensor 2");
  }
  if (isVehicleSelected) {
    if (k.equals("an2") || k.equals("an_2")) return tr("Sensor de torque esquerdo", "Left torque sensor");
    if (k.equals("an3") || k.equals("an_3")) return tr("Sensor de torque direito", "Right torque sensor");
    if (k.equals("lidar_cm") || k.equals("lidar") || k.equals("range_cm") || k.equals("sonar_cm")) return tr("Distância do LiDAR", "LiDAR distance");
    if (k.equals("vehicle_cam_pan_deg")) return tr("Pan da câmera", "Camera pan");
    if (k.equals("vehicle_cam_tilt_deg")) return tr("Tilt da câmera", "Camera tilt");
    if (k.equals("battery_raw") || k.equals("battery")) return tr("Bateria bruta", "Battery raw");
    if (k.equals("battery_pct") || k.equals("bat_pct")) return tr("Percentual da bateria", "Battery percent");
    if (k.equals("ina1_ma")) return tr("Sensor de corrente 1", "Current sensor 1");
    if (k.equals("ina2_ma")) return tr("Sensor de corrente 2", "Current sensor 2");
  }
  if (isManipulatorSelected) {
    if (k.equals("an2") || k.equals("an_2")) return tr("Sensor de torque da base", "Base torque sensor");
    if (k.equals("an3") || k.equals("an_3")) return tr("Sensor de torque do braço superior", "Upper arm torque sensor");
    if (k.equals("an4") || k.equals("an_4")) return tr("Sensor de torque vertical do antebraço", "Forearm vertical torque sensor");
    if (k.equals("an5") || k.equals("an_5")) return tr("Sensor de torque rotacional do antebraço", "Forearm rotational torque sensor");
    if (k.equals("pwm_12")) return tr("Controle de força da base", "Base force control");
    if (k.equals("pwm_13")) return tr("Controle de força do braço superior", "Upper arm force control");
    if (k.equals("pwm_14")) return tr("Controle de força vertical do antebraço", "Forearm vertical force control");
    if (k.equals("pwm_15")) return tr("Controle de força rotacional do antebraço", "Forearm rotation force control");
    if (k.equals("ina1_ma")) return tr("Corrente do grupo do punho 1", "Wrist group current 1");
    if (k.equals("ina2_ma")) return tr("Corrente do grupo do punho 2", "Wrist group current 2");
  }
  String pretty = key.replace('_', ' ');
  if (pretty.length() == 0) return tr("sensor", "sensor");
  return Character.toUpperCase(pretty.charAt(0)) + pretty.substring(1);
}

// Utility: diagnostics hidden sensor key.
boolean diagnosticsIsHiddenSensorKey(JSONObject sens, String key) {
  if (key == null) return false;
  String k = trim(key).toLowerCase();
  if (k.equals("robot_index")) return true;
  if (k.equals("base_deg") && sens != null && sens.hasKey("base_member_deg")) return true;
  if (k.equals("upper_deg") && sens != null && sens.hasKey("upper_member_deg")) return true;
  if (k.equals("fore_deg") && sens != null && sens.hasKey("fore_member_deg")) return true;
  if (k.equals("forearm_roll_deg") && sens != null && sens.hasKey("forearm_roll_member_deg")) return true;
  if (k.equals("wrist_pitch_deg") && sens != null && sens.hasKey("wrist_pitch_member_deg")) return true;
  if (k.equals("wrist_rot_deg") && sens != null && sens.hasKey("wrist_roll_member_deg")) return true;
  if (k.equals("grip_deg") && sens != null && sens.hasKey("grip_member_deg")) return true;
  if (k.startsWith("an_") && sens != null && sens.hasKey(k.replace("an_", "an"))) return true;
  if (k.equals("battery") && sens != null && sens.hasKey("battery_raw")) return true;
  if (k.equals("bat_pct") && sens != null && sens.hasKey("battery_pct")) return true;
  if (diagnosticsIsUnusedAnalogKey(k)) return true;
  if (isManipulatorSelected && (k.equals("battery") || k.equals("battery_raw") || k.equals("bat_pct") || k.equals("battery_pct"))) return true;
  if (isVehicleSelected && (k.equals("sonar_cm") || k.equals("range_cm")) && sens != null && sens.hasKey("lidar_cm")) return true;
  return false;
}

// Utility: diagnostics sensor key is numeric.
boolean diagnosticsSensorKeyIsNumeric(JSONObject sens, String key) {
  if (sens == null || key == null || !sens.hasKey(key)) return false;
  try {
    sens.getFloat(key);
    return true;
  }
  catch (Exception e1) {
    try {
      sens.getInt(key);
      return true;
    }
    catch (Exception e2) {
      return false;
    }
  }
}

// Utility: diagnostics sensor numeric value.
float diagnosticsSensorNumericValue(JSONObject sens, String key) {
  if (sens == null || key == null) return 0;
  try {
    return sens.getFloat(key);
  }
  catch (Exception e1) {
    try {
      return sens.getInt(key);
    }
    catch (Exception e2) {
      return 0;
    }
  }
}

// Utility: diagnostics packet header key.
boolean diagnosticsIsPacketHeaderKey(String key) {
  String k = key == null ? "" : trim(key).toLowerCase();
  return k.equals("seq") || k.equals("rxseq") || k.equals("stamp_ms") || k.equals("link_ms");
}

// Utility: diagnostics position key.
boolean diagnosticsIsPositionSensorKey(String key) {
  String k = key == null ? "" : trim(key).toLowerCase();
  if (isVehicleSelected) return k.equals("vehicle_x") || k.equals("vehicle_z") || k.equals("vehicle_yaw_deg");
  if (isDroneSelected) return k.equals("alt_cm") || k.equals("drone_x") || k.equals("drone_y") || k.equals("drone_z") || k.equals("drone_yaw_deg") || k.equals("drone_pitch_deg") || k.equals("drone_roll_deg") || k.equals("att_yaw_deg");
  return k.equals("base_member_deg") || k.equals("upper_member_deg") || k.equals("fore_member_deg") || k.equals("forearm_roll_member_deg") || k.equals("wrist_pitch_member_deg") || k.equals("wrist_roll_member_deg") || k.equals("grip_member_deg") || k.equals("step_deg") || k.equals("target_deg");
}

// Utility: diagnostics key priority.
int diagnosticsSensorKeyPriority(String key) {
  String k = key == null ? "" : trim(key).toLowerCase();
  if (diagnosticsIsPacketHeaderKey(k)) {
    if (k.equals("seq")) return 0;
    if (k.equals("rxseq")) return 1;
    if (k.equals("stamp_ms")) return 2;
    return 3;
  }
  if (diagnosticsIsPositionSensorKey(k)) return 200;
  return 100;
}

// Utility: diagnostics sorted keys.
ArrayList<String> diagnosticsSortedKeys(JSONObject sens, boolean numericOnly) {
  ArrayList<String> keys = new ArrayList<String>();
  if (sens == null) return keys;
  for (Object keyObj : sens.keys()) {
    String key = String.valueOf(keyObj);
    if (key == null) continue;
    if (diagnosticsIsStaticSensorKey(key)) continue;
    if (diagnosticsIsHiddenSensorKey(sens, key)) continue;
    if (!numericOnly || diagnosticsSensorKeyIsNumeric(sens, key)) keys.add(key);
  }
  java.util.Collections.sort(keys, new java.util.Comparator<String>() {
    public int compare(String a, String b) {
      int pa = diagnosticsSensorKeyPriority(a);
      int pb = diagnosticsSensorKeyPriority(b);
      if (pa != pb) return pa - pb;
      String aa = a == null ? "" : a.toLowerCase();
      String bb = b == null ? "" : b.toLowerCase();
      return aa.compareTo(bb);
    }
  });
  return keys;
}

// Utility: diagnostics csv escape.
String diagnosticsCsvEscape(String value) {
  String v = value == null ? "" : value;
  v = v.replace("\"", "\"\"");
  return "\"" + v + "\"";
}

// Utility: diagnostics timestamp string.
String diagnosticsTimestampString() {
  return nf(year(), 4) + "-" + nf(month(), 2) + "-" + nf(day(), 2) +
    "_" + nf(hour(), 2) + "-" + nf(minute(), 2) + "-" + nf(second(), 2);
}

// Utility: diagnostics can start sensor recording.
boolean diagnosticsCanStartSensorRecording() {
  return isHardwareRobotLockActive();
}

// Utility: diagnostics sensor record button enabled.
boolean diagnosticsSensorRecordButtonEnabled() {
  return diagnosticsSensorRecording || diagnosticsCanStartSensorRecording();
}

// Checks whether diagnostics has active sensor telemetry.
boolean diagnosticsHasActiveSensorTelemetry() {
  return latestSensors != null && latestSensors.size() > 0;
}

// Utility: diagnostics add header if missing.
void diagnosticsAddHeaderIfMissing(ArrayList<String> headers, String key) {
  if (headers == null || key == null || key.length() == 0) return;
  if (!headers.contains(key)) headers.add(key);
}

// Utility: diagnostics append runtime record headers.
void diagnosticsAppendRuntimeRecordHeaders(ArrayList<String> headers) {
  diagnosticsAddHeaderIfMissing(headers, "selected_robot");
  diagnosticsAddHeaderIfMissing(headers, "detected_robot");
  for (int i = 0; i < DIAGNOSTICS_ARM_LIVE_RECORD_KEYS.length; i++) diagnosticsAddHeaderIfMissing(headers, DIAGNOSTICS_ARM_LIVE_RECORD_KEYS[i]);
  for (int i = 0; i < DIAGNOSTICS_ARM_TARGET_RECORD_KEYS.length; i++) diagnosticsAddHeaderIfMissing(headers, DIAGNOSTICS_ARM_TARGET_RECORD_KEYS[i]);
}

// Utility: diagnostics manipulator runtime value.
String diagnosticsManipulatorRuntimeValue(int jointIndex, boolean live) {
  if (jointIndex < 0 || jointIndex >= angles.length) return "";
  if (live) return nf(getManipulatorVisualServoAngle(jointIndex), 0, 2);
  return str(angles[jointIndex]);
}

// Utility: diagnostics record runtime value.
String diagnosticsRecordRuntimeValue(String key) {
  if (key == null) return "";
  if (key.equals("selected_robot")) return currentModeName();
  if (key.equals("detected_robot")) return normalizeRobotTypeName(detectedRobotType);

  for (int i = 0; i < DIAGNOSTICS_ARM_LIVE_RECORD_KEYS.length && i < angles.length; i++) {
    if (key.equals(DIAGNOSTICS_ARM_LIVE_RECORD_KEYS[i])) return diagnosticsManipulatorRuntimeValue(i, true);
  }
  for (int i = 0; i < DIAGNOSTICS_ARM_TARGET_RECORD_KEYS.length && i < angles.length; i++) {
    if (key.equals(DIAGNOSTICS_ARM_TARGET_RECORD_KEYS[i])) return diagnosticsManipulatorRuntimeValue(i, false);
  }
  return null;
}

// Utility: diagnostics initial record snapshot.
JSONObject diagnosticsInitialRecordSnapshot() {
  if (latestSensors != null && latestSensors.size() > 0) return latestSensors;
  JSONObject lockedSnapshot = getRobotSensorSnapshot(lockedHardwareRobotName());
  if (lockedSnapshot != null) return lockedSnapshot;
  return new JSONObject();
}

// Starts diagnostics sensor recording.
void startDiagnosticsSensorRecording() {
  if (diagnosticsSensorRecording) return;
  if (!diagnosticsCanStartSensorRecording()) {
    updateMessage(tr("Conecte um robô antes de usar Gravar sensores.", "Connect a robot before using Record Sensors."));
    return;
  }

  File dir = new File(sketchPath("data/sensor_logs"));
  if (!dir.exists()) dir.mkdirs();

  diagnosticsSensorRecordFilename = "data/sensor_logs/sensors_" + diagnosticsTimestampString() + ".csv";
  diagnosticsSensorRecordWriter = createWriter(sketchPath(diagnosticsSensorRecordFilename));
  diagnosticsSensorRecordHeaders = diagnosticsSortedKeys(diagnosticsInitialRecordSnapshot(), false);
  diagnosticsAppendRuntimeRecordHeaders(diagnosticsSensorRecordHeaders);

  StringBuilder header = new StringBuilder();
  header.append("sample_ms,wall_clock");
  for (int i = 0; i < diagnosticsSensorRecordHeaders.size(); i++) {
    header.append(",").append(diagnosticsCsvEscape(diagnosticsSensorRecordHeaders.get(i)));
  }
  diagnosticsSensorRecordWriter.println(header.toString());
  diagnosticsSensorRecordWriter.flush();
  diagnosticsSensorRecording = true;
  updateMessage(tr("Gravação de sensores iniciada: ", "Sensor recording started: ") + diagnosticsSensorRecordFilename);
}

// Stops diagnostics sensor recording.
void stopDiagnosticsSensorRecording() {
  if (!diagnosticsSensorRecording) return;
  diagnosticsSensorRecording = false;
  if (diagnosticsSensorRecordWriter != null) {
    diagnosticsSensorRecordWriter.flush();
    diagnosticsSensorRecordWriter.close();
    diagnosticsSensorRecordWriter = null;
  }
  updateMessage(tr("Gravação de sensores salva em ", "Sensor recording saved to ") + diagnosticsSensorRecordFilename);
}

// Utility: toggle diagnostics sensor recording.
void toggleDiagnosticsSensorRecording() {
  if (diagnosticsSensorRecording) {
    stopDiagnosticsSensorRecording();
  } else if (diagnosticsCanStartSensorRecording()) {
    startDiagnosticsSensorRecording();
  } else {
    updateMessage(tr("Conecte um robô antes de usar Gravar sensores.", "Connect a robot before using Record Sensors."));
  }
}

// Utility: record diagnostics sensor snapshot.
void recordDiagnosticsSensorSnapshot(JSONObject sens) {
  if (!diagnosticsSensorRecording || diagnosticsSensorRecordWriter == null || sens == null) return;

  StringBuilder line = new StringBuilder();
  line.append(millis()).append(",").append(diagnosticsCsvEscape(diagnosticsTimestampString()));
  for (int i = 0; i < diagnosticsSensorRecordHeaders.size(); i++) {
    String key = diagnosticsSensorRecordHeaders.get(i);
    String runtimeValue = diagnosticsRecordRuntimeValue(key);
    line.append(",");
    line.append(diagnosticsCsvEscape(runtimeValue != null ? runtimeValue : getSensorJsonString(sens, key, "")));
  }
  diagnosticsSensorRecordWriter.println(line.toString());
  diagnosticsSensorRecordWriter.flush();
}

// Utility: append diagnostics sensor history snapshot.
void appendDiagnosticsSensorHistorySnapshot(JSONObject sens) {
  if (sens == null || sens.size() == 0) return;

  ArrayList<String> numericKeys = diagnosticsSortedKeys(sens, true);
  diagnosticsSensorNumericKeys = numericKeys;

  for (int i = 0; i < numericKeys.size(); i++) {
    String key = numericKeys.get(i);
    ArrayList<Float> history = diagnosticsSensorHistory.get(key);
    if (history == null) {
      history = new ArrayList<Float>();
      diagnosticsSensorHistory.put(key, history);
    }
    history.add(diagnosticsSensorNumericValue(sens, key));
    while (history.size() > DIAGNOSTICS_SENSOR_HISTORY_LIMIT) {
      history.remove(0);
    }
  }

  recordDiagnosticsSensorSnapshot(sens);
}

// Utility: clear diagnostics sensor state.
void clearDiagnosticsSensorState() {
  diagnosticsSensorHistory.clear();
  diagnosticsSensorNumericKeys.clear();
  diagnosticsGraphsPage = 0;
  diagnosticsActiveSlider = -1;
}

// Utility: append diagnostics sensor history.
void appendDiagnosticsSensorHistory() {
  appendDiagnosticsSensorHistorySnapshot(latestSensors);
}

// Utility: diagnostics graph window x.
int diagnosticsGraphWindowX() {
  return max(30, (width - 980) / 2);
}

// Utility: diagnostics graph window y.
int diagnosticsGraphWindowY() {
  return max(24, (height - 700) / 2);
}

// Utility: diagnostics graph window w.
int diagnosticsGraphWindowW() {
  return min(980, width - 60);
}

// Utility: diagnostics graph window h.
int diagnosticsGraphWindowH() {
  return min(700, height - 48);
}

// Utility: point in diagnostics graphs window.
boolean pointInDiagnosticsGraphsWindow(int mx, int my) {
  if (!diagnosticsGraphsWindowOpen) return false;
  return pointInRect(mx, my, diagnosticsGraphWindowX(), diagnosticsGraphWindowY(), diagnosticsGraphWindowW(), diagnosticsGraphWindowH());
}

// Utility: diagnostics graphs page count.
int diagnosticsGraphsPageCount() {
  int total = diagnosticsSensorNumericKeys.size();
  if (total <= 0) return 1;
  return max(1, (int)ceil(total / float(DIAGNOSTICS_GRAPHS_PER_PAGE)));
}

// Utility: draw diagnostics sensor chart.
void drawDiagnosticsSensorChart(float x, float y, float w, float h, String key) {
  fill(uiPanelBackgroundColor());
  stroke(uiBorderColor());
  rect(x, y, w, h, DIAGNOSTICS_SECTION_RADIUS);

  ArrayList<Float> history = diagnosticsSensorHistory.get(key);
  fill(uiPrimaryTextColor());
  textAlign(LEFT, TOP);
  text(formatSensorKeyLabel(key), x + 10, y + 8);

  if (history == null || history.size() < 2) {
    fill(uiSecondaryTextColor());
    text(tr("Aguardando dados", "Waiting for data"), x + 10, y + 32);
    return;
  }

  float minV = Float.MAX_VALUE;
  float maxV = -Float.MAX_VALUE;
  for (int i = 0; i < history.size(); i++) {
    float v = history.get(i);
    minV = min(minV, v);
    maxV = max(maxV, v);
  }
  if (abs(maxV - minV) < 0.0001f) {
    minV -= 1;
    maxV += 1;
  }

  float left = x + 12;
  float right = x + w - 12;
  float top = y + 36;
  float bottom = y + h - 22;

  stroke(isDarkThemeEnabled() ? color(102, 106, 112) : color(228));
  for (int i = 0; i < 4; i++) {
    float gy = lerp(top, bottom, i / 3.0f);
    line(left, gy, right, gy);
  }

  stroke(50, 120, 220);
  strokeWeight(1.6f);
  noFill();
  beginShape();
  for (int i = 0; i < history.size(); i++) {
    float xx = map(i, 0, max(1, history.size() - 1), left, right);
    float yy = map(history.get(i), minV, maxV, bottom, top);
    vertex(xx, yy);
  }
  endShape();
  strokeWeight(1);

  fill(uiPrimaryTextColor());
  float latest = history.get(history.size() - 1);
  textAlign(LEFT, BOTTOM);
  text("Latest: " + nf(latest, 1, 2), left, y + h - 6);
  textAlign(RIGHT, BOTTOM);
  text(nf(minV, 1, 2) + " .. " + nf(maxV, 1, 2), right, y + h - 6);
}

// Utility: draw diagnostics graphs window.
void drawDiagnosticsGraphsWindow() {
  if (!diagnosticsGraphsWindowOpen) return;

  int winX = diagnosticsGraphWindowX();
  int winY = diagnosticsGraphWindowY();
  int winW = diagnosticsGraphWindowW();
  int winH = diagnosticsGraphWindowH();

  hint(DISABLE_DEPTH_TEST);
  pushMatrix();
  resetMatrix();
  camera();
  pushStyle();
  noLights();

  fill(0, 120);
  noStroke();
  rect(0, 0, width, height);

  fill(uiPanelBackgroundColor());
  stroke(uiBorderColor());
  rect(winX, winY, winW, winH, 12);

  fill(uiPrimaryTextColor());
  textFont(diagnosticsFont);
  textSize(14);
  textAlign(LEFT, TOP);
  text(tr("Gráficos numéricos dos sensores", "Numeric sensor charts"), winX + 14, winY + 12);
  textSize(11);
  fill(uiSecondaryTextColor());
  text(tr("Todos os sinais numéricos dos sensores aparecem aqui, divididos em páginas.", "All numeric sensor streams are shown here, split across pages."), winX + 14, winY + 34);

  int topBtnY = winY + 10;
  int closeBtnW = 74;
  int recBtnW = 104;
  drawMiniButton2D(winX + winW - closeBtnW - 12, topBtnY, closeBtnW, 24, tr("Fechar", "Close"), false);
  drawMiniButton2D(winX + winW - closeBtnW - recBtnW - 20, topBtnY, recBtnW, 24, diagnosticsSensorRecording ? tr("Parar REC", "Stop REC") : tr("Gravar", "Record"), diagnosticsSensorRecording, diagnosticsSensorRecordButtonEnabled());

  int pageCount = diagnosticsGraphsPageCount();
  diagnosticsGraphsPage = constrain(diagnosticsGraphsPage, 0, pageCount - 1);

  int gridX = winX + 14;
  int gridY = winY + 62;
  int gridW = winW - 28;
  int gridH = winH - 110;
  int gap = 12;
  int cardW = (gridW - gap) / 2;
  int cardH = (gridH - gap) / 2;

  int startIdx = diagnosticsGraphsPage * DIAGNOSTICS_GRAPHS_PER_PAGE;
  for (int i = 0; i < DIAGNOSTICS_GRAPHS_PER_PAGE; i++) {
    int sensorIdx = startIdx + i;
    if (sensorIdx >= diagnosticsSensorNumericKeys.size()) break;
    int col = i % 2;
    int row = i / 2;
    float cx = gridX + col * (cardW + gap);
    float cy = gridY + row * (cardH + gap);
    drawDiagnosticsSensorChart(cx, cy, cardW, cardH, diagnosticsSensorNumericKeys.get(sensorIdx));
  }

  if (diagnosticsSensorNumericKeys.isEmpty()) {
    fill(uiSecondaryTextColor());
    textAlign(CENTER, CENTER);
    text(tr("Nenhum sensor numérico recebido ainda.", "No numeric sensors received yet."), winX + winW * 0.5f, winY + winH * 0.5f);
  }

  int navY = winY + winH - 34;
  drawMiniButton2D(winX + 14, navY, 72, 22, tr("< Ant", "< Prev"), diagnosticsGraphsPage > 0);
  drawMiniButton2D(winX + 94, navY, 72, 22, tr("Próx >", "Next >"), diagnosticsGraphsPage < pageCount - 1);
  fill(uiSecondaryTextColor());
  textAlign(LEFT, CENTER);
  text(tr("Página ", "Page ") + (diagnosticsGraphsPage + 1) + " / " + pageCount + "  |  " + tr("Sensores: ", "Sensors: ") + diagnosticsSensorNumericKeys.size(), winX + 182, navY + 11);

  popStyle();
  safePopMatrix("DiagnosticsPanel.pde:689");
  hint(ENABLE_DEPTH_TEST);
}

// Utility: toggle diagnostics graphs window.
void toggleDiagnosticsGraphsWindow() {
  diagnosticsGraphsWindowOpen = !diagnosticsGraphsWindowOpen;
}

// Utility: toggle environment scan.
void toggleEnvironmentScan() {
  setCurrentEnvironmentAutoScan(!currentEnvironmentAutoScanEnabled());
  saveSoftwareConfigNow();
  updateMessage(currentModeName() + " scan: " + (currentEnvironmentAutoScanEnabled() ? "ON" : "OFF"));
}

// Toggles only the drone downward sonar mapping path. It keeps the working
// map/snapshot plumbing but exposes it as the real bottom sonar instead of a
// generic environment scan.
void toggleDroneDownwardSonar() {
  setCurrentEnvironmentAutoScan(!currentEnvironmentAutoScanEnabled());
  saveSoftwareConfigNow();
  updateMessage(tr("Sonar inferior do drone: ", "Drone downward sonar: ") + (currentEnvironmentAutoScanEnabled() ? "ON" : "OFF"));
}

// Utility: toggle gimbal stab.
void toggleGimbalStab() {
  boolean newState = !gimbalStabEnabled;
  boolean sent = true;
  if (systemReady && !simulationMode && myPort != null) {
    sent = sendHardwareStreamCommand("STAB=" + (newState ? "1" : "0"), "gimbal stabilizer " + (newState ? "ON" : "OFF"), true);
  }
  if (!sent) {
    updateMessage(tr("Comando de estabilização do gimbal não enviado.", "Gimbal stabilization command not sent"));
    return;
  }
  gimbalStabEnabled = newState;
  saveSoftwareConfigNow();
  updateMessage(tr("Estabilização do gimbal: ", "Gimbal stabilization: ") + (gimbalStabEnabled ? "ON" : "OFF"));
}

// Utility: toggle arm stabilizer state in the firmware.
// The visual state is still synchronized from ASTAB telemetry whenever the
// firmware reports it, but the button can request the opposite state.
void toggleArmStab() {
  boolean firmwareState = getSensorFloat("astab", armStabEnabled ? 1 : 0) > 0.5f;
  boolean newState = !firmwareState;
  boolean sent = true;
  if (systemReady && !simulationMode && myPort != null) {
    sent = sendHardwareStreamCommand("ASTAB=" + (newState ? "1" : "0"), "arm stabilizer " + (newState ? "ON" : "OFF"), true);
  }
  if (!sent) {
    updateMessage(tr("Comando do estabilizador do braço não enviado.", "Arm stabilizer command not sent."));
    return;
  }
  armStabEnabled = newState;
  saveSoftwareConfigNow();
  updateMessage(tr("Estabilizador do braço: ", "Arm stabilizer: ") + (armStabEnabled ? "ON" : "OFF"));
}

// Clears trace map.
void clearTraceMap() {
  trajectoryPoints.clear();
  clearEnvironmentMap();
  clearCurrentDemoWorldAndDisable();
  updateMessage(tr("Mapa escaneado e mundo de demonstração limpos.", "Scanned map and demo world cleared."));
}

// Utility: remove imported world only.
void removeImportedWorldOnly() {
  if (currentImportedWorldPointList().isEmpty()) {
    updateMessage(tr("Nenhum mundo importado para remover.", "No imported world to remove."));
    return;
  }
  clearCurrentImportedWorld();
  updateMessage(tr("Mundo importado removido. O mapa escaneado foi preservado.", "Imported world removed. Scanned map preserved."));
}

// Utility: toggle remote collision from panel.
void toggleRemoteCollisionFromPanel() {
  environmentCollisionEnabled = !environmentCollisionEnabled;
  remoteCollisionEnabled = environmentCollisionEnabled;
  saveSoftwareConfigNow();
  updateMessage(tr("Colisão com ambiente: ", "Environment collision: ") + (environmentCollisionEnabled ? "ON" : "OFF"));
}

// Utility: toggle vehicle lights.
void toggleVehicleLights() {
  vehicleLightsEnabled = !vehicleLightsEnabled;
  setCommandContext(CONTROL_SOURCE_LOCAL);
  try {
    sendVehicleRuntimeCommand(true);
  }
  finally {
    restoreLocalCommandContext();
  }
  saveSoftwareConfigNow();
  updateMessage(tr("Luzes do veículo: ", "Vehicle lights: ") + (vehicleLightsEnabled ? "ON" : "OFF"));
}

// Utility: toggle vehicle lidar scan.
void toggleVehicleLidarScan() {
  vehicleLidarScanEnabled = !vehicleLidarScanEnabled;
  setCurrentEnvironmentAutoScan(vehicleLidarScanEnabled);
  setCommandContext(CONTROL_SOURCE_LOCAL);
  try {
    sendVehicleRuntimeCommand(true);
  }
  finally {
    restoreLocalCommandContext();
  }
  saveSoftwareConfigNow();
  updateMessage(tr("Lidar do veículo: ", "Vehicle lidar: ") + (vehicleLidarScanEnabled ? "ON" : "OFF"));
}

// Utility: toggle drone camera.
void toggleDroneCamera() {
  droneCameraStreamingEnabled = !droneCameraStreamingEnabled;
  setCommandContext(CONTROL_SOURCE_LOCAL);
  try {
    sendDroneRuntimeCommand(true);
  }
  finally {
    restoreLocalCommandContext();
  }
  saveSoftwareConfigNow();
  updateMessage(tr("Câmera do drone: ", "Drone camera: ") + (droneCameraStreamingEnabled ? "ON" : "OFF"));
}

// Utility: toggle drone stabilize.
void toggleDroneStabilize() {
  droneStabilizeEnabled = !droneStabilizeEnabled;
  saveSoftwareConfigNow();
  updateMessage(tr("Estabilização do drone: ", "Drone stabilize: ") + (droneStabilizeEnabled ? "ON" : "OFF"));
}

// Compass helper for calibration active.
boolean compassCalibrationActive() {
  return getSensorFloat("mag_cal_active", getSensorFloat("magcal_active", 0)) > 0.5f;
}

// Compass helper for calibration progress pct.
float compassCalibrationProgressPct() {
  return constrain(getSensorFloat("mag_cal_progress", getSensorFloat("magcal_progress", 0)), 0, 100);
}

// Utility: diagnostics current compass heading degrees.
float diagnosticsCurrentCompassHeadingDeg() {
  if (isVehicleSelected) return vehicleHeadingTelemetryDeg;
  if (isDroneSelected) return droneHeadingTelemetryDeg;
  return mpu1YawDeg;
}

// Utility: diagnostics compass summary text.
String diagnosticsCompassSummaryText() {
  if (!diagnosticsHasActiveSensorTelemetry()) return tr("Bússola: --", "Compass: --");
  String headingText = tr("Bússola: ", "Compass: ") + nf(normalizeAbsoluteAngleDeg(diagnosticsCurrentCompassHeadingDeg()), 1, 1) + "°";
  if (compassCalibrationActive()) {
    headingText += "  |  " + tr("Cal ", "Cal ") + nf(compassCalibrationProgressPct(), 1, 0) + "%";
  }
  if (worldCompassNorthLockedForRobotMode(currentRobotMode())) {
    headingText += "  |  " + tr("Mundo N ", "World N ") + nf(worldCompassNorthOffsetDegForRobotMode(currentRobotMode()), 1, 1) + "°";
  }
  return headingText;
}

// Utility: trigger compass calibration.
void triggerCompassCalibration() {
  if (compassCalibrationActive()) {
    queueSerialCommand("MAGCAL=0");
    updateMessage(currentModeName() + " compass calibration cancelled");
    return;
  }
  queueSerialCommand("MAGCAL=1");
  updateMessage(currentModeName() + " compass calibration started");
}

// Utility: diagnostics section top y.
int diagnosticsMapCardY(int panelY) {
  return panelY + 76;
}

// Utility: diagnostics section top y.
int diagnosticsManipulatorRuntimeCardY(int panelY) {
  return diagnosticsMapCardY(panelY) + DIAGNOSTICS_MAP_CARD_H + DIAGNOSTICS_SECTION_GAP;
}

// Utility: diagnostics section top y.
int diagnosticsManipulatorNeutralCardY(int panelY) {
  return diagnosticsManipulatorRuntimeCardY(panelY) + DIAGNOSTICS_MANIP_RUNTIME_CARD_H + DIAGNOSTICS_SECTION_GAP;
}

// Utility: diagnostics section top y.
int diagnosticsManipulatorSensorsCardY(int panelY) {
  return diagnosticsManipulatorNeutralCardY(panelY) + DIAGNOSTICS_MANIP_NEUTRAL_CARD_H + DIAGNOSTICS_SECTION_GAP;
}

// Utility: diagnostics section top y.
int diagnosticsManipulatorForceCardY(int panelY) {
  return diagnosticsManipulatorSensorsCardY(panelY) + DIAGNOSTICS_MANIP_SENSORS_CARD_H + DIAGNOSTICS_SECTION_GAP;
}

// Utility: diagnostics section top y.
int diagnosticsVehicleRuntimeCardY(int panelY) {
  return diagnosticsMapCardY(panelY) + DIAGNOSTICS_MAP_CARD_H + DIAGNOSTICS_SECTION_GAP;
}

// Utility: diagnostics section top y.
int diagnosticsVehicleSensorsCardY(int panelY) {
  return diagnosticsVehicleRuntimeCardY(panelY) + DIAGNOSTICS_VEHICLE_RUNTIME_CARD_H + DIAGNOSTICS_SECTION_GAP;
}

// Utility: diagnostics section top y.
int diagnosticsDroneRuntimeCardY(int panelY) {
  return diagnosticsMapCardY(panelY) + DIAGNOSTICS_MAP_CARD_H + DIAGNOSTICS_SECTION_GAP;
}

// Utility: diagnostics section top y.
int diagnosticsDroneSensorsCardY(int panelY) {
  return diagnosticsDroneRuntimeCardY(panelY) + DIAGNOSTICS_DRONE_RUNTIME_CARD_H + DIAGNOSTICS_SECTION_GAP;
}

// Utility: diagnostics map button row y.
int diagnosticsMapButtonRowY(int panelY, int row) {
  return diagnosticsMapCardY(panelY) + 36 + row * DIAGNOSTICS_BUTTON_ROW_STEP;
}

// Utility: diagnostics runtime button row y.
int diagnosticsRuntimeButtonRowY(int panelY, int row) {
  int baseY = isManipulatorSelected ? diagnosticsManipulatorRuntimeCardY(panelY) : (isVehicleSelected ? diagnosticsVehicleRuntimeCardY(panelY) : diagnosticsDroneRuntimeCardY(panelY));
  return baseY + 36 + row * DIAGNOSTICS_BUTTON_ROW_STEP;
}

// Utility: diagnostics manipulator neutral button y.
int diagnosticsManipulatorNeutralButtonY(int panelY, int buttonIndex) {
  return diagnosticsManipulatorNeutralCardY(panelY) + 42 + (buttonIndex / 2) * DIAGNOSTICS_BUTTON_ROW_STEP;
}

// Utility: diagnostics manipulator slider y.
int diagnosticsManipulatorSliderY(int panelY, int sliderIndex) {
  return diagnosticsManipulatorForceCardY(panelY) + 15 + sliderIndex * DIAGNOSTICS_SLIDER_ROW_STEP;
}


// Utility: diagnostics local control takeover.
void diagnosticsTakeLocalControl() {
  updateControlLocks();
  long now = millis();
  boolean takeover = currentControlOwner == CONTROL_SOURCE_WEB &&
    webClientConnected &&
    (now - lastWebControlMillis) < CONTROL_OWNER_TIMEOUT_MS;

  currentControlOwner = CONTROL_SOURCE_LOCAL;
  lastLocalControlMillis = now;
  if (takeover) {
    lastWebControlMillis = 0;
    notifyControlLockNotice("Local diagnostics takeover: web control overridden");
    sendSystemStatus();
  } else {
    clearControlLockNotice();
  }
  setCommandContext(CONTROL_SOURCE_LOCAL);
}

// Utility: diagnostics local control context end.
void diagnosticsEndLocalControl() {
  restoreLocalCommandContext();
}

// Utility: diagnostics slider hit y.
int diagnosticsManipulatorSliderDrawY(int panelY, int sliderIndex) {
  return diagnosticsManipulatorSliderY(panelY + 18, sliderIndex);
}

// Returns the top of the actual interactive slider row.
int diagnosticsManipulatorSliderHitY(int panelY, int sliderIndex) {
  return diagnosticsManipulatorSliderDrawY(panelY, sliderIndex) - 4;
}

// Handles manipulator slider interaction.
boolean handleManipulatorSliderInteraction(int mx, int my) {
  int panelY = diagnosticsPanelY();
  int panelX = diagnosticsPanelX();
  int contentX = panelX + DIAGNOSTICS_PANEL_CONTENT_MARGIN;
  int contentW = diagnosticsPanelW - DIAGNOSTICS_PANEL_CONTENT_MARGIN * 2;
  int sliderX = diagnosticsModuleInnerX(contentX) + DIAGNOSTICS_CARD_TEXT_INSET;
  int sliderW = max(40, diagnosticsModuleInnerW(contentW) - DIAGNOSTICS_CARD_TEXT_INSET * 2);

  for (int i = 0; i < 4; i++) {
    int hitY = diagnosticsManipulatorSliderHitY(panelY, i);
    if (pointInRect(mx, my, sliderX - 6, hitY, sliderW + 12, DIAGNOSTICS_SLIDER_HIT_H)) {
      diagnosticsActiveSlider = i;
      int clampedMx = constrain(mx, sliderX, sliderX + sliderW);
      int newPct = round(map(clampedMx, sliderX, sliderX + sliderW, 0, 100));
      diagnosticsTakeLocalControl();
      try {
        setDuty(12 + i, constrain(newPct, 0, 100));
      }
      finally {
        diagnosticsEndLocalControl();
      }
      return true;
    }
  }
  return false;
}

// Handles diagnostics panel mouse dragged.
boolean handleDiagnosticsPanelMouseDragged(int mx, int my) {
  if (diagnosticsActiveSlider < 0) return false;
  int panelX = diagnosticsPanelX();
  int contentX = panelX + DIAGNOSTICS_PANEL_CONTENT_MARGIN;
  int contentW = diagnosticsPanelW - DIAGNOSTICS_PANEL_CONTENT_MARGIN * 2;
  int sliderX = diagnosticsModuleInnerX(contentX) + DIAGNOSTICS_CARD_TEXT_INSET;
  int sliderW = max(40, diagnosticsModuleInnerW(contentW) - DIAGNOSTICS_CARD_TEXT_INSET * 2);
  int clampedMx = constrain(mx, sliderX, sliderX + sliderW);
  int newPct = round(map(clampedMx, sliderX, sliderX + sliderW, 0, 100));
  diagnosticsTakeLocalControl();
  try {
    setDuty(12 + diagnosticsActiveSlider, constrain(newPct, 0, 100));
  }
  finally {
    diagnosticsEndLocalControl();
  }
  return true;
}

// Clears diagnostics panel interaction.
void clearDiagnosticsPanelInteraction() {
  if (diagnosticsActiveSlider >= 0) {
    saveSoftwareConfigNow();
  }
  diagnosticsActiveSlider = -1;
}

// Handles diagnostics panel mouse pressed.
boolean handleDiagnosticsPanelMousePressed(int mx, int my) {
  if (diagnosticsGraphsWindowOpen && !pointInDiagnosticsGraphsWindow(mx, my)) {
    return true;
  }
  if (pointInDiagnosticsGraphsWindow(mx, my)) {
    int winX = diagnosticsGraphWindowX();
    int winY = diagnosticsGraphWindowY();
    int winW = diagnosticsGraphWindowW();
    int winH = diagnosticsGraphWindowH();
    if (pointInRect(mx, my, winX + winW - 86, winY + 10, 74, 24)) {
      diagnosticsGraphsWindowOpen = false;
      return true;
    }
    if (pointInRect(mx, my, winX + winW - 190, winY + 10, 104, 24)) {
      toggleDiagnosticsSensorRecording();
      return true;
    }
    if (pointInRect(mx, my, winX + 14, winY + winH - 34, 72, 22)) {
      diagnosticsGraphsPage = max(0, diagnosticsGraphsPage - 1);
      return true;
    }
    if (pointInRect(mx, my, winX + 94, winY + winH - 34, 72, 22)) {
      diagnosticsGraphsPage = min(diagnosticsGraphsPageCount() - 1, diagnosticsGraphsPage + 1);
      return true;
    }
    return true;
  }

  if (!pointInDiagnosticsPanel(mx, my)) return false;

  int panelX = diagnosticsPanelX();
  int panelY = diagnosticsPanelY();
  int contentX = panelX + DIAGNOSTICS_PANEL_CONTENT_MARGIN;
  int contentW = diagnosticsPanelW - DIAGNOSTICS_PANEL_CONTENT_MARGIN * 2;
  int gap = DIAGNOSTICS_MODULE_GAP;
  int col2W = (contentW - gap) / 2;
  int toolsInnerX = diagnosticsModuleInnerX(contentX);
  int toolsCol2W = diagnosticsTwoColButtonW(contentW);
  int neutralInnerX = diagnosticsModuleInnerX(contentX);
  int neutralCol2W = diagnosticsTwoColButtonW(contentW);

  if (pointInRect(mx, my, contentX, panelY + 42, col2W, DIAGNOSTICS_BUTTON_H)) { toggleDiagnosticsGraphsWindow(); return true; }
  if (pointInRect(mx, my, contentX + col2W + gap, panelY + 42, col2W, DIAGNOSTICS_BUTTON_H)) { toggleDiagnosticsSensorRecording(); return true; }

  if (isManipulatorSelected) {
    return handleManipulatorDiagnosticsPanelMousePressed(mx, my, panelX, panelY, contentX, contentW, gap, toolsInnerX, toolsCol2W, neutralInnerX, neutralCol2W);
  }

  if (isVehicleSelected) {
    return handleVehicleDiagnosticsPanelMousePressed(mx, my, panelX, panelY, contentX, contentW, gap, toolsInnerX, toolsCol2W);
  }

  if (isDroneSelected) {
    return handleDroneDiagnosticsPanelMousePressed(mx, my, panelX, panelY, contentX, contentW, gap, toolsInnerX, toolsCol2W);
  }

  return true;
}

// Utility: draw diagnostics slider 2 d.
void drawDiagnosticsSlider2D(int x, int y, int w, String label, int channel, int targetValue, int liveValue) {
  int idx = getDutyIndex(channel);
  boolean active = idx >= 0 && diagnosticsActiveSlider == idx;

  fill(uiPrimaryTextColor());
  drawDiagnosticsFittedTextLeft(label, x, y + 1, w - 110);
  textAlign(RIGHT, TOP);
  textSize(11);
  text(tr("Cmd ", "Cmd ") + targetValue + "%  |  " + tr("Real ", "Live ") + liveValue + "%", x + w, y + 1);

  int barY = y + 18;
  stroke(active ? color(80, 150, 255) : color(180));
  fill(active ? color(232, 242, 255) : color(240));
  rect(x, barY, w, DIAGNOSTICS_SLIDER_BAR_H, 5);

  float targetX = map(targetValue, 0, 100, x, x + w);
  noStroke();
  fill(active ? color(70, 140, 255) : color(100, 170, 255));
  rect(x, barY, max(0, targetX - x), DIAGNOSTICS_SLIDER_BAR_H, 5);

  float liveX = map(liveValue, 0, 100, x, x + w);
  stroke(active ? color(30, 130, 90) : color(40, 180, 120));
  line(liveX, barY - 3, liveX, barY + 11);

  stroke(active ? color(40, 110, 230) : color(70));
  fill(active ? color(80, 150, 255) : color(255));
  ellipse(targetX, barY + DIAGNOSTICS_SLIDER_BAR_H / 2, active ? 13 : 11, active ? 13 : 11);
  textSize(12);
}

// Utility: draw diagnostics panel.
void drawDiagnosticsPanel() {
  if (!diagnosticsPanelVisible()) {
    diagnosticsGraphsWindowOpen = false;
    return;
  }

  diagnosticsPanelW = preferredDiagnosticsPanelWidth();
  diagnosticsPanelH = preferredDiagnosticsPanelHeight();
  ensureDiagnosticsHUD(diagnosticsPanelW, diagnosticsPanelH);

  int panelX = diagnosticsPanelX();
  int panelY = diagnosticsPanelY();
  int contentX = panelX + DIAGNOSTICS_PANEL_CONTENT_MARGIN;
  int contentW = diagnosticsPanelW - DIAGNOSTICS_PANEL_CONTENT_MARGIN * 2;
  int gap = DIAGNOSTICS_MODULE_GAP;
  int col2W = (contentW - gap) / 2;
  int toolsInnerX = diagnosticsModuleInnerX(contentX);
  int toolsInnerW = diagnosticsModuleInnerW(contentW);
  int toolsCol2W = diagnosticsTwoColButtonW(contentW);
  int neutralInnerX = diagnosticsModuleInnerX(contentX);
  int neutralCol2W = diagnosticsTwoColButtonW(contentW);
  int sensorsInnerX = diagnosticsModuleInnerX(contentX);
  int sensorsInnerW = diagnosticsModuleInnerW(contentW);
  int slidersInnerX = diagnosticsModuleInnerX(contentX);
  int slidersInnerW = diagnosticsModuleInnerW(contentW);
  int sensorsContentX = sensorsInnerX + DIAGNOSTICS_CARD_TEXT_INSET;
  int sensorsContentW = max(40, sensorsInnerW - DIAGNOSTICS_CARD_TEXT_INSET * 2);
  int slidersContentX = slidersInnerX + DIAGNOSTICS_CARD_TEXT_INSET;
  int slidersContentW = max(40, slidersInnerW - DIAGNOSTICS_CARD_TEXT_INSET * 2);

  hint(DISABLE_DEPTH_TEST);
  pushMatrix();
  resetMatrix();
  camera();

  pushStyle();
  textFont(diagnosticsFont);
  textSize(12);
  noLights();
  synDrawFrame2D(panelX, panelY, diagnosticsPanelW, diagnosticsPanelH, uiBorderColor(), uiCanvasBackgroundColor(), 12, 1.0f);

  fill(uiPrimaryTextColor());
  textAlign(LEFT, TOP);
  text((isManipulatorSelected ? tr("Manipulador", "Manipulator") : (isVehicleSelected ? tr("Veículo", "Vehicle") : "Drone")) + tr(" - diagnóstico", " diagnostics"), panelX + 18, panelY + 14);
  fill(uiSecondaryTextColor());
  textSize(12);

  drawMiniButton2D(contentX, panelY + 42, col2W, DIAGNOSTICS_BUTTON_H, tr("Gráficos", "Charts"), diagnosticsGraphsWindowOpen);
  drawMiniButton2D(contentX + col2W + gap, panelY + 42, col2W, DIAGNOSTICS_BUTTON_H, diagnosticsSensorRecording ? tr("Parar gravação", "Stop recording") : tr("Gravar sensores", "Record sensors"), diagnosticsSensorRecording, diagnosticsSensorRecordButtonEnabled());

  int mapCardY = diagnosticsMapCardY(panelY);
  int manipRuntimeCardY = diagnosticsManipulatorRuntimeCardY(panelY);
  int manipNeutralCardY = diagnosticsManipulatorNeutralCardY(panelY);
  int manipSensorsCardY = diagnosticsManipulatorSensorsCardY(panelY);
  int manipForceCardY = diagnosticsManipulatorForceCardY(panelY);
  int vehicleRuntimeCardY = diagnosticsVehicleRuntimeCardY(panelY);
  int vehicleSensorsCardY = diagnosticsVehicleSensorsCardY(panelY);
  int droneRuntimeCardY = diagnosticsDroneRuntimeCardY(panelY);
  int droneSensorsCardY = diagnosticsDroneSensorsCardY(panelY);

  drawSectionCard2D(contentX, mapCardY, contentW, DIAGNOSTICS_MAP_CARD_H, tr("Mapa e mundo", "Map and world"));
  drawMiniButton2D(toolsInnerX, diagnosticsMapButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H, isManipulatorSelected ? tr("Colisão local", "Local collision") : tr("Colisão no mapa", "Map collision"), isManipulatorSelected ? (collisionSet || pendingManipulatorLocalCollisionEnableAfterHome) : environmentCollisionEnabled);
  drawMiniButton2D(toolsInnerX + toolsCol2W + gap, diagnosticsMapButtonRowY(panelY, 0), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Limpar mapa", "Clear map"), false);
  drawMiniButton2D(toolsInnerX, diagnosticsMapButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Carregar mundo", "Load world"), worldImportBusy || currentWorldSourceLabel().length() > 0);
  drawMiniButton2D(toolsInnerX + toolsCol2W + gap, diagnosticsMapButtonRowY(panelY, 1), toolsCol2W, DIAGNOSTICS_BUTTON_H, tr("Remover mundo", "Remove world"), !currentImportedWorldPointList().isEmpty());

  if (isManipulatorSelected) {
    drawManipulatorDiagnosticsPanelSections(panelY, contentX, contentW, gap, toolsInnerX, toolsCol2W, neutralInnerX, neutralCol2W, sensorsContentX, sensorsContentW, slidersContentX, slidersContentW, manipRuntimeCardY, manipNeutralCardY, manipSensorsCardY, manipForceCardY);
  } else if (isVehicleSelected) {
    drawVehicleDiagnosticsPanelSections(panelY, contentX, contentW, gap, toolsInnerX, toolsCol2W, sensorsContentX, sensorsContentW, vehicleRuntimeCardY, vehicleSensorsCardY);
  } else if (isDroneSelected) {
    drawDroneDiagnosticsPanelSections(panelY, contentX, contentW, gap, toolsInnerX, toolsCol2W, sensorsContentX, sensorsContentW, droneRuntimeCardY, droneSensorsCardY);
  }

  popStyle();
  safePopMatrix("DiagnosticsPanel.pde:1196");
  hint(ENABLE_DEPTH_TEST);

  drawDiagnosticsGraphsWindow();
}
