// =====================================================================
// SynROV Processing - Module contracts
// ---------------------------------------------------------------------
// Purpose:
//   Stable contract used by the core sketch to call robot modules without
//   hard-coding each robot implementation in the main runtime flow.
//
// Important:
//   Processing joins all .pde tabs into a single sketch during compilation.
//   This contract reduces coupling and makes the main flow module-oriented.
//   Full physical removal of a robot tab is possible only after all remaining
//   direct references to functions/variables from that robot are also moved
//   behind this interface or kept as stubs.
// =====================================================================

interface SynRovModule {
  public String id();
  public String displayName();
  public boolean available();
  public boolean supports(String capability);

  public void setupModule();
  public void loadConfigSafe();
  public void updateModule();
  public void updatePhysicsSafe();
  public void drawModule3D(boolean advanceCamera);
  public void resetInputStateSafe();

  public int[] captureFrameSafe();
  public void applyFrameSafe(int[] frame);
  public boolean moveSafelyTowardFrameSafe(int[] targetFrame, int maxStepDeg, int toleranceDeg);

  public JSONObject buildConfigObjectSafe();
}

class NullSynRovModule implements SynRovModule {
  String moduleId;
  String moduleLabel;

  NullSynRovModule(String moduleId, String moduleLabel) {
    this.moduleId = moduleId == null ? "unknown" : moduleId;
    this.moduleLabel = moduleLabel == null ? this.moduleId : moduleLabel;
  }

  public String id() { return moduleId; }
  public String displayName() { return moduleLabel; }
  public boolean available() { return false; }
  public boolean supports(String capability) { return false; }
  public void setupModule() { }
  public void loadConfigSafe() { }
  public void updateModule() { }
  public void updatePhysicsSafe() { }
  public void drawModule3D(boolean advanceCamera) {
    pushMatrix();
    hint(DISABLE_DEPTH_TEST);
    camera();
    fill(uiSecondaryTextColor());
    textAlign(CENTER, CENTER);
    textSize(16);
    text(tr("Módulo não instalado: ", "Module not installed: ") + moduleLabel, width / 2, height / 2);
    hint(ENABLE_DEPTH_TEST);
    safePopMatrix("ModuleContracts.pde:NullSynRovModule.drawModule3D");
  }
  public void resetInputStateSafe() { }
  public int[] captureFrameSafe() { return new int[0]; }
  public void applyFrameSafe(int[] frame) { }
  public boolean moveSafelyTowardFrameSafe(int[] targetFrame, int maxStepDeg, int toleranceDeg) { return true; }
  public JSONObject buildConfigObjectSafe() { return new JSONObject(); }
}

final String MODULE_CAP_DIAGNOSTICS = "diagnostics";
final String MODULE_CAP_ENVIRONMENT = "environment";
final String MODULE_CAP_RUNTIME = "runtime";
final String MODULE_CAP_COLLISION = "collision";
final String MODULE_CAP_TRAJECTORY = "trajectory";
final String MODULE_CAP_CAMERA = "camera";
final String MODULE_CAP_GPS = "gps";
final String MODULE_CAP_GRIPPER = "gripper";
final String MODULE_CAP_GROUND_VEHICLE = "groundVehicle";
final String MODULE_CAP_FLIGHT = "flight";

HashMap<String, SynRovModule> synRovModules = new HashMap<String, SynRovModule>();
boolean synRovModulesRegistered = false;

// Creates a module by name without referencing its concrete class at compile time.
// This lets the registry fall back to NullSynRovModule when a class is absent.
SynRovModule createSynRovModuleReflective(String className, String moduleId, String label) {
  if (className == null || className.length() == 0) {
    return new NullSynRovModule(moduleId, label);
  }

  try {
    Class outerClass = sketchApplet != null ? sketchApplet.getClass() : this.getClass();
    Class c = null;
    try {
      c = Class.forName(outerClass.getName() + "$" + className);
    }
    catch (Exception innerNameError) {
      c = Class.forName(className);
    }
    Object obj = null;

    try {
      java.lang.reflect.Constructor ctor = c.getDeclaredConstructor(outerClass);
      ctor.setAccessible(true);
      obj = ctor.newInstance(sketchApplet != null ? sketchApplet : this);
    }
    catch (Exception innerCtorError) {
      try {
        java.lang.reflect.Constructor ctor = c.getDeclaredConstructor();
        ctor.setAccessible(true);
        obj = ctor.newInstance();
      }
      catch (Exception defaultCtorError) {
        obj = null;
      }
    }

    if (obj instanceof SynRovModule) {
      return (SynRovModule)obj;
    }
  }
  catch (Exception e) {
    println("[SynROV][Module] " + moduleId + " unavailable: " + e.getMessage());
  }

  return new NullSynRovModule(moduleId, label);
}

// Registers all known modules. Missing module classes become NullSynRovModule.
void registerSynRovModules() {
  if (synRovModules == null) synRovModules = new HashMap<String, SynRovModule>();
  synRovModules.clear();

  synRovModules.put("Manipulator", createSynRovModuleReflective("Manipulator3DModule", "Manipulator", robotDisplayName("Manipulator")));
  synRovModules.put("Vehicle", createSynRovModuleReflective("Vehicle3DModule", "Vehicle", robotDisplayName("Vehicle")));
  synRovModules.put("Drone", createSynRovModuleReflective("Drone3DModule", "Drone", robotDisplayName("Drone")));

  synRovModulesRegistered = true;
}

void ensureSynRovModulesRegistered() {
  if (!synRovModulesRegistered || synRovModules == null || synRovModules.size() == 0) {
    registerSynRovModules();
  }
}

SynRovModule moduleForRobot(String robotName) {
  ensureSynRovModulesRegistered();
  String normalized = normalizeRobotTypeName(robotName);
  if (normalized.length() == 0) normalized = "Manipulator";
  SynRovModule module = synRovModules.get(normalized);
  if (module == null) {
    module = new NullSynRovModule(normalized, robotDisplayName(normalized));
    synRovModules.put(normalized, module);
  }
  return module;
}

SynRovModule activeModule() {
  return moduleForRobot(currentModeName());
}

void setupAllAvailableModules() {
  ensureSynRovModulesRegistered();
  String[] robotOrder = { "Manipulator", "Vehicle", "Drone" };
  for (int i = 0; i < robotOrder.length; i++) {
    SynRovModule module = moduleForRobot(robotOrder[i]);
    if (module != null && module.available()) {
      module.setupModule();
    }
  }
}

void loadAllModuleConfigsSafely() {
  ensureSynRovModulesRegistered();
  String[] robotOrder = { "Manipulator", "Vehicle", "Drone" };
  for (int i = 0; i < robotOrder.length; i++) {
    SynRovModule module = moduleForRobot(robotOrder[i]);
    if (module != null && module.available()) {
      module.loadConfigSafe();
    }
  }
  ensureRobotTelemetryDefaults();
}

boolean activeModuleSupports(String capability) {
  SynRovModule module = activeModule();
  return module != null && module.available() && module.supports(capability);
}
