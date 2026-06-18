// =====================================================================
// SynROV Firmware - Shared declarations
// ---------------------------------------------------------------------
// Purpose:
//   Shared includes, constants, enums, structs, globals, helpers and
//   forward declarations used by the firmware implementation tabs.
// =====================================================================

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <NewPing.h>
#include <Servo.h>

// Forward declarations for enum/types used in function signatures.
// This keeps custom types available before prototypes are emitted.
enum MotorDriveMode : uint8_t;
enum MotorType : uint8_t;
enum ArmColliderType : uint8_t;
struct ArmGeometryConfig;
struct ConfigPayload;

// Manipulator mapping helpers used before Arduino's auto-prototype pass can safely declare them.
uint8_t getManipulatorMemberChannel(uint8_t memberIdx);
float getManipulatorMemberActualServoDeg(uint8_t memberIdx);
float getManipulatorMemberTargetServoDeg(uint8_t memberIdx);

// Explicit prototypes kept near the top so the Arduino preprocessor does not
// emit auto-generated prototypes before these types exist.
ArmGeometryConfig makeDefaultArmGeometryConfig();
void sanitizeArmGeometryConfig(ArmGeometryConfig &cfg);
void copyArmGeometryToPayload(ConfigPayload &payload, const ArmGeometryConfig &cfg);
void loadArmGeometryFromPayload(const ConfigPayload &payload, ArmGeometryConfig &cfg);
float armFingerPivotOffset();
float armFingerLateralOffset();
bool armShouldIgnoreCollisionPair(ArmColliderType a, ArmColliderType b);

// ---------------- Hardware ----------------
const uint8_t MOTOR_COIL_PINS[4]  = {42, 43, 44, 45};
const uint8_t DC_MOTOR_DIR_PINS[2]   = {42, 43};
const uint8_t MOTOR_POWER_PWM_PIN = 46;
const uint8_t FALLBACK_SERVO_PINS[11] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
const uint8_t FALLBACK_PWM_PINS[4]    = {2, 3, 11, 12};
const uint8_t MOTOR_FEEDBACK_PIN  = A0;
const uint8_t SONAR_PIN = 7;
const uint16_t SONAR_MAX_DISTANCE_CM = 400;

// --- Direct hardware mapping for generic robots (reuses manipulator pins per active mode) ---
// Vehicle mode
const uint8_t VEHICLE_LEFT_TRACK_PWM_PIN   = 2;
const uint8_t VEHICLE_RIGHT_TRACK_PWM_PIN  = 3;
const uint8_t VEHICLE_LEFT_TRACK_DIR_PIN   = 42;
const uint8_t VEHICLE_RIGHT_TRACK_DIR_PIN  = 43;
const uint8_t VEHICLE_LIGHT_PIN            = 44;
const uint8_t SHARED_GIMBAL_PAN_PIN       = 11;
const uint8_t SHARED_GIMBAL_TILT_PIN      = 12;
const uint8_t VEHICLE_CAMERA_PAN_PIN       = SHARED_GIMBAL_PAN_PIN;
const uint8_t VEHICLE_CAMERA_TILT_PIN      = SHARED_GIMBAL_TILT_PIN;
const uint8_t VEHICLE_LIDAR_SCAN_SERVO_PIN = 24;

// Drone mode
const uint8_t DRONE_ESC_PINS[4]            = {22, 23, 24, 25}; // FL, FR, RR, RL
const uint8_t DRONE_CAMERA_RECORD_PIN      = 44;
const uint8_t DRONE_STATUS_LED_PIN         = 45;
const int DRONE_ESC_MIN_US                 = 1000;
const int DRONE_ESC_MAX_US                 = 2000;

const unsigned long SERIAL_BAUD_RATE   = 115200UL;
const unsigned long PRIMARY_SERIAL_BOOT_SETTLE_MS = 0UL;
const unsigned long I2C_CLOCK_SPEED    = 100000UL;  // Safe default for mixed/noisy I2C buses; 400 kHz can lock weak pull-ups/cables.
const unsigned long I2C_TIMEOUT_US     = 25000UL;
const unsigned long SENSOR_INTERVAL_MS = 33UL;
const unsigned long SENSOR_INTERVAL_COLLISION_MS = 85UL;
const float DRONE_AUTO_TAKEOFF_TARGET_CM = 120.0f;
const float DRONE_DOWNWARD_SONAR_VALID_MAX_CM = 400.0f;
const unsigned long SERVO_UPDATE_MS    = 25UL;
const unsigned long MOTOR_CONTROL_US = 1000UL;
const unsigned long HOLD_REDUCE_DELAY_MS = 600UL;
const unsigned long COMPASS_HMC_INIT_SETTLE_MS = 10UL;
const unsigned long COMPASS_QMC_RESET_SETTLE_MS = 2UL;
const unsigned long COMPASS_QMC_CONFIG_SETTLE_MS = 5UL;

// ---------------- Safety autonomy defaults ----------------
const uint32_t SAFETY_CONFIG_MAGIC = 0x53414645UL; // SAFE
const uint16_t SAFETY_CONFIG_SCHEMA_ID = 3;
const float SAFE_RETURN_BATTERY_THRESHOLD_DEFAULT_PCT = 15.0f;
const float SAFE_RETURN_BATTERY_HYSTERESIS_PCT = 5.0f;
const float SAFE_RETURN_HOME_REACHED_M = 2.0f;
const float VEHICLE_INCLINE_BASE_POWER_DEFAULT_PCT = 50.0f;
const float VEHICLE_INCLINE_MAX_POWER_DEFAULT_PCT = 100.0f;
const float VEHICLE_INCLINE_FULL_SCALE_DEFAULT_DEG = 35.0f;
const float VEHICLE_INCLINE_DEADBAND_DEFAULT_DEG = 3.0f;
const float VEHICLE_INCLINE_ALPHA_DEFAULT = 0.20f;
const unsigned long VEHICLE_INCLINE_SAMPLE_MS = 100UL;

const bool VERBOSE_RUNTIME_ACKS = false;
const uint16_t FIRMWARE_VERSION = 1;
const uint16_t FIRMWARE_RUNTIME_PROTOCOL_ID = 1;
const uint8_t TELEMETRY_GEOMETRY_DIV = 10;
const uint8_t TELEMETRY_LIMITS_DIV = 18;
const uint8_t TELEMETRY_IMU_DIV = 2;
const uint8_t TELEMETRY_POWER_DIV = 3;
const uint8_t TELEMETRY_ANALOG_DIV = 4;
const uint8_t TELEMETRY_COMPASS_DIV = 3;
const uint8_t TELEMETRY_SONAR_DIV = 2;
const uint16_t SERIAL_RX_BYTE_BUDGET = 192;
const uint8_t SERIAL_RX_LINE_BUDGET = 6;
const uint8_t TELEMETRY_TX_MIN_FREE_BYTES = 24;
const uint8_t MOTOR_FEEDBACK_SAMPLES = 3;
const unsigned long MOTOR_FEEDBACK_MIN_INTERVAL_US = 3000UL;
const uint32_t MOTOR_STEP_SERVICE_BUDGET_US = 220UL;

// Minimum position deadband for the manipulator base feedback on A0.
// This is applied after the analog/alpha filter and before motor.currentAngle
// is published to the PID and telemetry, so small ADC/potentiometer noise does
// not look like real base motion. Stored in cfg.payload.reserved1[0] as
// centi-degrees to avoid changing the EEPROM payload layout.
const float MOTOR_BASE_A0_DEADBAND_DEFAULT_DEG = 0.30f;
const float MOTOR_BASE_A0_DEADBAND_MIN_DEG = 0.05f;
const float MOTOR_BASE_A0_DEADBAND_MAX_DEG = 2.50f;

// DC PID settle bands for the bounded base axis.  These are intentionally
// separate from STOPENTER/STOPEXIT, which remain stepper-only.  The bands are
// stored in cfg.payload.reserved1[1..2] as deci-degrees so they can be tuned
// without changing the EEPROM payload layout.
const float MOTOR_DC_PID_ENTER_BAND_DEFAULT_DEG = 1.20f;
const float MOTOR_DC_PID_EXIT_BAND_DEFAULT_DEG  = 2.40f;
const float MOTOR_DC_PID_BAND_MIN_DEG = 0.30f;
const float MOTOR_DC_PID_BAND_MAX_DEG = 12.0f;

const uint8_t MAX_CHANNELS   = 16;
const uint8_t SERIAL_TEXT_BUFFER_SIZE = 192;
const uint16_t RUNTIME_TEXT_BUFFER_SIZE = 192;
const uint16_t RUNTIME_FRAME_BUFFER_SIZE = 96;
const uint8_t FIRST_SERVO_CH = 1;
const uint8_t LAST_SERVO_CH  = 11;
const uint8_t FIRST_PWM_CH   = 12;
const uint8_t LAST_PWM_CH    = 15;

const float SERVO_INPUT_MIN = 0.0f;
const float SERVO_INPUT_MAX = 180.0f;
const float SERVO_EXTENDED_INPUT_MAX = 359.0f;
const uint8_t SERVO_EXTENDED_RANGE_CHANNEL = 1;
const float UPPERARM_ZERO_REFERENCE_DEG = 45.0f;
const float STARTUP_SAFE_UPPER_DEG = 150.0f;
const float STARTUP_SAFE_FORE_DEG = 70.0f;
const float STARTUP_SAFE_FOREARM_ROLL_DEG = 90.0f;
const float STARTUP_SAFE_WRIST_PITCH_DEG = 95.0f;
const float STARTUP_SAFE_WRIST_ROT_DEG = 130.0f;
const float STARTUP_SAFE_GRIP_DEG = 0.0f;
const float MANIP_GRIPPER_MAX_DEG = 100.0f;
const float MOTOR_INPUT_MIN = 0.0f;
const float MOTOR_INPUT_MAX = 359.0f;
// Mechanical end guard for the manipulator base when CH0 uses absolute feedback.
// The base is a bounded axis, not a continuous rotary yaw; a reference that
// crosses 0/359 while the sensor is already at a mechanical end is treated as
// an end-stop request and is clamped to the physical stop.
const float MOTOR_LINEAR_END_GUARD_DEG = 35.0f;

// DC base reference-tracking tuning.  The control law is source-agnostic:
// every input layer only writes a target angle, and the firmware classifies the
// reference by its motion state.  A moving reference is tracked continuously; a
// static reference is approached with bounded lead and anti-stiction shaping so
// CH12 can remain high enough to overcome friction without causing large jumps.
const float MOTOR_SERVO_STATIC_TARGET_LEAD_DEG = 4.8f;
const float MOTOR_SERVO_DIRECT_ERROR_DEG = 14.0f;
const float MOTOR_SERVO_FINE_TARGET_SLEW_DPS = 380.0f;
const float MOTOR_SERVO_TRACK_TARGET_SLEW_DPS = 1500.0f;
const float MOTOR_SERVO_COMMAND_EPS_DEG = 0.08f;
const float MOTOR_SERVO_COMMAND_STREAM_RATE_DPS = 8.0f;
const uint16_t MOTOR_SERVO_COMMAND_STREAM_MS = 420;
const float MOTOR_DC_REFERENCE_FF_GAIN = 0.075f;
const float MOTOR_DC_REFERENCE_FF_LIMIT = 12.0f;
// Runtime DC settle bands are configurable with PIDENTER/PIDEXIT.
// Defaults above are used when EEPROM reserved bytes are empty.

const float MOTOR_DC_BREAKAWAY_WINDOW_DEG = 13.0f;
const float MOTOR_DC_STALL_SPEED_DPS = 1.8f;
const float MOTOR_DC_ARRIVAL_BRAKE_LOOKAHEAD_S = 0.055f;
const uint16_t MOTOR_DC_BREAKAWAY_MS = 32;
const uint16_t MOTOR_DC_BREAKAWAY_COOLDOWN_MS = 100;

const float DEFAULT_SERVO_MOVE_STEP = 1.0f;

const uint16_t SERVO_MIN_PULSE = 110;
const uint16_t SERVO_MAX_PULSE = 510;

// PCA9685 servo outputs are intentionally phase-staggered.  With ON=0 for
// every channel, all servo pulses rise at the same instant; while CH0/base is
// moving this current spike can look like control interference and make the
// PCA-driven servos buzz.  Fallback Arduino Servo output is already time-
// multiplexed, so this compensation is only used by the external PCA path.
const bool SERVO_PCA_PHASE_STAGGER_ENABLED = true;
const uint16_t SERVO_PCA_PHASE_BASE_TICKS = 32;
const uint16_t SERVO_PCA_PHASE_SPACING_TICKS = 320;

// While the base motor is actively moving, ignore IMU stabilization corrections
// for a short window.  This prevents base vibration/electrical noise from being
// converted into target changes on the other PCA servo channels.
const float MANIP_BASE_SERVO_QUIET_ERROR_DEG = 1.2f;
const float MANIP_BASE_SERVO_QUIET_SPEED_DPS = 2.0f;
const unsigned long MANIP_BASE_SERVO_QUIET_HOLD_MS = 140UL;

// Slew limit for DC motor PWM command changes.  It reduces supply dips/noise
// when CH0 starts/stops without changing the PID target logic.
const uint16_t MOTOR_DC_PWM_SLEW_TICKS_PER_CONTROL = 28;

const uint32_t EEPROM_MAGIC   = 0x53524F56UL; // SROV
const uint16_t EEPROM_SCHEMA_ID = 1;
const int EEPROM_ADDR         = 0;
const uint16_t CONFIG_UNLOCK_CODE = 1234;
const char GEOMETRY_UNITS[] PROGMEM = "cm";
const float VEHICLE_GEOMETRY_SCALE = 1.00f;
const float DRONE_GEOMETRY_SCALE = 1.00f;
const bool MOTOR_NATIVE_SENSOR_INVERTED = true;
const bool MOTOR_NATIVE_DIRECTION_INVERTED = true;

// ---------------- Helpers ----------------
template<typename T, typename L, typename H>
T clampValue(T v, L lo, H hi) {
  T low = (T)lo;
  T high = (T)hi;
  if (v < low) return low;
  if (v > high) return high;
  return v;
}

// Encodes/decodes the A0 base feedback deadband in centi-degrees.
// A stored value of 0 means "use the firmware default", preserving older
// EEPROM configurations that had this reserved byte cleared.
static inline float decodeBaseA0DeadbandDeg(uint8_t storedCentiDeg) {
  if (storedCentiDeg == 0) return MOTOR_BASE_A0_DEADBAND_DEFAULT_DEG;
  return clampValue((float)storedCentiDeg * 0.01f,
                    MOTOR_BASE_A0_DEADBAND_MIN_DEG,
                    MOTOR_BASE_A0_DEADBAND_MAX_DEG);
}

static inline uint8_t encodeBaseA0DeadbandDeg(float deadbandDeg) {
  float clipped = clampValue(deadbandDeg,
                             MOTOR_BASE_A0_DEADBAND_MIN_DEG,
                             MOTOR_BASE_A0_DEADBAND_MAX_DEG);
  int encoded = (int)roundf(clipped * 100.0f);
  return (uint8_t)clampValue(encoded, 1, 250);
}

// Encodes/decodes DC PID settle bands in deci-degrees.
// A stored value of 0 means "use the supplied default" so old EEPROM images
// continue to boot with sane DC PID behavior.
static inline float decodeDcPidBandDeg(uint8_t storedDeciDeg, float defaultDeg) {
  if (storedDeciDeg == 0) return defaultDeg;
  return clampValue((float)storedDeciDeg * 0.10f,
                    MOTOR_DC_PID_BAND_MIN_DEG,
                    MOTOR_DC_PID_BAND_MAX_DEG);
}

static inline uint8_t encodeDcPidBandDeg(float bandDeg) {
  float clipped = clampValue(bandDeg,
                             MOTOR_DC_PID_BAND_MIN_DEG,
                             MOTOR_DC_PID_BAND_MAX_DEG);
  int encoded = (int)roundf(clipped * 10.0f);
  return (uint8_t)clampValue(encoded, 1, 120);
}

// Utility: blend float.
float blendFloat(float current, float target, float alpha) {
  alpha = clampValue(alpha, 0.0f, 1.0f);
  return current + (target - current) * alpha;
}

// Checks whether servo channel.
bool isServoChannel(uint8_t ch) {
  return (ch >= FIRST_SERVO_CH && ch <= LAST_SERVO_CH);
}

// Checks whether PWM channel.
bool isPwmChannel(uint8_t ch) {
  return (ch >= FIRST_PWM_CH && ch <= LAST_PWM_CH);
}

// Checks whether angular actuator channel.
bool isAngularActuatorChannel(uint8_t ch) {
  return (ch <= LAST_SERVO_CH);
}

enum CompassType : uint8_t {
  COMPASS_NONE = 0,
  COMPASS_HMC5883 = 1,
  COMPASS_QMC5883 = 2
};

// Forward declarations to avoid Arduino auto-prototype issues with custom enum types
static void finishCompassInitializationSuccess(CompassType type, uint8_t addr);
static void finishCompassInitializationFailure();

// Utility: channel supports neutral.
bool channelSupportsNeutral(uint8_t ch) {
  return isAngularActuatorChannel(ch);
}

// Utility: channel supports calibration.
bool channelSupportsCalibration(uint8_t ch) {
  return isAngularActuatorChannel(ch);
}

// Returns auto PWM cap far.
uint16_t getAutoPwmCapFar() {
  return (uint16_t)max(120, (int)((long)ICR5 * 72L / 100L));
}

// Returns auto PWM cap near.
uint16_t getAutoPwmCapNear() {
  return (uint16_t)max(100, (int)((long)ICR5 * 64L / 100L));
}

// Returns auto PWM cap hold.
uint16_t getAutoPwmCapHold() {
  return (uint16_t)max(80, (int)((long)ICR5 * 52L / 100L));
}

// ---------------- Types ----------------
enum ValueMode : uint8_t {
  APPLY_TARGET = 0,
  APPLY_OFFSET,
  APPLY_SPAN,
  APPLY_RAMP,
  APPLY_OPTIMIZE,
  APPLY_MIN_LIMIT,
  APPLY_MAX_LIMIT
};

enum MotorDriveMode : uint8_t {
  MOTOR_MODE_FULLSTEP_BIPOLAR = 0,
  MOTOR_MODE_HALFSTEP_BIPOLAR,
  MOTOR_MODE_FULLSTEP_UNIPOLAR,
  MOTOR_MODE_HALFSTEP_UNIPOLAR
};

enum MotorType : uint8_t {
  MOTOR_TYPE_STEPPER = 0,
  MOTOR_TYPE_DC_MOTOR = 1
};

enum ControlPortOwner : uint8_t {
  CONTROL_OWNER_NONE = 0,
  CONTROL_OWNER_PRIMARY_ASCII,
  CONTROL_OWNER_RUNTIME_HEX
};

enum RobotControlMode : uint8_t {
  ROBOT_MODE_MANIPULATOR = 0,
  ROBOT_MODE_VEHICLE,
  ROBOT_MODE_DRONE
};

// Default motor electrical mode. This can also be changed inside CFG=1234
// with STEPPERMODE=<0..3> or a mode name, then persisted with SAVE.
// Pin order in MOTOR_COIL_PINS:
//   [0] = A+
//   [1] = A-
//   [2] = B+
//   [3] = B-
//
// Bipolar modes assume these four pins drive an H-bridge or equivalent polarity driver.
// Unipolar modes assume these four pins drive the four coil ends of a center-tapped motor.
MotorDriveMode motorDriveMode = MOTOR_MODE_HALFSTEP_UNIPOLAR;
MotorType motorType = MOTOR_TYPE_DC_MOTOR;
ControlPortOwner controlOwner = CONTROL_OWNER_NONE;
RobotControlMode robotControlMode = ROBOT_MODE_MANIPULATOR;
uint32_t telemetryFrameCounter = 0;
uint16_t telemetryBusySkips = 0;
const uint8_t STEPPERMODE_STORE_MAGIC = 0xA0;

int vehicleLeftTrackPct = 0;
int vehicleRightTrackPct = 0;
int vehicleCameraPanDeg = 0;
int vehicleCameraTiltDeg = -10;
bool vehicleLightsCommand = false;
bool vehicleLidarScanCommand = true;
float vehicleTelemetryX = 0.0f;
float vehicleTelemetryZ = 0.0f;
float vehicleTelemetryYawDeg = 0.0f;
float vehicleTelemetryLeftTrackActual = 0.0f;
float vehicleTelemetryRightTrackActual = 0.0f;

int droneThrottlePct = 0;
int droneYawPct = 0;
int dronePitchPct = 0;
int droneRollPct = 0;
int droneStrafePct = 0;
int droneForwardPct = 0;
bool droneCameraRecordCommand = false;
int droneCameraPanDeg = 0;
int droneCameraTiltDeg = -10;
bool droneAutoTakeoffActive = false;
bool droneAutoLandActive = false;
float droneAutoTargetAltitudeCm = 120.0f;
int droneAutoLastSonarCm = -1;
unsigned long droneAutoLastSonarReadMs = 0;
int droneLastRawSonarCm = -1;
float droneSonarVerticalCm = -1.0f;
float droneSonarGroundReferenceCm = -1.0f;
bool droneSonarGroundReferenceValid = false;
float droneAltitudeFromGroundCm = 0.0f;
float droneTelemetryX = 0.0f;
float droneTelemetryY = 0.0f;
float droneTelemetryZ = 0.0f;
float droneTelemetryYawDeg = 0.0f;
float droneTelemetryPitchDeg = 0.0f;
float droneTelemetryRollDeg = 0.0f;
float droneTelemetryAltitudeCm = 0.0f;
float droneTelemetryYawCmdFiltered = 0.0f;
float droneTelemetryPitchCmdFiltered = 0.0f;
float droneTelemetryRollCmdFiltered = 0.0f;
float droneTelemetryThrottleCmdFiltered = 0.0f;
float droneTelemetryStrafeCmdFiltered = 0.0f;
float droneTelemetryForwardCmdFiltered = 0.0f;
float droneTelemetryLift = 0.0f;
float droneTelemetryTargetLift = 0.0f;
unsigned long genericRobotLastUpdateMs = 0;
unsigned long lastMotorFeedbackReadUs = 0;

int vehicleLastAppliedLeftPct = 32767;
int vehicleLastAppliedRightPct = 32767;
int vehicleLastAppliedPanDeg = 32767;
int vehicleLastAppliedTiltDeg = 32767;
int vehicleLastAppliedLidarAngle = 32767;
bool vehicleLastAppliedLights = false;
bool vehicleOutputCacheValid = false;

int droneLastAppliedMicros[4] = { -1, -1, -1, -1};
int droneLastAppliedPanDeg = 32767;
int droneLastAppliedTiltDeg = 32767;
bool droneLastAppliedCameraRecord = false;
bool droneLastAppliedStatusLed = false;
bool droneOutputCacheValid = false;

float droneStabRefPitchDeg = 0.0f;
float droneStabRefRollDeg = 0.0f;
float droneStabRefHeadingDeg = 0.0f;
bool droneStabRefValid = false;

// Safety return-to-home and vehicle incline power assistance.
bool droneSafeReturnEnabled = false;
bool vehicleSafeReturnEnabled = false;
bool vehicleInclinePowerEnabled = false;
float droneSafeReturnBatteryThresholdPct = SAFE_RETURN_BATTERY_THRESHOLD_DEFAULT_PCT;
float vehicleSafeReturnBatteryThresholdPct = SAFE_RETURN_BATTERY_THRESHOLD_DEFAULT_PCT;

bool droneSafeReturnActive = false;
bool vehicleSafeReturnActive = false;
bool droneHomeCapturedThisFlight = false;
bool vehicleHomeCapturedThisSession = false;
int32_t droneHomeLatitudeE7 = 0;
int32_t droneHomeLongitudeE7 = 0;
int32_t droneHomeAltitudeCm = 0;
bool droneHomeValid = false;
int32_t vehicleHomeLatitudeE7 = 0;
int32_t vehicleHomeLongitudeE7 = 0;
int32_t vehicleHomeAltitudeCm = 0;
bool vehicleHomeValid = false;

float vehicleInclineBasePowerPct = VEHICLE_INCLINE_BASE_POWER_DEFAULT_PCT;
float vehicleInclineMaxPowerPct = VEHICLE_INCLINE_MAX_POWER_DEFAULT_PCT;
float vehicleInclineFullScaleDeg = VEHICLE_INCLINE_FULL_SCALE_DEFAULT_DEG;
float vehicleInclineDeadbandDeg = VEHICLE_INCLINE_DEADBAND_DEFAULT_DEG;
float vehicleInclinePowerAlpha = VEHICLE_INCLINE_ALPHA_DEFAULT;
float vehicleInclinePitchDeg = 0.0f;
float vehicleInclineRollDeg = 0.0f;
float vehicleInclineMagnitudeDeg = 0.0f;
float vehicleInclineAutoPowerPct = VEHICLE_INCLINE_BASE_POWER_DEFAULT_PCT;
unsigned long vehicleInclineLastSampleMs = 0;

// Processing-aligned robot profiles
struct ManipulatorProcessingProfile {
  uint8_t baseChannel;
  uint8_t upperChannel;
  uint8_t foreChannel;
  uint8_t forearmRollChannel;
  uint8_t wristPitchChannel;
  uint8_t wristRotChannel;
  uint8_t gripperChannel;
};

enum ManipulatorMemberIndex : uint8_t {
  MANIP_MEMBER_BASE = 0,
  MANIP_MEMBER_UPPER,
  MANIP_MEMBER_FORE,
  MANIP_MEMBER_FOREARM_ROLL,
  MANIP_MEMBER_WRIST_PITCH,
  MANIP_MEMBER_WRIST_ROLL,
  MANIP_MEMBER_GRIPPER,
  MANIP_MEMBER_COUNT
};

struct VehicleProcessingProfile {
  float bodyLength;
  float bodyWidth;
  float bodyHeight;
  float trackGauge;
  float trackRun;
  float driveRadius;
  float supportRadius;
  float topRollerRadius;
  float trackWidth;
  float trackLinkLength;
  float trackLinkThickness;
  float trackHeight;
  int   trackLinks;
  float trackSag;
  float bodyCenterYOffset;
  float cameraPanDefaultDeg;
  float cameraTiltDefaultDeg;
  float lidarMastYOffset;
  float lidarMastZOffset;
  float cameraHeadYOffset;
  float cameraHeadZOffset;
};

struct DroneProcessingProfile {
  float bodyLength;
  float bodyWidth;
  float bodyHeight;
  float armLength;
  float armThickness;
  float motorRadius;
  float motorHeight;
  float propRadius;
  float propThickness;
  float legHeight;
  float legSpan;
  float restYOffset;
  float visualYawOffsetDeg;
  float cameraYOffset;
  float cameraZOffset;
  float lampYOffset;
  float lampZOffset;
  float sonarXOffset;
  float sonarYOffset;
  float sonarZOffset;
};

struct VehicleControlProfile {
  float motorDeadbandPct;
  uint8_t motorPwmMin;
  uint8_t motorPwmMax;
  float motorSlewPctPerUpdate;
  bool leftMotorInvert;
  bool rightMotorInvert;
  float cameraPanMinDeg;
  float cameraPanMaxDeg;
  float cameraTiltMinDeg;
  float cameraTiltMaxDeg;
  uint8_t lidarSweepMinDeg;
  uint8_t lidarSweepMaxDeg;
  uint16_t lidarSweepCycleMs;
};

struct DroneControlProfile {
  uint16_t escMinUs;
  uint16_t escMaxUs;
  uint16_t escIdleUs;
  uint16_t throttleRangeUs;
  uint16_t pitchMixUs;
  uint16_t rollMixUs;
  uint16_t yawMixUs;
  float commandDeadbandPct;
  uint16_t spoolStepUsPerUpdate;
};

// Processing is the single source of truth for geometry and control references.
static const ManipulatorProcessingProfile kDefaultManipProfile = {0, 2, 3, 4, 5, 6, 7};
static const int16_t kDefaultManipHomePose[MANIP_MEMBER_COUNT] PROGMEM = {180, 150, 70, 90, 95, 130, 0};
static const VehicleProcessingProfile kDefaultVehicleProfile = {95.70f, 59.16f, 18.56f, 71.92f, 69.60f, 13.92f, 6.96f, 4.06f, 17.40f, 5.80f, 2.03f, 5.80f, 64, 2.32f, -24.13f, 0.0f, -10.0f, -17.63f, -7.66f, -12.06f, 19.14f};
static const DroneProcessingProfile kDefaultDroneProfile = {36.48f, 36.48f, 10.64f, 34.96f, 3.04f, 4.56f, 6.08f, 13.68f, 1.82f, 12.16f, 15.20f, -17.48f, 0.0f, -7.66f, 12.40f, -1.06f, 17.15f, 0.0f, 6.90f, 0.0f};
static const VehicleControlProfile kDefaultVehicleControlProfile = {5.0f, 36, 255, 12.0f, false, false, -90.0f, 90.0f, -45.0f, 35.0f, 30, 150, 1800};
static const DroneControlProfile kDefaultDroneControlProfile = {DRONE_ESC_MIN_US, DRONE_ESC_MAX_US, 1080, 900, 200, 200, 135, 1.5f, 34};

ManipulatorProcessingProfile manipProfile = kDefaultManipProfile;
VehicleProcessingProfile vehicleProfile = kDefaultVehicleProfile;
DroneProcessingProfile droneProfile = kDefaultDroneProfile;
VehicleControlProfile vehicleControlProfile = kDefaultVehicleControlProfile;
DroneControlProfile droneControlProfile = kDefaultDroneControlProfile;
int16_t manipHomePose[MANIP_MEMBER_COUNT] = {180, 150, 70, 90, 95, 130, 0};

// Firmware HOME convergence control.
// HOME uses the same strategy validated in Processing: exponential steps for
// large moves, but a guaranteed minimum one-degree step near the target so
// rounding/quantization cannot leave the arm parked outside the HOME tolerance.
static const float MANIPULATOR_FIRMWARE_HOME_SMOOTH_FACTOR = 0.05f;
static const float MANIPULATOR_FIRMWARE_HOME_MIN_STEP_DEG = 1.0f;
static const float MANIPULATOR_FIRMWARE_HOME_COMPLETE_TOLERANCE_DEG = 4.0f;
static const unsigned long MANIPULATOR_FIRMWARE_HOME_STEP_MS = SERVO_UPDATE_MS;
bool manipulatorFirmwareHomeActive = false;
unsigned long manipulatorFirmwareHomeLastStepMs = 0;
bool processingProfilesInitialized = false;
bool processingProfilesDirty = true;

// ---------------------------------------------------------------------
// Timer reservation for the stock Arduino Mega Servo library
// ---------------------------------------------------------------------
// On the Mega2560 the stock Servo library allocates 16-bit timers in this
// order: Timer5 -> Timer1 -> Timer3 -> Timer4.
//
// Pin 46 (OC5A) is driven by Timer5 and is the dedicated high-frequency PWM
// output for the motor power stage. If any Servo object lands on Timer5,
// the library reconfigures Timer5 for 50 Hz servo pulses and PWM on pin 46
//
// To keep Timer5 exclusively for the motor driver we reserve all Timer5
// slots with dummy Servo instances that are NEVER attached. We also reserve
// Timer3 so pins 2/3 remain usable as hardware PWM outputs in vehicle mode.
// Real servo/ESC objects are intentionally ordered so that:
//   - vehicle/drone Servo outputs live on Timer1
//   - manipulator fallback Servo outputs live on Timer4
static const uint8_t SERVO_TIMER_CHANNELS_PER_TIMER = 12;
static const uint8_t SERVO_REAL_VEHICLE_DRONE_COUNT = 9;
static const uint8_t SERVO_TIMER1_PADDING_COUNT = SERVO_TIMER_CHANNELS_PER_TIMER - SERVO_REAL_VEHICLE_DRONE_COUNT;
Servo reservedTimer5Servos[SERVO_TIMER_CHANNELS_PER_TIMER];

Servo vehicleCameraPanServo;
Servo vehicleCameraTiltServo;
Servo vehicleLidarScanServo;
bool vehicleCameraPanAttached = false;
bool vehicleCameraTiltAttached = false;
bool vehicleLidarScanAttached = false;

Servo droneEscServos[4];
Servo droneCameraPanServo;
Servo droneCameraTiltServo;
bool droneEscAttached[4] = {false, false, false, false};
bool droneCameraPanAttached = false;
bool droneCameraTiltAttached = false;
int droneMotorMicros[4] = {DRONE_ESC_MIN_US, DRONE_ESC_MIN_US, DRONE_ESC_MIN_US, DRONE_ESC_MIN_US};

Servo reservedTimer1Padding[SERVO_TIMER1_PADDING_COUNT];
Servo reservedTimer3Servos[SERVO_TIMER_CHANNELS_PER_TIMER];

enum MotorSequence : uint8_t {
  PHASE_0 = 0,
  PHASE_1,
  PHASE_2,
  PHASE_3,
  PHASE_4,
  PHASE_5,
  PHASE_6,
  PHASE_7
};

class ManualMotorPulse {
  public:
    typedef void (*StepCallback)();

    ManualMotorPulse(StepCallback forwardCb = NULL, StepCallback backwardCb = NULL)
      : _forwardCb(forwardCb),
        _backwardCb(backwardCb),
        _maxSpeedStepsPerSec(1.0f),
        _speedStepsPerSec(0.0f),
        _outputsEnabled(false),
        _lastStepUs(0UL) {}

    // Sets max speed.
    void setMaxSpeed(float maxSpeedStepsPerSec) {
      if (maxSpeedStepsPerSec < 0.0f) maxSpeedStepsPerSec = -maxSpeedStepsPerSec;
      if (maxSpeedStepsPerSec < 1.0f) maxSpeedStepsPerSec = 1.0f;
      _maxSpeedStepsPerSec = maxSpeedStepsPerSec;

      if (_speedStepsPerSec > _maxSpeedStepsPerSec) _speedStepsPerSec = _maxSpeedStepsPerSec;
      if (_speedStepsPerSec < -_maxSpeedStepsPerSec) _speedStepsPerSec = -_maxSpeedStepsPerSec;
    }

    // Sets speed.
    void setSpeed(float speedStepsPerSec) {
      _speedStepsPerSec = speedStepsPerSec;
      if (_speedStepsPerSec > _maxSpeedStepsPerSec) _speedStepsPerSec = _maxSpeedStepsPerSec;
      if (_speedStepsPerSec < -_maxSpeedStepsPerSec) _speedStepsPerSec = -_maxSpeedStepsPerSec;
      if (_speedStepsPerSec == 0.0f) {
        _lastStepUs = micros();
      }
    }

    // Utility: enable outputs.
    void enableOutputs() {
      _outputsEnabled = true;
      if (_lastStepUs == 0UL) _lastStepUs = micros();
    }

    // Utility: disable outputs.
    void disableOutputs() {
      _outputsEnabled = false;
      _lastStepUs = 0UL;
    }

    // Utility: run speed.
    void runSpeed() {
      if (!_outputsEnabled) return;
      if (_speedStepsPerSec == 0.0f) return;

      float absSpeed = (_speedStepsPerSec >= 0.0f) ? _speedStepsPerSec : -_speedStepsPerSec;
      if (absSpeed < 0.5f) return;

      unsigned long nowUs = micros();
      if (_lastStepUs == 0UL) {
        _lastStepUs = nowUs;
        return;
      }

      unsigned long stepIntervalUs = (unsigned long)(1000000.0f / absSpeed);
      if (stepIntervalUs < 1UL) stepIntervalUs = 1UL;

      StepCallback cb = (_speedStepsPerSec > 0.0f) ? _forwardCb : _backwardCb;
      if (!cb) return;

      uint8_t burstCount = 0;
      while ((unsigned long)(nowUs - _lastStepUs) >= stepIntervalUs && burstCount < 8) {
        _lastStepUs += stepIntervalUs;
        cb();
        burstCount++;
      }

      unsigned long burstWindowUs = stepIntervalUs * 8UL;
      if ((unsigned long)(nowUs - _lastStepUs) > burstWindowUs) {
        _lastStepUs = nowUs;
      }
    }

  private:
    StepCallback _forwardCb;
    StepCallback _backwardCb;
    float _maxSpeedStepsPerSec;
    float _speedStepsPerSec;
    bool _outputsEnabled;
    unsigned long _lastStepUs;
};

struct ServoState {
  float   target[MAX_CHANNELS];
  float   actual[MAX_CHANNELS];
  int16_t offset[MAX_CHANNELS];
  int16_t spanCenti[MAX_CHANNELS];
  uint16_t rampMask;
  int8_t  optimize[MAX_CHANNELS];
  int8_t  direction[MAX_CHANNELS];
  uint16_t neutralMask;
  int32_t minLimitCenti[MAX_CHANNELS];
  int32_t maxLimitCenti[MAX_CHANNELS];
};

extern ServoState servos;

// Utility: pack centi degrees 16.
static inline int16_t packCentiDeg16(float v) {
  return (int16_t)lroundf(v * 100.0f);
}
// Utility: unpack centi degrees 16.
static inline float unpackCentiDeg16(int16_t v) {
  return ((float)v) * 0.01f;
}
// Utility: pack centi degrees 32.
static inline int32_t packCentiDeg32(float v) {
  return (int32_t)lroundf(v * 100.0f);
}
// Utility: unpack centi degrees 32.
static inline float unpackCentiDeg32(int32_t v) {
  return ((float)v) * 0.01f;
}
// Returns servo span.
static inline float getServoSpan(uint8_t ch) {
  return unpackCentiDeg16(servos.spanCenti[ch]);
}
// Sets servo span.
static inline void setServoSpan(uint8_t ch, float v) {
  servos.spanCenti[ch] = packCentiDeg16(clampValue(v, 0.10f, 10.0f));
}
// Returns channel input max.
static inline float getChannelInputMax(uint8_t ch) {
  if (ch == SERVO_EXTENDED_RANGE_CHANNEL) return SERVO_EXTENDED_INPUT_MAX;
  return SERVO_INPUT_MAX;
}
// Clamps servo input to channel range.
static inline float clampServoInputToChannelRange(uint8_t ch, float v) {
  return clampValue(v, SERVO_INPUT_MIN, getChannelInputMax(ch));
}
// Returns servo min limit.
static inline float getServoMinLimit(uint8_t ch) {
  if (!isServoChannel(ch)) return SERVO_INPUT_MIN;
  float chMax = getChannelInputMax(ch);
  float mn = unpackCentiDeg32(servos.minLimitCenti[ch]);
  float mx = unpackCentiDeg32(servos.maxLimitCenti[ch]);
  mn = clampValue(mn, SERVO_INPUT_MIN, chMax);
  mx = clampValue(mx, SERVO_INPUT_MIN, chMax);
  return (mn <= mx) ? mn : mx;
}
// Sets servo min limit.
static inline void setServoMinLimit(uint8_t ch, float v) {
  float chMax = getChannelInputMax(ch);
  servos.minLimitCenti[ch] = packCentiDeg32(clampValue(v, SERVO_INPUT_MIN, chMax));
}
// Returns servo max limit.
static inline float getServoMaxLimit(uint8_t ch) {
  if (!isServoChannel(ch)) return SERVO_INPUT_MAX;
  float chMax = getChannelInputMax(ch);
  float mn = unpackCentiDeg32(servos.minLimitCenti[ch]);
  float mx = unpackCentiDeg32(servos.maxLimitCenti[ch]);
  mn = clampValue(mn, SERVO_INPUT_MIN, chMax);
  mx = clampValue(mx, SERVO_INPUT_MIN, chMax);
  return (mn <= mx) ? mx : mn;
}
// Sets servo max limit.
static inline void setServoMaxLimit(uint8_t ch, float v) {
  float chMax = getChannelInputMax(ch);
  servos.maxLimitCenti[ch] = packCentiDeg32(clampValue(v, SERVO_INPUT_MIN, chMax));
}
// Returns servo ramp.
static inline bool getServoRamp(uint8_t ch) {
  return ((servos.rampMask >> ch) & 0x01u) != 0;
}
// Sets servo ramp.
static inline void setServoRamp(uint8_t ch, bool en) {
  if (en) servos.rampMask |=  (uint16_t)(1u << ch);
  else    servos.rampMask &= (uint16_t)~(1u << ch);
}
// Returns servo neutral.
static inline bool getServoNeutral(uint8_t ch) {
  return ((servos.neutralMask >> ch) & 0x01u) != 0;
}
// Sets servo neutral.
static inline void setServoNeutral(uint8_t ch, bool en) {
  if (en) servos.neutralMask |=  (uint16_t)(1u << ch);
  else    servos.neutralMask &= (uint16_t)~(1u << ch);
}

struct MotorControl {
  float kp;
  float ki;
  float kd;

  float targetAngle;          // Final commanded target from UI/serial.
  float controlTargetAngle;   // Slewed internal target used by PID/stepper.
  float currentAngle;
  float filteredPot;
  float filteredAngle;
  float filteredSpeed;
  float lastAngle;
  float integral;

  uint16_t pwmValue;
  bool inHoldBand;
  bool directionForward;
  bool controlTargetInitialized;
  bool commandTrackerInitialized;
  int8_t breakawaySign;

  float lastCommandAngle;
  float commandVelocityDps;

  unsigned long lastControlUs;
  unsigned long lastAngleUs;
  unsigned long lastStepUs;
  unsigned long holdStartMs;
  unsigned long lastCommandChangeMs;
  unsigned long breakawayUntilMs;
  unsigned long lastBreakawayMs;
  unsigned long stepDelayUs;

  MotorSequence seq;
};


enum AutoMotorStage : uint8_t {
  AUTO_STAGE_IDLE = 0,
  AUTO_STAGE_TORQUE,
  AUTO_STAGE_BANDS,
  AUTO_STAGE_DONE
};

struct AutoMotorCandidate {
  uint16_t minStepUs;
  uint16_t maxStepUs;
  float stopEnter;
  float stopExit;
  uint16_t pwmFar;
  uint16_t pwmNear;
  uint16_t pwmHold;
};

struct AutoMotorBest {
  bool valid;
  float score;
  AutoMotorCandidate params;
};

struct AutoMotorState {
  bool active;
  bool stopRequested;
  AutoMotorStage stage;
  AutoMotorStage cachedStage;
  uint8_t candidateIndex;
  uint8_t candidateCount;
  uint8_t motionIndex;
  uint8_t motionCount;
  bool motionRunning;
  bool motionMoved;
  float motionStartAngle;
  float motionTargetAngle;
  float motionMaxOvershoot;
  float motionSumAbsError;
  float motionJitter;
  float motionStartMoveErr;
  float holdMeanAbsError;
  float holdMinAngle;
  float holdMaxAngle;
  uint16_t motionSamples;
  uint16_t holdSamples;
  unsigned long stageStartMs;
  unsigned long motionStartMs;
  unsigned long settleStartMs;
  unsigned long motionSettleMs;
  unsigned long lastProgressMs;
  AutoMotorCandidate seed;
  AutoMotorCandidate current;
  AutoMotorCandidate candidates[4];
  AutoMotorBest best;
  float testMotions[5];
  uint8_t modeAttemptIndex;
  MotorDriveMode originalMode;
};

enum MotorCalStage : uint8_t {
  MOTOR_CAL_IDLE = 0,
  MOTOR_CAL_WAIT_LIMIT1,
  MOTOR_CAL_WAIT_LIMIT2,
  MOTOR_CAL_DONE
};

struct MotorCalState {
  bool active;
  MotorCalStage stage;
  int16_t limit1Adc;
  int16_t limit2Adc;
};

struct DirectMotorTestState {
  bool active;
  bool forward;
  uint16_t totalSteps;
  uint16_t stepsRemaining;
  uint16_t pwmRaw;
  unsigned long stepDelayUs;
  unsigned long lastStepUs;
};
struct RuntimeState {
  bool active;
  bool configMode;
  bool pwmReady;
  bool ina1Ready;
  bool ina2Ready;
  bool mpu1Ready;
  bool mpu2Ready;
  bool compassReady;
  bool servoOutputsArmed;
  bool motorArmed;
  unsigned long lastSensorMs;
  unsigned long lastServoUpdateMs;
};

struct CompassCalibrationData {
  uint32_t magic;
  int16_t minX;
  int16_t maxX;
  int16_t minY;
  int16_t maxY;
  int16_t minZ;
  int16_t maxZ;
  float offsetX;
  float offsetY;
  float offsetZ;
  float scaleX;
  float scaleY;
  float scaleZ;
};

struct CompassCalibrationRuntime {
  bool active;
  bool saved;
  bool manualMode;
  uint8_t stepIndex;
  uint8_t subStep;
  unsigned long stepStartedMs;
  unsigned long startedMs;
  unsigned long lastSampleMs;
  float progress;
  int16_t minX;
  int16_t maxX;
  int16_t minY;
  int16_t maxY;
  int16_t minZ;
  int16_t maxZ;
};

enum CompassInitStage : uint8_t {
  COMPASS_INIT_IDLE = 0,
  COMPASS_INIT_SELECT_CANDIDATE,
  COMPASS_INIT_TRY_HMC,
  COMPASS_INIT_WAIT_HMC_SETTLE,
  COMPASS_INIT_TRY_QMC_RESET,
  COMPASS_INIT_WAIT_QMC_RESET,
  COMPASS_INIT_WAIT_QMC_CONFIG
};

struct CompassInitState {
  bool active;
  bool resultReported;
  uint8_t candidateCount;
  uint8_t candidateIndex;
  uint8_t candidateAddrs[2];
  uint8_t currentAddr;
  CompassInitStage stage;
  unsigned long waitStartMs;
};

struct ArmGeometryConfig {
  float baseRadius;
  float baseHeight;
  float baseBlockW;
  float baseBlockH;
  float baseBlockD;
  float upperW;
  float upperH;
  float upperD;
  float foreW;
  float foreH;
  float foreD;
  float wristW;
  float wristH;
  float wristD;
  float fingerW;
  float fingerH;
  float fingerD;
  float baseCylinderYOffset;
  float baseBlockYOffset;
  float gripperYOffset;
};

struct ManipulatorJointMapConfig {
  float servoZeroDeg;
  float memberZeroDeg;
  float servoSign;
};

struct ConfigExtrasPayload {
  bool wristGimbalEnabled;
  uint8_t reserved0[3];
  ManipulatorJointMapConfig manipJointMap[MANIP_MEMBER_COUNT];
};

struct SafetyConfigPayload {
  bool droneSafeReturnEnabled;
  bool vehicleSafeReturnEnabled;
  bool vehicleInclinePowerEnabled;
  uint8_t reserved0;
  float droneSafeReturnBatteryThresholdPct;
  float vehicleSafeReturnBatteryThresholdPct;
  float vehicleInclineBasePowerPct;
  float vehicleInclineMaxPowerPct;
  float vehicleInclineFullScaleDeg;
  float vehicleInclineDeadbandDeg;
  float vehicleInclinePowerAlpha;
  uint8_t reserved1[32];
};

struct ConfigPayload {
  uint8_t pca9685Addr;
  uint8_t ina219Addr1;
  uint8_t ina219Addr2;
  uint8_t mpu6050Addr1;
  uint8_t mpu6050Addr2;
  uint8_t compassAddr;
  uint8_t compassType;

  float kp;
  float ki;
  float kd;

  int16_t zeroAdc;
  int32_t spanRaw;

  uint16_t minStepDelayUs;
  uint16_t maxStepDelayUs;
  float stopEnter;
  float stopExit;
  uint16_t pwmFar;
  uint16_t pwmNear;
  uint16_t pwmHold;
  float angleFilterAlpha;
  float speedFilterAlpha;
  uint32_t controlIntervalUs;
  float servoMoveStep;
  float servoRampSmoothing;
  uint8_t reserved0;
  uint8_t manipMemberChannel[MANIP_MEMBER_COUNT];
  uint8_t reserved1[3];

  int16_t servoOffset[MAX_CHANNELS];
  float servoSpan[MAX_CHANNELS];
  bool servoRamp[MAX_CHANNELS];
  int8_t servoOptimize[MAX_CHANNELS];
  int8_t servoDirection[MAX_CHANNELS];
  float servoMinLimit[MAX_CHANNELS];
  float servoMaxLimit[MAX_CHANNELS];
  ArmGeometryConfig armGeom;
  uint8_t pwmBoot[4]; // CH12..15 startup percent
  bool armStabilizeEnable;
  float armStabilizeGain;
  float armStabilizeThreshold;
  bool gripPressureEnable;
  float gripPressureGain;
  bool collisionEnabled;
  bool vehicleHoldLastMotionOnLinkLoss;
  bool droneHoldLastMotionOnLinkLoss;
  uint8_t seqLossReserved[2];
  VehicleProcessingProfile vehicleProfile;
  DroneProcessingProfile droneProfile;
  VehicleControlProfile vehicleControlProfile;
  DroneControlProfile droneControlProfile;
};

struct PersistentConfig {
  uint32_t magic;
  uint16_t schemaId;
  ConfigPayload payload;
  uint32_t crc;
};

struct PersistentConfigExtras {
  uint32_t magic;
  uint16_t schemaId;
  ConfigExtrasPayload payload;
  uint32_t crc;
};

struct PersistentSafetyConfig {
  uint32_t magic;
  uint16_t schemaId;
  SafetyConfigPayload payload;
  uint32_t crc;
};

const int EEPROM_ADDR_ROBOT_MODE = EEPROM_ADDR + (int)sizeof(PersistentConfig) + 8;
const int EEPROM_ADDR_COMPASS_CAL = EEPROM_ADDR_ROBOT_MODE + 8;
const int EEPROM_ADDR_CONFIG_EXTRAS = EEPROM_ADDR_COMPASS_CAL + (int)sizeof(CompassCalibrationData) + 8;
const int EEPROM_ADDR_SAFETY_CONFIG = EEPROM_ADDR_CONFIG_EXTRAS + (int)sizeof(PersistentConfigExtras) + 8;
const uint32_t COMPASS_CAL_MAGIC = 0x43414C4DuL;
const uint32_t CONFIG_EXTRAS_MAGIC = 0x45585443uL; // EXTC
const uint16_t CONFIG_EXTRAS_SCHEMA_ID = 1;
const unsigned long STREAM_FAILSAFE_TIMEOUT_MS = 700UL;

// ---------------- Devices / Globals ----------------
uint8_t pca9685Addr = 0x50;
uint8_t ina219Addr1 = 0x41;
uint8_t ina219Addr2 = 0x44;
uint8_t mpu6050Addr1 = 0x68;
uint8_t mpu6050Addr2 = 0x69;
uint8_t compassAddr = 0; // 0 = auto detect
CompassType compassType = COMPASS_NONE;
CompassInitState compassInit = {false, false, 0, 0, {0, 0}, 0, COMPASS_INIT_IDLE, 0};
DirectMotorTestState directMotorTest = {false, true, 0, 0, 0, 0, 0};

Adafruit_PWMServoDriver pwm;
Adafruit_INA219 ina219_1(0x41);
Adafruit_INA219 ina219_2(0x44);
MPU6050 mpu6050_1(0x68);
MPU6050 mpu6050_2(0x69);
NewPing sonar(SONAR_PIN, SONAR_PIN, SONAR_MAX_DISTANCE_CM);
Servo fallbackServos[11];
uint16_t fallbackServoAttachedMask = 0;

// Checks whether fallback servo attached.
static inline bool isFallbackServoAttached(uint8_t idx) {
  return idx < 11 && ((fallbackServoAttachedMask >> idx) & 0x01u) != 0;
}
// Sets fallback servo attached.
static inline void setFallbackServoAttached(uint8_t idx, bool attached) {
  if (idx >= 11) return;
  if (attached) fallbackServoAttachedMask |= (uint16_t)(1u << idx);
  else fallbackServoAttachedMask &= (uint16_t)~(1u << idx);
}

ServoState servos;
MotorControl motor;
RuntimeState rt;

// Forward declarations required before manual step pulse generator constructor
void accelStepForward();
void accelStepBackward();
ManualMotorPulse motorPulse(accelStepForward, accelStepBackward);

int16_t  motorCalibrationZeroADC = 50;
int32_t  motorOneTurnSpanRaw     = 380;
uint16_t minStepDelayUs            = 2800;
uint16_t maxStepDelayUs            = 16000;
float    motorStopEnter          = 0.8f;
float    motorStopExit           = 1.4f;
uint16_t movePwmFar                = 285;
uint16_t movePwmNear               = 215;
uint16_t holdPwm                   = 155;
float    angleFilterAlpha          = 0.700f;
float    motorBaseA0DeadbandDeg    = MOTOR_BASE_A0_DEADBAND_DEFAULT_DEG;
float    motorDcPidEnterBandDeg    = MOTOR_DC_PID_ENTER_BAND_DEFAULT_DEG;
float    motorDcPidExitBandDeg     = MOTOR_DC_PID_EXIT_BAND_DEFAULT_DEG;
float    speedFilterAlpha          = 0.240f;
uint32_t controlIntervalUs         = 3000UL;
float    servoMoveStepDeg          = 2.00f;
float    servoRampSmoothing       = 1.00f;
float    servoHoldDeadbandDeg     = 1.50f;
float    servoWriteEpsilonDeg     = 1.20f;
float    servoTargetEpsilonDeg    = 0.60f;
float    servoLastWrittenDeg[MAX_CHANNELS];
uint16_t servoWriteValidMask = 0;
unsigned long manipBaseServoQuietUntilMs = 0;
bool motorDcDriveDirectionValid = false;
bool motorDcLastDriveForward = true;
// Returns servo write valid.
static inline bool getServoWriteValid(uint8_t ch) {
  return ch < MAX_CHANNELS && ((servoWriteValidMask >> ch) & 0x01u) != 0;
}
// Sets servo write valid.
static inline void setServoWriteValid(uint8_t ch, bool valid) {
  if (ch >= MAX_CHANNELS) return;
  if (valid) servoWriteValidMask |= (uint16_t)(1u << ch);
  else servoWriteValidMask &= (uint16_t)~(1u << ch);
}
unsigned long servoPlannerLastUpdateMs = 0;
uint8_t  pwmBootPercent[4]         = {50, 50, 50, 50};
const uint8_t DEFAULT_MANUAL_PWM_PERCENT = 50;

// Sets all manual PWM startup outputs to the neutral 50% value.
static inline void setAllPwmBootPercentToNeutral() {
  for (uint8_t i = 0; i < 4; i++) pwmBootPercent[i] = DEFAULT_MANUAL_PWM_PERCENT;
}


ArmGeometryConfig armGeom = {
  15.0f,
  30.0f,
  12.5f,
  20.0f,
  12.0f,
  7.65f,
  37.0f,
  7.5f,
  5.544f,
  29.0f,
  5.412f,
  3.219f,
  6.0f,
  3.219f,
  0.709f,
  10.0f,
  0.945f,
  -15.5f,
  -40.5f,
  -1.8f
};

#define armBaseRadius (armGeom.baseRadius)
#define armBaseHeight (armGeom.baseHeight)
#define armBaseBlockW (armGeom.baseBlockW)
#define armBaseBlockH (armGeom.baseBlockH)
#define armBaseBlockD (armGeom.baseBlockD)
#define armUpperW (armGeom.upperW)
#define armUpperH (armGeom.upperH)
#define armUpperD (armGeom.upperD)
#define armForeW (armGeom.foreW)
#define armForeH (armGeom.foreH)
#define armForeD (armGeom.foreD)
#define armWristW (armGeom.wristW)
#define armWristH (armGeom.wristH)
#define armWristD (armGeom.wristD)
#define armFingerW (armGeom.fingerW)
#define armFingerH (armGeom.fingerH)
#define armFingerD (armGeom.fingerD)
#define armBaseCylinderYOffset (armGeom.baseCylinderYOffset)
#define armBaseBlockYOffset (armGeom.baseBlockYOffset)
#define armGripperYOffset (armGeom.gripperYOffset)

bool collisionFlag = false;
bool collisionRuntimeEnabled = true;
bool configSessionDirty = false;

struct CollisionPoseCache {
  int16_t baseCenti;
  int16_t upperCenti;
  int16_t foreCenti;
  int16_t forearmRollCenti;
  int16_t wristPitchCenti;
  int16_t wristRotCenti;
  int16_t gripCenti;
};

bool collisionPoseCacheValid = false;
CollisionPoseCache collisionPoseCache = {0, 0, 0, 0, 0, 0, 0};

// Collision performance notes:
// The runtime stream alternates between proposed ARMJ poses and actual servo
// poses.  A single severity cache gets thrashed by that alternation and forces
// the OBB/cylinder solver to run on almost every packet.  Keep a tiny rotating
// cache instead so the proposed pose, actual pose and HOME pose can coexist.
const uint8_t COLLISION_SEVERITY_CACHE_SLOTS = 4;
bool collisionSeverityCacheValid[COLLISION_SEVERITY_CACHE_SLOTS] = {false, false, false, false};
CollisionPoseCache collisionSeverityCachePose[COLLISION_SEVERITY_CACHE_SLOTS] = {
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0}
};
uint8_t collisionSeverityCacheValue[COLLISION_SEVERITY_CACHE_SLOTS] = {0, 0, 0, 0};
unsigned long collisionSeverityCacheMs[COLLISION_SEVERITY_CACHE_SLOTS] = {0, 0, 0, 0};
uint8_t collisionSeverityCacheNextSlot = 0;

// Candidate guard cache: dense ARMJ streams frequently send pose changes much
// smaller than the mechanical servo resolution.  Reuse the last guard decision
// for very close candidate poses to keep the serial parser, CH0 PID and
// telemetry from waiting on the heavy collision solver.
bool collisionCandidateDecisionValid = false;
CollisionPoseCache collisionCandidateDecisionPose = {0, 0, 0, 0, 0, 0, 0};
bool collisionCandidateDecisionAllowed = true;
bool collisionCandidateDecisionCollisionFlag = false;
unsigned long collisionCandidateDecisionMs = 0;

// Last known collision-free manipulator pose.  Recovery is judged against this
// pose instead of only by the raw number of colliding pairs, because while the
// arm is backing out the pair count can stay constant for several degrees.
bool collisionLastSafePoseValid = false;
CollisionPoseCache collisionLastSafePose = {0, 0, 0, 0, 0, 0, 0};

const int16_t COLLISION_POSE_EPS_CENTI = 75;       // 0.75 deg: avoids recomputing OBBs for noise-sized motion.
const int16_t COLLISION_SEVERITY_CACHE_EPS_CENTI = 125;
const int16_t COLLISION_CANDIDATE_REUSE_EPS_CENTI = 175;
const int32_t COLLISION_RECOVERY_PROGRESS_EPS_CENTI = 60; // 0.60 deg of total angular progress toward last safe pose.
const unsigned long COLLISION_GUARD_INTERVAL_MS = 70UL;
const unsigned long COLLISION_SEVERITY_CACHE_MAX_MS = 140UL;
const unsigned long COLLISION_CANDIDATE_REUSE_MAX_MS = 80UL;
unsigned long lastCollisionBlockedNoticeMs = 0;
unsigned long lastCollisionGuardEvalMs = 0;
const unsigned long COLLISION_BLOCKED_NOTICE_INTERVAL_MS = 250UL;
bool armStabilizeEnable = false;
float armStabilizeGain = 0.25f;
float armStabilizeThreshold = 2.5f;
bool gripPressureEnable = true;
float gripPressureGain = 1.00f;
float gripPressureEstimate = 0.0f;
float gripIdleCurrentAvg = 0.0f;
int gripPressureDeltaMa = 0;
bool wristGimbalEnabled = false;
bool wristGimbalRefValid = false;
float wristGimbalPitchRef = 0.0f;
float wristGimbalRollRef = 0.0f;
const float wristGimbalGainPitch = 0.8f;
const float wristGimbalGainRoll = 0.8f;
const int8_t SERVO_OPTIMIZE_DESCEND_SIGN[MAX_CHANNELS] PROGMEM = {0, +1, +1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const float SERVO_OPTIMIZE_DESCEND_FACTOR[MAX_CHANNELS] PROGMEM = {1.0f, 0.70f, 0.75f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

bool     motorFeedbackInsideWindow = true;
uint8_t  motorRecoveryPwm          = 0;
int16_t  compassX = 0, compassY = 0, compassZ = 0;
float    compassHeadingDeg = 0.0f;
unsigned long lastRuntimeStreamRxMs = 0;
uint32_t lastRuntimeStreamSeq = 0;
bool runtimeStreamActive = false;
bool runtimeStreamFailsafeLatched = false;
bool vehicleHoldLastMotionOnLinkLoss = false;
bool droneHoldLastMotionOnLinkLoss = false;
int vehicleLinkHoldLastLeftPct = 0;
int vehicleLinkHoldLastRightPct = 0;
bool vehicleLinkHoldLastMotionValid = false;
bool vehicleLinkHoldReturnActive = false;
int droneLinkHoldLastThrottlePct = 0;
int droneLinkHoldLastYawPct = 0;
int droneLinkHoldLastPitchPct = 0;
int droneLinkHoldLastRollPct = 0;
int droneLinkHoldLastStrafePct = 0;
int droneLinkHoldLastForwardPct = 0;
bool droneLinkHoldLastMotionValid = false;
bool droneLinkHoldReturnActive = false;
CompassCalibrationData compassCal = {0, -2048, 2048, -2048, 2048, -2048, 2048, 0, 0, 0, 1, 1, 1};
CompassCalibrationRuntime compassCalRuntime = {false, false, false, 0, 0, 0, 0, 0, 0, -32768, 32767, -32768, 32767, -32768, 32767};

char serialBuffer[SERIAL_TEXT_BUFFER_SIZE];
bool serialHexMode = false;          // false = ASCII (boot/cfg), true = HEX-framed (runtime)
uint8_t serialIndex = 0;
uint8_t currentTextCommandSource = 0;
uint8_t runtimeFrameBuf[RUNTIME_FRAME_BUFFER_SIZE];
unsigned long bootCommandIgnoreUntilMs = 0;
AutoMotorState autoMotor;
MotorCalState motorCal;


struct RuntimeVec3 {
  float x;
  float y;
  float z;
};
struct RuntimeCapsule {
  RuntimeVec3 a;
  RuntimeVec3 b;
  float r;
};
struct ArmCollisionMath {
  static RuntimeVec3 v3(float x, float y, float z) {
    RuntimeVec3 v = {x, y, z};
    return v;
  }
  static RuntimeVec3 add(RuntimeVec3 a, RuntimeVec3 b) {
    return v3(a.x + b.x, a.y + b.y, a.z + b.z);
  }
  static RuntimeVec3 sub(RuntimeVec3 a, RuntimeVec3 b) {
    return v3(a.x - b.x, a.y - b.y, a.z - b.z);
  }
  static RuntimeVec3 mul(RuntimeVec3 a, float s) {
    return v3(a.x * s, a.y * s, a.z * s);
  }
  static float dot(RuntimeVec3 a, RuntimeVec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }
  // Utility: cross.
  static RuntimeVec3 cross(RuntimeVec3 a, RuntimeVec3 b) {
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
  }
  static float magSq(RuntimeVec3 a) {
    return dot(a, a);
  }
  static float mag(RuntimeVec3 a) {
    return sqrtf(magSq(a));
  }
  // Utility: normalize.
  static RuntimeVec3 normalize(RuntimeVec3 a) {
    float m = mag(a);
    if (m <= 1e-9f) return v3(0, 0, 0);
    return mul(a, 1.0f / m);
  }
  static RuntimeVec3 rotateZ(RuntimeVec3 v, float deg) {
    float r = deg * 0.01745329252f;
    float c = cosf(r), s = sinf(r);
    return v3(c * v.x - s * v.y, s * v.x + c * v.y, v.z);
  }
  static RuntimeVec3 rotateY(RuntimeVec3 v, float deg) {
    float r = deg * 0.01745329252f;
    float c = cosf(r), s = sinf(r);
    return v3(c * v.x + s * v.z, v.y, -s * v.x + c * v.z);
  }
  // Utility: seg seg dist sq.
  static float segSegDistSq(RuntimeVec3 p1, RuntimeVec3 q1, RuntimeVec3 p2, RuntimeVec3 q2) {
    RuntimeVec3 d1 = sub(q1, p1), d2 = sub(q2, p2), r = sub(p1, p2);
    float a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r), s = 0.0f, t = 0.0f;
    if (a <= 1e-6f && e <= 1e-6f) return magSq(sub(p1, p2));
    if (a <= 1e-6f) {
      t = clampValue(f / e, 0.0f, 1.0f);
    }
    else {
      float c = dot(d1, r);
      if (e <= 1e-6f) {
        s = clampValue(-c / a, 0.0f, 1.0f);
      }
      else {
        float b = dot(d1, d2), denom = a * e - b * b;
        s = (fabsf(denom) > 1e-9f) ? clampValue((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
        t = (b * s + f) / e;
        if (t < 0.0f) {
          t = 0.0f;
          s = clampValue(-c / a, 0.0f, 1.0f);
        }
        else if (t > 1.0f) {
          t = 1.0f;
          s = clampValue((b - c) / a, 0.0f, 1.0f);
        }
      }
    }
    RuntimeVec3 c1 = add(p1, mul(d1, s)), c2 = add(p2, mul(d2, t));
    return magSq(sub(c1, c2));
  }
  // Utility: capsule capsule.
  static bool capsuleCapsule(RuntimeCapsule c1, RuntimeCapsule c2, float margin) {
    float r = c1.r + c2.r + margin;
    return segSegDistSq(c1.a, c1.b, c2.a, c2.b) <= (r * r);
  }
};


// Builds default arm geometry config.
ArmGeometryConfig makeDefaultArmGeometryConfig() {
  ArmGeometryConfig cfg = {
    15.0f,
    30.0f,
    12.5f,
    20.0f,
    12.0f,
    7.65f,
    37.0f,
    7.5f,
    5.544f,
    29.0f,
    5.412f,
    3.219f,
    6.0f,
    3.219f,
    0.709f,
    10.0f,
    0.945f,
    -15.5f,
    -40.5f,
    -1.8f
  };
  return cfg;
}

// Sanitizes and clamps arm geometry config.
void sanitizeArmGeometryConfig(ArmGeometryConfig &cfg) {
  cfg.baseRadius = clampValue(cfg.baseRadius, 1.0f, 500.0f);
  cfg.baseHeight = clampValue(cfg.baseHeight, 1.0f, 500.0f);
  cfg.baseBlockW = clampValue(cfg.baseBlockW, 1.0f, 500.0f);
  cfg.baseBlockH = clampValue(cfg.baseBlockH, 1.0f, 500.0f);
  cfg.baseBlockD = clampValue(cfg.baseBlockD, 1.0f, 500.0f);
  cfg.upperW = clampValue(cfg.upperW, 1.0f, 500.0f);
  cfg.upperH = clampValue(cfg.upperH, 1.0f, 500.0f);
  cfg.upperD = clampValue(cfg.upperD, 1.0f, 500.0f);
  cfg.foreW = clampValue(cfg.foreW, 1.0f, 500.0f);
  cfg.foreH = clampValue(cfg.foreH, 1.0f, 500.0f);
  cfg.foreD = clampValue(cfg.foreD, 1.0f, 500.0f);
  cfg.wristW = clampValue(cfg.wristW, 1.0f, 500.0f);
  cfg.wristH = clampValue(cfg.wristH, 1.0f, 500.0f);
  cfg.wristD = clampValue(cfg.wristD, 1.0f, 500.0f);
  cfg.fingerW = clampValue(cfg.fingerW, 0.1f, 500.0f);
  cfg.fingerH = clampValue(cfg.fingerH, 0.1f, 500.0f);
  cfg.fingerD = clampValue(cfg.fingerD, 0.1f, 500.0f);
  cfg.baseCylinderYOffset = clampValue(cfg.baseCylinderYOffset, -1000.0f, 1000.0f);
  cfg.baseBlockYOffset = clampValue(cfg.baseBlockYOffset, -1000.0f, 1000.0f);
  cfg.gripperYOffset = clampValue(cfg.gripperYOffset, -1000.0f, 1000.0f);
}

// Copies arm geometry to payload.
void copyArmGeometryToPayload(ConfigPayload &payload, const ArmGeometryConfig &cfg) {
  ArmGeometryConfig sanitized = cfg;
  sanitizeArmGeometryConfig(sanitized);
  payload.armGeom = sanitized;
}

// Loads arm geometry from payload.
void loadArmGeometryFromPayload(const ConfigPayload &payload, ArmGeometryConfig &cfg) {
  cfg = payload.armGeom;
  sanitizeArmGeometryConfig(cfg);
}

enum ArmColliderType : uint8_t {
  ARM_COLL_GROUND = 0,
  ARM_COLL_BASE_CYLINDER,
  ARM_COLL_BASE_BLOCK,
  ARM_COLL_UPPER_ARM,
  ARM_COLL_FOREARM,
  ARM_COLL_WRIST_VERTICAL,
  ARM_COLL_FINGER_R,
  ARM_COLL_FINGER_L
};

struct ArmTransform {
  RuntimeVec3 p;
  RuntimeVec3 ux;
  RuntimeVec3 uy;
  RuntimeVec3 uz;
};

struct ArmOOBB {
  ArmColliderType type;
  RuntimeVec3 c;
  RuntimeVec3 ux;
  RuntimeVec3 uy;
  RuntimeVec3 uz;
  float ex;
  float ey;
  float ez;
};

struct ArmCylinderY {
  ArmColliderType type;
  float cx;
  float cz;
  float radius;
  float yMin;
  float yMax;
};

struct ArmGenericCollider {
  bool isCylinder;
  ArmColliderType type;
  ArmOOBB obb;
  ArmCylinderY cyl;
};

// Arm geometry is kept in a single runtime structure and synced to/from
// persistent storage through helper functions. This avoids carrying a second
// mutable copy of each geometry field in unrelated globals.
ArmGeometryConfig makeDefaultArmGeometryConfig();
void sanitizeArmGeometryConfig(ArmGeometryConfig &cfg);
void copyArmGeometryToPayload(ConfigPayload &payload, const ArmGeometryConfig &cfg);
void loadArmGeometryFromPayload(const ConfigPayload &payload, ArmGeometryConfig &cfg);

// Explicit prototypes to keep Arduino's auto-prototyper from breaking typed signatures.
ArmTransform armIdentityTransform();
void armTranslateLocal(ArmTransform &t, float x, float y, float z);
void armRotateLocalZ(ArmTransform &t, float deg);
void armRotateLocalY(ArmTransform &t, float deg);
float armForearmRollServoToLocalYRotationDeg(float servoDeg);
ArmOOBB armBuildOBB(const ArmTransform &t, float w, float h, float d, ArmColliderType type);
float armOBBProjectRadius(const ArmOOBB &b, RuntimeVec3 axis);
bool armOBBOverlapOnAxis(const ArmOOBB &a, const ArmOOBB &b, RuntimeVec3 axis, float tol);
bool armIntervalsOverlap(float aMin, float aMax, float bMin, float bMax, float tol);
float armDot2XZ(RuntimeVec3 a, RuntimeVec3 b);
float armClampFloat(float v, float lo, float hi);
bool armCylinderOBBIntersects(const ArmCylinderY &c, const ArmOOBB &b, float tol);
bool armOBBIntersectsOBB(const ArmOOBB &a, const ArmOOBB &b, float tol);
bool armColliderIntersects(const ArmGenericCollider &a, const ArmGenericCollider &b, float tol);
bool armShouldIgnoreCollisionPair(ArmColliderType a, ArmColliderType b);
uint8_t armCollectPoseColliders(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg, ArmGenericCollider outCols[]);
bool armPoseHasOOBBCollision(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg);

float armBaseServoZeroDeg = 0.0f;
float armBaseMemberZeroDeg = 0.0f;
float armBaseServoSign = 1.0f;
float armUpperServoZeroDeg = 45.0f;
float armUpperMemberZeroDeg = 0.0f;
float armUpperServoSign = 1.0f;
float armForeServoZeroDeg = 180.0f;
float armForeMemberZeroDeg = 0.0f;
float armForeServoSign = 1.0f;
float armForearmRollServoZeroDeg = 90.0f;
float armForearmRollMemberZeroDeg = 0.0f;
float armForearmRollServoSign = 1.0f;
float armWristPitchServoZeroDeg = 90.0f;
float armWristPitchMemberZeroDeg = 0.0f;
float armWristPitchServoSign = 1.0f;
float armWristRollServoZeroDeg = 90.0f;
float armWristRollMemberZeroDeg = 0.0f;
float armWristRollServoSign = 1.0f;
float armGripServoZeroDeg = 0.0f;
float armGripMemberZeroDeg = 0.0f;
float armGripServoSign = 1.0f;

// Wraps signed degrees.
float wrapSignedDeg(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

// Wraps absolute degrees.
float wrapAbsoluteDeg(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

// Protects the CH0 manipulator base from circular wrap-around at its
// mechanical limits.  Near 359 deg, a target near 0 deg means "keep pushing
// into the upper stop", not "travel the whole arm to the lower stop".
// Near 0 deg, the opposite rule applies.
float resolveLinearBaseTargetDeg(float requestedDeg, float actualDeg) {
  float target = clampValue(requestedDeg, MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);
  float actual = clampValue(actualDeg, MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);
  float guard = clampValue(MOTOR_LINEAR_END_GUARD_DEG, 1.0f, MOTOR_INPUT_MAX * 0.25f);

  if (actual >= (MOTOR_INPUT_MAX - guard) && target <= (MOTOR_INPUT_MIN + guard)) {
    return MOTOR_INPUT_MAX;
  }

  if (actual <= (MOTOR_INPUT_MIN + guard) && target >= (MOTOR_INPUT_MAX - guard)) {
    return MOTOR_INPUT_MIN;
  }

  return target;
}

// Utility: manip servo to member degrees.
float manipServoToMemberDeg(float servoDeg, float servoZeroDeg, float memberZeroDeg, float sign) {
  float safeSign = (fabs(sign) < 0.0001f) ? 1.0f : sign;
  return wrapSignedDeg(memberZeroDeg + (servoDeg - servoZeroDeg) * safeSign);
}

// Utility: manip member to servo degrees.
float manipMemberToServoDeg(float memberDeg, float servoZeroDeg, float memberZeroDeg, float sign) {
  float safeSign = (fabs(sign) < 0.0001f) ? 1.0f : sign;
  return servoZeroDeg + (memberDeg - memberZeroDeg) / safeSign;
}

// Arm helper for base servo to forward yaw degrees.
float armBaseServoToForwardYawDeg(float servoDeg) {
  float safeSign = (fabs(armBaseServoSign) < 0.0001f) ? 1.0f : armBaseServoSign;
  return wrapAbsoluteDeg(armBaseMemberZeroDeg + (servoDeg - armBaseServoZeroDeg) * safeSign);
}

// Arm helper for upper servo to ground degrees.
float armUpperServoToGroundDeg(float servoDeg) {
  return manipServoToMemberDeg(servoDeg, armUpperServoZeroDeg, armUpperMemberZeroDeg, armUpperServoSign);
}

// Arm helper for fore servo to relative degrees.
float armForeServoToRelativeDeg(float servoDeg) {
  return manipServoToMemberDeg(servoDeg, armForeServoZeroDeg, armForeMemberZeroDeg, armForeServoSign);
}

// Arm helper for fore servo to ground degrees.
float armForeServoToGroundDeg(float upperServoDeg, float foreServoDeg) {
  return wrapSignedDeg(armUpperServoToGroundDeg(upperServoDeg) + armForeServoToRelativeDeg(foreServoDeg));
}

// Arm helper for wrist servo to relative degrees.
float armWristServoToRelativeDeg(float servoDeg) {
  return manipServoToMemberDeg(servoDeg, armWristPitchServoZeroDeg, armWristPitchMemberZeroDeg, armWristPitchServoSign);
}

// Arm helper for forearm roll servo to member degrees.
float armForearmRollServoToMemberDeg(float servoDeg) {
  return manipServoToMemberDeg(servoDeg, armForearmRollServoZeroDeg, armForearmRollMemberZeroDeg, armForearmRollServoSign);
}

// Arm helper for wrist servo to ground degrees.
float armWristServoToGroundDeg(float upperServoDeg, float foreServoDeg, float wristServoDeg) {
  return wrapSignedDeg(armForeServoToGroundDeg(upperServoDeg, foreServoDeg) + armWristServoToRelativeDeg(wristServoDeg));
}

// Arm helper for wrist roll servo to member degrees.
float armWristRollServoToMemberDeg(float servoDeg) {
  return manipServoToMemberDeg(servoDeg, armWristRollServoZeroDeg, armWristRollMemberZeroDeg, armWristRollServoSign);
}

// Arm helper for grip servo to member degrees.
float armGripServoToMemberDeg(float servoDeg) {
  return clampValue(manipServoToMemberDeg(servoDeg, armGripServoZeroDeg, armGripMemberZeroDeg, armGripServoSign), 0.0f, MANIP_GRIPPER_MAX_DEG);
}

// Arm helper for base member to servo degrees.
float armBaseMemberToServoDeg(float memberDeg) {
  float safeSign = (fabs(armBaseServoSign) < 0.0001f) ? 1.0f : armBaseServoSign;
  return wrapAbsoluteDeg(armBaseServoZeroDeg + (memberDeg - armBaseMemberZeroDeg) / safeSign);
}

// Arm helper for upper member to servo degrees.
float armUpperMemberToServoDeg(float memberDeg) {
  return manipMemberToServoDeg(memberDeg, armUpperServoZeroDeg, armUpperMemberZeroDeg, armUpperServoSign);
}

// Arm helper for fore member to servo degrees.
float armForeMemberToServoDeg(float memberDeg) {
  return manipMemberToServoDeg(memberDeg, armForeServoZeroDeg, armForeMemberZeroDeg, armForeServoSign);
}

// Arm helper for forearm roll member to servo degrees.
float armForearmRollMemberToServoDeg(float memberDeg) {
  return manipMemberToServoDeg(memberDeg, armForearmRollServoZeroDeg, armForearmRollMemberZeroDeg, armForearmRollServoSign);
}

// Arm helper for wrist pitch member to servo degrees.
float armWristPitchMemberToServoDeg(float memberDeg) {
  return manipMemberToServoDeg(memberDeg, armWristPitchServoZeroDeg, armWristPitchMemberZeroDeg, armWristPitchServoSign);
}

// Arm helper for wrist roll member to servo degrees.
float armWristRollMemberToServoDeg(float memberDeg) {
  return manipMemberToServoDeg(memberDeg, armWristRollServoZeroDeg, armWristRollMemberZeroDeg, armWristRollServoSign);
}

// Arm helper for grip member to servo degrees.
float armGripMemberToServoDeg(float memberDeg) {
  return clampValue(manipMemberToServoDeg(memberDeg, armGripServoZeroDeg, armGripMemberZeroDeg, armGripServoSign), 0.0f, MANIP_GRIPPER_MAX_DEG);
}

// Utility: map equivalent angle into range.
float mapEquivalentAngleIntoRange(float angleDeg, float minDeg, float maxDeg) {
  if (minDeg > maxDeg) {
    float tmp = minDeg;
    minDeg = maxDeg;
    maxDeg = tmp;
  }

  float best = angleDeg;
  bool found = false;
  float bestDelta = 1.0e9f;
  for (int8_t k = -2; k <= 2; k++) {
    float candidate = angleDeg + 360.0f * (float)k;
    if (candidate < (minDeg - 0.0001f) || candidate > (maxDeg + 0.0001f)) continue;
    float delta = fabsf(candidate - angleDeg);
    if (!found || delta < bestDelta) {
      best = candidate;
      bestDelta = delta;
      found = true;
    }
  }

  if (found) return best;
  return clampValue(angleDeg, minDeg, maxDeg);
}

// Converts manipulator member to servo degrees.
float convertManipulatorMemberToServoDeg(uint8_t memberIdx, float memberDeg) {
  uint8_t ch = getManipulatorMemberChannel(memberIdx);
  if (memberIdx == MANIP_MEMBER_BASE) {
    if (ch == 0) {
      // BASECH=0 uses the A0 potentiometer as an absolute linear 0..359 axis.
      // Keep the target direct; do not wrap equivalent angles for this path.
      return clampValue(memberDeg, MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);
    }
    return mapEquivalentAngleIntoRange(armBaseMemberToServoDeg(memberDeg), getServoMinLimit(ch), getServoMaxLimit(ch));
  }

  float rawServoDeg = memberDeg;
  switch (memberIdx) {
    case MANIP_MEMBER_UPPER: rawServoDeg = armUpperMemberToServoDeg(memberDeg); break;
    case MANIP_MEMBER_FORE: rawServoDeg = armForeMemberToServoDeg(memberDeg); break;
    case MANIP_MEMBER_FOREARM_ROLL: rawServoDeg = armForearmRollMemberToServoDeg(memberDeg); break;
    case MANIP_MEMBER_WRIST_PITCH: rawServoDeg = armWristPitchMemberToServoDeg(memberDeg); break;
    case MANIP_MEMBER_WRIST_ROLL: rawServoDeg = armWristRollMemberToServoDeg(memberDeg); break;
    case MANIP_MEMBER_GRIPPER: rawServoDeg = armGripMemberToServoDeg(memberDeg); break;
    default: break;
  }

  if (isServoChannel(ch)) {
    return mapEquivalentAngleIntoRange(rawServoDeg, getServoMinLimit(ch), getServoMaxLimit(ch));
  }
  return rawServoDeg;
}

// Converts manipulator servo to member degrees.
float convertManipulatorServoToMemberDeg(uint8_t memberIdx, float servoDeg) {
  switch (memberIdx) {
    case MANIP_MEMBER_BASE: return armBaseServoToForwardYawDeg(servoDeg);
    case MANIP_MEMBER_UPPER: return armUpperServoToGroundDeg(servoDeg);
    case MANIP_MEMBER_FORE: return armForeServoToRelativeDeg(servoDeg);
    case MANIP_MEMBER_FOREARM_ROLL: return armForearmRollServoToMemberDeg(servoDeg);
    case MANIP_MEMBER_WRIST_PITCH: return armWristServoToRelativeDeg(servoDeg);
    case MANIP_MEMBER_WRIST_ROLL: return armWristRollServoToMemberDeg(servoDeg);
    case MANIP_MEMBER_GRIPPER: return armGripServoToMemberDeg(servoDeg);
    default: return servoDeg;
  }
}

// Returns manipulator member actual degrees.
float getManipulatorMemberActualDeg(uint8_t memberIdx) {
  return convertManipulatorServoToMemberDeg(memberIdx, getManipulatorMemberActualServoDeg(memberIdx));
}

// Returns manipulator member target degrees.
float getManipulatorMemberTargetDeg(uint8_t memberIdx) {
  return convertManipulatorServoToMemberDeg(memberIdx, getManipulatorMemberTargetServoDeg(memberIdx));
}

// Arm helper for base servo to local y rotation degrees.
float armBaseServoToLocalYRotationDeg(float servoDeg) {
  // Cardinal reference only: servo angle/member yaw remain identical.
  // The physical/model front marker is on local +X, so +90° makes
  // yaw 180° point to South on the Processing cardinal plane.
  return wrapAbsoluteDeg(armBaseServoToForwardYawDeg(servoDeg) + 90.0f);
}

// Arm helper for upper servo to local z rotation degrees.
float armUpperServoToLocalZRotationDeg(float servoDeg) {
  // Mirror Processing exactly: 135 - logical servo angle.
  return 135.0f - servoDeg;
}

// Arm helper for fore servo to local z rotation degrees.
float armForeServoToLocalZRotationDeg(float servoDeg) {
  return 180.0f - servoDeg;
}

// Arm helper for forearm roll servo to local y rotation degrees.
float armForearmRollServoToLocalYRotationDeg(float servoDeg) {
  // Mirror Processing exactly: -(servo - 90).
  return -(servoDeg - 90.0f);
}

// Arm helper for wrist servo to local z rotation degrees.
float armWristServoToLocalZRotationDeg(float servoDeg) {
  return 90.0f - servoDeg;
}

// Arm helper for gripper finger local z rotation degrees.
float armGripServoToLocalFingerZDeg(float gripDeg) {
  gripDeg = clampValue(gripDeg, 0.0f, MANIP_GRIPPER_MAX_DEG);
  return 40.0f - (0.8f * gripDeg);
}

static const uint8_t ARM_MAX_COLLIDERS = 8;
static const float ARM_COLLISION_CLEARANCE_CM = 1.0f;
static const float ARM_COLLISION_MAX_CLEARANCE_CM = 5.0f;
// Arm geometry is now stored natively in centimeters.
static const float ARM_GEOMETRY_CM_TO_SCENE = 1.0f;

float armSceneToCm(float sceneValue) {
  return sceneValue / ARM_GEOMETRY_CM_TO_SCENE;
}

// Arm helper for cm to scene.
float armCmToScene(float cmValue) {
  return cmValue * ARM_GEOMETRY_CM_TO_SCENE;
}

static const float ARM_GROUND_COLLIDER_W = 264.0f;
static const float ARM_GROUND_COLLIDER_H = 1.0f;
static const float ARM_GROUND_COLLIDER_D = 264.0f;
static const float ARM_GROUND_Y = 0.0f;
static const float ARM_BASE_GAP = 0.0f;
static const float ARM_FINGER_PIVOT_HEIGHT_RATIO = 0.22857143f;
static const float ARM_FINGER_LATERAL_CLEARANCE_CM = 0.321f;
static const float VEHICLE_BODY_CENTER_EXTRA_DROP = 20.0f;

// Arm helper for identity transform.
ArmTransform armIdentityTransform() {
  ArmTransform t;
  t.p = ArmCollisionMath::v3(0, 0, 0);
  t.ux = ArmCollisionMath::v3(1, 0, 0);
  t.uy = ArmCollisionMath::v3(0, 1, 0);
  t.uz = ArmCollisionMath::v3(0, 0, 1);
  return t;
}

// Arm helper for translate local.
void armTranslateLocal(ArmTransform &t, float x, float y, float z) {
  t.p = ArmCollisionMath::add(t.p,
                              ArmCollisionMath::add(
                                ArmCollisionMath::mul(t.ux, x),
                                ArmCollisionMath::add(
                                  ArmCollisionMath::mul(t.uy, y),
                                  ArmCollisionMath::mul(t.uz, z))));
}

// Arm helper for rotate local z.
void armRotateLocalZ(ArmTransform &t, float deg) {
  float r = deg * 0.01745329252f;
  float c = cosf(r), s = sinf(r);
  RuntimeVec3 nx = ArmCollisionMath::add(ArmCollisionMath::mul(t.ux, c), ArmCollisionMath::mul(t.uy, s));
  RuntimeVec3 ny = ArmCollisionMath::add(ArmCollisionMath::mul(t.ux, -s), ArmCollisionMath::mul(t.uy, c));
  t.ux = nx;
  t.uy = ny;
}

// Arm helper for rotate local y.
void armRotateLocalY(ArmTransform &t, float deg) {
  float r = deg * 0.01745329252f;
  float c = cosf(r), s = sinf(r);
  RuntimeVec3 nx = ArmCollisionMath::add(ArmCollisionMath::mul(t.ux, c), ArmCollisionMath::mul(t.uz, -s));
  RuntimeVec3 nz = ArmCollisionMath::add(ArmCollisionMath::mul(t.ux, s), ArmCollisionMath::mul(t.uz, c));
  t.ux = nx;
  t.uz = nz;
}

// Arm helper for build OBB.
ArmOOBB armBuildOBB(const ArmTransform &t, float w, float h, float d, ArmColliderType type) {
  ArmOOBB b;
  b.type = type;
  b.c = t.p;
  b.ux = ArmCollisionMath::normalize(t.ux);
  b.uy = ArmCollisionMath::normalize(t.uy);
  b.uz = ArmCollisionMath::normalize(t.uz);
  b.ex = w * 0.5f;
  b.ey = h * 0.5f;
  b.ez = d * 0.5f;
  return b;
}

// Arm helper for OBB project radius on an already normalized axis.
float armOBBProjectRadiusUnit(const ArmOOBB &b, RuntimeVec3 axisUnit) {
  return b.ex * fabsf(ArmCollisionMath::dot(b.ux, axisUnit)) +
         b.ey * fabsf(ArmCollisionMath::dot(b.uy, axisUnit)) +
         b.ez * fabsf(ArmCollisionMath::dot(b.uz, axisUnit));
}

// Arm helper for OBB project radius.
float armOBBProjectRadius(const ArmOOBB &b, RuntimeVec3 axis) {
  axis = ArmCollisionMath::normalize(axis);
  return armOBBProjectRadiusUnit(b, axis);
}

// Arm helper for OBB overlap on axis.
bool armOBBOverlapOnAxis(const ArmOOBB &a, const ArmOOBB &b, RuntimeVec3 axis, float tol) {
  if (ArmCollisionMath::magSq(axis) < 1e-9f) return true;
  axis = ArmCollisionMath::normalize(axis);
  float dist = fabsf(ArmCollisionMath::dot(ArmCollisionMath::sub(b.c, a.c), axis));
  float ra = armOBBProjectRadiusUnit(a, axis);
  float rb = armOBBProjectRadiusUnit(b, axis);
  return dist <= (ra + rb + tol);
}

// Arm helper for intervals overlap.
bool armIntervalsOverlap(float aMin, float aMax, float bMin, float bMax, float tol) {
  // Positive tol expands the collision envelope; negative tol adds clearance.
  // This matches the OBB SAT test and lets the configured collision tolerance
  // behave as a mechanical clearance instead of making false positives worse.
  return (aMin <= bMax + tol) && (aMax >= bMin - tol);
}

float armCollisionSolverTolerance() {
  float clearance = clampValue(ARM_COLLISION_CLEARANCE_CM, 0.0f, ARM_COLLISION_MAX_CLEARANCE_CM);
  return -clearance;
}

// Arm helper for dot 2 XZ.
float armDot2XZ(RuntimeVec3 a, RuntimeVec3 b) {
  return a.x * b.x + a.z * b.z;
}

// Arm helper for clamp float.
float armClampFloat(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// Arm helper for cylinder OBB intersects.
bool armCylinderOBBIntersects(const ArmCylinderY &c, const ArmOOBB &b, float tol) {
  RuntimeVec3 Y = ArmCollisionMath::v3(0, 1, 0);
  float rY = armOBBProjectRadiusUnit(b, Y);
  float bMin = b.c.y - rY;
  float bMax = b.c.y + rY;
  if (!armIntervalsOverlap(c.yMin, c.yMax, bMin, bMax, tol)) return false;

  RuntimeVec3 ax = ArmCollisionMath::v3(b.ux.x, 0, b.ux.z);
  RuntimeVec3 az = ArmCollisionMath::v3(b.uz.x, 0, b.uz.z);
  float axLen = sqrtf(ax.x * ax.x + ax.z * ax.z);
  float azLen = sqrtf(az.x * az.x + az.z * az.z);

  if (axLen < 1e-6f && azLen < 1e-6f) {
    float dx = b.c.x - c.cx;
    float dz = b.c.z - c.cz;
    float rr = c.radius + tol;
    return (dx * dx + dz * dz) <= rr * rr;
  }

  if (axLen > 1e-6f) ax = ArmCollisionMath::mul(ax, 1.0f / axLen);
  else ax = ArmCollisionMath::v3(1, 0, 0);
  if (azLen > 1e-6f) az = ArmCollisionMath::mul(az, 1.0f / azLen);
  else az = ArmCollisionMath::v3(0, 0, 1);

  float hx = armOBBProjectRadiusUnit(b, ArmCollisionMath::v3(ax.x, 0, ax.z));
  float hz = armOBBProjectRadiusUnit(b, ArmCollisionMath::v3(az.x, 0, az.z));

  RuntimeVec3 d = ArmCollisionMath::v3(c.cx - b.c.x, 0, c.cz - b.c.z);
  float localX = armDot2XZ(d, ax);
  float localZ = armDot2XZ(d, az);
  float clampedX = armClampFloat(localX, -hx, hx);
  float clampedZ = armClampFloat(localZ, -hz, hz);

  float dx = localX - clampedX;
  float dz = localZ - clampedZ;
  float rr = c.radius + tol;
  return (dx * dx + dz * dz) <= rr * rr;
}

// Arm helper for OBB intersects OBB.
bool armOBBIntersectsOBB(const ArmOOBB &a, const ArmOOBB &b, float tol) {
  RuntimeVec3 axes[15] = {
    a.ux, a.uy, a.uz,
    b.ux, b.uy, b.uz,
    ArmCollisionMath::cross(a.ux, b.ux), ArmCollisionMath::cross(a.ux, b.uy), ArmCollisionMath::cross(a.ux, b.uz),
    ArmCollisionMath::cross(a.uy, b.ux), ArmCollisionMath::cross(a.uy, b.uy), ArmCollisionMath::cross(a.uy, b.uz),
    ArmCollisionMath::cross(a.uz, b.ux), ArmCollisionMath::cross(a.uz, b.uy), ArmCollisionMath::cross(a.uz, b.uz)
  };
  for (uint8_t i = 0; i < 15; i++) {
    if (!armOBBOverlapOnAxis(a, b, axes[i], tol)) return false;
  }
  return true;
}

// Arm helper for collider center.
RuntimeVec3 armColliderCenter(const ArmGenericCollider &c) {
  if (c.isCylinder) {
    return ArmCollisionMath::v3(c.cyl.cx, (c.cyl.yMin + c.cyl.yMax) * 0.5f, c.cyl.cz);
  }
  return c.obb.c;
}

// Arm helper for a loose no-false-negative broad-phase radius.
float armColliderLooseRadius(const ArmGenericCollider &c) {
  if (c.isCylinder) {
    float halfH = fabsf(c.cyl.yMax - c.cyl.yMin) * 0.5f;
    return c.cyl.radius + halfH;
  }
  return c.obb.ex + c.obb.ey + c.obb.ez;
}

// Arm helper for broad-phase rejection.
bool armColliderBroadPhaseMayIntersect(const ArmGenericCollider &a, const ArmGenericCollider &b, float tol) {
  RuntimeVec3 d = ArmCollisionMath::sub(armColliderCenter(b), armColliderCenter(a));
  float r = armColliderLooseRadius(a) + armColliderLooseRadius(b) + tol;
  if (r <= 0.0f) return false;
  return ArmCollisionMath::magSq(d) <= (r * r);
}

// Arm helper for the large fixed ground collider.  Ground collisions only need
// vertical overlap plus a simple X/Z bounds check, avoiding a full 15-axis OBB
// SAT test for every link on every solver pass.
bool armGroundIntersectsCollider(const ArmGenericCollider &ground, const ArmGenericCollider &other, float tol) {
  if (ground.isCylinder || ground.type != ARM_COLL_GROUND) return true;

  float gMinY = ground.obb.c.y - ground.obb.ey;
  float gMaxY = ground.obb.c.y + ground.obb.ey;
  float oMinY, oMaxY;

  if (other.isCylinder) {
    oMinY = other.cyl.yMin;
    oMaxY = other.cyl.yMax;
    if (!armIntervalsOverlap(gMinY, gMaxY, oMinY, oMaxY, tol)) return false;
    float gMinX = ground.obb.c.x - ground.obb.ex;
    float gMaxX = ground.obb.c.x + ground.obb.ex;
    float gMinZ = ground.obb.c.z - ground.obb.ez;
    float gMaxZ = ground.obb.c.z + ground.obb.ez;
    return armIntervalsOverlap(gMinX, gMaxX, other.cyl.cx - other.cyl.radius, other.cyl.cx + other.cyl.radius, tol) &&
           armIntervalsOverlap(gMinZ, gMaxZ, other.cyl.cz - other.cyl.radius, other.cyl.cz + other.cyl.radius, tol);
  }

  RuntimeVec3 Y = ArmCollisionMath::v3(0, 1, 0);
  RuntimeVec3 X = ArmCollisionMath::v3(1, 0, 0);
  RuntimeVec3 Z = ArmCollisionMath::v3(0, 0, 1);
  float rY = armOBBProjectRadiusUnit(other.obb, Y);
  oMinY = other.obb.c.y - rY;
  oMaxY = other.obb.c.y + rY;
  if (!armIntervalsOverlap(gMinY, gMaxY, oMinY, oMaxY, tol)) return false;

  float rX = armOBBProjectRadiusUnit(other.obb, X);
  float rZ = armOBBProjectRadiusUnit(other.obb, Z);
  return armIntervalsOverlap(ground.obb.c.x - ground.obb.ex, ground.obb.c.x + ground.obb.ex, other.obb.c.x - rX, other.obb.c.x + rX, tol) &&
         armIntervalsOverlap(ground.obb.c.z - ground.obb.ez, ground.obb.c.z + ground.obb.ez, other.obb.c.z - rZ, other.obb.c.z + rZ, tol);
}

// Arm helper for collider intersects.
bool armColliderIntersects(const ArmGenericCollider &a, const ArmGenericCollider &b, float tol) {
  if (a.type == ARM_COLL_GROUND) return armGroundIntersectsCollider(a, b, tol);
  if (b.type == ARM_COLL_GROUND) return armGroundIntersectsCollider(b, a, tol);

  if (!armColliderBroadPhaseMayIntersect(a, b, tol)) return false;

  if (a.isCylinder && b.isCylinder) {
    bool yOverlap = armIntervalsOverlap(a.cyl.yMin, a.cyl.yMax, b.cyl.yMin, b.cyl.yMax, tol);
    if (!yOverlap) return false;
    float dx = a.cyl.cx - b.cyl.cx;
    float dz = a.cyl.cz - b.cyl.cz;
    float r = a.cyl.radius + b.cyl.radius + tol;
    if (r <= 0.0f) return false;
    return (dx * dx + dz * dz) <= r * r;
  }
  if (!a.isCylinder && !b.isCylinder) {
    return armOBBIntersectsOBB(a.obb, b.obb, tol);
  }
  if (a.isCylinder) return armCylinderOBBIntersects(a.cyl, b.obb, tol);
  return armCylinderOBBIntersects(b.cyl, a.obb, tol);
}

// Arm helper for are adjacent manipulator collider types.
bool armAreAdjacentManipulatorColliderTypes(ArmColliderType a, ArmColliderType b) {
  return
    (a == ARM_COLL_GROUND && b == ARM_COLL_BASE_BLOCK) ||
    (a == ARM_COLL_BASE_BLOCK && b == ARM_COLL_GROUND) ||
    (a == ARM_COLL_BASE_CYLINDER && b == ARM_COLL_BASE_BLOCK) ||
    (a == ARM_COLL_BASE_BLOCK && b == ARM_COLL_BASE_CYLINDER) ||
    (a == ARM_COLL_BASE_BLOCK && b == ARM_COLL_UPPER_ARM) ||
    (a == ARM_COLL_UPPER_ARM && b == ARM_COLL_BASE_BLOCK) ||
    (a == ARM_COLL_UPPER_ARM && b == ARM_COLL_FOREARM) ||
    (a == ARM_COLL_FOREARM && b == ARM_COLL_UPPER_ARM) ||
    (a == ARM_COLL_FOREARM && b == ARM_COLL_WRIST_VERTICAL) ||
    (a == ARM_COLL_WRIST_VERTICAL && b == ARM_COLL_FOREARM) ||
    (a == ARM_COLL_WRIST_VERTICAL && b == ARM_COLL_FINGER_R) ||
    (a == ARM_COLL_FINGER_R && b == ARM_COLL_WRIST_VERTICAL) ||
    (a == ARM_COLL_WRIST_VERTICAL && b == ARM_COLL_FINGER_L) ||
    (a == ARM_COLL_FINGER_L && b == ARM_COLL_WRIST_VERTICAL);
}

// Arm helper for should ignore collision pair.
bool armShouldIgnoreCollisionPair(ArmColliderType a, ArmColliderType b) {
  // Match the current Processing collision rules exactly.
  if ((a == ARM_COLL_FINGER_R && b == ARM_COLL_FINGER_L) ||
      (a == ARM_COLL_FINGER_L && b == ARM_COLL_FINGER_R)) return true;
  if ((a == ARM_COLL_GROUND && b == ARM_COLL_BASE_CYLINDER) ||
      (a == ARM_COLL_BASE_CYLINDER && b == ARM_COLL_GROUND)) return true;
  return armAreAdjacentManipulatorColliderTypes(a, b);
}

// Arm helper for collect pose colliders.
uint8_t armCollectPoseColliders(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg, ArmGenericCollider outCols[]) {
  uint8_t count = 0;

  outCols[count].isCylinder = false;
  outCols[count].type = ARM_COLL_GROUND;
  ArmTransform groundT = armIdentityTransform();
  armTranslateLocal(groundT, 0, ARM_GROUND_Y + ARM_GROUND_COLLIDER_H * 0.5f, 0);
  outCols[count].obb = armBuildOBB(groundT, ARM_GROUND_COLLIDER_W, ARM_GROUND_COLLIDER_H, ARM_GROUND_COLLIDER_D, ARM_COLL_GROUND);
  count++;

  outCols[count].isCylinder = true;
  outCols[count].type = ARM_COLL_BASE_CYLINDER;
  outCols[count].cyl.type = ARM_COLL_BASE_CYLINDER;
  outCols[count].cyl.cx = 0.0f;
  outCols[count].cyl.cz = 0.0f;
  outCols[count].cyl.radius = armBaseRadius;
  outCols[count].cyl.yMin = armBaseCylinderYOffset - (armBaseHeight * 0.5f);
  outCols[count].cyl.yMax = armBaseCylinderYOffset + (armBaseHeight * 0.5f);
  count++;

  const float upperArmYOffset = -armUpperH * 0.5f;
  const float forearmYOffset = -armForeH * 0.5f;
  const float wristVerticalYOffset = -armWristH * 0.5f;
  const float gripperYOffset = armGripperYOffset;
  const float fingerXOffset = armFingerLateralOffset();
  const float gripperFingerCenterYOffset = -armFingerPivotOffset();

  const float baseBlockColliderW = armBaseBlockW;
  const float baseBlockColliderH = armBaseBlockH;
  const float baseBlockColliderD = armBaseBlockD;
  const float upperArmColliderW = armUpperW;
  const float upperArmColliderH = armUpperH;
  const float upperArmColliderD = armUpperD;
  const float forearmColliderW = armForeW;
  const float forearmColliderH = armForeH;
  const float forearmColliderD = armForeD;
  const float wristVerticalColliderW = armWristW;
  const float wristVerticalColliderH = armWristH;
  const float wristVerticalColliderD = armWristD;
  const float gripperFingerColliderW = armFingerW;
  const float gripperFingerColliderH = armFingerH;
  const float gripperFingerColliderD = armFingerD;

  ArmTransform baseM = armIdentityTransform();
  armTranslateLocal(baseM, 0, armBaseBlockYOffset, 0);
  armRotateLocalY(baseM, armBaseServoToLocalYRotationDeg(baseDeg));
  outCols[count].isCylinder = false;
  outCols[count].type = ARM_COLL_BASE_BLOCK;
  outCols[count].obb = armBuildOBB(baseM, baseBlockColliderW, baseBlockColliderH, baseBlockColliderD, ARM_COLL_BASE_BLOCK);
  count++;

  ArmTransform shoulderJointM = baseM;
  armTranslateLocal(shoulderJointM, 0, -(armBaseBlockH * 0.5f), 0);

  ArmTransform upperArmM = shoulderJointM;
  armRotateLocalZ(upperArmM, armUpperServoToLocalZRotationDeg(upperDeg));
  armTranslateLocal(upperArmM, 0, upperArmYOffset, 0);
  outCols[count].isCylinder = false;
  outCols[count].type = ARM_COLL_UPPER_ARM;
  outCols[count].obb = armBuildOBB(upperArmM, upperArmColliderW, upperArmColliderH, upperArmColliderD, ARM_COLL_UPPER_ARM);
  count++;

  ArmTransform elbowJointM = upperArmM;
  armTranslateLocal(elbowJointM, 0, -(armUpperH * 0.5f), 0);

  ArmTransform forearmM = elbowJointM;
  armRotateLocalZ(forearmM, armForeServoToLocalZRotationDeg(foreDeg));
  armTranslateLocal(forearmM, 0, forearmYOffset, 0);
  outCols[count].isCylinder = false;
  outCols[count].type = ARM_COLL_FOREARM;
  outCols[count].obb = armBuildOBB(forearmM, forearmColliderW, forearmColliderH, forearmColliderD, ARM_COLL_FOREARM);
  count++;

  ArmTransform wristJointM = forearmM;
  armTranslateLocal(wristJointM, 0, -(armForeH * 0.5f), 0);
  armRotateLocalY(wristJointM, armForearmRollServoToLocalYRotationDeg(forearmRollDeg));

  ArmTransform wristVerticalM = wristJointM;
  armRotateLocalZ(wristVerticalM, armWristServoToLocalZRotationDeg(wristPitchDeg));
  armTranslateLocal(wristVerticalM, 0, wristVerticalYOffset, 0);
  outCols[count].isCylinder = false;
  outCols[count].type = ARM_COLL_WRIST_VERTICAL;
  outCols[count].obb = armBuildOBB(wristVerticalM, wristVerticalColliderW, wristVerticalColliderH, wristVerticalColliderD, ARM_COLL_WRIST_VERTICAL);
  count++;

  ArmTransform wristRotateJointM = wristVerticalM;
  armTranslateLocal(wristRotateJointM, 0, -(armWristH * 0.5f), 0);
  armRotateLocalY(wristRotateJointM, -(wristRotDeg - 90.0f));

  ArmTransform gripperBaseM = wristRotateJointM;
  armTranslateLocal(gripperBaseM, 0, gripperYOffset, 0);

  ArmTransform fingerRM = gripperBaseM;
  armTranslateLocal(fingerRM, -fingerXOffset, 0, 0);
  armRotateLocalZ(fingerRM, -armGripServoToLocalFingerZDeg(gripDeg));
  armTranslateLocal(fingerRM, 0, gripperFingerCenterYOffset, 0);
  outCols[count].isCylinder = false;
  outCols[count].type = ARM_COLL_FINGER_R;
  outCols[count].obb = armBuildOBB(fingerRM, gripperFingerColliderW, gripperFingerColliderH, gripperFingerColliderD, ARM_COLL_FINGER_R);
  count++;

  ArmTransform fingerLM = gripperBaseM;
  armTranslateLocal(fingerLM, fingerXOffset, 0, 0);
  armRotateLocalZ(fingerLM, +armGripServoToLocalFingerZDeg(gripDeg));
  armTranslateLocal(fingerLM, 0, gripperFingerCenterYOffset, 0);
  outCols[count].isCylinder = false;
  outCols[count].type = ARM_COLL_FINGER_L;
  outCols[count].obb = armBuildOBB(fingerLM, gripperFingerColliderW, gripperFingerColliderH, gripperFingerColliderD, ARM_COLL_FINGER_L);
  count++;

  return count;
}

// Arm helper for pose has OOBB collision.
bool armPoseHasOOBBCollision(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg) {
  ArmGenericCollider cols[ARM_MAX_COLLIDERS];
  uint8_t count = armCollectPoseColliders(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg, cols);
  for (uint8_t i = 0; i < count; i++) {
    for (uint8_t j = i + 1; j < count; j++) {
      if (armShouldIgnoreCollisionPair(cols[i].type, cols[j].type)) continue;
      if (armColliderIntersects(cols[i], cols[j], armCollisionSolverTolerance())) return true;
    }
  }
  return false;
}

bool servoAngleWithinConfiguredLimits(uint8_t ch, float value, float tol = 0.0f);
bool poseViolatesConfiguredLimits(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg);
float clampServoTargetToLimits(uint8_t ch, float value);

// Utility: pose collision severity.
uint8_t poseCollisionSeverity(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg) {
  if (poseViolatesConfiguredLimits(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg)) {
    return 0xFFu;
  }

  uint8_t baseCh = getManipulatorMemberChannel(MANIP_MEMBER_BASE);
  baseDeg = (baseCh == 0)
            ? clampValue(baseDeg, MOTOR_INPUT_MIN, MOTOR_INPUT_MAX)
            : clampServoTargetToLimits(baseCh, clampServoInputToChannelRange(baseCh, baseDeg));
  upperDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_UPPER), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_UPPER), upperDeg));
  foreDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_FORE), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_FORE), foreDeg));
  forearmRollDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_FOREARM_ROLL), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_FOREARM_ROLL), forearmRollDeg));
  wristPitchDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_PITCH), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_PITCH), wristPitchDeg));
  wristRotDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_ROLL), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_WRIST_ROLL), wristRotDeg));
  gripDeg = clampServoTargetToLimits(getManipulatorMemberChannel(MANIP_MEMBER_GRIPPER), clampServoInputToChannelRange(getManipulatorMemberChannel(MANIP_MEMBER_GRIPPER), gripDeg));

  ArmGenericCollider cols[ARM_MAX_COLLIDERS];
  uint8_t count = armCollectPoseColliders(baseDeg, upperDeg, foreDeg, forearmRollDeg, wristPitchDeg, wristRotDeg, gripDeg, cols);
  uint8_t collisions = 0;
  for (uint8_t i = 0; i < count; i++) {
    for (uint8_t j = i + 1; j < count; j++) {
      if (armShouldIgnoreCollisionPair(cols[i].type, cols[j].type)) continue;
      if (armColliderIntersects(cols[i], cols[j], armCollisionSolverTolerance())) {
        if (collisions < 0xFEu) collisions++;
      }
    }
  }
  return collisions;
}

// ---------------- Prototypes ----------------
uint32_t calculateCRC32(const uint8_t* data, size_t len);
uint32_t calculateConfigCRC(const PersistentConfig& cfg);
uint32_t calculateConfigExtrasCRC(const PersistentConfigExtras& cfg);
void fillDefaultConfigExtras(PersistentConfigExtras& cfg);
void applyConfigExtrasToRuntime(const PersistentConfigExtras& cfg);
void syncRuntimeToConfigExtras(PersistentConfigExtras& cfg);
bool loadConfigExtrasFromEEPROM();
void saveConfigExtrasToEEPROM();
void fillDefaultManipulatorHomePose(int16_t pose[MANIP_MEMBER_COUNT]);
void sanitizeManipulatorHomePose(int16_t pose[MANIP_MEMBER_COUNT]);
void sanitizeVehicleProcessingProfile(VehicleProcessingProfile &cfg);
void sanitizeDroneProcessingProfile(DroneProcessingProfile &cfg);
void clearAllNeutralStates();
void initRuntimeState();
void initServoBank();
void initMotorState();
void setupHighFreqPwmPin46();
bool isMotorPwmTimerHealthy();
void ensureMotorPwmTimerReady();
void configureFallbackOutputs();
void attachFallbackServos();
void detachFallbackServos();

void releaseMotorCoils();
uint8_t getDcMotorPwmMirrorPercent();
uint16_t pwmPercentToTimer5Duty(uint8_t percent);
uint16_t getDcMotorMirrorPwmDutyFromCh12();
void mirrorPwm46FromCh12ForDcMotor();
void motorSetCoils(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
void motorSetDriverPWM(uint16_t pwmVal);
uint8_t getMotorPhaseCount();
void applyStepPhase(MotorSequence seq);
void applyStepOutput(MotorSequence seq, uint16_t pwmVal);
void performStep(bool forward, uint16_t pwmVal);
void accelStepForward();
void accelStepBackward();
float readMotorAngle();
void resetMotorControlState();
float getMotorRawSpan();
float getMotorDegPerCount();
void getMotorAdaptiveBands(float &holdBand, float &enterBand, float &exitBand, float &nearBand, float &farBand);
void sanitizeMotorRuntimeParams();

uint16_t angleToExternalPwmTicks(uint8_t ch, float angleDeg);
void writeChannel(uint8_t ch, float value);
void forceServoNeutralOutput(uint8_t ch);
void updateServosSmooth();
void applyBootPwmDefaults();

void fillDefaultConfig(PersistentConfig& cfg);
void applyConfigToRuntime(const PersistentConfig& cfg);
void syncRuntimeToConfig(PersistentConfig& cfg);
uint8_t getMotorPhaseCount();
void applyStepPhase(MotorSequence seq);
const __FlashStringHelper* getMotorDriveModeName(MotorDriveMode mode);
MotorDriveMode decodeStoredMotorDriveMode(uint8_t rawMode);
MotorDriveMode sanitizeMotorDriveMode(uint8_t rawMode);
bool parseMotorDriveModeValue(const char* value, MotorDriveMode* outMode);
void setMotorDriveMode(MotorDriveMode mode, bool announce);
const __FlashStringHelper* getMotorTypeName(MotorType mode);
MotorType sanitizeMotorType(uint8_t rawMode);
bool parseMotorTypeValue(const char* value, MotorType* outMode);
void setMotorType(MotorType mode, bool announce);
uint8_t encodeControlModesForStore(MotorDriveMode motorMode, MotorType motorType);
MotorDriveMode decodeStoredMotorDriveMode(uint8_t rawMode);
MotorType decodeStoredMotorType(uint8_t rawMode);
RobotControlMode sanitizeRobotControlMode(uint8_t rawMode);
const __FlashStringHelper* getRobotControlModeName(RobotControlMode mode);
bool parseRobotControlModeValue(const char* value, RobotControlMode* outMode);
uint8_t sanitizeManipulatorMemberChannel(uint8_t memberIdx, uint8_t ch);
void sanitizeManipulatorProfile();
bool setManipulatorMemberChannel(uint8_t memberIdx, uint8_t ch);
bool setManipulatorBaseChannelFlexible(uint8_t ch);
uint8_t getManipulatorMemberChannel(uint8_t memberIdx);
int8_t getManipulatorMemberIndexForChannel(uint8_t ch);
float getManipulatorMemberMinLimit(uint8_t memberIdx);
float getManipulatorMemberMaxLimit(uint8_t memberIdx);
float clampManipulatorMemberTargetToLimits(uint8_t memberIdx, float value);
bool manipulatorMemberAngleWithinConfiguredLimits(uint8_t memberIdx, float value, float tol = 0.0f);
float getManipulatorMemberActualServoDeg(uint8_t memberIdx);
float getManipulatorMemberTargetServoDeg(uint8_t memberIdx);
bool baseTelemetryUsesPotentiometer();
void releaseInactiveManipulatorChannels();
void prepareManipulatorHardwareForActiveProfile();
void applyRobotControlMode(RobotControlMode mode, bool announce);
bool switchRobotModeForRuntime(RobotControlMode mode, bool announce, bool sendSnapshot);
void loadRobotControlModeFromEEPROM();
void saveRobotControlModeToEEPROM();
void resetGenericRobotRuntimeState();
void updateGenericRobotRuntime();
bool handleVehicleRuntimeCommand(char* cmd);
bool handleDroneRuntimeCommand(char* cmd);
void configureRobotModeHardware();
void releaseVehicleHardware();
void releaseDroneHardware();
void updateVehicleHardwareOutputs();
void updateDroneHardwareOutputs();
void updateDirectRobotHardwareOutputs();
void printRobotControlModeStatus();
void printRobotSpecificConfigSummary();
static void printBootBanner();
static void printBootSummary();
void restartBootHandshakeScreen();
void sendStaticTelemetryConfigPacket();
bool loadConfigFromEEPROM();
void saveConfigToEEPROM();
void eraseEEPROMRegion();
void rebindI2CDevices();
void startCompassInitialization();
void updateCompassInitialization();
bool isCompassInitializationBusy();
void updateDirectMotorTest();
void configureCh0PinsForCurrentMode();
bool detectAndInitCompass();
bool initHMC5883(uint8_t addr);
bool initQMC5883(uint8_t addr);
bool readCompassRaw(int16_t &mx, int16_t &my, int16_t &mz);
float computeCompassHeadingDeg(int16_t mx, int16_t my);
bool unwrapStreamCommand(char* cmd);
void recordVehicleLinkHoldMotion();
void recordDroneLinkHoldMotion();
void clearRuntimeLinkHoldReturnState();
void enforceRuntimeStreamFailSafe();
void loadCompassCalibrationFromEEPROM();
void saveCompassCalibrationToEEPROM();
void resetCompassCalibrationData();
bool parseCompassCalibrationConfigValue(const char* text, CompassCalibrationData& out);
void startCompassCalibration();
void cancelCompassCalibration(bool saveResult);
void updateCompassCalibration();
void setManipulatorCompassCalibrationPose(uint8_t index);

void printBootHelp();
void printCurrentConfig();
void printChannelConfig(uint8_t ch);
void printCompassCalibrationPanel();

bool parseChannelValueCommand(const char* text, const char* prefix, uint8_t& ch, float& value);
bool requireReadyForRuntimeCommands();
bool allowPrimaryAsciiStatusMessages();
void applyServoValue(uint8_t ch, float value, ValueMode mode);
void handleNumericCommand(const char* prefix, char* cmd, ValueMode mode);
void handleDirectionCommand(char* cmd);
bool applyRuntimeNeutralState(uint8_t ch, bool enabled);
bool applyRuntimePwmValue(uint8_t ch, float value);
bool applyRuntimeServoTarget(uint8_t ch, float value);
void canonicalizeCommandText(char* cmd);
bool isImmediateConfigCommand(const char* cmd);
bool isKnownConfigAssignmentCommand(const char* cmd);
bool isNumericCalibrationCommand(const char* cmd);
bool isRobotRuntimeCommandText(const char* cmd);
bool shouldRouteToConfigCommand(const char* cmd);
void handleConfigCommand(char* cmd);
void enterOperationMode();
void handleCommand(char* cmd);

void processSerial();
void sendHexFrame(uint8_t type, const uint8_t* payload, uint16_t len);
void initGpsSerial1();
void pollGpsSerial1();
void sendTelemetryHex();
void sendGpsTelemetryHex(boolean force);
void maybeSendGpsTelemetryHex();
void sendHexRuntimeAckOk();
void sendRuntimeTelemetryHeartbeatHex();
void armRuntimeHexHandshakeGuard();
bool isRuntimeHexHandshakeGuardActive();
void maybeSendRuntimeHexHandshakeHeartbeat();
void sendMinimalHexRuntimeSnapshot();
void sendHexBootSnapshot();
void handleSerial0HexFrame(uint8_t type, const uint8_t* payload, uint16_t len);
uint32_t currentTelemetryStreamSequence();
void enterOperationModeSilent();
void initAutoMotorState();
void initMotorCalState();
int16_t readMotorRawADC();
void startMotorCalibrationWizard();
void captureMotorCalibrationLimit1();
void captureMotorCalibrationLimit2();
void handleCallimitMotorCommand();
void printMotorCalibrationStatus();
MotorDriveMode getMotorModeAttemptByIndex(MotorDriveMode baseMode, uint8_t attemptIndex);
bool autoMotorAdvanceModeAttempt();
float getAutoMotorSafeMinAngle();
float getAutoMotorSafeMaxAngle();
void autoMotorApplyCandidate(const AutoMotorCandidate& c);
void autoMotorCaptureSeed();
void autoMotorStart();
void autoMotorStop(bool userRequested);
void autoMotorPrepareStage(AutoMotorStage stage);
uint8_t autoMotorBuildCandidates(AutoMotorStage stage, AutoMotorCandidate* out, uint8_t maxCount);
void autoMotorStartMotion(float deltaDeg);
void autoMotorFinishMotion(bool success);
void autoMotorFinalize();
void autoMotorUpdate();
void printMotorDiagnosticPanel();
bool handleMotorDiagnosticCommand(char* cmd);
void runDirectMotorTest(bool forward, uint16_t steps, uint16_t pwmPercent, unsigned long delayUs);
void printAutoMotorStatus();
void updateMotorControl();

float getServoMinLimit(uint8_t ch);
float getServoMaxLimit(uint8_t ch);
bool servoAngleWithinConfiguredLimits(uint8_t ch, float value, float tol = 0.0f);
bool poseViolatesConfiguredLimits(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg);
float clampServoTargetToLimits(uint8_t ch, float value);
float getLimitedServoTarget(uint8_t ch);
float getStructureReachFactor();
float getStructureMassFactor();
bool trySetStabilizedTarget(uint8_t ch, float desired);
void processIncomingStream(Stream& port, char* buffer, uint8_t bufferSize, uint8_t& index, uint8_t sourceId);
void updateOptimizePowerForServo(uint8_t ch, float prevActual, float newActual);
bool poseHasCollision(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg);
bool collisionGuardAllowsCandidatePose(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg);
bool candidateMoveAllowed(uint8_t ch, float candidate);
void armCollisionRecoveryLatch(uint8_t severity);
void applyCollisionRecoveryTargetsToLastSafePose();
bool applyManipulatorPoseRuntimeMemberValues(const int memberVals[7]);
bool applyManipulatorPoseRuntimeTargets(const int servoVals[7]);
bool applyManipulatorPoseRuntimeTargets(const int servoVals[7], const bool neutralMask[7]);
uint8_t poseCollisionSeverity(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg);
void cancelManipulatorFirmwareHome();
float firmwareHomeMemberDeltaDeg(uint8_t memberIdx, float currentDeg, float targetDeg);
float smoothManipulatorFirmwareHomeTarget(uint8_t memberIdx, float currentDeg, float targetDeg, float factor);
bool manipulatorFirmwareHomePoseClose(float toleranceDeg);
bool serviceManipulatorFirmwareHome(bool force);
bool applyManipulatorRuntimeHomePose();
void emitCollisionBlockedNotice();
void invalidateCollisionPoseCache();
bool captureLimitedActualCollisionPose(float& baseDeg, float& upperDeg, float& foreDeg, float& forearmRollDeg, float& wristPitchDeg, float& wristRotDeg, float& gripDeg);
void storeCollisionPoseCache(CollisionPoseCache& cache, float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg);
bool collisionPoseMatchesCache(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg, const CollisionPoseCache& cache, int16_t epsCenti);
bool collisionPoseChanged(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg);
uint8_t poseCollisionSeverityCached(float baseDeg, float upperDeg, float foreDeg, float forearmRollDeg, float wristPitchDeg, float wristRotDeg, float gripDeg, bool forceRefresh);
float armFingerPivotOffset();
float armFingerLateralOffset();
void recomputeManipulatorOffsetsFromProcessingModel();
void recomputeProcessingRobotDerivedOffsets();
void applyCollisionGuard();
void getCurrentPwmPercents(uint8_t outPerc[4]);
float estimatePitchDeg(int16_t ax, int16_t ay, int16_t az);
float estimateRollDeg(int16_t ax, int16_t ay, int16_t az);
void updateWristGimbalRuntime();
void updateGripPressureModel();
bool manipulatorBaseMotionQuietingServos();
void updateArmStabilization();

int readSonarDistanceCM();
uint16_t readBatteryAdcRaw();
float batteryPercentFromAdc(uint16_t raw);
float computeDroneTiltCompensatedSonarCm(int sonarCm, float pitchDeg, float rollDeg);
void updateDroneHeightEstimateFromSonar(int sonarCm, float pitchDeg, float rollDeg);
float readDroneAutoAltitudeCm();
float computeDroneAutoThrottlePct(float measuredAltCm);
void sendTelemetry();
uint8_t getDefaultManipulatorMemberChannel(uint8_t memberIdx);
float getManipulatorMemberStartupSafeDeg(uint8_t memberIdx);
void applySafeStartupPose();
void ensureSafeStartupPoseIfColliding();

// ---------------------------------------------------------------------
// Forward declarations added for single-translation-unit include layout.
// These functions are defined in later included implementation headers
// but are used earlier in the build order, so explicit prototypes are
// required when the firmware is assembled through #include-based modules.
// ---------------------------------------------------------------------
void releaseAllManipulatorActuators();
void noteRuntimeLinkActivity(uint32_t seq);
void applyConnectionHomePoseIfNeeded();
void disarmRuntimeOutputsAfterLinkTimeout();
void markProcessingProfilesDirty();
void syncProcessingRobotProfiles();
bool manipulatorMemberChannelIsUnique(uint8_t memberIdx, uint8_t ch);
bool isManipulatorMemberChannelAssigned(uint8_t ch);
bool parseManipulatorArmjValues(const char* poseText, int jointVals[7], int pwmVals[4], bool &hasPwm);
bool handleManipulatorRuntimeCommand(char* cmd);
bool parseSignedPair(const char* text, int &a, int &b);

void printRobotSpecificRuntimeExamples();
void printVehicleConfigPanels();
void printDroneConfigPanels();
void printManipulatorGeometryPanel();
void printI2CConfigPanel();
void printServoChannelsConfigPanel();
void printPwmBootConfigPanel();
void printMotorConfigPanel();
void printSerialRuntimePanel();
void printStabilizationPressureCollisionPanel();
void printActiveRobotRuntimeStateSummary();
bool runtimeReadyForCommands();
bool runtimeAsciiTelemetryOnly();
void clearConfigSessionDirty();
void noteConfigSessionEdited(bool showConfigNow);

void dcMotorStop();
void dcMotorDrive(bool forward, uint16_t pwmVal);
uint16_t angleToFallbackServoMicros(uint8_t ch, float angleDeg);
static bool i2cAddressValid(uint8_t addr);
static void i2cClearTimeoutFlag();
static bool i2cTimedOut();
static void recoverI2CBusPins();
static bool i2cBusLooksIdle();
static bool i2cPrepareBus();
static bool i2cTransactionOk(uint8_t wireErr);
static bool i2cDevicePresent(uint8_t addr);
static bool i2cPrepareDevice(uint8_t addr);
static bool i2cCompleteDeviceInit(bool initOk);
static bool i2cWrite8Checked(uint8_t addr, uint8_t reg, uint8_t val);
static void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val);
static bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t& val);
static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);
static bool i2cWriteBlockChecked(uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len);
static bool initExternalPwmControllerDirect(uint8_t addr);
static bool externalPwmSetPWM(uint8_t ch, uint16_t on, uint16_t off);
static void externalPwmFailoverToFallback();
static void advanceCompassCandidate();
static uint8_t binaryChecksum8(uint8_t type, uint16_t len, const uint8_t* payload);
static void pushU16(uint8_t* out, uint16_t& idx, uint16_t v);
static void pushI16(uint8_t* out, uint16_t& idx, int16_t v);
static void pushU32(uint8_t* out, uint16_t& idx, uint32_t v);
void serviceActiveMotorMotion(unsigned long nowUs);

uint32_t calculateSafetyConfigCRC(const PersistentSafetyConfig& cfg);
void fillDefaultSafetyConfig(PersistentSafetyConfig& cfg);
void applySafetyConfigToRuntime(const PersistentSafetyConfig& cfg);
void syncRuntimeToSafetyConfig(PersistentSafetyConfig& cfg);
bool loadSafetyConfigFromEEPROM();
void saveSafetyConfigToEEPROM();
bool captureDroneHomeFromGps();
bool captureVehicleHomeFromGps();
void resetSafetyAutonomyRuntimeState();
void updateSafetyAutonomy();
void updateVehicleInclinePowerRuntime();
void applyVehicleInclinePowerAssist(int &leftTargetPct, int &rightTargetPct);
float currentBatteryPercent();
