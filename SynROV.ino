#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <MPU6050_6Axis_MotionApps20.h>

// ***************************************************************
// --- 1. HARDWARE DEFINITIONS AND GLOBAL CONSTANTS ---
// ***************************************************************

// I2C Addresses
const uint8_t PCA9685_ADDR = 0x50;
const uint8_t INA219_ADDR = 0x41;

// Stepper Motor Pins (L298N)
const uint8_t STEPPER_IN_PINS[] = {2, 3, 5, 6}; // {IN1, IN2, IN3, IN4}

// Sensor Pins
const uint8_t STEPPER_FEEDBACK_PIN = A0;
const uint8_t SONAR_TRIG_PIN = 7;
const uint8_t SONAR_ECHO_PIN = 8;

// Instance Objects
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
Adafruit_INA219 ina219(INA219_ADDR);
MPU6050 mpu6050;

// Communication & Timing
const unsigned long SERIAL_BAUD_RATE = 115200;
const unsigned long I2C_CLOCK_SPEED = 400000;
const unsigned long SENSOR_INTERVAL_MS = 20;
const unsigned long PID_INTERVAL_MS = 10;
const unsigned long SERVO_UPDATE_INTERVAL_MS = 5;

// ***************************************************************
// --- 2. SERVO AND PCA9685 CONFIGURATION ---
// ***************************************************************

const uint8_t MAX_SERVOS = 16;
const uint16_t SERVO_MIN_PULSE = 102;
const uint16_t SERVO_MAX_PULSE = 505;
const float SERVO_MOVE_STEP = 0.2;

// Servo State Variables
uint8_t currentAngles[MAX_SERVOS] = {255}; // Target Angle (int)
float actualFloatAngle[MAX_SERVOS] = {90.0}; // Current position (float)
int8_t servoDirection[MAX_SERVOS] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

// Calibration
int16_t servoOffsets[MAX_SERVOS] = {0};
float servoSpan[MAX_SERVOS] = {1.0, 0.93, 0.90, 1.20, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

// Ramping Flag
bool servoRampingEnabled[MAX_SERVOS] = {
    false, // CH 0: Stepper - Ignored
    false, // CH 1: Servo 1
    false, // CH 2: Servo 2
    false, false, false, false, false, false, false, false, false,
    false, // CH 12: Duty Cycle - Ignored
    false, false, false,
};

// ***************************************************************
// --- 3. STEPPER/PID CONTROL (CH 0) ---
// ***************************************************************

// Limits and Calibration
const uint8_t STEPPER_MAX_ANGLE = 180;
const uint8_t STEPPER_TOLERANCE = 3;
const int16_t STEPPER_CALIBRATION_ZERO_ADC = 360;
const int STEPPER_MAX_ANGLE_SPAN_RAW = 430;

// PID Gains
float Kp = 3.0;
float Ki = 0.4;
float Kd = 0.0;
const float PID_DELTA_T = PID_INTERVAL_MS / 1000.0;

// Motor PWM Constants
const uint8_t IDLE_PWM_HOLD = 60;
const uint8_t ACTIVE_PWM_MIN = 220;
const unsigned long IDLE_DELAY_MS = 500;
const unsigned long MIN_STEP_DELAY_US = 100;

// Control and PID Variables
int stepper_target_angle = 0;
int current_stepper_angle = 0;
uint8_t currentPWMValue = IDLE_PWM_HOLD;
float integralError = 0.0;
float lastError = 0.0;
bool isStabilized = false;
unsigned long lastPIDTime = 0;
unsigned long stabilizationStartTime = 0;
unsigned long lastStepTime = 0;

enum StepperSequence {
    PHASE_0 = 0,
    PHASE_1 = 1,
    PHASE_2 = 2,
    PHASE_3 = 3
};
StepperSequence currentStepSequence = PHASE_0;

// EMA Filter
const float EMA_ALPHA = 0.9;
float filtered_pot_value = 0.0;

// ***************************************************************
// --- 4. SYSTEM STATE VARIABLES ---
// ***************************************************************

bool pwmReady = false;
bool inaReady = false;
bool mpuReady = false;
bool systemActive = false;
unsigned long lastSensorTime = 0;
unsigned long lastServoUpdateTime = 0;

const uint8_t SERIAL_BUFFER_SIZE = 32;
char serialRxBuffer[SERIAL_BUFFER_SIZE];
int serialRxBufferIndex = 0;

// ***************************************************************
// --- 5. FUNCTION PROTOTYPES ---
// ***************************************************************
void initializeHardware();
void handleSerial();
void handleCommand(char* cmd);
void handleServoCommand(char* cmd);
void handleOffsetCommand(char* cmd);
void handleRampCommand(char* cmd);
long readSonarDistanceCM();
void sendSensorPacket();
void writeServoFloat(uint8_t ch, float angle);
void updateServosSmooth();
void setDutyCycle(uint8_t channel, uint8_t percentage);
void updateStepperMotorPID();
void performStep(bool direction, int pwmVal);
void applyStepOutput(StepperSequence step, int pwmVal);
int getCurrentStepperAngleFromPot();

// ***************************************************************
// --- 6. SETUP ---
// ***************************************************************

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_SPEED);

    for (uint8_t pin : STEPPER_IN_PINS) {
        pinMode(pin, OUTPUT);
        analogWrite(pin, 0);
    }
    pinMode(SONAR_TRIG_PIN, OUTPUT);
    pinMode(SONAR_ECHO_PIN, INPUT);
    pinMode(STEPPER_FEEDBACK_PIN, INPUT);

    TCCR3B = (TCCR3B & 0b11111000) | 0x01; // Fast PWM, no prescaler
    TCCR3A |= 0b00000011;
    TCCR4B = (TCCR4B & 0b11111000) | 0x01;
    TCCR4A |= 0b00000011;

    Serial.println(F("SynROV"));
}

// ***************************************************************
// --- 7. MAIN LOOP ---
// ***************************************************************

void loop() {
    handleSerial();

    if (systemActive) {
        updateStepperMotorPID();
        updateServosSmooth();

        if (millis() - lastSensorTime >= SENSOR_INTERVAL_MS) {
            lastSensorTime = millis();
            sendSensorPacket();
        }
    }
}

// ***************************************************************
// --- 8. STEPPER AND PID FUNCTIONS ---
// ***************************************************************

int getCurrentStepperAngleFromPot() {
    int rawPotValue = analogRead(STEPPER_FEEDBACK_PIN);

    if (filtered_pot_value == 0.0) {
        filtered_pot_value = (float)rawPotValue;
    } else {
        filtered_pot_value = (EMA_ALPHA * (float)rawPotValue) + ((1.0 - EMA_ALPHA) * filtered_pot_value);
    }
    float linearizedValue = filtered_pot_value;

    int potMinRaw = STEPPER_CALIBRATION_ZERO_ADC + servoOffsets[0];
    int potMaxRaw = potMinRaw + (int)(STEPPER_MAX_ANGLE_SPAN_RAW * servoSpan[0]);
    potMaxRaw = constrain(potMaxRaw, potMinRaw + 10, 1023);

    current_stepper_angle = map((long)linearizedValue, potMinRaw, potMaxRaw, 0, STEPPER_MAX_ANGLE);
    current_stepper_angle = constrain(current_stepper_angle, 0, STEPPER_MAX_ANGLE);

    return current_stepper_angle;
}

void updateStepperMotorPID() {
    unsigned long currentMillis = millis();
    unsigned long currentMicros = micros();

    getCurrentStepperAngleFromPot();

    if (currentMillis - lastPIDTime >= PID_INTERVAL_MS) {
        lastPIDTime = currentMillis;

        float error = (float)stepper_target_angle - (float)current_stepper_angle;

        if (abs(error) < STEPPER_TOLERANCE) {
            if (!isStabilized) {
                stabilizationStartTime = currentMillis;
                isStabilized = true;
                currentPWMValue = ACTIVE_PWM_MIN;
            } else {
                currentPWMValue = (currentMillis - stabilizationStartTime >= IDLE_DELAY_MS) ? IDLE_PWM_HOLD : ACTIVE_PWM_MIN;
            }

            applyStepOutput(currentStepSequence, currentPWMValue);
            integralError = 0.0;
            lastError = error;
            return;
        }

        isStabilized = false;

        // PID Calculation
        integralError += error * PID_DELTA_T;
        integralError = constrain(integralError, -200.0, 200.0);

        float derivativeError = (error - lastError) / PID_DELTA_T;
        lastError = error;

        float outputPID = (Kp * error) + (Ki * integralError) + (Kd * derivativeError);

        int motorPWM = abs(outputPID);
        currentPWMValue = constrain(motorPWM, ACTIVE_PWM_MIN, 255);
    }

    if (!isStabilized && abs((float)stepper_target_angle - (float)current_stepper_angle) >= STEPPER_TOLERANCE) {
        int absError = abs(stepper_target_angle - current_stepper_angle);

        long stepDelayUs = map((long)absError, STEPPER_TOLERANCE, STEPPER_MAX_ANGLE, 15000L, (long)MIN_STEP_DELAY_US);
        stepDelayUs = constrain(stepDelayUs, MIN_STEP_DELAY_US, 15000L);

        if (currentMicros - lastStepTime >= (unsigned long)stepDelayUs) {
            lastStepTime = currentMicros;

            bool direction = (stepper_target_angle > current_stepper_angle);

            performStep(direction, currentPWMValue);
        }
    }
}

void performStep(bool direction, int pwmVal) {
    if (direction) {
        currentStepSequence = (StepperSequence)((currentStepSequence + 1) % 4);
    } else {
        currentStepSequence = (StepperSequence)((currentStepSequence - 1 + 4) % 4);
    }
    applyStepOutput(currentStepSequence, pwmVal);
}

void applyStepOutput(StepperSequence step, int pwmVal) {
    for (uint8_t pin : STEPPER_IN_PINS) {
        analogWrite(pin, 0);
    }

    switch (step) {
        case PHASE_0:
            analogWrite(STEPPER_IN_PINS[0], pwmVal); analogWrite(STEPPER_IN_PINS[2], pwmVal); break;
        case PHASE_1:
            analogWrite(STEPPER_IN_PINS[1], pwmVal); analogWrite(STEPPER_IN_PINS[2], pwmVal); break;
        case PHASE_2:
            analogWrite(STEPPER_IN_PINS[1], pwmVal); analogWrite(STEPPER_IN_PINS[3], pwmVal); break;
        case PHASE_3:
            analogWrite(STEPPER_IN_PINS[0], pwmVal); analogWrite(STEPPER_IN_PINS[3], pwmVal); break;
    }
}

// ***************************************************************
// --- 9. SERVO AND PWM FUNCTIONS (PCA9685) ---
// ***************************************************************

void writeServoFloat(uint8_t ch, float angle) {
    if (!pwmReady || ch >= MAX_SERVOS || (ch >= 12 && ch <= 15)) {
        return;
    }

    angle = constrain(angle, 0.0, 180.0);

    if (servoDirection[ch] == -1) {
        angle = 180.0 - angle;
    }

    float pulseRange = (float)SERVO_MAX_PULSE - (float)SERVO_MIN_PULSE;
    float basePulse = (float)SERVO_MIN_PULSE + pulseRange * (angle / 180.0);

    float spanAdjustedPulse = (float)SERVO_MIN_PULSE + (basePulse - (float)SERVO_MIN_PULSE) * servoSpan[ch];
    float finalPulse = spanAdjustedPulse + (float)servoOffsets[ch];

    uint16_t finalPulseInt = constrain((uint16_t)finalPulse, 0, 4095);

    pwm.setPWM(ch, 0, finalPulseInt);
}

void updateServosSmooth() {
    if (!pwmReady || millis() - lastServoUpdateTime < SERVO_UPDATE_INTERVAL_MS) {
        return;
    }
    lastServoUpdateTime = millis();

    for (uint8_t ch = 1; ch < 12; ch++) {
        float targetAngle = (float)currentAngles[ch];

        if (abs(actualFloatAngle[ch] - targetAngle) > 0.1) {

            if (servoRampingEnabled[ch]) {
                float error = targetAngle - actualFloatAngle[ch];
                float direction = (error > 0) ? 1.0 : -1.0;
                float step = SERVO_MOVE_STEP;
                if (abs(error) < SERVO_MOVE_STEP) {
                    step = abs(error);
                }

                actualFloatAngle[ch] += direction * step;
            }
            else {
                actualFloatAngle[ch] = targetAngle;
            }

            writeServoFloat(ch, actualFloatAngle[ch]);
        }
    }
}

void setDutyCycle(uint8_t channel, uint8_t percentage) {
    if (!pwmReady || channel < 12 || channel > 15) {
        Serial.println(F("Error: PCA9685 not ready or invalid channel for duty cycle (must be 12-15)."));
        return;
    }

    percentage = constrain(percentage, 0, 100);
    uint16_t pulseValue = map(percentage, 0, 100, 0, 4095);

    pwm.setPWM(channel, 0, pulseValue);
}

// ***************************************************************
// --- 10. INITIALIZATION AND SERIAL COMMUNICATION FUNCTIONS ---
// ***************************************************************

void initializeHardware() {
    Serial.println(F("SynROV"));

    if (ina219.begin()) {
        inaReady = true;
        ina219.setCalibration_16V_400mA();
        Serial.println(F("INA219 initialized."));
    } else {
        Serial.println(F("INA219 not found or error!"));
    }

    Wire.beginTransmission(PCA9685_ADDR);
    if (Wire.endTransmission() == 0) {
        pwm.begin();
        pwm.setPWMFreq(50);
        pwmReady = true;
        Serial.println(F("PCA9685 initialized."));
    } else {
        Serial.println(F("PCA9685 not found or error!"));
    }

    mpu6050.initialize();
    mpuReady = mpu6050.testConnection();
    if (mpuReady) {
        Serial.println(F("MPU6050 initialized."));
    } else {
        Serial.println(F("MPU6050 not found or error!"));
    }
    stepper_target_angle = getCurrentStepperAngleFromPot();
    currentStepSequence = PHASE_0;

    systemActive = true;
    Serial.println(F("READY!"));
}

void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            serialRxBuffer[serialRxBufferIndex] = '\0';
            if (serialRxBufferIndex > 0) {
                handleCommand(serialRxBuffer);
            }
            serialRxBufferIndex = 0;
        } else {
            if (serialRxBufferIndex < SERIAL_BUFFER_SIZE - 1) {
                serialRxBuffer[serialRxBufferIndex++] = toupper(c);
            } else {
                Serial.println(F("Serial buffer overflow!"));
                serialRxBufferIndex = 0;
            }
        }
    }
}

void handleCommand(char* cmd) {
    if (strcmp(cmd, "READY?") == 0) {
        initializeHardware();
    }
    else if (strncmp(cmd, "S", 1) == 0 && strchr(cmd, 'P') != NULL) {
        handleServoCommand(cmd);
    }
    else if (strncmp(cmd, "F", 1) == 0 && strchr(cmd, 'P') != NULL) {
        char* pCh = strtok(cmd + 1, "P");
        char* pPerc = strtok(NULL, "");
        if (pCh != NULL && pPerc != NULL) {
            uint8_t ch = atoi(pCh);
            uint8_t percentage = atoi(pPerc);
            setDutyCycle(ch, percentage);
        }
    }
}

void handleServoCommand(char* cmd) {
    char* chStr = cmd + 1;
    char* angleStr = strchr(chStr, 'P');

    if (angleStr != NULL) {
        *angleStr = '\0';
        angleStr++;

        uint8_t ch = atoi(chStr);
        uint8_t angle = atoi(angleStr);

        if (ch == 0) {
            if (angle <= STEPPER_MAX_ANGLE) {
                stepper_target_angle = angle;
                isStabilized = false;
            } else {
                Serial.print(F("Invalid angle for Stepper (S0Pxx): "));
                Serial.println(angle);
            }
        } else if (ch < MAX_SERVOS) {
            if (angle <= 180) {
                currentAngles[ch] = angle;
            } else {
                Serial.println(F("Invalid angle (must be 0-180)."));
            }
        }
    }
}

void handleOffsetCommand(char* cmd) {
    // This command handler was not fully detailed in the provided code,
    // so it's included as a placeholder based on the original structure.
    uint8_t ch;
    int16_t value;
    float fValue;

    if (sscanf(cmd, "OFF%hhu.%hd", &ch, &value) == 2) {
        if (ch < MAX_SERVOS) {
            servoOffsets[ch] = value;
            Serial.print(F("Offset CH")); Serial.print(ch); Serial.print(F(" set: ")); Serial.println(value);
        }
    } else if (sscanf(cmd, "SPAN%hhu.%f", &ch, &fValue) == 2) {
         if (ch < MAX_SERVOS) {
            servoSpan[ch] = fValue;
            Serial.print(F("Span CH")); Serial.print(ch); Serial.print(F(" set: ")); Serial.println(fValue);
        }
    }
}

void handleRampCommand(char* cmd) {
    // This command handler was not fully detailed in the provided code,
    // so it's included as a placeholder based on the original structure.
    uint8_t ch;
    int state;

    if (sscanf(cmd, "RAMP%hhu.%d", &ch, &state) == 2) {
        if (ch >= 1 && ch < 12) {
            bool newState = (state == 1);
            servoRampingEnabled[ch] = newState;
            Serial.print(F("Ramp CH")); Serial.print(ch); Serial.print(F(" set: ")); Serial.println(newState ? F("ON") : F("OFF"));
        }
    }
}

// ***************************************************************
// --- 11. SENSOR READING FUNCTIONS ---
// ***************************************************************

long readSonarDistanceCM() {
    digitalWrite(SONAR_TRIG_PIN, LOW);
    digitalWrite(SONAR_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG_PIN, LOW);

    long duration = pulseIn(SONAR_ECHO_PIN, HIGH, 20000);
    long dist = duration / 58;

    return (dist > 0 && dist < 400) ? dist : -1;
}

void sendSensorPacket() {
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (mpuReady) {
        mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    }

    Serial.print(F("#SENS|MPU:"));
    Serial.print(ax); Serial.print(','); Serial.print(ay); Serial.print(','); Serial.print(az); Serial.print(',');
    Serial.print(gx); Serial.print(','); Serial.print(gy); Serial.print(','); Serial.print(gz);

    Serial.print(F("|AN:"));
    Serial.print((int)filtered_pot_value); Serial.print(',');
    Serial.print(current_stepper_angle); Serial.print(',');
    for (uint8_t i = 2; i < 6; i++) {
        Serial.print(analogRead(i));
        if (i < 5) Serial.print(',');
    }

    Serial.print(F("|EX1:"));
    long dist = readSonarDistanceCM();
    Serial.print(dist > 0 ? dist : -1);

    Serial.print(F("|EX2:"));
    if (inaReady) {
        float current_mA = ina219.getCurrent_mA();
        Serial.print((int)current_mA);
    } else {
        Serial.print(-1);
    }
    Serial.println();
}
