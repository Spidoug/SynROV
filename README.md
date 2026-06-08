# SynROV

SynROV is a multi-robot control and visualization project that combines embedded firmware, a desktop 3D operator console, a browser-based WebSocket console, and a Python AiBot interface. The current source tree supports three robot profiles:

- **Manipulator**: articulated robotic arm with base rotation, upper arm, forearm, forearm roll, wrist pitch, wrist roll, gripper, collision guarding, grip pressure profiling, stabilization, and configurable joint mapping.
- **Vehicle**: tracked ground vehicle with left/right track control, camera pan/tilt, lights, LiDAR/scan servo support, incline assist, and low-battery return behavior.
- **Drone**: quadcopter-style control profile with ESC output, throttle/yaw/pitch/roll/strafe/forward commands, camera record output, takeoff/landing requests, and safety return logic.

  <img width="2556" height="1033" alt="image" src="https://github.com/user-attachments/assets/08edd819-6fb0-43d3-9fb8-2a9f2c930a30" />

The project is designed as a complete experimental robotics stack: the firmware owns real-time hardware I/O and safety behavior, Processing provides the main operator station and 3D simulation/telemetry view, the web console exposes remote browser controls, and the Python AiBot adds voice, vision, dataset collection, and learning-oriented control workflows.

> **Safety note:** SynROV can drive motors, servos, ESCs, and other actuators. Test with power-limited hardware first, keep the robot mechanically supported during configuration, and verify every output channel before operating a real mechanism.

## Repository layout

```text
firmware/
  SynROV_Firmware/
    SynROV_Firmware.ino                 Main Arduino sketch entry point
    FirmwareDeclarations.h              Shared constants, structs, globals, pins, prototypes
    01_CRC_And_EEPROM.h                 EEPROM persistence and CRC helpers
    02_Initialization.h                 Runtime state, boot setup, hardware initialization
    03_Stepper_Low_Level.h              Stepper motor low-level driving
    04_Servo_And_PWM_Output.h           Servo/PWM output handling and safe pose logic
    05_Config_Defaults_And_Sync.h       Default configuration, sync, calibration storage
    06_Runtime_Commands_And_Diagnostics.h
                                           Runtime commands, CFG menu, diagnostics, tuning
    07_Gps_Serial1.h                    GPS input on Serial1
    07_Serial_Control_And_Telemetry.h   Serial command parsing and telemetry protocol
    08_Setup_And_Loop.h                 Arduino setup() and loop()
    09_Safety_Autonomy.h                Safety/autonomy behaviors

software/
  processing/
    SynROV/
      SynROV.pde                        Main Processing operator console
      ModuleContracts.pde               Robot module interface/registry
      Logic_RuntimeBridge.pde           Serial/WebSocket runtime bridge logic
      DiagnosticsPanel_Base.pde         Diagnostics panel and telemetry widgets
      EnvironmentMapping.pde            Environment/map rendering and import helpers
      Manipulator3D.pde                 Manipulator model, controls, diagnostics
      Vehicle3D.pde                     Vehicle model, controls, diagnostics
      Drone3D.pde                       Drone model, controls, diagnostics

  web/
    SynROV.html                         Browser WebSocket console

  python/
    synrov_aibot.py                     Launcher script
    synrov_aibot/                       Modular Python AiBot package
      _legacy.py                        Original large compatibility core
      bridge.py                         WebSocket bridge exports
      config.py                         Constants and path exports
      helpers.py                        Numeric, text, and pose helper exports
      learning.py                       Dataset/model/inference helper exports
      media.py                          Vision/audio/webcam helper exports
      runtime.py                        Compatibility runtime exports
      state.py                          State and pose exports
      ui.py                             Tkinter UI exports
      voice.py                          Voice parser/audio engine exports
```

## Main capabilities

SynROV includes:

- Arduino firmware for an Arduino Mega-class board using Serial0 for the control/telemetry stream and Serial1 for GPS.
- EEPROM-backed configuration with CRC validation, default restore, load/save, channel mapping, robot mode selection, and calibration flows.
- Support for PCA9685 PWM/servo expansion with Arduino fallback outputs.
- Sensor integration points for INA219 power sensors, MPU6050 IMUs, GY-273/HMC/QMC compass modules, sonar, GPS, analog feedback, and actuator telemetry.
- Runtime HEX-framed serial protocol after the initial ASCII boot/configuration phase.
- Processing desktop operator station with 3D robot visualization, diagnostics, serial connection management, simulation mode, joystick integration, Leap Motion support, WebSocket server, and bilingual UI support.
- Browser WebSocket console for remote status, viewport streaming, mode-aware controls, quick actions, and runtime command shortcuts.
- Python AiBot package with WebSocket bridge, voice parsing, webcam/vision features, dataset collection, model training helpers, and a Tkinter control UI.

## Hardware overview

The firmware pin map in `FirmwareDeclarations.h` targets an Arduino Mega-style pinout. Important defaults include:

| Function | Default pins / interface |
| --- | --- |
| Serial control and telemetry | `Serial0` over USB at `115200` baud |
| GPS input | `Serial1` |
| Stepper coils | Digital pins `42, 43, 44, 45` |
| Stepper power PWM | Pin `46`, Timer5 channel A |
| Stepper feedback | `A0` |
| Vehicle left/right PWM | Pins `2` and `3` |
| Vehicle left/right direction | Pins `42` and `43` |
| Vehicle light | Pin `44` |
| Shared camera/gimbal pan/tilt | Pins `11` and `12` |
| Vehicle LiDAR scan servo | Pin `24` |
| Drone ESC outputs | Pins `22, 23, 24, 25` |
| Drone camera record output | Pin `44` |
| Drone status LED | Pin `45` |
| Sonar | Pin `7`, max distance `400 cm` |
| I2C bus | PCA9685, INA219 sensors, MPU6050 sensors, compass |

Several pins are intentionally shared between robot modes. Select the correct `ROBOT` mode and hardware configuration before enabling actuators.

## Software requirements

### Arduino firmware

Install the Arduino IDE or Arduino CLI and the required libraries:

- `Adafruit PWM Servo Driver`
- `Adafruit INA219`
- `MPU6050_6Axis_MotionApps20` compatible MPU6050 library
- `NewPing`
- `Servo`
- Standard Arduino libraries: `Wire`, `SPI`, `EEPROM`

Recommended board target: **Arduino Mega 2560 or compatible**, because the firmware uses Mega-style pins, `Serial1`, and Timer5/PWM pin 46.

### Processing operator console

Install Processing and add the libraries used by `SynROV.pde`:

- Processing Serial library
- Leap Motion library: `de.voidplus.leapmotion`
- Game Control Plus: `org.gamecontrolplus`
- JInput: `net.java.games.input`
- Processing WebSockets library: `websockets`

Open the folder `software/processing/SynROV/` as the Processing sketch.

### Web console


<img width="2538" height="912" alt="image" src="https://github.com/user-attachments/assets/f868ca5b-8abe-4b0e-b830-41a8fc1f97b3" />



The web console is a single HTML file:

```text
software/web/SynROV.html
```

It connects to the Processing WebSocket server, which defaults to:

```text
ws://localhost:9000/
```

### Python AiBot


<img width="2557" height="1031" alt="Captura de tela 2026-06-08 193250" src="https://github.com/user-attachments/assets/81ba9a30-fcf6-43bd-91a0-ea4251434a0c" />


The Python package is under `software/python/`. The current compatibility core imports these packages when available:

```bash
pip install websocket-client pillow numpy scikit-learn joblib SpeechRecognition sounddevice opencv-python
```

Depending on your operating system, `tkinter` may need to be installed separately, for example `python3-tk` on some Linux distributions. Microphone support may also require system audio drivers.

Run from the `software/python` directory:

```bash
python synrov_aibot.py
```

or:

```bash
python -m synrov_aibot
```

## Quick start

1. **Upload the firmware.** Open `firmware/SynROV_Firmware/SynROV_Firmware.ino`, select the Arduino Mega-compatible board and serial port, then upload.
2. **Start the Processing console.** Open `software/processing/SynROV/SynROV.pde` in Processing and run it.
3. **Connect hardware.** In the Processing sidebar, click the connection button. The firmware starts in ASCII boot/configuration mode, then Processing performs the runtime handshake.
4. **Choose the robot profile.** Select Manipulator, Vehicle, or Drone in the Processing UI. The firmware can also be configured with `ROBOT=<mode>` inside CFG mode.
5. **Use simulation when needed.** Double-clicking the connection button activates simulation mode from the UI, allowing desktop testing without hardware.
6. **Open the web console.** Open `software/web/SynROV.html`, keep host as `localhost`, port as `9000`, then click **Connect WebSocket**.
7. **Use the Python AiBot if desired.** Run the Python launcher and connect it to `ws://127.0.0.1:9000/`.

## Firmware modes

The firmware uses two major serial phases:

### ASCII boot/configuration phase

After reset, the firmware prints a boot banner, hardware detection summary, and quick help. During this phase you can use commands such as:

```text
READY?       Enter runtime mode
CFG=1234     Enter configuration mode
BOOT?        Reopen the boot screen
BOOTRESET    Reopen the boot screen without a hardware reset
```

### HEX runtime phase

After `READY?`, the runtime switches to HEX-framed packets on Serial0. Processing is the primary runtime client and handles the frame protocol automatically. The firmware periodically sends telemetry while processing incoming control frames and safety checks.

## Configuration workflow

Use the built-in Processing calibration monitor with the `K` key, or use a serial terminal at `115200` baud when Processing is not connected.

Typical flow:

```text
CFG=1234
SHOW
ROBOT=MANIPULATOR
SHOW
SAVE
EXIT
READY?
```

Common system commands:

```text
ERASEEEPROM     Erase EEPROM
DEFAULT         Restore default values
SAVE            Save current configuration to EEPROM
LOAD            Load configuration from EEPROM
SHOW            Print current settings
EXIT / CFGOFF   Leave configuration mode
ROBOT=<0|1|2|MANIPULATOR|VEHICLE|DRONE>
```

## Runtime control examples

Manipulator runtime command:

```text
ARMJ=base,upper,forearm,forearmRoll,wristPitch,wristRoll,gripper,duty0,duty1,duty2,duty3
```

Vehicle runtime commands:


<img width="2555" height="1031" alt="image" src="https://github.com/user-attachments/assets/198091eb-8778-41da-b83a-69c9074d1ed2" />


```text
TRACK=left,right,CAM=pan,tilt
MOVE=throttle,steer
LIGHT=0/1
SCAN=0/1
```

Drone runtime commands:


<img width="1108" height="687" alt="image" src="https://github.com/user-attachments/assets/7ff6bf96-8429-46d2-a565-b56639d0f3ea" />



```text
FLY=throttle,yaw,pitch,roll,strafe,forward
CAMREC=0/1
TAKEOFF
LAND
STOP
```

The web console sends mode-aware JSON control snapshots to Processing. It intentionally does not send arbitrary raw serial commands; only mapped runtime shortcuts such as `READY?`, `HOME`, `TAKEOFF`, `LAND`, and valid manipulator `ARMJ=` commands are converted.

## Keyboard controls in Processing

### Manipulator

| Key | Action |
| --- | --- |
| `Z` / `X` | Base positive / negative |
| `S` / `W` | Upper arm positive / negative |
| `F` / `R` | Forearm positive / negative |
| `A` / `D` | Forearm roll negative / positive |
| `G` / `T` | Wrist pitch positive / negative |
| `Q` / `E` | Wrist roll positive / negative |
| `Y` / `H` | Gripper positive / negative |
| `C` | Toggle manipulator collision guard |
| `O` | Toggle trace |
| `L` | Toggle Leap Motion |
| `J` | Open joystick control window |
| `K` | Open calibration monitor |

### Vehicle

| Key | Action |
| --- | --- |
| `W` / `S` | Forward / reverse |
| `A` / `D` | Turn left / right |
| `Q` / `E` | Pivot left / right |
| `Z` / `X` | Camera pan |
| `R` / `F` | Camera tilt |
| `L` | Toggle Leap Motion |
| `J` | Open joystick control window |
| `K` | Open calibration monitor |

### Drone

| Key | Action |
| --- | --- |
| `Q` / `E` | Yaw |
| `W` / `S` | Forward / backward |
| `A` / `D` | Strafe left / right |
| `T` / `G` | Altitude up / down |
| `Z` / `X` | Camera pan |
| `R` / `F` | Camera tilt |
| `L` | Toggle Leap Motion |
| `J` | Open joystick control window |
| `K` | Open calibration monitor |

For all modes, arrow keys rotate the 3D camera when the mouse is over the viewport. `+` and `-` zoom the scene.

## Detailed operating documentation

A longer operating guide is provided in:

```text
docs/SynROV_Operating_Guide.md
```

A DOCX version is also available as:

```text
docs/SynROV_Operating_Guide.docx
```
