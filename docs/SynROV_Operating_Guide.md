# SynROV Operating Guide

This document explains how to install, configure, operate, and troubleshoot the SynROV project. It is written for developers and operators who want to run the complete system from the Arduino firmware through the Processing desktop station, browser WebSocket console, and Python AiBot.

## 1. What SynROV does

SynROV is a multi-robot experimental robotics platform. The project is organized around one firmware stack and multiple operator interfaces. The firmware controls the hardware. The Processing application acts as the primary operator station. The web console connects to Processing through WebSocket for browser-based control. The Python AiBot connects through the same WebSocket path and adds voice, vision, dataset collection, and machine-learning-oriented control tools.

The firmware supports three robot personalities:

1. **Manipulator** - an articulated arm with seven controlled members: base, upper arm, forearm, forearm roll, wrist pitch, wrist roll, and gripper. It also includes joint mapping, geometry configuration, stabilizers, collision guard options, pressure profiling, and stepper calibration/tuning for the base when CH0 is configured as a stepper.
2. **Vehicle** - a tracked ground vehicle with left and right track outputs, camera pan/tilt, lights, optional LiDAR scan movement, incline assist using an MPU sensor, and safety return features.
3. **Drone** - a quadcopter-style robot profile with four ESC outputs, throttle/yaw/pitch/roll/strafe/forward control, camera record output, takeoff and landing requests, and low-battery safety return behavior.

SynROV can also be used as a simulator and visualization tool. Processing can render and control the robot scenes without live hardware, which is useful for UI testing, command validation, and demonstrations.

## 2. Safety rules before operating real hardware

SynROV can command real actuators. Treat every connection test as a live robotics test.

Before powering motors or ESCs:

- Disconnect propellers from drone motors during firmware upload, setup, and first control tests.
- Lift tracked vehicles off the ground or use a stand during first motor tests.
- Keep manipulator joints clear of hands, wires, tools, and loose objects.
- Use a current-limited bench supply for early tests.
- Verify that the selected `ROBOT` mode matches the wiring on the connected robot.
- Verify motor direction at low power before increasing speed.
- Confirm servo limits before saving configuration.
- Use `STOP`, `LAND`, or power removal as appropriate for the robot type.
- Do not use the Python AiBot autonomous/learning features on real hardware until the baseline Processing controls are already proven safe.

## 3. System architecture

The system has four layers:

### 3.1 Firmware layer

The firmware runs on an Arduino Mega-class board. It performs hardware initialization, EEPROM configuration, serial command parsing, telemetry generation, actuator control, sensor reading, safety checks, and calibration/tuning flows.

The main sketch is:

```text
firmware/SynROV_Firmware/SynROV_Firmware.ino
```

The `.ino` file includes the implementation tabs in a deliberate order. Most shared constants, structures, globals, and prototypes are in `FirmwareDeclarations.h`.

### 3.2 Processing operator station

Processing is the main desktop operator interface. It renders the robot in 3D, owns the serial connection, handles simulation/hardware mode switching, displays diagnostics, accepts keyboard/joystick/Leap Motion controls, and exposes a WebSocket server for external clients.

The Processing sketch folder is:

```text
software/processing/SynROV/
```

Open the folder itself in Processing so that all `.pde` tabs are compiled together.

### 3.3 Browser web console

The web console is a standalone HTML page that connects to Processing. It does not connect directly to the Arduino. Processing must be running first.

The web console file is:

```text
software/web/SynROV.html
```

Default WebSocket endpoint:

```text
ws://localhost:9000/
```

### 3.4 Python AiBot

The Python AiBot is a Tkinter application/package that connects to the Processing WebSocket bridge. It includes helper modules for state, bridge communication, voice parsing, media/vision, learning, and UI. The compatibility implementation remains in `_legacy.py`, while the smaller modules re-export stable groups of functionality.

Launcher:

```text
software/python/synrov_aibot.py
```

Package entry point:

```text
python -m synrov_aibot
```

## 4. Installation

### 4.1 Arduino IDE setup

Install the Arduino IDE or Arduino CLI. Then install these libraries:

- Adafruit PWM Servo Driver
- Adafruit INA219
- MPU6050 library compatible with `MPU6050_6Axis_MotionApps20.h`
- NewPing
- Servo

The firmware also uses standard Arduino libraries:

- Wire
- SPI
- EEPROM

Select an Arduino Mega 2560 or compatible board. This matters because the firmware uses Mega-style pins, Serial1, and Timer5/PWM pin 46.

### 4.2 Processing setup

Install Processing. Add the required libraries from the Processing Contribution Manager or your library source:

- Serial
- Leap Motion for Processing (`de.voidplus.leapmotion`)
- Game Control Plus (`org.gamecontrolplus`)
- JInput (`net.java.games.input`)
- WebSockets (`websockets`)

Open:

```text
software/processing/SynROV/SynROV.pde
```

Run the sketch. The default window size is `1100 x 650`, and the sketch uses the `P3D` renderer.

### 4.3 Python setup

From the repository root:

```bash
cd software/python
python -m pip install websocket-client pillow numpy scikit-learn joblib SpeechRecognition sounddevice opencv-python
```

On Linux, install Tkinter if it is not already available:

```bash
sudo apt install python3-tk
```

Some optional features depend on camera, microphone, or audio device support. If an optional package is missing, the Python application may still open, but the related feature will be disabled or limited.

### 4.4 Web console setup

No build step is required. Open:

```text
software/web/SynROV.html
```

Use a modern desktop browser. If the browser blocks local file WebSocket behavior, open the HTML from a simple local server, for example from `software/web`:

```bash
python -m http.server 8080
```

Then browse to the local server and connect to `localhost:9000`.

## 5. Firmware upload procedure

1. Connect the Arduino Mega-compatible board over USB.
2. Open `firmware/SynROV_Firmware/SynROV_Firmware.ino`.
3. Select the correct board and port.
4. Compile.
5. Upload.
6. Open a serial monitor at `115200` baud only if Processing is not using the same port.
7. Confirm that the firmware prints the SynROV boot banner and hardware summary.

The firmware starts in ASCII mode. You should see a quick help section with commands such as `READY?` and `CFG=1234`.

## 6. Firmware boot behavior

During boot, the firmware:

1. Starts Serial0 at `115200` baud.
2. Starts GPS input on Serial1.
3. Initializes I2C at `400 kHz`.
4. Sets up stepper, DC motor, PWM, fallback servo, and shared output pins.
5. Initializes runtime state, servo state, stepper state, auto-stepper state, and calibration state.
6. Loads configuration from EEPROM.
7. Applies a safe startup pose if required.
8. Rebinds I2C devices.
9. Detects compass hardware.
10. Configures robot-mode-specific hardware.
11. Prints a hardware summary and robot configuration summary.
12. Waits for either `READY?` or `CFG=1234`.

The boot summary reports PCA9685, INA219 sensors, MPU6050 sensors, GPS Serial1, and compass status.

## 7. Firmware serial phases

### 7.1 ASCII phase

The ASCII phase is used for boot, diagnostics, and configuration. Commands are human-readable. Examples:

```text
BOOT?
BOOTRESET
READY?
CFG=1234
SHOW
SAVE
EXIT
```

### 7.2 HEX runtime phase

After `READY?`, the firmware enters runtime mode and uses HEX-framed packets on Serial0. Processing manages this protocol automatically. Operators normally do not type HEX packets by hand.

In runtime, the firmware:

- Sends periodic telemetry.
- Processes control frames.
- Updates stepper PID and auto-stepper state.
- Updates manipulator stabilization and grip pressure logic when in manipulator mode.
- Updates vehicle/drone outputs when in those robot modes.
- Runs safety autonomy logic.
- Enforces runtime stream failsafe behavior.
- Smoothly updates servo outputs at the configured update interval.

## 8. Configuration mode

Enter configuration mode with:

```text
CFG=1234
```

Print the full menu and current settings:

```text
SHOW
```

Leave configuration mode:

```text
EXIT
```

or:

```text
CFGOFF
```

Save to EEPROM:

```text
SAVE
```

Load from EEPROM:

```text
LOAD
```

Restore defaults:

```text
DEFAULT
```

Erase EEPROM:

```text
ERASEEEPROM
```

Select robot mode:

```text
ROBOT=MANIPULATOR
ROBOT=VEHICLE
ROBOT=DRONE
```

Numeric aliases are also supported:

```text
ROBOT=0
ROBOT=1
ROBOT=2
```

Use `SHOW` after changes to confirm the active configuration before saving.

## 9. Hardware mapping and important pins

The firmware default hardware map includes:

| Area | Default |
| --- | --- |
| Control serial | Serial0 USB, `115200` baud |
| GPS | Serial1 |
| I2C speed | `400000` Hz |
| Stepper coil pins | `42, 43, 44, 45` |
| Stepper power PWM | `46` |
| Stepper feedback | `A0` |
| Sonar | `7` |
| Vehicle left/right PWM | `2`, `3` |
| Vehicle left/right direction | `42`, `43` |
| Vehicle light | `44` |
| Shared gimbal pan/tilt | `11`, `12` |
| Vehicle LiDAR scan servo | `24` |
| Drone ESC outputs | `22, 23, 24, 25` |
| Drone camera record | `44` |
| Drone status LED | `45` |

Because some pins are shared between modes, never assume all outputs are safe in every robot mode. Always set the correct robot mode and verify the channel mapping.

## 10. Manipulator operation

### 10.1 Manipulator members

The manipulator control model uses these members:

1. Base
2. Upper arm
3. Forearm
4. Forearm roll
5. Wrist pitch
6. Wrist roll
7. Gripper

The gripper convention is:

- `0 degrees` = fully closed
- `100 degrees` = fully open

### 10.2 Manipulator runtime command

The firmware accepts a runtime arm command in this form:

```text
ARMJ=base,upper,forearm,forearmRoll,wristPitch,wristRoll,gripper,duty0,duty1,duty2,duty3
```

The first seven values are joint/member commands. The final four values are auxiliary PWM/force outputs.

A neutral value can be represented internally as `1000` for a member in some control paths. The web console uses this when neutral mode is toggled for a joint.

### 10.3 Manipulator keyboard controls in Processing

| Key pair | Function |
| --- | --- |
| `Z` / `X` | Base positive / negative |
| `S` / `W` | Upper arm positive / negative |
| `F` / `R` | Forearm positive / negative |
| `A` / `D` | Forearm roll negative / positive |
| `G` / `T` | Wrist pitch positive / negative |
| `Q` / `E` | Wrist roll positive / negative |
| `Y` / `H` | Gripper positive / negative |
| `C` | Toggle local collision guard |
| `O` | Toggle trace |
| `L` | Toggle Leap Motion |
| `J` | Open joystick control window |
| `K` | Open calibration monitor |

### 10.4 Manipulator geometry configuration

The firmware includes many geometry settings so the visualization and collision model can match the physical arm. Common geometry commands include:

```text
ARMBASER=<v>       Base cylinder radius
ARMBASEH=<v>       Base cylinder height
ARMBBLW=<v>        Base block width
ARMBBLH=<v>        Base block height
ARMBBLD=<v>        Base block depth
ARMUPW=<v>         Upper arm width
ARMUPH=<v>         Upper arm height
ARMUPD=<v>         Upper arm depth
ARMFRW=<v>         Forearm width
ARMFRH=<v>         Forearm height
ARMFRD=<v>         Forearm depth
ARMWRW=<v>         Wrist width
ARMWRH=<v>         Wrist height
ARMWRD=<v>         Wrist depth
ARMFGW=<v>         Finger width
ARMFGH=<v>         Finger height
ARMFGD=<v>         Finger depth
ARMBCY=<v>         Base cylinder Y offset
ARMBBY=<v>         Base block Y offset
ARMGRY=<v>         Gripper Y offset
```

Joint mapping offsets use three values:

```text
ARMOFFUPPER=<servoZero,memberZero,sign>
ARMOFFFORE=<servoZero,memberZero,sign>
ARMOFFFORER=<servoZero,memberZero,sign>
ARMOFFWRISTP=<servoZero,memberZero,sign>
ARMOFFWRISTR=<servoZero,memberZero,sign>
ARMOFFGRIP=<servoZero,memberZero,sign>
```

The base offset is fixed so that the base servo angle equals base yaw.

### 10.5 Manipulator channel mapping

Manipulator channel commands include:

```text
BASECH=<0|1>         0 = CH0 potentiometer/stepper base, 1 = channel 1 base
UPPERCH=<2..11>      Upper arm channel
FORECH=<2..11>       Forearm channel
FOREROLLCH=<2..11>   Forearm roll channel
WRISTPITCHCH=<2..11> Wrist pitch channel
WRISTROTCH=<2..11>   Wrist rotation channel
GRIPCH=<2..11>       Gripper channel
```

Servo calibration examples:

```text
OFFSET1=10
SPAN1=1.050
DIR1=-1
RAMP1=1
MIN1=20
MAX1=160
```

Global ramp controls:

```text
RAMPSPD=1.0
RAMPSMOOTH=1.0
```

### 10.6 Manipulator stabilizers and pressure profiling

Available configuration/runtime controls include:

```text
GSTAB=0/1       Gripper/wrist stabilizer from MPU1
ASTAB=0/1       Arm stabilization from MPU2
ASTABGAIN=<v>   Arm stabilization gain
ASTABTH=<v>     Arm stabilization threshold
GPRESS=0/1      Grip pressure profiling using INA1
GPGAIN=<v>      Grip pressure gain
COLL=0/1        Collision guard disabled/enabled
```

Diagnostics fields include:

```text
GIDLEMA
GPRESSMA
GPRESSVAL
```

These are read-only diagnostic values.

## 11. Stepper setup and tuning

The manipulator base can use CH0 as a stepper/DC block depending on configuration.

Select CH0 drive mode:

```text
CH0MODE=STEPPER
CH0MODE=DC_MOTOR
```

or numeric:

```text
CH0MODE=0
CH0MODE=1
```

Select stepper mode:

```text
STEPMODE=FULLSTEP_BIPOLAR
STEPMODE=HALFSTEP_BIPOLAR
STEPMODE=FULLSTEP_UNIPOLAR
STEPMODE=HALFSTEP_UNIPOLAR
```

or numeric:

```text
STEPMODE=0
STEPMODE=1
STEPMODE=2
STEPMODE=3
```

### 11.1 Stepper limit calibration

Run inside CFG mode:

```text
LIMITSTEPPERCALIB
```

Procedure:

1. Enter CFG mode.
2. Send `LIMITSTEPPERCALIB`.
3. Move the motor to the first mechanical limit.
4. Send `LIMITSTEPPERCALIB` again.
5. Move the motor to the opposite mechanical limit.
6. Send `LIMITSTEPPERCALIB` again.
7. Review the resulting `ZEROADC` and `SPANRAW`.
8. Send `SAVE` if the values are correct.

The firmware rejects calibration if the two limits are too close.

### 11.2 Automatic stepper tuning

Run inside CFG mode after limit calibration:

```text
AUTOSTEPPER
```

Useful commands:

```text
AUTOSTATUS
AUTOSTOP
```

The automatic tuner explores torque, band, and PID behavior. At the end it reports best parameters, but they are not saved until you issue `SAVE`.

### 11.3 Direct stepper diagnostics

Use:

```text
STEPDIAG
STEPTEST=F,200,80,2200
STEPTEST=R,200,80,2200
```

`STEPTEST` format:

```text
STEPTEST=<F|R>,<steps>,<pwmPercent>,<delayMicroseconds>
```

The firmware requires the robot mode and CH0 mode to be compatible with stepper testing.

### 11.4 Stepper parameter commands

Common stepper configuration commands:

```text
ZEROADC=<v>
SPANRAW=<v>
MINSTEPUS=<v>
MAXSTEPUS=<v>
STOPENTER=<v>
STOPEXIT=<v>
STOPMOTORENTER=<v>
STOPMOTOREXIT=<v>
PWMFAR=<v>
PWMNEAR=<v>
PWMHOLD=<v>
ALPHAANG=<v>
```

## 12. Vehicle operation

### 12.1 Vehicle runtime commands

```text
TRACK=left,right,CAM=pan,tilt
MOVE=throttle,steer
LIGHT=0/1
SCAN=0/1
```

`TRACK` sends direct left/right track percentages and camera pan/tilt. `MOVE` sends a mixed throttle/steer style command.

### 12.2 Vehicle keyboard controls in Processing

| Key pair | Function |
| --- | --- |
| `W` / `S` | Forward / reverse |
| `A` / `D` | Turn left / right |
| `Q` / `E` | Pivot left / right |
| `Z` / `X` | Camera pan |
| `R` / `F` | Camera tilt |
| `L` | Toggle Leap Motion |
| `J` | Open joystick control window |
| `K` | Open calibration monitor |

### 12.3 Vehicle geometry configuration

```text
VEHBODYL=<v>      Body length in cm
VEHBODYW=<v>      Body width in cm
VEHBODYH=<v>      Body height in cm
VEHTGAUGE=<v>     Distance between tracks in cm
VEHTRUN=<v>       Track ground run length in cm
VEHDRIVE=<v>      Drive sprocket radius in cm
VEHSUPR=<v>       Support wheel radius in cm
VEHTOPR=<v>       Top roller radius in cm
VEHTWIDTH=<v>     Track belt width in cm
VEHTLINKL=<v>     Track link length in cm
VEHTLINKT=<v>     Track link thickness in cm
VEHTHEIGHT=<v>    Visual track height in cm
VEHTLINKS=<v>     Rendered track link count
VEHTSAG=<v>       Track sag amount
VEHCAMPAN0=<v>    Default camera pan angle
VEHCAMTILT0=<v>   Default camera tilt angle
VEHLIDARY=<v>     LiDAR mast Y offset
VEHLIDARZ=<v>     LiDAR mast Z offset
VEHCAMY=<v>       Camera head Y offset
VEHCAMZ=<v>       Camera head Z offset
```

### 12.4 Vehicle control configuration

```text
VEHDEAD=<v>          Motor command deadband in percent
VEHPWMMIN=<v>        Minimum motor PWM output
VEHPWMMAX=<v>        Maximum motor PWM output
VEHSLEW=<v>          Motor slew rate per update in percent
VEHINVLEFT=0/1       Invert left motor direction
VEHINVRIGHT=0/1      Invert right motor direction
VEHCAMPANMIN=<v>     Camera pan minimum
VEHCAMPANMAX=<v>     Camera pan maximum
VEHCAMTILTMIN=<v>    Camera tilt minimum
VEHCAMTILTMAX=<v>    Camera tilt maximum
VEHLIDARMIN=<v>      LiDAR sweep minimum
VEHLIDARMAX=<v>      LiDAR sweep maximum
VEHLIDARMS=<v>       LiDAR sweep cycle time in milliseconds
```

### 12.5 Vehicle safety features

```text
VEHRTH=0/1             Low-battery GPS return-to-base
VEHICLELINKHOLD=0/1    Invert last track movement while runtime link is lost
SAFEBAT=<pct>          Battery threshold for active robot return behavior
VEHINCL=0/1            MPU incline PWM assist for tracks
VEHINCLBASE=<v>        Incline assist base power in percent
VEHINCLMAX=<v>         Incline assist maximum power in percent
VEHINCLFULL=<deg>      Incline angle for full assist
VEHINCLDB=<deg>        Incline assist deadband
VEHINCLALPHA=<v>       Incline assist filter alpha
```

## 13. Drone operation

### 13.1 Drone runtime commands

```text
FLY=throttle,yaw,pitch,roll,strafe,forward
CAMREC=0/1
TAKEOFF
LAND
STOP
```

Start with very low values and test without propellers.

### 13.2 Drone keyboard controls in Processing

| Key pair | Function |
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

### 13.3 Drone geometry configuration

```text
DRNBODYL=<v>     Body length in cm
DRNBODYW=<v>     Body width in cm
DRNBODYH=<v>     Body height in cm
DRNARML=<v>      Arm length in cm
DRNARMT=<v>      Arm thickness in cm
DRNMOTR=<v>      Motor radius in cm
DRNMOTH=<v>      Motor height in cm
DRNPROPR=<v>     Propeller radius in cm
DRNPROPT=<v>     Propeller thickness in cm
DRNLEGH=<v>      Landing leg height in cm
DRNLEGS=<v>      Landing leg span in cm
DRNRESTY=<v>     Rest Y offset
DRNVISYAW=<v>    Visual yaw offset
DRNCAMY=<v>      Camera Y offset
DRNCAMZ=<v>      Camera Z offset
DRNLAMPY=<v>     Lamp Y offset
DRNLAMPZ=<v>     Lamp Z offset
DRNSONARX=<v>    Downward sonar X offset
DRNSONARY=<v>    Downward sonar Y offset
DRNSONARZ=<v>    Downward sonar Z offset
```

### 13.4 Drone control configuration

```text
DRNESCMIN=<v>     ESC minimum pulse in microseconds
DRNESCMAX=<v>     ESC maximum pulse in microseconds
DRNESCIDLE=<v>    ESC idle pulse in microseconds
DRNTHRSPAN=<v>    Throttle span above idle in microseconds
DRNPITCHMIX=<v>   Pitch mixer gain in microseconds
DRNROLLMIX=<v>    Roll mixer gain in microseconds
DRNYAWMIX=<v>     Yaw mixer gain in microseconds
DRNDEAD=<v>       Command deadband in percent
DRNSPOOL=<v>      ESC spool slew per update in microseconds
```

### 13.5 Drone safety configuration

```text
DRNRTH=0/1          Low-battery GPS return-to-base
DRONELINKHOLD=0/1   Invert last flight movement while runtime link is lost
SAFEBAT=<pct>       Battery threshold for active robot return behavior
```

## 14. I2C and sensor configuration

Common I2C address configuration commands:

```text
PCAADDR=<0xNN>      PCA9685 address
INAADDR=<0xNN>      INA219 #1 address
INA2ADDR=<0xNN>     INA219 #2 address
MPUADDR=<0xNN>      MPU6050 #1 address
MPU2ADDR=<0xNN>     MPU6050 #2 address, 0x00 disables
MAGADDR=<0xNN>      GY-273 address, 0x00 means auto-detect
```

Compass calibration commands:

```text
MAGCAL=1
MAGCAL=0
MAGCALCFG=minX,maxX,minY,maxY,minZ,maxZ,offX,offY,offZ,scaleX,scaleY,scaleZ
```

A typical compass calibration flow is:

1. Enter CFG mode.
2. Send `MAGCAL=1`.
3. Move the robot or sensor through a broad range of orientations.
4. Send `MAGCAL=0` when done.
5. Review the resulting calibration data.
6. Send `SAVE`.

## 15. Processing desktop operation

### 15.1 Starting the application

1. Open `software/processing/SynROV/SynROV.pde`.
2. Run the sketch.
3. Confirm that the sidebar appears and that the WebSocket status is online or listening.
4. Click the connection button to connect hardware.
5. Double-click the connection button to activate simulation mode when no hardware is connected.

### 15.2 UI language and theme

The Processing UI includes Portuguese and English text paths. Use the language button in the sidebar to toggle the UI language. Use the theme button to switch between dark and light mode.

### 15.3 Robot selection

The sidebar exposes robot modes:

- Manipulator
- Vehicle
- Drone

When live hardware is connected, selection may be locked to the robot detected or synchronized with the firmware. This prevents sending the wrong control stream to the wrong hardware profile.

### 15.4 Calibration monitor

Press:

```text
K
```

The calibration monitor lets you open the connected robot in CFG mode, send configuration commands, review output, and then restore normal runtime mode. It can use the active app port or a dedicated serial port.

### 15.5 Joystick

Press:

```text
J
```

This opens the joystick control window. The sketch uses Game Control Plus and JInput. The joystick system includes reconnection handling and profile options.

### 15.6 Leap Motion

Press:

```text
L
```

Leap Motion requires hardware mode. It can be toggled from Processing and from the web console shortcuts.

### 15.7 Camera controls

When the mouse is over the 3D viewport:

- Arrow keys rotate the scene.
- `+` zooms in.
- `-` zooms out.
- Mouse wheel also zooms.

## 16. Web console operation

### 16.1 Start order

Use this order:

1. Upload and connect the firmware if using real hardware.
2. Start the Processing sketch.
3. Confirm Processing WebSocket server is listening on port `9000`.
4. Open `software/web/SynROV.html`.
5. Set host to `localhost` and port to `9000`.
6. Click **Connect WebSocket**.
7. Click **Connect hardware** only if Processing has not already connected hardware.
8. Click **Sync state** if the page needs to refresh state.

### 16.2 Web console behavior

The web console follows the mode reported by Processing. It does not force robot switching by itself. This is intentional; Processing remains the authority for mode selection and hardware connection.

The page can:

- Display WebSocket connection state.
- Display selected mode and hardware status.
- Show viewport frames sent by Processing.
- Send manipulator joint snapshots.
- Send vehicle drive/camera snapshots.
- Send drone flight snapshots.
- Trigger quick actions.
- Toggle features such as joystick, Leap Motion, collision, and trace when supported.
- Convert approved runtime shortcut commands into structured WebSocket messages.

### 16.3 Supported runtime shortcut input

The web console maps selected text commands:

```text
READY?
HOME
TAKEOFF
LAND
ARMJ=...
```

Raw arbitrary serial commands are intentionally disabled in the web console. Use Processing calibration monitor or a serial terminal for full CFG access.

## 17. Python AiBot operation

### 17.1 Starting the AiBot

From `software/python`:

```bash
python synrov_aibot.py
```

or:

```bash
python -m synrov_aibot
```

The default bridge URI is:

```text
ws://127.0.0.1:9000/
```

Processing must be running first.

### 17.2 Python package organization

The Python package is modular at the public API level:

| Module | Purpose |
| --- | --- |
| `bridge.py` | WebSocket bridge classes |
| `config.py` | Constants, paths, robot keys, telemetry keys |
| `helpers.py` | Numeric, text, robot name, and pose helpers |
| `learning.py` | Dataset collection, feature extraction, training, inference helpers |
| `media.py` | Vision, webcam, image/audio feature helpers |
| `runtime.py` | Compatibility runtime exports |
| `state.py` | State dataclasses and pose conversion |
| `ui.py` | Tkinter UI classes |
| `voice.py` | Voice parser and audio recognition classes |

The large `_legacy.py` file preserves the original behavior and inheritance order. The smaller modules import and re-export pieces of that compatibility core.

### 17.3 Data folders

The Python package contains:

```text
synrov_aibot/synrov_multimodal_data/
  frames/
  models/
```

These folders are used for multimodal data, captured frames, datasets, and models.

### 17.4 Safe AiBot use

Use the AiBot first with Processing simulation mode. Once commands are predictable, test against real hardware with motors disabled or mechanically restrained. Only enable learned/autonomous behaviors on real hardware after verifying the baseline command mapping.

## 18. Typical first-run sequence

Use this sequence for a new installation:

1. Install Arduino, Processing, and Python dependencies.
2. Upload firmware.
3. Disconnect actuator power.
4. Start Processing.
5. Click connect.
6. Enter simulation if needed.
7. Open calibration monitor with `K`.
8. Enter CFG mode with `CFG=1234`.
9. Run `SHOW`.
10. Set the intended robot mode, for example `ROBOT=MANIPULATOR`.
11. Configure I2C addresses if needed.
12. Configure channel mappings and limits.
13. Save with `SAVE`.
14. Exit with `EXIT`.
15. Send `READY?` or reconnect through Processing.
16. Test low-risk commands.
17. Enable actuator power only after output direction and limits are verified.
18. Open the web console if remote control is needed.
19. Start Python AiBot only after Processing is stable.

## 19. Troubleshooting

### 19.1 Processing does not see the Arduino

Check:

- The Arduino port is not already open in the Arduino Serial Monitor.
- The board is connected over USB.
- The firmware was uploaded successfully.
- The baud rate is `115200`.
- The correct serial port is selected or discoverable.
- USB cable supports data, not only charging.

### 19.2 Firmware prints boot text but runtime does not start

Send:

```text
READY?
```

If you are in configuration mode, leave it first:

```text
EXIT
```

Then send `READY?` again.

### 19.3 Web console cannot connect

Check:

- Processing is running.
- Processing WebSocket server started without error.
- The web console host is `localhost` or the correct network address.
- The web console port is `9000`.
- Firewall settings allow local WebSocket connections.
- The path is `/`.

### 19.4 Python AiBot cannot connect

Check:

- Processing is running.
- WebSocket server is listening on `ws://127.0.0.1:9000/`.
- `websocket-client` is installed.
- No firewall is blocking the local connection.
- The Python UI is using the correct URI.

### 19.5 Servo moves in the wrong direction

Use the direction setting for that channel:

```text
DIR<n>=-1
```

Then test again at low speed and save only after confirming:

```text
SAVE
```

### 19.6 Servo exceeds safe range

Set channel minimum and maximum:

```text
MIN<n>=<angle>
MAX<n>=<angle>
```

Then test with small commands and save.

### 19.7 Stepper does not move

Check:

- `ROBOT=MANIPULATOR`
- `CH0MODE=STEPPER`
- Correct `STEPMODE`
- Coil wiring order
- Stepper power PWM pin and driver wiring
- Stepper feedback on `A0`
- `STEPDIAG`
- `STEPTEST=F,200,80,2200`

### 19.8 Stepper tuning fails

Run limit calibration first:

```text
LIMITSTEPPERCALIB
```

Make sure the two mechanical limit readings are far enough apart. Confirm `SPANRAW` is valid. Then run:

```text
AUTOSTEPPER
```

### 19.9 I2C devices are not found

Check:

- SDA/SCL wiring.
- Power and ground.
- Pull-up resistors if needed.
- Sensor voltage compatibility.
- I2C address conflicts.
- `PCAADDR`, `INAADDR`, `INA2ADDR`, `MPUADDR`, `MPU2ADDR`, and `MAGADDR`.

### 19.10 Drone motors behave unexpectedly

Immediately remove propellers and test with low-power ESC/motor setup only. Verify:

- ESC minimum/maximum/idle pulses.
- Motor order: FL, FR, RR, RL.
- Mixer signs for pitch, roll, and yaw.
- Command deadband.
- Spool slew rate.
- `STOP` and `LAND` behavior.

## 20. Development notes

### 20.1 Adding a new robot module in Processing

Processing uses a module interface in `ModuleContracts.pde`. A robot module implements:

```text
id()
displayName()
available()
supports(capability)
setupModule()
loadConfigSafe()
updateModule()
updatePhysicsSafe()
drawModule3D(advanceCamera)
resetInputStateSafe()
captureFrameSafe()
applyFrameSafe(frame)
moveSafelyTowardFrameSafe(targetFrame, maxStepDeg, toleranceDeg)
buildConfigObjectSafe()
```

Known modules are registered as Manipulator, Vehicle, and Drone. Missing modules fall back to `NullSynRovModule`.

### 20.2 Serial and WebSocket authority

Processing should be treated as the main runtime authority. The web console and Python AiBot send requests to Processing. Processing validates mode, ownership, and command context before relaying safe commands to firmware.

### 20.3 Recommended repository improvements

Before a public GitHub release, add:

- License file.
- Screenshots and hardware photos.
- Circuit diagrams.
- Exact library version list.
- Python `requirements.txt`.
- Processing library installation notes with tested versions.
- Known-good Arduino board and sensor module list.
- Safety test checklist.
- Example configuration dumps for each robot type.

## 21. Minimal command reference

### Boot/runtime

```text
BOOT?
BOOTRESET
READY?
CFG=1234
```

### CFG system

```text
SHOW
SAVE
LOAD
DEFAULT
ERASEEEPROM
EXIT
CFGOFF
ROBOT=MANIPULATOR
ROBOT=VEHICLE
ROBOT=DRONE
```

### Manipulator

```text
ARMJ=...
COLL=0/1
GSTAB=0/1
ASTAB=0/1
GPRESS=0/1
```

### Stepper

```text
CH0MODE=STEPPER
STEPMODE=HALFSTEP_BIPOLAR
LIMITSTEPPERCALIB
AUTOSTEPPER
AUTOSTATUS
AUTOSTOP
STEPDIAG
STEPTEST=F,200,80,2200
```

### Vehicle

```text
TRACK=l,r,CAM=p,t
MOVE=thr,steer
LIGHT=0/1
SCAN=0/1
```

### Drone

```text
FLY=t,y,p,r,s,f
CAMREC=0/1
TAKEOFF
LAND
STOP
```

### I2C/sensors

```text
PCAADDR=<0xNN>
INAADDR=<0xNN>
INA2ADDR=<0xNN>
MPUADDR=<0xNN>
MPU2ADDR=<0xNN>
MAGADDR=<0xNN>
MAGCAL=1
MAGCAL=0
```

## 22. Final operating advice

For day-to-day use, keep this workflow:

1. Start Processing first.
2. Connect hardware or simulation.
3. Confirm the selected robot mode.
4. Use low-power controls first.
5. Watch telemetry and diagnostics.
6. Use the web console or Python AiBot only after Processing is stable.
7. Save configuration only after a controlled test.
8. Document each hardware build separately, because pin sharing and sensor availability can change behavior across robot types.
