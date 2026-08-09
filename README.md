# SynROV

SynROV is an integrated platform for three robot profiles: **Manipulator**, **Vehicle**, and **Drone**. The repository combines Arduino firmware, a Processing 3D operator station, a browser-based WebSocket console, and the Python **AiBot** runtime.

The public protocol, configuration, and AiBot version reference for the project is **V1** (`version: 1`). V1 contracts are explicit: WebSocket messages, configuration files, datasets, snapshots, learned commands, and model packages use V1 identifiers. The Python AiBot package also declares version `1`.

> **Safety:** the firmware controls motors, servos, and ESCs. Perform the first validation with limited power, the mechanism supported, and the work area clear. Confirm the robot profile, limits, motor directions, and physical mapping before enabling full power.

## Repository structure

```text
firmware/SynROV_Firmware/
  SynROV_Firmware.ino
  FirmwareDeclarations.h
  01_CRC_And_EEPROM.h
  02_Initialization.h
  03_Motor_Low_Level.h
  04_Servo_And_PWM_Output.h
  05_Config_Defaults_And_Sync.h
  06_Runtime_Commands_And_Diagnostics.h
  07_Gps_Serial1.h
  07_Serial_Control_And_Telemetry.h
  08_Setup_And_Loop.h
  09_Safety_Autonomy.h

software/processing/SynROV/
  SynROV.pde
  CoreJsonUtils.pde
  ModuleContracts.pde
  Logic_RuntimeBridge.pde
  DiagnosticsPanel_Base.pde
  EnvironmentMapping.pde
  Manipulator3D.pde
  Vehicle3D.pde
  Drone3D.pde

software/web/
  SynROV.html

software/python/
  synrov_aibot.py
  pyproject.toml
  requirements.txt
  synrov_aibot/
    activity.py
    actuators.py
    auto_torque.py
    autonomy.py
    bridge.py
    command_contract.py
    config.py
    core.py
    dance.py
    dataset.py
    helpers.py
    launcher.py
    learning.py
    main.py
    media.py
    modern_runtime.py
    orchestrator.py
    perception.py
    policy.py
    primitives.py
    protocol.py
    replay.py
    robot_ai.py
    runtime.py
    safety.py
    state.py
    trainer.py
    ui.py
    voice.py
```

## V1 version reference

| Area | V1 reference |
| --- | --- |
| Public firmware version | `FIRMWARE_VERSION = 1` |
| Main EEPROM schema | `EEPROM_SCHEMA_ID = 1` |
| WebSocket command | `synrov.control.v1` + `version: 1` |
| WebSocket state | `synrov.state.v1` + `version: 1` |
| Processing configuration | `synrov.config.v1` + `version: 1` |
| Robot configuration | `synrov.robot_config.v1` + `version: 1` |
| Joystick configuration | `synrov.joystick.v1` + `version: 1` |
| Web console settings | `synrov.web.settings.v1` + `version: 1` |
| AiBot package | `version = "1"` / `__version__ = "1"` |
| AiBot dataset | `synrov.training.v1` |
| AiBot snapshot | `synrov.snapshot.v1` |
| Learned commands | `synrov.commands.v1` + `version: 1` |
| AiBot model package | `synrov.model.v1` + `version: 1` |

Processing requires the V1 schema and numeric version in WebSocket commands. The Web console and AiBot require `synrov.state.v1` with `version: 1`. Processing configuration files must also explicitly declare version 1; missing or invalid files are recreated with V1 defaults.

## Firmware

The firmware declares `FIRMWARE_VERSION = 1`. The Arduino uses `Serial0` at **115200 baud** for control/telemetry and `Serial1` for GPS when available. Processing manages the serial handshake and runtime transport.

Main firmware areas:

- **Manipulator:** joint control, gripper, PWM/torque, collision guard, and stabilization;
- **Vehicle:** traction, steering, pivot, camera, lighting, scan/LiDAR, tilt assistance, and safety return;
- **Drone:** ESCs, attitude/motion, camera, takeoff/landing, and safety return;
- battery, current, IMU, sonar, GPS, and actuator sensing/telemetry;
- EEPROM protected by magic values, schema/layout identifiers, and CRC.

EEPROM identifiers are internal persistence identifiers and do not replace the public V1 version.

## Processing

The main station is `software/processing/SynROV/SynROV.pde`. It provides:

- selection between Manipulator, Vehicle, and Drone;
- serial connection and firmware handshake;
- simulation and 3D visualization;
- keyboard, joystick, and Leap Motion input;
- diagnostics, maps, and environment tools;
- persistent configuration;
- WebSocket server for the Web console and AiBot.

Persistent V1 files are stored under `software/processing/SynROV/data/v1/`:

```text
data/v1/synrov.json
data/v1/joystick.json
data/v1/manipulator.json
data/v1/vehicle.json
data/v1/drone.json
```

## Web console

Open `software/web/SynROV.html`. The default endpoint is:

```text
ws://localhost:9000/
```

The console sends commands using the `synrov.control.v1` / `version: 1` envelope and accepts only `synrov.state.v1` / `version: 1` state packets. The browser does not expose raw serial command transmission.

## Python AiBot

Requires Python **3.10+**.

```bash
cd software/python
python -m pip install -r requirements.txt
```

Tkinter interface:

```bash
python -m synrov_aibot
```

Dedicated WebSocket runtime without the Tkinter interface:

```bash
python -m synrov_aibot --modern --uri ws://127.0.0.1:9000/
```

Dedicated runtime with continuous mission ticks:

```bash
python -m synrov_aibot --modern --uri ws://127.0.0.1:9000/ --loop
```

The dedicated architecture uses `RobotAIRegistry`, which keeps independent `ManipulatorAI`, `VehicleAI`, and `DroneAI` instances. Each robot has its own mission and model slot. Learned-command memory is separated by canonical robot name inside the V1 file.

Python persistence is stored under:

```text
software/python/synrov_aibot/synrov_multimodal_data/
```

Main runtime files and directories include:

```text
synrov_multimodal_data/learned_commands.json
synrov_multimodal_data/aibot_ai_profiles_v1.json
synrov_multimodal_data/models/
synrov_multimodal_data/datasets/
synrov_multimodal_data/frames/
```

## WebSocket control formats

Manipulator:

```json
{
  "schema": "synrov.control.v1",
  "version": 1,
  "control": {
    "robot": "Manipulator",
    "manipulator": {
      "angles": [180, 150, 70, 90, 95, 130, 0]
    }
  }
}
```

Vehicle:

```json
{
  "schema": "synrov.control.v1",
  "version": 1,
  "control": {
    "robot": "Vehicle",
    "drive": {
      "throttle": 0.0,
      "steer": 0.0,
      "pivot": 0.0,
      "camPan": 0.0,
      "camTilt": 0.0,
      "lights": false,
      "lidarScan": true
    }
  }
}
```

Drone:

```json
{
  "schema": "synrov.control.v1",
  "version": 1,
  "control": {
    "robot": "Drone",
    "flight": {
      "throttle": 0.0,
      "yaw": 0.0,
      "pitch": 0.0,
      "roll": 0.0,
      "strafe": 0.0,
      "forward": 0.0,
      "camPan": 0.0,
      "camTilt": 0.0,
      "cameraStreaming": false
    }
  }
}
```

Actions such as `home`, `takeoff`, and `land`, mode selection, synchronization, and toggles are also sent as V1 JSON messages.

## Manipulator HOME

The reference pose used by Processing is:

```text
Base          180 deg
Upper         150 deg
Fore           70 deg
Forearm roll   90 deg
Wrist pitch    95 deg
Wrist rot     130 deg
Grip            0
```

The dedicated Python runtime uses the same reference when updating its local state during the HOME action.

## Local validation

Python:

```bash
cd software/python
python -m compileall -q synrov_aibot
```

The Web console JavaScript can be checked by extracting the `<script>` block and running `node --check` on the extracted file.

Firmware and Processing must be compiled with Arduino/Processing toolchains compatible with the target hardware and installed libraries. Static validation does not replace bench testing.

## Recommended startup flow

1. Flash `firmware/SynROV_Firmware/SynROV_Firmware.ino` to an Arduino Mega 2560 or compatible board.
2. Open `software/processing/SynROV/SynROV.pde` in Processing.
3. Connect the hardware or enable simulation.
4. Select the correct profile: Manipulator, Vehicle, or Drone.
5. Validate STOP/HOVER, HOME, limits, sensors, and actuator directions.
6. Open the Web console when browser control is required.
7. Start AiBot when voice, vision, missions, or learning features are required.

For detailed operation, see `docs/SynROV_Operating_Guide.md`.
