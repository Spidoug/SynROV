# SynROV Operating Guide

## 1. Scope

This guide describes the SynROV repository according to the current code under `firmware/` and `software/`. The public protocol, configuration, and AiBot reference is **V1**, represented by `version: 1` and the V1 schemas listed below.

The Python AiBot package also declares version `1`, matching the project-wide V1 reference.

## 2. Components

### Arduino firmware

Directory: `firmware/SynROV_Firmware/`.

Main responsibilities:

- motor, servo, ESC, and peripheral I/O;
- telemetry and diagnostics;
- battery, current, IMU, sonar, and GPS handling;
- collision guard and safety routines;
- configuration and EEPROM persistence with magic/schema/CRC protection;
- robot-specific control for Manipulator, Vehicle, and Drone.

Relevant V1 references:

- `FIRMWARE_VERSION = 1`;
- `EEPROM_SCHEMA_ID = 1`;
- additional persistent blocks also use V1 schema identifiers.

`Serial0` operates at 115200 baud for communication with the station. `Serial1` is reserved for GPS when available.

### Processing

Directory: `software/processing/SynROV/`.

The Processing application is the main operator station and manages:

- serial connection and handshake;
- robot selection;
- simulation and 3D models;
- keyboard, joystick, and Leap Motion input;
- maps, environment, and diagnostics;
- persistent configuration;
- WebSocket server.

V1 files:

```text
data/v1/synrov.json
data/v1/joystick.json
data/v1/manipulator.json
data/v1/vehicle.json
data/v1/drone.json
```

Missing or invalid files are recreated with defaults. An existing file must declare the correct schema and `version: 1`.

### Web console

File: `software/web/SynROV.html`.

The console uses WebSocket, sends V1 JSON commands, and accepts only explicit V1 state packets. Browser preferences use `synrov.web.settings.v1` and `version: 1`.

Default endpoint:

```text
ws://localhost:9000/
```

### Python AiBot

Directory: `software/python/synrov_aibot/`.

There are two main execution modes:

- Tkinter interface (`python -m synrov_aibot`);
- dedicated runtime (`python -m synrov_aibot --modern ...`).

The dedicated runtime uses `RobotAIRegistry` and keeps three independent instances:

- `ManipulatorAI`;
- `VehicleAI`;
- `DroneAI`.

Persistence is stored under:

```text
software/python/synrov_aibot/synrov_multimodal_data/
```

This directory contains models, datasets, frames, profiles, and learned commands.

## 3. V1 contracts

| Area | Schema / version |
| --- | --- |
| WebSocket command | `synrov.control.v1`, `version: 1` |
| WebSocket state | `synrov.state.v1`, `version: 1` |
| Processing configuration | `synrov.config.v1`, `version: 1` |
| Robot configuration | `synrov.robot_config.v1`, `version: 1` |
| Joystick | `synrov.joystick.v1`, `version: 1` |
| Web configuration | `synrov.web.settings.v1`, `version: 1` |
| AiBot package | `version = "1"` / `__version__ = "1"` |
| AiBot dataset | `synrov.training.v1` |
| AiBot snapshot | `synrov.snapshot.v1` |
| Learned commands | `synrov.commands.v1`, `version: 1` |
| AiBot model | `synrov.model.v1`, `version: 1` |

Processing rejects WebSocket commands that do not include the `synrov.control.v1` schema and numeric version 1. The Web console and Python reject state packets without `synrov.state.v1` / `version: 1`.

## 4. WebSocket communication

### Command envelope

Minimal example:

```json
{
  "schema": "synrov.control.v1",
  "version": 1,
  "source": "client",
  "control": {
    "robot": "Vehicle",
    "drive": {
      "throttle": 0.2,
      "steer": 0.0
    }
  }
}
```

The Python `control_envelope()` helper forces `schema` and `version` to the V1 values, preventing callers from replacing those fields with another version.

### State

Every JSON object sent by Processing through `sendWebSocketJson()` receives the following metadata when needed:

```json
{
  "schema": "synrov.state.v1",
  "version": 1
}
```

V1 clients must require both fields.

## 5. Manipulator

### HOME

The Processing reference pose is:

| Joint | HOME |
| --- | ---: |
| Base | 180 deg |
| Upper | 150 deg |
| Fore | 70 deg |
| Forearm roll | 90 deg |
| Wrist pitch | 95 deg |
| Wrist rotation | 130 deg |
| Grip | 0 |

The dedicated Python runtime uses the same reference when updating its local state during the HOME action.

### Unified control

```json
{
  "schema": "synrov.control.v1",
  "version": 1,
  "control": {
    "robot": "Manipulator",
    "manipulator": {
      "angles": [180, 150, 70, 90, 95, 130, 0],
      "neutral": [false, false, false, false, false, false, false],
      "duty": [0, 0, 0, 0]
    }
  }
}
```

`neutral`, `duty`, and `autoTorque` are optional. Processing routes the pose through the same collision-safe path used by local control.

## 6. Vehicle

V1 control:

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

`throttle` and `steer` are clamped to `[-1, 1]`. `pivot` is also accepted: when throttle is effectively zero and pivot is non-zero, Processing performs an in-place rotation. Camera, light, and LiDAR settings can be updated in the same block.

## 7. Drone

V1 control:

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

All six motion axes are clamped to `[-1, 1]`. `takeoff` and `land` are separate V1 JSON actions.

## 8. Actions and system commands

In addition to the `control` block, the Processing dispatcher recognizes:

- client lifecycle (`client`);
- `sync`;
- `mode` selection;
- `camera` commands;
- `joystickType`;
- `toggle`;
- `action`;
- `connect`.

All messages must arrive inside the V1 envelope required by the dispatcher.

## 9. AiBot execution

Installation:

```bash
cd software/python
python -m pip install -r requirements.txt
```

Tkinter interface:

```bash
python -m synrov_aibot
```

Dedicated runtime:

```bash
python -m synrov_aibot --modern --uri ws://127.0.0.1:9000/
```

Dedicated runtime with a continuous mission loop:

```bash
python -m synrov_aibot --modern --uri ws://127.0.0.1:9000/ --loop
```

The dedicated runtime receives the active robot and language from Processing, synchronizes the matching AI instance, and cancels previous autonomy when the active robot changes.

## 10. AiBot V1 persistence

### Dataset

`dataset.py` stores `schema_version` as:

```text
synrov.training.v1
```

### Snapshot

`perception.py` identifies snapshots as:

```text
synrov.snapshot.v1
```

### Learned commands

`learned_commands.json` uses this envelope:

```json
{
  "schema": "synrov.commands.v1",
  "version": 1,
  "robots": {
    "Manipulator": {},
    "Vehicle": {},
    "Drone": {}
  }
}
```

Content without this envelope is not loaded as V1 learned-command memory.

### Models

Trained packages include:

```json
{
  "schema": "synrov.model.v1",
  "version": 1
}
```

The loader rejects packages without this metadata or packages explicitly associated with a different robot.

## 11. Recommended startup

1. Flash `SynROV_Firmware.ino` to an Arduino Mega 2560 or compatible board.
2. Open `software/processing/SynROV/SynROV.pde`.
3. Connect the hardware or enable simulation.
4. Select the correct robot.
5. Check battery, current, sensors, and telemetry.
6. Validate HOME and STOP/HOVER before unrestricted motion.
7. Open `software/web/SynROV.html` when the Web console is required.
8. Start AiBot when voice, vision, learning, or mission features are required.

## 12. Software validation

Python:

```bash
cd software/python
python -m compileall -q synrov_aibot
```

Web:

- extract the contents of the `<script>` block from `software/web/SynROV.html`;
- run `node --check` on the extracted JavaScript.

Firmware and Processing must be compiled with their respective toolchains and required libraries. Hardware bench testing remains mandatory before real operation.

## 13. Bench checklist

- correct robot profile selected;
- firmware reporting version 1;
- WebSocket using V1 schema and version;
- Processing configuration files using the correct schema and `version: 1`;
- HOME confirmed;
- motor/ESC directions confirmed at low power;
- STOP/HOVER validated;
- sensors and battery readings consistent;
- Manipulator collision guard validated;
- camera and auxiliary servos operating within expected ranges;
- AiBot connected to the same Processing endpoint.
