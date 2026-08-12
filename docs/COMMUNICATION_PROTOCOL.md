# SynROV Communication Protocol

SynROV uses one application protocol between external producers and Processing. The transport may change, but the envelope, message types, validation rules, control ownership, and safety path remain the same.

The current software release is identified only by the numeric field `softwareVersion: 1`. The version is not embedded in protocol names, schemas, variables, directories, persistent filenames, or ROS topics.

## Canonical envelope

```json
{
  "protocol": "synrov",
  "softwareVersion": 1,
  "messageType": "control",
  "source": "aibot",
  "timestampMs": 1786392000000,
  "seq": 42,
  "payload": {
    "control": {
      "robot": "Vehicle",
      "drive": {
        "throttle": 0.25,
        "steer": 0.0
      }
    }
  }
}
```

Required fields:

- `protocol`: always `synrov`;
- `softwareVersion`: numeric software version, currently `1`;
- `messageType`: semantic message type;
- `source`: logical producer (`web`, `aibot`, `ros`, or `processing`);
- `timestampMs`: timestamp in milliseconds;
- `seq`: producer sequence number;
- `payload`: JSON object matching the declared message type.

Processing accepts only the SynROV version-1 envelope and rejects packets with missing metadata, unknown producers, incompatible software versions, or mismatched `messageType` / payload combinations.

## External command types

| `messageType` | Expected `payload` content | Purpose |
| --- | --- | --- |
| `sync` | `{"sync": true}` | Request the current state snapshot. |
| `control` | `control` object | Unified movement/control block for all robots. |
| `camera` | `camera` object | Camera pan/tilt/stream control. |
| `mode` | `mode` string | Robot selection. |
| `joystick` | `joystickType` | Joystick profile selection. |
| `toggle` | `toggle` string | Allowed remote feature toggles. |
| `action` | `action` | HOME, takeoff, land, or indexed action. |
| `connect` | `{"connect": true}` | Request hardware connection. |

`client` is used only by the control WebSocket for Web/AiBot client lifecycle and heartbeat traffic. ROS uses the rosbridge connection state and does not send `client` lifecycle messages.

### Direction semantics

Control payloads already carry canonical semantic signs; Processing does not apply producer-specific direction flips. Vehicle `drive.steer` and `drive.pivot` are independent: negative means left, positive means right, and an explicit nonzero pivot has precedence when throttle is zero. Drone yaw uses positive = right/clockwise and negative = left; strafe uses positive = left and negative = right. Manipulator absolute joint targets are transported unchanged. The Processing keyboard is the physical reference for all signs; see `CONTROL_DIRECTION_AND_SAFE_HOME.md`.

## Messages emitted by Processing

Processing uses the same envelope with `source: "processing"`.

- `state`: primary runtime/state snapshot;
- `telemetry`: ROS-oriented telemetry payload;
- `perception.frame`: Processing 3D perception frame and context;
- `robot_camera.control`: start/stop/status control for the robot-camera stream;
- `heartbeat`: ROS integration heartbeat.

AiBot uses `perception.subscription` on port `9001` and `robot_camera.frame` on port `9002`, both with the same SynROV envelope.

## Transports

| Source / destination | Transport | Port | Application protocol |
| --- | --- | ---: | --- |
| HTML ↔ Processing | WebSocket | `9000` | SynROV envelope |
| AiBot ↔ Processing | WebSocket | `9000` | SynROV envelope |
| AiBot perception ↔ Processing | Dedicated WebSocket | `9001` | SynROV envelope |
| AiBot camera ↔ Processing | Dedicated WebSocket | `9002` | SynROV envelope |
| ROS ↔ Processing | rosbridge WebSocket | `9090` | SynROV envelope inside `std_msgs/String` |

Ports `9001` and `9002` are separated for bandwidth/latency isolation, not because they use different application protocols.

## Control ownership and safety

`web`, `aibot`, and `ros` are distinct producers. They enter the same Processing dispatcher and the same safety/collision path while retaining separate ownership identities. This prevents Processing from confusing AiBot with the browser or ROS with a WebSocket client.

The Arduino firmware continues to use its low-level serial/HEX runtime protocol. That interface is deliberately separate from the external application protocol because it serves different MCU, timing, and serial constraints.

## Language state

Processing publishes the active language-package code in state payloads using `language`, for example `"en"`, `"pt-br"`, `"es"`, or another installed package identifier. State metadata also includes `languageName`, `languageLocale`, and `languageAvailable` (a comma-separated list of package codes discovered by Processing).

The application protocol does not maintain a hard-coded language allow-list. AiBot and the HTML console resolve the Processing code against their installed packages and fall back to English if no matching package is available. The standalone HTML console starts in English before synchronization. See `LANGUAGE_PACKS.md`.

## Version-1 format

SynROV version 1 uses the envelope and stable schema names documented above. `softwareVersion: 1` is the compatibility marker used by Processing, Web, AiBot, and ROS-facing application messages.
