# ROS 2 / rosbridge Integration

Processing centralizes ROS integration in `software/processing/SynROV/RosIntegration.pde`. The bridge uses rosbridge WebSocket so the Processing application does not depend on native ROS Java bindings. The V1 interoperability target is the rosbridge 2.1 topic protocol: every wire object carries an `op`, topic registrations use full ROS 2 interface names, and publisher/subscriber registrations carry explicit IDs and QoS.

ROS uses the same application protocol described in `COMMUNICATION_PROTOCOL.md`. The software release appears only as `softwareVersion: 1` in the SynROV envelope.

## Configuration

ROS settings are stored in the normal Processing configuration, without a versioned directory or schema. Main parameters include:

- enable/disable ROS;
- rosbridge URI, normally `ws://127.0.0.1:9090`;
- SynROV namespace;
- whether ROS commands are accepted;
- publication interval and ownership timeout.

## Published topics

| Topic | ROS type | Content |
| --- | --- | --- |
| `/synrov/state` | `std_msgs/msg/String` | SynROV envelope with `messageType: "state"` |
| `/synrov/telemetry` | `std_msgs/msg/String` | SynROV envelope with `messageType: "telemetry"` |
| `/synrov/joint_states` | `sensor_msgs/msg/JointState` | Read-only native ROS joint-state view |
| `/synrov/bridge_heartbeat` | `std_msgs/msg/String` | Bridge heartbeat |

The configured namespace may change the `/synrov` prefix.

All topic registrations use `history=keep_last`, `reliability=reliable`, and `durability=volatile`. State, telemetry, joint state, and control use depth `10`; heartbeat uses depth `1`. QoS is explicit so the application does not inherit different publisher/subscriber policies from rosbridge defaults. The client performs one WebSocket connection attempt per reconnect cycle rather than probing the TCP endpoint separately first.

`/synrov/joint_states` uses a timestamped `sensor_msgs/msg/JointState`. The six arm/base joints are published in radians. The seventh entry, `gripper_finger`, is the modeled finger revolute articulation and is also published in radians. `velocity` and `effort` are empty when SynROV has no corresponding measurements.

## ROS command input

Commands enter only through `/synrov/control` as `std_msgs/msg/String`. The `data` field contains exactly the same envelope used by Web and AiBot, with `source` set to `ros`.

Vehicle movement example:

```json
{
  "protocol": "synrov",
  "softwareVersion": 1,
  "messageType": "control",
  "source": "ros",
  "timestampMs": 1786392000000,
  "seq": 1,
  "payload": {
    "control": {
      "robot": "Vehicle",
      "drive": {
        "throttle": 0.35,
        "steer": -0.15,
        "pivot": 0.0
      }
    }
  }
}
```

Drone selection example:

```json
{
  "protocol": "synrov",
  "softwareVersion": 1,
  "messageType": "mode",
  "source": "ros",
  "timestampMs": 1786392000100,
  "seq": 2,
  "payload": {
    "mode": "Drone"
  }
}
```

ROS may send the same external command types as Web/AiBot: `sync`, `control`, `camera`, `mode`, `joystick`, `toggle`, `action`, and `connect`.

## Single control path

There are no parallel command subscriptions for `/cmd_vel` or `/joint_targets`. This avoids competing control contracts. Every ROS command reaches `dispatchSynRovCommand()` and passes through the same validation, robot selection, ownership, collision guard, and hardware path used by other producers.

`/synrov/joint_states` remains a native ROS output because it is an interoperable telemetry representation, not a second control path.

## Ownership

ROS has `CONTROL_SOURCE_ROS`, independent from Web and AiBot. The ROS ownership timeout is based on configured command/bridge activity. A ROS command is never internally labeled as a WebSocket/Web producer.

## Version-1 command input

ROS command input uses `/synrov/control` with the canonical SynROV envelope. `/cmd_vel` and `/joint_targets` are not SynROV command inputs in version 1.
