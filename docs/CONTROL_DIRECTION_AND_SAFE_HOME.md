# Control Direction Contract and Safe HOME

SynROV uses the Processing keyboard as the single physical direction contract. Other producers (joystick, Leap Motion, HTML/WebSocket, and AiBot) must map into the same semantic signs before commands reach the runtime bridge.

## Canonical direction signs

### Manipulator

| Physical action | Processing key | Canonical change |
|---|---:|---:|
| Base left | `Z` | base yaw `-` |
| Base right | `X` | base yaw `+` |
| Upper arm up | `S` | upper angle `+` |
| Upper arm down | `W` | upper angle `-` |
| Forearm up | `F` | forearm angle `+` |
| Forearm down | `R` | forearm angle `-` |
| Forearm roll left | `A` | roll angle `+` |
| Forearm roll right | `D` | roll angle `-` |
| Wrist up | `G` | wrist vertical angle `+` |
| Wrist down | `T` | wrist vertical angle `-` |
| Wrist rotate left | `Q` | wrist rotation angle `+` |
| Wrist rotate right | `E` | wrist rotation angle `-` |
| Gripper open | `Y` | gripper `+` |
| Gripper close | `H` | gripper `-` |

Manipulator base yaw is absolute: `0 deg = North`, and positive yaw increases clockwise. Absolute WebSocket angles are never sign-flipped.

### Vehicle

| Physical action | Processing key | Canonical sign |
|---|---:|---:|
| Forward | `W` | throttle `+` |
| Reverse | `S` | throttle `-` |
| Gentle left | `A` | steer `-` |
| Gentle right | `D` | steer `+` |
| Pivot left | `Q` | pivot `-` |
| Pivot right | `E` | pivot `+` |
| Camera left | `Z` | pan `+` |
| Camera right | `X` | pan `-` |
| Camera down | `R` | tilt `-` |
| Camera up | `F` | tilt `+` |

Steer and pivot are separate channels. A zero-throttle steer command is never silently converted to pivot. Explicit pivot has precedence when active.

### Drone

| Physical action | Processing key | Canonical sign |
|---|---:|---:|
| Yaw right | `Q` | yaw `+` |
| Yaw left | `E` | yaw `-` |
| Forward | `W` | forward `+` |
| Backward | `S` | forward `-` |
| Strafe left | `A` | strafe `+` |
| Strafe right | `D` | strafe `-` |
| Climb | `T` | throttle/altitude `+` |
| Descend | `G` | throttle/altitude `-` |
| Gimbal pan right | `Z` | pan `+` |
| Gimbal pan left | `X` | pan `-` |
| Gimbal tilt down | `R` | tilt `-` |
| Gimbal tilt up | `F` | tilt `+` |

### Drone flight-state prerequisite

All control producers use the same physical flight-state machine: `grounded -> taking_off -> airborne -> landing`. Yaw and planar motion (`forward`, `strafe`, `pitch`, `roll`) stay locked until measured/local physical lift reaches the flight-ready threshold; a requested target altitude alone never counts as airborne. Automatic takeoff and landing own the vertical axis until their physical completion condition is reached, so a simultaneous Web/joystick/Leap update cannot cancel the vertical sequence. AiBot aerial missions and deferred movement commands wait for `flightReady` before continuing.

## Joystick policy

Joystick profile types may change bindings, but they may not carry hidden semantic polarity changes. The version-1 joystick configuration schema is `synrov.joystick`. Default mappings normalize standard JInput right/down-positive axes so the physical stick direction matches the keyboard action (for example, stick up = `S/F/G` on the corresponding Manipulator vertical joints, Vehicle/Drone camera up = `F`, camera down = `R`, and Drone forward/left = `W/A`). Axis inversion remains available only as an explicit physical-controller calibration, for example the common joystick Y axis where pushing forward reports a negative raw value.

## Leap Motion policy

Leap Motion maps gestures into the same canonical semantic signs; any optional inversion flag is a hardware/gesture calibration only, never a robot-specific sign exception. Vehicle opposed-hand differential input with near-zero throttle becomes an explicit in-place pivot, matching `Q/E`, rather than being converted into a forward curve.

## HTML and WebSocket policy

The HTML keyboard follows the same key assignments as Processing, and directional control buttons show the corresponding Processing key where practical. Arrow-key motion aliases are not part of the version-1 control contract. Vehicle `steer` and `pivot` are independent fields. WebSocket transports canonical semantic values and absolute Manipulator targets; it does not apply direction inversion.

## Safe GPS HOME lifetime

A runtime connection defines one safe HOME session:

1. When runtime operation opens, HOME session state is created in RAM.
2. The first fresh GPS fix becomes the HOME origin for the session. Capture does not depend on movement, takeoff, selected robot mode, or whether automatic RTH is enabled.
3. Landing, stopping, or changing robot mode does not replace or erase HOME while the connection remains active.
4. When the runtime connection ends or enters configuration/calibration mode, the RAM HOME session is cleared.

Vehicle HOME remains RAM-only.

Drone HOME also remains RAM-only during normal operation. When battery becomes strictly lower than `10%`, the current session HOME is written once to the emergency EEPROM slot. This is an emergency recovery snapshot, not the normal HOME store.

If the controller reboots while the Drone is still below `10%`, a valid emergency snapshot may be restored so low-battery return logic can retain the safe origin. A healthy connection (`>= 10%`) always uses its own first fresh GPS fix and invalidates a stale emergency snapshot after the new RAM HOME is captured.

The configurable automatic return threshold (default `15%`) is separate from the fixed emergency HOME persistence threshold (`< 10%`).
