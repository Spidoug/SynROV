# Control Direction Contract and Safe HOME

SynROV uses one runtime coordinate system for every producer. Web, AiBot, ROS, joystick, Leap Motion and local Processing controls all populate the same canonical fields, and Processing does not reinterpret a sign according to source. Each control surface may use a different visual reference, so buttons, stick axes, keyboard keys and natural-language intents are mapped into the canonical fields before a command enters the protocol.

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

### Manipulator HOME

The V1 Manipulator HOME pose is `base=180, upper=150, fore=70, forearm_roll=90, wrist_pitch=95, wrist_roll=130, gripper=0`. Processing, AiBot and firmware use this same fixed seven-member pose; there is no second configurable HOME target. Processing is the motion authority for HOME. Before homing starts it cancels pending remote interpolation and action playback, clears a latched local collision-motion hold, and, when fresh hardware telemetry exists, seeds the trajectory from the measured pose. AiBot cancels an active Manipulator mission before issuing the same canonical `action: home` command.

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
| Gimbal pan left | `Z` | pan `+` |
| Gimbal pan right | `X` | pan `-` |
| Gimbal tilt down | `R` | tilt `-` |
| Gimbal tilt up | `F` | tilt `+` |

### Drone flight-state prerequisite

All control producers use the same flight-state machine: `grounded -> taking_off -> airborne -> landing`. Yaw and planar motion (`forward`, `strafe`, `pitch`, `roll`) are authorized as soon as TAKEOFF or a positive climb command starts the flight sequence; measured/local lift still determines the physical `airborne` state and automatic-takeoff completion. Vertical throttle is not filtered by the browser flight-ready state. A held Web `drone-down` command therefore reaches the same Processing vertical gate as every other producer. Processing blocks negative throttle only while the Drone is inside the landed zone; once measurable lift exists, a manual negative command can cancel automatic takeoff and descend, while a manual positive command can cancel automatic landing and climb. AiBot aerial missions and deferred movement commands continue once flight command authority is established.

## AiBot visual-reference policy

AiBot uses the same protocol fields as every other producer. Natural-language directions are resolved before transmission. Drone yaw phrases use the operator's visual reference: `yaw left` selects canonical positive yaw and `yaw right` selects canonical negative yaw. This is an input mapping only; `flight.yaw` is not redefined and `send_drone()` transports the selected numeric value unchanged. Positive climb and negative descend use the same canonical throttle field as Web and Processing.

## Joystick policy

Joystick profile types may change bindings, but they may not carry hidden semantic polarity changes. The version-1 joystick configuration schema is `synrov.joystick`. Default mappings normalize standard JInput right/down-positive axes so the physical stick direction matches the keyboard action (for example, stick up = `S/F/G` on the corresponding Manipulator vertical joints, Vehicle/Drone camera up = `F`, camera down = `R`, and Drone forward/left = `W/A`). Axis inversion remains available only as an explicit physical-controller calibration, for example the common joystick Y axis where pushing forward reports a negative raw value.

## Leap Motion policy

Leap Motion maps gestures into the same canonical semantic signs; any optional inversion flag is a hardware/gesture calibration only, never a robot-specific sign exception. Vehicle opposed-hand differential input with near-zero throttle becomes an explicit in-place pivot, matching `Q/E`, rather than being converted into a forward curve.

## HTML and WebSocket policy

WebSocket transports only canonical numeric values and absolute Manipulator targets; it does not know whether a value came from a keyboard, a mouse button or another visual control. The HTML keyboard keeps the Processing key reference. The on-screen yaw/pivot buttons use the operator's visual camera reference and are mapped before transmission: the left visual button selects canonical positive rotation and the right visual button selects canonical negative rotation for both Vehicle and Drone. This button mapping changes only the visual reference; the protocol fields and signs are unchanged. Vehicle `steer` and `pivot` remain independent fields.

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
