# SynROV AiBot Robot Intelligence Orchestration

## Purpose

AiBot must not treat Manipulator, Vehicle, and Drone as the same machine with different actuator names. Each robot has a different physical task, sensor geometry, risk profile, and useful interaction model. SynROV therefore separates four concepts:

1. **control ownership** — who is currently allowed to move the robot;
2. **physical perception** — what the robot's real sensors and camera report;
3. **Processing world context** — what the reconstructed/imported 3D world contributes;
4. **robot intelligence context** — the fused, robot-specific input used by missions, vision, learning, and autonomous decisions.

Processing publishes `synrov.robot-sensors`. AiBot fuses it with the control/state channel, the dedicated 3D perception stream, the configured robot-camera source, and AiBot-local audio into `synrov.robot-intelligence`.

## Transport and authority

- `ws://localhost:9000/` carries the canonical SynROV control/state envelope.
- `ws://localhost:9001/` carries `perception.frame` inside the canonical SynROV envelope: a clean 3D viewport plus synchronized structured world context.
- `ws://localhost:9002/` is reserved for the Vehicle/Drone AiBot camera-view transport used by the Processing operator view. The physical camera is preferred when available; otherwise AiBot relays the current SynROV perception frame or a generated fallback so the channel remains valid. It is independent from both control and the digital-world perception channel.
- Physical sensor telemetry remains authoritative for real robot state when Firmware telemetry is selected.
- Local telemetry remains authoritative for the commanded/simulated pose when Local telemetry is selected.
- The robot's physical camera is preferred for object recognition and navigation vision whenever it is available. The Processing 3D view is complementary context and a simulation/training fallback, not a replacement for the embedded camera.

## Orchestration priority

The arbiter applies the same safety hierarchy to all robots:

1. emergency and safety actions, including explicit spoken safety actions;
2. direct human control — joystick, keyboard, Leap Motion, or operator Web control;
3. ordinary voice commands;
4. active missions;
5. vision/object actions;
6. music/audio-generated actions;
7. model/autonomy output;
8. curiosity/background exploration.

Direct human input is treated as temporary ownership. AiBot observes the resulting telemetry and can store it as a trusted human demonstration, but autonomous output cannot fight the operator while that input is active.

## Manipulator orchestration

### Primary inputs

- **Workspace camera:** primary visual source. It should see the arm/gripper and the surrounding workspace so object pose can be related to the manipulator itself.
- **Joint telemetry:** primary kinematic state for all seven controlled members.
- **MPU2 at the base:** structural attitude source for the complete manipulator. It is the IMU used to tilt the whole 3D arm relative to the level world grid.
- **MPU1 at the gripper/wrist:** end-effector attitude and motion source. Both MPU1 and MPU2 are retained as independent AiBot features.
- **Per-joint torque references:** Processing preserves the existing named torque/current channels (`Base torque sensor`, `Upper arm torque sensor`, `Forearm vertical torque sensor`, `Forearm rotational torque sensor`, `Wrist group current 1/2`, and gripper pressure/current where available). Current/raw values are interpreted as proportional torque references, not calibrated N.m unless a hardware calibration curve is provided.
- **Compass:** absolute heading / north-reference sensor. `heading_deg` is retained in the stable learned telemetry vector and raw magnetometer axes are carried in the enriched context when firmware provides them.
- **Battery:** energy state used by mission policy, model features, and resource guards.
- **Sonar:** near-field clearance context.
- **Processing 3D world:** imported-world geometry, collision samples, robot pose, and clean scene frame.
- **Keyboard / joystick / Leap Motion:** direct operator control and trusted demonstrations.
- **Voice:** command and mission input.
- **Music/rhythm:** interaction and actuation input only for Manipulator, and only when the rhythm mode has been explicitly enabled and no human controller owns the robot.

### Intelligence use

Object manipulation should fuse workspace-camera observations with joint state, MPU2 base attitude, MPU1 gripper attitude, and the named per-joint torque/load references before generating a grasp or placement target. The 3D world can add collision/context information, but real sonar/current and Processing collision guards remain higher-authority safety sources.

Human movement from Leap Motion, joystick, or keyboard remains high-authority operator input and can be used as future demonstration data. The current persistent mission learner does **not** imitate raw operator actuator trajectories; it learns only among bounded mission-strategy variants so it cannot invent unsafe joint or motor commands.

### Processing attitude model

With Firmware telemetry selected, the base IMU tilts the **complete arm structure** around its base reference. A small local support surface tilts with the manipulator so the measured base inclination is visually readable against the fixed global X/Z grid. Joint rotations remain relative to that tilted base.

The imported/global world and its main grid remain level. This makes it possible to see that the robot base is physically inclined rather than incorrectly rotating the entire world.

## Vehicle orchestration

### Primary inputs

- **Embedded front camera:** primary forward navigation and object/terrain vision.
- **Chassis IMU:** pitch, roll, yaw/heading and stability context.
- **Position telemetry:** local Processing/firmware position and motion state.
- **LiDAR:** primary short-range obstacle and clearance sensor.
- **GPS:** primary global-position/navigation source when a valid fix exists.
- **Compass:** primary absolute-heading / north-reference navigation sensor, independent from the chassis IMU attitude channels.
- **Drive torque references:** left/right movement-load references from the torque channels already named by Processing.
- **Battery:** energy budget for patrol, inspection, object search, and return decisions.
- **Communication link:** RSSI/explicit link quality when provided; otherwise AiBot may use `link_ms` only as a clearly labeled quality proxy.
- **Processing 3D world:** route/world geometry and supplementary relative obstacle context.
- **Keyboard / joystick / Leap Motion:** direct operator control and trusted demonstrations.
- **Voice:** command and mission input.
- **Music/audio:** context only; it does not directly steer the Vehicle.

### Intelligence use

Vehicle navigation should use GPS for global intent, the compass for absolute heading, the IMU for pitch/roll/motion attitude, the front camera for semantic scene understanding, LiDAR for local free-space/safety, drive-torque references for traction/load reasoning, and battery/link quality as mission resources. Processing 3D obstacles may supplement those inputs, but must not override a real LiDAR stop condition.

When direct human control is active, AiBot observes camera, LiDAR, GPS, IMU, and resulting position together with the operator action. This creates robot-specific demonstrations suitable for later policy training without granting the model simultaneous actuator ownership.

### Processing attitude model

With Firmware telemetry selected, the chassis IMU tilts the complete Vehicle model and a small local chassis/support surface relative to the fixed global grid. Local command-generated pitch/roll animation is suppressed while measured IMU attitude is authoritative so the same visual degree of freedom is not applied twice.

## Drone orchestration

### Primary inputs

- **Embedded front camera:** primary forward flight/navigation vision.
- **Flight IMU:** pitch, roll, yaw and flight-attitude state.
- **Position and altitude telemetry:** local flight position plus vertical state.
- **Downward sonar:** primary near-ground altitude/clearance safety input.
- **GPS:** primary global-position/navigation source when a valid fix exists.
- **Compass:** primary absolute-heading / north-reference navigation sensor, used together with the flight IMU rather than being hidden inside IMU yaw.
- **Battery:** energy budget used for mission range, return, and stop/land policy.
- **Communication link:** RSSI/explicit quality when available, with `link_ms` as a labeled fallback proxy.
- **Processing 3D world:** supplementary spatial context and relative obstacle samples.
- **Keyboard / joystick / Leap Motion:** direct operator control and trusted demonstrations.
- **Voice:** command and mission input, including safety commands such as hover or emergency land.
- **Music/audio:** context only; it never directly commands flight.

### Intelligence use

Drone navigation should combine GPS route intent, compass absolute heading, IMU flight attitude, front-camera scene understanding, sonar ground clearance, battery reserve, and communication-link quality. The Processing 3D context is useful for simulated training and additional obstacle awareness, but real flight telemetry and safety sensors keep authority.

### Processing attitude model

The global ground/grid remains level. Pitch and roll rotate only the airborne Drone body around its own center, matching the physical behavior of an aircraft above a fixed world reference. With Firmware telemetry selected, the measured IMU attitude drives this representation; with Local telemetry selected, the local flight model remains authoritative.

## Audio and sound policy

AiBot owns the microphone locally. The voice path captures 16 kHz mono PCM through `sounddevice`, segments phrases with local energy/VAD logic, and transcribes them through `SpeechRecognition`. Processing remains the source of truth for the UI language-package code; a language change makes AiBot resolve the matching installed package and updates both the Command Center and the package-defined speech-recognition locale without restarting the microphone.

Speech recognition produces command/mission intents. Explicit safety speech is elevated to the safety lane; ordinary motion speech remains below direct human controls. Audio-level/features remain available to learning and interaction logic. Rhythm-driven actuation is restricted to Manipulator; Vehicle and Drone can retain audio features as contextual data without converting music into motion.

## Camera fusion policy

Each robot has a physical camera role:

- Manipulator: `workspace_camera`;
- Vehicle: `front_camera`;
- Drone: `front_camera`.

AiBot marks that channel available only when its configured robot-camera source is producing frames. Local camera selection is centralized in `camera_devices.py`: the GUI exposes Auto plus validated devices, and the dedicated runtime uses the same implementation. Physical imagery is preferred by the object/navigation vision pipeline. The Processing 3D frame remains available in parallel through port 9001 for world geometry, imported-scene awareness, synthetic training, and debugging.

The two views are intentionally not collapsed into one source: a physical camera reports what the robot can actually see, while Processing reports what the digital world model knows.

## Structured context

Processing source manifest:

```json
{
  "schema": "synrov.robot-sensors",
  "version": 1,
  "robot": "Vehicle",
  "channels": [],
  "values": {},
  "inputs": {}
}
```

AiBot fused context:

```json
{
  "schema": "synrov.robot-intelligence",
  "version": 1,
  "robot": "Vehicle",
  "profile": {},
  "control": {},
  "audio": {},
  "channels": [],
  "features": {},
  "camera_fusion": {}
}
```

This split keeps Processing responsible for robot/world state and input ownership, while AiBot remains responsible for sensor fusion, learning policy, perception preference, command arbitration, missions, and autonomous decisions.

The robot-specific telemetry filter also feeds these task-relevant channels into the intelligence context used by mission decisions and learning rewards: all three robots retain the existing `heading_deg` compass field; Manipulator additionally uses joints, base attitude, workspace range/current and vision context; Vehicle uses pose, chassis attitude, LiDAR, GPS, camera pose and safety state; Drone uses pose/altitude, flight attitude, downward sonar, GPS, camera pose and safety state. GPS values are normalized from the runtime GPS block before strategy evaluation.

## Persistent bounded mission learning

Each robot owns a persistent policy at `runtime_data/models/<robot>/<robot>_policy.pkl` and an experience trail at `runtime_data/datasets/<robot>_training.jsonl`. The learner uses a bounded UCB strategy selector: during trainable search/inspection missions it changes strategy only at controlled time segments, tries each approved variant, scores the result from available vision, clearance/collision information, battery/link state and mission success, and reuses higher-performing variants in later segments and later mission runs.

This is deliberately **strategy learning rather than raw motor exploration**. The set of allowable variants is declared in code per robot and mission. Pick/place, emergency actions, direct intents and origin-navigation safety logic do not receive arbitrary exploratory actuator variants. Processing collision/ownership checks and firmware safety remain authoritative regardless of learned score. The AiBot log exposes the active strategy, reward and observation count so the operator can see when the system is exploring versus reusing learned behavior.

Manipulator learns bounded scan/inspection patterns using workspace vision as an outcome signal. Vehicle learns bounded patrol, corridor, terrain, visual-search and exit-selection behavior. Drone learns bounded aerial scan, orbit, search, terrain and 3D exit behavior. Vision-guided `follow_target` on Vehicle reacts to the observed target and performs a reacquisition sweep when the target is lost instead of following only a fixed sinusoidal command.

## Specialty mission strategies

### Vehicle

- **Terrain inspection:** systematic ground sweep with conservative speed, front-camera scanning, LiDAR collision authority, GPS/pose tracking, drive-load observation, and battery/link resource limits.
- **Locate objects:** front-camera target search and approach; LiDAR and the Processing 3D world constrain the path. The mission stops when a target is centered and close enough according to the active vision observation.
- **Discover exits:** compares left/front/right free corridors using LiDAR/Processing 3D obstacle context and drives toward the least occupied corridor.

### Drone

- **Terrain inspection:** aerial lawn-mower survey bands using GPS/pose, flight IMU, front camera, downward sonar, battery reserve, and link quality.
- **Locate objects:** yaw/strafe visual search, then camera-centered approach while maintaining sonar/IMU safety.
- **Discover exits:** evaluates a three-dimensional free corridor. Unlike the Vehicle, the Drone may choose lateral avoidance or climb when the forward path is blocked.

Battery and communication quality are not presentation-only values. They are part of the AiBot resource policy and model feature vector. At critical values, exploration does not continue as if the resource were healthy; Vehicle is held/stopped and Drone is held or landed according to the applicable safety/mission path.

## Contextual planning and long-context memory

AiBot resolves language in layers: robot-specific taught aliases, canonical catalog aliases, successful long-context episodes, and semantic goal matching. It can build explicit multi-step plans from sequencing language such as `depois`/`then`; a plain `e`/`and` is treated as a sequence only when every clause independently resolves to an audited skill. This allows novel goal phrasing and useful composition without generating arbitrary actuator output.

Long-context episodes are persisted in `runtime_data/long_context.jsonl` and remain isolated by canonical robot. Each compact episode stores the resolved skill, outcome, selected strategy metadata, and a bounded robot-state snapshot. Learned phrase aliases are likewise available independently for Manipulator, Vehicle, and Drone. Safety arbitration, collision handling, flight prerequisites, firmware constraints, and the audited skill catalog remain authoritative over both learned and semantic decisions.

Vehicle and Drone `return_home` use the Processing-local pose/heading contract and navigate toward the runtime origin instead of using timed reverse motion. The Vehicle aligns to the origin vector before approach; the Drone waits for physical flight readiness, aligns yaw, approaches the origin, and hovers on arrival.
