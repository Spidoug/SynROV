# SynROV AiBot — Robot Commands and Intentions

> Base-coordinate convention: physical feedback remains `0° = North` and increases clockwise. All maintained control sources use the same directional rule: `base_left` decreases the numeric yaw target and `base_right` increases it. Absolute degree targets are passed through without directional inversion.

This reference follows the same current robot catalog used by the AiBot command resolver and the **Intentions** tab. SynROV does not use one global command set; each robot exposes commands and missions appropriate to its physical role.

## Manipulator

### Direct commands

| Command | Intention | Description |
| --- | --- | --- |
| turn base left | `base_left` | Rotates only the base to the left. |
| turn base right | `base_right` | Rotates only the base to the right. |
| raise arm | `arm_up` | Raises the upper arm. |
| lower arm | `arm_down` | Lowers the upper arm. |
| raise forearm | `fore_up` | Raises the forearm. |
| lower forearm | `fore_down` | Lowers the forearm. |
| wrist up | `wrist_up` | Tilts the wrist upward. |
| wrist down | `wrist_down` | Tilts the wrist downward. |
| wrist left | `wrist_left` | Rotates the wrist to the left. |
| wrist right | `wrist_right` | Rotates the wrist to the right. |
| forearm roll left | `roll_left` | Rolls the forearm to the left. |
| forearm roll right | `roll_right` | Rolls the forearm to the right. |
| open gripper | `grip_open` | Opens the gripper. |
| close gripper | `grip_close` | Closes the gripper. |
| home position | `home` | Returns the Manipulator to its home position. |

### Autonomous missions

| Command | Intention | Description |
| --- | --- | --- |
| inspect workspace | `inspect_workspace` | Inspects the work area using sensors and vision. |
| 360 workspace scan | `scan_workspace` | Performs a complete workspace scan. |
| pick object | `pick_object` | Locates, approaches, and grasps an object. |
| place object | `place_object` | Positions and releases an object. |
| hold pose | `hold_pose` | Maintains the current pose. |
| return home | `return_home` | Autonomously returns to the home pose. |
| calibrate gripper | `calibrate_gripper` | Runs a safe gripper calibration cycle. |
| rhythm mode | `rhythm_mode` | Allows music-driven motor response only on Manipulator. |
| wave | `wave` | Performs a waving gesture. |
| stop mission | `stop_mission` | Cancels the active mission. |

## Vehicle

### Direct commands

| Command | Intention | Description |
| --- | --- | --- |
| move forward | `forward` | Drives the Vehicle forward. |
| reverse | `back` | Drives the Vehicle backward. |
| steer left | `left` | Steers the Vehicle to the left. |
| steer right | `right` | Steers the Vehicle to the right. |
| stop vehicle | `stop` | Stops Vehicle motion. |
| camera left | `camera_left` | Moves the camera gimbal to the left. |
| camera right | `camera_right` | Moves the camera gimbal to the right. |
| camera up | `camera_up` | Moves the camera gimbal upward. |
| camera down | `camera_down` | Moves the camera gimbal downward. |
| center camera | `camera_center` | Returns the gimbal to center. |
| toggle lights | `lights_toggle` | Toggles Vehicle lights. |
| toggle LiDAR | `lidar_toggle` | Toggles LiDAR scanning. |

### Autonomous missions

| Command | Intention | Description |
| --- | --- | --- |
| inspect terrain | `terrain_inspection` | Navigates while inspecting terrain and obstacles. |
| locate objects | `locate_objects` | Searches for objects using vision and 3D context. |
| find exit | `find_exit` | Searches for a safe exit route. |
| patrol area | `patrol_area` | Patrols an area using navigation and compass data. |
| scan perimeter | `perimeter_scan` | Travels and observes the perimeter. |
| inspect corridor | `corridor_scan` | Traverses a corridor while maintaining safe clearance. |
| follow target | `follow_target` | Follows a detected ground target. |
| dock | `dock` | Approaches and parks at the base/docking point. |
| hold position | `hold_position` | Stops and maintains local observation. |
| return home | `return_home` | Returns to the origin using navigation. |
| stop mission | `stop_mission` | Cancels the active mission and stops the Vehicle. |

## Drone

### Direct commands

| Command | Intention | Description |
| --- | --- | --- |
| take off | `takeoff` | Starts a controlled takeoff. |
| land | `land` | Starts a controlled landing. |
| climb | `up` | Increases altitude. |
| descend | `down` | Reduces altitude. |
| move forward | `forward` | Moves the Drone forward. |
| reverse | `back` | Moves the Drone backward. |
| move left | `left` | Strafes the Drone to the left. |
| move right | `right` | Strafes the Drone to the right. |
| yaw left | `yaw_left` | Rotates the Drone left in yaw. |
| yaw right | `yaw_right` | Rotates the Drone right in yaw. |
| hover | `hover` | Zeros translation and maintains stabilization. |
| camera left | `camera_left` | Moves the Drone camera gimbal to the left. |
| camera right | `camera_right` | Moves the Drone camera gimbal to the right. |
| camera up | `camera_up` | Moves the Drone camera gimbal upward. |
| camera down | `camera_down` | Moves the Drone camera gimbal downward. |
| center camera | `camera_center` | Returns the Drone gimbal to center. |
| toggle camera stream | `camera_stream_toggle` | Toggles the Drone camera stream. |

### Autonomous missions

| Command | Intention | Description |
| --- | --- | --- |
| aerial terrain inspection | `terrain_inspection` | Inspects terrain in flight using sensors and vision. |
| locate objects | `locate_objects` | Searches for objects from the air. |
| find exit | `find_exit` | Searches for a safe exit/route from the air. |
| aerial scan | `aerial_scan` | Performs an aerial scan pattern. |
| orbit point | `orbit_point` | Orbits a point while maintaining observation. |
| search pattern | `search_pattern` | Executes an autonomous search pattern. |
| hold altitude | `altitude_hold` | Maintains the current altitude with stabilization. |
| return home | `return_home` | Returns to the origin using GPS and compass data. |
| emergency land | `emergency_land` | Performs an immediate safety-priority landing. |
| stop mission | `stop_mission` | Cancels the mission and enters a safe hover state. |
