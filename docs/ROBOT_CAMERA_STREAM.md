# Vehicle/Drone Camera Stream

Vehicle and Drone share the same logical camera contract. The stream uses dedicated WebSocket port `9002` to isolate JPEG traffic from the control channel while still using the canonical SynROV application envelope.

## Stream control

Processing → AiBot:

```json
{
  "protocol": "synrov",
  "softwareVersion": 1,
  "messageType": "robot_camera.control",
  "source": "processing",
  "timestampMs": 1786392000000,
  "seq": 10,
  "payload": {
    "robot": "Vehicle",
    "enabled": true
  }
}
```

## Frame

AiBot → Processing:

```json
{
  "protocol": "synrov",
  "softwareVersion": 1,
  "messageType": "robot_camera.frame",
  "source": "aibot",
  "timestampMs": 1786392000020,
  "seq": 33,
  "payload": {
    "robot": "vehicle",
    "encoding": "jpeg_base64",
    "width": 640,
    "height": 360,
    "cameraSource": "physical_webcam",
    "fallback": false,
    "image": "..."
  }
}
```

Processing accepts only the current envelope and keeps the newest pending frame for each robot. `cameraSource` describes the image origin without conflicting with the envelope-level `source` field.

## Physical source and fallback

`synrov_aibot/camera_devices.py` provides the local webcam implementation. The AiBot selector offers Auto plus validated camera devices. When a physical webcam is unavailable, the bridge can keep the channel active using the current Processing image/fallback path.

## Gimbal and projection

Vehicle and Drone use the same mechanical safety envelope:

- pan: `-60° .. +60°`;
- tilt: `-20° .. +20°`;
- physical pan-servo range: `30° .. 150°`;
- physical tilt-servo range: `70° .. 110°`.

The 3D image monitor is projected from the actual robot-camera origin. Robot attitude and gimbal pan/tilt define the optical ray, and the image plane is placed along that ray using the configured projection distance. Vehicle and Drone use the same tilt sign everywhere: positive tilt points the camera up (`F`), negative tilt points it down (`R`).

## Processing camera mode

Enabling camera mode for the currently selected Vehicle or Drone also changes the Processing operator view:

- the view becomes a **rear chase view**, positioned conceptually behind the selected robot and looking toward its forward direction;
- Vehicle and Drone use the same third-person camera zoom reference (`1.15` before normal scene constraints);
- only the selected robot's camera mode drives the chase-view preset;
- the operator's previous camera rotation and zoom are saved before camera mode is applied and restored when camera mode is disabled.

The camera-mode viewpoint is separate from the physical image stream. Port `9002` carries image data; the Processing chase view controls how the 3D scene is observed.

## Transport separation

Port `9002` is a bandwidth/transport decision. SynROV does not define a parallel camera application protocol with a separate schema.
## Drone-centered scene reference

Drone altitude is rendered in a robot-referenced frame. Before zoom/orbit transforms are applied, Processing translates the 3D scene by the inverse of the Drone X/Y/Z scene position. The Drone therefore remains at the local view origin while the ground, operating grid, imported world, and other world-space references move relative to it during climb/descent. This reference is active for normal Drone orbit/zoom and for rear-chase camera mode, so camera mode does not apply a second Drone translation.

