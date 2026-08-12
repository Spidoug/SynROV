# SynROV AiBot — Software Version 1

AiBot provides the Python intelligence/runtime layer for SynROV. It includes the runtime (`runtime.py`), desktop Command Center (`app.py`), physical-camera support (`camera_devices.py`), robot-specific command/intention catalog (`robot_catalog.py`), perception, safety, orchestration, voice input, and protocol handling.

This Python package is part of SynROV software version 1. Shared application messages use `softwareVersion: 1`, while schema, variable, file, and directory names remain stable.

## Run

```bash
python -m pip install -r requirements.txt
python synrov_aibot.py
```

Headless:

```bash
python -m synrov_aibot --headless --uri ws://127.0.0.1:9000/ --loop
python -m synrov_aibot --headless --uri ws://127.0.0.1:9000/ --loop --camera auto
```

## Communication

AiBot uses the same canonical SynROV envelope as the browser and ROS:

- `9000`: control/state;
- `9001`: Processing 3D perception (`perception.subscription` / `perception.frame`);
- `9002`: Vehicle/Drone camera (`robot_camera.control` / `robot_camera.frame`).

See `../../docs/COMMUNICATION_PROTOCOL.md` for the canonical contract.

## Language packages and voice

AiBot loads external JSON packages from `synrov_aibot/languages/`. English is the fallback package; AiBot does not expose an independent language selector and always follows the package code published by Processing. The selected package updates the Command Center, robot command/intention display text, and the speech-recognition locale without restarting the microphone path.

Static Command Center widgets are authored from canonical English keys and translated through the package table; no Portuguese/English bilingual literal is used as the rendered source. The language-consistency tests also require exact key parity across all eight AiBot packages, preventing a missing translation from silently producing a mixed-language screen.

Audio capture uses `sounddevice` with 16 kHz mono PCM, ambient-noise calibration, and local phrase segmentation. `SpeechRecognition` is used for transcription. See `../../docs/LANGUAGE_PACKS.md` for the package schema.

## Robot-specific intelligence

`robot_catalog.py` does not define one global command set. Manipulator, Vehicle, and Drone have their own direct commands and missions. The same catalog displayed in the **Intentions** tab is used by the command resolver.

The **Operation** tab exposes adaptive AI, music/rhythm, vision/object handling, and webcam controls. Learning protection, shadow validation, arbitration, sensor fusion, safety, orchestration, and performance decisions remain internal to the runtime. Bounded mission-strategy learning and teachable spoken-phrase aliases run independently for Manipulator, Vehicle, and Drone and persist per robot. Long-context episodic memory is also isolated per robot; only music/rhythm actuation remains restricted to Manipulator.

Heading/compass is part of the sensor context for all three robots. Vehicle and Drone use camera pan `-60..+60°` and tilt `-20..+20°`.

Learned runtime data is persisted under `synrov_aibot/runtime_data/` and is not distributed as source code.

## Tests

```bash
python -m compileall synrov_aibot
python -m unittest discover -s tests -v
```


## Bounded mission-strategy learning

AiBot does not train arbitrary motor outputs. `AdaptiveMissionPolicy` selects only pre-approved safe variants for selected search/inspection missions, observes mission outcomes, and stores aggregate rewards in `runtime_data/models/<robot>/<robot>_policy.pkl` plus JSONL experience records in `runtime_data/datasets/<robot>_training.jsonl`. Strategy selection is persistent across launches. Direct commands, emergency actions, pick/place actions, and origin-navigation safety logic are not raw-exploration targets. Processing and firmware safety remain authoritative.
