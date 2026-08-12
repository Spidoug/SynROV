# SynROV validation report — 2026-08-11

This revision unifies keyboard, Web, joystick, Leap Motion and AiBot control around the Processing control-direction contract and adds explicit Drone flight-state prerequisites.

## Automated checks completed

- Python package syntax parsed successfully with the Python AST parser.
- `software/python/tests/`: 12/12 unit tests passed (9 control/AI tests + 3 language-consistency tests).
- Mission smoke validation exercised all declared Manipulator, Vehicle and Drone missions without runtime exceptions using simulated bridge/state data.
- Web console inline JavaScript passed `node --check`.
- All Web language JavaScript files passed `node --check`; all eight Web packages now have exact key parity.
- All AiBot language JSON files parsed successfully; all eight AiBot packages have exact key parity.
- All eight Processing language JSON packages have exact key parity.
- Supplied Manipulator/Vehicle/Drone Arduino Mega 2560 pin-map diagrams were cross-checked against the firmware declaration defaults for the principal pins and shared buses.
- Static/dynamic localization audit removed the remaining AiBot mixed-language title, Web runtime status literals, and Processing diagnostic/status literals that bypassed translation.
- All 20 Processing `.pde` files passed structural delimiter/comment/string-balance validation.
- The Operating Guide DOCX was rebuilt with the supplied project screenshots/pin maps, rendered after editing, and all 11 rendered pages were visually inspected.

## Safety/control behavior validated

- Drone horizontal/yaw motion is filtered while physically on the ground.
- Drone direct aerial commands and aerial missions initiate takeoff first and wait for confirmed flight readiness before planar movement.
- Takeoff and landing vertical control cannot be cancelled by neutral Web/joystick/Leap updates while the automatic transition owns the vertical axis.
- Vehicle and Drone return-home missions navigate toward the runtime origin instead of using a timed reverse fallback.
- Teachable aliases and long-context memory are isolated per robot and persist independently for Manipulator, Vehicle and Drone.
- Explicit multi-step natural-language plans resolve only to audited robot skills and preserve safety preconditions between steps.

## Environment limitation

The Processing CLI (`processing-java`) is not installed in the validation environment, so a full Processing compile/runtime launch could not be performed here. The Processing source received structural/static validation, but final hardware validation should still be performed with the actual Processing installation, joystick, Leap Motion device and robot hardware before unrestricted motion.
