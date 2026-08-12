# SynROV Project Review

This review keeps the established robot interfaces and current application architecture while making localization package-driven, keeping Processing as the language authority, and preserving the shared Web/Processing camera behavior.

## Main corrections

- All project Markdown documentation is written in English.
- The Word operating guide is synchronized with the English Markdown operating guide.
- Source comments and newly added implementation notes are English; Portuguese text is retained only where it is intentional runtime localization data.
- The standalone HTML console always starts in English instead of inheriting the browser locale.
- Processing loads external JSON language packages from `data/languages`, keeps English as the default, and cycles every discovered package from the language button. The bundled set contains English, Portuguese (Brazil), Spanish, French, German, Simplified Chinese, Japanese, and Arabic.
- AiBot loads its own JSON language packages, follows Processing's package code, and gets the speech-recognition locale from package metadata.
- The Web console explicitly loads all eight bundled language-package scripts so direct `file://` startup still works without an HTTP server; it opens in English and follows Processing after synchronization.
- Language codes are no longer limited to `en`/`pt`; the protocol accepts normalized package identifiers and clients fall back to English when a matching package is absent.
- After a valid Processing state is received, the Web console resolves the `language` package code reported by Processing, preserving Processing as the connected-system language authority and falling back to English when a matching Web package is unavailable.
- Static Vehicle/Drone HTML control labels now start in English before JavaScript/state synchronization.
- Vehicle and Drone share the same camera-mode zoom reference.
- Enabling camera mode for the selected Vehicle or Drone applies a rear chase view from behind the robot rather than a front-facing operator view.
- The previous operator camera rotation and zoom continue to be restored when camera mode is disabled.
- Drone orbit/zoom now uses the Drone itself as the full X/Y/Z scene reference, so climbing/descending moves the ground/world relative to the aircraft instead of allowing the aircraft to leave the frame while the ground remains centered.
- The canonical SynROV protocol remains unchanged: `softwareVersion: 1`, one application envelope, and distinct producer ownership for Web, AiBot, and ROS.
- Dedicated perception (`9001`) and robot-camera (`9002`) sockets continue to use the same application envelope.
- ROS command input remains centralized in `/synrov/control`; parallel `/cmd_vel` and `/joint_targets` command paths are not used.
- Runtime/config paths use the SynROV version-1 layout and `softwareVersion: 1` metadata where applicable.

## Validation performed

- Python bytecode compilation with `compileall`.
- Python protocol, language-package, and 3D reference regression tests (21 tests).
- Browser JavaScript syntax validation with Node.js.
- Static checks for Processing delimiter balance and the requested camera/language invariants.
- Search for remaining Portuguese text in project documentation, excluding intentional bilingual runtime localization dictionaries/strings.
- DOCX render-and-review validation for the updated operating guide.

## Toolchain limitation

If `processing-java` or `arduino-cli` is unavailable in the review environment, the Processing sketch and Arduino firmware cannot be fully compiled here. The normal SynROV development workstation should perform those toolchain builds before flashing or field use.

## Canonical references

- `COMMUNICATION_PROTOCOL.md`
- `LANGUAGE_PACKS.md`
- `ROBOT_CAMERA_STREAM.md`
- `ROS_INTEGRATION.md`
- `SynROV_Operating_Guide.md`
