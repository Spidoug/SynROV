from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from synrov_aibot.protocol import SOFTWARE_VERSION
from synrov_aibot.robot_ai import RobotAIRegistry
from synrov_aibot.robot_catalog import COMMANDS, MISSIONS, resolve_catalog
from synrov_aibot.runtime import SynROVBridge
from synrov_aibot.safety import MANIP_KEYS, MANIPULATOR_HOME_POSE


class Version1ControlTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.registry = RobotAIRegistry(
            model_root=root / "models",
            memory_path=root / "learned_commands.json",
            context_memory_path=root / "long_context.jsonl",
        )

    def tearDown(self) -> None:
        self.tmp.cleanup()

    @staticmethod
    def bridge(robot: str, *, phase: str = "airborne", ready: bool = True, airborne: bool = True) -> SynROVBridge:
        bridge = SynROVBridge()
        bridge._sent_for_test = []

        def capture(payload, immediate=False):
            bridge._sent_for_test.append((payload, bool(immediate)))
            return True

        bridge.send_control = capture
        bridge.state.robot = robot
        bridge.state.sensors = {
            "battery_pct": 100.0,
            "communication_quality_pct": 100.0,
            "alt_cm": 80.0 if airborne else 0.0,
            "heading_deg": 0.0,
        }
        bridge.state.control = {
            "robot": robot,
            "camera": {"pan": 0.0, "tilt": 0.0, "enabled": False},
            "drive": {"throttle": 0.0, "steer": 0.0, "pivot": 0.0, "lights": False, "lidarScan": False},
            "flight": {
                "phase": phase,
                "flightReady": ready,
                "airborne": airborne,
                "throttle": 0.0,
                "yaw": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "strafe": 0.0,
                "forward": 0.0,
            },
            "lights": False,
            "lidarScan": False,
        }
        bridge.state.perception_scene = {"pose": {"x_m": 0.0, "z_m": 0.0, "yaw_deg": 0.0}}
        return bridge

    def test_01_software_protocol_is_version_one(self) -> None:
        self.assertEqual(SOFTWARE_VERSION, 1)

    def test_02_every_catalog_phrase_resolves_to_its_declared_skill(self) -> None:
        for robot in COMMANDS:
            for kind, specs in (("intent", COMMANDS[robot]), ("mission", MISSIONS[robot])):
                for spec in specs:
                    for phrase in spec.aliases:
                        with self.subTest(robot=robot, kind=kind, intent=spec.intent, phrase=phrase):
                            self.assertEqual(resolve_catalog(robot, phrase)[:2], (kind, spec.intent))

    def test_03_return_home_aliases_resolve_for_vehicle_and_drone(self) -> None:
        for robot in ("Vehicle", "Drone"):
            for phrase in ("voltar base", "retornar home", "voltar para base", "retornar base", "ir para base", "return home"):
                with self.subTest(robot=robot, phrase=phrase):
                    self.assertEqual(resolve_catalog(robot, phrase)[:2], ("mission", "return_home"))

    def test_04_every_direct_intent_has_an_executable_implementation(self) -> None:
        for robot, specs in COMMANDS.items():
            ai = self.registry.get(robot)
            for spec in specs:
                bridge = self.bridge(robot)
                with self.subTest(robot=robot, intent=spec.intent):
                    result = ai.execute_intent(spec.intent, bridge)
                    self.assertNotEqual(result.reason, "unknown_intent")
                    self.assertTrue(result.ok, result)

    def test_05_every_declared_mission_starts_and_has_an_executable_tick(self) -> None:
        for robot, specs in MISSIONS.items():
            ai = self.registry.get(robot)
            for spec in specs:
                bridge = self.bridge(robot)
                ai.cancel_mission("cancelled")
                with self.subTest(robot=robot, mission=spec.intent):
                    started = ai.start_mission(spec.intent, bridge, spec.command_pt)
                    self.assertNotEqual(started.reason, "unknown_mission")
                    self.assertTrue(started.ok, started)
                    if spec.intent == "stop_mission":
                        self.assertEqual(started.reason, "stopped")
                        continue
                    ticked = ai.tick(bridge)
                    self.assertNotEqual(ticked.reason, "unknown_mission")
                    self.assertTrue(ticked.ok, ticked)

    def test_06_takeoff_immediately_grants_drone_command_authority(self) -> None:
        bridge = self.bridge("Drone", phase="grounded", ready=False, airborne=False)
        bridge.drone_takeoff()
        flight = bridge.state.control["flight"]
        self.assertEqual(flight["phase"], "taking_off")
        self.assertTrue(flight["flightReady"])
        self.assertTrue(bridge.drone_flight_status()["flight_ready"])

    def test_07_drone_forward_is_not_filtered_while_takeoff_is_in_progress(self) -> None:
        bridge = self.bridge("Drone", phase="taking_off", ready=False, airborne=False)
        result = self.registry.get("Drone").execute_intent("forward", bridge)
        self.assertTrue(result.ok)
        self.assertEqual(result.reason, "flight_sent")
        self.assertGreater(bridge.state.control["flight"]["forward"], 0.0)
        self.assertNotIn("flightStateFiltered", bridge.state.control["flight"])

    def test_08_climb_command_then_forward_works_without_waiting_for_altitude_threshold(self) -> None:
        bridge = self.bridge("Drone", phase="grounded", ready=False, airborne=False)
        ai = self.registry.get("Drone")
        up = ai.execute_intent("up", bridge)
        self.assertTrue(up.ok)
        self.assertEqual(up.reason, "flight_sent")
        self.assertGreater(bridge.state.control["flight"]["throttle"], 0.0)
        payload, _ = bridge._sent_for_test[-1]
        self.assertGreater(payload["control"]["flight"]["throttle"], 0.0)
        self.assertNotIn("phase", payload["control"]["flight"])
        self.assertNotIn("flightReady", payload["control"]["flight"])
        self.assertTrue(bridge.drone_flight_status()["flight_ready"])
        forward = ai.execute_intent("forward", bridge)
        self.assertTrue(forward.ok)
        self.assertEqual(forward.reason, "flight_sent")
        self.assertGreater(bridge.state.control["flight"]["forward"], 0.0)

    def test_09_drone_camera_left_and_right_use_canonical_pan_signs(self) -> None:
        bridge = self.bridge("Drone")
        ai = self.registry.get("Drone")
        left = ai.execute_intent("camera_left", bridge)
        self.assertTrue(left.ok)
        self.assertGreater(bridge.state.control["camera"]["pan"], 0.0)
        bridge.state.control["camera"]["pan"] = 0.0
        bridge.state.control["flight"]["camPan"] = 0.0
        right = ai.execute_intent("camera_right", bridge)
        self.assertTrue(right.ok)
        self.assertLess(bridge.state.control["camera"]["pan"], 0.0)

    def test_10_drone_return_home_turns_toward_processing_origin(self) -> None:
        bridge = self.bridge("Drone")
        bridge.state.perception_scene = {"pose": {"x_m": 1.0, "z_m": 0.0, "yaw_deg": 0.0}}
        ai = self.registry.get("Drone")
        self.assertTrue(ai.start_mission("return_home", bridge, "voltar base").ok)
        ticked = ai.tick(bridge)
        self.assertTrue(ticked.ok)
        self.assertEqual(ticked.metadata.get("phase"), "align_heading")
        self.assertLess(bridge.state.control["flight"]["yaw"], 0.0)
        self.assertEqual(bridge.state.control["flight"]["forward"], 0.0)

    def test_11_vehicle_and_drone_camera_pan_contract_match(self) -> None:
        for robot in ("Vehicle", "Drone"):
            ai = self.registry.get(robot)
            bridge = self.bridge(robot)
            with self.subTest(robot=robot, direction="left"):
                self.assertTrue(ai.execute_intent("camera_left", bridge).ok)
                self.assertGreater(bridge.state.control["camera"]["pan"], 0.0)
            bridge.state.control["camera"]["pan"] = 0.0
            with self.subTest(robot=robot, direction="right"):
                self.assertTrue(ai.execute_intent("camera_right", bridge).ok)
                self.assertLess(bridge.state.control["camera"]["pan"], 0.0)

    def test_12_language_packs_are_version_one_and_define_drone_home_behavior(self) -> None:
        language_dir = Path(__file__).resolve().parents[1] / "synrov_aibot" / "languages"
        current = "Returns to origin using local Processing pose and telemetry heading."
        for path in language_dir.glob("*.json"):
            with self.subTest(language=path.stem):
                data = json.loads(path.read_text(encoding="utf-8"))
                self.assertEqual(data.get("softwareVersion"), 1)
                self.assertIn(current, data.get("strings", {}))

    def test_13_processing_window_configuration_happens_before_show_and_animator(self) -> None:
        processing_main = (
            Path(__file__).resolve().parents[2]
            / "processing"
            / "SynROV"
            / "SynROV.pde"
        )
        source = processing_main.read_text(encoding="utf-8")

        setup_start = source.index("void setup()")
        setup_end = source.index("// Draws a visible diagnostic", setup_start)
        setup_body = source[setup_start:setup_end]
        self.assertNotIn("setResizable", setup_body)
        self.assertNotIn("surface.setSize", setup_body)
        self.assertNotIn("surface.setLocation", setup_body)

        hook_start = source.index("protected PSurface initSurface()")
        hook_end = source.index("// Blocks application shutdown", hook_start)
        hook_body = source[hook_start:hook_end]
        self.assertLess(
            hook_body.index("super.initSurface();"),
            hook_body.index("configureCreatedMainSurfaceForResize(createdSurface);"),
        )
        self.assertNotIn("startSurface()", source[hook_start:hook_end])

        resize_helper_start = source.index("void configureCreatedMainSurfaceForResize")
        resize_helper_end = source.index("// initSurface() is the last lifecycle point", resize_helper_start)
        resize_helper = source[resize_helper_start:resize_helper_end]
        self.assertIn("GLWindow", resize_helper)
        self.assertIn("setResizable(true)", resize_helper)
        self.assertNotIn("setSize", resize_helper)
        self.assertNotIn("setLocation", resize_helper)
        self.assertNotIn('getMethod("setResizable"', source)
        self.assertNotIn("configureMainWindowBeforeRendering", source)
        self.assertNotIn("protected void startSurface()", source)


    def test_14_processing_reuses_single_safety_and_session_reset_blocks(self) -> None:
        processing_dir = Path(__file__).resolve().parents[2] / "processing" / "SynROV"
        main_source = (processing_dir / "SynROV.pde").read_text(encoding="utf-8")
        runtime_source = (processing_dir / "Logic_RuntimeBridge.pde").read_text(encoding="utf-8")

        self.assertEqual(main_source.count("boolean enforceVehicleEmergencyStopForLocalInput()"), 1)
        self.assertEqual(main_source.count("enforceVehicleEmergencyStopForLocalInput()"), 4)
        self.assertEqual(runtime_source.count("void resetHardwareTelemetrySessionState()"), 1)
        self.assertEqual(runtime_source.count("resetHardwareTelemetrySessionState();"), 3)

    def test_15_processing_startup_and_draw_share_visible_runtime_fault_guard(self) -> None:
        processing_main = (
            Path(__file__).resolve().parents[2]
            / "processing"
            / "SynROV"
            / "SynROV.pde"
        )
        source = processing_main.read_text(encoding="utf-8")
        self.assertEqual(source.count("RuntimeException runtimeFault = null;"), 1)
        self.assertEqual(source.count("void latchRuntimeFault(String phase, RuntimeException ex)"), 1)
        self.assertEqual(source.count("void drawRuntimeFaultScreen(RuntimeException ex)"), 1)

        setup_start = source.index("void setup()")
        setup_end = source.index("// Draws a visible diagnostic", setup_start)
        setup_body = source[setup_start:setup_end]
        self.assertIn("initializeSynRovRuntime();", setup_body)
        self.assertIn('latchRuntimeFault("Startup", ex);', setup_body)

        draw_start = source.index("void draw()")
        draw_end = source.index("// Utility: steps system.", draw_start)
        draw_body = source[draw_start:draw_end]
        self.assertIn("runRuntimeFrame();", draw_body)
        self.assertIn('latchRuntimeFault("Render", ex);', draw_body)
        self.assertIn("drawRuntimeFaultScreen(ex);", draw_body)

    def test_16_processing_fault_screen_uses_valid_p3d_camera_transform(self) -> None:
        processing_main = (
            Path(__file__).resolve().parents[2]
            / "processing"
            / "SynROV"
            / "SynROV.pde"
        )
        source = processing_main.read_text(encoding="utf-8")
        start = source.index("void drawRuntimeFaultScreen(RuntimeException ex)")
        end = source.index("// Top-level frame guard", start)
        body = source[start:end]
        self.assertIn("camera();", body)
        self.assertNotIn("resetMatrix();", body)
        self.assertIn("background(24);", body)

    def test_17_processing_startup_window_stays_inside_usable_desktop(self) -> None:
        processing_main = (
            Path(__file__).resolve().parents[2]
            / "processing"
            / "SynROV"
            / "SynROV.pde"
        )
        source = processing_main.read_text(encoding="utf-8")

        settings_start = source.index("void settings()")
        settings_end = source.index("// Runs the complete V1 application initialization", settings_start)
        settings_body = source[settings_start:settings_end]
        self.assertIn("displayWidth - 48", settings_body)
        self.assertIn("displayHeight - 96", settings_body)
        self.assertIn("size(startupWidth, startupHeight, P3D);", settings_body)
        self.assertNotIn("surface.setSize", settings_body)

        show_start = source.index("protected void showSurface()")
        show_end = source.index("// Blocks application shutdown", show_start)
        show_body = source[show_start:show_end]
        self.assertLess(
            show_body.index("keepCreatedMainWindowInsideWorkingArea(surface);"),
            show_body.index("super.showSurface();"),
        )

        clamp_start = source.index("void keepCreatedMainWindowInsideWorkingArea")
        clamp_end = source.index("// runSketch() has already called placeWindow()", clamp_start)
        clamp_body = source[clamp_start:clamp_end]
        self.assertIn("InsetsImmutable", clamp_body)
        self.assertIn("getScreenInsets", source)
        self.assertIn("setTopLevelPosition", clamp_body)
        self.assertNotIn("setSize", clamp_body)
        self.assertNotIn("surface.setSize", clamp_body)

    def test_18_aibot_drone_yaw_visual_reference_maps_before_protocol(self) -> None:
        bridge = self.bridge("Drone")
        ai = self.registry.get("Drone")
        left = ai.execute_intent("yaw_left", bridge)
        self.assertTrue(left.ok)
        self.assertGreater(bridge.state.control["flight"]["yaw"], 0.0)
        right = ai.execute_intent("yaw_right", bridge)
        self.assertTrue(right.ok)
        self.assertLess(bridge.state.control["flight"]["yaw"], 0.0)

    def test_19_web_drone_descend_is_not_suppressed_by_browser_airborne_state(self) -> None:
        web_path = Path(__file__).resolve().parents[2] / "web" / "SynROV.html"
        source = web_path.read_text(encoding="utf-8")
        self.assertIn("throttle: state.drone.throttle / 100", source)
        self.assertIn("state.drone.throttle = heldAxis('drone-up', 'drone-down'", source)
        self.assertNotIn("(!airborne && state.drone.throttle < 0) ? 0", source)
        self.assertNotIn("!state.drone.airborne && requestedThrottle < 0", source)
        self.assertNotIn("def.key === 'throttle' && !state.drone.airborne", source)

    def test_20_web_camera_toggle_is_canonical_for_vehicle_and_drone(self) -> None:
        web_path = Path(__file__).resolve().parents[2] / "web" / "SynROV.html"
        source = web_path.read_text(encoding="utf-8")
        self.assertIn('data-control-action="vehicle-camera-toggle"', source)
        self.assertIn('data-control-action="drone-camera-toggle"', source)
        self.assertEqual(source.count("function setRobotCameraEnabled(group, enabled)"), 1)
        self.assertIn("safeSend({ camera: { robot: isVehicle ? MODE_VEHICLE : MODE_DRONE, enabled: target.camera } })", source)
        self.assertIn("if (action === 'vehicle-camera-toggle') return toggleRobotCameraEnabled('vehicle');", source)
        self.assertIn("if (action === 'drone-camera-toggle') return toggleRobotCameraEnabled('drone');", source)
        self.assertNotIn("cameraStreaming: !!state.drone.camera", source)
        self.assertNotIn("camera-stream", source)

    def test_21_processing_uses_one_camera_state_and_shared_joystick_tilt_reference(self) -> None:
        processing_dir = Path(__file__).resolve().parents[2] / "processing" / "SynROV"
        main_source = (processing_dir / "SynROV.pde").read_text(encoding="utf-8")
        camera_source = (processing_dir / "RobotCameraStream.pde").read_text(encoding="utf-8")
        drone_source = (processing_dir / "Drone3D.pde").read_text(encoding="utf-8")

        self.assertEqual(main_source.count("void applyRemoteRobotCameraCommand(int robotMode, JSONObject payload)"), 1)
        self.assertIn('setRobotCameraViewEnabled(ROBOT_MODE_VEHICLE, payload.getBoolean("enabled"))', main_source)
        self.assertIn('setRobotCameraViewEnabled(ROBOT_MODE_DRONE, payload.getBoolean("enabled"))', main_source)
        self.assertIn('camera.setBoolean("enabled", isRobotCameraViewEnabled(ROBOT_MODE_VEHICLE));', main_source)
        self.assertIn('camera.setBoolean("enabled", isRobotCameraViewEnabled(ROBOT_MODE_DRONE));', main_source)
        self.assertNotIn('getJsonBoolean(payload, "cameraStreaming"', main_source)
        self.assertNotIn("setCommandContext(CONTROL_SOURCE_LOCAL);", camera_source[camera_source.index("void setRobotCameraViewEnabled"):camera_source.index("void toggleRobotCameraView")])
        self.assertIn("fill(isRobotCameraViewEnabled(ROBOT_MODE_DRONE)", drone_source)

        vehicle_default = main_source[main_source.index("JoystickProfile createDefaultVehicleJoystickProfile()"):main_source.index("// Creates default drone joystick profile.")]
        drone_default = main_source[main_source.index("JoystickProfile createDefaultDroneJoystickProfile()"):main_source.index("// Ensures joystick profile arrays", main_source.index("JoystickProfile createDefaultDroneJoystickProfile()"))]
        self.assertIn('final boolean DEFAULT_CAMERA_TILT_AXIS_INVERTED = false;', main_source)
        self.assertIn('profile.setInvert("camTilt", DEFAULT_CAMERA_TILT_AXIS_INVERTED);', vehicle_default)
        self.assertIn('profile.setInvert("camTilt", DEFAULT_CAMERA_TILT_AXIS_INVERTED);', drone_default)

        vehicle_diag = (processing_dir / "Vehicle3D.pde").read_text(encoding="utf-8")
        drone_diag = (processing_dir / "DiagnosticsPanel_Base.pde").read_text(encoding="utf-8")
        self.assertIn("void toggleVehicleCamera()", camera_source)
        self.assertIn("toggleRobotCameraView(ROBOT_MODE_VEHICLE);", camera_source)
        self.assertIn("void toggleSelectedDroneCameraView()", camera_source)
        self.assertIn("toggleRobotCameraView(ROBOT_MODE_DRONE);", camera_source)
        self.assertIn("toggleVehicleCamera();", vehicle_diag)
        self.assertIn("toggleSelectedDroneCameraView();", drone_diag)

    def test_22_aibot_window_is_explicitly_resizable_and_visible(self) -> None:
        app_path = Path(__file__).resolve().parents[1] / "synrov_aibot" / "app.py"
        source = app_path.read_text(encoding="utf-8")
        self.assertEqual(source.count("def _configure_main_window(self)"), 1)
        self.assertIn("self.root.resizable(True, True)", source)
        self.assertIn("self.root.minsize(900, 600)", source)
        sash_start = source.index("def _set_initial_sash")
        sash_end = source.index("# ----------------------------------------------------------- connections", sash_start)
        self.assertNotIn("except Exception", source[sash_start:sash_end])

    def test_23_aibot_camera_enable_uses_one_protocol_command_for_both_robots(self) -> None:
        for robot in ("Vehicle", "Drone"):
            bridge = self.bridge(robot)
            bridge.set_camera(robot, enabled=True)
            with self.subTest(robot=robot):
                payload, immediate = bridge._sent_for_test[-1]
                self.assertTrue(immediate)
                self.assertEqual(payload, {"camera": {"robot": robot, "enabled": True}})
                self.assertTrue(bridge.state.control["camera"]["enabled"])
                



    def test_24_protocol_signs_are_not_producer_specific(self) -> None:
        bridge = self.bridge("Drone")
        bridge.send_drone(0.0, 0.37, 0.0, 0.0, 0.0, 0.0)
        payload, _ = bridge._sent_for_test[-1]
        self.assertEqual(payload["control"]["flight"]["yaw"], 0.37)
        self.assertNotIn("phase", payload["control"]["flight"])
        self.assertNotIn("flightReady", payload["control"]["flight"])

        bridge = self.bridge("Vehicle")
        bridge.send_vehicle(0.0, 0.0, pivot=0.37)
        payload, _ = bridge._sent_for_test[-1]
        self.assertEqual(payload["control"]["drive"]["pivot"], 0.37)

        web_path = Path(__file__).resolve().parents[2] / "web" / "SynROV.html"
        web = web_path.read_text(encoding="utf-8")
        self.assertIn("heldAxis('vehicle-pivot-positive', 'vehicle-pivot-negative'", web)
        self.assertIn("heldAxis('drone-yaw-positive', 'drone-yaw-negative'", web)
        self.assertIn("data-hold-command=\"vehicle-pivot-positive\" title=\"Visual yaw left\"", web)
        self.assertIn("data-hold-command=\"drone-yaw-positive\" title=\"Visual yaw left\"", web)
        self.assertIn("if (key === 'left') return setAxisValue('drone', DRONE_DEFS[1], state.drone.yaw + 10, true);", web)
        self.assertIn("if (key === 'right') return setAxisValue('drone', DRONE_DEFS[1], state.drone.yaw - 10, true);", web)
        self.assertIn("if (normalized === 'q') return 'vehicle-pivot-negative';", web)
        self.assertIn("if (normalized === 'e') return 'vehicle-pivot-positive';", web)
        self.assertIn("if (normalized === 'q') return 'drone-yaw-positive';", web)
        self.assertIn("if (normalized === 'e') return 'drone-yaw-negative';", web)

    def test_25_camera_protocol_uses_enabled_without_control_aliases(self) -> None:
        root = Path(__file__).resolve().parents[3]
        web = (root / "software" / "web" / "SynROV.html").read_text(encoding="utf-8")
        runtime = (root / "software" / "python" / "synrov_aibot" / "runtime.py").read_text(encoding="utf-8")
        processing = (root / "software" / "processing" / "SynROV" / "SynROV.pde").read_text(encoding="utf-8")
        self.assertIn("camera: { robot: isVehicle ? MODE_VEHICLE : MODE_DRONE, enabled: target.camera }", web)
        self.assertNotIn("cameraStreaming: !!state.drone.camera", web)
        self.assertIn('camera_payload["enabled"] = bool(enabled)', runtime)
        self.assertNotIn('camera_payload["stream"]', runtime)
        self.assertIn('payload.hasKey("enabled")', processing)
        self.assertNotIn('payload.hasKey("stream")', processing)

    def test_26_camera_session_state_is_not_persisted_and_both_models_use_yellow_indicator(self) -> None:
        processing_dir = Path(__file__).resolve().parents[2] / "processing" / "SynROV"
        main_source = (processing_dir / "SynROV.pde").read_text(encoding="utf-8")
        camera_source = (processing_dir / "RobotCameraStream.pde").read_text(encoding="utf-8")
        vehicle_source = (processing_dir / "Vehicle3D.pde").read_text(encoding="utf-8")
        drone_source = (processing_dir / "Drone3D.pde").read_text(encoding="utf-8")

        self.assertNotIn('systemCfg.setBoolean("droneCameraStreamingEnabled"', main_source)
        self.assertNotIn('getJsonBoolean(systemCfg, "droneCameraStreamingEnabled"', main_source)
        setter = camera_source[camera_source.index("void setRobotCameraViewEnabled"):camera_source.index("void toggleRobotCameraView")]
        self.assertNotIn("saveSoftwareConfigNow", setter)
        self.assertIn("fill(isRobotCameraViewEnabled(ROBOT_MODE_VEHICLE) ? color(255, 220, 0)", vehicle_source)
        self.assertIn("fill(isRobotCameraViewEnabled(ROBOT_MODE_DRONE) ? color(255, 220, 0)", drone_source)

    def test_27_aibot_camera_toggle_parity_vehicle_and_drone(self) -> None:
        for robot in ("Vehicle", "Drone"):
            bridge = self.bridge(robot)
            ai = self.registry.get(robot)
            result = ai.execute_intent("camera_toggle", bridge)
            with self.subTest(robot=robot):
                self.assertTrue(result.ok)
                self.assertTrue(bridge.state.control["camera"]["enabled"])
                payload, immediate = bridge._sent_for_test[-1]
                self.assertTrue(immediate)
                self.assertEqual(payload, {"camera": {"robot": robot, "enabled": True}})

    def test_28_version_one_docs_are_clean(self) -> None:
        root = Path(__file__).resolve().parents[3]
        self.assertFalse((root / "docs" / "VALIDATION_REPORT.md").exists())
        forbidden = (
            "leg" + "acy", "leg" + "ado", "revi" + "sion", "revi" + "são",
            "revi" + "sao", "re" + "view", "obso" + "lete", "obso" + "leto",
            "depre" + "cated", "previous " + "version", "versão " + "anterior",
        )
        text_paths = [root / "README.md"]
        text_paths.extend(sorted((root / "docs").glob("*.md")))
        for path in text_paths:
            content = path.read_text(encoding="utf-8").lower()
            for term in forbidden:
                with self.subTest(path=path.name, term=term):
                    self.assertNotIn(term, content)

        app_source = (root / "software" / "python" / "synrov_aibot" / "app.py").read_text(encoding="utf-8")
        styles = app_source[app_source.index("def _build_styles"):app_source.index("def _card")]
        self.assertNotIn("except Exception", styles)


    def test_29_manipulator_home_is_one_canonical_pose_end_to_end(self) -> None:
        bridge = self.bridge("Manipulator")
        ai = self.registry.get("Manipulator")
        started = ai.start_mission("scan_workspace", bridge, "scan workspace")
        self.assertTrue(started.ok)
        self.assertIsNotNone(ai.active_mission)

        # AiBot must send HOME as an asynchronous intent, not forge measured
        # servo feedback before Processing/hardware actually reaches the pose.
        bridge.state.servos = {index: 10.0 + index for index in range(len(MANIP_KEYS))}
        measured_before = dict(bridge.state.servos)
        result = ai.execute_intent("home", bridge)
        self.assertTrue(result.ok)
        self.assertIsNone(ai.active_mission)
        payload, immediate = bridge._sent_for_test[-1]
        self.assertTrue(immediate)
        self.assertEqual(payload, {"action": "home", "robot": "Manipulator"})
        self.assertEqual(bridge.state.servos, measured_before)

        root = Path(__file__).resolve().parents[3]
        processing = (root / "software" / "processing" / "SynROV" / "SynROV.pde").read_text(encoding="utf-8")
        runtime = (root / "software" / "processing" / "SynROV" / "Logic_RuntimeBridge.pde").read_text(encoding="utf-8")
        manip3d = (root / "software" / "processing" / "SynROV" / "Manipulator3D.pde").read_text(encoding="utf-8")
        self.assertIn("final int[] MANIPULATOR_HOME_POSE = {180, 150, 70, 90, 95, 130, 0};", processing)
        self.assertNotIn("STARTUP_POSE", processing)
        self.assertNotIn("homingPositions", processing + runtime + manip3d)
        self.assertNotIn('"homingPosition"', processing + runtime + manip3d)
        self.assertIn("void beginManipulatorHome(int source, boolean synchronizeFromHardware)", runtime)
        self.assertIn("cancelManipulatorTargetAutomation();", runtime)
        self.assertIn("syncManipulatorCommandPoseFromHardware();", runtime)
        self.assertIn("beginManipulatorHome(CONTROL_SOURCE_LOCAL, false);", runtime)
        dance = (root / "software" / "python" / "synrov_aibot" / "dance.py").read_text(encoding="utf-8")
        robot_ai = (root / "software" / "python" / "synrov_aibot" / "robot_ai.py").read_text(encoding="utf-8")
        self.assertIn("pose = dict(MANIPULATOR_HOME_POSE)", dance)
        self.assertNotIn("HOME_POSE:", dance)
        self.assertIn("defaults = dict(MANIPULATOR_HOME_POSE)", robot_ai)
        self.assertNotIn('"upper": 45.0', robot_ai)
        home_handler = processing[
            processing.index("void handleRemoteManipulatorHomeCommand()"):
            processing.index("// Applies vehicle runtime mirror.")
        ]
        self.assertIn("beginManipulatorHome(homeSource, true);", home_handler)

    def test_30_web_held_drone_descend_reaches_the_canonical_vertical_gate(self) -> None:
        root = Path(__file__).resolve().parents[2]
        web = (root / "web" / "SynROV.html").read_text(encoding="utf-8")
        ready_commands = web[web.index("const DRONE_FLIGHT_READY_COMMANDS"):web.index("function droneFlightControlReady")]
        # Vertical descent is never filtered by stale browser flightReady state.
        self.assertNotIn("'drone-down'", ready_commands)
        motion = web[web.index("function applyMomentaryMotion(sendNow = true)") : web.index("function beginHoldCommand", web.index("function applyMomentaryMotion(sendNow = true)"))]
        self.assertIn("state.drone.throttle = heldAxis('drone-up', 'drone-down'", motion)
        sender = web[web.index("function flushControlSend()") : web.index("function scheduleSend", web.index("function flushControlSend()"))]
        self.assertIn("throttle: state.drone.throttle / 100", sender)

        drone = (root / "processing" / "SynROV" / "Drone3D.pde").read_text(encoding="utf-8")
        vertical = drone[drone.index("void setDroneFlightNormalized") : drone.index("void cancelDroneAutomaticVerticalMotion")]
        self.assertIn("droneAutoTakeoffActive && boundedThrottle < -0.02f", vertical)
        self.assertIn("droneAutoTakeoffActive = false;", vertical)
        self.assertIn("boundedThrottle < 0.0f && !droneCanDescendNow()", vertical)
        self.assertNotIn("!droneIsAirborne() && boundedThrottle < 0.0f", vertical)

    def test_31_rosbridge_topic_operations_use_explicit_v21_qos_and_ids(self) -> None:
        ros = (Path(__file__).resolve().parents[2] / "processing" / "SynROV" / "RosIntegration.pde").read_text(encoding="utf-8")
        self.assertNotIn("rosEndpointReachable", ros)
        self.assertIn('qos.setString("history", "keep_last")', ros)
        self.assertIn('qos.setString("reliability", "reliable")', ros)
        self.assertIn('qos.setString("durability", "volatile")', ros)
        self.assertIn('packet.setString("id", rosOperationId("pub", leaf))', ros)
        self.assertIn('packet.setString("id", rosOperationId("sub", leaf))', ros)
        self.assertIn('rosAdvertise("state", "std_msgs/msg/String", 10);', ros)
        self.assertIn('rosAdvertise("joint_states", "sensor_msgs/msg/JointState", 10);', ros)
        self.assertIn('rosSubscribe("control", "std_msgs/msg/String", 10);', ros)
        self.assertNotIn('"latch"', ros)
        self.assertNotIn('"queue_size"', ros)

    def test_32_ros_joint_state_positions_are_angular_si_values(self) -> None:
        ros = (Path(__file__).resolve().parents[2] / "processing" / "SynROV" / "RosIntegration.pde").read_text(encoding="utf-8")
        self.assertIn('"gripper_finger"', ros)
        self.assertIn("value = radians(getGripperFingerModelZDeg());", ros)
        self.assertIn("value = radians(armUiServoToMemberDeg", ros)
        self.assertNotIn("memberDeg / 100.0f", ros)
        self.assertIn('header.setJSONObject("stamp", rosTime());', ros)
        self.assertIn('msg.setJSONArray("velocity", new JSONArray());', ros)
        self.assertIn('msg.setJSONArray("effort", new JSONArray());', ros)

    def test_33_action_dispatch_uses_safe_json_access_without_exception_flow(self) -> None:
        processing = (Path(__file__).resolve().parents[2] / "processing" / "SynROV" / "SynROV.pde").read_text(encoding="utf-8")
        body = processing[processing.index("void handleActionCommand(JSONObject jsonCommand)") : processing.index("// Handles a remote request to start the hardware connection.")]
        self.assertIn('getJsonString(jsonCommand, "action", "")', body)
        self.assertIn('getJsonInt(jsonCommand, "action", 0)', body)
        self.assertNotIn("try {", body)
        self.assertNotIn("catch (", body)

    def test_34_ros_commands_reuse_the_synrov_dispatcher(self) -> None:
        ros = (Path(__file__).resolve().parents[2] / "processing" / "SynROV" / "RosIntegration.pde").read_text(encoding="utf-8")
        handler = ros[ros.index("void handleRosControlMessage") : ros.index("void processRosBridgePacket")]
        self.assertIn("isSynRovMessage(message)", handler)
        self.assertIn("SYNROV_SOURCE_ROS", handler)
        self.assertIn("isSynRovExternalCommandType", handler)
        self.assertIn("dispatchSynRovCommand(message);", handler)
        self.assertNotIn("handleDroneFlightCommand", handler)
        self.assertNotIn("handleVehicleDriveCommand", handler)
        self.assertNotIn("handleUnifiedManipulatorControlCommand", handler)

    def test_35_manipulator_home_has_no_second_python_pose_definition(self) -> None:
        root = Path(__file__).resolve().parents[2] / "python" / "synrov_aibot"
        source = "\n".join(path.read_text(encoding="utf-8") for path in sorted(root.glob("*.py")))
        self.assertEqual(source.count("MANIPULATOR_HOME_POSE: Dict[str, float] ="), 1)
        self.assertNotIn("DANCE_HOME_POSE", source)
        self.assertNotRegex(source, r"(?m)^HOME_POSE\s*:\s*Dict\[str, float\]\s*=")


    def test_36_manipulator_home_pose_matches_processing_aibot_and_firmware(self) -> None:
        root = Path(__file__).resolve().parents[3]
        canonical = [int(MANIPULATOR_HOME_POSE[key]) for key in MANIP_KEYS]
        self.assertEqual(canonical, [180, 150, 70, 90, 95, 130, 0])

        processing = (root / "software" / "processing" / "SynROV" / "SynROV.pde").read_text(encoding="utf-8")
        firmware = (root / "firmware" / "SynROV_Firmware" / "FirmwareDeclarations.h").read_text(encoding="utf-8")
        firmware_tree = "\n".join(
            path.read_text(encoding="utf-8")
            for path in sorted((root / "firmware" / "SynROV_Firmware").glob("*"))
            if path.is_file()
        )

        pose_csv = ", ".join(str(value) for value in canonical)
        self.assertIn(f"MANIPULATOR_HOME_POSE = {{{pose_csv}}};", processing)
        self.assertIn(f"kManipulatorHomePose[MANIP_MEMBER_COUNT] PROGMEM = {{{pose_csv}}};", firmware)
        self.assertNotIn("manipHomePose", firmware_tree)
        self.assertNotIn("fillDefaultManipulatorHomePose", firmware_tree)
        self.assertNotIn("sanitizeManipulatorHomePose", firmware_tree)
        self.assertIn("getManipulatorHomeMemberDeg(member)", firmware_tree)



if __name__ == "__main__":
    unittest.main()
