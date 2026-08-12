"""SynROV AiBot Version 1 desktop Command Center.

This is the canonical Tkinter frontend for the current SynROV runtime. It uses
only the active runtime, robot catalog, camera contract, camera discovery and
voice-input modules, with one current frontend.
"""
from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, simpledialog, ttk
from typing import Any, Dict, List, Optional

try:
    from PIL import Image, ImageTk
except Exception:  # Pillow is optional until an image is displayed.
    Image = None
    ImageTk = None

from .camera_contract import (
    CAMERA_PAN_MAX_DEG,
    CAMERA_PAN_MIN_DEG,
    CAMERA_TILT_MAX_DEG,
    CAMERA_TILT_MIN_DEG,
    default_camera_pose,
)
from .camera_devices import CameraDevice, WebcamSource
from .robot_catalog import all_specs_for_robot
from .robot_types import ROBOTS, canonical_robot
from .runtime import DEFAULT_URI, SynROVBridge, SynROVRuntime
from .safety import MANIP_KEYS, MANIP_POSE_LIMITS
from .voice_input import VoiceInput
from .ui_i18n import format_text as ui_format, language_id, language_name, speech_locale, text as ui_text, tr_text

APP_TITLE = "SynROV Command Center - AiBot V1 - by Douglas Santana // @spidoug"
APP_VERSION = "1"
BG = "#e9ecef"
CARD = "#ffffff"
TEXT = "#25313d"
MUTED = "#607080"
ACCENT = "#0f766e"
ACCENT_SOFT = "#ccfbf1"
OK = "#198754"
WARN = "#fd7e14"
BAD = "#dc3545"
BLUE = "#0d6efd"


class ScrollablePane(ttk.Frame):
    """Simple vertical scrolling host used by the Command Center layout."""

    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent)
        self.canvas = tk.Canvas(self, bg=BG, highlightthickness=0, borderwidth=0)
        self.scroll = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.body = ttk.Frame(self.canvas, style="App.TFrame", padding=6)
        self.window_id = self.canvas.create_window((0, 0), window=self.body, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scroll.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scroll.pack(side="right", fill="y")
        self.body.bind("<Configure>", self._sync_region)
        self.canvas.bind("<Configure>", self._sync_width)

    def _sync_region(self, _event: Any = None) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _sync_width(self, event: Any) -> None:
        self.canvas.itemconfigure(self.window_id, width=max(1, int(event.width)))


class VisualStatusTile(tk.Frame):
    """Compact status tile used by the Command Center."""

    def __init__(self, parent: tk.Misc, title: str) -> None:
        super().__init__(parent, bg="#f7f8fa", highlightthickness=1, highlightbackground="#d9dee3")
        self.title = tk.Label(self, text=title, bg="#f7f8fa", fg=MUTED, font=("Segoe UI", 8), anchor="w")
        self.title.pack(fill="x", padx=8, pady=(6, 0))
        self.value = tk.Label(self, text="—", bg="#f7f8fa", fg=TEXT, font=("Segoe UI", 9, "bold"), anchor="w")
        self.value.pack(fill="x", padx=8, pady=(0, 6))

    def set(self, text: str, level: str = "neutral") -> None:
        colors = {"ok": OK, "warn": WARN, "bad": BAD, "info": BLUE, "neutral": TEXT}
        self.value.configure(text=str(text), fg=colors.get(level, TEXT))


class SynROVApp:
    """Classic-looking Tkinter frontend connected only to the canonical runtime."""

    POLL_MS = 120
    PREVIEW_SIZE = (360, 204)
    WEBCAM_SIZE = (360, 118)

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1320x820")
        self.root.minsize(1040, 690)
        self.root.configure(bg=BG)

        self.bridge = SynROVBridge()
        self.runtime = SynROVRuntime(self.bridge)
        self._closing = False
        self._initial_sash_centered = False
        self._camera_scan_running = False
        self._camera_scan_result: "queue.Queue[List[CameraDevice]]" = queue.Queue(maxsize=1)
        self._camera_label_to_index: Dict[str, int] = {"Auto": WebcamSource.AUTO_INDEX}
        self._preview_photo: Any = None
        self._webcam_photo: Any = None
        self._last_rate_ts = time.monotonic()
        self._last_telem_counter = 0
        self._last_frame_counter = 0
        self._telem_rate = 0.0
        self._frame_rate = 0.0
        self._last_learning_log_ts = 0.0
        self._last_learning_signature = ""
        self._ui_language = "en"
        self._voice_status_code = "stopped"
        self._activity_pair = ("Aguardando conexão com o Processing.", "Waiting for Processing connection.")

        self.uri_var = tk.StringVar(value=DEFAULT_URI)
        self.robot_var = tk.StringVar(value="Manipulator")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.language_var = tk.StringVar(value="—")
        self.command_var = tk.StringVar()
        self.voice_enabled_var = tk.BooleanVar(value=False)
        self.voice_status_var = tk.StringVar(value="Voice stopped")
        self.activity_var = tk.StringVar(value=self._activity_pair[1])

        # Only these four checkboxes are operator feature switches.
        self.adaptive_var = tk.BooleanVar(value=True)
        self.music_var = tk.BooleanVar(value=False)
        self.vision_var = tk.BooleanVar(value=True)
        self.webcam_var = tk.BooleanVar(value=False)
        self.camera_var = tk.StringVar(value="Auto")

        self.manip_vars: Dict[str, tk.DoubleVar] = {
            key: tk.DoubleVar(value=value)
            for key, value in zip(MANIP_KEYS, (180.0, 150.0, 70.0, 90.0, 95.0, 130.0, 0.0))
        }
        self.vehicle_vars: Dict[str, tk.DoubleVar] = {
            "throttle": tk.DoubleVar(value=0.0),
            "steer": tk.DoubleVar(value=0.0),
            "cam_pan": tk.DoubleVar(value=0.0),
            "cam_tilt": tk.DoubleVar(value=0.0),
        }
        self.drone_vars: Dict[str, tk.DoubleVar] = {
            "throttle": tk.DoubleVar(value=0.0),
            "yaw": tk.DoubleVar(value=0.0),
            "pitch": tk.DoubleVar(value=0.0),
            "roll": tk.DoubleVar(value=0.0),
            "strafe": tk.DoubleVar(value=0.0),
            "forward": tk.DoubleVar(value=0.0),
            "cam_pan": tk.DoubleVar(value=0.0),
            "cam_tilt": tk.DoubleVar(value=-10.0),
        }

        self.voice = VoiceInput(self._voice_text_from_thread, self._voice_status_from_thread)
        self._build_styles()
        self._build_ui()
        self._refresh_intentions()
        self._apply_robot_ui_policy()
        self._apply_ui_language(self._ui_language, force=True)
        self._schedule_camera_scan()
        self.root.after(self.POLL_MS, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # ------------------------------------------------------------------ UI
    def _build_styles(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("App.TFrame", background=BG)
        style.configure("Card.TFrame", background=CARD)
        style.configure("CardTitle.TLabel", background=CARD, foreground=TEXT, font=("Segoe UI", 10, "bold"))
        style.configure("CardBody.TLabel", background=CARD, foreground=TEXT, font=("Segoe UI", 8))
        style.configure("CardInfo.TLabel", background=CARD, foreground=BLUE, font=("Segoe UI", 8, "bold"))
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", padding=(10, 5), font=("Segoe UI", 9))
        style.configure("TButton", padding=4, font=("Segoe UI", 8))
        style.configure("TCheckbutton", background=CARD, font=("Segoe UI", 8))
        style.configure("Treeview", rowheight=24, font=("Segoe UI", 8))
        style.configure("Treeview.Heading", font=("Segoe UI", 8, "bold"))

    def _card(self, parent: tk.Misc, title: str) -> ttk.Frame:
        frame = ttk.Frame(parent, style="Card.TFrame", padding=8)
        ttk.Label(frame, text=title, style="CardTitle.TLabel").pack(anchor="w")
        return frame

    def _build_ui(self) -> None:
        self._build_topbar()
        self._build_banner()
        self.body = ttk.PanedWindow(self.root, orient="horizontal")
        self.body.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.left = ttk.Frame(self.body, style="App.TFrame", width=390)
        self.right = ttk.Frame(self.body, style="App.TFrame")
        self.body.add(self.left, weight=1)
        self.body.add(self.right, weight=2)
        self.right.columnconfigure(0, weight=1)
        self.right.rowconfigure(0, weight=1)

        self._build_left_status()
        self._build_right_tabs()
        self.root.after(150, self._set_initial_sash)

    def _build_topbar(self) -> None:
        bar = tk.Frame(self.root, bg="#1f2937", height=48)
        bar.pack(fill="x")
        bar.pack_propagate(False)
        tk.Label(
            bar, text="SynROV Command Center · AiBot V1", bg="#1f2937", fg="white",
            font=("Segoe UI", 13, "bold"), anchor="w",
        ).pack(side="left", padx=(14, 18))

        tk.Label(bar, text="Processing WS", bg="#1f2937", fg="#e5e7eb", font=("Segoe UI", 8)).pack(side="left")
        ttk.Entry(bar, textvariable=self.uri_var, width=27).pack(side="left", padx=(6, 6), pady=9)
        self.connect_button = ttk.Button(bar, text="Connect", command=self._toggle_connection)
        self.connect_button.pack(side="left", padx=(0, 12), pady=8)

        tk.Label(bar, text="Robot", bg="#1f2937", fg="#e5e7eb", font=("Segoe UI", 8)).pack(side="left")
        self.robot_combo = ttk.Combobox(bar, textvariable=self.robot_var, values=ROBOTS, state="readonly", width=13)
        self.robot_combo.pack(side="left", padx=(6, 12), pady=9)
        self.robot_combo.bind("<<ComboboxSelected>>", self._on_robot_selected)

        tk.Label(bar, textvariable=self.connection_var, bg="#1f2937", fg="#d1fae5", font=("Segoe UI", 9, "bold")).pack(side="right", padx=14)
        tk.Label(bar, textvariable=self.language_var, bg="#1f2937", fg="#cbd5e1", font=("Segoe UI", 8)).pack(side="right", padx=4)

    def _build_banner(self) -> None:
        self.banner = tk.Frame(self.root, bg=ACCENT, height=72)
        self.banner.pack(fill="x", padx=8, pady=(8, 6))
        self.banner.pack_propagate(False)
        self.banner_title = tk.Label(
            self.banner, text="AiBot V1 pronto", bg=ACCENT, fg="white",
            font=("Segoe UI", 17, "bold"), anchor="w",
        )
        self.banner_title.pack(fill="x", padx=14, pady=(8, 0))
        self.banner_detail = tk.Label(
            self.banner, textvariable=self.activity_var, bg=ACCENT, fg=ACCENT_SOFT,
            font=("Segoe UI", 9), anchor="w",
        )
        self.banner_detail.pack(fill="x", padx=14, pady=(0, 8))

    def _build_left_status(self) -> None:
        indicators = self._card(self.left, "Visual status")
        indicators.pack(fill="x", pady=(0, 6))
        grid = ttk.Frame(indicators, style="Card.TFrame")
        grid.pack(fill="x", pady=(4, 0))
        for c in range(2):
            grid.columnconfigure(c, weight=1)
        self.tile_conn = VisualStatusTile(grid, "Connection")
        self.tile_telem = VisualStatusTile(grid, "Telemetry")
        self.tile_frame = VisualStatusTile(grid, "Frame")
        self.tile_ctrl = VisualStatusTile(grid, "Control")
        self.tile_ai = VisualStatusTile(grid, "Adaptive AI")
        self.tile_camera = VisualStatusTile(grid, "Camera")
        self.tile_vision = VisualStatusTile(grid, "Vision")
        for i, tile in enumerate((
            self.tile_conn, self.tile_telem, self.tile_frame, self.tile_ctrl,
            self.tile_ai, self.tile_camera, self.tile_vision,
        )):
            tile.grid(row=i // 2, column=i % 2, sticky="nsew", padx=4, pady=4)

        live = self._card(self.left, "Live signals")
        live.pack(fill="x", pady=(0, 6))
        self.live_status = ttk.Label(live, text="Activity: —", style="CardBody.TLabel", wraplength=360, justify="left")
        self.live_status.pack(anchor="w", pady=(4, 4))
        self.telem_bar, self.telem_value = self._meter_row(live, "Telem/s", 40)
        self.frame_bar, self.frame_value = self._meter_row(live, "Frames/s", 30)

        preview = self._card(self.left, "Visualization")
        preview.pack(fill="both", expand=True)
        chips = ttk.Frame(preview, style="Card.TFrame")
        chips.pack(fill="x", pady=(4, 4))
        self.robot_chip = ttk.Label(chips, text="Manipulator", style="CardBody.TLabel")
        self.robot_chip.pack(side="left")
        self.sim_chip = ttk.Label(chips, text="telem 0", style="CardBody.TLabel")
        self.sim_chip.pack(side="left", padx=6)
        self.frame_chip = ttk.Label(chips, text="frames 0", style="CardBody.TLabel")
        self.frame_chip.pack(side="left", padx=6)

        self.preview_panel = tk.Label(
            preview, bg="#d9dee3", fg=MUTED, text="Waiting for Processing frame",
            anchor="center", compound="center", height=11,
        )
        self.preview_panel.pack(fill="x", pady=(0, 4))
        self.webcam_panel = tk.Label(
            preview, bg="#eef1f4", fg=MUTED, text="Physical webcam off",
            anchor="center", compound="center", height=6,
        )
        self.webcam_panel.pack(fill="x")
        self.preview_info = ttk.Label(preview, text="Camera port 9002: Processing/generated stream", style="CardBody.TLabel", wraplength=360)
        self.preview_info.pack(anchor="w", pady=(4, 0))

    def _meter_row(self, parent: tk.Misc, label: str, maximum: float) -> tuple[ttk.Progressbar, ttk.Label]:
        row = ttk.Frame(parent, style="Card.TFrame")
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=label, width=11, style="CardBody.TLabel").pack(side="left")
        bar = ttk.Progressbar(row, maximum=maximum)
        bar.pack(side="left", fill="x", expand=True, padx=4)
        value = ttk.Label(row, text="0", style="CardInfo.TLabel")
        value.pack(side="left")
        return bar, value

    def _build_right_tabs(self) -> None:
        self.notebook = ttk.Notebook(self.right)
        self.notebook.grid(row=0, column=0, sticky="nsew")
        self.tab_operation = ScrollablePane(self.notebook)
        self.tab_intents = ttk.Frame(self.notebook, style="App.TFrame", padding=8)
        self.tab_learning = ScrollablePane(self.notebook)
        self.tab_log = ttk.Frame(self.notebook, style="App.TFrame", padding=8)
        self.notebook.add(self.tab_operation, text="Operation")
        self.notebook.add(self.tab_intents, text="Intentions")
        self.notebook.add(self.tab_learning, text="Learning")
        self.notebook.add(self.tab_log, text="Log / State")
        self._build_operation_tab()
        self._build_intentions_tab()
        self._build_learning_tab()
        self._build_log_tab()

    def _build_operation_tab(self) -> None:
        host = self.tab_operation.body
        auto = self._card(host, "Automation")
        auto.pack(fill="x", pady=(0, 6))
        row = ttk.Frame(auto, style="Card.TFrame")
        row.pack(fill="x", pady=4)
        self.adaptive_check = ttk.Checkbutton(row, text="1 · Adaptive AI", variable=self.adaptive_var, command=self._apply_features)
        self.music_check = ttk.Checkbutton(row, text="2 · Music / rhythm", variable=self.music_var, command=self._apply_features)
        self.vision_check = ttk.Checkbutton(row, text="3 · Vision / object", variable=self.vision_var, command=self._apply_features)
        self.webcam_check = ttk.Checkbutton(row, text="4 · Webcam", variable=self.webcam_var, command=self._apply_features)
        for widget in (self.adaptive_check, self.music_check, self.vision_check, self.webcam_check):
            widget.pack(side="left", padx=(0, 12))

        camrow = ttk.Frame(auto, style="Card.TFrame")
        camrow.pack(fill="x", pady=(3, 0))
        ttk.Label(camrow, text="Physical camera", style="CardBody.TLabel").pack(side="left")
        self.camera_combo = ttk.Combobox(camrow, textvariable=self.camera_var, values=("Auto",), state="readonly", width=38)
        self.camera_combo.pack(side="left", padx=8)
        self.camera_combo.bind("<<ComboboxSelected>>", lambda _event: self._apply_camera_selection())
        ttk.Button(camrow, text="Refresh", command=self._schedule_camera_scan).pack(side="left")

        policy = self._card(host, "Internal AI policy")
        policy.pack(fill="x", pady=(0, 6))
        ttk.Label(
            policy,
            text=(
                "Learning protection, shadow validation, arbitration, sensor fusion, safety, orchestration and "
                "performance decisions are automatic. The interface does not expose internal controls that "
                "could break robot autonomy."
            ),
            style="CardBody.TLabel", wraplength=820, justify="left",
        ).pack(anchor="w", pady=(4, 0))

        manual = self._card(host, "Manual control")
        manual.pack(fill="x", pady=(0, 6))
        self.manual_stack = ttk.Frame(manual, style="Card.TFrame")
        self.manual_stack.pack(fill="x", pady=(4, 0))
        self.manip_frame = ttk.Frame(self.manual_stack, style="Card.TFrame")
        self.vehicle_frame = ttk.Frame(self.manual_stack, style="Card.TFrame")
        self.drone_frame = ttk.Frame(self.manual_stack, style="Card.TFrame")
        for frame in (self.manip_frame, self.vehicle_frame, self.drone_frame):
            frame.grid(row=0, column=0, sticky="nsew")
        self.manual_stack.columnconfigure(0, weight=1)
        self._build_manip_manual()
        self._build_vehicle_manual()
        self._build_drone_manual()

        command = self._card(host, "Quick command / voice")
        command.pack(fill="x", pady=(0, 6))
        line = ttk.Frame(command, style="Card.TFrame")
        line.pack(fill="x", pady=(4, 2))
        entry = ttk.Entry(line, textvariable=self.command_var)
        entry.pack(side="left", fill="x", expand=True)
        entry.bind("<Return>", lambda _event: self._send_command())
        ttk.Button(line, text="Send", command=self._send_command).pack(side="left", padx=(5, 0))
        self.voice_button = ttk.Checkbutton(line, text="Microphone", variable=self.voice_enabled_var, command=self._toggle_voice)
        self.voice_button.pack(side="left", padx=(10, 0))
        ttk.Label(command, textvariable=self.voice_status_var, style="CardBody.TLabel").pack(anchor="w", pady=(2, 0))

    def _slider_cell(
        self,
        parent: tk.Misc,
        row: int,
        col: int,
        label: str,
        variable: tk.DoubleVar,
        lo: float,
        hi: float,
        resolution: float,
        callback: Any,
        key_hint: str = "",
    ) -> None:
        cell = ttk.Frame(parent, style="Card.TFrame")
        cell.grid(row=row, column=col, sticky="nsew", padx=4, pady=3)
        label_widget = ttk.Label(cell, text=f"{label}{('  ' + key_hint) if key_hint else ''}  ({lo:g}…{hi:g})", style="CardBody.TLabel")
        label_widget._synrov_base_text = label
        label_widget._synrov_key_hint = key_hint
        label_widget._synrov_range_text = f"{lo:g}…{hi:g}"
        label_widget.pack(anchor="w")
        line = ttk.Frame(cell, style="Card.TFrame")
        line.pack(fill="x")
        scale = tk.Scale(
            line, from_=lo, to=hi, resolution=resolution, orient="horizontal", variable=variable,
            showvalue=False, length=205, bg="#f7f8fa", highlightthickness=0,
        )
        scale.pack(side="left", fill="x", expand=True)
        scale.bind("<ButtonRelease-1>", callback)
        ent = ttk.Entry(line, textvariable=variable, width=7)
        ent.pack(side="left", padx=(4, 0))
        ent.bind("<Return>", callback)
        ent.bind("<FocusOut>", callback)

    def _build_manip_manual(self) -> None:
        grid = ttk.Frame(self.manip_frame, style="Card.TFrame")
        grid.pack(fill="x")
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)
        labels = {
            "base": "Base", "upper": "Upper arm", "fore": "Forearm",
            "forearm_roll": "Forearm roll", "wrist_pitch": "Wrist pitch",
            "wrist_rot": "Wrist rotation", "grip": "Gripper",
        }
        key_hints = {
            "base": "[Z− / X+]", "upper": "[W− / S+]", "fore": "[R− / F+]",
            "forearm_roll": "[D− / A+]", "wrist_pitch": "[T− / G+]",
            "wrist_rot": "[E− / Q+]", "grip": "[H− / Y+]",
        }
        for index, key in enumerate(MANIP_KEYS):
            lo, hi = MANIP_POSE_LIMITS[key]
            self._slider_cell(grid, index // 2, index % 2, labels[key], self.manip_vars[key], lo, hi, 1.0, self._send_manip_manual, key_hints[key])
        btns = ttk.Frame(self.manip_frame, style="Card.TFrame")
        btns.pack(fill="x", pady=(5, 0))
        ttk.Button(btns, text="HOME", command=self._home_manipulator).pack(side="left")
        ttk.Button(btns, text="Send pose", command=self._send_manip_manual).pack(side="left", padx=5)

    def _build_vehicle_manual(self) -> None:
        grid = ttk.Frame(self.vehicle_frame, style="Card.TFrame")
        grid.pack(fill="x")
        for c in range(2):
            grid.columnconfigure(c, weight=1)
        specs = (
            ("Throttle", "throttle", -1.0, 1.0, 0.01, "[S− / W+]"),
            ("Steer", "steer", -1.0, 1.0, 0.01, "[A− / D+]"),
            ("Camera pan", "cam_pan", CAMERA_PAN_MIN_DEG, CAMERA_PAN_MAX_DEG, 1.0, "[X− / Z+]"),
            ("Camera tilt", "cam_tilt", CAMERA_TILT_MIN_DEG, CAMERA_TILT_MAX_DEG, 1.0, "[R− / F+]"),
        )
        for i, (label, key, lo, hi, res, key_hint) in enumerate(specs):
            self._slider_cell(grid, i // 2, i % 2, label, self.vehicle_vars[key], lo, hi, res, self._send_vehicle_manual, key_hint)
        btns = ttk.Frame(self.vehicle_frame, style="Card.TFrame")
        btns.pack(fill="x", pady=(5, 0))
        ttk.Button(btns, text="STOP", command=self._stop_vehicle).pack(side="left")
        ttk.Button(btns, text="Center camera", command=lambda: self._center_camera("Vehicle")).pack(side="left", padx=5)

    def _build_drone_manual(self) -> None:
        grid = ttk.Frame(self.drone_frame, style="Card.TFrame")
        grid.pack(fill="x")
        for c in range(2):
            grid.columnconfigure(c, weight=1)
        specs = (
            ("Throttle", "throttle", -1.0, 1.0, 0.01, "[G− / T+]"),
            ("Yaw", "yaw", -1.0, 1.0, 0.01, "[E− / Q+]"),
            ("Pitch", "pitch", -1.0, 1.0, 0.01, ""),
            ("Roll", "roll", -1.0, 1.0, 0.01, ""),
            ("Strafe", "strafe", -1.0, 1.0, 0.01, "[D− / A+]"),
            ("Forward", "forward", -1.0, 1.0, 0.01, "[S− / W+]"),
            ("Camera pan", "cam_pan", CAMERA_PAN_MIN_DEG, CAMERA_PAN_MAX_DEG, 1.0, "[X− / Z+]"),
            ("Camera tilt", "cam_tilt", CAMERA_TILT_MIN_DEG, CAMERA_TILT_MAX_DEG, 1.0, "[R− / F+]"),
        )
        for i, (label, key, lo, hi, res, key_hint) in enumerate(specs):
            self._slider_cell(grid, i // 2, i % 2, label, self.drone_vars[key], lo, hi, res, self._send_drone_manual, key_hint)
        btns = ttk.Frame(self.drone_frame, style="Card.TFrame")
        btns.pack(fill="x", pady=(5, 0))
        ttk.Button(btns, text="TAKEOFF", command=self.bridge.drone_takeoff).pack(side="left")
        ttk.Button(btns, text="LAND", command=self.bridge.drone_land).pack(side="left", padx=5)
        ttk.Button(btns, text="Hover", command=self._hover_drone).pack(side="left")
        ttk.Button(btns, text="Center camera", command=lambda: self._center_camera("Drone")).pack(side="left", padx=5)

    def _build_intentions_tab(self) -> None:
        header = ttk.Frame(self.tab_intents, style="App.TFrame")
        header.pack(fill="x")
        self.intentions_title = ttk.Label(header, text="Commands and intentions — Manipulator", font=("Segoe UI", 11, "bold"))
        self.intentions_title.pack(side="left")
        self.learn_button = ttk.Button(header, text="New learning", command=self._learn_new_alias)
        self.learn_button.pack(side="right")

        columns = ("type", "command", "intent", "description")
        container = ttk.Frame(self.tab_intents, style="App.TFrame")
        container.pack(fill="both", expand=True, pady=(8, 0))
        self.intent_tree = ttk.Treeview(container, columns=columns, show="headings")
        headings = {"type": "Type", "command": "Command", "intent": "Intention", "description": "Description"}
        for key, text in headings.items():
            self.intent_tree.heading(key, text=text)
        self.intent_tree.column("type", width=85, stretch=False)
        self.intent_tree.column("command", width=190)
        self.intent_tree.column("intent", width=170)
        self.intent_tree.column("description", width=500)
        scroll = ttk.Scrollbar(container, orient="vertical", command=self.intent_tree.yview)
        self.intent_tree.configure(yscrollcommand=scroll.set)
        self.intent_tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.intent_tree.bind("<Double-1>", self._execute_selected_intention)

    def _build_learning_tab(self) -> None:
        host = self.tab_learning.body
        card = self._card(host, "New learning")
        card.pack(fill="x", pady=(0, 6))
        self.learning_info = ttk.Label(
            card,
            text=(
                "Mission-strategy learning and long-context phrase learning are active for all robots. "
                "Learned language never bypasses the safety policy."
            ),
            style="CardBody.TLabel", wraplength=820, justify="left",
        )
        self.learning_info.pack(anchor="w", pady=(4, 6))
        ttk.Button(card, text="Open learning for selected intention", command=self._learn_new_alias).pack(anchor="w")

        learned = self._card(host, "Phrases learned for this robot")
        learned.pack(fill="both", expand=True)
        self.learned_text = tk.Text(learned, height=15, wrap="word", state="disabled", bg="#fbfcfd", relief="flat")
        self.learned_text.pack(fill="both", expand=True, pady=(4, 0))

    def _build_log_tab(self) -> None:
        state_card = self._card(self.tab_log, "State")
        state_card.pack(fill="x", pady=(0, 6))
        self.state_text = tk.Text(state_card, height=10, wrap="word", state="disabled", bg="#fbfcfd", relief="flat")
        self.state_text.pack(fill="x", pady=(4, 0))
        log_card = self._card(self.tab_log, "Log")
        log_card.pack(fill="both", expand=True)
        self.log_text = tk.Text(log_card, wrap="word", state="disabled", bg="#111827", fg="#e5e7eb", insertbackground="white", relief="flat")
        self.log_text.pack(fill="both", expand=True, pady=(4, 0))

    def _set_initial_sash(self) -> None:
        """Open the main drag divider at the real center of the client area."""
        if self._initial_sash_centered:
            return
        try:
            self.root.update_idletasks()
            width = int(self.body.winfo_width())
            if width <= 10:
                self.root.after(80, self._set_initial_sash)
                return
            self.body.sashpos(0, max(1, width // 2))
            self._initial_sash_centered = True
        except Exception:
            # Layout can still be settling on the first Tk frame. Retry once the
            # widget hierarchy has a measurable width instead of leaving the
            # divider at an edge.
            self.root.after(80, self._set_initial_sash)

    # ----------------------------------------------------------- connections
    def _toggle_connection(self) -> None:
        if self.bridge.state.connected:
            self.bridge.disconnect()
            return
        self.bridge.connect(self.uri_var.get().strip() or DEFAULT_URI)

    def _on_robot_selected(self, _event: Any = None) -> None:
        robot = canonical_robot(self.robot_var.get())
        self.bridge.state.robot = robot
        self.bridge.set_mode(robot)
        self.runtime.registry.sync_active_robot(self.bridge)
        self._apply_robot_ui_policy()
        self._refresh_intentions()
        self._apply_features()

    def _show_robot_panel(self) -> None:
        robot = canonical_robot(self.robot_var.get())
        {"Manipulator": self.manip_frame, "Vehicle": self.vehicle_frame, "Drone": self.drone_frame}[robot].tkraise()

    def _apply_robot_ui_policy(self) -> None:
        robot = canonical_robot(self.robot_var.get())
        self._show_robot_panel()
        self.learn_button.state(["!disabled"])
        if robot == "Manipulator":
            self.music_check.state(["!disabled"])
        else:
            self.music_var.set(False)
            self.music_check.state(["disabled"])
        self.runtime.set_music(self.music_var.get())

    # --------------------------------------------------------------- features
    def _apply_features(self) -> None:
        self.runtime.set_adaptive(self.adaptive_var.get())
        self.runtime.set_music(self.music_var.get())
        self.runtime.set_vision(self.vision_var.get())
        if self.webcam_var.get():
            self._apply_camera_selection()
        else:
            self.runtime.set_webcam(False)

    def _apply_camera_selection(self) -> None:
        label = self.camera_var.get() or "Auto"
        index = self._camera_label_to_index.get(label, WebcamSource.AUTO_INDEX)
        device = self.runtime.set_webcam(self.webcam_var.get(), index)
        if self.webcam_var.get() and device is None:
            self._log(ui_text(self._ui_language, "Webcam física indisponível; a porta 9002 permanece no stream gerado pelo Processing.", "Physical webcam unavailable; port 9002 remains on the Processing-generated stream."))

    def _schedule_camera_scan(self) -> None:
        if self._camera_scan_running:
            return
        self._camera_scan_running = True

        def worker() -> None:
            try:
                devices = self.runtime.refresh_camera_devices()
            except Exception:
                devices = []
            try:
                self._camera_scan_result.put_nowait(devices)
            except queue.Full:
                pass

        threading.Thread(target=worker, daemon=True, name="synrov-camera-discovery").start()

    def _consume_camera_scan(self) -> None:
        try:
            devices = self._camera_scan_result.get_nowait()
        except queue.Empty:
            return
        self._camera_scan_running = False
        mapping: Dict[str, int] = {"Auto": WebcamSource.AUTO_INDEX}
        for device in devices:
            mapping[device.label_pt] = device.index
        self._camera_label_to_index = mapping
        self.camera_combo.configure(values=tuple(mapping.keys()))
        current = self.runtime.camera_index
        self.camera_var.set(next((label for label, idx in mapping.items() if idx == current), "Auto"))
        if self.webcam_var.get():
            self._apply_camera_selection()

    # ---------------------------------------------------------- manual control
    def _send_manip_manual(self, _event: Any = None) -> None:
        pose = {key: self.manip_vars[key].get() for key in MANIP_KEYS}
        self.bridge.command_manipulator_pose(pose)
        self._set_activity("Controle manual do Manipulator enviado.", "Manipulator manual control sent.")

    def _home_manipulator(self) -> None:
        home = {"base": 180.0, "upper": 150.0, "fore": 70.0, "forearm_roll": 90.0, "wrist_pitch": 95.0, "wrist_rot": 130.0, "grip": 0.0}
        for key, value in home.items():
            self.manip_vars[key].set(value)
        self.bridge.manip_home()

    def _send_vehicle_manual(self, _event: Any = None) -> None:
        v = self.vehicle_vars
        self.bridge.send_vehicle(v["throttle"].get(), v["steer"].get(), v["cam_pan"].get(), v["cam_tilt"].get())
        self._set_activity("Controle manual do Vehicle enviado.", "Vehicle manual control sent.")

    def _stop_vehicle(self) -> None:
        self.vehicle_vars["throttle"].set(0.0)
        self.vehicle_vars["steer"].set(0.0)
        self._send_vehicle_manual()

    def _send_drone_manual(self, _event: Any = None) -> None:
        v = self.drone_vars
        self.bridge.send_drone(
            v["throttle"].get(), v["yaw"].get(), v["pitch"].get(), v["roll"].get(),
            v["strafe"].get(), v["forward"].get(), cam_pan=v["cam_pan"].get(), cam_tilt=v["cam_tilt"].get(),
        )
        self._set_activity("Controle manual do Drone enviado.", "Drone manual control sent.")

    def _hover_drone(self) -> None:
        for key in ("throttle", "yaw", "pitch", "roll", "strafe", "forward"):
            self.drone_vars[key].set(0.0)
        self._send_drone_manual()

    def _center_camera(self, robot: str) -> None:
        pose = default_camera_pose(robot)
        vars_ = self.vehicle_vars if robot == "Vehicle" else self.drone_vars
        vars_["cam_pan"].set(pose.pan_deg)
        vars_["cam_tilt"].set(pose.tilt_deg)
        self.bridge.set_camera(robot, pan=pose.pan_deg, tilt=pose.tilt_deg)

    # --------------------------------------------------------- commands/voice
    def _send_command(self, text: Optional[str] = None) -> None:
        phrase = (text if text is not None else self.command_var.get()).strip()
        if not phrase:
            return
        result = self.runtime.execute_text(phrase)
        self._set_activity(f"{result.robot}: {phrase} → {result.kind}:{result.name or '—'}", f"{result.robot}: {phrase} → {result.kind}:{result.name or '—'}")
        self._log(f"[{result.robot}] {phrase} → {result.kind}:{result.name or '-'} · {result.reason or 'ok'}")
        if text is None:
            self.command_var.set("")

    def _execute_selected_intention(self, _event: Any = None) -> None:
        selected = self.intent_tree.selection()
        if not selected:
            return
        values = self.intent_tree.item(selected[0], "values")
        if not values or values[0] in {"Aprendido", "Learned"}:
            return
        self._send_command(str(values[1]))

    def _toggle_voice(self) -> None:
        if not self.voice_enabled_var.get():
            self.voice.stop()
            self._voice_status_code = "stopped"
            self.voice_status_var.set(self._format_voice_status("stopped"))
            return
        try:
            self.voice.start(speech_locale(self._ui_language))
        except Exception as exc:
            self.voice_enabled_var.set(False)
            self._voice_status_code = f"microphone_error:{exc}"
            self.voice_status_var.set(self._format_voice_status(self._voice_status_code))

    def _voice_text_from_thread(self, text: str) -> None:
        if not self._closing:
            self.root.after(0, lambda: self._accept_voice_text(text))

    def _accept_voice_text(self, text: str) -> None:
        phrase = str(text or "").strip()
        if not phrase:
            return
        self.command_var.set(phrase)
        self._send_command(phrase)

    def _voice_status_from_thread(self, status: str) -> None:
        if not self._closing:
            self.root.after(0, lambda: self._apply_voice_status(status))

    def _apply_voice_status(self, status: str) -> None:
        self._voice_status_code = str(status or "stopped")
        self.voice_status_var.set(self._format_voice_status(self._voice_status_code))

    def _format_voice_status(self, status: str) -> str:
        code = str(status or "stopped")
        lang = self._ui_language
        if code.startswith("listening:"):
            device = code.split(":", 1)[1].strip()
            return ui_format(lang, "Listening · {device}", "Ouvindo · {device}", device=device)
        if code.startswith("heard:"):
            phrase = code.split(":", 1)[1].strip()
            return ui_format(lang, "Recognized: {phrase}", "Reconhecido: {phrase}", phrase=phrase)
        if code.startswith("language:"):
            return ui_text(lang, "Idioma do microfone sincronizado", "Microphone language synchronized")
        if code.startswith("microphone_error:"):
            detail = code.split(":", 1)[1].strip()
            return ui_format(lang, "Microphone error: {detail}", "Erro de microfone: {detail}", detail=detail)
        if code.startswith("recognition_error:"):
            detail = code.split(":", 1)[1].strip()
            return ui_format(lang, "Recognition unavailable: {detail}", "Reconhecimento indisponível: {detail}", detail=detail)
        mapping = {
            "stopped": ("Voz parada", "Voice stopped"),
            "calibrating": ("Calibrando ruído ambiente…", "Calibrating ambient noise…"),
            "recognizing": ("Reconhecendo frase…", "Recognizing phrase…"),
            "unrecognized": ("Frase não reconhecida", "Phrase not recognized"),
            "overflow": ("Áudio sobrecarregado; continuando…", "Audio overflow; continuing…"),
        }
        pt, en = mapping.get(code, (code, code))
        return ui_text(lang, pt, en)

    # -------------------------------------------------------------- intentions
    def _refresh_intentions(self) -> None:
        robot = canonical_robot(self.robot_var.get())
        lang = self._ui_language
        self.intentions_title.configure(text=ui_format(lang, "Commands and intentions — {robot}", "Comandos e intenções — {robot}", robot=robot))
        for item in self.intent_tree.get_children():
            self.intent_tree.delete(item)
        for spec in all_specs_for_robot(robot):
            kind = ui_text(lang, "Missão", "Mission") if spec.kind == "mission" else ui_text(lang, "Comando", "Command")
            self.intent_tree.insert("", "end", values=(kind, spec.command(lang), spec.intent, spec.description(lang)))
        for phrase, item in sorted(self.runtime.registry.learned_aliases(robot).items()):
            self.intent_tree.insert(
                "", "end",
                values=(ui_text(lang, "Aprendido", "Learned"), phrase, item.get("name", ""), ui_text(lang, "Alias aprendido nesta sessão.", "Alias learned this session.")),
            )
        self._refresh_learned_text()

    def _learn_new_alias(self) -> None:
        robot = canonical_robot(self.robot_var.get())
        selected = self.intent_tree.selection()
        if not selected:
            messagebox.showinfo(ui_text(self._ui_language, "Aprendizado novo", "New learning"), ui_text(self._ui_language, "Selecione uma intenção ou missão do robô ativo.", "Select an intention or mission for the active robot."), parent=self.root)
            return
        values = self.intent_tree.item(selected[0], "values")
        if not values or values[0] in {"Aprendido", "Learned"}:
            messagebox.showinfo(ui_text(self._ui_language, "Aprendizado novo", "New learning"), ui_text(self._ui_language, "Selecione uma intenção ou missão original.", "Select an original intention or mission."), parent=self.root)
            return
        phrase = simpledialog.askstring(ui_text(self._ui_language, "Aprendizado novo", "New learning"), ui_text(self._ui_language, "Nova frase para esta intenção:", "New phrase for this intention:"), parent=self.root)
        if not phrase:
            return
        kind = "mission" if values[0] in {"Missão", "Mission"} else "intent"
        intent = str(values[2])
        if self.runtime.learn_alias(phrase, kind, intent):
            self._log(ui_format(self._ui_language, "[{robot}] Learned: {phrase} → {kind}:{intent}", "[{robot}] Aprendido: {phrase} → {kind}:{intent}", robot=robot, phrase=phrase, kind=kind, intent=intent))
            self._refresh_intentions()
        else:
            messagebox.showerror(ui_text(self._ui_language, "Aprendizado novo", "New learning"), ui_text(self._ui_language, "Não foi possível registrar esta frase.", "Could not register this phrase."), parent=self.root)

    def _refresh_learned_text(self) -> None:
        robot = canonical_robot(self.robot_var.get())
        aliases = self.runtime.registry.learned_aliases(robot)
        lines = [f"{phrase} → {item.get('kind', 'intent')}:{item.get('name', '')}" for phrase, item in sorted(aliases.items())]
        if not lines:
            lines = [ui_text(self._ui_language, "Nenhuma nova frase ensinada para este robô.", "No new phrase has been taught for this robot yet.")]
        self.learned_text.configure(state="normal")
        self.learned_text.delete("1.0", "end")
        self.learned_text.insert("1.0", "\n".join(lines))
        self.learned_text.configure(state="disabled")

    # ------------------------------------------------------------- language
    def _preferred_ui_font_family(self, language: str) -> str:
        """Pick a Unicode-capable font already installed on the host OS."""
        lang = language_id(language)
        candidates = {
            "zh-cn": ("Microsoft YaHei UI", "Microsoft YaHei", "PingFang SC", "Noto Sans CJK SC", "Noto Sans SC"),
            "ja": ("Yu Gothic UI", "Yu Gothic", "Meiryo UI", "Meiryo", "Hiragino Sans", "Noto Sans CJK JP"),
            "ar": ("Segoe UI", "Tahoma", "Arial", "Noto Sans Arabic", "DejaVu Sans"),
        }.get(lang, ("Segoe UI", "Arial", "Helvetica", "DejaVu Sans", "Noto Sans"))
        try:
            installed = {name.casefold(): name for name in tkfont.families(self.root)}
        except Exception:
            installed = {}
        for family in candidates:
            found = installed.get(family.casefold())
            if found:
                return found
        try:
            return str(tkfont.nametofont("TkDefaultFont", root=self.root).actual("family"))
        except Exception:
            return "Arial"

    def _apply_ui_font_family(self, language: str) -> None:
        """Apply language-aware system fonts without bundling external files."""
        family = self._preferred_ui_font_family(language)
        style = ttk.Style(self.root)
        style.configure("CardTitle.TLabel", font=(family, 10, "bold"))
        style.configure("CardBody.TLabel", font=(family, 8))
        style.configure("CardInfo.TLabel", font=(family, 8, "bold"))
        style.configure("TNotebook.Tab", font=(family, 9))
        style.configure("TButton", font=(family, 8))
        style.configure("TCheckbutton", font=(family, 8))
        style.configure("Treeview", font=(family, 8))
        style.configure("Treeview.Heading", font=(family, 8, "bold"))

        def apply_tk_font(widget: tk.Misc) -> None:
            # Only classic Tk widgets need direct family replacement; ttk uses
            # the styles above. Text/log widgets intentionally keep their mono
            # or editor defaults unless they are labels used for UI captions.
            if isinstance(widget, tk.Label):
                try:
                    current = tkfont.Font(font=widget.cget("font"), root=self.root)
                    actual = current.actual()
                    size = int(actual.get("size") or 9)
                    weight = str(actual.get("weight") or "normal")
                    slant = str(actual.get("slant") or "roman")
                    options = []
                    if weight == "bold":
                        options.append("bold")
                    if slant == "italic":
                        options.append("italic")
                    widget.configure(font=(family, size, *options))
                except Exception:
                    pass
            try:
                children = widget.winfo_children()
            except Exception:
                children = []
            for child in children:
                apply_tk_font(child)

        apply_tk_font(self.root)

    def _set_activity(self, pt: str, en: str) -> None:
        self._activity_pair = (str(pt), str(en))
        self.activity_var.set(ui_text(self._ui_language, self._activity_pair[0], self._activity_pair[1]))

    def _translate_widget_tree(self, widget: tk.Misc) -> None:
        base_text = getattr(widget, "_synrov_base_text", None)
        if isinstance(base_text, str) and base_text:
            key_hint = str(getattr(widget, "_synrov_key_hint", "") or "")
            range_text = str(getattr(widget, "_synrov_range_text", "") or "")
            translated_base = tr_text(base_text, self._ui_language)
            suffix = f"  {key_hint}" if key_hint else ""
            if range_text:
                suffix += f"  ({range_text})"
            try:
                widget.configure(text=translated_base + suffix)
            except Exception:
                pass
            current = None
        else:
            try:
                current = widget.cget("text")
            except Exception:
                current = None
        if isinstance(current, str) and current:
            translated = tr_text(current, self._ui_language)
            if translated != current:
                try:
                    widget.configure(text=translated)
                except Exception:
                    pass
        try:
            children = widget.winfo_children()
        except Exception:
            children = []
        for child in children:
            self._translate_widget_tree(child)

    def _apply_ui_language(self, language: str, *, force: bool = False) -> None:
        lang = language_id(language)
        if not force and lang == self._ui_language:
            # Voice can still have been started after the UI language was set.
            self.voice.set_language(speech_locale(lang))
            return
        self._ui_language = lang
        self.voice.set_language(speech_locale(lang))
        self._apply_ui_font_family(lang)
        self._translate_widget_tree(self.root)
        if hasattr(self, "notebook"):
            tab_texts = (
                ui_text(lang, "Operação", "Operation"),
                ui_text(lang, "Intenções", "Intentions"),
                ui_text(lang, "Aprendizado", "Learning"),
                ui_text(lang, "Log / Estado", "Log / State"),
            )
            for index, label in enumerate(tab_texts):
                self.notebook.tab(index, text=label)
        if hasattr(self, "intent_tree"):
            headings = {
                "type": ui_text(lang, "Tipo", "Type"),
                "command": ui_text(lang, "Comando", "Command"),
                "intent": ui_text(lang, "Intenção", "Intention"),
                "description": ui_text(lang, "Descrição", "Description"),
            }
            for key, label in headings.items():
                self.intent_tree.heading(key, text=label)
            self._refresh_intentions()
        self.voice_status_var.set(self._format_voice_status(self._voice_status_code))
        self.activity_var.set(ui_text(lang, self._activity_pair[0], self._activity_pair[1]))
        prefix = ui_text(lang, "Idioma", "Language")
        current = language_name(self.bridge.state.language or lang)
        self.language_var.set(f"{prefix}: {current}")

    def _sync_language_from_processing(self) -> None:
        incoming = str(self.bridge.state.language or "").strip()
        if not incoming:
            return
        if language_id(incoming) != self._ui_language:
            self._apply_ui_language(incoming)

    # --------------------------------------------------------------- refresh
    def _sync_robot_from_processing(self) -> None:
        robot = canonical_robot(self.bridge.state.robot)
        if self.robot_var.get() != robot:
            self.robot_var.set(robot)
            self._apply_robot_ui_policy()
            self._refresh_intentions()

    def _update_rates(self) -> None:
        now = time.monotonic()
        dt = now - self._last_rate_ts
        if dt < 0.8:
            return
        state = self.bridge.state
        self._telem_rate = max(0.0, (state.telemetry_counter - self._last_telem_counter) / max(dt, 0.001))
        self._frame_rate = max(0.0, (state.frame_counter - self._last_frame_counter) / max(dt, 0.001))
        self._last_telem_counter = state.telemetry_counter
        self._last_frame_counter = state.frame_counter
        self._last_rate_ts = now

    def _update_preview(self, panel: tk.Label, image: Any, size: tuple[int, int], attr: str, empty_text: str) -> None:
        if image is None or Image is None or ImageTk is None:
            setattr(self, attr, None)
            panel.configure(image="", text=empty_text)
            return
        try:
            img = image.copy()
            img.thumbnail(size)
            photo = ImageTk.PhotoImage(img)
            setattr(self, attr, photo)
            panel.configure(image=photo, text="")
        except Exception:
            setattr(self, attr, None)
            panel.configure(image="", text=ui_text(self._ui_language, "Frame indisponível", "Frame unavailable"))

    def _refresh_visuals(self) -> None:
        status = self.runtime.status()
        state = self.bridge.state
        robot = status.get("active_robot", "Manipulator")
        features = status.get("intelligence", {}).get("features", {})
        telemetry = features.get("telemetry", {}) if isinstance(features, dict) else {}
        cam9002 = status.get("robot_camera_9002", {})
        vision = state.vision_info or {}

        connected = bool(status.get("connected"))
        self.connection_var.set(ui_text(self._ui_language, "Conectado", "Connected") if connected else ui_text(self._ui_language, "Desconectado", "Disconnected"))
        self.connect_button.configure(text=ui_text(self._ui_language, "Desconectar", "Disconnect") if connected else ui_text(self._ui_language, "Conectar", "Connect"))
        language = str(status.get("language") or "")
        prefix = ui_text(self._ui_language, "Idioma", "Language")
        self.language_var.set(f"{prefix}: {language_name(language) if language else '—'}")
        self.banner_title.configure(text=f"{robot} · {'ONLINE' if connected else 'OFFLINE'}")
        self.banner.configure(bg=ACCENT if connected else "#475569")
        self.banner_title.configure(bg=ACCENT if connected else "#475569")
        self.banner_detail.configure(bg=ACCENT if connected else "#475569")

        self.tile_conn.set("ONLINE" if connected else "OFFLINE", "ok" if connected else "bad")
        telem_fresh = time.time() - state.last_telemetry_ts < 1.5 if state.last_telemetry_ts else False
        frame_fresh = time.time() - state.last_frame_ts < 2.5 if state.last_frame_ts else False
        self.tile_telem.set(f"{self._telem_rate:.1f}/s", "ok" if telem_fresh else "warn")
        self.tile_frame.set(f"{self._frame_rate:.1f}/s", "ok" if frame_fresh else "warn")
        last = self.runtime.last_result
        self.tile_ctrl.set(f"{last.kind}:{last.name or '—'}", "ok" if last.ok else "neutral")
        self.tile_ai.set("ON" if self.runtime.adaptive_enabled else "OFF", "ok" if self.runtime.adaptive_enabled else "warn")
        camera_label = "USB" if self.runtime.webcam.enabled else str(cam9002.get("source", cam9002.get("last_source", "fallback")))
        self.tile_camera.set(camera_label, "ok" if self.runtime.webcam.enabled else "info")
        self.tile_vision.set(f"{float(vision.get('confidence', 0.0)):.0%}" if vision else "—", "ok" if vision else "neutral")
        heading = float(telemetry.get("heading_deg", 0.0) or 0.0)
        heading_available = bool(telemetry.get("heading_available", False))

        self.telem_bar["value"] = min(40.0, self._telem_rate)
        self.telem_value.configure(text=f"{self._telem_rate:.1f}")
        self.frame_bar["value"] = min(30.0, self._frame_rate)
        self.frame_value.configure(text=f"{self._frame_rate:.1f}")
        self.live_status.configure(text=f"{ui_text(self._ui_language, 'Atividade', 'Activity')}: {self.activity_var.get()}")
        self.robot_chip.configure(text=robot)
        self.sim_chip.configure(text=f"telem {state.telemetry_counter}")
        self.frame_chip.configure(text=f"frames {state.frame_counter}")

        self._update_preview(
            self.preview_panel, state.last_frame_pil, self.PREVIEW_SIZE, "_preview_photo",
            ui_text(self._ui_language, "Aguardando frame do Processing", "Waiting for Processing frame"),
        )
        self._update_preview(
            self.webcam_panel, self.runtime.webcam.last_frame_pil if self.runtime.webcam.enabled else None, self.WEBCAM_SIZE, "_webcam_photo",
            ui_text(self._ui_language, "Webcam física desligada", "Physical webcam off"),
        )
        source = cam9002.get("source", cam9002.get("last_source", "fallback"))
        self.preview_info.configure(text=ui_format(
            self._ui_language,
            "Port 9002 · source: {source} · physical camera: {backend}",
            "Porta 9002 · fonte: {source} · câmera física: {backend}",
            source=source, backend=self.runtime.webcam.backend_name,
        ))

        camera = status.get("camera", {})
        catalog = status.get("catalog", {})
        connected_text = ui_text(self._ui_language, "conectado", "connected") if connected else ui_text(self._ui_language, "desconectado", "disconnected")
        heading_text = ui_text(self._ui_language, "disponível", "available") if heading_available else ui_text(self._ui_language, "sem leitura", "no reading")
        webcam_text = ui_text(self._ui_language, "ativa", "active") if self.runtime.webcam.enabled else ui_text(self._ui_language, "inativa", "inactive")
        lines = [
            ui_format(self._ui_language, "Version: {version}", "Versão: {version}", version=status.get("softwareVersion")),
            ui_format(self._ui_language, "Active robot: {robot}", "Robô ativo: {robot}", robot=robot),
            ui_format(self._ui_language, "Processing: {status}", "Processing: {status}", status=connected_text),
            ui_format(self._ui_language, "Telemetry heading: {heading:.1f}° ({availability}) · source={source}", "Heading da telemetria: {heading:.1f}° ({availability}) · fonte={source}", heading=heading, availability=heading_text, source=telemetry.get("heading_source", "n/a")),
            ui_format(self._ui_language, "Catalog: {commands} commands · {missions} missions", "Catálogo: {commands} comandos · {missions} missões", commands=catalog.get("commands", 0), missions=catalog.get("missions", 0)),
            ui_format(self._ui_language, "Gimbal: pan {pan_min}°…{pan_max}° · tilt {tilt_min}°…{tilt_max}°", "Gimbal: pan {pan_min}°…{pan_max}° · tilt {tilt_min}°…{tilt_max}°", pan_min=camera.get("pan_min_deg"), pan_max=camera.get("pan_max_deg"), tilt_min=camera.get("tilt_min_deg"), tilt_max=camera.get("tilt_max_deg")),
            ui_format(self._ui_language, "Webcam: {status} · index={index} · backend={backend}", "Webcam: {status} · índice={index} · backend={backend}", status=webcam_text, index=camera.get("active_index"), backend=camera.get("backend")),
            ui_format(self._ui_language, "Video 9002: {source}", "Vídeo 9002: {source}", source=source),
            ui_format(self._ui_language, "Adaptive AI: {status} · protection={protection} · shadow={shadow}", "IA adaptativa: {status} · proteção={protection} · shadow={shadow}", status="ON" if self.runtime.adaptive_enabled else "OFF", protection=self.runtime.learning_protection, shadow=self.runtime.shadow_validation),
            ui_format(self._ui_language, "Last action: {kind}:{name} · {reason}", "Última ação: {kind}:{name} · {reason}", kind=last.kind, name=last.name or "-", reason=last.reason or "ok"),
        ]
        self.state_text.configure(state="normal")
        self.state_text.delete("1.0", "end")
        self.state_text.insert("1.0", "\n".join(lines))
        self.state_text.configure(state="disabled")

    def _log(self, text: Any) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", str(text) + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _poll(self) -> None:
        if self._closing:
            return
        self._consume_camera_scan()
        while True:
            try:
                kind, payload = self.bridge.event_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                self._log(payload)
            elif kind == "state":
                self._sync_robot_from_processing()
                self._sync_language_from_processing()
            elif kind == "language":
                language = payload.get("language") if isinstance(payload, dict) else payload
                if language:
                    self._apply_ui_language(str(language))
            elif kind == "connected":
                self._set_activity("Processing conectado. IA sincronizando sensores e robô ativo.", "Processing connected. AI is synchronizing sensors and active robot.")
                self._log(str(payload or "connected"))
            elif kind == "disconnected":
                self._set_activity("Processing desconectado.", "Processing disconnected.")
                self._log(str(payload or "disconnected"))
            elif kind == "command_result" and payload is not None:
                self._set_activity(f"{payload.robot}: {payload.kind}:{payload.name or '—'}", f"{payload.robot}: {payload.kind}:{payload.name or '—'}")
            elif kind == "mission_tick" and payload is not None:
                learning = payload.metadata.get("learning", {}) if isinstance(getattr(payload, "metadata", None), dict) else {}
                if isinstance(learning, dict) and learning.get("enabled"):
                    variant = str(learning.get("variant", "default"))
                    signature = f"{payload.robot}:{payload.name}:{variant}"
                    now = time.monotonic()
                    changed = bool(learning.get("variant_changed")) or signature != self._last_learning_signature
                    if changed or (now - self._last_learning_log_ts) >= 4.0:
                        self._log(ui_format(
                            self._ui_language,
                            "[Learning] {robot} · {mission} · strategy={variant} · reward={reward:.2f} · observations={observations}",
                            "[Aprendizado] {robot} · {mission} · estratégia={variant} · recompensa={reward:.2f} · observações={observations}",
                            robot=payload.robot, mission=payload.name or "—", variant=variant,
                            reward=float(learning.get("reward", learning.get("mean_reward", 0.0)) or 0.0),
                            observations=int(learning.get("observations", 0) or 0),
                        ))
                        self._last_learning_signature = signature
                        self._last_learning_log_ts = now
        self._sync_language_from_processing()
        try:
            self.runtime.tick()
        except Exception as exc:
            self._log(f"runtime: {exc}")
        self._update_rates()
        self._refresh_visuals()
        self.root.after(self.POLL_MS, self._poll)

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self.voice.stop()
        try:
            self.runtime.stop_active()
        except Exception:
            pass
        self.runtime.close()
        self.bridge.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    SynROVApp(root)
    root.mainloop()


__all__ = ["APP_TITLE", "APP_VERSION", "SynROVApp", "main"]
