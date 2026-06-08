"""Auto Torque bridge and Tkinter integration for SynROV AiBot."""
from __future__ import annotations

from typing import Any, MutableMapping


def install_auto_torque_integration(namespace: MutableMapping[str, Any]) -> None:
    """Attach Auto Torque controls to the active bridge and UI classes."""
    tk = namespace.get("tk")
    ttk = namespace.get("ttk")
    time_mod = namespace.get("time")
    messagebox = namespace.get("messagebox")

    def bridge_set_auto_torque(self: Any, enabled: bool) -> str:
        enabled_bool = bool(enabled)
        setattr(self, "_auto_torque_enabled", enabled_bool)
        self.send_control({
            "control": {
                "robot": "Manipulator",
                "manipulator": {
                    "autoTorque": enabled_bool,
                    "auto_torque": enabled_bool,
                },
            },
            "action": "autoTorque",
            "enabled": enabled_bool,
        })
        return "CONTROL:Manipulator:autoTorque=" + ("ON" if enabled_bool else "OFF")

    for bridge_cls_name in ("SynROVBridge", "EnhancedSynROVBridge"):
        bridge_cls = namespace.get(bridge_cls_name)
        if bridge_cls is not None and not hasattr(bridge_cls, "set_auto_torque"):
            setattr(bridge_cls, "set_auto_torque", bridge_set_auto_torque)

    app_cls = (
        namespace.get("SynROV")
        or namespace.get("CompactResponsiveAutoLearnStudio")
        or namespace.get("CompactMultimodalAutoLearnStudio")
    )
    if app_cls is None or tk is None or ttk is None:
        return

    def ensure_auto_torque_var(self: Any) -> None:
        if hasattr(self, "auto_torque_var"):
            return
        try:
            self.auto_torque_var = tk.BooleanVar(master=self.root, value=False)
        except Exception:
            self.auto_torque_var = tk.BooleanVar(value=False)

    def toggle_auto_torque(self: Any) -> None:
        ensure_auto_torque_var(self)
        enabled = bool(self.auto_torque_var.get())
        try:
            command = self.bridge.set_auto_torque(enabled)
            self.last_command_source = "auto_torque"
            if time_mod is not None:
                self.last_command_ts = time_mod.time()
            try:
                self.log_i18n(
                    f"[auto torque] {'ligado' if enabled else 'desligado'} | {command}",
                    f"[auto torque] {'enabled' if enabled else 'disabled'} | {command}",
                )
            except Exception:
                self.log(f"[auto torque] {'ON' if enabled else 'OFF'} | {command}")
        except Exception as exc:
            try:
                if messagebox is not None:
                    messagebox.showerror("Auto Torque", str(exc))
            except Exception:
                pass
            try:
                self.log_i18n(f"[auto torque] erro: {exc}", f"[auto torque] error: {exc}")
            except Exception:
                pass

    def auto_torque_label(self: Any) -> str:
        try:
            return self._ui_text("Torque automático", "Auto Torque")
        except Exception:
            return "Auto Torque"

    def inject_auto_torque_widget(self: Any) -> None:
        ensure_auto_torque_var(self)
        if hasattr(self, "chk_auto_torque"):
            try:
                self.chk_auto_torque.configure(text=auto_torque_label(self))
            except Exception:
                pass
            return

        parent = None
        for attr_name in ("btn_home_arm", "btn_home_arm2"):
            widget = getattr(self, attr_name, None)
            try:
                if widget is not None and widget.winfo_exists():
                    parent = widget.master
                    break
            except Exception:
                pass
        if parent is None:
            for attr_name in ("manip_frame", "manual_container", "tab_control", "tab_multi"):
                widget = getattr(self, attr_name, None)
                try:
                    if widget is not None and widget.winfo_exists():
                        parent = widget
                        break
                except Exception:
                    pass
        if parent is None:
            return

        try:
            self.chk_auto_torque = ttk.Checkbutton(
                parent,
                text=auto_torque_label(self),
                variable=self.auto_torque_var,
                command=self.toggle_auto_torque,
            )
            try:
                self.chk_auto_torque.pack(side="left", padx=6)
            except Exception:
                self.chk_auto_torque.grid(row=99, column=0, sticky="w", padx=6, pady=2)
        except Exception:
            pass

    if not hasattr(app_cls, "toggle_auto_torque"):
        setattr(app_cls, "toggle_auto_torque", toggle_auto_torque)
    if not hasattr(app_cls, "_ensure_auto_torque_var"):
        setattr(app_cls, "_ensure_auto_torque_var", ensure_auto_torque_var)
    if not hasattr(app_cls, "_inject_auto_torque_widget"):
        setattr(app_cls, "_inject_auto_torque_widget", inject_auto_torque_widget)

    build_control = getattr(app_cls, "_build_control_tab_compact", None)
    if callable(build_control) and not getattr(build_control, "_synrov_auto_torque_ready", False):
        def wrapped_build_control(self: Any, *args: Any, **kwargs: Any) -> Any:
            result = build_control(self, *args, **kwargs)
            try:
                self._inject_auto_torque_widget()
            except Exception:
                pass
            return result
        wrapped_build_control._synrov_auto_torque_ready = True  # type: ignore[attr-defined]
        setattr(app_cls, "_build_control_tab_compact", wrapped_build_control)

    apply_language = getattr(app_cls, "_apply_language", None)
    if callable(apply_language) and not getattr(apply_language, "_synrov_auto_torque_ready", False):
        def wrapped_apply_language(self: Any, *args: Any, **kwargs: Any) -> Any:
            result = apply_language(self, *args, **kwargs)
            try:
                if hasattr(self, "chk_auto_torque"):
                    self.chk_auto_torque.configure(text=auto_torque_label(self))
            except Exception:
                pass
            return result
        wrapped_apply_language._synrov_auto_torque_ready = True  # type: ignore[attr-defined]
        setattr(app_cls, "_apply_language", wrapped_apply_language)
