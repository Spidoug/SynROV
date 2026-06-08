"""Shared launch helpers for the SynROV AiBot application."""
from __future__ import annotations

import os
import traceback
from typing import Callable


def show_startup_error(title: str = "SynROV AiBot") -> None:
    """Print and display the latest startup traceback when a GUI is available."""
    tb = traceback.format_exc()
    print("\nSYNROV_AIBOT STARTUP ERROR:\n")
    print(tb)
    try:
        import tkinter as tk
        from tkinter import messagebox

        root = tk.Tk()
        root.withdraw()
        messagebox.showerror(f"{title} - startup error", tb[-3500:])
        root.destroy()
    except Exception:
        pass
    if os.name == "nt":
        try:
            input("\nPress ENTER to close...")
        except Exception:
            pass


def run_entrypoint(entrypoint: Callable[[], None]) -> None:
    """Run an entrypoint with the common SynROV startup error handler."""
    try:
        entrypoint()
    except SystemExit:
        raise
    except Exception:
        show_startup_error()
        raise SystemExit(1)
