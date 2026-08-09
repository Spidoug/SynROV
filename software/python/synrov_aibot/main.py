"""SynROV AiBot entry point.

Default: launches the Tkinter interface. Pass --modern or set
SYNROV_MODERN=1 to run the dedicated runtime without the GUI.
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import Optional, Sequence

_TRUE_VALUES = {"1", "true", "yes", "sim"}


def _env_enabled(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in _TRUE_VALUES


def main(argv: Optional[Sequence[str]] = None) -> None:
    raw_args = list(sys.argv[1:] if argv is None else argv)

    if "--modern" in raw_args or _env_enabled("SYNROV_MODERN"):
        from .modern_runtime import main as runtime_main

        runtime_args = [arg for arg in raw_args if arg != "--modern"]
        runtime_main(runtime_args)
        return

    parser = argparse.ArgumentParser(description="SynROV AiBot")
    parser.add_argument(
        "--modern",
        action="store_true",
        help="Run the dedicated runtime without the Tkinter interface",
    )
    parser.parse_args(raw_args)

    from .core import main as gui_main

    gui_main()


if __name__ == "__main__":
    main()
