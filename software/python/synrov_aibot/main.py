"""SynROV AiBot entrypoint.

Default: launches the Tkinter interface.  Pass --modern or set
SYNROV_MODERN=1 to run the dedicated runtime without the GUI.
"""
from __future__ import annotations

import argparse
import os
from typing import Optional, Sequence


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="SynROV AiBot")
    parser.add_argument(
        "--modern",
        action="store_true",
        help="Run the dedicated runtime without the Tkinter interface",
    )
    args, rest = parser.parse_known_args(argv)

    use_modern = args.modern or os.environ.get(
        "SYNROV_MODERN", ""
    ).strip().lower() in {"1", "true", "yes", "sim"}

    if use_modern:
        from .modern_runtime import main as runtime_main
        runtime_main(list(rest))
        return

    from .core import main as gui_main
    gui_main()


if __name__ == "__main__":
    main()
