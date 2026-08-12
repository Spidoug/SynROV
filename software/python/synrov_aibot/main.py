"""SynROV AiBot Version 1 entry point."""
from __future__ import annotations

import argparse
import sys
from typing import Optional, Sequence


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description="SynROV AiBot — Versão 1")
    parser.add_argument("--headless", action="store_true", help="Run without the Tkinter interface")
    known, remaining = parser.parse_known_args(args)
    if known.headless:
        from .runtime import main as runtime_main
        runtime_main(remaining)
    else:
        if remaining:
            parser.error("argumentos extras só são aceitos com --headless")
        from .app import main as app_main
        app_main()


if __name__ == "__main__":
    main()
