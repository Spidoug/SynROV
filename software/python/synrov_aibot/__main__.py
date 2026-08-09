"""Package entry point: python -m synrov_aibot."""
from __future__ import annotations

from .launcher import run_entrypoint
from .main import main


if __name__ == "__main__":
    run_entrypoint(main)
