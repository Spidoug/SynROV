
"""Launch SynROV AiBot with the full interface."""
from __future__ import annotations

from synrov_aibot.launcher import run_entrypoint
from synrov_aibot.main import main


if __name__ == "__main__":
    run_entrypoint(main)
