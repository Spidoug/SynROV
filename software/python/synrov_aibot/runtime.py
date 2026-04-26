"""Runtime compatibility layer for SynROV AiBot.

Import this module when you need the same behavior as the original single-file
application. More focused modules re-export groups from this runtime.
"""

from ._legacy import *  # noqa: F401,F403
