"""Small dependency-free primitives shared by SynROV runtime modules."""
from __future__ import annotations

import math
import re
import unicodedata
from typing import Any, List, Optional


def safe_float(
    value: Any,
    default: float = 0.0,
    lo: Optional[float] = None,
    hi: Optional[float] = None,
) -> float:
    """Return *value* as a finite float, optionally clamped to [lo, hi]."""
    try:
        out = float(value)
    except Exception:
        try:
            out = float(default or 0.0)
        except Exception:
            out = 0.0

    if not math.isfinite(out):
        try:
            out = float(default or 0.0)
        except Exception:
            out = 0.0
    if not math.isfinite(out):
        out = 0.0

    if lo is not None:
        try:
            low = float(lo)
            if math.isfinite(low):
                out = max(low, out)
        except Exception:
            pass
    if hi is not None:
        try:
            high = float(hi)
            if math.isfinite(high):
                out = min(high, out)
        except Exception:
            pass
    return out


def safe_int(value: Any, default: int = 0) -> int:
    """Return *value* as a rounded integer using safe_float's fallback rules."""
    return int(round(safe_float(value, float(default))))


def to_list(values: Any) -> List[Any]:
    """Return *values* as a list without truth-testing array-like objects."""
    if values is None:
        return []
    if isinstance(values, list):
        return values
    if isinstance(values, tuple):
        return list(values)
    if isinstance(values, (str, bytes, bytearray)):
        return [values]
    try:
        return list(values)
    except Exception:
        return [values]


def normalize_text(value: Any) -> str:
    """Lowercase, remove accents/punctuation and collapse whitespace."""
    text = unicodedata.normalize("NFKD", str(value or "").lower())
    text = "".join(ch for ch in text if not unicodedata.combining(ch))
    text = re.sub(r"[^a-z0-9\s]", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def normalize_identifier(value: Any) -> str:
    """Lowercase and de-accent identifiers while preserving separators."""
    text = unicodedata.normalize("NFKD", str(value or "").strip().lower())
    return "".join(ch for ch in text if not unicodedata.combining(ch)).replace("ç", "c")


def json_compatible(value: Any) -> Any:
    import json
    from pathlib import Path

    try:
        if hasattr(value, "item"):
            return json_compatible(value.item())
    except Exception:
        pass
    try:
        if hasattr(value, "tolist"):
            return json_compatible(value.tolist())
    except Exception:
        pass
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): json_compatible(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [json_compatible(v) for v in value]
    try:
        json.dumps(value)
        return value
    except Exception:
        return str(value)
