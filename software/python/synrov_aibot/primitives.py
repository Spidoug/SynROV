"""Small dependency-free primitives shared by SynROV runtime modules."""
from __future__ import annotations

import math
import re
import unicodedata
from typing import Any, Iterator, List, Optional


def safe_float(
    value: Any,
    default: float = 0.0,
    lo: Optional[float] = None,
    hi: Optional[float] = None,
) -> float:
    """Return *value* as a finite float, optionally clamped to [lo, hi]."""
    try:
        out = float(value)
    except (TypeError, ValueError, OverflowError):
        try:
            out = float(default if default is not None else 0.0)
        except (TypeError, ValueError, OverflowError):
            out = 0.0

    if not math.isfinite(out):
        try:
            out = float(default if default is not None else 0.0)
        except (TypeError, ValueError, OverflowError):
            out = 0.0
    if not math.isfinite(out):
        out = 0.0

    if lo is not None:
        try:
            low = float(lo)
            if math.isfinite(low):
                out = max(low, out)
        except (TypeError, ValueError, OverflowError):
            pass
    if hi is not None:
        try:
            high = float(hi)
            if math.isfinite(high):
                out = min(high, out)
        except (TypeError, ValueError, OverflowError):
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
    except TypeError:
        return [values]



def iter_or_empty(values: Any) -> Iterator[Any]:
    """Return an iterator, treating non-iterable/None inputs as empty."""
    if values is None:
        return iter(())
    try:
        return iter(values)
    except TypeError:
        return iter(())

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
