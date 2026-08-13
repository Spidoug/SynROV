"""Language-pack loader for the SynROV AiBot Command Center.

English is the canonical/default UI language.  Additional JSON packages live in
``synrov_aibot/languages`` and are selected only from the language code published
by Processing.
"""
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Dict, Mapping

LANGUAGE_PACK_SCHEMA = "synrov.language-pack"
DEFAULT_LANGUAGE = "en"
_LANGUAGE_DIR = Path(__file__).resolve().with_name("languages")
_CODE_RE = re.compile(r"[^a-z0-9-]+")


def normalize_language_code(value: str) -> str:
    return _CODE_RE.sub("", str(value or "").strip().lower().replace("_", "-"))


def _load_packs() -> Dict[str, dict]:
    packs: Dict[str, dict] = {}
    if _LANGUAGE_DIR.is_dir():
        for path in sorted(_LANGUAGE_DIR.glob("*.json")):
            try:
                pack = json.loads(path.read_text(encoding="utf-8"))
            except Exception:
                continue
            code = normalize_language_code(pack.get("code", ""))
            strings = pack.get("strings")
            if pack.get("schema") != LANGUAGE_PACK_SCHEMA or not code or not isinstance(strings, dict):
                continue
            packs[code] = pack
    if DEFAULT_LANGUAGE not in packs:
        packs[DEFAULT_LANGUAGE] = {
            "schema": LANGUAGE_PACK_SCHEMA,
            "softwareVersion": 1,
            "code": DEFAULT_LANGUAGE,
            "name": "English",
            "nativeName": "English",
            "locale": "en-US",
            "strings": {},
        }
    return packs


LANGUAGE_PACKS: Dict[str, dict] = _load_packs()
AVAILABLE_LANGUAGES = tuple(LANGUAGE_PACKS)


def language_id(value: str) -> str:
    """Resolve a Processing language code against installed AiBot packages."""
    requested = normalize_language_code(value)
    if requested in LANGUAGE_PACKS:
        return requested
    if requested:
        for code in AVAILABLE_LANGUAGES:
            if code.startswith(requested + "-") or requested.startswith(code + "-"):
                return code
    return DEFAULT_LANGUAGE


def language_pack(language: str) -> Mapping[str, object]:
    return LANGUAGE_PACKS.get(language_id(language), LANGUAGE_PACKS[DEFAULT_LANGUAGE])


def language_name(language: str) -> str:
    pack = language_pack(language)
    return str(pack.get("nativeName") or pack.get("name") or language_id(language))


def speech_locale(language: str) -> str:
    pack = language_pack(language)
    return str(pack.get("locale") or language_id(language) or "en-US")


def _canonical_english(text: str) -> str:
    raw = str(text or "")
    english = LANGUAGE_PACKS[DEFAULT_LANGUAGE].get("strings", {})
    if raw in english:
        return raw
    for pack in LANGUAGE_PACKS.values():
        strings = pack.get("strings", {})
        if not isinstance(strings, dict):
            continue
        for source, translated in strings.items():
            if translated == raw:
                return str(source)
            # Slider labels append a numeric range after two spaces.
            prefix = str(translated) + "  ("
            if raw.startswith(prefix):
                return str(source) + raw[len(str(translated)):]
    return raw


def tr_text(text: str, language: str) -> str:
    """Translate an existing widget label through the installed language pack."""
    raw = str(text or "")
    canonical = _canonical_english(raw)
    suffix = ""
    lookup = canonical
    # Preserve slider range suffixes generated at runtime.
    if "  (" in canonical:
        base, tail = canonical.split("  (", 1)
        lookup = base
        suffix = "  (" + tail
    strings = language_pack(language).get("strings", {})
    translated = strings.get(lookup, lookup) if isinstance(strings, dict) else lookup
    return str(translated) + suffix


def format_text(language: str, en_template: str, **values: object) -> str:
    """Translate an English format template and interpolate runtime values."""
    lang = language_id(language)
    strings = language_pack(lang).get("strings", {})
    template = strings.get(str(en_template), str(en_template)) if isinstance(strings, dict) else str(en_template)
    try:
        return str(template).format(**values)
    except (KeyError, ValueError):
        return str(template)


def text(language: str, en: str) -> str:
    """Translate canonical English UI text using the selected language package."""
    lang = language_id(language)
    strings = language_pack(lang).get("strings", {})
    translated = strings.get(str(en), str(en)) if isinstance(strings, dict) else str(en)
    return str(translated)


__all__ = [
    "LANGUAGE_PACK_SCHEMA", "DEFAULT_LANGUAGE", "LANGUAGE_PACKS",
    "AVAILABLE_LANGUAGES", "normalize_language_code",
    "language_id", "language_pack", "language_name", "speech_locale",
    "tr_text", "format_text", "text",
]
