from __future__ import annotations

import ast
import json
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PYTHON_LANG = ROOT / "software" / "python" / "synrov_aibot" / "languages"
PROCESSING = ROOT / "software" / "processing" / "SynROV"
PROCESSING_LANG = PROCESSING / "data" / "languages"
WEB = ROOT / "software" / "web"
WEB_LANG = WEB / "languages"


def load_json_packs(directory: Path) -> dict[str, dict[str, str]]:
    return {
        path.stem: json.loads(path.read_text(encoding="utf-8"))["strings"]
        for path in directory.glob("*.json")
    }


def load_web_packs() -> dict[str, dict[str, str]]:
    packs: dict[str, dict[str, str]] = {}
    pattern = re.compile(r"=\s*(\{.*\})\s*;?\s*$", re.S)
    for path in WEB_LANG.glob("*.js"):
        match = pattern.search(path.read_text(encoding="utf-8"))
        if match is None:
            raise AssertionError(f"Could not parse web language pack: {path.name}")
        packs[path.stem] = json.loads(match.group(1))["strings"]
    return packs


def literal_tr_keys(source: str) -> set[str]:
    """Return Processing tr() arguments made only from Java string literals."""
    keys: set[str] = set()
    start = 0
    string_token = re.compile(r'"(?:\\.|[^"\\])*"')
    literal_expr = re.compile(
        r'^\s*"(?:\\.|[^"\\])*"\s*(?:\+\s*"(?:\\.|[^"\\])*"\s*)*$'
    )
    while True:
        index = source.find("tr(", start)
        if index < 0:
            break
        pos = index + 3
        depth = 1
        in_string = False
        escaped = False
        while pos < len(source) and depth:
            ch = source[pos]
            if in_string:
                if escaped:
                    escaped = False
                elif ch == "\\":
                    escaped = True
                elif ch == '"':
                    in_string = False
            else:
                if ch == '"':
                    in_string = True
                elif ch == "(":
                    depth += 1
                elif ch == ")":
                    depth -= 1
            pos += 1
        if depth == 0:
            expression = source[index + 3 : pos - 1]
            if literal_expr.fullmatch(expression):
                parts = string_token.findall(expression)
                keys.add("".join(json.loads(part) for part in parts))
        start = max(pos, index + 3)
    return keys


class I18nIntegrityTests(unittest.TestCase):
    def assert_same_keys(self, packs: dict[str, dict[str, str]], label: str) -> None:
        self.assertIn("en", packs, f"{label}: English canonical pack is missing")
        canonical = set(packs["en"])
        self.assertGreater(len(canonical), 0, f"{label}: English pack is empty")
        for code, strings in packs.items():
            with self.subTest(surface=label, language=code):
                self.assertEqual(set(strings), canonical)

    def test_01_all_surfaces_keep_language_pack_key_parity(self) -> None:
        self.assert_same_keys(load_json_packs(PYTHON_LANG), "AiBot")
        self.assert_same_keys(load_json_packs(PROCESSING_LANG), "Processing")
        self.assert_same_keys(load_web_packs(), "Web")

    def test_02_aibot_literal_translation_keys_exist(self) -> None:
        source_path = ROOT / "software" / "python" / "synrov_aibot" / "app.py"
        source = source_path.read_text(encoding="utf-8")
        tree = ast.parse(source)
        english = load_json_packs(PYTHON_LANG)["en"]

        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if isinstance(node.func, ast.Name) and node.func.id in {"ui_text", "ui_format"}:
                self.assertGreaterEqual(len(node.args), 2)
                if isinstance(node.args[1], ast.Constant) and isinstance(node.args[1].value, str):
                    with self.subTest(line=node.lineno, key=node.args[1].value):
                        self.assertIn(node.args[1].value, english)
            for keyword in node.keywords:
                if keyword.arg not in {"text", "value", "title"}:
                    continue
                if not isinstance(keyword.value, ast.Constant) or not isinstance(keyword.value.value, str):
                    continue
                value = keyword.value.value
                if any(char.isalpha() for char in value):
                    with self.subTest(line=node.lineno, static_ui=value):
                        self.assertIn(value, english)

    def test_03_processing_literal_tr_keys_exist_in_every_pack(self) -> None:
        used: set[str] = set()
        for path in PROCESSING.glob("*.pde"):
            used.update(literal_tr_keys(path.read_text(encoding="utf-8")))
        packs = load_json_packs(PROCESSING_LANG)
        self.assertGreater(len(used), 400)
        for code, strings in packs.items():
            with self.subTest(language=code):
                self.assertFalse(sorted(used - set(strings)))

    def test_04_processing_operator_messages_do_not_bypass_translation(self) -> None:
        allowed = {'updateMessage("");'}
        for path in PROCESSING.glob("*.pde"):
            for line_no, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                line = raw.strip()
                if "updateMessage(" not in line or '"' not in line or "tr(" in line:
                    continue
                if line in allowed:
                    continue
                with self.subTest(file=path.name, line=line_no):
                    self.fail(f"operator message bypasses tr(): {line}")

    def test_05_web_t_keys_exist_and_accessibility_is_localized(self) -> None:
        source = (WEB / "SynROV.html").read_text(encoding="utf-8")
        used = set(re.findall(r"\bt\(\s*['\"]([^'\"]+)['\"]", source))
        packs = load_web_packs()
        for code, strings in packs.items():
            with self.subTest(language=code):
                self.assertFalse(sorted(used - set(strings)))
        for key in ("web_console_title", "yaw_left_title", "yaw_right_title", "live_view_alt"):
            self.assertIn(f"t('{key}')", source)
        for key in ("camera_up_aria", "camera_left_aria", "camera_right_aria", "camera_down_aria"):
            self.assertIn(f"'{key}'", source)
        self.assertIn("node.setAttribute('aria-label', t(key))", source)

    def test_06_aibot_robot_display_is_decoupled_from_protocol_name(self) -> None:
        source = (ROOT / "software" / "python" / "synrov_aibot" / "app.py").read_text(encoding="utf-8")
        self.assertIn("self.robot_display_var", source)
        self.assertIn("self._robot_label_to_name", source)
        self.assertIn("def _refresh_robot_combo_labels(self)", source)
        self.assertNotIn("textvariable=self.robot_var, values=ROBOTS", source)


if __name__ == "__main__":
    unittest.main()
