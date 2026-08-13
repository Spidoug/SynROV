# SynROV Language Packs

SynROV uses external language packages instead of hard-coded UI translations. **English is always the default language**, and Processing is the runtime language authority for the connected system.

## Included languages

The project ships with eight synchronized language packages:

| Code | Language | Native name | Speech locale |
|---|---|---|---|
| `en` | English | English | `en-US` |
| `pt-br` | Portuguese (Brazil) | Português | `pt-BR` |
| `es` | Spanish | Español | `es-ES` |
| `fr` | French | Français | `fr-FR` |
| `de` | German | Deutsch | `de-DE` |
| `zh-cn` | Chinese (Simplified) | 简体中文 | `zh-CN` |
| `ja` | Japanese | 日本語 | `ja-JP` |
| `ar` | Arabic | العربية | `ar-SA` |

Arabic Web content declares `direction: "rtl"`; the other bundled packages use `ltr`.

## Runtime behavior

1. Processing loads every valid `data/languages/*.json` package when the sketch starts.
2. `en` is always the default and is placed first in the Processing language cycle.
3. The language button displays the native name of the active package. Each press selects the next installed package and wraps back to English.
4. The selected package code is stored in the Processing software configuration.
5. Processing publishes the active code in `state.language`, its display name in `state.languageName`, its locale in `state.languageLocale`, and the installed code list in `state.languageAvailable`.
6. AiBot and the HTML console do not make an independent runtime language choice. They resolve the code received from Processing against their own installed packages and fall back to English if that package is unavailable.
7. The standalone HTML console opens in English before Processing synchronization.
8. AiBot uses the active package `locale` for speech recognition, so changing the Processing language also changes the recognition locale without restarting the microphone path.

The protocol accepts normalized BCP-47-like identifiers such as `en`, `pt-br`, `es`, `es-mx`, `de`, or `fr-ca`; language selection is not restricted to the eight bundled packages.

## Package locations

```text
software/processing/SynROV/data/languages/
  en.json
  pt-br.json
  es.json
  fr.json
  de.json
  zh-cn.json
  ja.json
  ar.json

software/python/synrov_aibot/languages/
  en.json
  pt-br.json
  es.json
  fr.json
  de.json
  zh-cn.json
  ja.json
  ar.json

software/web/languages/
  en.js
  pt-br.js
  es.js
  fr.js
  de.js
  zh-cn.js
  ja.js
  ar.js
```

Processing and AiBot enumerate their package directories directly. A browser opened from a local `file://` path cannot reliably enumerate sibling files, so `SynROV.html` explicitly loads the bundled Web package scripts. If another Web language is added, add its `<script src="languages/<code>.js"></script>` entry to `SynROV.html` as well.

## JSON package schema

Processing and AiBot packages use this structure:

```json
{
  "schema": "synrov.language-pack",
  "softwareVersion": 1,
  "code": "es",
  "name": "Spanish",
  "nativeName": "Español",
  "locale": "es-ES",
  "direction": "ltr",
  "strings": {
    "Language: ": "Idioma: "
  }
}
```

Fields:

- `schema`: must be `synrov.language-pack`.
- `softwareVersion`: SynROV version marker; version 1 uses the value `1`.
- `code`: normalized package identifier published by Processing.
- `name`: descriptive English package name.
- `nativeName`: native label displayed by the Processing language button.
- `locale`: speech/locale identifier; AiBot uses it for speech recognition.
- `direction`: `ltr` or `rtl`; the Web console applies it to the document direction.
- `strings`: component-specific translation table keyed by canonical English text/identifier.

English is the canonical source language. Missing translations always fall back to English.

## Web package format

The Web package contains the same metadata and translation dictionary but registers itself as JavaScript so it works when `SynROV.html` is opened directly from disk:

```javascript
window.SYNROV_LANGUAGE_PACKS = window.SYNROV_LANGUAGE_PACKS || {};
window.SYNROV_LANGUAGE_PACKS["es"] = {
  schema: "synrov.language-pack",
  softwareVersion: 1,
  code: "es",
  name: "Spanish",
  nativeName: "Español",
  locale: "es-ES",
  direction: "ltr",
  strings: {
    connection: "Conexión"
  }
};
```


## Consistency guarantees

Each frontend has its own translation key set because its UI is different, but **every installed language inside that frontend must contain exactly the same keys as its English package**. AiBot static widgets are authored from canonical English keys; Web dynamic status messages use `t(...)`; Processing visible status/diagnostic text uses `tr(...)`. This avoids mixed-language screens caused by literal strings bypassing the package loader.

The automated test `software/python/tests/test_language_packages.py` validates:

- exact key parity for all eight AiBot JSON packages;
- exact key parity for all eight Processing JSON packages;
- exact key parity for all eight Web JavaScript packages;
- AiBot static widget labels against the installed translation dictionaries;
- known Web and Processing runtime status paths that must not be hard-coded.

When adding a new UI string, add the English canonical key and translations for every bundled language in that same frontend before committing the change.

## Adding another language

1. Keep the English packages unchanged because English is the canonical fallback.
2. Add a JSON package to `software/processing/SynROV/data/languages/`.
3. Add an AiBot JSON package with the **same `code`** to `software/python/synrov_aibot/languages/`.
4. Add a Web JavaScript package with the **same `code`** to `software/web/languages/` and add its script tag to `SynROV.html`.
5. Preserve the English source keys/placeholders exactly (`%d`, `{name}`, HTML tags, protocol tokens, and similar runtime markers).
6. Restart Processing so it discovers the new package. The package joins the language-button cycle automatically; AiBot and Web follow `state.language` when that code is selected.

Stable robot names where required by the protocol, protocol identifiers, message types, intent identifiers, serial commands, JSON field names, and safety logic must not be translated.

## Unicode fonts and compact UI labels

SynROV does not bundle font files. Processing, AiBot, and the Web console select suitable fonts already installed on the operating system, with language-aware fallbacks for Simplified Chinese, Japanese, Arabic, and Latin-script languages. Processing also builds the active font glyph set from the selected package so translated characters are available to the P3D user interface.

Fixed-size controls use fitted text rather than assuming every translation has the same width as English. The Web console measures localized controls after rendering and reduces the font size only when the translated label would overflow. Processing uses the same principle for compact buttons. Package authors should still prefer concise control labels: use a short, natural UI term or a familiar technical abbreviation when a literal translation is substantially longer than the English control. Long descriptions, help text, diagnostics, and explanatory messages should remain complete rather than being abbreviated merely to match English character count.
