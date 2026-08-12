// =====================================================================
// SynROV Processing - Language packs
// ---------------------------------------------------------------------
// English is always the default language. Every valid package discovered in
// data/languages is selectable from the Processing language button. Processing
// remains the language authority for Web and AiBot clients.
// =====================================================================

final String LANGUAGE_PACK_SCHEMA = "synrov.language-pack";
final String DEFAULT_UI_LANGUAGE_CODE = "en";

HashMap<String, JSONObject> uiLanguagePacks = new HashMap<String, JSONObject>();
ArrayList<String> uiLanguageCodes = new ArrayList<String>();
String uiLanguageCode = DEFAULT_UI_LANGUAGE_CODE;

// Unicode-aware UI font selected from fonts already installed on the host OS.
// No font file is bundled with SynROV; each language picks the best available
// system family and falls back to Java SansSerif.
PFont synUiFont = null;
String synUiFontLanguageCode = "";
String[] synInstalledFontNames = null;

boolean synHasInstalledFont(String family) {
  if (family == null || family.length() == 0) return false;
  if (synInstalledFontNames == null) {
    try {
      synInstalledFontNames = PFont.list();
      if (synInstalledFontNames == null) synInstalledFontNames = new String[0];
    }
    catch (Exception ex) {
      synInstalledFontNames = new String[0];
    }
  }
  for (String installed : synInstalledFontNames) {
    if (installed != null && installed.equalsIgnoreCase(family)) return true;
  }
  return false;
}

String synPreferredUiFontFamily(String languageCode) {
  String code = normalizeUiLanguageCode(languageCode);
  String[][] candidates;
  if (code.startsWith("zh")) {
    candidates = new String[][] {{"Microsoft YaHei UI"}, {"Microsoft YaHei"}, {"PingFang SC"}, {"Noto Sans CJK SC"}, {"Noto Sans SC"}, {"WenQuanYi Zen Hei"}};
  } else if (code.startsWith("ja")) {
    candidates = new String[][] {{"Yu Gothic UI"}, {"Yu Gothic"}, {"Meiryo UI"}, {"Meiryo"}, {"Hiragino Sans"}, {"Noto Sans CJK JP"}, {"Noto Sans JP"}};
  } else if (code.startsWith("ar")) {
    candidates = new String[][] {{"Segoe UI"}, {"Tahoma"}, {"Arial"}, {"Noto Sans Arabic"}, {"DejaVu Sans"}};
  } else {
    candidates = new String[][] {{"Segoe UI"}, {"Arial"}, {"Helvetica Neue"}, {"DejaVu Sans"}, {"Noto Sans"}};
  }
  for (String[] item : candidates) {
    if (item.length > 0 && synHasInstalledFont(item[0])) return item[0];
  }
  return "SansSerif";
}

char[] synActiveLanguageGlyphSet() {
  LinkedHashSet<Character> glyphs = new LinkedHashSet<Character>();
  for (char c = 32; c < 127; c++) glyphs.add(c);
  StringBuilder source = new StringBuilder();
  JSONObject pack = activeUiLanguagePack();
  if (pack != null) {
    source.append(getJsonString(pack, "nativeName", ""));
    JSONObject strings = getJsonObjectSafe(pack, "strings");
    if (strings != null) source.append(strings.toString());
  }
  // Common UI glyphs used outside the packages.
  source.append("°…→←↑↓±×·—–✓✕");
  for (int i = 0; i < source.length(); i++) glyphs.add(source.charAt(i));
  char[] result = new char[glyphs.size()];
  int idx = 0;
  for (Character c : glyphs) result[idx++] = c.charValue();
  return result;
}

void ensureSynUiFontForLanguage() {
  String code = activeUiLanguageCode();
  if (synUiFont != null && code.equals(synUiFontLanguageCode)) return;
  String family = synPreferredUiFontFamily(code);
  char[] glyphs = synActiveLanguageGlyphSet();
  try {
    synUiFont = createFont(family, 14, true, glyphs);
  }
  catch (Exception ex) {
    try {
      synUiFont = createFont("SansSerif", 14, true, glyphs);
    }
    catch (Exception fallbackEx) {
      synUiFont = createFont("SansSerif", 14, true);
    }
  }
  synUiFontLanguageCode = code;
  if (synUiFont != null) textFont(synUiFont);
  diagnosticsFont = synUiFont;
}

void applySynUiFont() {
  ensureSynUiFontForLanguage();
  if (synUiFont != null) textFont(synUiFont);
}

String normalizeUiLanguageCode(String value) {
  String code = value == null ? "" : trim(value).toLowerCase().replace('_', '-');
  StringBuilder safe = new StringBuilder();
  for (int i = 0; i < code.length(); i++) {
    char c = code.charAt(i);
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-') safe.append(c);
  }
  return safe.toString();
}

String resolveInstalledUiLanguageCode(String requested) {
  String code = normalizeUiLanguageCode(requested);
  if (code.length() == 0) return DEFAULT_UI_LANGUAGE_CODE;
  if (uiLanguagePacks != null && uiLanguagePacks.containsKey(code)) return code;
  if (uiLanguageCodes != null) {
    for (String installed : uiLanguageCodes) {
      if (installed.equals(code)) return installed;
      if (installed.startsWith(code + "-") || code.startsWith(installed + "-")) return installed;
    }
  }
  return DEFAULT_UI_LANGUAGE_CODE;
}

void loadUiLanguagePackages() {
  if (uiLanguagePacks == null) uiLanguagePacks = new HashMap<String, JSONObject>();
  if (uiLanguageCodes == null) uiLanguageCodes = new ArrayList<String>();
  uiLanguagePacks.clear();
  uiLanguageCodes.clear();

  File languageDir = new File(dataPath("languages"));
  if (!languageDir.exists()) languageDir.mkdirs();
  String[] names = languageDir.list();
  if (names != null) {
    names = sort(names);
    for (String filename : names) {
      if (filename == null || !filename.toLowerCase().endsWith(".json")) continue;
      try {
        JSONObject pack = loadJSONObject(new File(languageDir, filename).getAbsolutePath());
        if (pack == null) continue;
        if (!LANGUAGE_PACK_SCHEMA.equals(getJsonString(pack, "schema", ""))) continue;
        String code = normalizeUiLanguageCode(getJsonString(pack, "code", ""));
        JSONObject strings = getJsonObjectSafe(pack, "strings");
        if (code.length() == 0 || strings == null) continue;
        uiLanguagePacks.put(code, pack);
        if (!uiLanguageCodes.contains(code)) uiLanguageCodes.add(code);
      }
      catch (Exception ex) {
        println("[SynROV][Language] Could not load " + filename + ": " + ex.getMessage());
      }
    }
  }

  // English remains a valid built-in fallback even if its file is missing and
  // is always presented first, regardless of filename sorting.
  uiLanguageCodes.remove(DEFAULT_UI_LANGUAGE_CODE);
  uiLanguageCodes.add(0, DEFAULT_UI_LANGUAGE_CODE);
  uiLanguageCode = resolveInstalledUiLanguageCode(uiLanguageCode);
  println("[SynROV][Language] Active=" + uiLanguageCode + " installed=" + installedUiLanguageCodesCsv());
}

JSONObject activeUiLanguagePack() {
  if (uiLanguagePacks == null) return null;
  return uiLanguagePacks.get(resolveInstalledUiLanguageCode(uiLanguageCode));
}

String activeUiLanguageCode() {
  String code = normalizeUiLanguageCode(uiLanguageCode);
  return code.length() == 0 ? DEFAULT_UI_LANGUAGE_CODE : code;
}

String uiLanguageNativeName(String code) {
  String resolved = resolveInstalledUiLanguageCode(code);
  JSONObject pack = uiLanguagePacks == null ? null : uiLanguagePacks.get(resolved);
  String fallback = DEFAULT_UI_LANGUAGE_CODE.equals(resolved) ? "English" : resolved;
  return getJsonString(pack, "nativeName", getJsonString(pack, "name", fallback));
}

String activeUiLanguageNativeName() {
  return uiLanguageNativeName(activeUiLanguageCode());
}

String activeUiLanguageLocale() {
  JSONObject pack = activeUiLanguagePack();
  return getJsonString(pack, "locale", DEFAULT_UI_LANGUAGE_CODE.equals(activeUiLanguageCode()) ? "en-US" : activeUiLanguageCode());
}

String installedUiLanguageCodesCsv() {
  if (uiLanguageCodes == null || uiLanguageCodes.size() == 0) return DEFAULT_UI_LANGUAGE_CODE;
  String[] codes = uiLanguageCodes.toArray(new String[uiLanguageCodes.size()]);
  return join(codes, ",");
}

String translateFromUiPack(String englishText) {
  if (englishText == null) return "";
  String code = activeUiLanguageCode();
  if (DEFAULT_UI_LANGUAGE_CODE.equals(code)) return englishText;
  JSONObject pack = activeUiLanguagePack();
  JSONObject strings = getJsonObjectSafe(pack, "strings");
  if (strings == null) return englishText;
  return getJsonString(strings, englishText, englishText);
}

// English is the canonical UI key and fallback for every installed language pack.
String tr(String englishText) {
  return translateFromUiPack(englishText);
}

String languageToggleButtonLabel() {
  return activeUiLanguageNativeName();
}

int activeUiLanguageIndex() {
  String active = activeUiLanguageCode();
  for (int i = 0; i < uiLanguageCodes.size(); i++) {
    if (active.equals(uiLanguageCodes.get(i))) return i;
  }
  return 0;
}

String nextInstalledUiLanguageCode() {
  if (uiLanguageCodes == null || uiLanguageCodes.size() <= 1) return DEFAULT_UI_LANGUAGE_CODE;
  int nextIndex = (activeUiLanguageIndex() + 1) % uiLanguageCodes.size();
  return uiLanguageCodes.get(nextIndex);
}

void setUiLanguage(String requested) {
  String next = resolveInstalledUiLanguageCode(requested);
  if (next.equals(activeUiLanguageCode())) return;
  uiLanguageCode = next;
  synUiFontLanguageCode = "";
  synUiFont = null;
  diagnosticsFont = null;
  saveSoftwareConfigNow();
  refreshCalibrationMonitorControls();
  refreshJoystickWindowsForLanguageChange();
  updateMessage(tr("Language: ") + activeUiLanguageNativeName());
  sendSystemStatus();
}

void toggleUiLanguage() {
  if (uiLanguageCodes == null || uiLanguageCodes.size() <= 1) {
    updateMessage(tr("Language: ") + activeUiLanguageNativeName());
    return;
  }
  setUiLanguage(nextInstalledUiLanguageCode());
}
