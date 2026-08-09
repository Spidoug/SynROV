// =====================================================================
// SynROV Processing - Core JSON utilities (V1)
// ---------------------------------------------------------------------
// Centralized JSON access/copy rules. Optional values use one shared
// fallback policy instead of repeated exception-driven type probing.
// =====================================================================

boolean jsonHasValue(JSONObject obj, String key) {
  return obj != null && key != null && obj.hasKey(key) && !obj.isNull(key);
}

Object getJsonValueSafe(JSONObject obj, String key) {
  return jsonHasValue(obj, key) ? obj.get(key) : null;
}

float getJsonFloat(JSONObject obj, String key, float fallback) {
  if (obj == null || key == null) return fallback;
  return obj.getFloat(key, fallback);
}

int getJsonInt(JSONObject obj, String key, int fallback) {
  if (obj == null || key == null) return fallback;
  return obj.getInt(key, fallback);
}

boolean getJsonBoolean(JSONObject obj, String key, boolean fallback) {
  Object value = getJsonValueSafe(obj, key);
  if (value == null) return fallback;
  if (value instanceof Boolean) return ((Boolean)value).booleanValue();
  if (value instanceof Number) return ((Number)value).doubleValue() != 0.0;

  String normalized = trim(String.valueOf(value)).toLowerCase();
  if (normalized.equals("true") || normalized.equals("1") || normalized.equals("yes") || normalized.equals("on")) return true;
  if (normalized.equals("false") || normalized.equals("0") || normalized.equals("no") || normalized.equals("off")) return false;
  return fallback;
}

String getJsonString(JSONObject obj, String key, String fallback) {
  Object value = getJsonValueSafe(obj, key);
  return value == null ? fallback : String.valueOf(value);
}

JSONObject getJsonObjectSafe(JSONObject parent, String key) {
  Object value = getJsonValueSafe(parent, key);
  return value instanceof JSONObject ? (JSONObject)value : null;
}

JSONArray getJsonArraySafe(JSONObject parent, String key) {
  Object value = getJsonValueSafe(parent, key);
  return value instanceof JSONArray ? (JSONArray)value : null;
}

JSONObject safeParseJsonObject(String text) {
  if (text == null || trim(text).length() == 0) return null;
  try {
    return parseJSONObject(text);
  }
  catch (RuntimeException e) {
    return null;
  }
}

void copyJsonFields(JSONObject target, JSONObject src) {
  if (target == null || src == null) return;
  for (Object keyObj : src.keys()) {
    if (keyObj == null) continue;
    String key = String.valueOf(keyObj);
    if (key.length() == 0 || !src.hasKey(key)) continue;
    target.put(key, src.get(key));
  }
}

JSONObject cloneJsonObjectShallow(JSONObject src) {
  JSONObject out = new JSONObject();
  copyJsonFields(out, src);
  return out;
}

int getSensorJsonInt(JSONObject sens, String key, int fallback) {
  return getJsonInt(sens, key, fallback);
}

String getSensorJsonString(JSONObject sens, String key, String fallback) {
  return getJsonString(sens, key, fallback);
}

float getSensorFloat(String key, float fallback) {
  return getJsonFloat(latestSensors, key, fallback);
}

String getSensorText(String key, String fallback) {
  return getJsonString(latestSensors, key, fallback);
}
