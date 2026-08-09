// =====================================================================
// SynROV Firmware V1 - CRC and EEPROM
// ---------------------------------------------------------------------
// Purpose:
//   Configuration checksums and persistent storage.
// =====================================================================

// Calculates 32.
uint32_t calculateCRC32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1UL) crc = (crc >> 1) ^ 0xEDB88320UL;
      else crc >>= 1;
    }
  }
  return ~crc;
}

// Calculates config CRC.
uint32_t calculateConfigCRC(const PersistentConfig& cfg) {
  return calculateCRC32((const uint8_t*)&cfg, sizeof(PersistentConfig) - sizeof(uint32_t));
}

// Calculates config extras CRC.
uint32_t calculateConfigExtrasCRC(const PersistentConfigExtras& cfg) {
  return calculateCRC32((const uint8_t*)&cfg, sizeof(PersistentConfigExtras) - sizeof(uint32_t));
}

// Calculates safety config CRC.
uint32_t calculateSafetyConfigCRC(const PersistentSafetyConfig& cfg) {
  return calculateCRC32((const uint8_t*)&cfg, sizeof(PersistentSafetyConfig) - sizeof(uint32_t));
}

// Applies the same V1 persistence rule to every magic/schema/CRC block.
// Keeping validation in one place prevents individual EEPROM loaders from
// drifting into slightly different compatibility rules.
template <typename T>
static inline bool persistentBlockValid(const T& cfg, uint32_t expectedMagic, uint16_t expectedSchema, uint32_t calculatedCrc) {
  return cfg.magic == expectedMagic &&
         cfg.schemaId == expectedSchema &&
         calculatedCrc == cfg.crc;
}


