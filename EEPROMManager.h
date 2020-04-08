/*
 * EEPROMManager
 * 
 * Esta clase se encarga de gestionar la memoria EEPROM de Arduino (WIP)
 */

#ifndef __EEPROM_MANAGER__H__
#define __EEPROM_MANAGER__H__

enum EEPROMDataAddress {
    // Primeros 256 bytes reservados para integers ...
    ADDR_INTERVAL_BETWEEN_PACKETS      = 0,   // int16
    ADDR_BAUD_RATE                     = 4,   // int32
    ADDR_INTAKE_RPM_SWITCHOVER_NORMAL  = 8,   // int16
    ADDR_EXHAUST_RPM_SWITCHOVER_NORMAL = 12,   // int16
    ADDR_INTAKE_RPM_SWITCHOVER_RACE    = 16,  // int16
    ADDR_EXHAUST_RPM_SWITCHOVER_RACE   = 20,  // int16
    ADDR_RPM_LIMITER_HYSTERESIS        = 24,  // int16
    ADDR_MAX_RPM_DIFF_BETWEEN_CHECKS   = 28,  // int16
    ADDR_RPM_INPUT_CHECK_INTERVAL      = 32,  // int16
    ADDR_RPM_FAILURES_RESET_TIMER      = 36,  // int32
    ADDR_MAX_RPM_SIGNAL_ERRORS         = 40,  // int8
    ADDR_EMERGENCY_REV_LIMITER         = 44,  // int16
    ADDR_CAMS_SWITCHOVER_COOLDOWN      = 48,  // int16
    // ... siguientes 256 para floats.
    ADDR_ENGINE_OIL_COLD_TEMP_LIMIT    = 256,
    ADDR_ENGINE_OIL_TEMP_WARNING       = 260,
    ADDR_ENGINE_OIL_TEMP_DANGER        = 264,
    ADDR_GEARBOX_OIL_COLD_TEMP_LIMIT   = 268,
    ADDR_GEARBOX_OIL_TEMP_WARNING      = 272,
    ADDR_GEARBOX_OIL_TEMP_DANGER       = 276,
    ADDR_AFR_RICH_WARNING              = 280,
    ADDR_AFR_RICH_DANGER               = 284,
    ADDR_AFR_LEAN_WARNING              = 288,
    ADDR_AFR_LEAN_DANGER               = 292,
    ADDR_VOLTAGE_LOW_WARNING           = 296,
    ADDR_VOLTAGE_LOW_DANGER            = 300,
    ADDR_VOLTAGE_HIGH_WARNING          = 304,
    ADDR_VOLTAGE_HIGH_DANGER           = 308,
    ADDR_ENGINE_OIL_PRESS_MIN          = 312,
    ADDR_ENGINE_OIL_PRESS_MIN_HOT      = 316,
    ADDR_ENGINE_OIL_PRESS_MIN_RPMS     = 320
};

class EEPROMManager {
  public:
    EEPROMManager();

    // Integers
    bool SaveToEEPROM(EEPROMDataAddress addr, int8_t value);
    bool SaveToEEPROM(EEPROMDataAddress addr, int16_t value);
    bool SaveToEEPROM(EEPROMDataAddress addr, int32_t value);
    // Float
    bool SaveToEEPROM(EEPROMDataAddress addr, float value);

    // Para recuperar los datos
    int8_t LoadInt8FromEEPROM(EEPROMDataAddress addr);
    int16_t LoadInt16FromEEPROM(EEPROMDataAddress addr);
    int32_t LoadInt32FromEEPROM(EEPROMDataAddress addr);
    float LoadFloatFromEEPROM(EEPROMDataAddress addr);
};

#endif