/*
 * EEPROMManager
 * 
 * Esta clase se encarga de gestionar la memoria EEPROM de Arduino (WIP)
 */

#include <stdint.h>
#include <OneWire.h>
#include <EEPROM.h>
#include "EEPROMManager.h"

EEPROMManager::EEPROMManager() {}

bool EEPROMManager::SaveToEEPROM(EEPROMDataAddress addr, int8_t value) {
    EEPROM.write(addr, value);

    return true;
}

bool EEPROMManager::SaveToEEPROM(EEPROMDataAddress addr, int16_t value) {
    union u_int16 {
        byte b[2];
        int16_t value;
    } u;

    u.value = value;

    for (uint8_t i = 0; i < 2; ++i) {
        EEPROM.write(addr + i, u.b[i]);
    }

    return true;
}

bool EEPROMManager::SaveToEEPROM(EEPROMDataAddress addr, int32_t value) {
    union u_int32 {
        byte b[4];
        int32_t value;
    } u;

    u.value = value;

    for (uint8_t i = 0; i < 4; ++i) {
        EEPROM.write(addr + i, u.b[i]);
    }

    return true;
}

bool EEPROMManager::SaveToEEPROM(EEPROMDataAddress addr, float value) {
    union u_float {
        byte b[4];
        float value;
    } u;

    u.value = value;

    for (uint8_t i = 0; i < 4; ++i) {
        EEPROM.write(addr + i, u.b[i]);
    }

    return true;
}

int8_t EEPROMManager::LoadInt8FromEEPROM(EEPROMDataAddress addr) {
    return EEPROM.read(addr);
}

int16_t EEPROMManager::LoadInt16FromEEPROM(EEPROMDataAddress addr) {
    union u_int16 {
        byte b[2];
        int16_t value;
    } u;

    for (uint8_t i = 0; i < 2; ++i) {
        EEPROM.read(addr + i);
    }

    return u.value;
}

int32_t EEPROMManager::LoadInt32FromEEPROM(EEPROMDataAddress addr) {
    union u_int32 {
        byte b[4];
        int16_t value;
    } u;

    for (uint8_t i = 0; i < 4; ++i) {
        EEPROM.read(addr + i);
    }

    return u.value;
}

float EEPROMManager::LoadFloatFromEEPROM(EEPROMDataAddress addr) {
    union u_float {
        byte b[4];
        float value;
    } u;

    for (uint8_t i = 0; i < 4; ++i) {
        EEPROM.read(addr + i);
    }

    return u.value;
}
