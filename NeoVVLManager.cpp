/*
 * NeoVVLManager
 * 
 * Esta clase tiene como objetivo gestionar los solenoides de activación del sistema NeoVVL.
 * Controla de forma independiente las levas de escape y admisión.
 */

#include <stdint.h>
#include <OneWire.h>
#include "DataManager.h"
#include "DataMonitor.h"
#include "AuxManager.h"
#include "EEPROMManager.h"
#include "NeoVVLManager.h"

NeoVVLManager::NeoVVLManager() {
    _isIntakeEnabled = false;
    _isExhaustEnabled = false;
    _intakeCamCooldownTimer = 0;
    _exhaustCamCooldownTimer = 0;
    _dataManager = NULL;
    _auxManager = NULL;

    // Inicializamos los pines y solenoides
    pinMode(OUTPUT_INTAKE_SOLENOID, OUTPUT);
    pinMode(OUTPUT_EXHAUST_SOLENOID, OUTPUT);
    SwitchIntakeCam(false);
    SwitchExhaustCam(false);

    // Cargamos las variables desde la EEPROM si las hay
    LoadCamsSwitchPointsFromEEPROM();
}

void NeoVVLManager::Initialize(DataManager *dataManager, AuxManager *auxManager, EEPROMManager *eepromManager) {
    _dataManager = dataManager;
    _auxManager = auxManager;
    _eepromManager = eepromManager;
}

void NeoVVLManager::Update(uint32_t diff) {
    // Aquí no hay timers, las levas son controladas constantemente, es uno de los puntos más relevantes
    // De todas formas pasamos el parámetro diff por si se quiere utilizar en un futuro
    if (!_dataManager)
        return;

    // Llamamos a la función auxiliar para cálculo de las RPM
    _dataManager->RetrieveRPM(micros());

    // Comprobamos que hacer en función de los mapas activos en la ECU.
    ECUMaps currentMap = _auxManager->GetCurrentECUMap();

    switch (currentMap) {
        case ECU_MAP_NORMAL:
            // Comprobar que el motor esté encendido. Obviamente no podemos jugar con las levas con el motor apagado.
            // Modo normal, operación estándar de las levas.
            if (_dataManager->IsEngineOn()) {
                uint16_t rpm = _dataManager->GetRPM();

                if (rpm >= INTAKE_RPM_SWITCHOVER_NORMAL && !_isIntakeEnabled) {
                    SwitchIntakeCam(true);
                } else if (rpm < INTAKE_RPM_SWITCHOVER_NORMAL && _isIntakeEnabled) {
                    SwitchIntakeCam(false);
                }

                if (rpm >= EXHAUST_RPM_SWITCHOVER_NORMAL && !_isExhaustEnabled) {
                    SwitchExhaustCam(true);
                } else if (rpm < EXHAUST_RPM_SWITCHOVER_NORMAL && _isExhaustEnabled) {
                    SwitchExhaustCam(false);
                }
            }
            break;
        case ECU_MAP_RACE:
            // Modo carrera, igual al modo normal pero cambian los puntos de activación de las levas.
            if (_dataManager->IsEngineOn()) {
                uint16_t rpm = _dataManager->GetRPM();

                if (rpm >= INTAKE_RPM_SWITCHOVER_RACE && !_isIntakeEnabled) {
                    SwitchIntakeCam(true);
                } else if (rpm < INTAKE_RPM_SWITCHOVER_RACE && _isIntakeEnabled) {
                    SwitchIntakeCam(false);
                }

                if (rpm >= EXHAUST_RPM_SWITCHOVER_RACE && !_isExhaustEnabled) {
                    SwitchExhaustCam(true);
                } else if (rpm < EXHAUST_RPM_SWITCHOVER_RACE && _isExhaustEnabled) {
                    SwitchExhaustCam(false);
                }
            }
            break;
        case ECU_MAP_EMERGENCY:
            // En el modo emergencia, las levas quedan activadas permanentemente en ALTAS.
            // Esto es debido a que si las levas de bajas entran a altas vueltas, el tren de válvulas puede sufrir daños.
            SwitchIntakeCam(true);
            SwitchExhaustCam(true);
            break;
        default:
            break;
    }

    if (_isIntakeInCooldown) {
        if (_intakeCamCooldownTimer >= CAMS_SWITCHOVER_COOLDOWN) {
            _isIntakeInCooldown = false;
            _intakeCamCooldownTimer = 0;
        } else {
            _intakeCamCooldownTimer += diff;
        }
    }

    if (_isExhaustInCooldown) {
        if (_exhaustCamCooldownTimer >= CAMS_SWITCHOVER_COOLDOWN) {
            _isExhaustInCooldown = false;
            _exhaustCamCooldownTimer = 0;
        } else {
            _exhaustCamCooldownTimer += diff;
        }
    }
}

/****************************************************************************************************************************
 * Respecto a las funciones para cambiar entre las diferentes levas:                                                        *
 *                                                                                                                          *
 * Ojo porque la lógica está "doblemente invertida", poniendo el pin a 0v activamos el relé, y a 5v lo desactivamos.        *
 * A su vez, por defecto, son las levas de altas las que están siempre activadas, para evitar que en caso de algún fallo    *
 * en el sistema, se activen las levas de bajas a altas vueltas y se dañe el tren de vávulas. Es decir, hay que activar el  *
 * relé para activar las levas de bajas y desactivarlo para activar las levas de altas.                                     *
 *                                                                                                                          *
 * En los comentarios de ecu_software.ino se explica algo mejor el motivo.                                                  *
 ****************************************************************************************************************************/
void NeoVVLManager::SwitchIntakeCam(bool on) {
    if (_isIntakeInCooldown)
        return;

    if (on) {
        //Desactivamos el relé
        digitalWrite(OUTPUT_INTAKE_SOLENOID, HIGH);
        _isIntakeEnabled = true;
    } else {
        // Activamos el relé
        digitalWrite(OUTPUT_INTAKE_SOLENOID, LOW);
        _isIntakeEnabled = false;
    }
    _isIntakeInCooldown = true;
}

void NeoVVLManager::SwitchExhaustCam(bool on) {
    if (_isExhaustInCooldown)
        return;

    if (on) {
        //Desactivamos el relé
        digitalWrite(OUTPUT_EXHAUST_SOLENOID, HIGH);
        _isExhaustEnabled = true;
    } else {
        // Activamos el relé
        digitalWrite(OUTPUT_EXHAUST_SOLENOID, LOW);
        _isExhaustEnabled = false;
    }
    _isExhaustInCooldown = true;
}

int8_t NeoVVLManager::GetIntakeCamStatus() {
    if (_isIntakeEnabled) {
        return CAM_STATUS_ENABLED;
    }

    return CAM_STATUS_DISABLED;
}

int8_t NeoVVLManager::GetExhaustCamStatus() {
    if (_isExhaustEnabled) {
        return CAM_STATUS_ENABLED;
    }

    return CAM_STATUS_DISABLED;
}

void NeoVVLManager::LoadCamsSwitchPointsFromEEPROM() {
    _intakeSwitchNormal = _eepromManager->LoadInt16FromEEPROM(ADDR_INTAKE_RPM_SWITCHOVER_NORMAL);
    _exhaustSwitchNormal = _eepromManager->LoadInt16FromEEPROM(ADDR_EXHAUST_RPM_SWITCHOVER_NORMAL);
    _intakeSwitchRace = _eepromManager->LoadInt16FromEEPROM(ADDR_INTAKE_RPM_SWITCHOVER_RACE);
    _exhaustSwitchRace = _eepromManager->LoadInt16FromEEPROM(ADDR_EXHAUST_RPM_SWITCHOVER_RACE);

    // Veirificamos que los valores son correctos
    if (_intakeSwitchNormal > 7000 || _intakeSwitchNormal < 2000) {
        _intakeSwitchNormal = INTAKE_RPM_SWITCHOVER_NORMAL;
    }

    if (_exhaustSwitchNormal > 7000 || _exhaustSwitchNormal < 2000) {
        _exhaustSwitchNormal = EXHAUST_RPM_SWITCHOVER_NORMAL;
    }

    if (_intakeSwitchRace > 7000 || _intakeSwitchRace < 2000) {
        _intakeSwitchRace = INTAKE_RPM_SWITCHOVER_RACE;
    }

    if (_exhaustSwitchRace > 7000 || _exhaustSwitchRace < 2000) {
        _exhaustSwitchRace = EXHAUST_RPM_SWITCHOVER_RACE;
    }
}
