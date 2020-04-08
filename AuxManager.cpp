/*
 * AuxManager
 * 
 * Esta clase se encarga de activar los mapas de emergencia en caso de que algo falle, 
 * y controlar el corte de inyección por encima de 8000 rpm cambiando entre mapas. También
 * gestiona funciones secundarias o menos relevantes como la alimentación de la controladora
 * de la sonda wideband, señal emulada de la sonda lambda, control de los botones y mapas, etc.
 */

#include <stdint.h>
#include <OneWire.h>
#include "DataManager.h"
#include "DataMonitor.h"
#include "AuxManager.h"

AuxManager::AuxManager() {
    _dataMonitor = NULL;
    _dataManager = NULL;

    _currentECUMap = ECU_MAP_NORMAL;
    _nextCommand = COMMAND_NONE;
    _afrGaugeOn = false;
    _controlButtonTimer = 0;
    _controlButtonCooldownTimer = 0;
    _mapSwitchCooldownTimer = 0;
    _isControlButtonInCooldown = false;
    _isMapSwitchInCooldown = false;
    _isLimiterEnabled = false;

    pinMode(INPUT_CONTROL_BUTTON, INPUT);
    pinMode(INPUT_MAPS_SWITCH_BUTTON, INPUT);

    pinMode(OUTPUT_AFR_GAUGE_VCC, OUTPUT);
    digitalWrite(OUTPUT_AFR_GAUGE_VCC, HIGH);
    pinMode(OUTPUT_LAMBDA, OUTPUT);
    digitalWrite(OUTPUT_LAMBDA, LOW);
    pinMode(OUTPUT_MAP_SWITCH, OUTPUT);
    digitalWrite(OUTPUT_MAP_SWITCH, LOW);
}

void AuxManager::Initialize(DataManager *dataManager, DataMonitor *dataMonitor) {
    _dataManager = dataManager;
    _dataMonitor = dataMonitor;
}

void AuxManager::Update(uint32_t diff) {
    if (!_dataManager || !_dataMonitor)
        return;

    // Llamamos a la función auxiliar para cálculo de las RPM
    _dataManager->RetrieveRPM(micros());

    // Primero comprobamos el mapa que activar
    ECUMaps newMap = _currentECUMap;
    // Modo carreras?
    if (!_isMapSwitchInCooldown) { // Cooldown, intervalo de tiempo mínimo entre cambios de mapa
        if (digitalRead(INPUT_MAPS_SWITCH_BUTTON) == HIGH) {
            newMap = ECU_MAP_RACE;
        } else {
            newMap = ECU_MAP_NORMAL;
        }
    } else {
        uint32_t cdTimer = _isLimiterEnabled ? RPM_LIMITER_HYSTERESIS : MAP_SWITCH_COOLDOWN;
        if (_mapSwitchCooldownTimer >= cdTimer) {
            _mapSwitchCooldownTimer = 0;
            _isMapSwitchInCooldown = false;
            _isLimiterEnabled = false;
        } else {
            _mapSwitchCooldownTimer += diff;
        }
    }
    
    // Comprobaciones para parámetros fuera de lo normal. Primero comprobamos si bajamos al modo normal por sobretemperatura
    if (_dataMonitor->GetEngineOilTempStatus() == STATUS_DANGER || _dataMonitor->GetGearboxOilTempStatus() == STATUS_DANGER)
        newMap = ECU_MAP_NORMAL; // Aunque estos parámetros estén en valores peligrosos, lo mejor que podemos hacer es volver al modo normal, no al de emergencia, para seguir controlando las levas
    // Comprobamos si nos hemos pasado de vueltas. Con los mapas de carreras, el limitador de serie no funciona, así que forzamos los mapas de calle para activar el limitador.
    if (_dataMonitor->GetRPMStatus() == STATUS_DANGER) {
        newMap = ECU_MAP_NORMAL;
        _isLimiterEnabled = true;
    }
    // Ahora comprobamos si es necesario activar el modo emergencia, sólo en el caso de una mala señal de RPMs, ya que no podremos controlar las levas
    if (_dataMonitor->GetRPMStatus() == STATUS_ERROR)
        newMap = ECU_MAP_EMERGENCY;

    if (newMap != _currentECUMap)
        SwitchMaps(newMap);

    // Control de la alimentación de la sonda wideband
    if (!_afrGaugeOn && _dataManager->IsEngineOn()) {
        SwitchAFRGaugePower(true);
    } else if (_afrGaugeOn && !_dataManager->IsEngineOn()) {
        SwitchAFRGaugePower(false);
    }

    // Control de la emulación de la sonda lambda
    if (_dataMonitor->GetAFRStatus() != STATUS_COLD) {
        float afr = _dataManager->GetAFR();
        if (afr > 14.7) {
            SetLambdaEmulation(true);
        } else if (afr < 14.7) {
            SetLambdaEmulation(false);
        }
    }

    // Botón de control
    if (!_isControlButtonInCooldown) {
        if (digitalRead(INPUT_CONTROL_BUTTON) == HIGH) {
            if (_controlButtonTimer >= INTERVAL_SWITCH_TFT_MODE) {
                // Establecemos el comando para cambiar el modo de la pantalla TFT
                _nextCommand = COMMAND_CHANGE_SCREEN;
                _controlButtonTimer = 0;
                _isControlButtonInCooldown = true; // Para no enviar otro comando a continuación por error
            } else {
                _controlButtonTimer += diff;
            }
        } else {
            // Si el intervalo empezó a contar pero no ha llegado al límite, enviamos el comando de cambio de brillo
            if (_controlButtonTimer > 0)
                _nextCommand = COMMAND_CHANGE_BRIGHTNESS;

            _controlButtonTimer = 0;
            // _isControlButtonInCooldown = false; En este caso no es necesario ningún tipo de cooldown
        }
    } else {
        if (_controlButtonCooldownTimer >= INTERVAL_COOLDOWN) {
            _isControlButtonInCooldown = false;
            _controlButtonCooldownTimer = 0;
        } else {
            _controlButtonCooldownTimer += diff;
        }
    }
}

void AuxManager::SwitchAFRGaugePower(bool on) {
    if (on)
        digitalWrite(OUTPUT_AFR_GAUGE_VCC, LOW); // Lógica invertida
    else
        digitalWrite(OUTPUT_AFR_GAUGE_VCC, HIGH);

    _afrGaugeOn = on;
}

void AuxManager::SwitchMaps(ECUMaps map) {
    if (map == ECU_MAP_RACE)
        digitalWrite(OUTPUT_MAP_SWITCH, HIGH);
    else
        digitalWrite(OUTPUT_MAP_SWITCH, LOW); // Modo emergencia y modo normal desde el punto de vista de la ECU del coche son lo mismo

    _currentECUMap = map;
    _isMapSwitchInCooldown = true;
}

void AuxManager::SetLambdaEmulation(bool lean) {
    if (lean)
        digitalWrite(OUTPUT_LAMBDA, HIGH);
    else
        digitalWrite(OUTPUT_LAMBDA, LOW);    
}

Commands AuxManager::GetNextCommand() {
    Commands nextCommand = _nextCommand;
    _nextCommand = COMMAND_NONE;
    return nextCommand;
}

ECUMaps AuxManager::GetCurrentECUMap() {
    return (_isLimiterEnabled ? ECU_MAP_RACE : _currentECUMap); // El limitador sólo puede estar activo en los mapas de carreras.
}

bool AuxManager::IsLimiterEnabled() {
    return _isLimiterEnabled;
}