/*
 * DataMonitor
 *
 * Esta clase monitorea los datos recopilados por el DataManager para detectar posibles problemas en el motor
 * (como temperaturas o presiones fuera de rango), inconsistencia en los valores de los sensores, etc.
 */

#include <stdint.h>
#include <OneWire.h>
// #include <DallasTemperature.h>
#include "DataManager.h"
#include "DataMonitor.h"

DataMonitor::DataMonitor() {
    _engOilPressureStatus = STATUS_OK;
    _engOilTempStatus = STATUS_OK;
    _gbOilTempStatus = STATUS_OK;
    _afrStatus = STATUS_OK;
    _tpsStatus = STATUS_OK;
    _rpmStatus = STATUS_OK;
    _voltageStatus = STATUS_OK;
    _lastRPMValue = 0;
    _rpmInputCheckTimer = 0;
    _rpmFailureResetTimer = 0;
    _rpmErrorsCount = 0;
    _engineOilPressTimer = 0;
    _engineOilPressStatusCooldown = false;
    _dataManager = NULL;
}

void DataMonitor::Initialize(DataManager *dataManager) {
    _dataManager = dataManager;
}

void DataMonitor::Update(uint32_t diff) {
    // Como en otros casos, no podemos hacer nada mientras no se haya inicializado
    // el DataManager y tengamos una referencia al mismo.
    if (!_dataManager)
        return;

    // Llamamos a la función auxiliar para cálculo de las RPM
    _dataManager->RetrieveRPM(micros());

    // Primero comprobamos los parámetros que no dependen necesariamente de si el motor está encendido o no
    // Estos son las temperaturas, TPS y voltaje
    // Temperatura del aceite del motor
    float engOilTemp = _dataManager->GetEngineOilTemp();
    if (engOilTemp == DS18B20_ERROR_TEMP) {
        _engOilTempStatus = STATUS_ERROR;
    } else if (engOilTemp < ENGINE_OIL_COLD_TEMP_LIMIT) {
        _engOilTempStatus = STATUS_COLD;
    } else if (engOilTemp < ENGINE_OIL_TEMP_WARNING) {
        _engOilTempStatus = STATUS_OK;
    } else if (engOilTemp < ENGINE_OIL_TEMP_DANGER) {
        _engOilTempStatus = STATUS_WARNING;
    } else if (engOilTemp >= ENGINE_OIL_TEMP_DANGER) {
        _engOilTempStatus = STATUS_DANGER;
    }
    // Temperatura del aceite de la caja de cambios
    float gbOilTemp = _dataManager->GetGearboxOilTemp();
    if (gbOilTemp == DS18B20_ERROR_TEMP) {
        _gbOilTempStatus = STATUS_ERROR;
    } else if (gbOilTemp < GEARBOX_OIL_COLD_TEMP_LIMIT) {
        _gbOilTempStatus = STATUS_COLD;
    } else if (gbOilTemp < GEARBOX_OIL_TEMP_WARNING) {
        _gbOilTempStatus = STATUS_OK;
    } else if (gbOilTemp < GEARBOX_OIL_TEMP_DANGER) {
        _gbOilTempStatus = STATUS_WARNING;
    } else if (gbOilTemp >= GEARBOX_OIL_TEMP_DANGER) {
        _gbOilTempStatus = STATUS_DANGER;
    }
    // TPS. Realmente no se puede comprobar mucho en este parámetro, devolveremos siempre OK por el momento
    _tpsStatus = STATUS_OK;
    // Voltaje. La comparación varía si el motor está encendido o no
    float voltage = _dataManager->GetVoltage();
    // Si el motor está encendido, sumamos 1.2v a los valores bajos definidos en DataMonitor.h
    // para compensar el voltaje adicional del alternador, y dar más tiempo de reacción si falla.
    if (_dataManager->IsEngineOn()) {
        if (voltage <= VOLTAGE_LOW_DANGER + 1.2) {
            _voltageStatus = STATUS_DANGER;
        } else if (voltage <= VOLTAGE_LOW_WARNING + 1.2) {
            _voltageStatus = STATUS_WARNING;
        } else if (voltage < VOLTAGE_HIGH_WARNING) {
            _voltageStatus = STATUS_OK;
        } else if (voltage < VOLTAGE_HIGH_DANGER) {
            _voltageStatus = STATUS_WARNING;
        } else if (voltage >= VOLTAGE_HIGH_DANGER) {
            _voltageStatus = STATUS_DANGER;
        }
    } else {
        if (voltage <= VOLTAGE_LOW_DANGER) {
            _voltageStatus = STATUS_DANGER;
        } else if (voltage <= VOLTAGE_LOW_WARNING) {
            _voltageStatus = STATUS_WARNING;
        } else if (voltage < VOLTAGE_HIGH_WARNING) {
            _voltageStatus = STATUS_OK;
        } else if (voltage < VOLTAGE_HIGH_DANGER) {
            _voltageStatus = STATUS_WARNING;
        } else if (voltage >= VOLTAGE_HIGH_DANGER) {
            _voltageStatus = STATUS_DANGER;
        }
    }

    // Comprobamos el estado de la señal de RPMs. Para ello utilizaremos dos técnicas:
    //   1) El bucle de control de fluctuación de la señal (que no varíe demasiado en un intervalo de tiempo pequeño)
    //   2) Si hay presión de aceite, comprobar que también haya RPMs. No puede haber presión de aceite sin RPMs!!
    //
    // En caso de fallo, la señal de RPMs queda marcada como defectuosa hasta que se reinicie el microprocesador
    int16_t rpms = (int16_t) _dataManager->GetRPM();
    if (_rpmStatus != STATUS_ERROR) {
        // Comprobamos si no nos hemos pasado de vueltas. La ECU de serie no interpreta valores por encima de las 8000 RPM
        // aproximadamente a la hora de cortar inyección. Así que si nos hemos pasado de RPMs con los mapas de carreras
        // (corte a unas 8500), podemos volver a los mapas de calle temporalmente para "activar" el corte a 8000 y no pasar
        // el motor de vueltas. El AuxManager se encarga de controlar este limitador simulado.
        if (rpms >= EMERGENCY_REV_LIMITER) {
            _rpmStatus = STATUS_DANGER;
        } else {
            _rpmStatus = STATUS_OK;
        }

        // Comprobamos que el valor no haya fluctuado demasiado desde la última comprobación
        if (_rpmInputCheckTimer >= RPM_INPUT_CHECK_INTERVAL) {
            // Ojo que este tiene que ser int, no unsigned
            int16_t rpmDiff = rpms - _lastRPMValue;
            if (rpmDiff < 0)
                rpmDiff *= -1; // Pasamos a positivo el valor
            
            if (rpmDiff >= MAX_RPM_DIFF_BETWEEN_CHECKS)
                ++_rpmErrorsCount;

            // Comprobamos ahora la señal de las RPMs contra la de presión de aceite
            if (_dataManager->GetEngineOilPressure() >= 1.0)
                if (rpms < 100) // Por debajo de 100 RPMS es imposible tener más de 1 bar de presión de aceite a no ser que estemos en el polo norte
                    ++_rpmErrorsCount;

            _lastRPMValue = rpms;
            _rpmInputCheckTimer = 0;
        } else {
            _rpmInputCheckTimer += diff;
        }

        // Reseteamos los errores acumulados si es necesario
        if (_rpmFailureResetTimer >= RPM_FAILURES_RESET_TIMER) {
            _rpmErrorsCount = 0;
            _rpmFailureResetTimer = 0;
        } else {
            _rpmFailureResetTimer += diff;
        }

        if (_rpmErrorsCount >= MAX_RPM_SIGNAL_ERRORS) {
            // Ponemos la señal de RPMs como defectuosa permanentemente si se han detectado el suficiente número de fallos en la señal
            _rpmStatus = STATUS_ERROR;
            _rpmFailureResetTimer = 0;
        }
    } else {
        // Reiniciamos el comprobador de RPM
        if (_rpmFailureResetTimer >= RPM_FAILURES_RESET_TIMER) {
            _rpmErrorsCount = 0;
            _rpmFailureResetTimer = 0;
            _rpmStatus = STATUS_DANGER;
        } else {
            _rpmFailureResetTimer += diff;
        }
    }

    // Ahora comprobamos parámetros que requieren que el motor esté encendido para funcionar correctamente.
    // Estos parámetros son presión de aceite del motor, RPMs y AFR.
    if (_dataManager->IsEngineOn()) {
        // Presión de aceite. Es uno de los parámetros más importantes.
        // Para comprobarla nos basaremos en las RPMs y la temperatura del aceite.
        float oilPress = _dataManager->GetEngineOilPressure();
        bool badOilPress = false;
        // Comprobamos el timer de cooldown, que se activará si se detecta una caída en la presión y mantendrá el estado DANGER
        // durante un mínimo de tiempo para asegurarnos de que el CommsManager lo pilla y envía el fallo al TFT
        if (_engineOilPressStatusCooldown) {
            if (_engineOilPressTimer >= MIN_OIL_PRESS_DANGER_TIMER) {
                _engineOilPressStatusCooldown = false;
                _engineOilPressTimer = 0;
            } else {
                _engineOilPressTimer += diff;
            }
        } else {
            if (engOilTemp <= ENGINE_OIL_COLD_TEMP_LIMIT) {
                if (oilPress < ENGINE_OIL_PRESS_MIN)
                    badOilPress = true;
            } else {
                if (oilPress < ENGINE_OIL_PRESS_MIN_HOT)
                    badOilPress = true;
            }
            // Comprobamos que a partir de ciertas RPMs la presión de aceite no caiga de cierto valor
            if (rpms >= ENGINE_OIL_PRESS_RPM_CHECK) {
                if (oilPress < ENGINE_OIL_PRESS_MIN_RPMS) {
                    badOilPress = true;
                }
            }

            if (badOilPress) {
                _engOilPressureStatus = STATUS_DANGER; // Con la presión de aceite no hay medias tintas, o va bien o no va.
                _engineOilPressStatusCooldown = true;
            } else {
                _engOilPressureStatus = STATUS_OK;
            }
        }

        // AFR. Comprobamos si la sonda Wideband todavía está calentándose o fuera de parámetros.
        float afr = _dataManager->GetAFR();
        if (afr == -1.0) {
            _afrStatus = STATUS_COLD; // Sonda calentándose
        } else {
            if (afr <= AFR_RICH_DANGER) {
                _afrStatus = engOilTemp >= 50.0 ? STATUS_DANGER : STATUS_OK;  // Ignoramos valores demasiado ricos si el motor está frío
            } else if (afr <= AFR_RICH_WARNING) {
                _afrStatus = engOilTemp >= 50.0 ? STATUS_WARNING : STATUS_OK; // Ídem
            } else if (afr < AFR_LEAN_WARNING) {
                _afrStatus = STATUS_OK;
            } else if (afr < AFR_LEAN_DANGER) {
                _afrStatus = STATUS_WARNING;
            } else if (afr >= AFR_LEAN_DANGER) {
                _afrStatus = STATUS_DANGER;
            }
        }
    } else {
        _afrStatus = STATUS_COLD;
        _engOilPressureStatus = STATUS_OK;
    }
}
