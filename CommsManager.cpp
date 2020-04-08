/*
 * CommsManager
 * 
 * Esta clase se encarga de gestionar la conexión serial y la transmisión de datos al Arduino
 * que controla la pantalla TFT.
 */

#include <stdint.h>
#include <OneWire.h>
#include "EEPROMManager.h"
#include "DataManager.h"
#include "DataMonitor.h"
#include "AuxManager.h"
#include "NeoVVLManager.h"
#include "CommsManager.h"

CommsManager::CommsManager() {
    _eepromManager = NULL;
    _dataManager = NULL;
    _dataMonitor = NULL;
    _neoVVLManager = NULL;
    _auxManager = NULL;
    _intervalBetweenPacketsTimer = 0;
    _packet = {
        .rpms = 0,
        .engOilPress = 0.0,
        .engOilTemp = 0.0,
        .gbOilTemp = 0.0,
        .afr = 0.0,
        .voltage = 0.0,
        .tps = 0,
        .engOilPressStatus = STATUS_OK,
        .engOilTempStatus = STATUS_OK,
        .gbOilTempStatus = STATUS_OK,
        .afrStatus = STATUS_OK,
        .voltageStatus = STATUS_OK,
        .tpsStatus = STATUS_OK,
        .selectedECUMap = STATUS_OK,
        .neoVVLStatus = STATUS_OK,
        .command = COMMAND_NONE
    };
    Serial1.begin(BAUD_RATE);
}

void CommsManager::Initialize(EEPROMManager *eepromManager, DataManager *dataManager, DataMonitor *dataMonitor, AuxManager *auxManager, NeoVVLManager *neoVVLManager) {
    _eepromManager = eepromManager;
    _dataManager = dataManager;
    _dataMonitor = dataMonitor;
    _auxManager = auxManager;
    _neoVVLManager = neoVVLManager;
}

void CommsManager::Update(uint32_t diff) {
    if (!_dataManager || !_dataMonitor || !_neoVVLManager || !_auxManager)
        return;

    if (_intervalBetweenPacketsTimer >= INTERVAL_BETWEEN_PACKETS) {
        // Montamos el paquete a enviar. Con la librería Wire, el tamaño máximo de cada transmisión es de 32 bytes.
        // Primero obtenemos todos los valores que queremos enviar
        _packet.rpms = _dataManager->GetRPM();
        _packet.engOilPress = _dataManager->GetEngineOilPressure();
        _packet.engOilTemp = _dataManager->GetEngineOilTemp();
        _packet.gbOilTemp = _dataManager->GetGearboxOilTemp();
        _packet.afr = _dataManager->GetAFR();
        _packet.voltage = _dataManager->GetVoltage();
        _packet.tps = _dataManager->GetTPS();
        _packet.engOilPressStatus = _dataMonitor->GetEngineOilPressureStatus();
        _packet.engOilTempStatus = _dataMonitor->GetEngineOilTempStatus();
        _packet.gbOilTempStatus = _dataMonitor->GetGearboxOilTempStatus();
        _packet.afrStatus = _dataMonitor->GetAFRStatus();
        _packet.voltageStatus = _dataMonitor->GetVoltageStatus();
        _packet.tpsStatus = _dataMonitor->GetTPSStatus();
        _packet.selectedECUMap = _auxManager->GetCurrentECUMap();
        uint8_t intakeCamStatus = _neoVVLManager->GetIntakeCamStatus();
        uint8_t exhaustCamStatus = _neoVVLManager->GetExhaustCamStatus();
        if (intakeCamStatus == CAM_STATUS_ENABLED && exhaustCamStatus == CAM_STATUS_ENABLED) {
            _packet.neoVVLStatus = NEOVVL_STATUS_BOTH_ON;
        } else if (intakeCamStatus == CAM_STATUS_DISABLED && exhaustCamStatus == CAM_STATUS_DISABLED) {
            _packet.neoVVLStatus = NEOVVL_STATUS_BOTH_OFF;
        } else if (intakeCamStatus == CAM_STATUS_ENABLED) {
            _packet.neoVVLStatus = NEOVVL_STATUS_INTAKE_ON;
        } else if (exhaustCamStatus == CAM_STATUS_ENABLED) {
            _packet.neoVVLStatus = NEOVVL_STATUS_EXHAUST_ON;
        }  else {
            _packet.neoVVLStatus = NEOVVL_STATUS_BOTH_OFF;
        }
        _packet.command = _auxManager->GetNextCommand();
        // Enviamos el paquete con todos los datos
        SendPacket();
        // Llamamos a la función auxiliar para cálculo de las RPM
        _dataManager->RetrieveRPM(micros());

        _intervalBetweenPacketsTimer = 0;
    } else {
        _intervalBetweenPacketsTimer += diff;
    }

    // TODO: Habría que transformar este tocho de código en unas bonitas funciones más genéricas...
    if (Serial1.available()) {
        String usbCommand = Serial1.readStringUntil(';');
        if (usbCommand.indexOf("set INTERVAL_BETWEEN_PACKETS") != -1) {
            String strValue = usbCommand.substring(29);
            int16_t value = strValue.toInt();
            if (value) {
                if (value < 10 || value > 10000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_INTERVAL_BETWEEN_PACKETS, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set BAUD_RATE") != -1) {
            String strValue = usbCommand.substring(14);
            int32_t value = strValue.toInt();
            if (value) {
                if (value < 9600 || value > 2000000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_BAUD_RATE, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set INTAKE_RPM_SWITCHOVER_NORMAL") != -1) {
            String strValue = usbCommand.substring(33);
            int16_t value = strValue.toInt();
            if (value) {
                if (value < 2000 || value > 8000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_INTAKE_RPM_SWITCHOVER_NORMAL, value);
                    Serial1.println("success;");
                    _neoVVLManager->LoadCamsSwitchPointsFromEEPROM();
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set EXHAUST_RPM_SWITCHOVER_NORMAL") != -1) {
            String strValue = usbCommand.substring(34);
            int16_t value = strValue.toInt();
            if (value) {
                if (value < 2000 || value > 8000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_EXHAUST_RPM_SWITCHOVER_NORMAL, value);
                    Serial1.println("success;");
                    _neoVVLManager->LoadCamsSwitchPointsFromEEPROM();
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set INTAKE_RPM_SWITCHOVER_RACE") != -1) {
            String strValue = usbCommand.substring(31);
            int16_t value = strValue.toInt();
            if (value) {
                if (value < 2000 || value > 8000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_INTAKE_RPM_SWITCHOVER_RACE, value);
                    Serial1.println("success;");
                    _neoVVLManager->LoadCamsSwitchPointsFromEEPROM();
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set EXHAUST_RPM_SWITCHOVER_RACE") != -1) {
            String strValue = usbCommand.substring(32);
            int16_t value = strValue.toInt();
            if (value) {
                if (value < 2000 || value > 8000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_EXHAUST_RPM_SWITCHOVER_RACE, value);
                    Serial1.println("success;");
                    _neoVVLManager->LoadCamsSwitchPointsFromEEPROM();
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set CAMS_SWITCHOVER_COOLDOWN") != -1) {
            String strValue = usbCommand.substring(29);
            int16_t value = strValue.toInt();
            if (value) {
                if (value < 100 || value > 1000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_CAMS_SWITCHOVER_COOLDOWN, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set RPM_LIMITER_HYSTERESIS") != -1) {
            String strValue = usbCommand.substring(27);
            int16_t value = strValue.toInt();
            if (value) {
                if (value < 100 || value > 1000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_RPM_LIMITER_HYSTERESIS, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set MAX_RPM_DIFF_BETWEEN_CHECKS") != -1) {
            String strValue = usbCommand.substring(32);
            int16_t value = strValue.toInt();
            if (value) {
                if (value < 1000 || value > 10000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_MAX_RPM_DIFF_BETWEEN_CHECKS, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set RPM_INPUT_CHECK_INTERVAL") != -1) {
            String strValue = usbCommand.substring(29);
            int16_t value = strValue.toInt();
            if (value) {
                if (value < 10 || value > 1000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_RPM_INPUT_CHECK_INTERVAL, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set RPM_FAILURES_RESET_TIMER") != -1) {
            String strValue = usbCommand.substring(29);
            int32_t value = strValue.toInt();
            if (value) {
                if (value < 1000 || value > 10000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_RPM_FAILURES_RESET_TIMER, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set MAX_RPM_SIGNAL_ERRORS") != -1) {
            String strValue = usbCommand.substring(26);
            int8_t value = strValue.toInt();
            if (value) {
                if (value < 1 || value > 100) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_MAX_RPM_SIGNAL_ERRORS, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set EMERGENCY_REV_LIMITER") != -1) {
            String strValue = usbCommand.substring(26);
            int16_t value = strValue.toInt();
            if (value) {
                if (value < 2000 || value > 10000) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_EMERGENCY_REV_LIMITER, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        }
        // ***********************
        // Floats a partir de aquí
        // ***********************
        else if (usbCommand.indexOf("set ENGINE_OIL_COLD_TEMP_LIMIT") != -1) {
            String strValue = usbCommand.substring(31);
            float value = strValue.toFloat();
            if (value) {
                if (value < 0.0 || value > 100.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_ENGINE_OIL_COLD_TEMP_LIMIT, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set ENGINE_OIL_TEMP_WARNING") != -1) {
            String strValue = usbCommand.substring(30);
            float value = strValue.toFloat();
            if (value) {
                if (value < 90.0 || value > 120.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_ENGINE_OIL_TEMP_WARNING, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set ENGINE_OIL_TEMP_DANGER") != -1) {
            String strValue = usbCommand.substring(29);
            float value = strValue.toFloat();
            if (value) {
                if (value < 90.0 || value > 130.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_ENGINE_OIL_TEMP_DANGER, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set GEARBOX_OIL_COLD_TEMP_LIMIT") != -1) {
            String strValue = usbCommand.substring(35);
            float value = strValue.toFloat();
            if (value) {
                if (value < 0.0 || value > 100.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_GEARBOX_OIL_COLD_TEMP_LIMIT, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set GEARBOX_OIL_TEMP_WARNING") != -1) {
            String strValue = usbCommand.substring(32);
            float value = strValue.toFloat();
            if (value) {
                if (value < 70.0 || value > 120.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_GEARBOX_OIL_TEMP_WARNING, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set GEARBOX_OIL_TEMP_DANGER") != -1) {
            String strValue = usbCommand.substring(31);
            float value = strValue.toFloat();
            if (value) {
                if (value < 70.0 || value > 130.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_GEARBOX_OIL_TEMP_DANGER, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set AFR_RICH_WARNING") != -1) {
            String strValue = usbCommand.substring(24);
            float value = strValue.toFloat();
            if (value) {
                if (value < 8.0 || value > 15.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_AFR_RICH_WARNING, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set AFR_RICH_DANGER") != -1) {
            String strValue = usbCommand.substring(23);
            float value = strValue.toFloat();
            if (value) {
                if (value < 8.0 || value > 15.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_AFR_RICH_DANGER, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set AFR_LEAN_WARNING") != -1) {
            String strValue = usbCommand.substring(24);
            float value = strValue.toFloat();
            if (value) {
                if (value < 12.0 || value > 17.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_AFR_LEAN_WARNING, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set AFR_LEAN_DANGER") != -1) {
            String strValue = usbCommand.substring(23);
            float value = strValue.toFloat();
            if (value) {
                if (value < 12.0 || value > 17.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_AFR_LEAN_DANGER, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set VOLTAGE_LOW_WARNING") != -1) {
            String strValue = usbCommand.substring(27);
            float value = strValue.toFloat();
            if (value) {
                if (value < 10.0 || value > 14.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_VOLTAGE_LOW_WARNING, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set VOLTAGE_LOW_DANGER") != -1) {
            String strValue = usbCommand.substring(26);
            float value = strValue.toFloat();
            if (value) {
                if (value < 10.0 || value > 14.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_VOLTAGE_LOW_DANGER, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set VOLTAGE_HIGH_WARNING") != -1) {
            String strValue = usbCommand.substring(28);
            float value = strValue.toFloat();
            if (value) {
                if (value < 12.0 || value > 20.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_VOLTAGE_HIGH_WARNING, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set VOLTAGE_HIGH_DANGER") != -1) {
            String strValue = usbCommand.substring(27);
            float value = strValue.toFloat();
            if (value) {
                if (value < 12.0 || value > 20.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_VOLTAGE_HIGH_DANGER, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set ENGINE_OIL_PRESS_MIN") != -1) {
            String strValue = usbCommand.substring(28);
            float value = strValue.toFloat();
            if (value) {
                if (value < 0.8 || value > 3.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_ENGINE_OIL_PRESS_MIN, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set ENGINE_OIL_PRESS_MIN_HOT") != -1) {
            String strValue = usbCommand.substring(32);
            float value = strValue.toFloat();
            if (value) {
                if (value < 0.5 || value > 3.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_ENGINE_OIL_PRESS_MIN_HOT, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else if (usbCommand.indexOf("set ENGINE_OIL_PRESS_MIN_RPMS") != -1) {
            String strValue = usbCommand.substring(33);
            float value = strValue.toFloat();
            if (value) {
                if (value < 1.0 || value > 5.0) {
                    Serial1.println("error: out of range;");
                } else {
                    _eepromManager->SaveToEEPROM(ADDR_ENGINE_OIL_PRESS_MIN_RPMS, value);
                    Serial1.println("success;");
                }
            }
            Serial1.println("syntax error;");
        } else {
            Serial1.println("error: unrecognized command;");
        }
    }
}

void CommsManager::SendPacket() {
    // uint32_t currentMicros = micros();
    union u_int16 {
        byte b[2];
        int16_t value;
    } u;
    
    Serial1.write("#"); // Byte de control, indica el comienzo de la transmisión
    // Pasamos las RPM de un int16 a 2 bytes separados
    u.value = _packet.rpms;
    // Serial.print("Sending "); Serial.println(u.value);
    Serial1.write(u.b[0]);
    Serial1.write(u.b[1]);
    // Pasamos presión de aceite a int16
    u.value = (int16_t) (_packet.engOilPress * 100);
    Serial1.write(u.b[0]);
    Serial1.write(u.b[1]);
    // Temperatura del aceite del motor
    u.value = (int16_t) (_packet.engOilTemp * 100);
    Serial1.write(u.b[0]);
    Serial1.write(u.b[1]);
    // Temperatura del aceite de la caja de cambios
    u.value = (int16_t) (_packet.gbOilTemp * 100);
    Serial1.write(u.b[0]);
    Serial1.write(u.b[1]);
    // AFR
    u.value = (int16_t) (_packet.afr * 100);
    Serial1.write(u.b[0]);
    Serial1.write(u.b[1]);
    // Voltaje
    u.value = (int16_t) (_packet.voltage * 100);
    Serial1.write(u.b[0]);
    Serial1.write(u.b[1]);
    // A partir de aquí ya son todo uint8_t (bytes sueltos, no hay que convertir nada)
    Serial1.write(_packet.tps);
    Serial1.write(_packet.engOilPressStatus);
    Serial1.write(_packet.engOilTempStatus);
    Serial1.write(_packet.gbOilTempStatus);
    Serial1.write(_packet.afrStatus);
    Serial1.write(_packet.voltageStatus);
    Serial1.write(_packet.tpsStatus);
    Serial1.write(_packet.selectedECUMap);
    Serial1.write(_packet.neoVVLStatus);
    Serial1.write(_packet.command);
    Serial1.write("*"); // Byte de control, indica el final de la transmisión

    // Serial.println(micros() - currentMicros);
}
