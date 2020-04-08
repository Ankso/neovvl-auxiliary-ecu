/*
 * DataManager
 * 
 * Esta clase se encarga de recuperar los datos desde las diferentes entradas y sensores para procesarlos
 * y obetener valores fácilmente manipulables para el resto de Managers.
 */

#include <stdint.h>
#include <OneWire.h>
// #include <DallasTemperature.h> // Implementación asíncrona propia, la librería normal tiene varios delays y no nos sirve para esto
#include "DataManager.h"

DataManager::DataManager() : _engOilTempWire(INPUT_ENG_OIL_TEMP), _gbOilTempWire(INPUT_GEARBOX_OIL_TEMP) {
    // Simplemente inicializamos las variables
    _engineOilTemp = 0;
    _gearboxOilTemp = 0;
    _engineOilPressure = 0;
    _tps = 0;
    _tpsMinValue = analogRead(INPUT_TPS);
    _tpsMaxValue = 3.5;
    _voltage = 0;
    _micros = 0;
    _lastMicros = 0;
    _rpmInterval = 0;
    _rpmLowVoltageInputCount = 0;
    _rpmTriggerCooldown = false;
    _selectAuxRPMInput = false;
    _startupCheckExecuted = false;
    _secondaryDataTimer = 0;
    _startupCheckTimer = 0;
    _tempDataTimer = TEMP_DATA_INTERVAL; // Así se solicitarán las temperaturas en la primera iteración de Update()
    _engOilTempTimer = 0;
    _gbOilTempTimer = 0;
    _engOilTempRequested = false;
    _gbOilTempRequested = false;
    for (uint8_t i = 0; i < AVERAGE_AFR_COUNT_LIMIT; ++i) {
        _afr[i] = 0;
    }
    for (uint8_t i = 0; i < AVERAGE_RPM_COUNT_LIMIT; ++i) {
        _rpm[i] = 0;
    }
    _rpmIndex = 0;
    _afrIndex = 0;

    // Inicialización de los pines con los diferentes inputs
    pinMode(INPUT_ENG_OIL_PRESSURE, INPUT);
    pinMode(INPUT_RPM_SIGNAL, INPUT);
    pinMode(INPUT_RPM_SIGNAL_AUX, INPUT);
    pinMode(INPUT_TPS, INPUT);
    pinMode(INPUT_AFR, INPUT);
    pinMode(INPUT_VOLTAGE, INPUT);

    // Inicialización de los buses OneWire
    // OneWire _engOilTempWire(INPUT_ENG_OIL_TEMP);
    // OneWire _gbOilTempWire(INPUT_GEARBOX_OIL_TEMP);
    // Buscamos y almacenamos las IDs de los sensores
    _engOilTempWire.reset_search();
    _gbOilTempWire.reset_search();
    _engOilTempWire.search(_engOilTempAddress);
    _gbOilTempWire.search(_gbOilTempAddress);
    // Ajustamos la resolución de los sensores
    Dallas_setResolution(_engOilTempAddress, &_engOilTempWire);
    Dallas_setResolution(_gbOilTempAddress, &_gbOilTempWire);
}

void DataManager::Update(uint32_t diff) {
    // Recuperamos siempre la información más reciente de los sensores más importantes
    // Comprobamos si el voltaje de la señal de las RPM es demasiado bajo, y si se da el caso, pasamos al modo auxiliar.
    // En caso contrario, las RPMs se calculan con interrupts.
    uint16_t auxRPMSignal = analogRead(INPUT_RPM_SIGNAL_AUX);
    if (auxRPMSignal <= RPM_ANALOG_MIN_VALUE && auxRPMSignal > 200) {
        ++_rpmLowVoltageInputCount;
        if (_rpmLowVoltageInputCount >= MAX_RPM_INPUT_LOW_VOLTAGE_ERRORS) {
            _selectAuxRPMInput = true;
        }
    } else {
        _selectAuxRPMInput = false;
        _rpmLowVoltageInputCount = 0;
    }
    RetrieveRPM(micros());
    RetrieveEngineOilPressure();
    RetrieveAFR();

    // Este timer sólo se ejecuta una vez al iniciar la centralita, para dar tiempo a que la ECU del coche se inicialice
    if (!_startupCheckExecuted) {
        if (_startupCheckTimer >= STARTUP_CHECK_INTERVAL) {
            ExecuteStartupCheck();
            _startupCheckExecuted = true;
        } else {
            _startupCheckTimer += diff;
        }
    }

    // Ahora controlamos los timers para obtener información menos relevante.
    if (_secondaryDataTimer >= SECONDARY_DATA_INTERVAL) {
        RetrieveTPS();
        RetrieveVoltage();
        _secondaryDataTimer = 0;
    } else {
        _secondaryDataTimer += diff;
    }

    // Esta parte es interesante, porque necesitamos varios timers para recuperar las temperaturas.
    // Las sondas DS18B20 necesitan un tiempo para procesar la respuesta desde que les llega la petición
    // de temperatura. La librería DallasTemperature espera a la respuesta utilizando delay(), pero Nosotros
    // no podemos hacer eso, porque paralizaría la ejecución del programa. Con la implementación de abajo
    // de la librería Dallas y estas líneas de código, convertimos esas peticiones en asíncronas con timers.
    if (_tempDataTimer >= TEMP_DATA_INTERVAL) {
        RetrieveEngineOilTemp(false);
        RetrieveGearboxOilTemp(false);
        _tempDataTimer = 0;
    } else {
        _tempDataTimer += diff;
    }

    if (_engOilTempRequested) {
        if (_engOilTempTimer >= DS18B20_UPDATE_INTERVAL) {
            RetrieveEngineOilTemp(true);
            _engOilTempTimer = 0;
            _engOilTempRequested = false;
        } else {
            _engOilTempTimer += diff;
        }
    }

    if (_gbOilTempRequested) {
        if (_gbOilTempTimer >= DS18B20_UPDATE_INTERVAL) {
            RetrieveGearboxOilTemp(true);
            _gbOilTempTimer = 0;
            _gbOilTempRequested = false;
        } else {
            _gbOilTempTimer += diff;
        }
    }

    if (micros() - _lastMicros >= RPM_INPUT_INTERVAL_MAX && !_selectAuxRPMInput) {
        // Ponemos las RPM a 0 en caso de que bajen de 60
        _rpm[_rpmIndex] = 0;
        _lastMicros = 0;
        ++_rpmIndex;
        if (_rpmIndex >= AVERAGE_RPM_COUNT_LIMIT) {
            _rpmIndex = 0;
        }
    }
}

void DataManager::RetrieveEngineOilPressure() {
    _engineOilPressure = analogRead(INPUT_ENG_OIL_PRESSURE);
}

void DataManager::RetrieveEngineOilTemp(bool requestCompleted) {
    // Iniciamos una nueva petición al sensor. En la próxima llamada a la función, hay que pasar
    // el parámetro requestCompleted como true, para indicar que ya podemos obtener la temperatura.
    if (!requestCompleted) {
        _engOilTempWire.reset();
        _engOilTempWire.skip();
        _engOilTempWire.write(STARTCONVO, false);
        _engOilTempRequested = true;
    } else {
        // Aquí el sensor ya debería de estar listo para enviar la temperatura procesada.
        _engineOilTemp = Dallas_getTemp(_engOilTempAddress, &_engOilTempWire);
    }
}

void DataManager::RetrieveGearboxOilTemp(bool requestCompleted) {
    // Lo mismo que la función anterior
    if (!requestCompleted) {
        _gbOilTempWire.reset();
        _gbOilTempWire.skip();
        _gbOilTempWire.write(STARTCONVO, false);
        _gbOilTempRequested = true;
    } else {
        // Aquí el sensor ya debería de estar listo para enviar la temperatura procesada.
        _gearboxOilTemp = Dallas_getTemp(_gbOilTempAddress, &_gbOilTempWire);
    }
}

void DataManager::RetrieveTPS() {
    _tps = analogRead(INPUT_TPS);

    // Ajustamos el valor máximo si es necesario
    if (_tps * ANALOG_TO_VOLTS > _tpsMaxValue)
        _tpsMaxValue = (_tps * ANALOG_TO_VOLTS) - 0.2; // Restamos 0.2 voltios para asegurarnos de que nunca nos pasamos. Desde el punto de vista de la ECU del coche, 80% o más es WOT.
}

void DataManager::RetrieveAFR() {
    _afr[_afrIndex] = analogRead(INPUT_AFR);
    ++_afrIndex;
    if (_afrIndex >= AVERAGE_AFR_COUNT_LIMIT) {
        _afrIndex = 0;
    }
}

void DataManager::RetrieveRPM(uint32_t newMicros) {
    // Obtenemos los microsegundos desde la última ejecución.
    // Sólo utilizamos este tipo de precisión aquí, para calcular RPMs, en el resto del programa no es necesaria, con millis vale.
    uint32_t microDiff = newMicros - _micros;
    _micros = newMicros;
    
    if (newMicros < _micros) {
        _micros = 0; // Para evitar problemas con el overflow de la variable interna de Arduino (cada 70 minutos más o menos)
    }

    // A partir de aquí sólo sigue si estamos usando el sistema auxiliar para recuperar las RPM
    if (!_selectAuxRPMInput)
        return;

    uint16_t rpmStatus = analogRead(INPUT_RPM_SIGNAL);
    
    // La ECU del coche por defecto pone a masa este pin. Cuando la bobina se activa, la ECU corta la masa y el voltaje aumenta
    if (!_rpmTriggerCooldown && rpmStatus >= RPM_INPUT_HIGH_VALUE) {
        // Obtenemos las RPMs con el intervalo entre encendidos
        _rpm[_rpmIndex] = _rpmInterval;
        // Reiniciamos las variables para esperar al siguiente encendido
        _rpmTriggerCooldown = true;
        _rpmInterval = 0;
        ++_rpmIndex;
        if (_rpmIndex >= AVERAGE_RPM_COUNT_LIMIT) {
            _rpmIndex = 0;
        }
    } else if (rpmStatus < RPM_INPUT_HIGH_VALUE) {
        // Leemos como mínimo 60 RPM
        if (_rpmInterval <= RPM_INPUT_INTERVAL_MAX) {
            _rpmInterval += microDiff;
        } else {
            _rpm[_rpmIndex] = 0;
            ++_rpmIndex;
            if (_rpmIndex >= AVERAGE_RPM_COUNT_LIMIT) {
                _rpmIndex = 0;
            }
        }
        _rpmTriggerCooldown = false;
    }
}

void DataManager::CalculateRPM(uint32_t currentMicros) {
    // Comprobamos si estamos usando el sistema auxiliar.
    if (_selectAuxRPMInput)
        return;

    // Primer encendido del coche, simplemente almacenamos el tiempo para calcular las RPM en el siguiente chispazo.
    // También comprobamos si se ha reiniciado la variable que gestiona los micros()
    if (_lastMicros == 0 || _lastMicros > currentMicros) {
        _lastMicros = currentMicros;
        return;
    }

    _rpm[_rpmIndex] = currentMicros - _lastMicros;
    ++_rpmIndex;
    if (_rpmIndex >= AVERAGE_RPM_COUNT_LIMIT) {
        _rpmIndex = 0;
    }
    _lastMicros = currentMicros;
}

void DataManager::RetrieveVoltage() {
    _voltage = analogRead(INPUT_VOLTAGE);
}

void DataManager::ExecuteStartupCheck() {
    // En esta función se gestiona cualquier acción que requiera que ambas centralitas (esta y la del coche)
    // estén completamente inicializadas y listas.
    _tpsMinValue = analogRead(INPUT_TPS) * ANALOG_TO_VOLTS;
    _startupCheckExecuted = true;
}

float DataManager::GetEngineOilPressure(bool raw) {
    if (raw)
        return _engineOilPressure;

    // El sensor envía una señal analógica y lineal de entre 0.5v @0 PSI -> 4.5v @150 PSI
    // Primero multiplicamos el valor por 0.0048828125‬ para obtener los voltios en el pin
    float value = (float) _engineOilPressure * ANALOG_TO_VOLTS;
    // Después descartamos valores por debajo o por encima de los límites del sensor
    if (value <= 0.5)
        return 0.0;

    if (value >= 4.5)
        return 10.5; // Devolvemos bares, no PSI (150 PSI <> 10.34 bares)

    // Es una escala lineal así que podemos obtener el valor final despejando una regla de tres
    // Restamos al valor 0.5 para ajustar la escala a un valor entre 0-4
    value -= 0.5;
    // Ahora podemos calcular la ecuación
    value = 150 / (4 / value);
    // Pasamos a bares (dividimos por 14.5038) y devolvemos el valor
    return (value / PSI_TO_BAR);
}

float DataManager::GetEngineOilTemp(bool raw) {
    if (raw)
        return _engineOilTemp;

    // Convertimos a Cº
    return (float) _engineOilTemp * DALLAS_RAW_TO_CELSIUS;
}

float DataManager::GetGearboxOilTemp(bool raw) {
    if (raw)
        return _gearboxOilTemp;

    return (float) _gearboxOilTemp * DALLAS_RAW_TO_CELSIUS;
}

uint16_t DataManager::GetTPS(bool raw) {
    if (raw)
        return _tps;

    // El sensor envía una señal analógica y lineal de entre 0.35-0.65v @0% -> 4v @100% (según manual de taller)
    // Primero multiplicamos el valor por 0.0048828125‬ para obtener los voltios en el pin
    float value = (float) _tps * ANALOG_TO_VOLTS;
    // Después descartamos valores por debajo o por encima de los límites del sensor
    if (value <= _tpsMinValue)
        return 0;

    if (value >= _tpsMaxValue)
        return 100; // Devolvemos un porcentaje entre 0-100

    // Es una escala lineal así que podemos obtener el valor final despejando una regla de tres
    // Restamos al valor _tpsMinValue para ajustar la escala
    value -= _tpsMinValue;
    // Ahora podemos calcular la ecuación
    value = 100 / ((_tpsMaxValue - _tpsMinValue) / value);
    // Devolvemos un porcentaje
    return (uint16_t) value;
}

float DataManager::GetAFR(bool noAverage, bool raw) {
    if (raw)
        return _afr[_afrIndex - 1];

    uint32_t averageAfr = 0;
    if (noAverage) {
        averageAfr = _afr[_afrIndex - 1];
    } else {
        for (uint8_t i = 0; i < AVERAGE_AFR_COUNT_LIMIT; ++i) {
            averageAfr += _afr[i];
        }
        averageAfr = averageAfr / AVERAGE_AFR_COUNT_LIMIT;
    }

    // El controlador de la sonda Wideband envía una señal analógica y lineal de entre 0.5v @8.5:1 AFR -> 4.5v @18:1 AFR
    // Por debajo de 0.5v, el sensor aún está calentando, por encima de 4.5v, está dando algún tipo de error o fuera de escala.
    // Primero multiplicamos el valor por 0.0048828125‬ para obtener los voltios en el pin
    float value = (float) averageAfr * ANALOG_TO_VOLTS;
    // Después descartamos valores por debajo o por encima de los límites del sensor
    if (value <= 0.5)
        return -1.0; // El sensor todavía no está listo (está calentándose)

    if (value >= 4.5)
        return 18.1; // Devolvemos un valor un mínimo por encima de 18:1. Esto puede ser utilizado después en el controlador del TFT

    // Es una escala lineal así que podemos obtener el valor final despejando una regla de tres
    // Restamos al valor 0.5 para ajustar la escala de voltios a un valor entre 0-4
    value -= 0.5;
    // Ahora podemos calcular la ecuación, ajustanto la escala de AFR entre 8.5 y 18.0
    value = (9.5 / (4 / value)) + 8.5;
    value += 0.5; // Añadimos 0.5 de nuevo
    // Devolvemos el valor
    return value;
}

uint32_t DataManager::GetRPM(bool noAverage, bool raw) {
    // La función puede devolver las RPMs de tres formas:
    // - Raw, o valor sin procesar.
    // - Valor en ese instante, procesado para que sea legible.
    // - Valor medio entre las últimas x lecturas, donde x se define como AVERAGE_RPM_COUNT_LIMIT.
    // El último es un valor más suavizado y realista. Hay que tener en cuenta que estamos recuperando las 
    // RPM cientos de veces por segundo, por lo tanto la media entre los últimos 5 valores, por ejemplo,
    // sigue siendo sólamente la media durante unos pocos milisegundos.
    if (raw)
        return _rpm[_rpmIndex - 1];

    uint32_t averageRpm = 0;
    if (noAverage) {
        averageRpm = _rpm[_rpmIndex - 1];
    } else {
        for (uint8_t i = 0; i < AVERAGE_RPM_COUNT_LIMIT; ++i) {
            averageRpm += _rpm[i];
        }
        averageRpm = averageRpm / AVERAGE_RPM_COUNT_LIMIT;
    }

    if (averageRpm <= 0)
        return 0;

    // Nosotros guardamos el intervalo entre encendidos de la bobina en microsegundos
    // Para pasarlo a RPM, basta con multiplicar por 2 (cuatro cilindros -> 2 encendidos/rpm)
    // y dividir 60.000.000 (60 segundos en microsegundos) entre el valor de arriba.
    return 60000000 / (averageRpm * 2);
}

float DataManager::GetVoltage(bool raw) {
    if (raw)
        return _voltage;

    // El sensor de voltage es un simple divisor de tensión
    float vout = (_voltage * 5.0) / 1024.0;
    float vin = vout / (9850.0 / (100000.0 + 9850.0)); 
    if (vin < 0.09) {
        vin = 0.0; // Para filtrar ruidos
    }

    return vin;
}

bool DataManager::IsEngineOn() {
    if (GetRPM() > 500 || GetEngineOilPressure() >= 1.0)
        return true;

    return false;
}

/* 
 * A partir de aquí encontramos las funciones modificadas de la librería DallasTemperature.
 */
// Returns true if all bytes of scratchPad are '\0'
bool DataManager::Dallas_isAllZeros(const uint8_t * const scratchPad, const size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (scratchPad[i] != 0) {
            return false;
        }
    }

    return true;
}

bool DataManager::Dallas_isConnected(const uint8_t* deviceAddress, uint8_t* scratchPad, OneWire *wire) {
    bool b = Dallas_readScratchPad(deviceAddress, scratchPad, wire);
    return b && !Dallas_isAllZeros(scratchPad) && (wire->crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]);
}

bool DataManager::Dallas_readScratchPad(const uint8_t* deviceAddress, uint8_t* scratchPad, OneWire *wire) {
    // send the reset command and fail fast
    int b = wire->reset();
    if (b == 0)
        return false;

    wire->select(deviceAddress);
    wire->write(READSCRATCH);

    // Read all registers in a simple loop
    // byte 0: temperature LSB
    // byte 1: temperature MSB
    // byte 2: high alarm temp
    // byte 3: low alarm temp
    // byte 4: DS18S20: store for crc
    //         DS18B20 & DS1822: configuration register
    // byte 5: internal use & crc
    // byte 6: DS18S20: COUNT_REMAIN
    //         DS18B20 & DS1822: store for crc
    // byte 7: DS18S20: COUNT_PER_C
    //         DS18B20 & DS1822: store for crc
    // byte 8: SCRATCHPAD_CRC
    for (uint8_t i = 0; i < 9; i++) {
        scratchPad[i] = wire->read();
    }

    b = wire->reset();
    return (b == 1);
}

// reads scratchpad and returns fixed-point temperature, scaling factor 2^-7
int16_t DataManager::Dallas_calculateTemperature(const uint8_t* deviceAddress, uint8_t* scratchPad) {
    int16_t fpTemperature = (((int16_t) scratchPad[TEMP_MSB]) << 11)
            | (((int16_t) scratchPad[TEMP_LSB]) << 3);

    return fpTemperature;
}

// returns temperature in 1/128 degrees C or DEVICE_DISCONNECTED_RAW if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_RAW is defined in
// DallasTemperature.h. It is a large negative number outside the
// operating range of the device
int16_t DataManager::Dallas_getTemp(const uint8_t* deviceAddress, OneWire *wire) {
    uint8_t scratchPad[9];

    if (Dallas_isConnected(deviceAddress, scratchPad, wire))
        return Dallas_calculateTemperature(deviceAddress, scratchPad);
    return DEVICE_DISCONNECTED_RAW;
}

// returns the current resolution of the device, 9-12
// returns 0 if device not found
uint8_t DataManager::Dallas_getResolution(const uint8_t* deviceAddress, OneWire *wire) {
    uint8_t scratchPad[9];
    if (Dallas_isConnected(deviceAddress, scratchPad, wire)) {
        switch (scratchPad[CONFIGURATION]) {
            case TEMP_12_BIT:
                return 12;

            case TEMP_11_BIT:
                return 11;

            case TEMP_10_BIT:
                return 10;

            case TEMP_9_BIT:
                return 9;
        }
    }
    return 0;
}

// set resolution of a device to 9, 10, 11, or 12 bits
// if new resolution is out of range, 9 bits is used.
bool DataManager::Dallas_setResolution(const uint8_t* deviceAddress, OneWire *wire) {
    // Para este uso, 10 bits de resolución es más que suficiente
    uint8_t newResolution = DS18B20_RESOLUTION;

    // return when stored value == new value
    if (Dallas_getResolution(deviceAddress, wire) == newResolution)
        return true;

    uint8_t scratchPad[9];
    if (Dallas_isConnected(deviceAddress, scratchPad, wire)) {
        switch (newResolution) {
            case 12:
                scratchPad[CONFIGURATION] = TEMP_12_BIT;
                break;
            case 11:
                scratchPad[CONFIGURATION] = TEMP_11_BIT;
                break;
            case 10:
                scratchPad[CONFIGURATION] = TEMP_10_BIT;
                break;
            case 9:
            default:
                scratchPad[CONFIGURATION] = TEMP_9_BIT;
                break;
        }
        Dallas_writeScratchPad(deviceAddress, scratchPad, wire);

        return true;  // new value set
    }

    return false;
}

void DataManager::Dallas_writeScratchPad(const uint8_t* deviceAddress, const uint8_t* scratchPad, OneWire *wire) {
    wire->reset();
    wire->select(deviceAddress);
    wire->write(WRITESCRATCH);
    wire->write(scratchPad[HIGH_ALARM_TEMP]); // high alarm temp
    wire->write(scratchPad[LOW_ALARM_TEMP]); // low alarm temp
    wire->write(scratchPad[CONFIGURATION]);
    wire->reset();

    // save the newly written values to eeprom
    wire->select(deviceAddress);
    wire->write(COPYSCRATCH, false); // En este caso no usamos las sondas en parasite mode, por lo tanto parasite = false
    // Este delay se puede dejar, ya que estas funciones sólo se llaman una vez al inicializar la centralita
    delay(20); // <--- added 20ms delay to allow 10ms long EEPROM write operation (as specified by datasheet)

    wire->reset();
}
