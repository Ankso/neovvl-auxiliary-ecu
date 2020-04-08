/*
 * DataManager
 * 
 * Esta clase se encarga de recuperar los datos desde las diferentes entradas y procesarlos
 * para obetener valores fácilmente manipulables y dejarlos a disposición de los otros Managers.
 *
 * La clase incluye una implementación asíncrona de las funciones básicas de la librería
 * DallasTemperature, para evitar utilizar delay(); ya que un software como este con delays es inviable.
 */

#ifndef __DATA_MANAGER__H__
#define __DATA_MAANGER__H__

#define DS18B20_RESOLUTION      10             // 10 Bits
#define ANALOG_TO_VOLTS         0.0048828125   // Para pasar 10 bits analógicos a voltios en el pin
#define PSI_TO_BAR              14.5038        // Para pasar PSI a bares
#define DALLAS_RAW_TO_CELSIUS   0.0078125      // Para pasar los valores devueltos por los sensores DS18B20 a Cº
#define RPM_INPUT_HIGH_VALUE    200            // Para el input de las RPM, consideraremos el pin analógico como HIGH a partir de este valor
#define RPM_INPUT_INTERVAL_MAX  500000         // Máximo valor posible para el intervalo entre encendidos de bobina, en microsegundos. 500.000 (0,5 segundos) = 60 RPM.
#define RPM_ANALOG_MIN_VALUE    610            // Mínimo valor de voltaje en el pin de la señal de RPM antes de pasar a la comprobación secundaria.
#define MAX_RPM_INPUT_LOW_VOLTAGE_ERRORS 10
#define AVERAGE_RPM_COUNT_LIMIT 5              // Número de comprobaciones de RPM que se guardan para devolver una media entre todos los valores.
#define AVERAGE_AFR_COUNT_LIMIT 5              // Número de comprobaciones de AFR que se guardan para devolver una media entre todos los valores.
// TIMERS E INTERVALS
#define SECONDARY_DATA_INTERVAL 100            // 0.1 segundos
#define TEMP_DATA_INTERVAL      1000           // 1 segundo
#define DS18B20_UPDATE_INTERVAL 188            // 94 milisegundos a 9 bits de resolución (0.5º), 188 a 10 bits (0.25º)
#define STARTUP_CHECK_INTERVAL  1500           // 1.5 segundos

// INPUTS (Nº de pin en el Arduino)
#define INPUT_ENG_OIL_PRESSURE  A1             // Input para el sensor de presión del aceite del motor (0-5v analógica)
#define INPUT_ENG_OIL_TEMP      30             // Input para el sensor de temperatura del aceite del motor (DS18B20 - OneWire bus)
#define INPUT_GEARBOX_OIL_TEMP  31             // Input para el sensor de temperatura del aceite de la caja de cambios (DS18B20 - OneWire bus)
#define INPUT_RPM_SIGNAL        3              // Input para la señal de RPM proveniente de la ECU de origen (digital)
#define INPUT_RPM_SIGNAL_AUX    A5             // Input para la señal de RPM proveniente de la ECU de origen, se puede usar como backup si por alguna razón el voltaje en el pin digital es demasiado bajo (analógico)
#define INPUT_TPS               A2             // Input para el sensor de posición de la mariposa (0-5v analógica)
#define INPUT_AFR               A3             // Input para la señal de AFR proveniente de la sonda Wideband (0-5v analógica)
#define INPUT_VOLTAGE           A0             // Input para la señal de voltaje directo de la batería/sistema de carga (0-5v analógica)

// Definiciones para las funciones migradas de la librería DallasTemperature
#define DEVICE_DISCONNECTED_RAW -7040
#define READSCRATCH              0xBE          // Read EEPROM
// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8
// OneWire commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition
// Device resolution
#define TEMP_9_BIT      0x1F  //  9 bit
#define TEMP_10_BIT     0x3F  // 10 bit
#define TEMP_11_BIT     0x5F  // 11 bit
#define TEMP_12_BIT     0x7F  // 12 bit
// Fin de las definiciones de DallasTemperature

class DataManager {
    // Variables para almacenar los datos de manera interna y sin procesar
    int16_t _engineOilTemp;
    int16_t _gearboxOilTemp;
    uint16_t _engineOilPressure;
    uint16_t _tps;
    float _tpsMinValue; // Valor mínimo del TPS, se carga al inicializar el DataManager (se supone que al dar contacto no se tiene el acelerador pisado)
    float _tpsMaxValue; // Valor máximo del TPS, por defecto 3.5v. El DataManager lo ajusta automáticamente si detecta un valor mayor
    uint16_t _afr[AVERAGE_AFR_COUNT_LIMIT];
    uint32_t _rpm[AVERAGE_RPM_COUNT_LIMIT];
    uint16_t _voltage;

    // Índices de control para obtener valores medios de RPM y AFR
    uint8_t _rpmIndex;
    uint8_t _afrIndex;

    // Variables para el control de las RPMs
    uint32_t _micros;           // Microsegundos desde el inicio del Arduino.
    uint32_t _lastMicros;       // Microsegundos desde el inicio del Arduino en los que se produjo el último encendido de la bobina.
    uint32_t _rpmInterval;      // Microsegundos entre encendidos de la bobina según la ECU del coche.
    uint8_t _rpmLowVoltageInputCount;
    bool _rpmTriggerCooldown;
    bool _startupCheckExecuted;
    bool _selectAuxRPMInput;
    
    // Timers internos para recuperar los datos de los sensores con diferente prioridad
    // Los datos de alta prioridad (RPMs, presión de aceite y AFR) se recuperan constantemente
    uint32_t _secondaryDataTimer; // Se consideran datos secundarios el AFR, TPS y voltaje (prioridad media)
    uint32_t _tempDataTimer;      // Intervalo para recuperar los datos de las sondas DS18B20 (baja prioridad)
    uint32_t _startupCheckTimer;  // Este timer sólo se ejecuta una vez al arrancar la centralita, para dar tiempo a la ECU del coche a inicializarse

    // Timers internos para gestionar las sondas DS18B20 de forma asíncrona
    uint32_t _engOilTempTimer;
    uint32_t _gbOilTempTimer;

    // Objetos OneWire para recuperar la información de las sondas DS18B20
    OneWire _engOilTempWire;
    OneWire _gbOilTempWire;

    // Array para guardar los identificadores de las dos sondas DS18B20
    uint8_t _engOilTempAddress[8];
    uint8_t _gbOilTempAddress[8];
    
    // Variables para guardar el estado de los timers para las sondas DS18B20
    bool _engOilTempRequested;
    bool _gbOilTempRequested;

    // Funciones internas para recuperar los valores directamente de los inputs
    void RetrieveEngineOilPressure();
    void RetrieveEngineOilTemp(bool requestCompleted);
    void RetrieveGearboxOilTemp(bool requestCompleted);
    void RetrieveTPS();
    void RetrieveAFR();
    void RetrieveVoltage();
    void ExecuteStartupCheck();

    // Funciones importadas de la librería DallasTemperature, para una implementación asíncrona
    bool Dallas_isAllZeros(const uint8_t * const scratchPad, const size_t length = 9);
    bool Dallas_isConnected(const uint8_t* deviceAddress, uint8_t* scratchPad, OneWire *wire);
    bool Dallas_readScratchPad(const uint8_t* deviceAddress, uint8_t* scratchPad, OneWire *wire);
    int16_t Dallas_calculateTemperature(const uint8_t* deviceAddress, uint8_t* scratchPad);
    int16_t Dallas_getTemp(const uint8_t* deviceAddress, OneWire *wire);
    uint8_t Dallas_getResolution(const uint8_t* deviceAddress, OneWire *wire);
    bool Dallas_setResolution(const uint8_t* deviceAddress, OneWire *wire);
    void Dallas_writeScratchPad(const uint8_t* deviceAddress, const uint8_t* scratchPad, OneWire *wire);

  public:
    // Constructor para inicializar la clase
    DataManager();

    // Función principal que gestiona los timers para recuperar la información de los sensores
    // Se llama en cada iteración de la función loop()
    void Update(uint32_t diff);

    // Función llamada desde el interrupt para calcular las RPM
    void CalculateRPM(uint32_t currentMicros);
    // Función auxiliar para obtener las RPM
    void RetrieveRPM(uint32_t microDiff);

    // Functiones públicas para recuperar la información de los sensores
    // Por defecto devuelven un valor fácilmente legible (Cº, bares, porcentaje, etc...)
    // Todas las funciones pueden devolver el valor raw (sin procesar) de forma opcional
    float GetEngineOilPressure(bool raw = false);
    float GetEngineOilTemp(bool raw = false);
    float GetGearboxOilTemp(bool raw = false);
    uint16_t GetTPS(bool raw = false);
    float GetAFR(bool noAverage = false, bool raw = false);
    uint32_t GetRPM(bool noAverage = false, bool raw = false);
    float GetVoltage(bool raw = false);
    bool IsEngineOn();
};

#endif
