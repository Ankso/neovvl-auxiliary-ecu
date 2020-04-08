/*
 * DataMonitor
 *
 * Esta clase monitorea los datos recopilados por el DataManager para detectar posibles problemas en el motor
 * (como temperaturas o presiones fuera de rango), inconsistencia en los valores de los sensores, etc.
 */

#ifndef __DATA_MONITOR__H__
#define __DATA_MONITOR__H__

#define DS18B20_ERROR_TEMP          -55.0 // Valor devuelto por las funciones del DataManager si hay algún problema con la conexión de los sensores
// Para el controlador de señal de RPMs
#define MAX_RPM_DIFF_BETWEEN_CHECKS 5000  // Máxima variación permitida de RPMs en x tiempo definido a continuación
#define RPM_INPUT_CHECK_INTERVAL    100   // 0.1 segundos
#define RPM_FAILURES_RESET_TIMER    5000  // 5 segundos entre reseteo de errores en el algoritmo de RPMs, para evitar falsos positivos
#define MIN_OIL_PRESS_DANGER_TIMER  200   // 0.2 segundos, tiempo suficiente para asegurar que al menos un paquete con el estado DANGER será enviado al TFT
#define MAX_RPM_SIGNAL_ERRORS       10    // 10 fallos detectados en la integridad de la señal de RPMs
#define EMERGENCY_REV_LIMITER       8500  // 8800 son las máximas RPMs permitidas, si se pasa de vueltas, pasamos a los mapas de calle para activar el limitador.
// Temperaturas del aceite del motor
#define ENGINE_OIL_COLD_TEMP_LIMIT  70.0  // 70 Cº
#define ENGINE_OIL_TEMP_WARNING     110.0 // 110 Cº
#define ENGINE_OIL_TEMP_DANGER      120.0 // 120 Cº
// Temperaturas del aceite de la caja de cambios
#define GEARBOX_OIL_COLD_TEMP_LIMIT 70.0  // 70 Cº
#define GEARBOX_OIL_TEMP_WARNING    100.0 // 100 Cº
#define GEARBOX_OIL_TEMP_DANGER     110.0 // 110 Cº
// AFRs
#define AFR_RICH_WARNING            10.0  // 10:1 AFR
#define AFR_RICH_DANGER             9.0   // 9:1 AFR
#define AFR_LEAN_WARNING            15.2  // 15.2:1 AFR
#define AFR_LEAN_DANGER             16.0  // 16:1 AFR
// Voltajes
#define VOLTAGE_LOW_WARNING         11.8  // 11.8 voltios
#define VOLTAGE_LOW_DANGER          11.0  // 11 voltios
#define VOLTAGE_HIGH_WARNING        15.0  // 15 voltios
#define VOLTAGE_HIGH_DANGER         16.0  // 16 voltios
// Presiones de aceite
#define ENGINE_OIL_PRESS_MIN        1.0   // 1.0 bares hasta 80º
#define ENGINE_OIL_PRESS_MIN_HOT    0.6   // 0.6 bares a partir de 80º
#define ENGINE_OIL_PRESS_RPM_CHECK  4000  // 4000 RPMs
#define ENGINE_OIL_PRESS_MIN_RPMS   3.0   // 3.0 bares a partir de 4000 RPMs mínimo

// Posibles estados de los parámetros
enum ParameterStatus {
    STATUS_OK      = 1, // Todo correcto
    STATUS_WARNING = 2, // El parámetro se está acercando valores peligrosos, hay que vigilarlo
    STATUS_DANGER  = 3, // El parámetro está fuera de valores seguros.
    STATUS_COLD    = 4, // En caso de que los aceites todavía no estén en temperatura de trabajo o la sonda wideband esté calentándose
    STATUS_ERROR   = 5, // Hay algún error con el sensor (por ejemplo se ha perdido la conexión)
};

class DataMonitor {
    DataManager *_dataManager; // Puntero al DataManager, de donde recuperaremos los datos

    // Almacenamos el estado de cada parámetro
    ParameterStatus _engOilPressureStatus;
    ParameterStatus _engOilTempStatus;
    ParameterStatus _gbOilTempStatus;
    ParameterStatus _afrStatus;
    ParameterStatus _tpsStatus;
    ParameterStatus _rpmStatus;
    ParameterStatus _voltageStatus;
    // Con estas dos variables controlaremos la entrada de datos de las RPM
    // Si varían de forma abrupta demasiado, por ejemplo más de 3000 RPM cada
    // décima de segundo, es muy probable que la señal esté corrupta y recibiendo
    // interferencias. Es muy importante detectar esto por si falla algo, no cargarse
    // el tren de vávulas jugando con el NeoVVL!!
    uint16_t _lastRPMValue;
    uint32_t _rpmInputCheckTimer;
    uint32_t _rpmFailureResetTimer;
    uint8_t _rpmErrorsCount;
    // Timer para mantener el status de la presión de aceite durante un mínimo de ms para asegurarnos de que al menos se manda la señal al TFT
    uint32_t _engineOilPressTimer;
    bool _engineOilPressStatusCooldown;

  public:
    // Constructor, en este caso sólo inicializa las variables privadas de la clase
    DataMonitor();

    // Función de inicialización, como el resto de Managers, necesita esperar a que el DataManager esté operativo
    void Initialize(DataManager *dataManager);
    // En el Update() se actualiza el estado de todos los parámetros.
    // El DataMonitor no toma ninguna acción específica si se alcanzan valores peligrosos, sin embargo el AuxManager utiliza
    // estos valores para decidir si permanecer en el mapa de alto rendimiento de la ECU o volver al mapa de calle.
    // Además el estado de los valores se emite vía el CommsManager al TFT para que el piloto pueda ver el estado del motor mejor.
    void Update(uint32_t diff);

    // Funciones para devolver el estado de cada parámetro monitoreado. Son simples "Getters", así que ya hacemos la declaración inline.
    ParameterStatus GetEngineOilPressureStatus() { return _engOilPressureStatus; };
    ParameterStatus GetEngineOilTempStatus() { return _engOilTempStatus; };
    ParameterStatus GetGearboxOilTempStatus() { return _gbOilTempStatus; };
    ParameterStatus GetAFRStatus() { return _afrStatus; };
    ParameterStatus GetTPSStatus() { return _tpsStatus; };
    ParameterStatus GetRPMStatus() { return _rpmStatus; };
    ParameterStatus GetVoltageStatus() { return _voltageStatus; };
    uint8_t GetTotalRPMInputErrors() { return _rpmErrorsCount; };
};

#endif
