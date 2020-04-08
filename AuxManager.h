/*
 * AuxManager
 * 
 * Esta clase se encarga de activar los mapas de emergencia en caso de que algo falle, 
 * y controlar el corte de inyección por encima de 8000 rpm cambiando entre mapas. También
 * gestiona funciones secundarias o menos relevantes como la alimentación de la controladora
 * de la sonda wideband, señal emulada de la sonda lambda, control de los botones y mapas, etc.
 */

#ifndef __AUX_MANAGER__H__
#define __AUX_MANAGER__H__

// INPUTS
#define INPUT_CONTROL_BUTTON           10       // Input para la señal del botón de control general (0-5v digital)
#define INPUT_MAPS_SWITCH_BUTTON       8        // Input para la señal del botón de cambio de mapas (0-5v digital)

// OUTPUTS
#define OUTPUT_AFR_GAUGE_VCC           6       // Pin para controlar la alimentación al controlador (y por lo tanto a la sonda) wideband
#define OUTPUT_LAMBDA                  46      // Pin para controlar la señal de salida del emulador de sondas lambda (0-5v digital)
#define OUTPUT_MAP_SWITCH              52      // Pin para manejar los mapas de la ECU

// Intervalos
#define INTERVAL_SWITCH_TFT_MODE       2000    // Mantener el botón de control pulsado 2 segundos para cambiar el modo del TFT
#define INTERVAL_COOLDOWN              2000    // Tras recibir el comando de cambiar el modo, esperar 2 segundos hasta recibir un nuevo comando
#define MAP_SWITCH_COOLDOWN            1000    // 1 segundo como mínimo entre cambio de mapas, para evitar que por interferencias al darle al botón la ECU OEM se ponga en limp mode
#define RPM_LIMITER_HYSTERESIS         250     // Tiempo (en ms) que tarda el manager en reactivar los mapas de carreras después de pasar a los de calle para "simular" un corte de inyección

enum ECUMaps {
    ECU_MAP_NORMAL    = 1, // Mapa normal, para uso en calle
    ECU_MAP_RACE      = 2, // Mapa para pista, más pensado para gasolina de 98 octanos, ganas 500 RPM antes del corte
    ECU_MAP_EMERGENCY = 3  // Modo emergencia, mapa normal + levas en el lóbulo de ALTAS (si, el de altas, no el de bajas)
};

enum Commands {
    COMMAND_NONE              = 0,
    COMMAND_CHANGE_BRIGHTNESS = 1,
    COMMAND_CHANGE_SCREEN     = 2
};

class AuxManager {
    DataManager *_dataManager; // Puntero al DataManager, de donde recuperaremos los datos
    DataMonitor *_dataMonitor; // Puntero al DataMonitor, para recuperar el estado de los parámetros

    ECUMaps _currentECUMap;
    bool _afrGaugeOn;
    Commands _nextCommand;
    uint32_t _controlButtonTimer;
    uint32_t _controlButtonCooldownTimer;
    uint32_t _mapSwitchCooldownTimer;
    bool _isControlButtonPressed;
    bool _isControlButtonInCooldown;
    bool _isMapSwitchInCooldown;
    bool _isLimiterEnabled;

    void SwitchAFRGaugePower(bool on);    // Para activar/desactivar el relé que da corriente al controlador de la sonda wideband
    void SwitchMaps(ECUMaps map);         // Para cambiar entre los mapas de la ECU
    void SetLambdaEmulation(bool lean);   // Para cambiar entre rico/pobre en el emulador de sonda lambda
  public:
    // Constructor, en este caso sólo inicializa las variables privadas de la clase
    AuxManager();

    // Función de inicialización, aquí es donde realmente empieza a funcionar este manager, en cuanto el DataManager y el DataMonitor estén operativos
    void Initialize(DataManager *dataManager, DataMonitor *dataMonitor);
    void Update(uint32_t diff);

    Commands GetNextCommand();

    ECUMaps GetCurrentECUMap();

    bool IsLimiterEnabled();
};

#endif
