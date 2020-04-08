/*
 * NeoVVLManager
 * 
 * Esta clase tiene como objetivo gestionar los solenoides de activación del sistema NeoVVL.
 * Controla de forma independiente las levas de escape y admisión.
 */

#ifndef __NEOVVL_MANAGER__H__
#define __NEOVVL_MANAGER__H__

// RPMs a las que cambiar las levas
#define INTAKE_RPM_SWITCHOVER_NORMAL   5600 // En el modo normal, cambiamos las levas de la admisión algo antes que las de escape
#define EXHAUST_RPM_SWITCHOVER_NORMAL  5600 // para suavizar el momento de cambio y que no sea demasiado brusco.
#define INTAKE_RPM_SWITCHOVER_RACE     5500 // Con los árboles de levas del SR16VE N1, hay que cambiar las levas de escape a
#define EXHAUST_RPM_SWITCHOVER_RACE    5700 // RPMs más cercanas a las 6000 que a las 5000, si no se perderá potencia

// Timer, o cooldown, entre cambios en las levas, para evitar cambios demasiado rápidos, por ejemplo al mantener las RPM en el límite de activación.
#define CAMS_SWITCHOVER_COOLDOWN       500  // 500 milisegundos

// Pines necesarios para controlar los solenoides
#define OUTPUT_INTAKE_SOLENOID  4    // Pin para controlar el relé del solenoide de las levas de admisión (0-5v digital)
#define OUTPUT_EXHAUST_SOLENOID 2    // Pin para controlar el relé del solenoide de las levas de escape (0-5v digital)

// Posibles estados de las levas
#define CAM_STATUS_ENABLED      1    // Leva de altas activadas
#define CAM_STATUS_DISABLED     2    // Leva de altas desactivadas
#define CAM_STATUS_ENGINE_OFF   3    // Motor apagado
#define CAM_STATUS_ERROR        -1   // Hay algún problema en el sistema

class NeoVVLManager {
    DataManager *_dataManager;    // Puntero al DataManager, de donde recuperaremos los datos
    AuxManager *_auxManager;      // Puntero al AuxManager, para obtener los mapas activos en la ECU
    EEPROMManager *_eepromManager;

    int16_t _intakeSwitchNormal;  // Variables que guardan los puntos en los que cambian las levas si se cargan desde la EEPROM
    int16_t _exhaustSwitchNormal; // Si no, se utlizan los valores de arriba.
    int16_t _intakeSwitchRace;
    int16_t _exhaustSwitchRace;

    bool _isIntakeEnabled;        // Variables de control para saber si las levas de altas están o no activadas
    bool _isExhaustEnabled;
    bool _isIntakeInCooldown;     // Variables para controlar el cooldown entre cambios en las levas
    bool _isExhaustInCooldown;

    uint32_t _intakeCamCooldownTimer;
    uint32_t _exhaustCamCooldownTimer;

  public:
    // Constructor, en este caso sólo inicializa las variables privadas de la clase
    NeoVVLManager();

    // Función de inicialización, aquí es donde realmente empieza a funcionar este manager, en cuanto el DataManager y el AuxManager estén operativos
    void Initialize(DataManager *dataManager, AuxManager *auxManager, EEPROMManager *eepromManager);
    // Función que controla la activación/desactivación de las levas en función de las RPMs
    void Update(uint32_t diff);
    // Funciones para cambiar las levas
    void SwitchIntakeCam(bool on);
    void SwitchExhaustCam(bool on);
    // Funciones para recuperar el estado de las levas de forma externa
    int8_t GetIntakeCamStatus();
    int8_t GetExhaustCamStatus();
    // Funciones para cargar valores dinámicos de la EEPROM
    void LoadCamsSwitchPointsFromEEPROM();
};

#endif
