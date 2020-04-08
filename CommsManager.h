/*
 * CommsManager
 * 
 * Esta clase se encarga de gestionar la conexión I2C y la transmisión de datos al Arduino
 * que controla la pantalla TFT.
 */

#ifndef __COMMS_MANAGER__H__
#define __COMMS_MANAGER__H__

#define INTERVAL_BETWEEN_PACKETS       100     // Enviamos 10 paquetes de datos por segundo

// Bus de datos TWI (I2C)
#define I2C_SDA                        20      // Pin SDA para conectarse al Arduino Due que controla una pantalla TFT para monitorear los datos
#define I2C_SDL                        21      // Pin SDL (...)

#define TFT_CONTROLLER_DEVICE_ID       1       // ID del arduino que controla la pantalla TFT en el bus I2C

#define BAUD_RATE                      250000

enum NeoVVLStatus {
    NEOVVL_STATUS_BOTH_OFF   = 0,
    NEOVVL_STATUS_INTAKE_ON  = 1,
    NEOVVL_STATUS_EXHAUST_ON = 2,
    NEOVVL_STATUS_BOTH_ON    = 3
};

struct Packet {
    uint16_t rpms;              // 2 bytes
    float engOilPress;          // 4 bytes
    float engOilTemp;           // 4 bytes
    float gbOilTemp;            // 4 bytes
    float afr;                  // 4 bytes
    float voltage;              // 4 bytes
    uint8_t tps;                // 1 byte, conversión de uint16_t a uint8_t (la señal del TPS sólo varía entre 0-100)
    uint8_t engOilPressStatus;  // 1 byte
    uint8_t engOilTempStatus;   // 1 byte
    uint8_t gbOilTempStatus;    // 1 byte
    uint8_t afrStatus;          // 1 byte
    uint8_t voltageStatus;      // 1 byte
    uint8_t tpsStatus;          // 1 byte
    uint8_t selectedECUMap;     // 1 byte
    uint8_t neoVVLStatus;       // 1 byte
    uint8_t command;            // 1 byte
    //-------------------------------------
    // TOTAL                      32 bytes
};

class CommsManager {
    EEPROMManager *_eepromManager; // Puntero al EEPROMManager, para cargar/grabar datos en la memoria EEPROM de Arduino
    DataManager *_dataManager; // Puntero al DataManager, de donde recuperaremos los datos
    DataMonitor *_dataMonitor; // Puntero al DataMonitor, para recuperar el estado de los parámetros
    NeoVVLManager *_neoVVLManager; // Puntero al la clase que controla las levas, para obtener su estado
    AuxManager *_auxManager; // Puntero al AuxManager, para obtener lo mapas de la ECU

    Packet _packet;
    uint32_t _intervalBetweenPacketsTimer;

    void SendPacket();

  public:
    // Constructor, en este caso sólo inicializa las variables privadas de la clase
    CommsManager();

    // Función de inicialización, aquí es donde realmente empieza a funcionar este manager, en cuanto el resto de managers estén operativas
    void Initialize(EEPROMManager *eepromManager, DataManager *dataManager, DataMonitor *dataMonitor, AuxManager *auxManager, NeoVVLManager *neoVVLManager);
    // Función que controla el intervalo de envío de los paquetes
    void Update(uint32_t diff);
};

#endif
