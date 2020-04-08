/*
 * Software de control para swap a motores NeoVVL.
 * 
 * Este software utiliza un Arduino Mega 2560 para:
 *   - Controlar los solenoides del sistema NeoVVL.
 *   - Monitorear las RPMs del motor.
 *   - Monitorear la presión y temperatura del aceite de motor.
 *   - Monitorear la temperatura de la caja de cambios.
 *   - Monitorear la posición de la mariposa (TPS)
 *   - Monitorear la mezcla de aire/combustible (AFR) con la señal de una sonda Wideband externa (en mi caso una AEM UEGO)
 *   - Monitorear el voltaje de la batería/alternador.
 *   - Controlar el encendido de la propia sonda Wideband, para evitar que se encienda con el motor apagado y alagar su vida útil.
 *   - Emitir una señal tipo 0-1v para emular una sonda lambda (para conectarla después a la ECU de origen si es necesario).
 *   - Controlar los mapas seleccionados en la ECU del motor si esta está equipada con una daughterboard debidamente modificada/configurada.
 *   - Emitir todos estos parámetros vía serial a otro Arduino para su visualización/procesamiento (por ejemplo para visualizarlos en una pantalla TFT, datalogs, etc)
 */

#include <OneWire.h>
#include "ecu_software.h"
#include "EEPROMManager.h"
#include "DataManager.h"
#include "DataMonitor.h"
#include "AuxManager.h"
#include "NeoVVLManager.h"
#include "CommsManager.h"

EEPROMManager eepromManager;
DataManager dataManager;
DataMonitor dataMonitor;
AuxManager auxManager;
NeoVVLManager neoVVLManager;
CommsManager commsManager;

unsigned long time;
unsigned long microseconds;
bool isFailSafeModeEnabled;
bool isDebugEnabled;
uint32_t debugTimer;

void setup()
{
    debugTimer = 0;
    isDebugEnabled = false;
    if (DEBUG) {
        isDebugEnabled = true;
        Serial.begin(9600);
        Serial.println("---- DEBUG MODE ENABLED - Serial communication started ----");
    }

    // Configuramos un interrupt que se ejecutará cada vez que la ECU mande una señal de encendido a la bobina.
    attachInterrupt(digitalPinToInterrupt(INPUT_RPM_SIGNAL), IgnitionEvent, RISING);
    // Recordemos que el DataManager no tiene función Initialize(), es el primero y no depende de otros Managers,
    // por lo tanto el constructor inicializa todo lo necesario.
    dataMonitor.Initialize(&dataManager);
    auxManager.Initialize(&dataManager, &dataMonitor);
    neoVVLManager.Initialize(&dataManager, &auxManager, &eepromManager);
    commsManager.Initialize(&eepromManager, &dataManager, &dataMonitor, &auxManager, &neoVVLManager);

    time = millis();

    /* 
     * Ahora comprobamos si nuestra centralita se está iniciado en modo Fail Safe.
     *
     * Éste se activa al pulsar el botón de control mientras se enciende la centralita (al darle a la llave básicamente).
     *
     * En el modo Fail Safe, la centralita desactiva el NeoVVLManager, es decir, las levas no se cambian y permanecen
     * constantemente en bajas. Es un modo pensado para, si falla algo, tener la posibilidad de seguir conduciendo el coche
     * de forma normal, pero ojo, no se deben superar las 7000-7500 RPMs en este modo, o el tren de válvulas puede resultar dañado.
     * 
     * Según los rumores que circulan por internet, con los muelles de serie, es más probable dañar el tren de válvulas 
     * a altas vueltas con las levas de bajas que con las de altas, ya que a pesar de que las segundas tienen mucho más lift, 
     * también tienen mucha más duración lo que genera un perfil del árbol de levas más suave. Digamos que las levas de altas
     * equivalen a empujar la válvula más abajo pero poco a poco, mientras que el perfil de bajas se parece más a un martillazo
     * rápido en el que las vávulas no bajan tanto, pero lo hacen de forma mucho más brusca.
     * 
     * No se hasta que punto esto es cierto, lo que si se es que con las levas de altas y muelles de serie, puedes
     * superar las 9000 RPM y no explota :) jiji
     */
    if (digitalRead(INPUT_CONTROL_BUTTON) == HIGH) {
        isFailSafeModeEnabled = true;
        isDebugEnabled = true; // Activamos también el modo Debug, de todas formas al no funcionar el NeoVVL los lagazos del Serial nos dan un poco igual.
    } else {
        isFailSafeModeEnabled = false;
    }
    pinMode(13, OUTPUT);
}

void loop()
{
    uint32_t diff = millis() - time;
    uint32_t microsDiff = micros() - microseconds;
    time = millis();
    microseconds = micros();
    // Actualizamos antes de nada el DataManager para asegurarnos que los datos están actualizados
    // a la hora de llamar al resto de Managers
    dataManager.Update(diff);
    // Justo después de actualizar los valores de los sensores, llamamos al DataMonitor para que los compruebe
    dataMonitor.Update(diff);
    // Ahora actualizamos el manager de funciones auxiliares
    auxManager.Update(diff);
    // Ajustamos el estado de los árboles de levas, si no estamos en modo fail safe
    if (!isFailSafeModeEnabled)
        neoVVLManager.Update(diff);
    // Y por último nos comunicamos con el Arduino que controla el TFT
    commsManager.Update(diff);

    // Modo debug, para pasar parámetros a un ordenador conectado al Arduino y hacer pruebas/verificaciones
    // OJO, este modo debug tiene que ser activado manualmente en el código o activando la centralita en modo
    // fail safe, porque causa latencia entre ciclos y puede hacer que la centralita vaya a saltos.
    if (isDebugEnabled) {
        if (debugTimer >= 1000) {
            Serial.print("Diff: ");
            Serial.println(diff);
            //Serial.print("Diff (uS): ");
            //Serial.println(microsDiff);
            // Aquí podemos llamar a las funciones de los diferentes managers para analizar los datos
            // Por ejemplo:
            //Serial.print("Engine: ");
            //Serial.println(dataManager.IsEngineOn() ? "On" : "Off");
            //Serial.print("RPM: ");
            //Serial.println(dataManager.GetRPM(true));
            //Serial.print("TPS: ");
            //Serial.println(dataManager.GetTPS());
            //Serial.print("Oil press.: ");
            //Serial.println(dataManager.GetEngineOilPressure());
            //Serial.print("Voltage: ");
            //Serial.println(dataManager.GetVoltage());
            //Serial.print("Eng. oil temp.: ");
            //Serial.println(dataManager.GetEngineOilTemp());
            //Serial.print("Gbox oil temp.: ");
            //Serial.println(dataManager.GetGearboxOilTemp());
            //Serial.print("Next command:");
            //Serial.println(auxManager.GetNextCommand());
            //Serial.print("Analog pin status:");
            //Serial.println(analogRead(INPUT_TPS));
            //Serial.print("Digital pin status:");
            //Serial.println(digitalRead(INPUT_RPM_SIGNAL));
            debugTimer = 0;
        } else {
            debugTimer += diff;
        }
    }
}

void IgnitionEvent() {
    dataManager.CalculateRPM(micros());
}
