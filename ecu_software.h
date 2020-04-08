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

#ifndef __ECU_SOFTWARE__H__
#define __ECU_SOFTWARE__H__

// MODO DEBUG
#define DEBUG                      false       // Activa el modo debug. El puerto serie se utilizará para monitorear todos los parámetros
                                               // ATENCIÓN: El modo debug hará el software MUCHO más lento! Desactivar para uso real.

// EEPROM ON/OFF
#define ENABLE_EEPROM_USAGE        true        // Activa el uso de la EEPROM (desactivar durante pruebas para ahorrar usos)

// TIMERS E INTERVALS
// Los timers están definidos en sus Managers

#endif
