#ifndef RS485_WIND_H
#define RS485_WIND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/** Init RS-485 vind-sensor modul
 *  - huart_rs485 : UART til MAX3485 (fx &huart3 @ 9600 8N1)
 *  - de_port/pin : GPIO for DE (DE og /RE bundet sammen). Aktiv HIGH = TX, LOW = RX
 */
void WindRS485_Init(UART_HandleTypeDef *huart_rs485,
                    GPIO_TypeDef *de_port, uint16_t de_pin);

/** Poll én gang: sender forespørgsel og forsøger at læse ét svar.
 *  Returnerer true ved gyldigt svar og udfylder speed_ms og dir_deg.
 */
bool WindRS485_Get(float *speed_ms, float *dir_deg);

/** (Valgfrit) Sæt tidsouts:
 *  - total_ms   : max samlet RX-vindue efter en forespørgsel
 *  - perbyte_ms : timeout for HAL_UART_Receive for hver byte
 */
void WindRS485_SetTimeouts(uint32_t total_ms, uint32_t perbyte_ms);

#ifdef __cplusplus
}
#endif

#endif /* RS485_WIND_H */
