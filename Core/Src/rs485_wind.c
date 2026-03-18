#include "rs485_wind.h"
#include <string.h>

/* Gemte handles/pins fra init */
static UART_HandleTypeDef *s_rs485  = NULL;
static GPIO_TypeDef       *s_de_port = NULL;
static uint16_t            s_de_pin  = 0;

/* Modbus-RTU forespørgsel (samme som din Python/kørende kode) */
static const uint8_t REQ[8] = {0x01,0x03,0x00,0x00,0x00,0x04,0x44,0x09};
#define RESP_LEN 13

/* Tidsouts (kan justeres med WindRS485_SetTimeouts) */
static uint32_t s_total_to_ms   = 80;   // samlet RX-vindue (hurtigere end 120 ms)
static uint32_t s_perbyte_to_ms = 1;    // pr. byte (giver responsivitet)

/* DE helpers */
static inline void DE_RX(void){ HAL_GPIO_WritePin(s_de_port, s_de_pin, GPIO_PIN_RESET); }
static inline void DE_TX(void){ HAL_GPIO_WritePin(s_de_port, s_de_pin, GPIO_PIN_SET);   }

/* CRC16 Modbus (poly 0xA001, init 0xFFFF) */
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len) {
  uint16_t c = 0xFFFF;
  for (uint16_t i=0;i<len;i++){
    c ^= buf[i];
    for (uint8_t b=0;b<8;b++){
      c = (c & 1) ? (uint16_t)((c >> 1) ^ 0xA001) : (uint16_t)(c >> 1);
    }
  }
  return c;
}

/* Læs præcis RESP_LEN bytes inden for s_total_to_ms (1 byte ad gangen) */
static bool rs485_exchange(uint8_t *resp, uint32_t *got)
{
  // Flush evt. gammel RX
  uint8_t d;
  while (HAL_UART_Receive(s_rs485, &d, 1, 0) == HAL_OK) {}

  // Send forespørgsel
  DE_TX();
  if (HAL_UART_Transmit(s_rs485, (uint8_t*)REQ, sizeof(REQ), 50) != HAL_OK) {
    DE_RX(); return false;
  }
  while (!__HAL_UART_GET_FLAG(s_rs485, UART_FLAG_TC)) { /* vent TC */ }
  DE_RX();

  // Modtag svar
  uint32_t n = 0, t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < s_total_to_ms && n < RESP_LEN) {
    if (HAL_UART_Receive(s_rs485, &resp[n], 1, s_perbyte_to_ms) == HAL_OK) n++;
  }
  *got = n;
  return (n == RESP_LEN);
}

/* Parse 01 03 08 [8 data] CRClo CRChi → m/s og grader */
static bool parse_speed_dir(const uint8_t *resp, float *speed_ms, float *dir_deg)
{
  if (resp[0] != 0x01 || resp[1] != 0x03 || resp[2] != 0x08) return false;

  uint16_t rxcrc = (uint16_t)resp[11] | ((uint16_t)resp[12] << 8);
  uint16_t calc  = modbus_crc16(resp, 11);
  if (rxcrc != calc) return false;

  const uint8_t *d = &resp[3];
  uint16_t speed_raw = ((uint16_t)d[0] << 8) | d[1]; // big-endian
  uint16_t dir_raw   = ((uint16_t)d[6] << 8) | d[7];

  *speed_ms = speed_raw / 10.0f;
  *dir_deg  = (dir_raw <= 3600) ? (dir_raw / 10.0f) : ((dir_raw % 3600) / 10.0f);
  return true;
}

/* === Public API === */
void WindRS485_Init(UART_HandleTypeDef *huart_rs485,
                    GPIO_TypeDef *de_port, uint16_t de_pin)
{
  s_rs485  = huart_rs485;
  s_de_port = de_port;
  s_de_pin  = de_pin;
  DE_RX(); // default til modtag

  // Tøm evt. RX
  uint8_t d; while (HAL_UART_Receive(s_rs485, &d, 1, 0) == HAL_OK) {}
}

bool WindRS485_Get(float *speed_ms, float *dir_deg)
{
  uint8_t  resp[RESP_LEN];
  uint32_t got = 0;

  if (!rs485_exchange(resp, &got)) return false;
  return parse_speed_dir(resp, speed_ms, dir_deg);
}

void WindRS485_SetTimeouts(uint32_t total_ms, uint32_t perbyte_ms)
{
  if (total_ms   >= 10 && total_ms   <= 500) s_total_to_ms   = total_ms;
  if (perbyte_ms >= 0  && perbyte_ms <= 10 ) s_perbyte_to_ms = perbyte_ms;
}
