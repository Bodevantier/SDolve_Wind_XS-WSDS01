#include "serial_debug.h"
#include <string.h>

#define RX_BUF_SIZE 32

static UART_HandleTypeDef *s_huart = NULL;
static char rx_buffer[RX_BUF_SIZE];
static uint8_t rx_index = 0;

/* internal helpers */
static void process_command(const char *cmd)
{
  /* Normalize: remove spaces and uppercase */
  char norm[RX_BUF_SIZE];
  uint8_t ni = 0;
  for (uint8_t i = 0; cmd[i] != '\0' && ni < (RX_BUF_SIZE - 1); i++)
  {
    char c = cmd[i];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
    if (c >= 'a' && c <= 'z') c = c - ('a' - 'A');
    norm[ni++] = c;
  }
  norm[ni] = '\0';

  if (strcmp(norm, "LED1ON") == 0)
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(s_huart, (uint8_t *)"OK: LED1 ON\r\n", sizeof("OK: LED1 ON\r\n") - 1, HAL_MAX_DELAY);
  }
  else if (strcmp(norm, "LED1OFF") == 0)
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_UART_Transmit(s_huart, (uint8_t *)"OK: LED1 OFF\r\n", sizeof("OK: LED1 OFF\r\n") - 1, HAL_MAX_DELAY);
  }
  else if (strcmp(norm, "LED2ON") == 0)
  {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(s_huart, (uint8_t *)"OK: LED2 ON\r\n", sizeof("OK: LED2 ON\r\n") - 1, HAL_MAX_DELAY);
  }
  else if (strcmp(norm, "LED2OFF") == 0)
  {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_UART_Transmit(s_huart, (uint8_t *)"OK: LED2 OFF\r\n", sizeof("OK: LED2 OFF\r\n") - 1, HAL_MAX_DELAY);
  }
  else if (strcmp(norm, "STATUS") == 0)
  {
    GPIO_PinState s1 = HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin);
    GPIO_PinState s2 = HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin);
    HAL_UART_Transmit(s_huart, (uint8_t *)(s1 == GPIO_PIN_SET ? "STATUS: LED1=ON " : "STATUS: LED1=OFF "), s1 == GPIO_PIN_SET ? strlen("STATUS: LED1=ON ") : strlen("STATUS: LED1=OFF "), HAL_MAX_DELAY);
    HAL_UART_Transmit(s_huart, (uint8_t *)(s2 == GPIO_PIN_SET ? "LED2=ON\r\n" : "LED2=OFF\r\n"), s2 == GPIO_PIN_SET ? strlen("LED2=ON\r\n") : strlen("LED2=OFF\r\n"), HAL_MAX_DELAY);
  }
  else
  {
    HAL_UART_Transmit(s_huart, (uint8_t *)"ERROR: Unknown command\r\n", sizeof("ERROR: Unknown command\r\n") - 1, HAL_MAX_DELAY);
  }
}

void SerialDebug_Init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  HAL_UART_Transmit(s_huart, (uint8_t *)"Serial debug ready\r\n", sizeof("Serial debug ready\r\n") - 1, HAL_MAX_DELAY);
}

void SerialDebug_Poll(void)
{
  uint8_t ch;
  /* Non-blocking poll */
  HAL_StatusTypeDef stat = HAL_UART_Receive(s_huart, &ch, 1, 0);
  if (stat == HAL_OK)
  {
    if (ch != '\r' && ch != '\n')
    {
      if (rx_index < (RX_BUF_SIZE - 1))
      {
        rx_buffer[rx_index++] = (char)ch;
      }
      else
      {
        rx_index = 0;
        HAL_UART_Transmit(s_huart, (uint8_t *)"ERROR: command too long\r\n", sizeof("ERROR: command too long\r\n") - 1, HAL_MAX_DELAY);
      }
    }
    else
    {
      rx_buffer[rx_index] = '\0';
      if (rx_index > 0)
      {
        process_command(rx_buffer);
      }
      rx_index = 0;
    }
  }
  else if (stat == HAL_TIMEOUT)
  {
    if (rx_index > 0)
    {
      uint32_t gather_start = HAL_GetTick();
      uint8_t extra;
      uint8_t got = 0;
      while ((HAL_GetTick() - gather_start) < 100)
      {
        if (HAL_UART_Receive(s_huart, &extra, 1, 20) == HAL_OK)
        {
          if (extra != '\r' && extra != '\n')
          {
            if (rx_index < (RX_BUF_SIZE - 1)) rx_buffer[rx_index++] = (char)extra;
          }
          else
          {
            rx_buffer[rx_index] = '\0';
            process_command(rx_buffer);
            rx_index = 0;
            got = 1;
            break;
          }
        }
      }

      if (!got)
      {
        rx_buffer[rx_index] = '\0';
        process_command(rx_buffer);
        rx_index = 0;
      }
    }
  }
}
