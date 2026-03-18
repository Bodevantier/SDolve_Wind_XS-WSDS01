#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

#include "main.h"

void SerialDebug_Init(UART_HandleTypeDef *huart);
void SerialDebug_Poll(void);

#endif // SERIAL_DEBUG_H
