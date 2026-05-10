/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART3_DE_RE_Pin GPIO_PIN_2
#define USART3_DE_RE_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* ---------------------------------------------------------------------------
 * POWER_SAVE_LOW_CLOCK
 *   0 = Default. SYSCLK = 72 MHz (HSE 8 MHz x PLL9), PCLK1 = 36 MHz.
 *   1 = SYSCLK = HCLK = PCLK1 = PCLK2 = 8 MHz (HSE bypass, PLL OFF,
 *       FLASH 0 ws). Drops core current dramatically and is more than
 *       enough for the 5 Hz RS485 wind poll + 10 Hz PGN 130306 TX cadence.
 *
 * IMPORTANT: lowering the system clock changes PCLK1, which is the source
 * for both the bxCAN bit timing and the USART3 (RS485) baud generator.
 * The CAN prescaler is auto-adjusted via N2K_CAN_PRESCALER below; HAL
 * recomputes USART BRR from PCLK1 at runtime so the RS485 baud rate is
 * preserved. However, the wind sensor cannot be tested off-boat, so this
 * flag is left at 0 by default. Flip to 1 only after verifying RS485
 * still talks to the masthead anemometer at the new clock.
 * ---------------------------------------------------------------------*/
#ifndef POWER_SAVE_LOW_CLOCK
#define POWER_SAVE_LOW_CLOCK 0
#endif

#if POWER_SAVE_LOW_CLOCK
  /* PCLK1 = 8 MHz: prescaler 2 -> 4 MHz TQ clock, 16 TQ/bit -> 250 kbps. */
  #define N2K_CAN_PRESCALER 2u
#else
  /* PCLK1 = 36 MHz: prescaler 9 -> 4 MHz TQ clock, 16 TQ/bit -> 250 kbps. */
  #define N2K_CAN_PRESCALER 9u
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
