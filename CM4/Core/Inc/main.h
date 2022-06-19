/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* This needs to be the same for both cores */
typedef struct
{
  unsigned int bridgeError[4];
  unsigned int bridgeCount[4];
  unsigned int bridgeStale[4];
  unsigned int bridgeBadstatus[4];
  uint32_t bridgeValue[4];
  uint32_t weight[4];
	unsigned int setZero[4];
	unsigned int unsetZero[4];
  uint16_t touchData[4], touchData2[4];
} CM4_CM7_SharedDataTypeDef;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c4;
extern UART_HandleTypeDef huart1;
extern volatile uint32_t threadInitDone;
extern volatile CM4_CM7_SharedDataTypeDef sharedData;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void my_Delay(uint32_t);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NE4_A_Pin GPIO_PIN_3
#define NE4_A_GPIO_Port GPIOD
#define NE4_B_Pin GPIO_PIN_11
#define NE4_B_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define NE1_A_Pin GPIO_PIN_7
#define NE1_A_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define STLINK_TX_Pin GPIO_PIN_10
#define STLINK_TX_GPIO_Port GPIOA
#define STLINK_RX_Pin GPIO_PIN_9
#define STLINK_RX_GPIO_Port GPIOA
#define CEC_CK_MCO1_Pin GPIO_PIN_8
#define CEC_CK_MCO1_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOI
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOI
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define NE1_B_Pin GPIO_PIN_6
#define NE1_B_GPIO_Port GPIOA
#define I2C4_SCL_Pin GPIO_PIN_12
#define I2C4_SCL_GPIO_Port GPIOD
#define I2C4_SDA_Pin GPIO_PIN_13
#define I2C4_SDA_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

#define HSEM_ID_0 (0U) /* HW semaphore 0 - used to coordinate boot with CM4 */
#define HSEM_ID_1 (1U) /* HW semaphore 1 - CM4 sends touchdata to CM7 */
#define HSEM_ID_2 (2U) /* HW semaphore 2 - CM4 signals camera data to CM7 */
#define HSEM_ID_3 (3U) /* HW semaphore 3 - CM4 signals USB stick status change to CM7 */
#define HSEM_ID_4 (4U) /* HW semaphore 4 - CM7 asks CM4 to perform some actions on USB stick */
#define HSEM_0 (__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0))
#define HSEM_1 (__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_1))
#define HSEM_2 (__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_2))
#define HSEM_3 (__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_3))
#define HSEM_4 (__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_4))

#define SDRAM_BANK_0                0xD0000000UL
#define SDRAM_BANK_1                0xD0800000UL
#define SDRAM_BANK_2                0xD1000000UL
#define SDRAM_BANK_3                0xD1800000UL

#define ADV7533_MAIN_I2C_ADDR           0x7AU
#define ADV7533_CEC_DSI_I2C_ADDR        0x78U
#define ADV7533_MAIN_POWER_DOWN_REG     0x41U

#define I2Cx_TIMEOUT_MAX               0x3000 /* The value of the maximal timeout for I2C waiting loops */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
