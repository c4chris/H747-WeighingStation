/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct _cell_t {
	I2C_HandleTypeDef *handle;
	volatile uint8_t address;
	GPIO_TypeDef *gpio;
	uint16_t pin;
} CellTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEFAULT_STACK_SIZE               (1 * 1024)
/* fx_sd_thread priority */
#define DEFAULT_THREAD_PRIO              10

/* fx_sd_thread preemption priority */
#define DEFAULT_PREEMPTION_THRESHOLD      DEFAULT_THREAD_PRIO

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* Define ThreadX global data structures.  */
TX_THREAD       cm4_main_thread;
TX_THREAD       cm4_i2c1_thread;
TX_THREAD       cm4_i2c4_thread;
TX_THREAD       cm4_uart_thread;

/* ...  */
volatile unsigned int u2rc;
volatile unsigned int u2hrc;
volatile unsigned int u2tc;
volatile unsigned int u2htc;
volatile unsigned int u2ec;
volatile unsigned int u2ic;
__attribute__((section(".sram3.bridgeError"))) volatile unsigned int bridgeError[4];
__attribute__((section(".sram3.bridgeCount"))) volatile unsigned int bridgeCount[4];
__attribute__((section(".sram3.bridgeStale"))) volatile unsigned int bridgeStale[4];
__attribute__((section(".sram3.bridgeBadstatus"))) volatile unsigned int bridgeBadstatus[4];
__attribute__((section(".sram3.bridgeValue"))) volatile uint32_t bridgeValue[4];
unsigned char dbgBuf[256];
unsigned char input[64];
unsigned char u2tx[256];
volatile uint8_t setZero[4];
volatile uint8_t setZero3;
volatile uint8_t setZero4;
CellTypeDef cell[4] = {
		{&hi2c4, 0x28 << 1, NE4_A_GPIO_Port, NE4_A_Pin},
		{&hi2c4, 0x36 << 1, NE4_B_GPIO_Port, NE4_B_Pin},
		{&hi2c1, 0x28 << 1, NE1_A_GPIO_Port, NE1_A_Pin},
		{&hi2c1, 0x36 << 1, NE1_B_GPIO_Port, NE1_B_Pin}
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void tx_cm4_main_thread_entry(ULONG thread_input);
void tx_cm4_i2c1_thread_entry(ULONG thread_input);
void tx_cm4_i2c4_thread_entry(ULONG thread_input);
void tx_cm4_uart_thread_entry(ULONG thread_input);
void Error_Handler(void);

HAL_StatusTypeDef read_cell(CellTypeDef *, uint8_t, uint8_t *, const uint32_t);
HAL_StatusTypeDef powerup_and_read(unsigned int, uint8_t *, const uint32_t);
HAL_StatusTypeDef write_word(CellTypeDef *, uint8_t *, uint8_t, uint8_t, uint8_t, const uint32_t);
HAL_StatusTypeDef exit_command_mode(CellTypeDef *, uint8_t *, const uint32_t);

/* USER CODE END PFP */
/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_Init */
	CHAR *pointer;

	/*Allocate memory for main_thread_entry*/
	ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

	/* Check DEFAULT_STACK_SIZE allocation*/
	if (ret != TX_SUCCESS)
	{
		Error_Handler();
	}

	/* Create the main thread.  */
	ret = tx_thread_create(&cm4_main_thread, "tx_cm4_main_thread", tx_cm4_main_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
												 DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

	/* Check main thread creation */
	if (ret != TX_SUCCESS)
	{
		Error_Handler();
	}

	/*Allocate memory for i2c1_thread_entry*/
	ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

	/* Check DEFAULT_STACK_SIZE allocation*/
	if (ret != TX_SUCCESS)
	{
		Error_Handler();
	}

	/* Create the i2c1 thread.  */
	ret = tx_thread_create(&cm4_i2c1_thread, "tx_cm4_i2c1_thread", tx_cm4_i2c1_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
												 DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

	/* Check i2c1 thread creation */
	if (ret != TX_SUCCESS)
	{
		Error_Handler();
	}

	/*Allocate memory for i2c4_thread_entry*/
	ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

	/* Check DEFAULT_STACK_SIZE allocation*/
	if (ret != TX_SUCCESS)
	{
		Error_Handler();
	}

	/* Create the i2c4 thread.  */
	ret = tx_thread_create(&cm4_i2c4_thread, "tx_cm4_i2c4_thread", tx_cm4_i2c4_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
												 DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

	/* Check i2c4 thread creation */
	if (ret != TX_SUCCESS)
	{
		Error_Handler();
	}

	/*Allocate memory for uart_thread_entry*/
	ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);

	/* Check DEFAULT_STACK_SIZE allocation*/
	if (ret != TX_SUCCESS)
	{
		Error_Handler();
	}

	/* Create the uart thread.  */
	ret = tx_thread_create(&cm4_uart_thread, "tx_cm4_uart_thread", tx_cm4_uart_thread_entry, 0, pointer, DEFAULT_STACK_SIZE, DEFAULT_THREAD_PRIO,
												 DEFAULT_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);

	/* Check uart thread creation */
	if (ret != TX_SUCCESS)
	{
		Error_Handler();
	}
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/* USER CODE BEGIN 1 */

int UART_Receive(unsigned char *dest, const unsigned char *rx, UART_HandleTypeDef *huart, unsigned int *uxcc, const unsigned int max)
{
	unsigned int cc = __HAL_DMA_GET_COUNTER(huart->hdmarx);
	if (*uxcc != cc)
	{
		HAL_UART_DMAPause(huart);
  	int len = 0;
		if (cc > *uxcc)
		{
			for (unsigned int i = max - *uxcc; i < max; i++)
				dest[len++] = rx[i];
			for (unsigned int i = 0; i < max - cc; i++)
				dest[len++] = rx[i];
		}
		else
		{
			for (unsigned int i = max - *uxcc; i < max - cc; i++)
				dest[len++] = rx[i];
		}
		HAL_UART_DMAResume(huart);
  	*uxcc = cc;
  	return len;
	}
	return 0;
}

HAL_StatusTypeDef read_cell(CellTypeDef *cell, uint8_t a, uint8_t *dataBuf, const uint32_t I2C_Timeout)
{
	dataBuf[0] = a;
	dataBuf[1] = 0;
	dataBuf[2] = 0;
	HAL_I2C_Master_Transmit(cell->handle, cell->address, dataBuf, 3, I2C_Timeout);
	memset(dataBuf, 0, 3);
	return HAL_I2C_Master_Receive(cell->handle, cell->address | 1, dataBuf, 3, I2C_Timeout);
}

HAL_StatusTypeDef powerup_and_read(unsigned int c, uint8_t *dataBuf, const uint32_t I2C_Timeout)
{
	dataBuf[0] = 0xA0;
	dataBuf[1] = 0;
	dataBuf[2] = 0;
	// we power up both sides, and then power off the side we do not want to talk to
	// this seems to do the trick - when powering up only one side we get no answer
	unsigned int o = c ^ 1;
	HAL_GPIO_WritePin(cell[c].gpio, cell[c].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(cell[o].gpio, cell[o].pin, GPIO_PIN_RESET);
	HAL_Delay(3);
	HAL_GPIO_WritePin(cell[o].gpio, cell[o].pin, GPIO_PIN_SET);
	HAL_I2C_Master_Transmit(cell[c].handle, cell[c].address, dataBuf, 3, I2C_Timeout);
	return read_cell(cell + c, 2, dataBuf, I2C_Timeout);
}

HAL_StatusTypeDef write_word(CellTypeDef *cell, uint8_t *dataBuf, uint8_t a, uint8_t d1, uint8_t d2, const uint32_t I2C_Timeout)
{
	dataBuf[0] = 0x40 | a;
	dataBuf[1] = d1;
	dataBuf[2] = d2;
	HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(cell->handle, cell->address, dataBuf, 3, I2C_Timeout);
	HAL_Delay(15); // wait 15 ms according to DS
	return res;
}

HAL_StatusTypeDef exit_command_mode(CellTypeDef *cell, uint8_t *dataBuf, const uint32_t I2C_Timeout)
{
	dataBuf[0] = 0x80;
	dataBuf[1] = 0;
	dataBuf[2] = 0;
	HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(cell->handle, cell->address, dataBuf, 3, I2C_Timeout);
	HAL_Delay(15); // wait another 15 ms to update EEPROM signature
	return res;
}

void tx_cm4_main_thread_entry(ULONG thread_input)
{
	uint32_t low[4] = { 950, 950, 950, 950 };
	unsigned int c[4] = { 0, 0, 0, 0 };
	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
	/* Infinite loop */
	for(;;)
	{
		ULONG ticks = tx_time_get() / TX_TIMER_TICKS_PER_SECOND;
		printf("WS %8lu",ticks);
		printf(" | %u %u %u %u",bridgeError[0],bridgeBadstatus[0],bridgeError[1],bridgeBadstatus[1]);
		printf(" %u %u %u %u",bridgeError[2],bridgeBadstatus[2],bridgeError[3],bridgeBadstatus[3]);
		for (unsigned int i = 0; i < 4; i++)
		{
			if (c[i] != bridgeCount[i])
			{
				uint32_t weight = (bridgeValue[i] >> 16) & 0x3fff;
				if (setZero[i])
				{
					low[i] = weight;
					setZero[i] = 0;
				}
				if (weight < low[i])
					weight = 0;
				else
					weight -= low[i];
				weight *= 5000;
				weight /= 14000;
				printf(" | %2d.%02d kg", (uint16_t)(weight / 100), (uint16_t)(weight % 100));
				uint32_t temp = (bridgeValue[i] >> 5) & 0x7ff;
				temp *= 2000;
				temp /= 2048; // just a guess at this point...
				temp -= 500;
				printf(" %2d.%01d C", (uint16_t)(temp / 10), (uint16_t)(temp % 10));
				c[i] = bridgeCount[i];
			}
			else
				printf(" |                ");
		}
		printf("\r\n");
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
	}
}

void tx_cm4_i2c1_thread_entry(ULONG thread_input)
{
	const unsigned int cellStart = 2;
	const unsigned int cellEnd = 4;
	const uint32_t I2C_Timeout = I2Cx_TIMEOUT_MAX;
	HAL_StatusTypeDef res;
	uint8_t dataBuf[4];
	for (unsigned int i = cellStart; i < cellEnd; i++)
		HAL_GPIO_WritePin(cell[i].gpio, cell[i].pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	for (unsigned int i = cellStart; i < cellEnd; i++)
	{
		bridgeError[i] = 0;
		bridgeStale[i] = 0;
		bridgeCount[i] = 0;
		bridgeBadstatus[i] = 0;
	}
	/* Infinite loop */
	for(;;)
	{
		for (unsigned int i = cellStart; i < cellEnd; i++)
		{
			memset(dataBuf, 0, 4);
			res = HAL_I2C_Master_Receive(cell[i].handle, cell[i].address | 1, dataBuf, 4, I2C_Timeout);
			if (res != HAL_OK)
			{
				bridgeError[i] += 1;
				HAL_I2C_DeInit(cell[i].handle);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
				HAL_I2C_Init(cell[i].handle);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
			}
			else
			{
				uint8_t status = (dataBuf[0] >> 6) & 0x3;
				if (status == 0)
				{
					bridgeValue[i] = (dataBuf[0] << 24) | (dataBuf[1] << 16) | (dataBuf[2] << 8) | dataBuf[3];
					bridgeCount[i] += 1;
				} else if (status == 2)
					bridgeStale[i] += 1;
				else
					bridgeBadstatus[i] += 1;
			}
		}
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 20);
	}
}

void tx_cm4_i2c4_thread_entry(ULONG thread_input)
{
	const unsigned int cellStart = 0;
	const unsigned int cellEnd = 2;
	const uint32_t I2C_Timeout = I2Cx_TIMEOUT_MAX;
	HAL_StatusTypeDef res;
	uint8_t dataBuf[4];
	for (unsigned int i = cellStart; i < cellEnd; i++)
		HAL_GPIO_WritePin(cell[i].gpio, cell[i].pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	for (unsigned int i = cellStart; i < cellEnd; i++)
	{
		bridgeError[i] = 0;
		bridgeStale[i] = 0;
		bridgeCount[i] = 0;
		bridgeBadstatus[i] = 0;
	}
	/* Infinite loop */
	for(;;)
	{
		ULONG ticks_target = tx_time_get() + (TX_TIMER_TICKS_PER_SECOND / 20);
		for (unsigned int i = cellStart; i < cellEnd; i++)
		{
			memset(dataBuf, 0, 4);
			res = HAL_I2C_Master_Receive(cell[i].handle, cell[i].address | 1, dataBuf, 4, I2C_Timeout);
			if (res != HAL_OK)
			{
				bridgeError[i] += 1;
				HAL_I2C_DeInit(cell[i].handle);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
				HAL_I2C_Init(cell[i].handle);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
			}
			else
			{
				uint8_t status = (dataBuf[0] >> 6) & 0x3;
				if (status == 0)
				{
					bridgeValue[i] = (dataBuf[0] << 24) | (dataBuf[1] << 16) | (dataBuf[2] << 8) | dataBuf[3];
					bridgeCount[i] += 1;
				} else if (status == 2)
					bridgeStale[i] += 1;
				else
					bridgeBadstatus[i] += 1;
			}
		}
		ULONG ticks = tx_time_get();
		if (ticks < ticks_target)
			tx_thread_sleep(ticks_target - ticks);
	}
}

void tx_cm4_uart_thread_entry(ULONG thread_input)
{
	//UINT status;
	UCHAR read_buffer[32];
	CHAR data[] = "This is ThreadX working on STM32 CM4";

	printf("\r\n%s\r\nStarting Run on %s\r\n", data, _tx_version_id);
	/* Infinite Loop */
	for( ;; )
	{

		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);

	}
	HAL_UART_Receive_DMA(&huart1, read_buffer, 32);
	unsigned int u2cc = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	//HAL_StatusTypeDef res;
	int inLen = 0;
	/* Infinite loop */
	for(;;)
	{
		int len = UART_Receive(input + inLen, read_buffer, &huart1, &u2cc, 32);
		if (len > 0)
		{
			printf("%.*s", len, input + inLen);
			inLen += len;
			if (input[inLen - 1] == '\r')
			{
				int cmdLen = inLen - 1;
				printf("\nReceived command '%.*s'\r\n# ", cmdLen, input);
				inLen = 0;
				if (strncmp((char *) input, "UART", cmdLen) == 0)
					printf("u2rc = %u u2hrc = %u u2tc = %u u2htc = %u u2ec = %u u2ic = %u u2cc = %u\r\n# ", u2rc, u2hrc, u2tc, u2htc, u2ec, u2ic, u2cc);
			}
		}
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 20);
	}
}

/* USER CODE END 1 */
