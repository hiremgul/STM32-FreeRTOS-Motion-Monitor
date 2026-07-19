/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stddef.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
    uint32_t time_ms;

    int8_t acc_x;
    int8_t acc_y;
    int8_t acc_z;

    uint8_t motion_detected;
    uint8_t accel_ok;
    uint8_t green_led_state;

} SystemData_t;


typedef struct __attribute__((packed, aligned(4)))
{
    uint32_t magic;
    uint32_t time_ms;

    int8_t acc_x;
    int8_t acc_y;
    int8_t acc_z;

    uint8_t motion_detected;
    uint8_t accel_ok;
    uint8_t green_led_state;

    uint16_t crc;

} FlashRecord_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FLASH_LOG_MAGIC          0x53454E53U

/*
 * STM32F407VGT6 1 MB flash için son sektör.
 * Sektör 11:
 * Başlangıç: 0x080E0000
 * Bitiş:     0x08100000
 */
#define FLASH_LOG_START_ADDRESS  0x080E0000U
#define FLASH_LOG_END_ADDRESS    0x08100000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for orangeLed */
osThreadId_t orangeLedHandle;
const osThreadAttr_t orangeLed_attributes = {
  .name = "orangeLed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for greenLed */
osThreadId_t greenLedHandle;
const osThreadAttr_t greenLed_attributes = {
  .name = "greenLed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for redLed */
osThreadId_t redLedHandle;
const osThreadAttr_t redLed_attributes = {
  .name = "redLed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for blueLed */
osThreadId_t blueLedHandle;
const osThreadAttr_t blueLed_attributes = {
  .name = "blueLed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for button */
osThreadId_t buttonHandle;
const osThreadAttr_t button_attributes = {
  .name = "button",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for gyro */
osThreadId_t gyroHandle;
const osThreadAttr_t gyro_attributes = {
  .name = "gyro",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for flashTask */
osThreadId_t flashTaskHandle;
const osThreadAttr_t flashTask_attributes = {
  .name = "flashTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for displayQueue */
osMessageQueueId_t displayQueueHandle;
const osMessageQueueAttr_t displayQueue_attributes = {
  .name = "displayQueue"
};
/* Definitions for flashQueue */
osMessageQueueId_t flashQueueHandle;
const osMessageQueueAttr_t flashQueue_attributes = {
  .name = "flashQueue"
};
/* Definitions for dataMutex */
osMutexId_t dataMutexHandle;
const osMutexAttr_t dataMutex_attributes = {
  .name = "dataMutex"
};
/* USER CODE BEGIN PV */

volatile uint8_t accel_ok = 0;
volatile uint8_t motion_detected = 0;
volatile uint8_t green_led_state = 0;

int8_t acc_x, acc_y, acc_z;

uint32_t flash_write_address = FLASH_LOG_START_ADDRESS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartDisplayTask(void *argument);
void StartFlashTask(void *argument);

/* USER CODE BEGIN PFP */

static void Telemetry_SendJson(const SystemData_t *data);

static uint16_t Flash_CalculateCrc(
    const uint8_t *data,
    uint32_t length
);

static void FlashLog_Init(void);
static HAL_StatusTypeDef FlashLog_Erase(void);

static HAL_StatusTypeDef FlashLog_Append(
    const SystemData_t *data
);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define LIS302DL_WHO_AM_I     0x0F
#define LIS302DL_CTRL_REG1    0x20
#define LIS302DL_OUT_X        0x29
#define LIS302DL_OUT_Y        0x2B
#define LIS302DL_OUT_Z        0x2D

#define LIS302DL_READ         0x80
#define LIS302DL_WHO_VALUE    0x3B

void LIS302DL_Write(uint8_t reg, uint8_t data)
{
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

uint8_t LIS302DL_Read(uint8_t reg)
{
    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    return rx[1];
}

uint8_t LIS302DL_Init(void)
{
    uint8_t id = LIS302DL_Read(LIS302DL_WHO_AM_I);

    if(id == 0x3B || id == 0x3F)
    {
        LIS302DL_Write(LIS302DL_CTRL_REG1, 0x47);
        return 1;
    }

    return 0;
}

static void Telemetry_SendJson(const SystemData_t *data)
{
    char message[160];

    int length = snprintf(
        message,
        sizeof(message),

        "{\"time\":%lu,"
        "\"x\":%d,"
        "\"y\":%d,"
        "\"z\":%d,"
        "\"motion\":%u,"
        "\"sensor_ok\":%u,"
        "\"green\":%u}\r\n",

        (unsigned long)data->time_ms,

        (int)data->acc_x,
        (int)data->acc_y,
        (int)data->acc_z,

        (unsigned int)data->motion_detected,
        (unsigned int)data->accel_ok,
        (unsigned int)data->green_led_state
    );

    if(length <= 0)
    {
        return;
    }

    if(length >= sizeof(message))
    {
        return;
    }

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)message,
        (uint16_t)length,
        100
    );
}


static uint16_t Flash_CalculateCrc(
    const uint8_t *data,
    uint32_t length
)
{
    uint16_t crc = 0xFFFF;

    for(uint32_t index = 0; index < length; index++)
    {
        crc ^= ((uint16_t)data[index] << 8);

        for(uint8_t bit = 0; bit < 8; bit++)
        {
            if(crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }

    return crc;
}


static void FlashLog_Init(void)
{
    FlashRecord_t record;

    flash_write_address = FLASH_LOG_START_ADDRESS;

    while(
        flash_write_address + sizeof(FlashRecord_t)
        <= FLASH_LOG_END_ADDRESS
    )
    {
        memcpy(
            &record,
            (const void *)flash_write_address,
            sizeof(FlashRecord_t)
        );

        if(record.magic != FLASH_LOG_MAGIC)
        {
            break;
        }

        uint16_t calculated_crc =
            Flash_CalculateCrc(
                (const uint8_t *)&record,
                offsetof(FlashRecord_t, crc)
            );

        if(record.crc != calculated_crc)
        {
            break;
        }

        flash_write_address += sizeof(FlashRecord_t);
    }
}


static HAL_StatusTypeDef FlashLog_Erase(void)
{
    FLASH_EraseInitTypeDef erase_config;

    uint32_t sector_error = 0;

    memset(
        &erase_config,
        0,
        sizeof(erase_config)
    );

    erase_config.TypeErase =
        FLASH_TYPEERASE_SECTORS;

    erase_config.Sector =
        FLASH_SECTOR_11;

    erase_config.NbSectors = 1;

    erase_config.VoltageRange =
        FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();

    HAL_StatusTypeDef status =
        HAL_FLASHEx_Erase(
            &erase_config,
            &sector_error
        );

    HAL_FLASH_Lock();

    if(status == HAL_OK)
    {
        flash_write_address =
            FLASH_LOG_START_ADDRESS;
    }

    return status;
}


static HAL_StatusTypeDef FlashLog_Append(
    const SystemData_t *data
)
{
    if(
        flash_write_address + sizeof(FlashRecord_t)
        > FLASH_LOG_END_ADDRESS
    )
    {
        if(FlashLog_Erase() != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    FlashRecord_t record;

    memset(
        &record,
        0,
        sizeof(record)
    );

    record.magic = FLASH_LOG_MAGIC;
    record.time_ms = data->time_ms;

    record.acc_x = data->acc_x;
    record.acc_y = data->acc_y;
    record.acc_z = data->acc_z;

    record.motion_detected =
        data->motion_detected;

    record.accel_ok =
        data->accel_ok;

    record.green_led_state =
        data->green_led_state;

    record.crc =
        Flash_CalculateCrc(
            (const uint8_t *)&record,
            offsetof(FlashRecord_t, crc)
        );

    HAL_FLASH_Unlock();

    HAL_StatusTypeDef status = HAL_OK;

    const uint32_t *words =
        (const uint32_t *)&record;

    uint32_t word_count =
        sizeof(FlashRecord_t) / sizeof(uint32_t);

    for(uint32_t index = 0; index < word_count; index++)
    {
        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_WORD,

            flash_write_address
            + index * sizeof(uint32_t),

            words[index]
        );

        if(status != HAL_OK)
        {
            break;
        }
    }

    HAL_FLASH_Lock();

    if(status == HAL_OK)
    {
        flash_write_address +=
            sizeof(FlashRecord_t);
    }

    return status;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  FlashLog_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of dataMutex */
  dataMutexHandle = osMutexNew(&dataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of displayQueue */
  displayQueueHandle = osMessageQueueNew (8, sizeof(SystemData_t), &displayQueue_attributes);

  /* creation of flashQueue */
  flashQueueHandle = osMessageQueueNew (8, sizeof(SystemData_t), &flashQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of orangeLed */
  orangeLedHandle = osThreadNew(StartDefaultTask, NULL, &orangeLed_attributes);

  /* creation of greenLed */
  greenLedHandle = osThreadNew(StartTask02, NULL, &greenLed_attributes);

  /* creation of redLed */
  redLedHandle = osThreadNew(StartTask03, NULL, &redLed_attributes);

  /* creation of blueLed */
  blueLedHandle = osThreadNew(StartTask04, NULL, &blueLed_attributes);

  /* creation of button */
  buttonHandle = osThreadNew(StartTask05, NULL, &button_attributes);

  /* creation of gyro */
  gyroHandle = osThreadNew(StartTask06, NULL, &gyro_attributes);

  /* creation of displayTask */
  displayTaskHandle = osThreadNew(StartDisplayTask, NULL, &displayTask_attributes);

  /* creation of flashTask */
  flashTaskHandle = osThreadNew(StartFlashTask, NULL, &flashTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Green_Led_Pin|Orange_Led_Pin|Red_Led_Pin|Blue_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_Led_Pin Orange_Led_Pin Red_Led_Pin Blue_Led_Pin */
  GPIO_InitStruct.Pin = Green_Led_Pin|Orange_Led_Pin|Red_Led_Pin|Blue_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the orangeLed thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	    HAL_GPIO_TogglePin(GPIOD, Orange_Led_Pin);
	    osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the greenLed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */

void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

  for(;;)
  {
    if(green_led_state)
    {
      HAL_GPIO_WritePin(
          GPIOD,
          Green_Led_Pin,
          GPIO_PIN_SET
      );
    }
    else
    {
      HAL_GPIO_WritePin(
          GPIOD,
          Green_Led_Pin,
          GPIO_PIN_RESET
      );
    }

    osDelay(50);
  }

  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the redLed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	    if(accel_ok == 0)
	        HAL_GPIO_WritePin(GPIOD, Red_Led_Pin, GPIO_PIN_SET);
	    else
	        HAL_GPIO_WritePin(GPIOD, Red_Led_Pin, GPIO_PIN_RESET);

	    osDelay(100);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the blueLed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
	    if(motion_detected)
	    {
	        HAL_GPIO_TogglePin(GPIOD, Blue_Led_Pin);
	        osDelay(100);
	    }
	    else
	    {
	        HAL_GPIO_WritePin(GPIOD, Blue_Led_Pin, GPIO_PIN_RESET);
	        osDelay(100);
	    }
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the button thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  uint8_t last_state = 0;
  for(;;)
  {
	    uint8_t current_state = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);

	    if(current_state == GPIO_PIN_SET && last_state == GPIO_PIN_RESET)
	    {
	        green_led_state = !green_led_state;
	        osDelay(200);
	    }

	    last_state = current_state;
	    osDelay(20);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the gyro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */

void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */

  accel_ok = LIS302DL_Init();

  uint32_t last_flash_time = 0;

  for(;;)
  {
    if(accel_ok)
    {
      acc_x = (int8_t)LIS302DL_Read(
          LIS302DL_OUT_X
      );

      acc_y = (int8_t)LIS302DL_Read(
          LIS302DL_OUT_Y
      );

      acc_z = (int8_t)LIS302DL_Read(
          LIS302DL_OUT_Z
      );

      if(
          acc_x > 60 || acc_x < -60 ||
          acc_y > 60 || acc_y < -60 ||
          acc_z > 60 || acc_z < -60
      )
      {
        motion_detected = 1;
      }
      else
      {
        motion_detected = 0;
      }
    }
    else
    {
      motion_detected = 0;
    }

    SystemData_t current_data;

    current_data.time_ms = HAL_GetTick();

    current_data.acc_x = acc_x;
    current_data.acc_y = acc_y;
    current_data.acc_z = acc_z;

    current_data.motion_detected =
        motion_detected;

    current_data.accel_ok =
        accel_ok;

    current_data.green_led_state =
        green_led_state;

    osMessageQueuePut(
        displayQueueHandle,
        &current_data,
        0,
        0
    );

    if(
        HAL_GetTick() - last_flash_time
        >= 1000
    )
    {
      osMessageQueuePut(
          flashQueueHandle,
          &current_data,
          0,
          0
      );

      last_flash_time = HAL_GetTick();
    }

    osDelay(100);
  }

  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */

void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */

  SystemData_t received_data;

  for(;;)
  {
    osStatus_t queue_status =
        osMessageQueueGet(
            displayQueueHandle,
            &received_data,
            NULL,
            osWaitForever
        );

    if(queue_status == osOK)
    {
      Telemetry_SendJson(&received_data);
    }
  }

  /* USER CODE END StartDisplayTask */
}


/* USER CODE BEGIN Header_StartFlashTask */
/**
* @brief Function implementing the flashTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFlashTask */
void StartFlashTask(void *argument)
{
  /* USER CODE BEGIN StartFlashTask */

  SystemData_t received_data;

  for(;;)
  {
    osStatus_t queue_status =
        osMessageQueueGet(
            flashQueueHandle,
            &received_data,
            NULL,
            osWaitForever
        );

    if(queue_status == osOK)
    {
      FlashLog_Append(&received_data);
    }
  }

  /* USER CODE END StartFlashTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
