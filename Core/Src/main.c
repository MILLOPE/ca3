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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "network.h"
#include "network_data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACC_CAPTURE_WINDOW_SIZE          52U
#define HAR_MODEL_WINDOW_SIZE            AI_NETWORK_IN_1_HEIGHT
#define ACC_STEP_BASE_MS                 19U
#define ACC_STEP_FRACTION_NUM            3U
#define ACC_STEP_FRACTION_DEN            13U

#define GYRO_PERIOD_MS                   1000U  /* 1 Hz */
#define HUMIDITY_PERIOD_MS               1000U  /* 1 Hz */
#define TEMPERATURE_PERIOD_MS            1000U  /* 1 Hz */
#define MAGNETO_PERIOD_MS                1000U  /* 1 Hz */
#define PRESSURE_PERIOD_MS               1000U  /* 1 Hz */
#define REPORT_PERIOD_MS                 1000U

#define ACC_ODR_MHZ                      52000U
#define GYRO_ODR_MHZ                     1000U
#define HUMIDITY_ODR_MHZ                 1000U
#define TEMPERATURE_ODR_MHZ              1000U
#define MAGNETO_ODR_MHZ                  1000U
#define PRESSURE_ODR_MHZ                 1000U

#define INFERENCE_QUEUE_DEPTH            4U
#define UART_TX_TIMEOUT_MS               50U
/* Reference HAR example uses accelerometer normalization by 4000.0f. */
#define HAR_INPUT_SCALE                  (1.0f / 4000.0f)

/* Option 1 (External Task Resilience): simulated urgent blocking task. */
#define ENABLE_OPTION1_EXTERNAL_TASK     0
#define URGENT_TASK_PERIOD_MS            1000U
#define URGENT_TASK_BLOCK_MS             30U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} Vec3i16_t;

typedef struct
{
  float x;
  float y;
  float z;
} Vec3f_t;

typedef struct
{
  uint64_t captured_updates;
  uint64_t missed_updates;
  uint32_t last_poll_tick_ms;
  uint8_t initialized;
} TaskStats_t;

static Vec3i16_t g_latest_accel_mg;
static Vec3f_t g_latest_gyro_mdps;
static float g_latest_humidity_rh;
static float g_latest_temperature_c;
static Vec3i16_t g_latest_magneto_mgauss;
static float g_latest_pressure_hpa;

static Vec3i16_t g_acc_window[ACC_CAPTURE_WINDOW_SIZE];
static uint32_t g_acc_window_count;

static Vec3i16_t g_inference_queue[INFERENCE_QUEUE_DEPTH][HAR_MODEL_WINDOW_SIZE];
static uint32_t g_inference_release_tick[INFERENCE_QUEUE_DEPTH];
static uint8_t g_inference_head;
static uint8_t g_inference_tail;
static uint8_t g_inference_count;
static uint32_t g_inference_windows_dropped;
static uint32_t g_inference_completed;

static uint32_t g_last_inference_exec_ms;
static uint32_t g_last_inference_latency_ms;
static uint32_t g_last_uart_latency_ms;

static char g_last_classification[16] = "Unknown";
static char g_uart_line[640];

static TaskStats_t g_stat_accel;
static TaskStats_t g_stat_gyro;
static TaskStats_t g_stat_humidity;
static TaskStats_t g_stat_temperature;
static TaskStats_t g_stat_magneto;
static TaskStats_t g_stat_pressure;

static uint32_t g_acc_next_tick_ms;
static uint32_t g_acc_step_error_accum;
static uint32_t g_gyro_next_tick_ms;
static uint32_t g_humidity_next_tick_ms;
static uint32_t g_temperature_next_tick_ms;
static uint32_t g_magneto_next_tick_ms;
static uint32_t g_pressure_next_tick_ms;
static uint32_t g_report_next_tick_ms;

static uint32_t g_system_start_tick_ms;
static uint32_t g_scheduler_iterations;
static uint32_t g_sensor_init_failures;
static uint32_t g_ai_init_failures;

static ai_handle g_ai_network = AI_HANDLE_NULL;
AI_ALIGNED(32) static ai_u8 g_ai_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
static ai_float g_ai_input[AI_NETWORK_IN_1_SIZE];
static ai_float g_ai_output[AI_NETWORK_OUT_1_SIZE];
static ai_buffer *g_ai_input_buf;
static ai_buffer *g_ai_output_buf;
static uint8_t g_ai_ready;
static float g_last_ai_scores[AI_NETWORK_OUT_1_SIZE];

#if ENABLE_OPTION1_EXTERNAL_TASK
static uint32_t g_urgent_task_next_tick_ms;
static uint32_t g_urgent_task_runs;
static uint32_t g_inference_deferrals;
static uint32_t g_uart_report_deferrals;
static uint8_t g_uart_report_pending;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void App_Init(void);
static void Scheduler_RunOnce(void);
static void Poll_Accelerometer(void);
static void Poll_Gyroscope(void);
static void Poll_Humidity(void);
static void Poll_Temperature(void);
static void Poll_Magnetometer(void);
static void Poll_Pressure(void);
static void Queue_Inference_Window(uint32_t window_ready_tick);
static void Run_Inference_IfReady(void);
static const char *Run_HAR_Net(const Vec3i16_t *window);
static bool AI_Model_Init(void);
static void Send_Uart_Line(const char *line);
static void Report_Status(uint32_t now_ms);
static void FormatFixedSigned(char *dst, size_t dst_len, int32_t value, uint32_t scale, uint8_t frac_digits);
static uint32_t Calc_MissRate_x100(const TaskStats_t *stat);
static void Update_MissStats(TaskStats_t *stat, uint32_t now_ms, uint32_t odr_mhz);
static bool Critical_Task_Due(uint32_t now_ms);
#if ENABLE_OPTION1_EXTERNAL_TASK
static uint8_t Run_Urgent_External_Task_IfDue(uint32_t now_ms);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Team members:
 * - Name 1 (Matric No.)
 * - Name 2 (Matric No.) A0318154H 
 */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Send_Uart_Line("UART boot ok");
  App_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Scheduler_RunOnce();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DFSDM1_DATIN2_Pin DFSDM1_CKOUT_Pin */
  GPIO_InitStruct.Pin = DFSDM1_DATIN2_Pin|DFSDM1_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : QUADSPI_CLK_Pin QUADSPI_NCS_Pin OQUADSPI_BK1_IO0_Pin QUADSPI_BK1_IO1_Pin
                           QUAD_SPI_BK1_IO2_Pin QUAD_SPI_BK1_IO3_Pin */
  GPIO_InitStruct.Pin = QUADSPI_CLK_Pin|QUADSPI_NCS_Pin|OQUADSPI_BK1_IO0_Pin|QUADSPI_BK1_IO1_Pin
                          |QUAD_SPI_BK1_IO2_Pin|QUAD_SPI_BK1_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_UART3_TX_Pin INTERNAL_UART3_RX_Pin */
  GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_ID_Pin USB_OTG_FS_DM_Pin USB_OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin|USB_OTG_FS_DM_Pin|USB_OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_SPI3_SCK_Pin INTERNAL_SPI3_MISO_Pin INTERNAL_SPI3_MOSI_Pin */
  GPIO_InitStruct.Pin = INTERNAL_SPI3_SCK_Pin|INTERNAL_SPI3_MISO_Pin|INTERNAL_SPI3_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Baseline requirement: polling scheduler only (no DRDY interrupt-driven logic). */

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void App_Init(void)
{
  const uint32_t now = HAL_GetTick();

  g_system_start_tick_ms = now;
  g_acc_next_tick_ms = now;
  g_gyro_next_tick_ms = now;
  g_humidity_next_tick_ms = now;
  g_temperature_next_tick_ms = now;
  g_magneto_next_tick_ms = now;
  g_pressure_next_tick_ms = now;
  g_report_next_tick_ms = now + REPORT_PERIOD_MS;

#if ENABLE_OPTION1_EXTERNAL_TASK
  g_urgent_task_next_tick_ms = now + URGENT_TASK_PERIOD_MS;
#endif

  BSP_LED_Init(LED2);
  BSP_LED_Off(LED2);

  if (BSP_ACCELERO_Init() != ACCELERO_OK) { g_sensor_init_failures++; }
  if (BSP_GYRO_Init() != GYRO_OK) { g_sensor_init_failures++; }
  if (BSP_HSENSOR_Init() != HSENSOR_OK) { g_sensor_init_failures++; }
  if (BSP_TSENSOR_Init() != TSENSOR_OK) { g_sensor_init_failures++; }
  if (BSP_MAGNETO_Init() != MAGNETO_OK) { g_sensor_init_failures++; }
  if (BSP_PSENSOR_Init() != PSENSOR_OK) { g_sensor_init_failures++; }
  if (!AI_Model_Init()) { g_ai_init_failures++; }

  snprintf(g_uart_line, sizeof(g_uart_line),
           "CA3 start t=%lu ms, sensor_init_fail=%lu, ai_init_fail=%lu, option1=%u",
           (unsigned long)now,
           (unsigned long)g_sensor_init_failures,
           (unsigned long)g_ai_init_failures,
           (unsigned int)ENABLE_OPTION1_EXTERNAL_TASK);
  Send_Uart_Line(g_uart_line);
  Send_Uart_Line("ODR[Hz] acc=52 gyro=1 hum=1 temp=1 mag=1 prs=1; polling cyclic scheduler, HAR input downsampled 52->26");
}

static void Scheduler_RunOnce(void)
{
  uint32_t now = HAL_GetTick();
  g_scheduler_iterations++;
  bool critical_due;
#if ENABLE_OPTION1_EXTERNAL_TASK
  uint8_t blocked_now = 0U;
#endif

  while ((int32_t)(now - g_acc_next_tick_ms) >= 0)
  {
    uint32_t t_poll = HAL_GetTick();
    Update_MissStats(&g_stat_accel, t_poll, ACC_ODR_MHZ);
    Poll_Accelerometer();

    g_acc_next_tick_ms += ACC_STEP_BASE_MS;
    g_acc_step_error_accum += ACC_STEP_FRACTION_NUM;
    if (g_acc_step_error_accum >= ACC_STEP_FRACTION_DEN)
    {
      g_acc_step_error_accum -= ACC_STEP_FRACTION_DEN;
      g_acc_next_tick_ms += 1U;
    }
  }

  while ((int32_t)(now - g_gyro_next_tick_ms) >= 0)
  {
    uint32_t t_poll = HAL_GetTick();
    Update_MissStats(&g_stat_gyro, t_poll, GYRO_ODR_MHZ);
    Poll_Gyroscope();
    g_gyro_next_tick_ms += GYRO_PERIOD_MS;
  }

  while ((int32_t)(now - g_humidity_next_tick_ms) >= 0)
  {
    uint32_t t_poll = HAL_GetTick();
    Update_MissStats(&g_stat_humidity, t_poll, HUMIDITY_ODR_MHZ);
    Poll_Humidity();
    g_humidity_next_tick_ms += HUMIDITY_PERIOD_MS;
  }

  while ((int32_t)(now - g_temperature_next_tick_ms) >= 0)
  {
    uint32_t t_poll = HAL_GetTick();
    Update_MissStats(&g_stat_temperature, t_poll, TEMPERATURE_ODR_MHZ);
    Poll_Temperature();
    g_temperature_next_tick_ms += TEMPERATURE_PERIOD_MS;
  }

  while ((int32_t)(now - g_magneto_next_tick_ms) >= 0)
  {
    uint32_t t_poll = HAL_GetTick();
    Update_MissStats(&g_stat_magneto, t_poll, MAGNETO_ODR_MHZ);
    Poll_Magnetometer();
    g_magneto_next_tick_ms += MAGNETO_PERIOD_MS;
  }

  while ((int32_t)(now - g_pressure_next_tick_ms) >= 0)
  {
    uint32_t t_poll = HAL_GetTick();
    Update_MissStats(&g_stat_pressure, t_poll, PRESSURE_ODR_MHZ);
    Poll_Pressure();
    g_pressure_next_tick_ms += PRESSURE_PERIOD_MS;
  }

  now = HAL_GetTick();
  critical_due = Critical_Task_Due(now);

#if ENABLE_OPTION1_EXTERNAL_TASK
  blocked_now = Run_Urgent_External_Task_IfDue(now);
  now = HAL_GetTick();
  critical_due = critical_due || Critical_Task_Due(now);
#endif

  if (!critical_due
#if ENABLE_OPTION1_EXTERNAL_TASK
      && (blocked_now == 0U)
#endif
      )
  {
    Run_Inference_IfReady();
  }
#if ENABLE_OPTION1_EXTERNAL_TASK
  else if (g_inference_count > 0U)
  {
    g_inference_deferrals++;
  }
#endif

  now = HAL_GetTick();
  if ((int32_t)(now - g_report_next_tick_ms) >= 0)
  {
#if ENABLE_OPTION1_EXTERNAL_TASK
    if (critical_due || (blocked_now != 0U))
    {
      if (g_uart_report_pending == 0U)
      {
        g_uart_report_deferrals++;
        g_uart_report_pending = 1U;
      }
    }
    else
#endif
    {
      Report_Status(now);
      g_report_next_tick_ms += REPORT_PERIOD_MS;
#if ENABLE_OPTION1_EXTERNAL_TASK
      g_uart_report_pending = 0U;
#endif
    }
  }
}

static void Poll_Accelerometer(void)
{
  int16_t xyz[3];

  BSP_ACCELERO_AccGetXYZ(xyz);

  g_latest_accel_mg.x = xyz[0];
  g_latest_accel_mg.y = xyz[1];
  g_latest_accel_mg.z = xyz[2];

  g_acc_window[g_acc_window_count].x = xyz[0];
  g_acc_window[g_acc_window_count].y = xyz[1];
  g_acc_window[g_acc_window_count].z = xyz[2];
  g_acc_window_count++;

  if (g_acc_window_count >= ACC_CAPTURE_WINDOW_SIZE)
  {
    Queue_Inference_Window(HAL_GetTick());
    g_acc_window_count = 0U;
  }
}

static void Poll_Gyroscope(void)
{
  float xyz[3];

  BSP_GYRO_GetXYZ(xyz);
  g_latest_gyro_mdps.x = xyz[0];
  g_latest_gyro_mdps.y = xyz[1];
  g_latest_gyro_mdps.z = xyz[2];
}

static void Poll_Humidity(void)
{
  g_latest_humidity_rh = BSP_HSENSOR_ReadHumidity();
}

static void Poll_Temperature(void)
{
  g_latest_temperature_c = BSP_TSENSOR_ReadTemp();
}

static void Poll_Magnetometer(void)
{
  int16_t xyz[3];

  BSP_MAGNETO_GetXYZ(xyz);
  g_latest_magneto_mgauss.x = xyz[0];
  g_latest_magneto_mgauss.y = xyz[1];
  g_latest_magneto_mgauss.z = xyz[2];
}

static void Poll_Pressure(void)
{
  g_latest_pressure_hpa = BSP_PSENSOR_ReadPressure();
}

static void Queue_Inference_Window(uint32_t window_ready_tick)
{
  if (g_inference_count < INFERENCE_QUEUE_DEPTH)
  {
    uint8_t slot = g_inference_tail;
    uint32_t i;

    /* The assignment requires collecting 52 accelerometer samples per second.
     * The current generated HAR model still expects 26x3 input, so use every
     * other sample from the 52-sample capture window to preserve 1 second span. */
    for (i = 0U; i < HAR_MODEL_WINDOW_SIZE; i++)
    {
      g_inference_queue[slot][i] = g_acc_window[i * 2U];
    }
    g_inference_release_tick[slot] = window_ready_tick;

    g_inference_tail = (uint8_t)((g_inference_tail + 1U) % INFERENCE_QUEUE_DEPTH);
    g_inference_count++;
  }
  else
  {
    g_inference_windows_dropped++;
  }
}

static void Run_Inference_IfReady(void)
{
  const char *label;
  uint8_t slot;
  uint32_t start_ms;
  uint32_t end_ms;

  if (g_inference_count == 0U)
  {
    return;
  }

  slot = g_inference_head;
  start_ms = HAL_GetTick();

  label = Run_HAR_Net(g_inference_queue[slot]);

  end_ms = HAL_GetTick();
  g_last_inference_exec_ms = end_ms - start_ms;
  g_last_inference_latency_ms = end_ms - g_inference_release_tick[slot];
  strncpy(g_last_classification, label, sizeof(g_last_classification) - 1U);
  g_last_classification[sizeof(g_last_classification) - 1U] = '\0';
  g_inference_completed++;

  g_inference_head = (uint8_t)((g_inference_head + 1U) % INFERENCE_QUEUE_DEPTH);
  g_inference_count--;
}

static bool AI_Model_Init(void)
{
  ai_error err;
  const ai_handle act_addr[] = { g_ai_activations };

  err = ai_network_create_and_init(&g_ai_network, act_addr, NULL);
  if (err.type != AI_ERROR_NONE)
  {
    g_ai_network = AI_HANDLE_NULL;
    g_ai_ready = 0U;
    return false;
  }

  g_ai_input_buf = ai_network_inputs_get(g_ai_network, NULL);
  g_ai_output_buf = ai_network_outputs_get(g_ai_network, NULL);

  if ((g_ai_input_buf == NULL) || (g_ai_output_buf == NULL))
  {
    g_ai_ready = 0U;
    return false;
  }

  g_ai_input_buf[0].data = AI_HANDLE_PTR(g_ai_input);
  g_ai_output_buf[0].data = AI_HANDLE_PTR(g_ai_output);
  g_ai_ready = 1U;
  return true;
}

static const char *Run_HAR_Net(const Vec3i16_t *window)
{
  static const char *k_labels[3] = { "Stationary", "Walking", "Running" };
  uint32_t i;
  uint32_t best_idx = 0U;
  ai_i32 n_batches;
  float best_val;

  if (g_ai_ready == 0U)
  {
    return "AI_NOT_READY";
  }

  /* The generated model expects 26x3 float input. The scheduler now captures
   * 52 accelerometer samples per second and Queue_Inference_Window() reduces
   * that 52-sample span to 26 evenly spaced samples for inference. */
  for (i = 0U; i < AI_NETWORK_IN_1_HEIGHT; i++)
  {
    g_ai_input[(i * 3U) + 0U] = (ai_float)window[i].x * HAR_INPUT_SCALE;
    g_ai_input[(i * 3U) + 1U] = (ai_float)window[i].y * HAR_INPUT_SCALE;
    g_ai_input[(i * 3U) + 2U] = (ai_float)window[i].z * HAR_INPUT_SCALE;
  }

  n_batches = ai_network_run(g_ai_network, g_ai_input_buf, g_ai_output_buf);
  if (n_batches != 1)
  {
    return "AI_RUN_ERR";
  }

  for (i = 0U; i < AI_NETWORK_OUT_1_SIZE; i++)
  {
    g_last_ai_scores[i] = g_ai_output[i];
  }

  best_val = g_ai_output[0];
  for (i = 1U; i < AI_NETWORK_OUT_1_SIZE; i++)
  {
    if (g_ai_output[i] > best_val)
    {
      best_val = g_ai_output[i];
      best_idx = i;
    }
  }

  if (best_idx >= (sizeof(k_labels) / sizeof(k_labels[0])))
  {
    return "Unknown";
  }

  return k_labels[best_idx];
}

static void Send_Uart_Line(const char *line)
{
  uint32_t start_ms = HAL_GetTick();
  size_t line_len = strlen(line);

  if (line_len > 0U)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)line_len, UART_TX_TIMEOUT_MS);
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2U, UART_TX_TIMEOUT_MS);

  g_last_uart_latency_ms = HAL_GetTick() - start_ms;
}

static void FormatFixedSigned(char *dst, size_t dst_len, int32_t value, uint32_t scale, uint8_t frac_digits)
{
  uint32_t abs_value;
  uint32_t int_part;
  uint32_t frac_part;
  const char *sign = "";

  if (value < 0)
  {
    sign = "-";
    abs_value = (uint32_t)(-value);
  }
  else
  {
    abs_value = (uint32_t)value;
  }

  int_part = abs_value / scale;
  frac_part = abs_value % scale;

  if (frac_digits == 3U)
  {
    (void)snprintf(dst, dst_len, "%s%lu.%03lu", sign, (unsigned long)int_part, (unsigned long)frac_part);
  }
  else
  {
    (void)snprintf(dst, dst_len, "%s%lu.%02lu", sign, (unsigned long)int_part, (unsigned long)frac_part);
  }
}

static uint32_t Calc_MissRate_x100(const TaskStats_t *stat)
{
  uint64_t total_updates;

  total_updates = stat->captured_updates + stat->missed_updates;
  if (total_updates == 0U)
  {
    return 0U;
  }

  return (uint32_t)((stat->missed_updates * 10000ULL) / total_updates);
}

static void Update_MissStats(TaskStats_t *stat, uint32_t now_ms, uint32_t odr_mhz)
{
  uint32_t delta_ms;
  uint64_t elapsed_updates;

  if ((stat == NULL) || (odr_mhz == 0U))
  {
    return;
  }

  if (stat->initialized == 0U)
  {
    stat->initialized = 1U;
    stat->last_poll_tick_ms = now_ms;
    stat->captured_updates = 1U;
    return;
  }

  delta_ms = now_ms - stat->last_poll_tick_ms;
  elapsed_updates = ((uint64_t)delta_ms * (uint64_t)odr_mhz) / 1000000ULL;
  /* Poll executed once now; count one captured sample attempt per poll.
   * Missed updates are estimated from long gaps only (no FIFO assumed). */
  stat->captured_updates++;
  if (elapsed_updates > 1U)
  {
    stat->missed_updates += (elapsed_updates - 1U);
  }
  stat->last_poll_tick_ms = now_ms;
}

static bool Critical_Task_Due(uint32_t now_ms)
{
  if ((int32_t)(now_ms - g_acc_next_tick_ms) >= 0) { return true; }
  if ((int32_t)(now_ms - g_gyro_next_tick_ms) >= 0) { return true; }
  if ((int32_t)(now_ms - g_humidity_next_tick_ms) >= 0) { return true; }
  if ((int32_t)(now_ms - g_temperature_next_tick_ms) >= 0) { return true; }
  if ((int32_t)(now_ms - g_magneto_next_tick_ms) >= 0) { return true; }
  if ((int32_t)(now_ms - g_pressure_next_tick_ms) >= 0) { return true; }
  return false;
}

static void Report_Status(uint32_t now_ms)
{
  char gyro_x_s[16], gyro_y_s[16], gyro_z_s[16];
  char humidity_s[16], temp_s[16], pressure_s[16];
  char mag_x_s[16], mag_y_s[16], mag_z_s[16];
  char p0_s[16], p1_s[16], p2_s[16];
  uint32_t miss_acc_x100, miss_gyro_x100, miss_hum_x100, miss_tmp_x100, miss_mag_x100, miss_prs_x100;
  uint32_t uptime_s = (now_ms - g_system_start_tick_ms) / 1000U;

  int32_t gyro_x_dps_x100 = (int32_t)(g_latest_gyro_mdps.x / 10.0f);
  int32_t gyro_y_dps_x100 = (int32_t)(g_latest_gyro_mdps.y / 10.0f);
  int32_t gyro_z_dps_x100 = (int32_t)(g_latest_gyro_mdps.z / 10.0f);
  int32_t humidity_x100 = (int32_t)(g_latest_humidity_rh * 100.0f);
  int32_t temp_x100 = (int32_t)(g_latest_temperature_c * 100.0f);
  int32_t pressure_x100 = (int32_t)(g_latest_pressure_hpa * 100.0f);
  int32_t p0_x1000 = (int32_t)(g_last_ai_scores[0] * 1000.0f);
  int32_t p1_x1000 = (int32_t)(g_last_ai_scores[1] * 1000.0f);
  int32_t p2_x1000 = (int32_t)(g_last_ai_scores[2] * 1000.0f);

  FormatFixedSigned(gyro_x_s, sizeof(gyro_x_s), gyro_x_dps_x100, 100U, 2U);
  FormatFixedSigned(gyro_y_s, sizeof(gyro_y_s), gyro_y_dps_x100, 100U, 2U);
  FormatFixedSigned(gyro_z_s, sizeof(gyro_z_s), gyro_z_dps_x100, 100U, 2U);
  FormatFixedSigned(humidity_s, sizeof(humidity_s), humidity_x100, 100U, 2U);
  FormatFixedSigned(temp_s, sizeof(temp_s), temp_x100, 100U, 2U);
  FormatFixedSigned(pressure_s, sizeof(pressure_s), pressure_x100, 100U, 2U);
  FormatFixedSigned(mag_x_s, sizeof(mag_x_s), g_latest_magneto_mgauss.x, 1000U, 3U);
  FormatFixedSigned(mag_y_s, sizeof(mag_y_s), g_latest_magneto_mgauss.y, 1000U, 3U);
  FormatFixedSigned(mag_z_s, sizeof(mag_z_s), g_latest_magneto_mgauss.z, 1000U, 3U);
  FormatFixedSigned(p0_s, sizeof(p0_s), p0_x1000, 1000U, 3U);
  FormatFixedSigned(p1_s, sizeof(p1_s), p1_x1000, 1000U, 3U);
  FormatFixedSigned(p2_s, sizeof(p2_s), p2_x1000, 1000U, 3U);

  miss_acc_x100 = Calc_MissRate_x100(&g_stat_accel);
  miss_gyro_x100 = Calc_MissRate_x100(&g_stat_gyro);
  miss_hum_x100 = Calc_MissRate_x100(&g_stat_humidity);
  miss_tmp_x100 = Calc_MissRate_x100(&g_stat_temperature);
  miss_mag_x100 = Calc_MissRate_x100(&g_stat_magneto);
  miss_prs_x100 = Calc_MissRate_x100(&g_stat_pressure);

  (void)snprintf(g_uart_line, sizeof(g_uart_line),
                 "t=%lus ACC[mg]=%ld,%ld,%ld "
                 "GYR[dps]=%s,%s,%s HUM[rH%%]=%s TMP[C]=%s "
                 "MAG[G]=%s,%s,%s P[hPa]=%s "
                 "CLS=%s p=%s,%s,%s infLat=%lums infExec=%lums uartLat=%lums "
                 "miss%% a/g/h/t/m/p=%lu.%02lu/%lu.%02lu/%lu.%02lu/%lu.%02lu/%lu.%02lu/%lu.%02lu "
                 "iq=%u drop=%lu done=%lu iter=%lu"
#if ENABLE_OPTION1_EXTERNAL_TASK
                 " urgentRuns=%lu blkMs=%u defInf=%lu defUart=%lu"
#endif
                 ,
                 (unsigned long)uptime_s,
                 (long)g_latest_accel_mg.x, (long)g_latest_accel_mg.y, (long)g_latest_accel_mg.z,
                 gyro_x_s, gyro_y_s, gyro_z_s,
                 humidity_s, temp_s,
                 mag_x_s, mag_y_s, mag_z_s, pressure_s,
                 g_last_classification,
                 p0_s, p1_s, p2_s,
                 (unsigned long)g_last_inference_latency_ms,
                 (unsigned long)g_last_inference_exec_ms,
                 (unsigned long)g_last_uart_latency_ms,
                 (unsigned long)(miss_acc_x100 / 100U), (unsigned long)(miss_acc_x100 % 100U),
                 (unsigned long)(miss_gyro_x100 / 100U), (unsigned long)(miss_gyro_x100 % 100U),
                 (unsigned long)(miss_hum_x100 / 100U), (unsigned long)(miss_hum_x100 % 100U),
                 (unsigned long)(miss_tmp_x100 / 100U), (unsigned long)(miss_tmp_x100 % 100U),
                 (unsigned long)(miss_mag_x100 / 100U), (unsigned long)(miss_mag_x100 % 100U),
                 (unsigned long)(miss_prs_x100 / 100U), (unsigned long)(miss_prs_x100 % 100U),
                 (unsigned int)g_inference_count,
                 (unsigned long)g_inference_windows_dropped,
                 (unsigned long)g_inference_completed,
                 (unsigned long)g_scheduler_iterations
#if ENABLE_OPTION1_EXTERNAL_TASK
                 , (unsigned long)g_urgent_task_runs, (unsigned int)URGENT_TASK_BLOCK_MS,
                 (unsigned long)g_inference_deferrals, (unsigned long)g_uart_report_deferrals
#endif
                 );

  BSP_LED_Toggle(LED2);
  Send_Uart_Line(g_uart_line);
}

#if ENABLE_OPTION1_EXTERNAL_TASK
static uint8_t Run_Urgent_External_Task_IfDue(uint32_t now_ms)
{
  uint8_t blocked = 0U;

  while ((int32_t)(now_ms - g_urgent_task_next_tick_ms) >= 0)
  {
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < URGENT_TASK_BLOCK_MS)
    {
      /* Intentional blocking: simulates urgent external task load. */
    }

    blocked = 1U;
    g_urgent_task_runs++;
    g_urgent_task_next_tick_ms += URGENT_TASK_PERIOD_MS;
    now_ms = HAL_GetTick();
  }

  return blocked;
}
#endif

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
