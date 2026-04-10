/**
  ******************************************************************************
  * @file    stm32l475e_iot01_accelero.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the accelerometer sensor
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l475e_iot01_accelero.h"
/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32L475E_IOT01
  * @{
  */

/** @defgroup STM32L475E_IOT01_ACCELERO ACCELERO
  * @{
  */

/** @defgroup STM32L475E_IOT01_ACCELERO_Private_Variables ACCELERO Private Variables
  * @{
  */
LSM6DSL_Object_t g_lsm6dsl_obj;
static uint8_t g_lsm6dsl_is_ready = 0U;
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_ACCELERO_Private_FunctionPrototypes ACCELERO Private Function Prototypes
  * @{
  */
static int32_t LSM6DSL_PlatformInit(void);
static int32_t LSM6DSL_PlatformDeInit(void);
static int32_t LSM6DSL_PlatformGetTick(void);
static int32_t LSM6DSL_PlatformWriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t LSM6DSL_PlatformReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);

/* Sensor IO bridge functions are implemented in stm32l475e_iot01.c */
extern void SENSOR_IO_Init(void);
extern void SENSOR_IO_DeInit(void);
extern uint16_t SENSOR_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
extern void SENSOR_IO_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
/**
  * @}
  */

int32_t BSP_LSM6DSL_EnsureInit(void)
{
  LSM6DSL_IO_t io_ctx;
  uint8_t id = 0U;

  if (g_lsm6dsl_is_ready == 1U)
  {
    return LSM6DSL_OK;
  }

  io_ctx.Init = LSM6DSL_PlatformInit;
  io_ctx.DeInit = LSM6DSL_PlatformDeInit;
  io_ctx.BusType = LSM6DSL_I2C_BUS;
  io_ctx.Address = (uint8_t)(LSM6DSL_I2C_ADD_L & 0xFEU); /* HAL uses 8-bit address without R/W bit. */
  io_ctx.WriteReg = LSM6DSL_PlatformWriteReg;
  io_ctx.ReadReg = LSM6DSL_PlatformReadReg;
  io_ctx.GetTick = LSM6DSL_PlatformGetTick;
  io_ctx.Delay = HAL_Delay;

  if (LSM6DSL_RegisterBusIO(&g_lsm6dsl_obj, &io_ctx) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }
  if (LSM6DSL_Init(&g_lsm6dsl_obj) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }
  if (LSM6DSL_ReadID(&g_lsm6dsl_obj, &id) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }
  if (id != LSM6DSL_ID)
  {
    return LSM6DSL_ERROR;
  }

  g_lsm6dsl_is_ready = 1U;
  return LSM6DSL_OK;
}

/** @defgroup STM32L475E_IOT01_ACCELERO_Private_Functions ACCELERO Private Functions
  * @{
  */ 
/**
  * @brief  Initialize the ACCELERO.
  * @retval ACCELERO_OK or ACCELERO_ERROR
  */
ACCELERO_StatusTypeDef BSP_ACCELERO_Init(void)
{
  if (BSP_LSM6DSL_EnsureInit() != LSM6DSL_OK)
  {
    return ACCELERO_ERROR;
  }
  if (LSM6DSL_ACC_SetFullScale(&g_lsm6dsl_obj, 2) != LSM6DSL_OK)
  {
    return ACCELERO_ERROR;
  }
  if (LSM6DSL_ACC_SetOutputDataRate(&g_lsm6dsl_obj, 26.0f) != LSM6DSL_OK)
  {
    return ACCELERO_ERROR;
  }
  if (LSM6DSL_ACC_Enable(&g_lsm6dsl_obj) != LSM6DSL_OK)
  {
    return ACCELERO_ERROR;
  }

  return ACCELERO_OK;
}

/**
  * @brief  DeInitialize the ACCELERO.
  * @retval None.
  */
void BSP_ACCELERO_DeInit(void)
{
  if (g_lsm6dsl_is_ready == 1U)
  {
    (void)LSM6DSL_ACC_Disable(&g_lsm6dsl_obj);
  }
}

/**
  * @brief  Set/Unset the ACCELERO in low power mode.
  * @param  status 0 means disable Low Power Mode, otherwise Low Power Mode is enabled
  * @retval None
  */
void BSP_ACCELERO_LowPower(uint16_t status)
{
  if (g_lsm6dsl_is_ready == 1U)
  {
    /* Keep baseline default at 26 Hz when not in low power mode. */
    (void)LSM6DSL_ACC_SetOutputDataRate(&g_lsm6dsl_obj, (status != 0U) ? 13.0f : 26.0f);
  }
}

/**
  * @brief  Get XYZ acceleration values.
  * @param  pDataXYZ Pointer on 3 angular accelerations table with  
  *                  pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis
  * @retval None
  */
void BSP_ACCELERO_AccGetXYZ(int16_t *pDataXYZ)
{
  LSM6DSL_Axes_t axes;

  if ((pDataXYZ == NULL) || (g_lsm6dsl_is_ready == 0U))
  {
    return;
  }

  if (LSM6DSL_ACC_GetAxes(&g_lsm6dsl_obj, &axes) == LSM6DSL_OK)
  {
    pDataXYZ[0] = (int16_t)axes.x;
    pDataXYZ[1] = (int16_t)axes.y;
    pDataXYZ[2] = (int16_t)axes.z;
  }
  else
  {
    pDataXYZ[0] = 0;
    pDataXYZ[1] = 0;
    pDataXYZ[2] = 0;
  }
}

static int32_t LSM6DSL_PlatformInit(void)
{
  SENSOR_IO_Init();
  return LSM6DSL_OK;
}

static int32_t LSM6DSL_PlatformDeInit(void)
{
  SENSOR_IO_DeInit();
  return LSM6DSL_OK;
}

static int32_t LSM6DSL_PlatformGetTick(void)
{
  return (int32_t)HAL_GetTick();
}

static int32_t LSM6DSL_PlatformWriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  SENSOR_IO_WriteMultiple((uint8_t)Addr, (uint8_t)Reg, pData, Length);
  return LSM6DSL_OK;
}

static int32_t LSM6DSL_PlatformReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  return (SENSOR_IO_ReadMultiple((uint8_t)Addr, (uint8_t)Reg, pData, Length) == HAL_OK) ? LSM6DSL_OK : LSM6DSL_ERROR;
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
