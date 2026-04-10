/**
  ******************************************************************************
  * @file    stm32l475e_iot01_gyro.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the gyroscope sensor
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
#include "stm32l475e_iot01_gyro.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L475E_IOT01
  * @{
  */
      
/** @defgroup STM32L475E_IOT01_GYROSCOPE GYROSCOPE
  * @{
  */ 

extern LSM6DSL_Object_t g_lsm6dsl_obj;
int32_t BSP_LSM6DSL_EnsureInit(void);


/** @defgroup STM32L475E_IOT01_GYROSCOPE_Private_Functions GYROSCOPE Private Functions
  * @{
  */ 
/**
  * @brief  Initialize Gyroscope.
  * @retval GYRO_OK or GYRO_ERROR
  */
uint8_t BSP_GYRO_Init(void)
{
  if (BSP_LSM6DSL_EnsureInit() != LSM6DSL_OK)
  {
    return GYRO_ERROR;
  }
  if (LSM6DSL_GYRO_SetFullScale(&g_lsm6dsl_obj, 2000) != LSM6DSL_OK)
  {
    return GYRO_ERROR;
  }
  /* Minimum supported continuous gyroscope ODR for this assignment: 12.5 Hz. */
  if (LSM6DSL_GYRO_SetOutputDataRate(&g_lsm6dsl_obj, 12.5f) != LSM6DSL_OK)
  {
    return GYRO_ERROR;
  }
  if (LSM6DSL_GYRO_Enable(&g_lsm6dsl_obj) != LSM6DSL_OK)
  {
    return GYRO_ERROR;
  }

  return GYRO_OK;
}


/**
  * @brief  DeInitialize Gyroscope.
  */
void BSP_GYRO_DeInit(void)
{
  (void)LSM6DSL_GYRO_Disable(&g_lsm6dsl_obj);
}


/**
  * @brief  Set/Unset Gyroscope in low power mode.
  * @param  status 0 means disable Low Power Mode, otherwise Low Power Mode is enabled
  */
void BSP_GYRO_LowPower(uint16_t status)
{
  (void)status;
  (void)LSM6DSL_GYRO_SetOutputDataRate(&g_lsm6dsl_obj, 12.5f);
}

/**
  * @brief  Get XYZ angular acceleration from the Gyroscope.
  * @param  pfData: pointer on floating array         
  */
void BSP_GYRO_GetXYZ(float* pfData)
{
  LSM6DSL_Axes_t axes;

  if (pfData == NULL)
  {
    return;
  }

  if (LSM6DSL_GYRO_GetAxes(&g_lsm6dsl_obj, &axes) == LSM6DSL_OK)
  {
    /* Keep legacy API behavior: output values in mdps as float. */
    pfData[0] = (float)axes.x;
    pfData[1] = (float)axes.y;
    pfData[2] = (float)axes.z;
  }
  else
  {
    pfData[0] = 0.0f;
    pfData[1] = 0.0f;
    pfData[2] = 0.0f;
  }
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
