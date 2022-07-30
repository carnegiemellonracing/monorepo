/**
  ******************************************************************************
<<<<<<<< HEAD:HAL/L431/Inc/stm32l4xx_hal_flash_ramfunc.h
  * @file    stm32l4xx_hal_flash_ramfunc.h
========
  * @file    stm32f4xx_hal_flash_ramfunc.h
>>>>>>>> origin/alternative_stms:HAL/F413/Inc/stm32f4xx_hal_flash_ramfunc.h
  * @author  MCD Application Team
  * @brief   Header file of FLASH RAMFUNC driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
<<<<<<<< HEAD:HAL/L431/Inc/stm32l4xx_hal_flash_ramfunc.h
  *                       opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4xx_FLASH_RAMFUNC_H
#define __STM32L4xx_FLASH_RAMFUNC_H
========
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_FLASH_RAMFUNC_H
#define __STM32F4xx_FLASH_RAMFUNC_H
>>>>>>>> origin/alternative_stms:HAL/F413/Inc/stm32f4xx_hal_flash_ramfunc.h

#ifdef __cplusplus
 extern "C" {
#endif
<<<<<<<< HEAD:HAL/L431/Inc/stm32l4xx_hal_flash_ramfunc.h

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal_def.h"

/** @addtogroup STM32L4xx_HAL_Driver
========
#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F412Zx) ||\
    defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx)  

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_def.h"

/** @addtogroup STM32F4xx_HAL_Driver
>>>>>>>> origin/alternative_stms:HAL/F413/Inc/stm32f4xx_hal_flash_ramfunc.h
  * @{
  */

/** @addtogroup FLASH_RAMFUNC
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup FLASH_RAMFUNC_Exported_Functions
  * @{
  */

/** @addtogroup FLASH_RAMFUNC_Exported_Functions_Group1
  * @{
<<<<<<<< HEAD:HAL/L431/Inc/stm32l4xx_hal_flash_ramfunc.h
  */
/* Peripheral Control functions  ************************************************/
__RAM_FUNC  HAL_FLASHEx_EnableRunPowerDown(void);
__RAM_FUNC  HAL_FLASHEx_DisableRunPowerDown(void);
#if defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)
__RAM_FUNC  HAL_FLASHEx_OB_DBankConfig(uint32_t DBankConfig);
#endif
/**
  * @}
  */
========
  */   
__RAM_FUNC HAL_StatusTypeDef HAL_FLASHEx_StopFlashInterfaceClk(void);
__RAM_FUNC HAL_StatusTypeDef HAL_FLASHEx_StartFlashInterfaceClk(void);
__RAM_FUNC HAL_StatusTypeDef HAL_FLASHEx_EnableFlashSleepMode(void);
__RAM_FUNC HAL_StatusTypeDef HAL_FLASHEx_DisableFlashSleepMode(void);
/**
  * @}
  */ 
>>>>>>>> origin/alternative_stms:HAL/F413/Inc/stm32f4xx_hal_flash_ramfunc.h

/**
  * @}
  */

/**
  * @}
<<<<<<<< HEAD:HAL/L431/Inc/stm32l4xx_hal_flash_ramfunc.h
  */
========
  */ 
>>>>>>>> origin/alternative_stms:HAL/F413/Inc/stm32f4xx_hal_flash_ramfunc.h

/**
  * @}
  */

<<<<<<<< HEAD:HAL/L431/Inc/stm32l4xx_hal_flash_ramfunc.h
========
#endif /* STM32F410xx || STM32F411xE || STM32F446xx || STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F412Cx */  
>>>>>>>> origin/alternative_stms:HAL/F413/Inc/stm32f4xx_hal_flash_ramfunc.h
#ifdef __cplusplus
}
#endif

<<<<<<<< HEAD:HAL/L431/Inc/stm32l4xx_hal_flash_ramfunc.h
#endif /* __STM32L4xx_FLASH_RAMFUNC_H */
========

#endif /* __STM32F4xx_FLASH_RAMFUNC_H */
>>>>>>>> origin/alternative_stms:HAL/F413/Inc/stm32f4xx_hal_flash_ramfunc.h

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
