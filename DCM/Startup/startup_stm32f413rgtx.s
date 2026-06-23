
/**
  ******************************************************************************
  * @file      startup_stm32f413xx.s
  * @author    MCD Application Team
  * @brief     STM32F413xx Devices vector table for GCC based toolchains. 
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M4 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#ifdef F413
  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  g_pfnVectors
.global  Default_Handler

.word  _sidata
.word  _sdata
.word  _edata
.word  _sbss
.word  _ebss

    .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function
Reset_Handler:  
  ldr   sp, =_estack

  movs  r1, #0
  b  LoopCopyDataInit

CopyDataInit:
  ldr  r3, =_sidata
  ldr  r3, [r3, r1]
  str  r3, [r0, r1]
  adds  r1, r1, #4
    
LoopCopyDataInit:
  ldr  r0, =_sdata
  ldr  r3, =_edata
  adds  r2, r0, r1
  cmp  r2, r3
  bcc  CopyDataInit
  ldr  r2, =_sbss
  b  LoopFillZerobss
FillZerobss:
  movs  r3, #0
  str  r3, [r2], #4
    
LoopFillZerobss:
  ldr  r3, = _ebss
  cmp  r2, r3
  bcc  FillZerobss

  bl  SystemInit   
    bl __libc_init_array
  bl  main
  bx  lr    
.size  Reset_Handler, .-Reset_Handler

	.global HardFault_Handler_C
    .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
  ldr r3, NVIC_INT_CTRL_CONST
  ldr r2, [r3, #0]
  uxtb r2, r2
Infinite_Loop:
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler

.align 4
NVIC_INT_CTRL_CONST: .word 0xe000ed04
   .section  .isr_vector,"a",%progbits
  .type  g_pfnVectors, %object
  .size  g_pfnVectors, .-g_pfnVectors
    
g_pfnVectors:
  .word  _estack
  .word  Reset_Handler
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler

  .word     WWDG_IRQHandler
  .word     PVD_IRQHandler
  .word     TAMP_STAMP_IRQHandler
  .word     RTC_WKUP_IRQHandler
  .word     FLASH_IRQHandler
  .word     RCC_IRQHandler
  .word     EXTI0_IRQHandler
  .word     EXTI1_IRQHandler
  .word     EXTI2_IRQHandler
  .word     EXTI3_IRQHandler
  .word     EXTI4_IRQHandler
  .word     DMA1_Stream0_IRQHandler
  .word     DMA1_Stream1_IRQHandler
  .word     DMA1_Stream2_IRQHandler
  .word     DMA1_Stream3_IRQHandler
  .word     DMA1_Stream4_IRQHandler
  .word     DMA1_Stream5_IRQHandler
  .word     DMA1_Stream6_IRQHandler
  .word     ADC_IRQHandler
  .word     CAN1_TX_IRQHandler
  .word     CAN1_RX0_IRQHandler
  .word     CAN1_RX1_IRQHandler
  .word     CAN1_SCE_IRQHandler
  .word     EXTI9_5_IRQHandler
  .word     TIM1_BRK_TIM9_IRQHandler
  .word     TIM1_UP_TIM10_IRQHandler
  .word     TIM1_TRG_COM_TIM11_IRQHandler
  .word     TIM1_CC_IRQHandler
  .word     TIM2_IRQHandler
  .word     TIM3_IRQHandler
  .word     TIM4_IRQHandler
  .word     I2C1_EV_IRQHandler
  .word     I2C1_ER_IRQHandler
  .word     I2C2_EV_IRQHandler
  .word     I2C2_ER_IRQHandler
  .word     SPI1_IRQHandler
  .word     SPI2_IRQHandler
  .word     USART1_IRQHandler
  .word     USART2_IRQHandler
  .word     USART3_IRQHandler
  .word     EXTI15_10_IRQHandler
  .word     RTC_Alarm_IRQHandler
  .word     OTG_FS_WKUP_IRQHandler
  .word     TIM8_BRK_TIM12_IRQHandler
  .word     TIM8_UP_TIM13_IRQHandler
  .word     TIM8_TRG_COM_TIM14_IRQHandler
  .word     TIM8_CC_IRQHandler
  .word     DMA1_Stream7_IRQHandler
  .word     FSMC_IRQHandler
  .word     SDIO_IRQHandler
  .word     TIM5_IRQHandler
  .word     SPI3_IRQHandler
  .word     UART4_IRQHandler
  .word     UART5_IRQHandler
  .word     TIM6_DAC_IRQHandler
  .word     TIM7_IRQHandler
  .word     DMA2_Stream0_IRQHandler
  .word     DMA2_Stream1_IRQHandler
  .word     DMA2_Stream2_IRQHandler
  .word     DMA2_Stream3_IRQHandler
  .word     DMA2_Stream4_IRQHandler
  .word     DFSDM1_FLT0_IRQHandler
  .word     DFSDM1_FLT1_IRQHandler
  .word     CAN2_TX_IRQHandler
  .word     CAN2_RX0_IRQHandler
  .word     CAN2_RX1_IRQHandler
  .word     CAN2_SCE_IRQHandler
  .word     OTG_FS_IRQHandler
  .word     DMA2_Stream5_IRQHandler
  .word     DMA2_Stream6_IRQHandler
  .word     DMA2_Stream7_IRQHandler
  .word     USART6_IRQHandler
  .word     I2C3_EV_IRQHandler
  .word     I2C3_ER_IRQHandler
  .word     CAN3_TX_IRQHandler
  .word     CAN3_RX0_IRQHandler
  .word     CAN3_RX1_IRQHandler
  .word     CAN3_SCE_IRQHandler
  .word     0
  .word     0
  .word     RNG_IRQHandler
  .word     FPU_IRQHandler
  .word     UART7_IRQHandler
  .word     UART8_IRQHandler
  .word     SPI4_IRQHandler
  .word     SPI5_IRQHandler
  .word     0
  .word     SAI1_IRQHandler
  .word     UART9_IRQHandler
  .word     UART10_IRQHandler
  .word     0
  .word     0
  .word     QUADSPI_IRQHandler
  .word     0
  .word     0
  .word     FMPI2C1_EV_IRQHandler
  .word     FMPI2C1_ER_IRQHandler
  .word     LPTIM1_IRQHandler
  .word     DFSDM2_FLT0_IRQHandler
  .word     DFSDM2_FLT1_IRQHandler
  .word     DFSDM2_FLT2_IRQHandler
  .word     DFSDM2_FLT3_IRQHandler
   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler
   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler
   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,Default_Handler
   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,Default_Handler
   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,Default_Handler
   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler
   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,Default_Handler
   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler
   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,Default_Handler
   .weak      WWDG_IRQHandler
   .thumb_set WWDG_IRQHandler,Default_Handler
   .weak      PVD_IRQHandler
   .thumb_set PVD_IRQHandler,Default_Handler
   .weak      TAMP_STAMP_IRQHandler
   .thumb_set TAMP_STAMP_IRQHandler,Default_Handler
   .weak      RTC_WKUP_IRQHandler
   .thumb_set RTC_WKUP_IRQHandler,Default_Handler
   .weak      FLASH_IRQHandler
   .thumb_set FLASH_IRQHandler,Default_Handler
   .weak      RCC_IRQHandler
   .thumb_set RCC_IRQHandler,Default_Handler
   .weak      EXTI0_IRQHandler
   .thumb_set EXTI0_IRQHandler,Default_Handler
   .weak      EXTI1_IRQHandler
   .thumb_set EXTI1_IRQHandler,Default_Handler
   .weak      EXTI2_IRQHandler
   .thumb_set EXTI2_IRQHandler,Default_Handler 
   .weak      EXTI3_IRQHandler
   .thumb_set EXTI3_IRQHandler,Default_Handler
   .weak      EXTI4_IRQHandler
   .thumb_set EXTI4_IRQHandler,Default_Handler
   .weak      DMA1_Stream0_IRQHandler
   .thumb_set DMA1_Stream0_IRQHandler,Default_Handler
   .weak      DMA1_Stream1_IRQHandler
   .thumb_set DMA1_Stream1_IRQHandler,Default_Handler
   .weak      DMA1_Stream2_IRQHandler
   .thumb_set DMA1_Stream2_IRQHandler,Default_Handler
   .weak      DMA1_Stream3_IRQHandler
   .thumb_set DMA1_Stream3_IRQHandler,Default_Handler
   .weak      DMA1_Stream4_IRQHandler
   .thumb_set DMA1_Stream4_IRQHandler,Default_Handler
   .weak      DMA1_Stream5_IRQHandler
   .thumb_set DMA1_Stream5_IRQHandler,Default_Handler
   .weak      DMA1_Stream6_IRQHandler
   .thumb_set DMA1_Stream6_IRQHandler,Default_Handler
   .weak      ADC_IRQHandler
   .thumb_set ADC_IRQHandler,Default_Handler
   .weak      CAN1_TX_IRQHandler
   .thumb_set CAN1_TX_IRQHandler,Default_Handler
   .weak      CAN1_RX0_IRQHandler
   .thumb_set CAN1_RX0_IRQHandler,Default_Handler
   .weak      CAN1_RX1_IRQHandler
   .thumb_set CAN1_RX1_IRQHandler,Default_Handler
   .weak      CAN1_SCE_IRQHandler
   .thumb_set CAN1_SCE_IRQHandler,Default_Handler
   .weak      EXTI9_5_IRQHandler
   .thumb_set EXTI9_5_IRQHandler,Default_Handler
   .weak      TIM1_BRK_TIM9_IRQHandler
   .thumb_set TIM1_BRK_TIM9_IRQHandler,Default_Handler
   .weak      TIM1_UP_TIM10_IRQHandler
   .thumb_set TIM1_UP_TIM10_IRQHandler,Default_Handler
   .weak      TIM1_TRG_COM_TIM11_IRQHandler
   .thumb_set TIM1_TRG_COM_TIM11_IRQHandler,Default_Handler
   .weak      TIM1_CC_IRQHandler
   .thumb_set TIM1_CC_IRQHandler,Default_Handler
   .weak      TIM2_IRQHandler
   .thumb_set TIM2_IRQHandler,Default_Handler
   .weak      TIM3_IRQHandler
   .thumb_set TIM3_IRQHandler,Default_Handler
   .weak      TIM4_IRQHandler
   .thumb_set TIM4_IRQHandler,Default_Handler
   .weak      I2C1_EV_IRQHandler
   .thumb_set I2C1_EV_IRQHandler,Default_Handler
   .weak      I2C1_ER_IRQHandler
   .thumb_set I2C1_ER_IRQHandler,Default_Handler
   .weak      I2C2_EV_IRQHandler
   .thumb_set I2C2_EV_IRQHandler,Default_Handler
   .weak      I2C2_ER_IRQHandler
   .thumb_set I2C2_ER_IRQHandler,Default_Handler
   .weak      SPI1_IRQHandler
   .thumb_set SPI1_IRQHandler,Default_Handler
   .weak      SPI2_IRQHandler
   .thumb_set SPI2_IRQHandler,Default_Handler
   .weak      USART1_IRQHandler
   .thumb_set USART1_IRQHandler,Default_Handler
   .weak      USART2_IRQHandler
   .thumb_set USART2_IRQHandler,Default_Handler
   .weak      USART3_IRQHandler
   .thumb_set USART3_IRQHandler,Default_Handler
   .weak      EXTI15_10_IRQHandler
   .thumb_set EXTI15_10_IRQHandler,Default_Handler
   .weak      RTC_Alarm_IRQHandler
   .thumb_set RTC_Alarm_IRQHandler,Default_Handler
   .weak      OTG_FS_WKUP_IRQHandler
   .thumb_set OTG_FS_WKUP_IRQHandler,Default_Handler
   .weak      TIM8_BRK_TIM12_IRQHandler
   .thumb_set TIM8_BRK_TIM12_IRQHandler,Default_Handler
   .weak      TIM8_UP_TIM13_IRQHandler
   .thumb_set TIM8_UP_TIM13_IRQHandler,Default_Handler
   .weak      TIM8_TRG_COM_TIM14_IRQHandler
   .thumb_set TIM8_TRG_COM_TIM14_IRQHandler,Default_Handler
   .weak      TIM8_CC_IRQHandler
   .thumb_set TIM8_CC_IRQHandler,Default_Handler
   .weak      DMA1_Stream7_IRQHandler
   .thumb_set DMA1_Stream7_IRQHandler,Default_Handler
   .weak      FSMC_IRQHandler
   .thumb_set FSMC_IRQHandler,Default_Handler   
   .weak      SDIO_IRQHandler
   .thumb_set SDIO_IRQHandler,Default_Handler
   .weak      TIM5_IRQHandler
   .thumb_set TIM5_IRQHandler,Default_Handler
   .weak      SPI3_IRQHandler
   .thumb_set SPI3_IRQHandler,Default_Handler
   .weak      UART4_IRQHandler
   .thumb_set UART4_IRQHandler,Default_Handler
   .weak      UART5_IRQHandler
   .thumb_set UART5_IRQHandler,Default_Handler   
   .weak      TIM6_DAC_IRQHandler
   .thumb_set TIM6_DAC_IRQHandler,Default_Handler
   .weak      TIM7_IRQHandler
   .thumb_set TIM7_IRQHandler,Default_Handler
   .weak      DMA2_Stream0_IRQHandler
   .thumb_set DMA2_Stream0_IRQHandler,Default_Handler
   .weak      DMA2_Stream1_IRQHandler
   .thumb_set DMA2_Stream1_IRQHandler,Default_Handler
   .weak      DMA2_Stream2_IRQHandler
   .thumb_set DMA2_Stream2_IRQHandler,Default_Handler
   .weak      DMA2_Stream3_IRQHandler
   .thumb_set DMA2_Stream3_IRQHandler,Default_Handler
   .weak      DMA2_Stream4_IRQHandler
   .thumb_set DMA2_Stream4_IRQHandler,Default_Handler
   .weak      DFSDM1_FLT0_IRQHandler
   .thumb_set DFSDM1_FLT0_IRQHandler,Default_Handler
   .weak      DFSDM1_FLT1_IRQHandler
   .thumb_set DFSDM1_FLT1_IRQHandler,Default_Handler
   .weak      CAN2_TX_IRQHandler
   .thumb_set CAN2_TX_IRQHandler,Default_Handler
   .weak      CAN2_RX0_IRQHandler
   .thumb_set CAN2_RX0_IRQHandler,Default_Handler
   .weak      CAN2_RX1_IRQHandler
   .thumb_set CAN2_RX1_IRQHandler,Default_Handler
   .weak      CAN2_SCE_IRQHandler
   .thumb_set CAN2_SCE_IRQHandler,Default_Handler
   .weak      OTG_FS_IRQHandler
   .thumb_set OTG_FS_IRQHandler,Default_Handler
   .weak      DMA2_Stream5_IRQHandler
   .thumb_set DMA2_Stream5_IRQHandler,Default_Handler
   .weak      DMA2_Stream6_IRQHandler
   .thumb_set DMA2_Stream6_IRQHandler,Default_Handler
   .weak      DMA2_Stream7_IRQHandler
   .thumb_set DMA2_Stream7_IRQHandler,Default_Handler
   .weak      USART6_IRQHandler
   .thumb_set USART6_IRQHandler,Default_Handler
   .weak      I2C3_EV_IRQHandler
   .thumb_set I2C3_EV_IRQHandler,Default_Handler
   .weak      I2C3_ER_IRQHandler
   .thumb_set I2C3_ER_IRQHandler,Default_Handler
   .weak      CAN3_TX_IRQHandler
   .thumb_set CAN3_TX_IRQHandler,Default_Handler
   .weak      CAN3_RX0_IRQHandler
   .thumb_set CAN3_RX0_IRQHandler,Default_Handler
   .weak      CAN3_RX1_IRQHandler
   .thumb_set CAN3_RX1_IRQHandler,Default_Handler
   .weak      CAN3_SCE_IRQHandler
   .thumb_set CAN3_SCE_IRQHandler,Default_Handler   
   .weak      RNG_IRQHandler
   .thumb_set RNG_IRQHandler,Default_Handler
   .weak      FPU_IRQHandler
   .thumb_set FPU_IRQHandler,Default_Handler
   .weak      UART7_IRQHandler
   .thumb_set UART7_IRQHandler,Default_Handler
   .weak      UART8_IRQHandler
   .thumb_set UART8_IRQHandler,Default_Handler   
   .weak      SPI4_IRQHandler
   .thumb_set SPI4_IRQHandler,Default_Handler
   .weak      SPI5_IRQHandler
   .thumb_set SPI5_IRQHandler,Default_Handler
   .weak      SAI1_IRQHandler
   .thumb_set SAI1_IRQHandler,Default_Handler
   .weak      UART9_IRQHandler
   .thumb_set UART9_IRQHandler,Default_Handler
   .weak      UART10_IRQHandler
   .thumb_set UART10_IRQHandler,Default_Handler   
   .weak      QUADSPI_IRQHandler
   .thumb_set QUADSPI_IRQHandler,Default_Handler
    .weak     FMPI2C1_EV_IRQHandler
   .thumb_set FMPI2C1_EV_IRQHandler,Default_Handler
   .weak      FMPI2C1_ER_IRQHandler
   .thumb_set FMPI2C1_ER_IRQHandler,Default_Handler
   .weak      LPTIM1_IRQHandler
   .thumb_set LPTIM1_IRQHandler,Default_Handler
   .weak      DFSDM2_FLT0_IRQHandler
   .thumb_set DFSDM2_FLT0_IRQHandler,Default_Handler
   .weak      DFSDM2_FLT1_IRQHandler
   .thumb_set DFSDM2_FLT1_IRQHandler,Default_Handler
   .weak      DFSDM2_FLT2_IRQHandler
   .thumb_set DFSDM2_FLT2_IRQHandler,Default_Handler
   .weak      DFSDM2_FLT3_IRQHandler
   .thumb_set DFSDM2_FLT3_IRQHandler,Default_Handler
#endif /* F413 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/