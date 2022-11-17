################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32f413-drivers/HAL/F413/Src/Legacy/stm32f4xx_hal_can.c 

OBJS += \
./stm32f413-drivers/HAL/F413/Src/Legacy/stm32f4xx_hal_can.o 

C_DEPS += \
./stm32f413-drivers/HAL/F413/Src/Legacy/stm32f4xx_hal_can.d 


# Each subdirectory must supply rules for building sources it contributes
stm32f413-drivers/HAL/F413/Src/Legacy/%.o stm32f413-drivers/HAL/F413/Src/Legacy/%.su: ../stm32f413-drivers/HAL/F413/Src/Legacy/%.c stm32f413-drivers/HAL/F413/Src/Legacy/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F413xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DF413 -c -I"C:/Users/CMR/Documents/GitHub/HVC" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/HAL/F413/Inc" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Device/F413" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32f413-2d-drivers-2f-HAL-2f-F413-2f-Src-2f-Legacy

clean-stm32f413-2d-drivers-2f-HAL-2f-F413-2f-Src-2f-Legacy:
	-$(RM) ./stm32f413-drivers/HAL/F413/Src/Legacy/stm32f4xx_hal_can.d ./stm32f413-drivers/HAL/F413/Src/Legacy/stm32f4xx_hal_can.o ./stm32f413-drivers/HAL/F413/Src/Legacy/stm32f4xx_hal_can.su

.PHONY: clean-stm32f413-2d-drivers-2f-HAL-2f-F413-2f-Src-2f-Legacy

