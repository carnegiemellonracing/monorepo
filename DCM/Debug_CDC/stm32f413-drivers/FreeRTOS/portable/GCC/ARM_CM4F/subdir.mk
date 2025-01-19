################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/port.c 

OBJS += \
./stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/port.o 

C_DEPS += \
./stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/port.d 


# Each subdirectory must supply rules for building sources it contributes
stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/%.o stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/%.su stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/%.cyclo: ../stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/%.c stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H725xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DH725 -c -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/cvxgen" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/DSP/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/osqp/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/HAL/H725/Inc" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Device/H725" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32f413-2d-drivers-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM4F

clean-stm32f413-2d-drivers-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM4F:
	-$(RM) ./stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/port.cyclo ./stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/port.d ./stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/port.o ./stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F/port.su

.PHONY: clean-stm32f413-2d-drivers-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM4F

