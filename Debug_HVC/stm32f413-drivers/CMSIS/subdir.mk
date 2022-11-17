################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../stm32f413-drivers/CMSIS/startup_stm32f413xx.s \
../stm32f413-drivers/CMSIS/startup_stm32l431xx.s 

C_SRCS += \
../stm32f413-drivers/CMSIS/system_stm32f4xx.c \
../stm32f413-drivers/CMSIS/system_stm32l4xx.c 

OBJS += \
./stm32f413-drivers/CMSIS/startup_stm32f413xx.o \
./stm32f413-drivers/CMSIS/startup_stm32l431xx.o \
./stm32f413-drivers/CMSIS/system_stm32f4xx.o \
./stm32f413-drivers/CMSIS/system_stm32l4xx.o 

S_DEPS += \
./stm32f413-drivers/CMSIS/startup_stm32f413xx.d \
./stm32f413-drivers/CMSIS/startup_stm32l431xx.d 

C_DEPS += \
./stm32f413-drivers/CMSIS/system_stm32f4xx.d \
./stm32f413-drivers/CMSIS/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
stm32f413-drivers/CMSIS/%.o: ../stm32f413-drivers/CMSIS/%.s stm32f413-drivers/CMSIS/subdir.mk
	arm-none-eabi-gcc -c -mcpu=cortex-m4 -g3 -DSTM32F413xx -DF413 -c -I"C:/Users/CMR/Documents/GitHub/HVC" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Device/F413" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/HAL/F413/Inc" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
stm32f413-drivers/CMSIS/%.o stm32f413-drivers/CMSIS/%.su: ../stm32f413-drivers/CMSIS/%.c stm32f413-drivers/CMSIS/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F413xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DF413 -c -I"C:/Users/CMR/Documents/GitHub/HVC" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/HAL/F413/Inc" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Device/F413" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32f413-2d-drivers-2f-CMSIS

clean-stm32f413-2d-drivers-2f-CMSIS:
	-$(RM) ./stm32f413-drivers/CMSIS/startup_stm32f413xx.d ./stm32f413-drivers/CMSIS/startup_stm32f413xx.o ./stm32f413-drivers/CMSIS/startup_stm32l431xx.d ./stm32f413-drivers/CMSIS/startup_stm32l431xx.o ./stm32f413-drivers/CMSIS/system_stm32f4xx.d ./stm32f413-drivers/CMSIS/system_stm32f4xx.o ./stm32f413-drivers/CMSIS/system_stm32f4xx.su ./stm32f413-drivers/CMSIS/system_stm32l4xx.d ./stm32f413-drivers/CMSIS/system_stm32l4xx.o ./stm32f413-drivers/CMSIS/system_stm32l4xx.su

.PHONY: clean-stm32f413-2d-drivers-2f-CMSIS

