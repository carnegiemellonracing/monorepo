################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../stm32f413-drivers/CMSIS/startup_stm32f413xx.s \
../stm32f413-drivers/CMSIS/startup_stm32h725xx.s \
../stm32f413-drivers/CMSIS/startup_stm32l431xx.s 

C_SRCS += \
../stm32f413-drivers/CMSIS/system_stm32f4xx.c \
../stm32f413-drivers/CMSIS/system_stm32h7xx.c \
../stm32f413-drivers/CMSIS/system_stm32l4xx.c 

OBJS += \
./stm32f413-drivers/CMSIS/startup_stm32f413xx.o \
./stm32f413-drivers/CMSIS/startup_stm32h725xx.o \
./stm32f413-drivers/CMSIS/startup_stm32l431xx.o \
./stm32f413-drivers/CMSIS/system_stm32f4xx.o \
./stm32f413-drivers/CMSIS/system_stm32h7xx.o \
./stm32f413-drivers/CMSIS/system_stm32l4xx.o 

S_DEPS += \
./stm32f413-drivers/CMSIS/startup_stm32f413xx.d \
./stm32f413-drivers/CMSIS/startup_stm32h725xx.d \
./stm32f413-drivers/CMSIS/startup_stm32l431xx.d 

C_DEPS += \
./stm32f413-drivers/CMSIS/system_stm32f4xx.d \
./stm32f413-drivers/CMSIS/system_stm32h7xx.d \
./stm32f413-drivers/CMSIS/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
stm32f413-drivers/CMSIS/%.o: ../stm32f413-drivers/CMSIS/%.s stm32f413-drivers/CMSIS/subdir.mk
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -g3 -DSTM32H725xx -DH725 -c -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/cvxgen" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/DSP/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/osqp/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Device/H725" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/HAL/H725/Inc" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@" "$<"
stm32f413-drivers/CMSIS/%.o stm32f413-drivers/CMSIS/%.su stm32f413-drivers/CMSIS/%.cyclo: ../stm32f413-drivers/CMSIS/%.c stm32f413-drivers/CMSIS/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H725xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DH725 -c -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/cvxgen" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/DSP/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/osqp/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/HAL/H725/Inc" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Device/H725" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32f413-2d-drivers-2f-CMSIS

clean-stm32f413-2d-drivers-2f-CMSIS:
	-$(RM) ./stm32f413-drivers/CMSIS/startup_stm32f413xx.d ./stm32f413-drivers/CMSIS/startup_stm32f413xx.o ./stm32f413-drivers/CMSIS/startup_stm32h725xx.d ./stm32f413-drivers/CMSIS/startup_stm32h725xx.o ./stm32f413-drivers/CMSIS/startup_stm32l431xx.d ./stm32f413-drivers/CMSIS/startup_stm32l431xx.o ./stm32f413-drivers/CMSIS/system_stm32f4xx.cyclo ./stm32f413-drivers/CMSIS/system_stm32f4xx.d ./stm32f413-drivers/CMSIS/system_stm32f4xx.o ./stm32f413-drivers/CMSIS/system_stm32f4xx.su ./stm32f413-drivers/CMSIS/system_stm32h7xx.cyclo ./stm32f413-drivers/CMSIS/system_stm32h7xx.d ./stm32f413-drivers/CMSIS/system_stm32h7xx.o ./stm32f413-drivers/CMSIS/system_stm32h7xx.su ./stm32f413-drivers/CMSIS/system_stm32l4xx.cyclo ./stm32f413-drivers/CMSIS/system_stm32l4xx.d ./stm32f413-drivers/CMSIS/system_stm32l4xx.o ./stm32f413-drivers/CMSIS/system_stm32l4xx.su

.PHONY: clean-stm32f413-2d-drivers-2f-CMSIS

