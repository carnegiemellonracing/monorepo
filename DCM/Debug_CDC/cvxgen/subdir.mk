################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../cvxgen/ldl.c \
../cvxgen/matrix_support.c \
../cvxgen/solver.c \
../cvxgen/util.c 

OBJS += \
./cvxgen/ldl.o \
./cvxgen/matrix_support.o \
./cvxgen/solver.o \
./cvxgen/util.o 

C_DEPS += \
./cvxgen/ldl.d \
./cvxgen/matrix_support.d \
./cvxgen/solver.d \
./cvxgen/util.d 


# Each subdirectory must supply rules for building sources it contributes
cvxgen/%.o cvxgen/%.su cvxgen/%.cyclo: ../cvxgen/%.c cvxgen/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H725xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DH725 -c -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/cvxgen" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/DSP/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/osqp/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/HAL/H725/Inc" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Device/H725" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@"

clean: clean-cvxgen

clean-cvxgen:
	-$(RM) ./cvxgen/ldl.cyclo ./cvxgen/ldl.d ./cvxgen/ldl.o ./cvxgen/ldl.su ./cvxgen/matrix_support.cyclo ./cvxgen/matrix_support.d ./cvxgen/matrix_support.o ./cvxgen/matrix_support.su ./cvxgen/solver.cyclo ./cvxgen/solver.d ./cvxgen/solver.o ./cvxgen/solver.su ./cvxgen/util.cyclo ./cvxgen/util.d ./cvxgen/util.o ./cvxgen/util.su

.PHONY: clean-cvxgen

