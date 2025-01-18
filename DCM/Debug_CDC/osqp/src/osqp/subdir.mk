################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../osqp/src/osqp/auxil.c \
../osqp/src/osqp/error.c \
../osqp/src/osqp/kkt.c \
../osqp/src/osqp/lin_alg.c \
../osqp/src/osqp/osqp.c \
../osqp/src/osqp/proj.c \
../osqp/src/osqp/qdldl.c \
../osqp/src/osqp/qdldl_interface.c \
../osqp/src/osqp/scaling.c \
../osqp/src/osqp/util.c \
../osqp/src/osqp/workspace.c 

OBJS += \
./osqp/src/osqp/auxil.o \
./osqp/src/osqp/error.o \
./osqp/src/osqp/kkt.o \
./osqp/src/osqp/lin_alg.o \
./osqp/src/osqp/osqp.o \
./osqp/src/osqp/proj.o \
./osqp/src/osqp/qdldl.o \
./osqp/src/osqp/qdldl_interface.o \
./osqp/src/osqp/scaling.o \
./osqp/src/osqp/util.o \
./osqp/src/osqp/workspace.o 

C_DEPS += \
./osqp/src/osqp/auxil.d \
./osqp/src/osqp/error.d \
./osqp/src/osqp/kkt.d \
./osqp/src/osqp/lin_alg.d \
./osqp/src/osqp/osqp.d \
./osqp/src/osqp/proj.d \
./osqp/src/osqp/qdldl.d \
./osqp/src/osqp/qdldl_interface.d \
./osqp/src/osqp/scaling.d \
./osqp/src/osqp/util.d \
./osqp/src/osqp/workspace.d 


# Each subdirectory must supply rules for building sources it contributes
osqp/src/osqp/%.o osqp/src/osqp/%.su osqp/src/osqp/%.cyclo: ../osqp/src/osqp/%.c osqp/src/osqp/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H725xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DH725 -c -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/cvxgen" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/DSP/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/osqp/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/HAL/H725/Inc" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Device/H725" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@"

clean: clean-osqp-2f-src-2f-osqp

clean-osqp-2f-src-2f-osqp:
	-$(RM) ./osqp/src/osqp/auxil.cyclo ./osqp/src/osqp/auxil.d ./osqp/src/osqp/auxil.o ./osqp/src/osqp/auxil.su ./osqp/src/osqp/error.cyclo ./osqp/src/osqp/error.d ./osqp/src/osqp/error.o ./osqp/src/osqp/error.su ./osqp/src/osqp/kkt.cyclo ./osqp/src/osqp/kkt.d ./osqp/src/osqp/kkt.o ./osqp/src/osqp/kkt.su ./osqp/src/osqp/lin_alg.cyclo ./osqp/src/osqp/lin_alg.d ./osqp/src/osqp/lin_alg.o ./osqp/src/osqp/lin_alg.su ./osqp/src/osqp/osqp.cyclo ./osqp/src/osqp/osqp.d ./osqp/src/osqp/osqp.o ./osqp/src/osqp/osqp.su ./osqp/src/osqp/proj.cyclo ./osqp/src/osqp/proj.d ./osqp/src/osqp/proj.o ./osqp/src/osqp/proj.su ./osqp/src/osqp/qdldl.cyclo ./osqp/src/osqp/qdldl.d ./osqp/src/osqp/qdldl.o ./osqp/src/osqp/qdldl.su ./osqp/src/osqp/qdldl_interface.cyclo ./osqp/src/osqp/qdldl_interface.d ./osqp/src/osqp/qdldl_interface.o ./osqp/src/osqp/qdldl_interface.su ./osqp/src/osqp/scaling.cyclo ./osqp/src/osqp/scaling.d ./osqp/src/osqp/scaling.o ./osqp/src/osqp/scaling.su ./osqp/src/osqp/util.cyclo ./osqp/src/osqp/util.d ./osqp/src/osqp/util.o ./osqp/src/osqp/util.su ./osqp/src/osqp/workspace.cyclo ./osqp/src/osqp/workspace.d ./osqp/src/osqp/workspace.o ./osqp/src/osqp/workspace.su

.PHONY: clean-osqp-2f-src-2f-osqp

