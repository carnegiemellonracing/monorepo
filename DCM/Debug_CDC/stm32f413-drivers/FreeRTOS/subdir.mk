################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32f413-drivers/FreeRTOS/croutine.c \
../stm32f413-drivers/FreeRTOS/event_groups.c \
../stm32f413-drivers/FreeRTOS/list.c \
../stm32f413-drivers/FreeRTOS/queue.c \
../stm32f413-drivers/FreeRTOS/tasks.c \
../stm32f413-drivers/FreeRTOS/timers.c 

OBJS += \
./stm32f413-drivers/FreeRTOS/croutine.o \
./stm32f413-drivers/FreeRTOS/event_groups.o \
./stm32f413-drivers/FreeRTOS/list.o \
./stm32f413-drivers/FreeRTOS/queue.o \
./stm32f413-drivers/FreeRTOS/tasks.o \
./stm32f413-drivers/FreeRTOS/timers.o 

C_DEPS += \
./stm32f413-drivers/FreeRTOS/croutine.d \
./stm32f413-drivers/FreeRTOS/event_groups.d \
./stm32f413-drivers/FreeRTOS/list.d \
./stm32f413-drivers/FreeRTOS/queue.d \
./stm32f413-drivers/FreeRTOS/tasks.d \
./stm32f413-drivers/FreeRTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
stm32f413-drivers/FreeRTOS/%.o stm32f413-drivers/FreeRTOS/%.su stm32f413-drivers/FreeRTOS/%.cyclo: ../stm32f413-drivers/FreeRTOS/%.c stm32f413-drivers/FreeRTOS/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H725xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DH725 -c -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/cvxgen" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/DSP/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/osqp/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/HAL/H725/Inc" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Device/H725" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32f413-2d-drivers-2f-FreeRTOS

clean-stm32f413-2d-drivers-2f-FreeRTOS:
	-$(RM) ./stm32f413-drivers/FreeRTOS/croutine.cyclo ./stm32f413-drivers/FreeRTOS/croutine.d ./stm32f413-drivers/FreeRTOS/croutine.o ./stm32f413-drivers/FreeRTOS/croutine.su ./stm32f413-drivers/FreeRTOS/event_groups.cyclo ./stm32f413-drivers/FreeRTOS/event_groups.d ./stm32f413-drivers/FreeRTOS/event_groups.o ./stm32f413-drivers/FreeRTOS/event_groups.su ./stm32f413-drivers/FreeRTOS/list.cyclo ./stm32f413-drivers/FreeRTOS/list.d ./stm32f413-drivers/FreeRTOS/list.o ./stm32f413-drivers/FreeRTOS/list.su ./stm32f413-drivers/FreeRTOS/queue.cyclo ./stm32f413-drivers/FreeRTOS/queue.d ./stm32f413-drivers/FreeRTOS/queue.o ./stm32f413-drivers/FreeRTOS/queue.su ./stm32f413-drivers/FreeRTOS/tasks.cyclo ./stm32f413-drivers/FreeRTOS/tasks.d ./stm32f413-drivers/FreeRTOS/tasks.o ./stm32f413-drivers/FreeRTOS/tasks.su ./stm32f413-drivers/FreeRTOS/timers.cyclo ./stm32f413-drivers/FreeRTOS/timers.d ./stm32f413-drivers/FreeRTOS/timers.o ./stm32f413-drivers/FreeRTOS/timers.su

.PHONY: clean-stm32f413-2d-drivers-2f-FreeRTOS

