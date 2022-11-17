################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
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
stm32f413-drivers/FreeRTOS/%.o stm32f413-drivers/FreeRTOS/%.su: ../stm32f413-drivers/FreeRTOS/%.c stm32f413-drivers/FreeRTOS/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F413xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DF413 -c -I"C:/Users/CMR/Documents/GitHub/HVC" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/HAL/F413/Inc" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Device/F413" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32f413-2d-drivers-2f-FreeRTOS

clean-stm32f413-2d-drivers-2f-FreeRTOS:
	-$(RM) ./stm32f413-drivers/FreeRTOS/croutine.d ./stm32f413-drivers/FreeRTOS/croutine.o ./stm32f413-drivers/FreeRTOS/croutine.su ./stm32f413-drivers/FreeRTOS/event_groups.d ./stm32f413-drivers/FreeRTOS/event_groups.o ./stm32f413-drivers/FreeRTOS/event_groups.su ./stm32f413-drivers/FreeRTOS/list.d ./stm32f413-drivers/FreeRTOS/list.o ./stm32f413-drivers/FreeRTOS/list.su ./stm32f413-drivers/FreeRTOS/queue.d ./stm32f413-drivers/FreeRTOS/queue.o ./stm32f413-drivers/FreeRTOS/queue.su ./stm32f413-drivers/FreeRTOS/tasks.d ./stm32f413-drivers/FreeRTOS/tasks.o ./stm32f413-drivers/FreeRTOS/tasks.su ./stm32f413-drivers/FreeRTOS/timers.d ./stm32f413-drivers/FreeRTOS/timers.o ./stm32f413-drivers/FreeRTOS/timers.su

.PHONY: clean-stm32f413-2d-drivers-2f-FreeRTOS

