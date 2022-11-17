################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BMB_task.c \
../adc.c \
../bms_error.c \
../bms_relay.c \
../can.c \
../crc.c \
../gpio.c \
../i2c.c \
../main.c \
../sensors.c \
../state_task.c \
../watchdog.c 

OBJS += \
./BMB_task.o \
./adc.o \
./bms_error.o \
./bms_relay.o \
./can.o \
./crc.o \
./gpio.o \
./i2c.o \
./main.o \
./sensors.o \
./state_task.o \
./watchdog.o 

C_DEPS += \
./BMB_task.d \
./adc.d \
./bms_error.d \
./bms_relay.d \
./can.d \
./crc.d \
./gpio.d \
./i2c.d \
./main.d \
./sensors.d \
./state_task.d \
./watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
%.o %.su: ../%.c subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F413xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DF413 -c -I"C:/Users/CMR/Documents/GitHub/HVC" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/HAL/F413/Inc" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Device/F413" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean--2e-

clean--2e-:
	-$(RM) ./BMB_task.d ./BMB_task.o ./BMB_task.su ./adc.d ./adc.o ./adc.su ./bms_error.d ./bms_error.o ./bms_error.su ./bms_relay.d ./bms_relay.o ./bms_relay.su ./can.d ./can.o ./can.su ./crc.d ./crc.o ./crc.su ./gpio.d ./gpio.o ./gpio.su ./i2c.d ./i2c.o ./i2c.su ./main.d ./main.o ./main.su ./sensors.d ./sensors.o ./sensors.su ./state_task.d ./state_task.o ./state_task.su ./watchdog.d ./watchdog.o ./watchdog.su

.PHONY: clean--2e-

