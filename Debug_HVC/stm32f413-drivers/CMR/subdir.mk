################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32f413-drivers/CMR/adc.c \
../stm32f413-drivers/CMR/can.c \
../stm32f413-drivers/CMR/config.c \
../stm32f413-drivers/CMR/config_screen_helper.c \
../stm32f413-drivers/CMR/dma.c \
../stm32f413-drivers/CMR/dsp.c \
../stm32f413-drivers/CMR/f413.c \
../stm32f413-drivers/CMR/gpio.c \
../stm32f413-drivers/CMR/i2c.c \
../stm32f413-drivers/CMR/interrupts.c \
../stm32f413-drivers/CMR/l431.c \
../stm32f413-drivers/CMR/panic.c \
../stm32f413-drivers/CMR/pwm.c \
../stm32f413-drivers/CMR/qspi.c \
../stm32f413-drivers/CMR/rcc.c \
../stm32f413-drivers/CMR/sensors.c \
../stm32f413-drivers/CMR/spi.c \
../stm32f413-drivers/CMR/tasks.c \
../stm32f413-drivers/CMR/uart.c \
../stm32f413-drivers/CMR/watchdog.c 

OBJS += \
./stm32f413-drivers/CMR/adc.o \
./stm32f413-drivers/CMR/can.o \
./stm32f413-drivers/CMR/config.o \
./stm32f413-drivers/CMR/config_screen_helper.o \
./stm32f413-drivers/CMR/dma.o \
./stm32f413-drivers/CMR/dsp.o \
./stm32f413-drivers/CMR/f413.o \
./stm32f413-drivers/CMR/gpio.o \
./stm32f413-drivers/CMR/i2c.o \
./stm32f413-drivers/CMR/interrupts.o \
./stm32f413-drivers/CMR/l431.o \
./stm32f413-drivers/CMR/panic.o \
./stm32f413-drivers/CMR/pwm.o \
./stm32f413-drivers/CMR/qspi.o \
./stm32f413-drivers/CMR/rcc.o \
./stm32f413-drivers/CMR/sensors.o \
./stm32f413-drivers/CMR/spi.o \
./stm32f413-drivers/CMR/tasks.o \
./stm32f413-drivers/CMR/uart.o \
./stm32f413-drivers/CMR/watchdog.o 

C_DEPS += \
./stm32f413-drivers/CMR/adc.d \
./stm32f413-drivers/CMR/can.d \
./stm32f413-drivers/CMR/config.d \
./stm32f413-drivers/CMR/config_screen_helper.d \
./stm32f413-drivers/CMR/dma.d \
./stm32f413-drivers/CMR/dsp.d \
./stm32f413-drivers/CMR/f413.d \
./stm32f413-drivers/CMR/gpio.d \
./stm32f413-drivers/CMR/i2c.d \
./stm32f413-drivers/CMR/interrupts.d \
./stm32f413-drivers/CMR/l431.d \
./stm32f413-drivers/CMR/panic.d \
./stm32f413-drivers/CMR/pwm.d \
./stm32f413-drivers/CMR/qspi.d \
./stm32f413-drivers/CMR/rcc.d \
./stm32f413-drivers/CMR/sensors.d \
./stm32f413-drivers/CMR/spi.d \
./stm32f413-drivers/CMR/tasks.d \
./stm32f413-drivers/CMR/uart.d \
./stm32f413-drivers/CMR/watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
stm32f413-drivers/CMR/%.o stm32f413-drivers/CMR/%.su: ../stm32f413-drivers/CMR/%.c stm32f413-drivers/CMR/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F413xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DF413 -c -I"C:/Users/CMR/Documents/GitHub/HVC" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/include" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/HAL/F413/Inc" -I"C:/Users/CMR/Documents/GitHub/HVC/stm32f413-drivers/CMSIS/Device/F413" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32f413-2d-drivers-2f-CMR

clean-stm32f413-2d-drivers-2f-CMR:
	-$(RM) ./stm32f413-drivers/CMR/adc.d ./stm32f413-drivers/CMR/adc.o ./stm32f413-drivers/CMR/adc.su ./stm32f413-drivers/CMR/can.d ./stm32f413-drivers/CMR/can.o ./stm32f413-drivers/CMR/can.su ./stm32f413-drivers/CMR/config.d ./stm32f413-drivers/CMR/config.o ./stm32f413-drivers/CMR/config.su ./stm32f413-drivers/CMR/config_screen_helper.d ./stm32f413-drivers/CMR/config_screen_helper.o ./stm32f413-drivers/CMR/config_screen_helper.su ./stm32f413-drivers/CMR/dma.d ./stm32f413-drivers/CMR/dma.o ./stm32f413-drivers/CMR/dma.su ./stm32f413-drivers/CMR/dsp.d ./stm32f413-drivers/CMR/dsp.o ./stm32f413-drivers/CMR/dsp.su ./stm32f413-drivers/CMR/f413.d ./stm32f413-drivers/CMR/f413.o ./stm32f413-drivers/CMR/f413.su ./stm32f413-drivers/CMR/gpio.d ./stm32f413-drivers/CMR/gpio.o ./stm32f413-drivers/CMR/gpio.su ./stm32f413-drivers/CMR/i2c.d ./stm32f413-drivers/CMR/i2c.o ./stm32f413-drivers/CMR/i2c.su ./stm32f413-drivers/CMR/interrupts.d ./stm32f413-drivers/CMR/interrupts.o ./stm32f413-drivers/CMR/interrupts.su ./stm32f413-drivers/CMR/l431.d ./stm32f413-drivers/CMR/l431.o ./stm32f413-drivers/CMR/l431.su ./stm32f413-drivers/CMR/panic.d ./stm32f413-drivers/CMR/panic.o ./stm32f413-drivers/CMR/panic.su ./stm32f413-drivers/CMR/pwm.d ./stm32f413-drivers/CMR/pwm.o ./stm32f413-drivers/CMR/pwm.su ./stm32f413-drivers/CMR/qspi.d ./stm32f413-drivers/CMR/qspi.o ./stm32f413-drivers/CMR/qspi.su ./stm32f413-drivers/CMR/rcc.d ./stm32f413-drivers/CMR/rcc.o ./stm32f413-drivers/CMR/rcc.su ./stm32f413-drivers/CMR/sensors.d ./stm32f413-drivers/CMR/sensors.o ./stm32f413-drivers/CMR/sensors.su ./stm32f413-drivers/CMR/spi.d ./stm32f413-drivers/CMR/spi.o ./stm32f413-drivers/CMR/spi.su ./stm32f413-drivers/CMR/tasks.d ./stm32f413-drivers/CMR/tasks.o ./stm32f413-drivers/CMR/tasks.su ./stm32f413-drivers/CMR/uart.d ./stm32f413-drivers/CMR/uart.o ./stm32f413-drivers/CMR/uart.su ./stm32f413-drivers/CMR/watchdog.d ./stm32f413-drivers/CMR/watchdog.o ./stm32f413-drivers/CMR/watchdog.su

.PHONY: clean-stm32f413-2d-drivers-2f-CMR

