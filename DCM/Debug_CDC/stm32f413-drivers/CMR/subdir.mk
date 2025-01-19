################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
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
../stm32f413-drivers/CMR/fir_filter.c \
../stm32f413-drivers/CMR/gpio.c \
../stm32f413-drivers/CMR/h725.c \
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
./stm32f413-drivers/CMR/fir_filter.o \
./stm32f413-drivers/CMR/gpio.o \
./stm32f413-drivers/CMR/h725.o \
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
./stm32f413-drivers/CMR/fir_filter.d \
./stm32f413-drivers/CMR/gpio.d \
./stm32f413-drivers/CMR/h725.d \
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
stm32f413-drivers/CMR/%.o stm32f413-drivers/CMR/%.su stm32f413-drivers/CMR/%.cyclo: ../stm32f413-drivers/CMR/%.c stm32f413-drivers/CMR/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H725xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DH725 -c -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/cvxgen" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/DSP/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/osqp/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/HAL/H725/Inc" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Device/H725" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32f413-2d-drivers-2f-CMR

clean-stm32f413-2d-drivers-2f-CMR:
	-$(RM) ./stm32f413-drivers/CMR/adc.cyclo ./stm32f413-drivers/CMR/adc.d ./stm32f413-drivers/CMR/adc.o ./stm32f413-drivers/CMR/adc.su ./stm32f413-drivers/CMR/can.cyclo ./stm32f413-drivers/CMR/can.d ./stm32f413-drivers/CMR/can.o ./stm32f413-drivers/CMR/can.su ./stm32f413-drivers/CMR/config.cyclo ./stm32f413-drivers/CMR/config.d ./stm32f413-drivers/CMR/config.o ./stm32f413-drivers/CMR/config.su ./stm32f413-drivers/CMR/config_screen_helper.cyclo ./stm32f413-drivers/CMR/config_screen_helper.d ./stm32f413-drivers/CMR/config_screen_helper.o ./stm32f413-drivers/CMR/config_screen_helper.su ./stm32f413-drivers/CMR/dma.cyclo ./stm32f413-drivers/CMR/dma.d ./stm32f413-drivers/CMR/dma.o ./stm32f413-drivers/CMR/dma.su ./stm32f413-drivers/CMR/dsp.cyclo ./stm32f413-drivers/CMR/dsp.d ./stm32f413-drivers/CMR/dsp.o ./stm32f413-drivers/CMR/dsp.su ./stm32f413-drivers/CMR/f413.cyclo ./stm32f413-drivers/CMR/f413.d ./stm32f413-drivers/CMR/f413.o ./stm32f413-drivers/CMR/f413.su ./stm32f413-drivers/CMR/fir_filter.cyclo ./stm32f413-drivers/CMR/fir_filter.d ./stm32f413-drivers/CMR/fir_filter.o ./stm32f413-drivers/CMR/fir_filter.su ./stm32f413-drivers/CMR/gpio.cyclo ./stm32f413-drivers/CMR/gpio.d ./stm32f413-drivers/CMR/gpio.o ./stm32f413-drivers/CMR/gpio.su ./stm32f413-drivers/CMR/h725.cyclo ./stm32f413-drivers/CMR/h725.d ./stm32f413-drivers/CMR/h725.o ./stm32f413-drivers/CMR/h725.su ./stm32f413-drivers/CMR/i2c.cyclo ./stm32f413-drivers/CMR/i2c.d ./stm32f413-drivers/CMR/i2c.o ./stm32f413-drivers/CMR/i2c.su ./stm32f413-drivers/CMR/interrupts.cyclo ./stm32f413-drivers/CMR/interrupts.d ./stm32f413-drivers/CMR/interrupts.o ./stm32f413-drivers/CMR/interrupts.su ./stm32f413-drivers/CMR/l431.cyclo ./stm32f413-drivers/CMR/l431.d ./stm32f413-drivers/CMR/l431.o ./stm32f413-drivers/CMR/l431.su ./stm32f413-drivers/CMR/panic.cyclo ./stm32f413-drivers/CMR/panic.d ./stm32f413-drivers/CMR/panic.o ./stm32f413-drivers/CMR/panic.su ./stm32f413-drivers/CMR/pwm.cyclo ./stm32f413-drivers/CMR/pwm.d ./stm32f413-drivers/CMR/pwm.o ./stm32f413-drivers/CMR/pwm.su ./stm32f413-drivers/CMR/qspi.cyclo ./stm32f413-drivers/CMR/qspi.d ./stm32f413-drivers/CMR/qspi.o ./stm32f413-drivers/CMR/qspi.su ./stm32f413-drivers/CMR/rcc.cyclo ./stm32f413-drivers/CMR/rcc.d ./stm32f413-drivers/CMR/rcc.o ./stm32f413-drivers/CMR/rcc.su ./stm32f413-drivers/CMR/sensors.cyclo ./stm32f413-drivers/CMR/sensors.d ./stm32f413-drivers/CMR/sensors.o ./stm32f413-drivers/CMR/sensors.su ./stm32f413-drivers/CMR/spi.cyclo ./stm32f413-drivers/CMR/spi.d ./stm32f413-drivers/CMR/spi.o ./stm32f413-drivers/CMR/spi.su ./stm32f413-drivers/CMR/tasks.cyclo ./stm32f413-drivers/CMR/tasks.d ./stm32f413-drivers/CMR/tasks.o ./stm32f413-drivers/CMR/tasks.su ./stm32f413-drivers/CMR/uart.cyclo ./stm32f413-drivers/CMR/uart.d ./stm32f413-drivers/CMR/uart.o ./stm32f413-drivers/CMR/uart.su ./stm32f413-drivers/CMR/watchdog.cyclo ./stm32f413-drivers/CMR/watchdog.d ./stm32f413-drivers/CMR/watchdog.o ./stm32f413-drivers/CMR/watchdog.su

.PHONY: clean-stm32f413-2d-drivers-2f-CMR

