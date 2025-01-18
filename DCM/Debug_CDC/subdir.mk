################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../adc.c \
../brakelight.c \
../can.c \
../cmr_error.c \
../controls_23e.c \
../controls_helper.c \
../cvxgen_interface.c \
../daq.c \
../drs_controls.c \
../fans.c \
../gpio.c \
../i2c.c \
../imu.c \
../lut_3d.c \
../main.c \
../mc_power.c \
../motor_efficiency.c \
../motors.c \
../motors_helper.c \
../osqp_interface.c \
../pumps.c \
../safety_filter.c \
../sensors.c \
../servo.c \
../spi.c \
../state.c \
../syscalls.c \
../sysmem.c \
../ve.c 

OBJS += \
./adc.o \
./brakelight.o \
./can.o \
./cmr_error.o \
./controls_23e.o \
./controls_helper.o \
./cvxgen_interface.o \
./daq.o \
./drs_controls.o \
./fans.o \
./gpio.o \
./i2c.o \
./imu.o \
./lut_3d.o \
./main.o \
./mc_power.o \
./motor_efficiency.o \
./motors.o \
./motors_helper.o \
./osqp_interface.o \
./pumps.o \
./safety_filter.o \
./sensors.o \
./servo.o \
./spi.o \
./state.o \
./syscalls.o \
./sysmem.o \
./ve.o 

C_DEPS += \
./adc.d \
./brakelight.d \
./can.d \
./cmr_error.d \
./controls_23e.d \
./controls_helper.d \
./cvxgen_interface.d \
./daq.d \
./drs_controls.d \
./fans.d \
./gpio.d \
./i2c.d \
./imu.d \
./lut_3d.d \
./main.d \
./mc_power.d \
./motor_efficiency.d \
./motors.d \
./motors_helper.d \
./osqp_interface.d \
./pumps.d \
./safety_filter.d \
./sensors.d \
./servo.d \
./spi.d \
./state.d \
./syscalls.d \
./sysmem.d \
./ve.d 


# Each subdirectory must supply rules for building sources it contributes
%.o %.su %.cyclo: ../%.c subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H725xx -DCMR_ENABLE_BOOTLOADER=2 -DconfigASSERT -DH725 -c -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/cvxgen" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/DSP/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/osqp/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/include" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/HAL/H725/Inc" -I"/Users/albertluo/Desktop/edu/cmu/cmr-workspace/CDC/stm32f413-drivers/CMSIS/Device/H725" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@"

clean: clean--2e-

clean--2e-:
	-$(RM) ./adc.cyclo ./adc.d ./adc.o ./adc.su ./brakelight.cyclo ./brakelight.d ./brakelight.o ./brakelight.su ./can.cyclo ./can.d ./can.o ./can.su ./cmr_error.cyclo ./cmr_error.d ./cmr_error.o ./cmr_error.su ./controls_23e.cyclo ./controls_23e.d ./controls_23e.o ./controls_23e.su ./controls_helper.cyclo ./controls_helper.d ./controls_helper.o ./controls_helper.su ./cvxgen_interface.cyclo ./cvxgen_interface.d ./cvxgen_interface.o ./cvxgen_interface.su ./daq.cyclo ./daq.d ./daq.o ./daq.su ./drs_controls.cyclo ./drs_controls.d ./drs_controls.o ./drs_controls.su ./fans.cyclo ./fans.d ./fans.o ./fans.su ./gpio.cyclo ./gpio.d ./gpio.o ./gpio.su ./i2c.cyclo ./i2c.d ./i2c.o ./i2c.su ./imu.cyclo ./imu.d ./imu.o ./imu.su ./lut_3d.cyclo ./lut_3d.d ./lut_3d.o ./lut_3d.su ./main.cyclo ./main.d ./main.o ./main.su ./mc_power.cyclo ./mc_power.d ./mc_power.o ./mc_power.su ./motor_efficiency.cyclo ./motor_efficiency.d ./motor_efficiency.o ./motor_efficiency.su ./motors.cyclo ./motors.d ./motors.o ./motors.su ./motors_helper.cyclo ./motors_helper.d ./motors_helper.o ./motors_helper.su ./osqp_interface.cyclo ./osqp_interface.d ./osqp_interface.o ./osqp_interface.su ./pumps.cyclo ./pumps.d ./pumps.o ./pumps.su ./safety_filter.cyclo ./safety_filter.d ./safety_filter.o ./safety_filter.su ./sensors.cyclo ./sensors.d ./sensors.o ./sensors.su ./servo.cyclo ./servo.d ./servo.o ./servo.su ./spi.cyclo ./spi.d ./spi.o ./spi.su ./state.cyclo ./state.d ./state.o ./state.su ./syscalls.cyclo ./syscalls.d ./syscalls.o ./syscalls.su ./sysmem.cyclo ./sysmem.d ./sysmem.o ./sysmem.su ./ve.cyclo ./ve.d ./ve.o ./ve.su

.PHONY: clean--2e-

