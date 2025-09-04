################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/sensors/dps310.c \
../Core/Inc/sensors/gps6m.c \
../Core/Inc/sensors/icm42688.c 

C_DEPS += \
./Core/Inc/sensors/dps310.d \
./Core/Inc/sensors/gps6m.d \
./Core/Inc/sensors/icm42688.d 

OBJS += \
./Core/Inc/sensors/dps310.o \
./Core/Inc/sensors/gps6m.o \
./Core/Inc/sensors/icm42688.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/sensors/%.o Core/Inc/sensors/%.su Core/Inc/sensors/%.cyclo: ../Core/Inc/sensors/%.c Core/Inc/sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H753xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-sensors

clean-Core-2f-Inc-2f-sensors:
	-$(RM) ./Core/Inc/sensors/dps310.cyclo ./Core/Inc/sensors/dps310.d ./Core/Inc/sensors/dps310.o ./Core/Inc/sensors/dps310.su ./Core/Inc/sensors/gps6m.cyclo ./Core/Inc/sensors/gps6m.d ./Core/Inc/sensors/gps6m.o ./Core/Inc/sensors/gps6m.su ./Core/Inc/sensors/icm42688.cyclo ./Core/Inc/sensors/icm42688.d ./Core/Inc/sensors/icm42688.o ./Core/Inc/sensors/icm42688.su

.PHONY: clean-Core-2f-Inc-2f-sensors

