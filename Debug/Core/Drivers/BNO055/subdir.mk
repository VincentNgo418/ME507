################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Drivers/BNO055/bno055.c \
../Core/Drivers/BNO055/bno055_hal.c 

OBJS += \
./Core/Drivers/BNO055/bno055.o \
./Core/Drivers/BNO055/bno055_hal.o 

C_DEPS += \
./Core/Drivers/BNO055/bno055.d \
./Core/Drivers/BNO055/bno055_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Drivers/BNO055/%.o Core/Drivers/BNO055/%.su Core/Drivers/BNO055/%.cyclo: ../Core/Drivers/BNO055/%.c Core/Drivers/BNO055/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Drivers-2f-BNO055

clean-Core-2f-Drivers-2f-BNO055:
	-$(RM) ./Core/Drivers/BNO055/bno055.cyclo ./Core/Drivers/BNO055/bno055.d ./Core/Drivers/BNO055/bno055.o ./Core/Drivers/BNO055/bno055.su ./Core/Drivers/BNO055/bno055_hal.cyclo ./Core/Drivers/BNO055/bno055_hal.d ./Core/Drivers/BNO055/bno055_hal.o ./Core/Drivers/BNO055/bno055_hal.su

.PHONY: clean-Core-2f-Drivers-2f-BNO055

