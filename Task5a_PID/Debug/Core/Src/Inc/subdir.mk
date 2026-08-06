################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Inc/Color.c 

OBJS += \
./Core/Src/Inc/Color.o 

C_DEPS += \
./Core/Src/Inc/Color.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Inc/%.o Core/Src/Inc/%.su Core/Src/Inc/%.cyclo: ../Core/Src/Inc/%.c Core/Src/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Inc

clean-Core-2f-Src-2f-Inc:
	-$(RM) ./Core/Src/Inc/Color.cyclo ./Core/Src/Inc/Color.d ./Core/Src/Inc/Color.o ./Core/Src/Inc/Color.su

.PHONY: clean-Core-2f-Src-2f-Inc

