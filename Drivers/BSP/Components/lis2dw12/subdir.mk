################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lis2dw12/lis2dw12.c \
../Drivers/BSP/Components/lis2dw12/lis2dw12_reg.c 

OBJS += \
./Drivers/BSP/Components/lis2dw12/lis2dw12.o \
./Drivers/BSP/Components/lis2dw12/lis2dw12_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lis2dw12/lis2dw12.d \
./Drivers/BSP/Components/lis2dw12/lis2dw12_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lis2dw12/%.o Drivers/BSP/Components/lis2dw12/%.su Drivers/BSP/Components/lis2dw12/%.cyclo: ../Drivers/BSP/Components/lis2dw12/%.c Drivers/BSP/Components/lis2dw12/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/lis2dw12 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lis2dw12

clean-Drivers-2f-BSP-2f-Components-2f-lis2dw12:
	-$(RM) ./Drivers/BSP/Components/lis2dw12/lis2dw12.cyclo ./Drivers/BSP/Components/lis2dw12/lis2dw12.d ./Drivers/BSP/Components/lis2dw12/lis2dw12.o ./Drivers/BSP/Components/lis2dw12/lis2dw12.su ./Drivers/BSP/Components/lis2dw12/lis2dw12_reg.cyclo ./Drivers/BSP/Components/lis2dw12/lis2dw12_reg.d ./Drivers/BSP/Components/lis2dw12/lis2dw12_reg.o ./Drivers/BSP/Components/lis2dw12/lis2dw12_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lis2dw12

