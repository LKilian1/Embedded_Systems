################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lis2mdl/lis2mdl.c \
../Drivers/BSP/Components/lis2mdl/lis2mdl_reg.c 

OBJS += \
./Drivers/BSP/Components/lis2mdl/lis2mdl.o \
./Drivers/BSP/Components/lis2mdl/lis2mdl_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lis2mdl/lis2mdl.d \
./Drivers/BSP/Components/lis2mdl/lis2mdl_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lis2mdl/%.o Drivers/BSP/Components/lis2mdl/%.su Drivers/BSP/Components/lis2mdl/%.cyclo: ../Drivers/BSP/Components/lis2mdl/%.c Drivers/BSP/Components/lis2mdl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/lis2dw12 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lis2mdl

clean-Drivers-2f-BSP-2f-Components-2f-lis2mdl:
	-$(RM) ./Drivers/BSP/Components/lis2mdl/lis2mdl.cyclo ./Drivers/BSP/Components/lis2mdl/lis2mdl.d ./Drivers/BSP/Components/lis2mdl/lis2mdl.o ./Drivers/BSP/Components/lis2mdl/lis2mdl.su ./Drivers/BSP/Components/lis2mdl/lis2mdl_reg.cyclo ./Drivers/BSP/Components/lis2mdl/lis2mdl_reg.d ./Drivers/BSP/Components/lis2mdl/lis2mdl_reg.o ./Drivers/BSP/Components/lis2mdl/lis2mdl_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lis2mdl

