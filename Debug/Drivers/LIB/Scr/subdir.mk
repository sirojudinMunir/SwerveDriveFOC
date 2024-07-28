################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LIB/Scr/DLPF_lib.c \
../Drivers/LIB/Scr/FLASH_lib.c \
../Drivers/LIB/Scr/PID_lib.c 

OBJS += \
./Drivers/LIB/Scr/DLPF_lib.o \
./Drivers/LIB/Scr/FLASH_lib.o \
./Drivers/LIB/Scr/PID_lib.o 

C_DEPS += \
./Drivers/LIB/Scr/DLPF_lib.d \
./Drivers/LIB/Scr/FLASH_lib.d \
./Drivers/LIB/Scr/PID_lib.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LIB/Scr/%.o Drivers/LIB/Scr/%.su Drivers/LIB/Scr/%.cyclo: ../Drivers/LIB/Scr/%.c Drivers/LIB/Scr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/munir/git/repository/DUAL_BLDC_FOC/Drivers/LIB/Inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-LIB-2f-Scr

clean-Drivers-2f-LIB-2f-Scr:
	-$(RM) ./Drivers/LIB/Scr/DLPF_lib.cyclo ./Drivers/LIB/Scr/DLPF_lib.d ./Drivers/LIB/Scr/DLPF_lib.o ./Drivers/LIB/Scr/DLPF_lib.su ./Drivers/LIB/Scr/FLASH_lib.cyclo ./Drivers/LIB/Scr/FLASH_lib.d ./Drivers/LIB/Scr/FLASH_lib.o ./Drivers/LIB/Scr/FLASH_lib.su ./Drivers/LIB/Scr/PID_lib.cyclo ./Drivers/LIB/Scr/PID_lib.d ./Drivers/LIB/Scr/PID_lib.o ./Drivers/LIB/Scr/PID_lib.su

.PHONY: clean-Drivers-2f-LIB-2f-Scr

