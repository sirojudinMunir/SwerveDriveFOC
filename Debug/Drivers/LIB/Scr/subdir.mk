################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LIB/Scr/DLPF_lib.c \
../Drivers/LIB/Scr/FLASH_lib.c \
../Drivers/LIB/Scr/MAGNETIC_SENSOR_AS5048A.c \
../Drivers/LIB/Scr/PID_lib.c \
../Drivers/LIB/Scr/SWERVE_DRIVE_BLDC.c \
../Drivers/LIB/Scr/SWERVE_DRIVE_CAN.c \
../Drivers/LIB/Scr/SWERVE_DRIVE_FOC.c \
../Drivers/LIB/Scr/TASK_BLINK.c \
../Drivers/LIB/Scr/TASK_COMMAND.c 

OBJS += \
./Drivers/LIB/Scr/DLPF_lib.o \
./Drivers/LIB/Scr/FLASH_lib.o \
./Drivers/LIB/Scr/MAGNETIC_SENSOR_AS5048A.o \
./Drivers/LIB/Scr/PID_lib.o \
./Drivers/LIB/Scr/SWERVE_DRIVE_BLDC.o \
./Drivers/LIB/Scr/SWERVE_DRIVE_CAN.o \
./Drivers/LIB/Scr/SWERVE_DRIVE_FOC.o \
./Drivers/LIB/Scr/TASK_BLINK.o \
./Drivers/LIB/Scr/TASK_COMMAND.o 

C_DEPS += \
./Drivers/LIB/Scr/DLPF_lib.d \
./Drivers/LIB/Scr/FLASH_lib.d \
./Drivers/LIB/Scr/MAGNETIC_SENSOR_AS5048A.d \
./Drivers/LIB/Scr/PID_lib.d \
./Drivers/LIB/Scr/SWERVE_DRIVE_BLDC.d \
./Drivers/LIB/Scr/SWERVE_DRIVE_CAN.d \
./Drivers/LIB/Scr/SWERVE_DRIVE_FOC.d \
./Drivers/LIB/Scr/TASK_BLINK.d \
./Drivers/LIB/Scr/TASK_COMMAND.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LIB/Scr/%.o Drivers/LIB/Scr/%.su Drivers/LIB/Scr/%.cyclo: ../Drivers/LIB/Scr/%.c Drivers/LIB/Scr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/munir/git/SwerveDriveFOC/DUAL_BLDC_FOC/Drivers/LIB/Inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-LIB-2f-Scr

clean-Drivers-2f-LIB-2f-Scr:
	-$(RM) ./Drivers/LIB/Scr/DLPF_lib.cyclo ./Drivers/LIB/Scr/DLPF_lib.d ./Drivers/LIB/Scr/DLPF_lib.o ./Drivers/LIB/Scr/DLPF_lib.su ./Drivers/LIB/Scr/FLASH_lib.cyclo ./Drivers/LIB/Scr/FLASH_lib.d ./Drivers/LIB/Scr/FLASH_lib.o ./Drivers/LIB/Scr/FLASH_lib.su ./Drivers/LIB/Scr/MAGNETIC_SENSOR_AS5048A.cyclo ./Drivers/LIB/Scr/MAGNETIC_SENSOR_AS5048A.d ./Drivers/LIB/Scr/MAGNETIC_SENSOR_AS5048A.o ./Drivers/LIB/Scr/MAGNETIC_SENSOR_AS5048A.su ./Drivers/LIB/Scr/PID_lib.cyclo ./Drivers/LIB/Scr/PID_lib.d ./Drivers/LIB/Scr/PID_lib.o ./Drivers/LIB/Scr/PID_lib.su ./Drivers/LIB/Scr/SWERVE_DRIVE_BLDC.cyclo ./Drivers/LIB/Scr/SWERVE_DRIVE_BLDC.d ./Drivers/LIB/Scr/SWERVE_DRIVE_BLDC.o ./Drivers/LIB/Scr/SWERVE_DRIVE_BLDC.su ./Drivers/LIB/Scr/SWERVE_DRIVE_CAN.cyclo ./Drivers/LIB/Scr/SWERVE_DRIVE_CAN.d ./Drivers/LIB/Scr/SWERVE_DRIVE_CAN.o ./Drivers/LIB/Scr/SWERVE_DRIVE_CAN.su ./Drivers/LIB/Scr/SWERVE_DRIVE_FOC.cyclo ./Drivers/LIB/Scr/SWERVE_DRIVE_FOC.d ./Drivers/LIB/Scr/SWERVE_DRIVE_FOC.o ./Drivers/LIB/Scr/SWERVE_DRIVE_FOC.su ./Drivers/LIB/Scr/TASK_BLINK.cyclo ./Drivers/LIB/Scr/TASK_BLINK.d ./Drivers/LIB/Scr/TASK_BLINK.o ./Drivers/LIB/Scr/TASK_BLINK.su ./Drivers/LIB/Scr/TASK_COMMAND.cyclo ./Drivers/LIB/Scr/TASK_COMMAND.d ./Drivers/LIB/Scr/TASK_COMMAND.o ./Drivers/LIB/Scr/TASK_COMMAND.su

.PHONY: clean-Drivers-2f-LIB-2f-Scr

