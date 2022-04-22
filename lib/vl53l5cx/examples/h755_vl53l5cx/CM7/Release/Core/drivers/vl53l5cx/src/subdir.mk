################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/drivers/vl53l5cx/src/platform.c \
../Core/drivers/vl53l5cx/src/vl53l5cx_api.c \
../Core/drivers/vl53l5cx/src/vl53l5cx_plugin_async.c \
../Core/drivers/vl53l5cx/src/vl53l5cx_plugin_detection_thresholds.c \
../Core/drivers/vl53l5cx/src/vl53l5cx_plugin_motion_indicator.c \
../Core/drivers/vl53l5cx/src/vl53l5cx_plugin_xtalk.c 

OBJS += \
./Core/drivers/vl53l5cx/src/platform.o \
./Core/drivers/vl53l5cx/src/vl53l5cx_api.o \
./Core/drivers/vl53l5cx/src/vl53l5cx_plugin_async.o \
./Core/drivers/vl53l5cx/src/vl53l5cx_plugin_detection_thresholds.o \
./Core/drivers/vl53l5cx/src/vl53l5cx_plugin_motion_indicator.o \
./Core/drivers/vl53l5cx/src/vl53l5cx_plugin_xtalk.o 

C_DEPS += \
./Core/drivers/vl53l5cx/src/platform.d \
./Core/drivers/vl53l5cx/src/vl53l5cx_api.d \
./Core/drivers/vl53l5cx/src/vl53l5cx_plugin_async.d \
./Core/drivers/vl53l5cx/src/vl53l5cx_plugin_detection_thresholds.d \
./Core/drivers/vl53l5cx/src/vl53l5cx_plugin_motion_indicator.d \
./Core/drivers/vl53l5cx/src/vl53l5cx_plugin_xtalk.d 


# Each subdirectory must supply rules for building sources it contributes
Core/drivers/vl53l5cx/src/platform.o: ../Core/drivers/vl53l5cx/src/platform.c Core/drivers/vl53l5cx/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx/inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/drivers/vl53l5cx/src/platform.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/drivers/vl53l5cx/src/vl53l5cx_api.o: ../Core/drivers/vl53l5cx/src/vl53l5cx_api.c Core/drivers/vl53l5cx/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx/inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/drivers/vl53l5cx/src/vl53l5cx_api.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/drivers/vl53l5cx/src/vl53l5cx_plugin_async.o: ../Core/drivers/vl53l5cx/src/vl53l5cx_plugin_async.c Core/drivers/vl53l5cx/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx/inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/drivers/vl53l5cx/src/vl53l5cx_plugin_async.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/drivers/vl53l5cx/src/vl53l5cx_plugin_detection_thresholds.o: ../Core/drivers/vl53l5cx/src/vl53l5cx_plugin_detection_thresholds.c Core/drivers/vl53l5cx/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx/inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/drivers/vl53l5cx/src/vl53l5cx_plugin_detection_thresholds.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/drivers/vl53l5cx/src/vl53l5cx_plugin_motion_indicator.o: ../Core/drivers/vl53l5cx/src/vl53l5cx_plugin_motion_indicator.c Core/drivers/vl53l5cx/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx/inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/drivers/vl53l5cx/src/vl53l5cx_plugin_motion_indicator.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/drivers/vl53l5cx/src/vl53l5cx_plugin_xtalk.o: ../Core/drivers/vl53l5cx/src/vl53l5cx_plugin_xtalk.c Core/drivers/vl53l5cx/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx/inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/drivers/vl53l5cx/src/vl53l5cx_plugin_xtalk.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

