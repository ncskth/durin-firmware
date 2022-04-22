################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32h755zitx.s 

OBJS += \
./Core/Startup/startup_stm32h755zitx.o 

S_DEPS += \
./Core/Startup/startup_stm32h755zitx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32h755zitx.o: ../Core/Startup/startup_stm32h755zitx.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -c -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx" -I"/home/olof/programs/ncs/time_of_flight/h755_vl53l5cx/examples/h755_vl53l5cx/CM7/Core/drivers/vl53l5cx/inc" -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32h755zitx.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

