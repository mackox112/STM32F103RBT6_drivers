################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/gpio_led.c \
../drivers/Src/stm32f103rb_gpio_driver.c \
../drivers/Src/test_practice.c 

OBJS += \
./drivers/Src/gpio_led.o \
./drivers/Src/stm32f103rb_gpio_driver.o \
./drivers/Src/test_practice.o 

C_DEPS += \
./drivers/Src/gpio_led.d \
./drivers/Src/stm32f103rb_gpio_driver.d \
./drivers/Src/test_practice.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/gpio_led.o: ../drivers/Src/gpio_led.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -c -I../Inc -I"C:/Users/macie/OneDrive/Pulpit/STM32_Workspace/stm32f103rb_driver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/gpio_led.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103rb_gpio_driver.o: ../drivers/Src/stm32f103rb_gpio_driver.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -c -I../Inc -I"C:/Users/macie/OneDrive/Pulpit/STM32_Workspace/stm32f103rb_driver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103rb_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/test_practice.o: ../drivers/Src/test_practice.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -c -I../Inc -I"C:/Users/macie/OneDrive/Pulpit/STM32_Workspace/stm32f103rb_driver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/test_practice.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

