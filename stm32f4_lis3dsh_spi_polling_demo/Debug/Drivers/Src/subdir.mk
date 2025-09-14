################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/gpio_driver.c \
../Drivers/Src/lis3dsh.c \
../Drivers/Src/spi_driver.c 

OBJS += \
./Drivers/Src/gpio_driver.o \
./Drivers/Src/lis3dsh.o \
./Drivers/Src/spi_driver.o 

C_DEPS += \
./Drivers/Src/gpio_driver.d \
./Drivers/Src/lis3dsh.d \
./Drivers/Src/spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/nikhi/Desktop/Embedded C/my_workspace/target/stm32f4_lis3dsh_spi_polling_demo/Drivers/Inc" -I"C:/Users/nikhi/Desktop/Embedded C/my_workspace/target/stm32f4_lis3dsh_spi_polling_demo/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/gpio_driver.cyclo ./Drivers/Src/gpio_driver.d ./Drivers/Src/gpio_driver.o ./Drivers/Src/gpio_driver.su ./Drivers/Src/lis3dsh.cyclo ./Drivers/Src/lis3dsh.d ./Drivers/Src/lis3dsh.o ./Drivers/Src/lis3dsh.su ./Drivers/Src/spi_driver.cyclo ./Drivers/Src/spi_driver.d ./Drivers/Src/spi_driver.o ./Drivers/Src/spi_driver.su

.PHONY: clean-Drivers-2f-Src

