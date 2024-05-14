################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/BayesFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/CommonTables" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/DistanceFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/InterpolationFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/SupportFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Middlewares/Third_Party/ARM_CMSIS/Source/TransformFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/BasicMathFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/BayesFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/CommonTables" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/ComplexMathFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/ControllerFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/DistanceFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/FastMathFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/FilteringFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/InterpolationFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/MatrixFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/QuaternionMathFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/StatisticsFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/SupportFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/SVMFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/TransformFunctions" -I"C:/Users/bcc35/Desktop/Elec/T2_week5.3/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-FastMathFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-FastMathFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-FastMathFunctions

