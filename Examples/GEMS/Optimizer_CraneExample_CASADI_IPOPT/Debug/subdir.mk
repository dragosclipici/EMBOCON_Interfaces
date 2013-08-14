################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../CASADI.cpp 

C_SRCS += \
../EMBOCON_DataTypes.c \
../EMBOCON_ModelInterface.c \
../EMBOCON_OptimizerInterface.c 

OBJS += \
./CASADI.o \
./EMBOCON_DataTypes.o \
./EMBOCON_ModelInterface.o \
./EMBOCON_OptimizerInterface.o 

C_DEPS += \
./EMBOCON_DataTypes.d \
./EMBOCON_ModelInterface.d \
./EMBOCON_OptimizerInterface.d 

CPP_DEPS += \
./CASADI.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -fPIC -I/usr/local/include/casadi -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	g++  -fPIC -I/usr/local/include/casadi -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


