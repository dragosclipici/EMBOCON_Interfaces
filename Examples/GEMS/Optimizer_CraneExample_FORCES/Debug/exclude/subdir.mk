################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../exclude/gfcn.c \
../exclude/grad_jfcn.c \
../exclude/hess_lfcn.c \
../exclude/jac_gfcn.c \
../exclude/jfcn.c 

OBJS += \
./exclude/gfcn.o \
./exclude/grad_jfcn.o \
./exclude/hess_lfcn.o \
./exclude/jac_gfcn.o \
./exclude/jfcn.o 

C_DEPS += \
./exclude/gfcn.d \
./exclude/grad_jfcn.d \
./exclude/hess_lfcn.d \
./exclude/jac_gfcn.d \
./exclude/jfcn.d 


# Each subdirectory must supply rules for building sources it contributes
exclude/%.o: ../exclude/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc  -fPIC -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


