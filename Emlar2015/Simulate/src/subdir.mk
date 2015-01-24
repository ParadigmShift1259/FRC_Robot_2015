################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/AccelToDisp.cpp \
../src/GyroPID.cpp \
../src/MechanumDriveTrain.cpp \
../src/OI.cpp \
../src/Robot.cpp 

OBJS += \
./src/AccelToDisp.o \
./src/GyroPID.o \
./src/MechanumDriveTrain.o \
./src/OI.o \
./src/Robot.o 

CPP_DEPS += \
./src/AccelToDisp.d \
./src/GyroPID.d \
./src/MechanumDriveTrain.d \
./src/OI.d \
./src/Robot.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"C:\Users\Programming.Robotics1259\git\FRC_Robot_2015\Emlar2015/src" -IC:\Users\Programming.Robotics1259/wpilib/cpp/current/sim/include -I/usr/include -I/usr/include/gazebo-3.1 -I/usr/include/sdformat-2.2 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


