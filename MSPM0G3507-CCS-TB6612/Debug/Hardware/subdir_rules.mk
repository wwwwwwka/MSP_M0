################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Hardware/%.o: ../Hardware/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/TI_MSP/ti/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612" -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612/Hardware" -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612/Debug" -I"D:/TI_MSP/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"D:/TI_MSP/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"Hardware/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

