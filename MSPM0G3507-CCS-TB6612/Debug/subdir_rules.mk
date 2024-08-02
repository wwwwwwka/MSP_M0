################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/TI_MSP/ti/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612" -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612/Hardware" -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612/Debug" -I"D:/TI_MSP/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"D:/TI_MSP/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1548898625: ../empty.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"D:/TI_MSP/ti/ccs/utils/sysconfig_1.20.0/sysconfig_cli.bat" --script "D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612/empty.syscfg" -o "." -s "D:/TI_MSP/mspm0_sdk_2_01_00_03/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1548898625 ../empty.syscfg
device.opt: build-1548898625
device.cmd.genlibs: build-1548898625
ti_msp_dl_config.c: build-1548898625
ti_msp_dl_config.h: build-1548898625
Event.dot: build-1548898625

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/TI_MSP/ti/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612" -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612/Hardware" -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612/Debug" -I"D:/TI_MSP/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"D:/TI_MSP/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/TI_MSP/mspm0_sdk_2_01_00_03/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/TI_MSP/ti/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612" -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612/Hardware" -I"D:/TI_MSP/workspace/MSPM0G3507-CCS-TB6612/Debug" -I"D:/TI_MSP/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"D:/TI_MSP/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

