################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := c:/ti/ccsv6/tools/compiler/c2000_6.2.7

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"C:/CTCJ_DSP/Debug/DSP28_DefaultIsr.obj" \
"C:/CTCJ_DSP/Debug/DSP28_Ev.obj" \
"C:/CTCJ_DSP/Debug/DSP28_GlobalVariableDefs.obj" \
"C:/CTCJ_DSP/Debug/DSP28_Gpio.obj" \
"C:/CTCJ_DSP/Debug/DSP28_PieCtrl.obj" \
"C:/CTCJ_DSP/Debug/DSP28_PieVect.obj" \
"C:/CTCJ_DSP/Debug/DSP28_Sci.obj" \
"C:/CTCJ_DSP/Debug/DSP28_SysCtrl.obj" \
"C:/CTCJ_DSP/Debug/fir16.obj" \
"C:/CTCJ_DSP/Debug/main.obj" \
"../Flash2811_API_V210.lib" \
"../fft.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
AD.out \

EXE_OUTPUTS__QUOTED += \
"AD.out" \

BIN_OUTPUTS += \
AD.hex \

BIN_OUTPUTS__QUOTED += \
"AD.hex" \


# All Target
all: AD.out

# Tool invocations
AD.out: $(OBJS) $(CMD_SRCS) $(CMD_UPPER_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.7/bin/cl2000" -v28 -ml -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --display_error_number --diag_wrap=off --obj_directory="C:/CTCJ_DSP/Debug" -z -m"../AD.MAP" --warn_sections -i"c:/ti/ccsv6/tools/compiler/c2000_6.2.7/lib" -i"c:/ti/ccsv6/tools/compiler/c2000_6.2.7/include" -i"D:/chen/Core/FirmWare_CT/CTCJ_DSP2811" --reread_libs --display_error_number --diag_wrap=off --xml_link_info="AD_linkInfo.xml" --absolute_exe --rom_model -o "AD.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

AD.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: C2000 Hex Utility'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.7/bin/hex2000"  -o "AD.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "DSP28_DefaultIsr.pp" "DSP28_Ev.pp" "DSP28_GlobalVariableDefs.pp" "DSP28_Gpio.pp" "DSP28_PieCtrl.pp" "DSP28_PieVect.pp" "DSP28_Sci.pp" "DSP28_SysCtrl.pp" "main.pp" 
	-$(RM) "C:\CTCJ_DSP\Debug\DSP28_DefaultIsr.obj" "C:\CTCJ_DSP\Debug\DSP28_Ev.obj" "C:\CTCJ_DSP\Debug\DSP28_GlobalVariableDefs.obj" "C:\CTCJ_DSP\Debug\DSP28_Gpio.obj" "C:\CTCJ_DSP\Debug\DSP28_PieCtrl.obj" "C:\CTCJ_DSP\Debug\DSP28_PieVect.obj" "C:\CTCJ_DSP\Debug\DSP28_Sci.obj" "C:\CTCJ_DSP\Debug\DSP28_SysCtrl.obj" "C:\CTCJ_DSP\Debug\fir16.obj" "C:\CTCJ_DSP\Debug\main.obj" 
	-$(RM) "fir16.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

