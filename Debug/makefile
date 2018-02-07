################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

EMPTY := 
SPACE := $(EMPTY) $(EMPTY)

ORDERED_OBJS += \
$(GEN_CMDS_QUOTED) \
"./enhance.obj" \
"./dsp_bios_cfg_c.obj" \
"./dsp_bios_cfg.obj" \
-l"rts6700.lib" \
-l"C:/EEE/CCStudio4.1/ccsv4\C6000\dsk6713\FFT\complex.obj" \
-l"C:/EEE/CCStudio4.1/ccsv4\C6000\dsk6713\FFT\fft_functions.obj" \
-l"C:/EEE/CCStudio4.1/ccsv4\C6000\csl\lib\csl6713.lib" \
-l"C:/EEE/CCStudio4.1/ccsv4\C6000\dsk6713\lib\dsk6713bslccsv4.lib" \

-include ../makefile.init

RM := DEL /F

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: RTDSP.out

# Tool invocations
RTDSP.out: $(GEN_CMDS) $(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Linker'
	"C:/EEE/CCStudio4.1/ccsv4/tools/compiler/c6000/bin/cl6x" -mv6700 -g --define="_DEBUG" --define="CHIP_6713" --diag_warning=225 -z -m"RTDSP.map" --warn_sections -i"C:/EEE/CCStudio4.1/ccsv4/tools/compiler/c6000/lib" -i"C:/EEE/CCStudio4.1/ccsv4/tools/compiler/c6000/include" -i"C:/EEE/CCStudio4.1/bios_5_41_02_14/packages/ti/rtdx/lib/c6000" -i"C:/EEE/CCStudio4.1/bios_5_41_02_14/packages/ti/bios/lib" --reread_libs --rom_model -o "RTDSP.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_CMDS__QTD)$(GEN_SRCS__QTD)$(C6000_EXECUTABLE_OUTPUTS__QTD) "RTDSP.out"
	-$(RM) ".\dsp_bios_cfg.obj" ".\dsp_bios_cfg_c.obj" ".\enhance.obj" 
	-$(RM) ".\dsp_bios_cfg.pp" 
	-$(RM) ".\dsp_bios_cfg_c.pp" ".\enhance.pp" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
