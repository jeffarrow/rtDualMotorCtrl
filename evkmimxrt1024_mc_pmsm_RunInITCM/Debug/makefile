################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include xip/subdir.mk
-include utilities/subdir.mk
-include startup/subdir.mk
-include source/subdir.mk
-include motor_control/pmsm/pmsm_float/state_machine/subdir.mk
-include motor_control/pmsm/pmsm_float/mc_state_machine/subdir.mk
-include motor_control/pmsm/pmsm_float/mc_identification/subdir.mk
-include motor_control/pmsm/pmsm_float/mc_algorithms/subdir.mk
-include motor_control/pmsm/pmsm_float/subdir.mk
-include mc_drivers/subdir.mk
-include freemaster/drivers/subdir.mk
-include freemaster/subdir.mk
-include drivers/subdir.mk
-include device/subdir.mk
-include component/uart/subdir.mk
-include component/serial_manager/subdir.mk
-include component/lists/subdir.mk
-include board/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := evkmimxrt1024_mc_pmsm_RunInITCM
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: evkmimxrt1024_mc_pmsm_RunInITCM.axf

# Tool invocations
evkmimxrt1024_mc_pmsm_RunInITCM.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\rtcesl\CM7F_RTCESL_4.6.2_MCUX\AMCLIB" -L"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\rtcesl\CM7F_RTCESL_4.6.2_MCUX\GFLIB" -L"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\rtcesl\CM7F_RTCESL_4.6.2_MCUX\GDFLIB" -L"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\rtcesl\CM7F_RTCESL_4.6.2_MCUX\GMCLIB" -L"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\rtcesl\CM7F_RTCESL_4.6.2_MCUX\MLIB" -Xlinker -Map="evkmimxrt1024_mc_pmsm_RunInITCM.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -Xlinker --cref -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -T "evkmimxrt1024_mc_pmsm_RunInITCM_Debug.ld" -o "evkmimxrt1024_mc_pmsm_RunInITCM.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) evkmimxrt1024_mc_pmsm_RunInITCM.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "evkmimxrt1024_mc_pmsm_RunInITCM.axf"; # arm-none-eabi-objcopy -v -O binary "evkmimxrt1024_mc_pmsm_RunInITCM.axf" "evkmimxrt1024_mc_pmsm_RunInITCM.bin" ; # checksum -p MIMXRT1024xxxxx -d "evkmimxrt1024_mc_pmsm_RunInITCM.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
