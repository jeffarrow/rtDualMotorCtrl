################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../utilities/fsl_assert.c \
../utilities/fsl_debug_console.c \
../utilities/fsl_str.c 

OBJS += \
./utilities/fsl_assert.o \
./utilities/fsl_debug_console.o \
./utilities/fsl_str.o 

C_DEPS += \
./utilities/fsl_assert.d \
./utilities/fsl_debug_console.d \
./utilities/fsl_str.d 


# Each subdirectory must supply rules for building sources it contributes
utilities/%.o: ../utilities/%.c utilities/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MIMXRT1024DAG5A -DCPU_MIMXRT1024DAG5A_cm7 -DSDK_DEBUGCONSOLE=1 -DXIP_EXTERNAL_FLASH=1 -DXIP_BOOT_HEADER_ENABLE=1 -DSERIAL_PORT_TYPE_UART=1 -DMCUXPRESSO_SDK -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DRAM_RELOCATION -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\board" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\source" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\drivers" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\rtcesl\CM7F_RTCESL_4.6.2_MCUX\AMCLIB\Include" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\rtcesl\CM7F_RTCESL_4.6.2_MCUX\GDFLIB\Include" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\rtcesl\CM7F_RTCESL_4.6.2_MCUX\GFLIB\Include" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\rtcesl\CM7F_RTCESL_4.6.2_MCUX\GMCLIB\Include" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\rtcesl\CM7F_RTCESL_4.6.2_MCUX\MLIB\Include" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\motor_control\pmsm\pmsm_float\mc_identification" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\motor_control\pmsm\pmsm_float\mc_algorithms" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\motor_control\pmsm\pmsm_float\state_machine" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\motor_control\pmsm\pmsm_float\mc_state_machine" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\motor_control\pmsm\pmsm_float\mc_drivers" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\motor_control\pmsm\pmsm_float" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\device" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\utilities" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\component\uart" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\component\serial_manager" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\component\lists" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\xip" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\freemaster" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\freemaster\platforms" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\freemaster\drivers" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\CMSIS" -I"C:\Users\140597\Documents\MCUXpressoIDE_11.5.0_7232\QuadMtrCtrl\evkmimxrt1024_mc_pmsm_RunInITCM\mc_drivers" -O3 -fno-common -g3 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


