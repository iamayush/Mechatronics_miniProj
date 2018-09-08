################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
UARTfuncs.obj: E:/UIUC/Spring18/Mechatronics/HW/mycode/proj_ver1/UARTfuncs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP430 Compiler'
	"E:/ti/i-cgt-msp430_17.9.0.STS/bin/cl430" -vmsp -O0 --include_path="E:/ti/ccsv7/ccs_base/msp430/include" --include_path="E:/ti/i-cgt-msp430_17.9.0.STS/include" --define=__MSP430G2553__ -g --printf_support=minimal --diag_warning=225 --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="UARTFuncs.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

explore_traj.obj: ../explore_traj.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP430 Compiler'
	"E:/ti/i-cgt-msp430_17.9.0.STS/bin/cl430" -vmsp -O0 --include_path="E:/ti/ccsv7/ccs_base/msp430/include" --include_path="E:/ti/i-cgt-msp430_17.9.0.STS/include" --define=__MSP430G2553__ -g --printf_support=minimal --diag_warning=225 --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="explore_traj.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

user_proj_ver1.obj: E:/UIUC/Spring18/Mechatronics/HW/mycode/proj_ver1/user_proj_ver1.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP430 Compiler'
	"E:/ti/i-cgt-msp430_17.9.0.STS/bin/cl430" -vmsp -O0 --include_path="E:/ti/ccsv7/ccs_base/msp430/include" --include_path="E:/ti/i-cgt-msp430_17.9.0.STS/include" --define=__MSP430G2553__ -g --printf_support=minimal --diag_warning=225 --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="user_proj_ver1.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


