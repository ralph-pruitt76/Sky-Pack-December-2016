################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/subdir.mk
-include device/EFR32_B_1_1_B/gcc/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: soc-CRP-RoadPack.axf

# Tool invocations
soc-CRP-RoadPack.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "D:\GoogleDriveCLT\GoogleDrive\WeatherCloud\Skypack\Software\Sky-Pack-December-2016\BGM111\Studio_Work\soc-CRP-RoadPack\linker\GCC\efr32bg1b232f256gm56.ld" -Xlinker -no-enum-size-warning -Xlinker -no-wchar-size-warning -Xlinker --gc-sections -Xlinker -Map="soc-CRP-RoadPack.map" -mfpu=fpv4-sp-d16 -mfloat-abi=hard -o soc-CRP-RoadPack.axf "./application_properties.o" "./gatt_db.o" "./gpiointerrupt.o" "./main.o" "./uart_echo.o" "./device/EFR32_B_1_1_B/gcc/startup_efr32bg1b.o" "./emlib/em_cmu.o" "./emlib/em_cryotimer.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_msc.o" "./emlib/em_rtcc.o" "./emlib/em_system.o" "./emlib/em_timer.o" "./emlib/em_usart.o" "./src/InitDevice.o" "D:/SiliconLabs/SimplicityStudio/v4_2/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/EABI/binbootloader.o" "D:/SiliconLabs/SimplicityStudio/v4_2/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/GCC/bgapi.a" "D:/SiliconLabs/SimplicityStudio/v4_2/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/EABI/stack.a" "D:/SiliconLabs/SimplicityStudio/v4_2/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/EABI/binstack.o" -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: soc-CRP-RoadPack.hex'
	arm-none-eabi-objcopy -O ihex "soc-CRP-RoadPack.axf" "soc-CRP-RoadPack.hex"
	@echo ' '

	@echo 'Building bin file: soc-CRP-RoadPack.bin'
	arm-none-eabi-objcopy -O binary "soc-CRP-RoadPack.axf" "soc-CRP-RoadPack.bin"
	@echo ' '

	@echo 'Building s37 file: soc-CRP-RoadPack.s37'
	arm-none-eabi-objcopy -O srec "soc-CRP-RoadPack.axf" "soc-CRP-RoadPack.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "soc-CRP-RoadPack.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) soc-CRP-RoadPack.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
