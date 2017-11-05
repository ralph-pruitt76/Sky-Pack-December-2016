################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include kit_lcd/BRD4300ArevA03/subdir.mk
-include kit/BRD4300ArevA03/subdir.mk
-include glib/BRD4300ArevA03/subdir.mk
-include emlib/subdir.mk
-include device/EFR32_B_1_1_B/gcc/subdir.mk
-include bgapi/subdir.mk
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
all: soc-smartPhone.axf

# Tool invocations
soc-smartPhone.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "D:\GoogleDriveCLT\GoogleDrive\WeatherCloud\Roadboard\Software\Road-Board-December-2016N8\BGM111\Studio_Work\soc-smartPhone\linker\GCC\efr32bg1b232f256gm48.ld" -Xlinker -no-enum-size-warning -Xlinker -no-wchar-size-warning -Xlinker --gc-sections -Xlinker -Map="soc-smartPhone.map" -mfpu=fpv4-sp-d16 -mfloat-abi=hard -o soc-smartPhone.axf "./advertisement.o" "./app.o" "./app_hw.o" "./app_ui.o" "./application_properties.o" "./beacon.o" "./gatt_db.o" "./graphics.o" "./htm.o" "./ia.o" "./main.o" "./bgapi/dmadrv.o" "./bgapi/em_adc.o" "./bgapi/em_ldma.o" "./bgapi/em_leuart.o" "./device/EFR32_B_1_1_B/gcc/startup_efr32bg1b.o" "./emlib/em_cmu.o" "./emlib/em_cryotimer.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_msc.o" "./emlib/em_rtcc.o" "./emlib/em_system.o" "./emlib/em_usart.o" "./glib/BRD4300ArevA03/bmp.o" "./glib/BRD4300ArevA03/dmd_display.o" "./glib/BRD4300ArevA03/glib.o" "./glib/BRD4300ArevA03/glib_bitmap.o" "./glib/BRD4300ArevA03/glib_circle.o" "./glib/BRD4300ArevA03/glib_font_narrow_6x8.o" "./glib/BRD4300ArevA03/glib_font_normal_8x8.o" "./glib/BRD4300ArevA03/glib_font_number_16x20.o" "./glib/BRD4300ArevA03/glib_line.o" "./glib/BRD4300ArevA03/glib_polygon.o" "./glib/BRD4300ArevA03/glib_rectangle.o" "./glib/BRD4300ArevA03/glib_string.o" "./kit/BRD4300ArevA03/bsp_bcc.o" "./kit/BRD4300ArevA03/bsp_stk.o" "./kit/BRD4300ArevA03/bsp_stk_leds.o" "./kit/BRD4300ArevA03/i2cspm.o" "./kit/BRD4300ArevA03/si7013.o" "./kit/BRD4300ArevA03/tempsens.o" "./kit/BRD4300ArevA03/udelay.o" "./kit_lcd/BRD4300ArevA03/display.o" "./kit_lcd/BRD4300ArevA03/displayls013b7dh03.o" "./kit_lcd/BRD4300ArevA03/displaypalemlib.o" "./src/InitDevice.o" "D:/SiliconLabs/SimplicityStudio/v4_2/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/GCC/bgapi.a" "D:/SiliconLabs/SimplicityStudio/v4_2/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/EABI/binbootloader.o" "D:/SiliconLabs/SimplicityStudio/v4_2/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/EABI/stack.a" "D:/SiliconLabs/SimplicityStudio/v4_2/developer/sdks/gecko_sdk_suite/v1.1//protocol/bluetooth_2.4/lib/EFR32BG1B/EABI/binstack.o" -lm -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: soc-smartPhone.hex'
	arm-none-eabi-objcopy -O ihex "soc-smartPhone.axf" "soc-smartPhone.hex"
	@echo ' '

	@echo 'Building bin file: soc-smartPhone.bin'
	arm-none-eabi-objcopy -O binary "soc-smartPhone.axf" "soc-smartPhone.bin"
	@echo ' '

	@echo 'Building s37 file: soc-smartPhone.s37'
	arm-none-eabi-objcopy -O srec "soc-smartPhone.axf" "soc-smartPhone.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "soc-smartPhone.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) soc-smartPhone.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
