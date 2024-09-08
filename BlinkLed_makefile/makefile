PRO_DIR		:= .
PROJ_NAME	:= BlinkLed
OUTPUT_PATH	:= $(PRO_DIR)/output

INC_DIR		:= $(PRO_DIR)/Core/Inc
INC_DIR		+= $(PRO_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Include
INC_DIR		+= $(PRO_DIR)/Drivers/CMSIS/Include
INC_DIR		+= $(PRO_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc
INC_DIR		+= $(PRO_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
SRC_DIR		:= $(PRO_DIR)/Core/Src
SRC_DIR		+= $(PRO_DIR)/Drivers/STM32F4xx_HAL_Driver/Src
STLink_DIR	:= "C:/Program Files (x86)/STMicroelectronics/STM32 ST-LINK Utility/ST-LINK Utility"

LINKER_FILES	:= $(PRO_DIR)/STM32F407VGTX_FLASH.ld
SRC_FILES		:= $(foreach SRC_DIR,$(SRC_DIR),$(wildcard $(SRC_DIR)/*.c))
INC_FILES		:= $(foreach INC_DIR,$(INC_DIR),$(wildcard $(INC_DIR)/*.h))
ASM_FILES		:= $(PRO_DIR)/Core/Startup/startup_stm32f407vgtx.s
INC_DIR_OPT		:= $(foreach INC_DIR, $(INC_DIR), -I$(INC_DIR))

OBJ_FILES		:= $(notdir $(SRC_FILES)) $(notdir $(ASM_FILES))
OBJ_FILES		:= $(subst .c,.o,$(OBJ_FILES))
OBJ_FILES		:= $(subst .s,.o,$(OBJ_FILES))
PATH_OBJS		:= $(foreach OBJ_FILES,$(OBJ_FILES),$(OUTPUT_PATH)/$(OBJ_FILES))

vpath %.c $(SRC_DIR)
vpath %.h $(INC_DIR)
COMPILER_DIR 	:=
CC				:= arm-none-eabi-gcc
ASM				:= arm-none-eabi-as
CC_OPT			:= -c -mcpu=cortex-m4 -mthumb -std=gnu11 -Wall -O0 $(INC_DIR_OPT) -DSTM32F407xx -DUSE_HAL_DRIVER -DDEBUG
LD_OPT			:= -nostdlib -T $(LINKER_FILES) -Wl,-Map=$(OUTPUT_PATH)/$(PROJ_NAME).map
AOPS 			:= --warn --fatal-warnings 

build: $(OBJ_FILES) $(LINKER_FILES)
	$(CC) $(LD_OPT) $(PATH_OBJS) -o $(OUTPUT_PATH)/$(PROJ_NAME).elf
	arm-none-eabi-objcopy -O ihex $(OUTPUT_PATH)/$(PROJ_NAME).elf $(OUTPUT_PATH)/$(PROJ_NAME).hex
	size $(OUTPUT_PATH)/$(PROJ_NAME).elf
%.o: %.c $(INC_FILES)
	$(CC) $(CC_OPT) -c $< -o $(OUTPUT_PATH)/$@
startup_stm32f407vgtx.o:
	$(ASM) $(AOPS) $(ASM_FILES) $< -o $(OUTPUT_PATH)/$@

run:
	$(STLink_DIR)/ST-LINK_CLI.exe -p "./Debug/$(PROJ_NAME).hex" 0x08000000
	$(STLink_DIR)/ST-LINK_CLI.exe -rst

erase:
	$(STLink_DIR)/ST-LINK_CLI.exe -ME

clean:
	rm -rf *.o *.elf *.hex *.map $(OUTPUT_PATH)/*
	
print-%:
	@echo $($(subst print-,,$@))