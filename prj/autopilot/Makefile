##############################################################################################
# DEFAULT

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CXX  = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
BIN  = $(CP) -O ihex 

STM32F4_STDPERIPH_LIB_DIR = ../../lib
DRIVERS_DIR = ../../drivers
COMMON_DIR = ../../common

MCU = cortex-m4

# List all default C defines, e.g. -D_DEBUG=1
DDEFS = -D_VERBOSE 

# List all default ASM defines, e.g. -D_DEBUG=1
DADEFS = 

# List all default include directories
DINCDIR = 

# List all default library directories
DLIBDIR =

# List all default libraries
DLIBS = 


##############################################################################################
# PROJECT

# Define project name
PRJ_NAME     = autopilot
USE_HARD_FPU = 1

# Define linker script
LDSCRIPT = ./src/STM32F405RG_FLASH.ld

# Define FPU settings
ifeq ($(USE_HARD_FPU), 0)
	FPU = -mthumb -mfloat-abi=soft -mfpu=fpv4-sp-d16
else
	FPU = -mthumb -mcpu=$(MCU) -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant
endif

# List all project C defines
PDEFS = -D STM32F40_41xxx -D USE_STDPERIPH_DRIVER

# List all project ASM defines
PADEFS = 

# List all project ASM source files
ASRC = ../../lib/CMSIS/Device/ST/STM32F4xx/Source/startup_stm32f40_41xxx.s

# List all project C source files
SRC =	$(wildcard src/*.c) \
			$(COMMON_DIR)/data_struct/circular_buffer.c \
			$(COMMON_DIR)/data_struct/double_buffer.c \
			$(COMMON_DIR)/drivers/can.c \
			$(COMMON_DIR)/drivers/clock.c \
			$(COMMON_DIR)/drivers/i2c.c \
			$(COMMON_DIR)/drivers/iwdg.c \
			$(COMMON_DIR)/drivers/led.c \
			$(COMMON_DIR)/drivers/pwm.c \
			$(COMMON_DIR)/drivers/sdio.c \
			$(COMMON_DIR)/drivers/tim.c \
			$(COMMON_DIR)/drivers/uart.c \
			$(COMMON_DIR)/memory/24xx64.c \
			$(wildcard $(COMMON_DIR)/memory/sdcard/*.c) \
			$(wildcard $(COMMON_DIR)/memory/sdcard/fatfs/*.c) \
			$(COMMON_DIR)/sensors/imu/mpu9150.c \
			$(COMMON_DIR)/sensors/imu/mpu9250.c \
			$(COMMON_DIR)/sensors/pressure/ms5607.c \

# List all project include directories
PINCDIR =	./src \
					$(STM32F4_STDPERIPH_LIB_DIR)/CMSIS/Device/ST/STM32F4xx/Include \
					$(STM32F4_STDPERIPH_LIB_DIR)/CMSIS/Include \
					$(STM32F4_STDPERIPH_LIB_DIR)/STM32F4xx_StdPeriph_Driver/inc \
					$(COMMON_DIR)/data_struct \
					$(COMMON_DIR)/drivers/ \
					$(COMMON_DIR)/memory/ \
					$(COMMON_DIR)/memory/sdcard/ \
					$(COMMON_DIR)/memory/sdcard/fatfs/ \
					$(COMMON_DIR)/sensors/imu/ \
					$(COMMON_DIR)/sensors/pressure/ \

# List all project library directories
PLIBDIR = $(STM32F4_STDPERIPH_LIB_DIR)

# List all projects libraries
PLIBS = -lstm32f4 --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group 

# Define optimisation level
OPT = -O2 -falign-functions=16 -fno-inline -fomit-frame-pointer


##############################################################################################
# Generate Flags

INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(PINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(PLIBDIR))

DEFS    = $(DDEFS) $(PDEFS)

ADEFS   = $(DADEFS) $(PADEFS)
OBJS    = $(ASRC:.s=.o) $(SRC:.c=.o) $(STM32F4_STDPERIPH_LIB_DIR)/libstm32f4.a
LIBS    = $(DLIBS) $(PLIBS)
MCFLAGS = -mthumb -mcpu=$(MCU)

ASFLAGS = $(MCFLAGS) -g -gdwarf-2 -Wa,-amhls=$(<:.s=.lst) $(ADEFS)
CFLAGS = $(MCFLAGS) $(OPT) $(FPU) -gdwarf-2 -Wall -fverbose-asm $(DEFS)

# Generate dependency information
CFLAGS += -MD -MP -MF .dep/$(@F).d

CPFLAGS = $(CFLAGS)
CFLAGS += -Wstrict-prototypes -Wa,-ahlms=$(<:.c=.lst)

LDFLAGS = $(MCFLAGS) $(FPU) -T$(LDSCRIPT) -Wl,-Map=$(PRJ_NAME).map,--cref,--no-warn-mismatch $(LIBDIR)
CPFLAGS = $(MCFLAGS) $(OPT) $(FPU) -gdwarf-2 -Wall -fverbose-asm $(DEFS)


##############################################################################################
# OpenOCD Configurations
TARGET_CONFIG = -f "interface/stlink-v2.cfg" -f "target/stm32f4x_stlink.cfg"
FLASH_CMDS += -c "init"
FLASH_CMDS += -c "reset halt"
FLASH_CMDS += -c "flash write_image erase $(PRJ_NAME).elf"
FLASH_CMDS += -c "reset run"
FLASH_CMDS += -c "exit"


##############################################################################################
# Targets
all: $(OBJS) $(PRJ_NAME).elf $(PRJ_NAME).hex $(PRJ_NAME).bin

$(STM32F4_STDPERIPH_LIB_DIR)/libstm32f4.a :
	cd $(STM32F4_STDPERIPH_LIB_DIR) && $(MAKE)

%.o : %.s
	$(AS) -c $(ASFLAGS) $< -o $@

%.o : %.c
	$(CC) -c $(CFLAGS) -I . $(INCDIR) $< -o $@

%.o : %.cpp
	$(CXX) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

%elf: $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
  
%hex: %elf
	$(CP) -O ihex $< $@

%bin: %elf
	$(CP) -O binary $< $@

clean:
	-rm -f $(OBJS)
	-rm -f $(PRJ_NAME).elf
	-rm -f $(PRJ_NAME).map
	-rm -f $(PRJ_NAME).hex
	-rm -f $(PRJ_NAME).bin
	-rm -f $(SRC:.c=.c.bak)
	-rm -f $(SRC:.c=.lst)
	-rm -f $(ASRC:.s=.s.bak)
	-rm -f $(ASRC:.s=.lst)
	-rm -fR .dep

flash: $(PRJ_NAME).elf
	-openocd $(TARGET_CONFIG) $(FLASH_CMDS)

# Include the dependency files
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
