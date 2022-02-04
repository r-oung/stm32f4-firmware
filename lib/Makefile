##############################################################################################
# DEFAULT

CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

DEFINES = -D STM32F40_41xxx
STM32F4_STDPERIPH_LIB_DIR = .


##############################################################################################
# Flags

CFLAGS += -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffreestanding -nostdlib

CMSIS_INC_DIR  = $(STM32F4_STDPERIPH_LIB_DIR)/CMSIS/Include
DEVICE_INC_DIR = $(STM32F4_STDPERIPH_LIB_DIR)/CMSIS/Device/ST/STM32F4xx/Include
DRIVER_INC_DIR = $(STM32F4_STDPERIPH_LIB_DIR)/STM32F4xx_StdPeriph_Driver/inc
DRIVER_SRC_DIR = $(STM32F4_STDPERIPH_LIB_DIR)/STM32F4xx_StdPeriph_Driver/src

INCDIR := $(patsubst %, -I%, $(CMSIS_INC_DIR) $(DEVICE_INC_DIR) $(DRIVER_INC_DIR))

SRCS := $(wildcard $(DRIVER_SRC_DIR)/*.c)
OBJS := $(SRCS:.c=.o)


##############################################################################################
# Targets
all: libstm32f4.a

%.o : %.c
	$(CC) -c $(CFLAGS) $(INCDIR) $(DEFINES) $< -o $@

libstm32f4.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) libstm32f4.a
