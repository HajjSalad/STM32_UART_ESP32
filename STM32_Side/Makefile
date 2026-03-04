# ========================================================
# Makefile for STM32 and OpenOCD demo project
# ========================================================

# Target binary name
TARGET = Sensor_Node_UART

# Build directory
BUILD_DIR = Build

# Source files directory
C_SOURCES = Src/main.c \
			Src/uart1_driver.c \
			Src/uart2_driver.c \
			Src/systick.c \
            Src/syscalls.c \
			Src/uart_tx.c \
			Src/uart_rx.c \
			Src/crc_16.c 

# ASM sources
ASM_SOURCES = Startup/startup_stm32f446retx.s

# Options for float-abi:
# -mfloat-abi=soft     : No FPU, software emulation (slower, smaller code, more compatible)
# -mfloat-abi=softfp   : FPU used but software calling convention (compatible with soft)
# -mfloat-abi=hard     : FPU used with hardware calling convention (faster, larger code, less compatible)
# Third option used

# MCU definition
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

# C inlcudes
C_INCLUDES = -IInc \
             -IInc/CMSIS/Core/Include \
             -IInc/STM32F4xx/Include

# C defines
C_DEFS = -DSTM32F446xx \
         -DUSE_FULL_ASSERT \

# C flags
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -O2 -g3 -Wall -fdata-sections -ffunction-sections

# Linker script
LDSCRIPT = STM32F446RETX_FLASH.ld

# Libraries
LIBS = -lc -lm -lnosys
LIBDIR =

# Linker flags
LDFLAGS = $(MCU) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -specs=nano.specs \
          -Wl,-Map=$(TARGET).map,--cref -Wl,--gc-sections \
          -Wl,--no-warn-mismatch

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))

all: $(BUILD_DIR)/$(TARGET).elf
	@echo "Build complete."
	$(SZ) $(BUILD_DIR)/$(TARGET).elf

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)/Src $(BUILD_DIR)/Startup

# Compile C sources
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "Compiling $< ..."
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble startup
$(BUILD_DIR)/Startup/%.o: Startup/%.s | $(BUILD_DIR)
	@echo "Assembling $< ..."
	$(AS) $(CFLAGS) -c $< -o $@

# Link to create ELF
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	@echo "Linking $(TARGET).elf"
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

clean:
	@echo "Cleaning build files..."
	rm -rf $(BUILD_DIR)
	@echo "Clean complete"

# Flash target using OpenOCD
flash: all
	@echo "Flashing $(TARGET).elf to STM32..."
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

erase:
	@echo "Erasing STM32 flash memory..."
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init; halt; stm32f4x mass_erase 0; exit"