# IMC-22 SDK Makefile
# RISC-V RV32IMAC 工具链

# 工具链配置
CROSS_COMPILE = riscv32-unknown-elf-
CC = $(CROSS_COMPILE)gcc
AS = $(CROSS_COMPILE)as
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size

# 项目配置
TARGET = hive_node
SDK_DIR = imc22_sdk
BUILD_DIR = build

# 编译选项
ARCH_FLAGS = -march=rv32imac -mabi=ilp32
OPT_FLAGS = -O2 -g
WARN_FLAGS = -Wall -Wextra -Werror

CFLAGS = $(ARCH_FLAGS) $(OPT_FLAGS) $(WARN_FLAGS) \
         -I$(SDK_DIR) \
         -ffunction-sections -fdata-sections \
         -ffreestanding

LDFLAGS = $(ARCH_FLAGS) \
          -T $(SDK_DIR)/linker.ld \
          -nostartfiles \
          -Wl,--gc-sections \
          -Wl,-Map=$(BUILD_DIR)/$(TARGET).map

# 源文件
SDK_SRCS = $(SDK_DIR)/startup.c \
           $(SDK_DIR)/imc22_can.c \
           $(SDK_DIR)/imc22_npu.c

APP_SRCS = hive_node_ctrl.c

SRCS = $(SDK_SRCS) $(APP_SRCS)

# 目标文件
OBJS = $(SRCS:%.c=$(BUILD_DIR)/%.o)

# 默认目标
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex
	@echo "Build complete!"
	@$(SIZE) $(BUILD_DIR)/$(TARGET).elf

# 创建构建目录
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)/$(SDK_DIR)

# 编译规则
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# 链接规则
$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	@echo "LD $@"
	@$(CC) $(LDFLAGS) $(OBJS) -o $@

# 生成二进制文件
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	@echo "OBJCOPY $@"
	@$(OBJCOPY) -O binary $< $@

# 生成 HEX 文件
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	@echo "OBJCOPY $@"
	@$(OBJCOPY) -O ihex $< $@

# 反汇编
disasm: $(BUILD_DIR)/$(TARGET).elf
	@$(OBJDUMP) -d $< > $(BUILD_DIR)/$(TARGET).asm

# 清理
clean:
	rm -rf $(BUILD_DIR)

# 烧录 (假设使用 OpenOCD)
flash: $(BUILD_DIR)/$(TARGET).bin
	openocd -f interface/jlink.cfg -f target/riscv.cfg \
	        -c "program $(BUILD_DIR)/$(TARGET).bin verify reset exit"

.PHONY: all clean disasm flash
