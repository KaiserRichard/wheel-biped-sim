CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size
SHA256 := shasum -a 256
FW := firmware
CAN_KBPS ?= 500
LD := ../can_monitor/firmware/f446.ld
CFLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -std=gnu11 -O2 -g -Wall -Wextra -Werror \
          -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin \
          -fno-tree-loop-distribute-patterns -DCAN_KBPS=$(CAN_KBPS)
LDFLAGS := -T $(LD) -nostdlib -nostartfiles -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=build/imu_can_bench$(SUFFIX).map -lgcc
SUFFIX := $(if $(filter 500,$(CAN_KBPS)),,_$(CAN_KBPS)k)
ELF := build/imu_can_bench$(SUFFIX).elf
BIN := $(FW)/prebuilt/imu_can_bench$(SUFFIX).bin

.PHONY: all clean test
all: $(BIN)
	cd $(FW)/prebuilt && $(SHA256) imu_can_bench*.bin > SHA256SUMS && cat SHA256SUMS

$(ELF): $(FW)/main.c $(LD)
	mkdir -p build
	$(CC) $(CFLAGS) $(FW)/main.c $(LDFLAGS) -o $@
	$(SIZE) $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

test:
	python3 -m unittest discover -s tests -v

clean:
	rm -rf build
