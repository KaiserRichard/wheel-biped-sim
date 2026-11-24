CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size
SHA256 := shasum -a 256
LIBROOT ?= $(HOME)/Desktop/Research/agv-libbuild/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros
FW := firmware
CPUFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS := $(CPUFLAGS) -std=gnu11 -O2 -g -Wall -Wextra -Werror -ffunction-sections -fdata-sections \
          -fno-common -fno-unwind-tables -fno-asynchronous-unwind-tables -DSTM32F446xx -DUROS_USART1 \
          -I$(FW) -I$(LIBROOT)/microros_include
LDFLAGS := $(CPUFLAGS) -T $(FW)/f446.ld -nostartfiles --specs=nano.specs --specs=nosys.specs \
           -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=build/can_monitor.map
SOURCES := $(FW)/startup.c $(FW)/main.c $(FW)/syscalls.c $(FW)/can_rx.c $(FW)/robstride_decode.c
ELF := build/can_monitor.elf
BIN := $(FW)/prebuilt/can_monitor.bin

.PHONY: all clean size test
all: $(BIN) checksum

$(ELF): $(SOURCES) $(wildcard $(FW)/*.h) $(FW)/f446.ld
	mkdir -p build
	$(CC) $(CFLAGS) $(SOURCES) $(LDFLAGS) -L$(LIBROOT) -Wl,--start-group -lmicroros -lc -lm -lgcc -Wl,--end-group -o $@
	$(SIZE) $@

$(BIN): $(ELF)
	mkdir -p $(dir $@)
	$(OBJCOPY) -O binary $< $@

size: $(ELF)
	$(SIZE) $(ELF)

checksum: $(BIN)
	$(SHA256) $(BIN) > $(FW)/prebuilt/SHA256SUMS
	cat $(FW)/prebuilt/SHA256SUMS

test:
	mkdir -p build
	$(CC_HOST) -std=c11 -Wall -Wextra -Werror -O2 -Ifirmware tests/test_decoder.c firmware/robstride_decode.c -lm -o build/test_decoder
	./build/test_decoder

CC_HOST ?= cc

clean:
	rm -rf build $(BIN) $(FW)/prebuilt/SHA256SUMS
