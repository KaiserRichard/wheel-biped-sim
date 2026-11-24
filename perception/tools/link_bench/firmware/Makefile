# Nucleo-F446RE bench firmware. Needs only arm-none-eabi-gcc (+ objcopy).
# Flash: copy build/link_bench_f446.bin onto the NOD_F446RE USB drive, or `make flash` with st-flash.
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
PROTO = ../../link_proto
CFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -std=c99 -Os -g -Wall -Wextra -Werror \
 -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -I. -I$(PROTO)
LDFLAGS = -T f446.ld -nostdlib -nostartfiles -Wl,--gc-sections -Wl,-Map=build/link_bench_f446.map
SRC = startup.c main.c bench.c $(PROTO)/link_proto.c
.PHONY: all flash clean
all: build/link_bench_f446.bin
build/link_bench_f446.elf: $(SRC) bench.h f446.h $(PROTO)/link_proto.h f446.ld
	mkdir -p build
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -lgcc -o $@
	$(SIZE) $@
build/link_bench_f446.bin: build/link_bench_f446.elf
	$(OBJCOPY) -O binary $< $@
flash: build/link_bench_f446.bin
	st-flash --reset write $< 0x08000000
clean:
	rm -rf build
# Emulator build (QEMU netduinoplus2 = STM32F405, same USART/TIM2 addresses): skips RCC/UID.
build/link_bench_emu.elf: $(SRC) bench.h f446.h $(PROTO)/link_proto.h f446.ld
	mkdir -p build
	$(CC) $(CFLAGS) -DBENCH_EMU $(SRC) -T f446.ld -nostdlib -nostartfiles -Wl,--gc-sections -lgcc -o $@
