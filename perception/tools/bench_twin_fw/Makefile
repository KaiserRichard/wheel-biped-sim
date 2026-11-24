CC = /usr/local/bin/arm-none-eabi-gcc
OBJCOPY = /usr/local/bin/arm-none-eabi-objcopy
SIZE = /usr/local/bin/arm-none-eabi-size
M_HZ ?= 20
CFLAGS = -DM_HZ=$(M_HZ) -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -std=gnu11 -O2 -Wall -Wextra -Werror -ffunction-sections -fdata-sections -ffreestanding -fno-builtin
LD = ../can_monitor/firmware/f446.ld
SRC = firmware/main.c firmware/dm.c firmware/twin.c firmware/link_proto.c
# OPTIONAL AGV codec compilation only; never changes the WBR DM bus runtime.
AGV_CODEC ?= 0
ifeq ($(AGV_CODEC),1)
SRC += firmware/robstride.c
CFLAGS += -DBENCH_OPTIONAL_AGV_CODEC=1
endif
all: firmware/bench_twin.bin firmware/SHA256SUMS
build/bench_twin.elf: $(SRC) firmware/uart_queue.h firmware/dm.h firmware/twin.h firmware/link_proto.h ../link_proto/link_proto.c ../link_proto/link_proto.h $(LD) Makefile
	mkdir -p build
	$(CC) $(CFLAGS) $(SRC) -T $(LD) -nostdlib -nostartfiles -Wl,--gc-sections -Wl,-Map=build/bench_twin.map -lgcc -o $@
	$(SIZE) $@
firmware/bench_twin.bin: build/bench_twin.elf
	$(OBJCOPY) -O binary $< $@
firmware/SHA256SUMS: firmware/bench_twin.bin
	cd firmware && shasum -a 256 bench_twin.bin > SHA256SUMS
test:
	python3 -m unittest discover -s tests -v
.PHONY: all test
