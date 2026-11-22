# Bench link test: host simulator + tests. Firmware lives in firmware/ (own Makefile).
CC = cc
CFLAGS = -std=c99 -Wall -Wextra -Werror -O2 -Ifirmware -I../link_proto
.PHONY: test firmware clean
host/sim: host/sim.c firmware/bench.c firmware/bench.h ../link_proto/link_proto.c ../link_proto/link_proto.h
	$(CC) $(CFLAGS) host/sim.c firmware/bench.c ../link_proto/link_proto.c -o $@
test: host/sim
	$(CC) $(CFLAGS) tests/test_leds.c firmware/bench.c ../link_proto/link_proto.c -o tests/test_leds
	./tests/test_leds
	PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
firmware:
	$(MAKE) -C firmware
clean:
	rm -f host/sim
	rm -f tests/test_leds
	$(MAKE) -C firmware clean
