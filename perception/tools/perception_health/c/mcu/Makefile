CC ?= cc
CFLAGS = -std=c99 -Wall -Wextra -Werror -pedantic -I..
SOURCES = ph_rx.c ph_policy.c ../perception_health.c test_mcu.c
BUILD = .build

.PHONY: test clean
test: clean
	@set -eu; mkdir -p $(BUILD); \
	python3 generate_vectors.py $(BUILD)/vectors.txt; \
	$(CC) $(CFLAGS) $(SOURCES) -o $(BUILD)/test_mcu; \
	$(BUILD)/test_mcu $(BUILD)/vectors.txt; \
	if $(CC) $(CFLAGS) -fsanitize=address,undefined $(SOURCES) -o $(BUILD)/test_mcu_san >/dev/null 2>&1; then \
	  ASAN_OPTIONS=detect_leaks=0 $(BUILD)/test_mcu_san $(BUILD)/vectors.txt; \
	else echo 'sanitizers unavailable; strict C99 build still exercised'; fi; \
	$(CC) $(CFLAGS) -c ph_rx.c -o $(BUILD)/ph_rx.o; \
	$(CC) $(CFLAGS) -c ph_policy.c -o $(BUILD)/ph_policy.o; \
	size $(BUILD)/ph_rx.o $(BUILD)/ph_policy.o

clean:
	rm -rf $(BUILD)
