PYTHON ?= python3
CC ?= cc
CFLAGS = -std=c11 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -Wall -Wextra
ITERATIONS ?= $(shell python3 -c 'import json; print(json.load(open("fuzz_config.json"))["iterations_per_c_codec"]["value"])')
SEED ?= $(shell python3 -c 'import json; print(json.load(open("fuzz_config.json"))["seed"]["value"])')

.PHONY: test python c clean
test: python c
python:
	PYTHONPATH=../.. PYTHONDONTWRITEBYTECODE=1 $(PYTHON) -m unittest -v test_fuzz.py
c: fuzz_ph fuzz_link fuzz_ph_rx
	ASAN_OPTIONS=detect_leaks=0 ./fuzz_ph $(ITERATIONS) $(SEED)
	ASAN_OPTIONS=detect_leaks=0 ./fuzz_link $(ITERATIONS) $(SEED)
	ASAN_OPTIONS=detect_leaks=0 ./fuzz_ph_rx $(ITERATIONS) $(SEED)
fuzz_ph: c/fuzz_ph.c ../../tools/perception_health/c/perception_health.c
	$(CC) $(CFLAGS) -I../../tools/perception_health/c $^ -o $@
fuzz_link: c/fuzz_link.c ../../tools/link_proto/link_proto.c
	$(CC) $(CFLAGS) -I../../tools/link_proto $^ -o $@
fuzz_ph_rx: c/fuzz_ph_rx.c ../../tools/perception_health/c/mcu/ph_rx.c ../../tools/perception_health/c/perception_health.c
	$(CC) $(CFLAGS) -I../../tools/perception_health/c/mcu -I../../tools/perception_health/c $^ -o $@
clean:
	rm -f fuzz_ph fuzz_link fuzz_ph_rx
	rm -rf fuzz_ph.dSYM fuzz_link.dSYM fuzz_ph_rx.dSYM
