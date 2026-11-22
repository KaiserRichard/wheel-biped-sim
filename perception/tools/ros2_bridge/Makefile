CXX ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -pedantic
.PHONY: test python-test clean
python-test:
	PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
build/core_probe: tests/core_probe.cpp src/mapping.hpp src/clock_policy.hpp ../vio_adapter/adapter.h
	mkdir -p build
	$(CXX) $(CXXFLAGS) -Isrc -I../vio_adapter tests/core_probe.cpp -o $@
test: build/core_probe
	./build/core_probe < /dev/null
	PYTHONDONTWRITEBYTECODE=1 WBR_CPP_PROBE=$(CURDIR)/build/core_probe python3 -m unittest discover -s tests -v
clean:
	rm -f build/core_probe
