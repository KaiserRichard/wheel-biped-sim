CXX ?= c++
PYTHON ?= python3
CPPFLAGS += -Iinclude -Ibuild -I../../tools/vio_adapter -I../../tools/ros2_bridge/src -I../../tools/runtime_observer
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
SOURCES = src/core.cpp ../../tools/vio_adapter/adapter.cpp
HEADERS = include/wbr_orb_node/core.hpp ../../tools/vio_adapter/adapter.h ../../tools/vio_adapter/fake_engine.h ../../tools/ros2_bridge/src/mapping.hpp ../../tools/runtime_observer/event_recorder.h
.PHONY: test clean core-test
build/defaults.hpp: config/defaults.json scripts/generate_config.py
	$(PYTHON) scripts/generate_config.py $< $@
build/core_test: tests/core_test.cpp $(SOURCES) $(HEADERS) build/defaults.hpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -pthread tests/core_test.cpp $(SOURCES) -o $@
core-test: build/core_test
	./build/core_test all build
test: build/core_test map-points-test
	PYTHONDONTWRITEBYTECODE=1 $(PYTHON) -m unittest discover -s tests -v
clean:
	rm -rf build

.PHONY: map-points-test
build/map_points_test: tests/map_points_test.cpp include/wbr_orb_node/map_points.hpp
	mkdir -p build
	$(CXX) -Iinclude $(CXXFLAGS) tests/map_points_test.cpp -o $@
map-points-test: build/map_points_test
	./build/map_points_test
