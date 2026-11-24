CXX := clang++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wpedantic
BUILD := build$(if $(filter 1,$(WBR_WITH_ORBSLAM3)),/orb,)
SOURCES := adapter.cpp
# ORB_INCLUDE_FLAGS must include ORB/include, cameras, Eigen, Sophus, OpenCV etc.
# ORB_LIB_FLAGS supplies the ORB library and its required transitive libraries.
ifeq ($(WBR_WITH_ORBSLAM3),1)
ifeq ($(strip $(ORB_INCLUDE_FLAGS)),)
$(error ORB_INCLUDE_FLAGS is required for WBR_WITH_ORBSLAM3=1)
endif
ifeq ($(strip $(ORB_LIB_FLAGS)),)
$(error ORB_LIB_FLAGS is required for WBR_WITH_ORBSLAM3=1)
endif
CXXFLAGS += -DWBR_WITH_ORBSLAM3 $(ORB_INCLUDE_FLAGS)
SOURCES += orb_engine.cpp
LDLIBS += $(ORB_LIB_FLAGS)
endif
OBJECTS := $(SOURCES:%.cpp=$(BUILD)/%.o)
.PHONY: all test clean
all: $(BUILD)/libvio_adapter.a $(BUILD)/replay_fake
$(BUILD):
	mkdir -p $@
$(BUILD)/%.o: %.cpp adapter.h orb_engine.h orb_imu.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@
$(BUILD)/libvio_adapter.a: $(OBJECTS)
	ar rcs $@ $^
$(BUILD)/test_adapter: tests/test_adapter.cpp fake_engine.h orb_imu.hpp $(BUILD)/libvio_adapter.a
	$(CXX) $(CXXFLAGS) -I. $< $(BUILD)/libvio_adapter.a $(LDLIBS) -o $@
$(BUILD)/replay_fake: replay_fake.cpp fake_engine.h $(BUILD)/libvio_adapter.a
	$(CXX) $(CXXFLAGS) -I. $< $(BUILD)/libvio_adapter.a $(LDLIBS) -o $@
test: $(BUILD)/test_adapter $(BUILD)/replay_fake
	./$(BUILD)/test_adapter
	python3 -m unittest discover -s tests -v
clean:
	rm -rf $(BUILD)
