# CMake Toolchain File for STM32H723VGT6 (Cortex-M7, FPv5-D16 hard-float)
#
# Can be used directly via:
#   cmake -S . -B build_stm32 -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake
# In addition, the root CMakeLists.txt also builds the cross-compiled STM32H723
# target (`wbr_stm32h723`) alongside the host unit tests whenever `arm-none-eabi-gcc`
# is available on PATH.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(ARM_GCC_COMPILER arm-none-eabi-gcc REQUIRED)
find_program(ARM_GXX_COMPILER arm-none-eabi-g++)
find_program(ARM_AS_COMPILER  arm-none-eabi-gcc REQUIRED)
find_program(ARM_AR_TOOL      arm-none-eabi-ar REQUIRED)
find_program(ARM_OBJCOPY_TOOL arm-none-eabi-objcopy)
find_program(ARM_SIZE_TOOL    arm-none-eabi-size)

set(CMAKE_C_COMPILER   "${ARM_GCC_COMPILER}")
if(ARM_GXX_COMPILER)
    set(CMAKE_CXX_COMPILER "${ARM_GXX_COMPILER}")
endif()
set(CMAKE_ASM_COMPILER "${ARM_AS_COMPILER}")
set(CMAKE_AR           "${ARM_AR_TOOL}")

set(ARM_MCU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT   "${ARM_MCU_FLAGS} -ffunction-sections -fdata-sections -fno-common")
set(CMAKE_ASM_FLAGS_INIT "${ARM_MCU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${ARM_MCU_FLAGS} -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs")
