cmake_minimum_required(VERSION 3.16)
project(pi_link_c_ref C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(c_ref SHARED
    ../c_ref/cobs.c
    ../c_ref/crc32c.c
    ../c_ref/usb_link.c
    ../c_ref/log_packer.c
    c_msg_packers.c
)

target_include_directories(c_ref PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../c_ref
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(c_ref PRIVATE -Wall -Wextra -Werror -O2)
endif()

set_target_properties(c_ref PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    OUTPUT_NAME "c_ref"
)
