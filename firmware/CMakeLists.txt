cmake_minimum_required(VERSION 3.16)
project(wbr_firmware_host_slice C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

find_package(Threads REQUIRED)

set(FW_COMMON_WARNINGS
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Werror
    -ffp-contract=off
)

set(S11_TEST_WARNINGS
    -Wall
    -Wextra
    -Wpedantic
    -ffp-contract=off
)

set(WBR_FIRMWARE_SOURCES
    firmware/crc32c.c
    firmware/cobs.c
    firmware/usb_link.c
    firmware/damiao_mit.c
    firmware/safety_fsm.c
    firmware/s11_packet_policy.c
    firmware/log_packer.c
    firmware/can_sched.c
    firmware/control/pid.c
    firmware/control/VMC_calc.c
    firmware/control/s10_estimator.c
    firmware/control/h5_balance.c
    firmware/control/balance_ctrl.c
)

add_library(wbr_firmware STATIC ${WBR_FIRMWARE_SOURCES})
set_target_properties(wbr_firmware PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(wbr_firmware PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/firmware"
)
target_compile_options(wbr_firmware PRIVATE ${FW_COMMON_WARNINGS})
target_link_libraries(wbr_firmware PUBLIC m)

# Shared library for Python ctypes firmware-in-the-loop (FIL) testing
add_library(wbr_balance_ctrl SHARED ${WBR_FIRMWARE_SOURCES})
target_include_directories(wbr_balance_ctrl PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/firmware"
)
target_compile_options(wbr_balance_ctrl PRIVATE ${FW_COMMON_WARNINGS})
target_link_libraries(wbr_balance_ctrl PUBLIC m)

enable_testing()

# Mandatory -UNDEBUG guard on all test targets so assert() never compiles away in Release builds
function(apply_undebug_guard target_name)
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /UNDEBUG)
    else()
        target_compile_options(${target_name} PRIVATE -UNDEBUG)
    endif()
endfunction()

function(add_fw_test test_name test_src)
    add_executable(${test_name} "tests/${test_src}")
    target_link_libraries(${test_name} PRIVATE wbr_firmware m)
    target_compile_options(${test_name} PRIVATE ${FW_COMMON_WARNINGS})
    apply_undebug_guard(${test_name})
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

function(add_s11_verified_test test_name test_rel_path)
    add_executable(${test_name} "${CMAKE_CURRENT_SOURCE_DIR}/${test_rel_path}")
    target_link_libraries(${test_name} PRIVATE wbr_firmware m)
    target_include_directories(${test_name} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/s11_verified/tests"
    )
    target_compile_options(${test_name} PRIVATE ${S11_TEST_WARNINGS})
    apply_undebug_guard(${test_name})
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

# Existing 8 firmware slice host tests
add_fw_test(test_crc32c_cobs       test_crc32c_cobs.c)
add_fw_test(test_usb_link          test_usb_link.c)
add_fw_test(test_damiao_mit        test_damiao_mit.c)
add_fw_test(test_safety_fsm        test_safety_fsm.c)
add_fw_test(test_s11_packet_policy test_s11_packet_policy.c)
add_fw_test(test_log_packer        test_log_packer.c)
add_fw_test(test_can_sched         test_can_sched.c)
add_fw_test(test_balance_ctrl      test_balance_ctrl.c)

# Verified S11 packet-policy suite (18 tests, including reviewer_probes & concurrent mailbox)
add_s11_verified_test(test_a1_latched_frame              s11_verified/tests/test_a1_latched_frame.c)
add_s11_verified_test(test_a2_arming_state_machine       s11_verified/tests/test_a2_arming_state_machine.c)
add_s11_verified_test(test_a3_transport_age_limit_window s11_verified/tests/test_a3_transport_age_limit_window.c)
add_s11_verified_test(test_a4_stuck_detector             s11_verified/tests/test_a4_stuck_detector.c)
add_s11_verified_test(test_a5_stale_out_of_order         s11_verified/tests/test_a5_stale_out_of_order.c)
add_s11_verified_test(test_a6_plausibility_limits        s11_verified/tests/test_a6_plausibility_limits.c)
add_s11_verified_test(test_a7_wheel_angle_wrap           s11_verified/tests/test_a7_wheel_angle_wrap.c)
add_s11_verified_test(test_a8_isolated_invalid           s11_verified/tests/test_a8_isolated_invalid.c)
add_s11_verified_test(test_a9_timer_wrap                 s11_verified/tests/test_a9_timer_wrap.c)
add_s11_verified_test(test_a10_isr_snapshot              s11_verified/tests/test_a10_isr_snapshot.c)
add_s11_verified_test(test_a10_concurrent_mailbox        s11_verified/tests/test_a10_concurrent_mailbox.c)
target_link_libraries(test_a10_concurrent_mailbox PRIVATE Threads::Threads)
add_s11_verified_test(test_a11_variable_dt               s11_verified/tests/test_a11_variable_dt.c)
add_s11_verified_test(test_a12_predict_cap_precedence    s11_verified/tests/test_a12_predict_cap_precedence.c)
add_s11_verified_test(test_age_budget                    s11_verified/tests/test_age_budget.c)
add_s11_verified_test(test_nan_and_inf                   s11_verified/tests/test_nan_and_inf.c)
add_s11_verified_test(test_stale_packet_sequence         s11_verified/tests/test_stale_packet_sequence.c)
add_s11_verified_test(test_latch_and_rearm               s11_verified/tests/test_latch_and_rearm.c)
add_s11_verified_test(reviewer_probes                    s11_verified/review2/reviewer_probes.c)
