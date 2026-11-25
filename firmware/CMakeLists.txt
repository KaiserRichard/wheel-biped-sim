cmake_minimum_required(VERSION 3.16)
project(wbr_firmware_host_slice C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

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
    firmware/s11_frozen.c
    firmware/log_packer.c
    firmware/can_sched.c
    firmware/control/pid.c
    firmware/control/VMC_calc.c
    firmware/control/s10_estimator.c
    firmware/control/h5_balance.c
    firmware/control/balance_ctrl.c
    firmware/control/yaw_track.c
)

set(WBR_STM32H723_TARGET_SOURCES
    firmware/target/target_fdcan.c
    firmware/target/target_crsf.c
    firmware/target/target_usb_cdc.c
    firmware/target/target_iwdg.c
    firmware/target/target_estop.c
    firmware/target/target_timer.c
    firmware/target/target_safety_io.c
    firmware/target/main.c
    firmware/target/syscalls_minimal.c
)

# The STM32H7 HAL and CMSIS device files are vendor code and are not part of
# this repository. Point WBR_STM32_HAL_DIR at a directory that contains the
# STM32CubeH7 `Drivers/` tree to link the STM32H723 image. Without it, the host
# build and host tests (including the target layer against a local HAL stub)
# still run; only the linked ELF is skipped.
set(WBR_STM32_HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hal_ref" CACHE PATH
    "Directory containing the STM32CubeH7 Drivers/ tree (vendor HAL, not distributed)")

set(WBR_STM32H723_HAL_SOURCES
    Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/system_stm32h7xx.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_iwdg.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
)

set(WBR_STM32H723_STARTUP_ASM
    Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h723xx.s
)

set(WBR_STM32H723_LINKER_SCRIPT
    "${CMAKE_CURRENT_SOURCE_DIR}/firmware/target/STM32H723VGTx_FLASH.ld"
)

set(WBR_STM32H723_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/firmware/target"
    "${CMAKE_CURRENT_SOURCE_DIR}/firmware"
    "${CMAKE_CURRENT_SOURCE_DIR}/firmware/control"
    "${CMAKE_CURRENT_SOURCE_DIR}/s11_verified"
)

# Vendor inline headers keep their existing relaxed warning policy. Our own
# source and headers still compile with -Wconversion -Werror.
set(WBR_STM32H723_VENDOR_INCLUDES
    "${WBR_STM32_HAL_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc"
    "${WBR_STM32_HAL_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
    "${WBR_STM32_HAL_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
    "${WBR_STM32_HAL_DIR}/Drivers/CMSIS/Include"
)

set(WBR_STM32H723_MCU_FLAGS
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
)

set(WBR_STM32H723_COMMON_CFLAGS
    ${WBR_STM32H723_MCU_FLAGS}
    -std=c11
    -O2
    -g
    -ffp-contract=off
    -ffunction-sections
    -fdata-sections
    -fno-common
    -DUSE_HAL_DRIVER
    -DSTM32H723xx
)

# Strict warnings for portable core (firmware/) and target (firmware/target/)
set(WBR_STM32H723_FW_CFLAGS
    ${WBR_STM32H723_COMMON_CFLAGS}
    -Wall
    -Wextra
    -Wconversion
    -Werror
)

# Relaxed warnings (-w, no -Werror) for unmodified vendor HAL sources
set(WBR_STM32H723_HAL_CFLAGS
    ${WBR_STM32H723_COMMON_CFLAGS}
    -w
)

set(WBR_STM32H723_HAL_ABS_SOURCES "")
foreach(src_rel IN LISTS WBR_STM32H723_HAL_SOURCES)
    list(APPEND WBR_STM32H723_HAL_ABS_SOURCES "${WBR_STM32_HAL_DIR}/${src_rel}")
endforeach()
set(WBR_STM32H723_STARTUP_ABS "${WBR_STM32_HAL_DIR}/${WBR_STM32H723_STARTUP_ASM}")

if(EXISTS "${WBR_STM32_HAL_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal.h")
    set(WBR_STM32_HAL_FOUND TRUE)
else()
    set(WBR_STM32_HAL_FOUND FALSE)
endif()

# ==============================================================================
# 1. Host build and host unit tests (active when not cross-compiling)
# ==============================================================================
if(NOT CMAKE_CROSSCOMPILING)
    find_package(Threads REQUIRED)

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
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME review_contract
        COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_review_contract.py")

    # Target hardware layer compiled for the host against tests/target/hal_stub.h
    add_executable(test_target_hal
        tests/target/test_target_hal.c
        tests/target/hal_stub.c
        firmware/target/target_estop.c
        firmware/target/target_fdcan.c
        firmware/target/target_crsf.c
        firmware/target/target_usb_cdc.c
        firmware/target/target_iwdg.c
        firmware/target/target_safety_io.c
    )
    target_compile_definitions(test_target_hal PRIVATE TARGET_HOST_TEST)
    target_include_directories(test_target_hal PRIVATE tests/target firmware/target)
    target_compile_options(test_target_hal PRIVATE -Wall -Wextra -Wconversion -Werror -UNDEBUG)
    target_link_libraries(test_target_hal PRIVATE wbr_firmware m)
    foreach(scenario estop can rx operator watchdog imu radio usb
            opto_pending_permit opto_pending_clear opto_pending_tx
            rev4_mapping emergency_clock watchdog_config fatal_halt_repeat)
        add_test(NAME target_${scenario} COMMAND test_target_hal ${scenario})
    endforeach()

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
    # YAW1: heading wrap, rate limit, path geometry/progress, default-off pass-through
    add_fw_test(test_yaw_track         test_yaw_track.c)

    # FW8: confirmed should-fix findings of the firmware safety review (F-04, F-06).
    add_executable(test_fw8_safety tests/test_fw8_safety.c)
    target_link_libraries(test_fw8_safety PRIVATE wbr_firmware m)
    target_compile_options(test_fw8_safety PRIVATE ${FW_COMMON_WARNINGS})
    apply_undebug_guard(test_fw8_safety)
    foreach(scenario arm_stop startup_motion startup_gravity)
        add_test(NAME fw8_${scenario} COMMAND test_fw8_safety ${scenario})
    endforeach()

    # FW6: STEP3RC part-B frozen-encoder detector (ported host tests) and the
    # firmware reaction (DEGRADED -> CONTROLLED_STOP, partner wheel, escalation).
    add_fw_test(test_s11_frozen_reaction test_s11_frozen_reaction.c)
    add_fw_test(test_runup_balance_ctrl  test_runup_balance_ctrl.c)
    add_fw_test(test_fz_fixtures         test_fz_fixtures.c)
    add_fw_test(test_fz_behaviour        test_fz_behaviour.c)
    add_executable(test_fz_healthy_replay tests/test_fz_healthy_replay.c)
    target_link_libraries(test_fz_healthy_replay PRIVATE wbr_firmware m)
    apply_undebug_guard(test_fz_healthy_replay)
    add_test(NAME test_fz_healthy_replay
        COMMAND test_fz_healthy_replay "${CMAKE_CURRENT_SOURCE_DIR}/tests/fz_data/healthy_corpus.bin")
    # LAT1: 30 N / 100 ms lateral push at mu 0.25/0.38/0.5/0.8 on four mass models (16 FIL runs,
    # -0.5..+3.0 s around the push). The pre-FW6 S11 stuck rule false-tripped on this kind of
    # stream (wheel-slip contact chatter) and cut torque -> fall; must stay CONFIRMED/ADVISORY-free.
    add_test(NAME test_fz_lateral_push_replay
        COMMAND test_fz_healthy_replay "${CMAKE_CURRENT_SOURCE_DIR}/tests/fz_data/lateral_lowmu_push_corpus.bin")

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

    # ==========================================================================
    # 2. Side-by-side STM32H723 image (`wbr_stm32h723`), only when both
    #    arm-none-eabi-gcc and the vendor HAL (WBR_STM32_HAL_DIR) are present.
    #    Portable core + firmware/target/ use -Wall -Wextra -Wconversion -Werror;
    #    vendor HAL/CMSIS sources use -w.
    # ==========================================================================
    find_program(ARM_NONE_EABI_GCC  arm-none-eabi-gcc)
    find_program(ARM_NONE_EABI_AR   arm-none-eabi-ar)
    find_program(ARM_NONE_EABI_SIZE arm-none-eabi-size)

    if(ARM_NONE_EABI_GCC AND WBR_STM32_HAL_FOUND)
        set(STM32_OBJ_DIR "${CMAKE_CURRENT_BINARY_DIR}/stm32h723_objs")
        file(MAKE_DIRECTORY "${STM32_OBJ_DIR}")

        set(STM32_INC_FLAGS "")
        foreach(inc_dir IN LISTS WBR_STM32H723_INCLUDES)
            list(APPEND STM32_INC_FLAGS "-I${inc_dir}")
        endforeach()
        foreach(inc_dir IN LISTS WBR_STM32H723_VENDOR_INCLUDES)
            list(APPEND STM32_INC_FLAGS "-isystem" "${inc_dir}")
        endforeach()

        set(STM32_FW_C_SOURCES
            ${WBR_FIRMWARE_SOURCES}
            ${WBR_STM32H723_TARGET_SOURCES}
        )

        set(STM32_OBJECTS "")
        foreach(src_rel IN LISTS STM32_FW_C_SOURCES)
            string(REPLACE "/" "_" obj_flat "${src_rel}")
            set(obj_file "${STM32_OBJ_DIR}/${obj_flat}.o")
            set(src_abs  "${CMAKE_CURRENT_SOURCE_DIR}/${src_rel}")
            add_custom_command(
                OUTPUT "${obj_file}"
                COMMAND "${ARM_NONE_EABI_GCC}"
                        ${WBR_STM32H723_FW_CFLAGS}
                        ${STM32_INC_FLAGS}
                        -c "${src_abs}"
                        -o "${obj_file}"
                DEPENDS "${src_abs}"
                COMMENT "[STM32H723] Cross-compiling ${src_rel} (-Wall -Wextra -Wconversion -Werror)"
                VERBATIM
            )
            list(APPEND STM32_OBJECTS "${obj_file}")
        endforeach()

        foreach(src_rel IN LISTS WBR_STM32H723_HAL_SOURCES)
            string(REPLACE "/" "_" obj_flat "${src_rel}")
            set(obj_file "${STM32_OBJ_DIR}/hal_${obj_flat}.o")
            set(src_abs  "${WBR_STM32_HAL_DIR}/${src_rel}")
            add_custom_command(
                OUTPUT "${obj_file}"
                COMMAND "${ARM_NONE_EABI_GCC}"
                        ${WBR_STM32H723_HAL_CFLAGS}
                        ${STM32_INC_FLAGS}
                        -c "${src_abs}"
                        -o "${obj_file}"
                DEPENDS "${src_abs}"
                COMMENT "[STM32H723] Cross-compiling vendor HAL ${src_rel} (-w)"
                VERBATIM
            )
            list(APPEND STM32_OBJECTS "${obj_file}")
        endforeach()

        set(startup_obj "${STM32_OBJ_DIR}/startup_stm32h723xx.s.o")
        add_custom_command(
            OUTPUT "${startup_obj}"
            COMMAND "${ARM_NONE_EABI_GCC}"
                    ${WBR_STM32H723_MCU_FLAGS}
                    -x assembler-with-cpp
                    -c "${WBR_STM32H723_STARTUP_ABS}"
                    -o "${startup_obj}"
            DEPENDS "${WBR_STM32H723_STARTUP_ABS}"
            COMMENT "[STM32H723] Assembling startup_stm32h723xx.s"
            VERBATIM
        )
        list(APPEND STM32_OBJECTS "${startup_obj}")

        set(STM32_ELF "${CMAKE_CURRENT_BINARY_DIR}/wbr_stm32h723.elf")
        set(STM32_MAP "${CMAKE_CURRENT_BINARY_DIR}/wbr_stm32h723.map")
        set(STM32_SIZE_COMMAND "")
        if(ARM_NONE_EABI_SIZE)
            set(STM32_SIZE_COMMAND COMMAND "${ARM_NONE_EABI_SIZE}" "${STM32_ELF}")
        endif()
        add_custom_command(
            OUTPUT "${STM32_ELF}"
            COMMAND "${ARM_NONE_EABI_GCC}"
                    ${WBR_STM32H723_MCU_FLAGS}
                    "-T${WBR_STM32H723_LINKER_SCRIPT}"
                    -Wl,--gc-sections
                    "-Wl,-Map=${STM32_MAP}"
                    -specs=nano.specs
                    -specs=nosys.specs
                    ${STM32_OBJECTS}
                    -lm
                    -o "${STM32_ELF}"
            ${STM32_SIZE_COMMAND}
            DEPENDS ${STM32_OBJECTS} "${WBR_STM32H723_LINKER_SCRIPT}"
            COMMENT "[STM32H723] Linking wbr_stm32h723.elf"
            VERBATIM
        )
        add_custom_target(wbr_stm32h723 ALL DEPENDS "${STM32_ELF}")
    elseif(NOT ARM_NONE_EABI_GCC)
        message(STATUS "arm-none-eabi-gcc not found; skipping the linked STM32H723 image")
    else()
        message(STATUS "STM32H7 HAL not found in WBR_STM32_HAL_DIR; skipping the linked STM32H723 image")
    endif()

else()
    # ==========================================================================
    # 3. Direct cross-compile mode (-DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake)
    #    Requires WBR_STM32_HAL_DIR; produces wbr_stm32h723.elf
    # ==========================================================================
    if(NOT WBR_STM32_HAL_FOUND)
        message(FATAL_ERROR "Cross-compiling needs the STM32CubeH7 Drivers/ tree; set WBR_STM32_HAL_DIR")
    endif()
    add_library(wbr_stm32h723_hal OBJECT
        ${WBR_STM32H723_HAL_ABS_SOURCES}
    )
    target_include_directories(wbr_stm32h723_hal PRIVATE ${WBR_STM32H723_INCLUDES})
    target_include_directories(wbr_stm32h723_hal SYSTEM PRIVATE ${WBR_STM32H723_VENDOR_INCLUDES})
    target_compile_options(wbr_stm32h723_hal PRIVATE
        $<$<COMPILE_LANGUAGE:C>:${WBR_STM32H723_HAL_CFLAGS}>
    )

    set_source_files_properties(${WBR_STM32H723_STARTUP_ABS} PROPERTIES
        LANGUAGE ASM
        COMPILE_FLAGS "-x assembler-with-cpp"
    )
    add_executable(wbr_stm32h723
        ${WBR_FIRMWARE_SOURCES}
        ${WBR_STM32H723_TARGET_SOURCES}
        ${WBR_STM32H723_STARTUP_ABS}
        $<TARGET_OBJECTS:wbr_stm32h723_hal>
    )
    set_target_properties(wbr_stm32h723 PROPERTIES
        OUTPUT_NAME "wbr_stm32h723"
        SUFFIX ".elf"
    )
    target_include_directories(wbr_stm32h723 PUBLIC ${WBR_STM32H723_INCLUDES})
    target_include_directories(wbr_stm32h723 SYSTEM PRIVATE ${WBR_STM32H723_VENDOR_INCLUDES})
    target_compile_options(wbr_stm32h723 PRIVATE
        $<$<COMPILE_LANGUAGE:C>:${WBR_STM32H723_FW_CFLAGS}>
        $<$<COMPILE_LANGUAGE:ASM>:${WBR_STM32H723_MCU_FLAGS}>
    )
    target_link_options(wbr_stm32h723 PRIVATE
        ${WBR_STM32H723_MCU_FLAGS}
        "-T${WBR_STM32H723_LINKER_SCRIPT}"
        -Wl,--gc-sections
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/wbr_stm32h723.map"
        -specs=nano.specs
        -specs=nosys.specs
    )
    target_link_libraries(wbr_stm32h723 PRIVATE m)

    find_program(ARM_NONE_EABI_SIZE arm-none-eabi-size)
    if(ARM_NONE_EABI_SIZE)
        add_custom_command(TARGET wbr_stm32h723 POST_BUILD
            COMMAND "${ARM_NONE_EABI_SIZE}" "$<TARGET_FILE:wbr_stm32h723>"
            COMMENT "[STM32H723] ELF section sizes for wbr_stm32h723.elf"
            VERBATIM
        )
    endif()
endif()
