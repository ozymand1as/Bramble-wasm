cmake_minimum_required(VERSION 3.10)
project(Bramble
    VERSION 0.43.0
    DESCRIPTION "A lightweight RP2040/RP2350 emulator in C"
    LANGUAGES C
)

# C Standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -pedantic -g")

if(NOT APPLE AND NOT EMSCRIPTEN)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-stringop-overflow")
endif()

# ========================================================================
# Optional Features
# ========================================================================

option(ENABLE_FUSE "Enable FUSE filesystem mount support (requires libfuse3)" OFF)

# Include directories
include_directories(include include/rp2040 include/rp2350_rv include/rp2350_arm)

# ========================================================================
# Source files (shared between all targets)
# ========================================================================

set(EMU_SOURCES
    src/cpu.c
    src/instructions.c
    src/thumb32.c
    src/membus.c
    src/uf2.c
    src/elf.c
    src/gpio.c
    src/timer.c
    src/nvic.c
    src/clocks.c
    src/adc.c
    src/rom.c
    src/uart.c
    src/spi.c
    src/i2c.c
    src/pwm.c
    src/dma.c
    src/pio.c
    src/gdb.c
    src/usb.c
    src/rtc.c
    src/netbridge.c
    src/wire.c
    src/storage.c
    src/sdcard.c
    src/emmc.c
    src/fatfs.c
    src/fuse_mount.c
    src/w5500.c
    src/bme280.c
    src/corepool.c
    src/cyw43.c
    src/tapif.c
    src/devtools.c
    src/rp2350_rv/rv_cpu.c
    src/rp2350_rv/rv_clint.c
    src/rp2350_rv/rv_membus.c
    src/rp2350_rv/rv_bootrom.c
    src/rp2350_rv/rp2350_periph.c
    src/rp2350_rv/picobin.c
    src/rp2350_arm/m33_cpu.c
)

# ========================================================================
# FUSE support (auto-detect or explicit)
# ========================================================================

if(ENABLE_FUSE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(FUSE3 REQUIRED fuse3)
    add_compile_definitions(ENABLE_FUSE)
    include_directories(${FUSE3_INCLUDE_DIRS})
    set(FUSE_LIBS ${FUSE3_LIBRARIES})
    message(STATUS "FUSE support: enabled (${FUSE3_VERSION})")
else()
    set(FUSE_LIBS "")
    message(STATUS "FUSE support: disabled (use -DENABLE_FUSE=ON to enable)")
endif()

# Common link libraries
set(COMMON_LIBS m pthread ${FUSE_LIBS})

# ========================================================================
# Targets
# ========================================================================

# Main executable
add_executable(bramble src/main.c ${EMU_SOURCES})
target_link_libraries(bramble ${COMMON_LIBS})
add_custom_command(TARGET bramble POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:bramble> ${CMAKE_SOURCE_DIR}/bramble
    COMMENT "Copying bramble to project root"
)

# Test executable
enable_testing()
add_executable(bramble_tests tests/test_suite.c ${EMU_SOURCES})
target_link_libraries(bramble_tests ${COMMON_LIBS})
add_test(NAME bramble_tests COMMAND bramble_tests)

# Benchmark executable
add_executable(bramble_bench tests/benchmark.c ${EMU_SOURCES})
target_link_libraries(bramble_bench ${COMMON_LIBS})

# Install target
install(TARGETS bramble DESTINATION bin)

# ========================================================================
# Build summary
# ========================================================================

message(STATUS "")
message(STATUS "Bramble RP2040 Emulator v${PROJECT_VERSION}")
message(STATUS "  Build type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "  FUSE mount:    ${ENABLE_FUSE}")
message(STATUS "  C compiler:    ${CMAKE_C_COMPILER}")
message(STATUS "")

if(EMSCRIPTEN)
    add_executable(bramble_web src/picocalc_web.c ${EMU_SOURCES})
    set_target_properties(bramble_web PROPERTIES OUTPUT_NAME "bramble")
    set_target_properties(bramble_web PROPERTIES SUFFIX ".js")
    target_link_options(bramble_web PRIVATE 
        "-sEXPORTED_FUNCTIONS=_picocalc_web_init,_picocalc_web_load_uf2,_picocalc_web_load_sd_image,_picocalc_web_start,_picocalc_web_get_display_buffer,_picocalc_web_set_key,_picocalc_web_get_spi_count,_picocalc_web_get_pixel_count,_picocalc_web_get_uart_count,_picocalc_web_get_sh_count,_picocalc_web_get_uart_enabled,_picocalc_web_get_last_sd_cmd,_picocalc_web_get_last_sd_sector,_picocalc_web_get_last_sd_miso,_picocalc_web_get_sio_launch_count,_picocalc_web_get_nvic_pending,_picocalc_web_get_nvic_enabled,_picocalc_web_get_pc0,_picocalc_web_get_pc1,_picocalc_web_get_sp0,_picocalc_web_get_sp1,_picocalc_web_get_core_flags,_picocalc_web_get_irq,_picocalc_web_get_fault_frame,_picocalc_web_get_pc_trace,_picocalc_web_get_regs,_picocalc_web_dump_mem,_picocalc_web_read_mem16,_picocalc_web_get_uart_log,_picocalc_web_get_total_steps,_picocalc_web_get_i2c_write_count,_picocalc_web_get_i2c_write_log,_malloc,_free"
        "-sEXPORTED_RUNTIME_METHODS=ccall,cwrap,addFunction,removeFunction,writeArrayToMemory,HEAPU8,getValue,setValue,UTF8ToString"
        "-sDEFAULT_LIBRARY_FUNCS_TO_INCLUDE=malloc,free"
        "-sASYNCIFY"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sMODULARIZE=1"
        "-sEXPORT_NAME=createBramble"
        "-sERROR_ON_UNDEFINED_SYMBOLS=0"
    )
endif()
