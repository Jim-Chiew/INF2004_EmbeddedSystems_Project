# Robo Pico build file (top level). Each buddy adds their own target below.
#
# Build, from the repo root:
#   cmake -S . -B ../build-robocar
#   cmake --build ../build-robocar -j4
# (the build output goes outside the repo, so it is never committed)

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w)

# Needs the PICO_SDK_PATH environment variable.
include(pico_sdk_import.cmake)

project(robocar C CXX ASM)
pico_sdk_init()

# ---------------------------------------------------------------------------
# Buddy 1: subsystem 1 bench test (WiFi / communication / telemetry)
# The modules are plain C and are compiled straight from app_program/.
# pico_cyw43_arch_none = WiFi chip driver without the network stack; enough
# to switch the on-board LED, which is wired to the WiFi chip.
# ---------------------------------------------------------------------------
set(S1_DIR ${CMAKE_CURRENT_LIST_DIR}/app_program/subsystem1)

add_executable(buddy1_bench
    testing/subsystem1/bench/buddy1_bench.c
    ${S1_DIR}/com_state.c
    ${S1_DIR}/status_led.c
    ${S1_DIR}/telemetry.c
    ${S1_DIR}/trans_queue.c
)

target_include_directories(buddy1_bench PRIVATE ${S1_DIR})
target_compile_options(buddy1_bench PRIVATE -Wall -Wextra)

pico_set_program_name(buddy1_bench "buddy1_bench")
pico_set_program_version(buddy1_bench "0.1")

# Messages go to USB serial (open it with a serial monitor).
pico_enable_stdio_usb(buddy1_bench 1)
pico_enable_stdio_uart(buddy1_bench 0)

target_link_libraries(buddy1_bench PRIVATE
    pico_stdlib
    pico_cyw43_arch_none
)

pico_add_extra_outputs(buddy1_bench)