cmake_minimum_required(VERSION 3.13)
enable_language(ASM)

project( codal-rp2040 )

include("${CODAL_UTILS_LOCATION}")

RECURSIVE_FIND_DIR(TOP_LEVEL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/inc" "*.h")
RECURSIVE_FIND_FILE(TOP_LEVEL_SOURCE0 "${CMAKE_CURRENT_SOURCE_DIR}/source" "*.cpp")
RECURSIVE_FIND_FILE(TOP_LEVEL_SOURCE1 "${CMAKE_CURRENT_SOURCE_DIR}/source" "*.c")

list(APPEND TOP_LEVEL_SOURCE0 "asm/CortexContextSwitch.s")

set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk")
set(PICO_PLATFORM "rp2040")
set(PICO_BOARD "pico")
set(PICO_CONFIG_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/inc/sdk_config.h")
# codal-core provides malloc & friends, do not let pico-sdk wrap them
set(SKIP_PICO_MALLOC 1)
# picotool is not needed (UF2 conversion is done by the target), do not let pico-sdk fetch and build it from git
set(PICO_NO_PICOTOOL 1)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
pico_sdk_init()

set(TARGET_DEPS
    cmsis_core
    hardware_sync
    hardware_irq
    hardware_gpio
    hardware_spi
    hardware_i2c
    hardware_uart
    hardware_pwm
    hardware_flash
    hardware_pio
    hardware_dma
    hardware_regs
    hardware_structs
    hardware_exception
    hardware_divider
    hardware_timer
    pico_platform
    pico_runtime
    pico_float
    pico_double)

# create our target
add_library(codal-rp2040 ${TOP_LEVEL_SOURCE} ${TOP_LEVEL_SOURCE0} ${TOP_LEVEL_SOURCE1})

target_link_libraries(codal-rp2040 codal-core ${TARGET_DEPS})

# Keep the pico-sdk isr_* names for IRQ handlers. Our handlers (isr_timer_0, ...) are otherwise renamed to their CMSIS
# names (TIMER_IRQ_0_Handler, ...) while the pico-sdk 2.0.0 crt0 vector table keeps referencing isr_irqN, leaving all
# IRQs on the default handler. Must be a compile definition, it is evaluated when the pico config header is included.
target_compile_definitions(codal-rp2040 PUBLIC PICO_CMSIS_RENAME_EXCEPTIONS=0)

# expose it to parent cmake.
target_include_directories(codal-rp2040 PUBLIC ${TOP_LEVEL_INCLUDE_DIRS})
