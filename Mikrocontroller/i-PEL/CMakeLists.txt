cmake_minimum_required(VERSION 3.7)

include(CMakeMacro.cmake)
set(CMAKE_CROSSCOMPILING yes)
set(CURRENT_COMPILER HOST)


SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F767ZITx_FLASH.ld)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")


# NATIVE SYSTEM
set(NATIVE_SYSTEM_NAME Generic)
set(NATIVE_SYSTEM_PROCESSOR arm)
set(NATIVE_C_COMPILER arm-none-eabi-gcc)
set(NATIVE_C_FLAGS "-g3 -std=gnu99")
set(NATIVE_CXX_COMPILER arm-none-eabi-g++)
set(NATIVE_CXX_FLAGS "-g3 -std=c++17")
set(NATIVE_ASM_COMPILER arm-none-eabi-as)
set(NATIVE_ASM_FLAGS "")

# HOST_SYSTEM
set(HOST_C_COMPILER /usr/bin/gcc)
set(HOST_CXX_COMPILER /usr/bin/g++)
set(HOST_C_FLAGS "-std=gnu99")
set(HOST_CXX_FLAGS "-std=c++17")
set(HOST_ASM_COMPILER /usr/bin/as)
set(HOST_ASM_FLAGS "")

use_native_compiler()

if(False)
    set(TP TRANSPORT_PROTOCOL)
else()
    set(TP)
endif()

PROJECT(i-PEL_DrCTL C CXX ASM)

file(GLOB_RECURSE SOURCES "Drivers/*.*" "Core/*.*")

add_executable(${PROJECT_NAME}.elf ${SOURCES} startup/startup_stm32f767xx.s Core/Src/motor_controller.cpp Core/Src/encoder.cpp Core/Src/pid_controller.cpp Core/Src/timer_initialize.cpp Core/Inc/timer_initialize.h Core/Src/backup.cpp Core/Inc/main.h Core/Inc/gpio.h)

target_include_directories(${PROJECT_NAME}.elf PUBLIC
        Core/Inc
        Drivers/STM32F7xx_HAL_Driver/Inc
        Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F7xx/Include
        Drivers/CMSIS/Include
        ../tool-libs)

set(CPU_FLAGS
        -mcpu=cortex-m7
        -mfloat-abi=hard -mfpu=fpv4-sp-d16
        -mthumb -mthumb-interwork)
set(COMMON_FLAGS
        -ffunction-sections
        -fdata-sections -g -fno-common -fmessage-length=0 -specs=nosys.specs -specs=nano.specs -u _printf_float)
set_source_files_properties(${SOURCES} PROPERTIES
        COMPILE_DEFINITIONS "ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING;__FPU_PRESENT=1;\
        __weak=__attribute__\(\(weak\)\);__packed=__attribute__\(\(__packed__\)\);USE_HAL_DRIVER;STM32F767xx;\
        USE_HAL_TIM_REGISTER_CALLBACKS=1;USE_HAL_UART_REGISTER_CALLBACKS=1;USE_HAL_SPI_REGISTER_CALLBACKS=1;\
        USE_HAL_ADC_REGISTER_CALLBACKS=1;"
        COMPILE_OPTIONS ${COMMON_FLAGS}
        )
target_compile_options(${PROJECT_NAME}.elf PUBLIC
        ${CPU_FLAGS}
        )
target_link_options(${PROJECT_NAME}.elf PUBLIC
        ${CPU_FLAGS}
        ${COMMON_FLAGS}
        LINKER:-gc-sections
        LINKER:--print-memory-usage
        -T ${LINKER_SCRIPT}
        LINKER:-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
        )

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

add_custom_target(UPLOAD
        arm-none-eabi-gdb -iex "target remote tcp:127.0.0.1:3333"
        -iex "monitor program $<TARGET_FILE:${PROJECT_NAME}.elf>"
        -iex "monitor reset init"
        -iex "disconnect" -iex "quit")

add_subdirectory(../tool-libs ./libstests)
