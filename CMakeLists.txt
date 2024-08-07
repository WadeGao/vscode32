#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.25)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# project settings
project(STM32 C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# Uncomment for software floating point
# add_compile_options(-mfloat-abi=soft)
add_compile_options (-mcpu=cortex-m4 -mthumb -mthumb-interwork --specs=nano.specs --specs=nosys.specs)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_link_options(-u _printf_float)

# uncomment to mitigate c++17 absolute addresses warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
    add_definitions(-DDEBUG)
endif ()

include_directories ( Drivers/STM32F4xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32F4xx/Include Drivers/CMSIS/Include ThreadX/common/inc ThreadX/ports/cortex_m4/gnu/inc Drivers/STM32F4xx_HAL_Driver/Inc/ Core/Inc/ /usr/arm-none-eabi/include)

add_definitions(-DSTM32F407xx)

option(USE_LL "use ll library" OFF)
if(USE_LL)
    message(STATUS "use LL library")
    add_definitions(-DUSE_FULL_LL_DRIVER)
endif ()

option(USE_HAL "use HAL library" OFF)
if(USE_HAL)
    message(STATUS "use HAL library")
    add_definitions(-DUSE_HAL_DRIVER) 
endif ()

file ( GLOB_RECURSE SOURCES "startup_stm32f407xx.s" "Drivers/STM32F4xx_HAL_Driver/*.*" "Core/Src/*.*" "ThreadX/ports/cortex_m4/gnu/src/*.*" "ThreadX/ports/cortex_m4/gnu/example_build/tx_initialize_low_level.S" "ThreadX/common/src/*.*" )

option(TO_RAM "download to ram" OFF)
if (TO_RAM)
    message(STATUS "download to ram")
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407ZGTX_RAM.ld)
else ()
    message(STATUS "default download to flash")
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407ZGTX_FLASH.ld)
endif ()

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options (-mcpu=cortex-m4 -mthumb -mthumb-interwork --specs=nano.specs --specs=nosys.specs)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
