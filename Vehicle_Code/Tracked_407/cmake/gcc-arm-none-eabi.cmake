set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Toolchain prefix.
# By default, we assume the ARM GNU toolchain is in the system PATH
# (the standard setup for STM32CubeCLT / STM32CubeIDE).
#
# If the toolchain is NOT in PATH on your machine, override the prefix
# below with the absolute path to the toolchain's bin directory,
# keeping the trailing slash.
#
# Example:
#   set(TOOLCHAIN_PREFIX "C:/ST/STM32CubeCLT/GNU-tools-for-STM32/bin/")
#
# Default (PATH-based):
# set(TOOLCHAIN_PREFIX arm-none-eabi-)

# Local override: absolute path to the toolchain bin directory.
set(TOOLCHAIN_PREFIX "D:/RuanJianChengXu1/EmbededSystem/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin/")

# 每个工具都拼上完整文件名，并且带 .exe 后缀
set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}arm-none-eabi-gcc.exe)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}arm-none-eabi-g++.exe)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}arm-none-eabi-g++.exe)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}arm-none-eabi-objcopy.exe)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}arm-none-eabi-size.exe)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m3 ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")