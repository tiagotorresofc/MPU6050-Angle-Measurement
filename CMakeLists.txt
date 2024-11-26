cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH /home/tiago/freertos-pico/pico-sdk)

#Name project
set(ProjectName mpu6050)

# Include o Pico SDK usando variável de ambiente
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Include o FreeRTOS Kernel (se necessário) usando variável de ambiente
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Define o projeto
project(${ProjectName} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Inicializa o SDK do Raspberry Pi Pico
pico_sdk_init()

# Adiciona o executável
add_executable(${ProjectName}
        mpu6050.hpp
        mpu6050.cpp
        main.cpp
)

# Diretórios de inclusão
include_directories(${CMAKE_SOURCE_DIR})

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PROJECT_SOURCE_DIR}/lib
)

# Adiciona a biblioteca mpu
add_library(mpu
        mpu6050.hpp
        mpu6050.cpp
        main.cpp
)

# Linka as bibliotecas para a biblioteca mpu
target_link_libraries(mpu
        FreeRTOS-Kernel-Heap4
        pico_stdlib
        hardware_i2c
)

# Diretórios de inclusão para a biblioteca mpu
target_include_directories(mpu PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
)

# Linka as bibliotecas para o executável
target_link_libraries(${ProjectName}
    FreeRTOS-Kernel-Heap4
    pico_stdlib
    mpu
)

# Habilita saída USB e desabilita UART padrão
pico_enable_stdio_usb(${ProjectName} TRUE)
pico_enable_stdio_uart(${ProjectName} FALSE)

# Adiciona saídas extras
pico_add_extra_outputs(${ProjectName})

