cmake_minimum_required(VERSION 3.12)
project(tasks)

# Crear la biblioteca estática "tasks" con los archivos fuente
add_library(tasks STATIC
    task_bluetooth.c
    task_curva.c
    task_recto.c
    task_tacometro_left.c
    task_tacometro_right.c
)

# Linkeo dependencias de la bibliotecas
target_link_libraries(tasks
    pico_stdlib
    hardware_pwm
    hardware_clocks
    hardware_i2c
    freertos
)

# Incluir las cabeceras de la biblioteca
target_include_directories(tasks PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    # Allow tasks sources to include project-wide headers (globals.h, motor.h)
    ${CMAKE_CURRENT_LIST_DIR}/..
    # lcd and rtc are sibling directories to firmware; step up one more level
    ${CMAKE_CURRENT_LIST_DIR}/../../lcd/include
    ${CMAKE_CURRENT_LIST_DIR}/../../rtc/include
)
