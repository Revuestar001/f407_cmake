cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RoboF407Cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Bsp/Src/bsp_gpio/bsp_gpio.c
    Bsp/Src/bsp_uart/bsp_uart.c
    Bsp/Src/bsp_spi/bsp_spi.c
    Bsp/Src/bsp_can/bsp_can.c
    Bsp/Src/bsp_dwt/bsp_dwt.c
    Bsp/Src/bsp_i2c/bsp_i2c.c
    Bsp/Src/bsp_critical/bsp_critical.c
    Bsp/Src/bsp_board.c

    Module/Src/util/ring_buffer.c
    Module/Src/protocol/sbus.c
    Module/Src/rc_mapper/rc_mapper.c
    Module/Src/motor_actuator/motor_actuator.c
    Module/Src/motor_actuator/motor_actuator_group.c
    Module/Src/motor_actuator/motor_actuator_dji_adapter.c
    Module/Src/motor_actuator/motor_actuator_myactuator_adapter.c
    Module/Src/device/imu/bmi088.c
    Module/Src/device/magnetometer/ist8310.c
    Module/Src/device/motor/dji_m3508.c
    Module/Src/device/motor/rmd_v2_x6.c
    Module/Src/algorithm/math/quaternion.c
    Module/Src/algorithm/math/vector3.c
    Module/Src/algorithm/math/matrix.c
    Module/Src/algorithm/math/angle.c
    Module/Src/algorithm/math/general_math.c
    Module/Src/algorithm/eskf/error_state_kalman_filter.c
    Module/Src/algorithm/pid/pid.c
    Module/Src/algorithm/filter/1st_order_lp_filter.c
    Module/Src/topic_bus/topic_bus.c
    Module/Src/serial_stream/serial_stream.c

    Application/Src/remote_control/app_remote_control.c
    Application/Src/imu/app_imu.c
    Application/Src/ins/app_ins.c
    Application/Src/chassis/app_chassis.c
    Application/Src/app_topic_bus_test.c
    Application/Src/topics/app_topics.c
    Application/Src/log/app_log.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Bsp/Inc/bsp_gpio/
    Bsp/Inc/bsp_uart/
    Bsp/Inc/bsp_spi/
    Bsp/Inc/bsp_can/
    Bsp/Inc/bsp_dwt/
    Bsp/Inc/bsp_i2c/
    Bsp/Inc/bsp_critical/
    Bsp/Inc/

    Module/Inc/util/
    Module/Inc/protocol/
    Module/Inc/rc_mapper/
    Module/Inc/motor_actuator/
    Module/Inc/device/imu/
    Module/Inc/device/magnetometer/
    Module/Inc/device/motor/
    Module/Inc/algorithm/math/
    Module/Inc/algorithm/eskf/
    Module/Inc/algorithm/pid/
    Module/Inc/algorithm/filter/
    Module/Inc/topic_bus/
    Module/Inc/serial_stream/

    Application/Inc/remote_control/
    Application/Inc/imu/
    Application/Inc/ins/
    Application/Inc/chassis/
    Application/Inc/topics/
    Application/Inc/messages/
    Application/Inc/log/
    Application/Inc/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
