cmake_minimum_required(VERSION 3.26)
include(pico_sdk_import.cmake)
pico_sdk_init()
project(drone C CXX ASM)
set(CMAKE_CXX_STANDARD 17)

add_executable(drone Main.cpp Control/Drone.cpp Control/FlightSystem.cpp Math/Vector3.cpp Control/FlightMotor.cpp IO/Router.cpp Sensors/IMU.cpp)
target_link_libraries(drone PUBLIC
        pico_stdlib
        hardware_spi
        hardware_i2c
        )
