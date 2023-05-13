# cmake_minimum_required(VERSION <specify CMake version here>)
project(drone)

set(CMAKE_CXX_STANDARD 14)

add_executable(drone
        FlightSystem.cpp FlightSystem.h FlightMotor.cpp FlightMotor.h Gyro.cpp Gyro.h Main.cpp Main.h Accelerometer.cpp Accelerometer.h Runnable.h Vector3.cpp Vector3.h Control/Drone.cpp Control/Drone.h)
