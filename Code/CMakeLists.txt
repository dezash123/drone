# cmake_minimum_required(VERSION <specify CMake version here>)
project(drone)

set(CMAKE_CXX_STANDARD 14)

add_executable(drone
        Control/FlightSystem.cpp Control/FlightSystem.h Output/FlightMotor.cpp Output/FlightMotor.h Main.cpp Main.h Math/Vector3.cpp Math/Vector3.h Control/Drone.cpp Control/Drone.h Control/Router.cpp Control/Router.h Sensors/IMU.cpp Sensors/IMU.h)
