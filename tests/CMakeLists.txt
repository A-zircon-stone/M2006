cmake_minimum_required(VERSION 3.20)
project(ShooterControllerTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(shooter_controller_test
    ShooterControllerTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter/ShooterController.cpp
)

target_include_directories(shooter_controller_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter
)
