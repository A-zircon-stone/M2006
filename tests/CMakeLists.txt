cmake_minimum_required(VERSION 3.20)
project(ShooterControllerTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(shooter_controller_test
    ShooterControllerTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter/ShooterController.cpp
)

add_executable(shooter_remote_test
    ShooterRemoteTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter/ShooterRemote.cpp
)

add_executable(rc_print_formatter_test
    RCPrintFormatterTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter/RCPrintFormatter.cpp
)

add_executable(feeder_telemetry_formatter_test
    FeederTelemetryFormatterTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter/FeederTelemetryFormatter.cpp
)

add_executable(feeder_pid_safety_test
    FeederPidSafetyTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter/FeederPidSafety.cpp
)

add_executable(feeder_fire_controller_test
    FeederFireControllerTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter/FeederFireController.cpp
)

add_executable(sliding_integral_limit_test
    SlidingIntegralLimitTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Framework/Sliding/Sliding.cpp
)

add_executable(feeder_stall_recovery_controller_test
    FeederStallRecoveryControllerTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter/FeederStallRecoveryController.cpp
)

add_executable(friction_ready_controller_test
    FrictionReadyControllerTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter/FrictionReadyController.cpp
)

add_executable(shooter_telemetry_formatter_test
    ShooterTelemetryFormatterTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter/ShooterTelemetryFormatter.cpp
)

target_include_directories(shooter_controller_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter
)

target_include_directories(shooter_remote_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter
)

target_include_directories(rc_print_formatter_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter
)

target_include_directories(feeder_telemetry_formatter_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter
)

target_include_directories(feeder_pid_safety_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter
)

target_include_directories(feeder_fire_controller_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter
)

target_include_directories(sliding_integral_limit_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/stubs
    ${CMAKE_CURRENT_LIST_DIR}/../User/Framework/Sliding
    ${CMAKE_CURRENT_LIST_DIR}/../User/Framework/Debug
)

target_include_directories(feeder_stall_recovery_controller_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter
)

target_include_directories(friction_ready_controller_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter
)

target_include_directories(shooter_telemetry_formatter_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter
)
