cmake_minimum_required(VERSION 3.10)
project(simorgh_flight_software CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

include_directories(include)

set(CORE_SOURCES
    src/attitude_estimator.cpp
    src/attitude_controller.cpp
    src/spacecraft_dynamics.cpp
    src/adcs_system.cpp
)

add_library(simorgh_core ${CORE_SOURCES})

add_executable(demo_slew examples/demo_slew.cpp)
target_link_libraries(demo_slew simorgh_core)

enable_testing()
add_executable(test_simorgh tests/test_simorgh.cpp)
target_link_libraries(test_simorgh simorgh_core)
add_test(NAME unit_tests COMMAND test_simorgh)
