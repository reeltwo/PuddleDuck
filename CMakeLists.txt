cmake_minimum_required(VERSION 3.10)

project(Puddle VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include(CheckSymbolExists)
include(CheckFunctionExists)

check_function_exists(clock_gettime HAVE_CLOCK_GETTIME)
check_function_exists(gettimeofday HAVE_GETTIMEOFDAY)
check_function_exists(ftime HAVE_FTIME)

check_symbol_exists(CLOCK_REALTIME "time.h" HAVE_CLOCK_REALTIME)
check_symbol_exists(CLOCK_MONOTONIC "time.h" HAVE_CLOCK_MONOTONIC)

configure_file(include/config.h.in ${CMAKE_BINARY_DIR}/config.h)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/rimim/yaml-cpp.git
  GIT_TAG yaml-cpp-0.7.0
)
FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
  message(STATUS "Fetching yaml-cpp...")
  FetchContent_Populate(yaml-cpp)
  add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()

add_executable(puddle src/puddle.cpp)

add_custom_command(
        TARGET puddle POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_SOURCE_DIR}/default.yaml
                ${CMAKE_CURRENT_BINARY_DIR})

target_include_directories(puddle PRIVATE include ${CMAKE_BINARY_DIR})
target_link_libraries(puddle PRIVATE yaml-cpp::yaml-cpp)
target_link_libraries(puddle PRIVATE ${EXTRA_LIBS})

add_executable(gochangeid src/gochangeid.cpp)
target_include_directories(gochangeid PRIVATE include ${CMAKE_BINARY_DIR})
