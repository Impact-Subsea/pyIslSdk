cmake_minimum_required(VERSION 3.5)
project(pyIslSdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DPYTHON_WRAPPER)

if (X32_BIT)
    add_definitions(-DSYSTEM_32BIT)
    message(STATUS "App build config 32 bit")  
else()
    add_definitions(-DSYSTEM_64BIT)
    message(STATUS "App build config 64 bit") 
endif()

if (WIN32)
    add_definitions(-DOS_WINDOWS)
    message(STATUS "App build config Windows")
elseif (UNIX)
    add_definitions(-DOS_UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fvisibility=hidden")
    message(STATUS "App build config Unix")
endif()

find_package(PythonLibs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/islSdk/src")
include_directories("${PROJECT_SOURCE_DIR}/pybind11/include")

add_subdirectory(pybind11)
add_subdirectory(islSdk)

set(SOURCES
    src/sdkBinding.cpp
    src/typesBinding.cpp
    src/ahrsBinding.cpp
    src/loggingBinding.cpp
    src/sysPortBinding.cpp
    src/deviceBinding.cpp 
    src/isa500Binding.cpp
    src/isd4000Binding.cpp
    src/ism3dBinding.cpp
    src/sonarBinding.cpp
)

set(HEADERS
    src/typesBinding.h
    src/ahrsBinding.h
    src/loggingBinding.h
    src/sysPortBinding.h
    src/deviceBinding.h
    src/isa500Binding.h
    src/isd4000Binding.h
    src/ism3dBinding.h
    src/sonarBinding.h
)

pybind11_add_module(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} PRIVATE islSdk)