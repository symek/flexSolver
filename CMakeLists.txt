cmake_minimum_required(VERSION 3.17)
project(flexSolver)
set(CMAKE_CXX_STANDARD 14)

# Houdini
list( APPEND CMAKE_PREFIX_PATH "$ENV{HT}/cmake")
find_package( Houdini REQUIRED )
message (STATUS "Houdini HDK: ${Houdini_VERSION}")

# CUDA
# FIXME: Add CUDA CMake module
if(NOT DEFINED ENV{CUDA_HOME})
    message(FATAL_ERROR "Can't proceed without CUDA_HOME point to CUDA installation folder")
else()
    message(STATUS "Building with $ENV{CUDA_HOME}")
endif()

set(CUDA_HOME $ENV{CUDA_HOME})
include_directories(${CUDA_HOME}/include)
link_directories(${CUDA_HOME}/lib64)

# FleX
# TODO: Make FleX submodule?
if(NOT DEFINED ENV{FLEX_HOME})
    message(FATAL_ERROR "Can't proceed without FLEX_HOME point to FLEX installation folder")
else()
    message(STATUS "Building with $ENV{FLEX_HOME}")
endif()

set(FLEX_HOME $ENV{FLEX_HOME})
include_directories(${FLEX_HOME}/include)
link_directories(${FLEX_HOME}/lib/linux64)

include_directories(src)
add_library(flexSolver SHARED
#        src/cloth.h
#        src/fur.cpp
#        src/fur.hpp
#        src/solver.cpp
#        src/solver.hpp
        src/SOP_FlexSolver.cpp
        src/SOP_FlexSolver.hpp
#        src/SOP_FlexWires.cpp
#        src/SOP_FlexWires.hpp
        )

# FIXME
target_link_libraries(flexSolver cudart )
target_link_libraries(flexSolver ${FLEX_HOME}/lib/linux64/NvFlexDeviceDebug_x64.a )
target_link_libraries(flexSolver ${FLEX_HOME}/lib/linux64/NvFlexDebugCUDA_x64.a )
target_link_libraries(flexSolver ${FLEX_HOME}/lib/linux64/NvFlexExtDebugCUDA_x64.a )
target_link_libraries( flexSolver Houdini )
houdini_configure_target( flexSolver )


