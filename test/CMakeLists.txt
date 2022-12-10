cmake_minimum_required(VERSION 3.0.0)
project(cfPlanning_test VERSION 0.1.0)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(Planning_SRCS
    plannerDstar.cpp
    openList.cpp
    nodeDstar.cpp
    sensor.cpp
    world.cpp
    test.cpp
)

SET(Planning_HDRS
    plannerDstar.hpp
    openList.hpp
    nodeDstar.hpp
    planner.h
    sensor.h
    world.h
    util.h
)

# add_library(cfPlanning_lib ${Planning_SRCS})
add_executable(cfPlanning_test ${Planning_SRCS})

# install to /bin by default
install(TARGETS cfPlanning_test RUNTIME DESTINATION ../bin)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
