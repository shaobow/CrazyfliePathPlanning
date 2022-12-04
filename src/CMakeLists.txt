cmake_minimum_required(VERSION 3.0.0)
project(cfPlanning VERSION 0.1.0)

include(CTest)
enable_testing()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})


SET(Planning_SRCS
    #sensor.cpp
    #world.cpp
    main.cpp
)

SET(Planning_HDRS
    #planner.h
    #sensor.h
    #world.h
    util.h
)

add_executable(cfPlanning ${Planning_SRCS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
