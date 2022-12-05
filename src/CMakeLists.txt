cmake_minimum_required(VERSION 3.0.0)
project(cfPlanning VERSION 0.1.0)

SET(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake) # add FindMatlab module

add_definitions(/DMATLAB_MEX_FILE) #define matlab macros
add_definitions(/DMX_COMPAT_32)

find_package(Matlab REQUIRED)

IF(MATLAB_FOUND)
    message(STATUS "MATLAB Found, MATLAB MEX will be compiled.")
    # add_subdirectory(mex)
ELSE(MATLAB_FOUND)
    MESSAGE("MATLAB not found...nothing will be built.")
ENDIF(MATLAB_FOUND)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIR})

SET(Planning_SRCS
    sensor.cpp
    world.cpp
    main.cpp
)

SET(Planning_HDRS
    planner.h
    sensor.h
    world.h
    util.h
)

# add_executable(cfPlanning ${Planning_SRCS})
set(CPP_FILE cfPlanning)

add_library(${CPP_FILE} SHARED ${Planning_SRCS} ${CMAKE_SOURCE_DIR}/Matlabdef.def)
target_link_libraries(${CPP_FILE} ${MATLAB_LIBRARIES})

# 32-bit or 64-bit mex
if(WIN32)
  if (CMAKE_CL_64)
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexw64)
  else(CMAKE_CL_64)
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexw32)
  endif(CMAKE_CL_64)
else(WIN32)
  if (CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexa64 PREFIX "")
  else(CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexglx PREFIX "")
  endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
endif(WIN32)

# install to /bin by default
install(TARGETS ${CPP_FILE} DESTINATION ../bin)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
