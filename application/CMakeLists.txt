# provide path to dependency libraries

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
#INCLUDE_DIRECTORIES(${XENOMAI_RTDM_INCLUDE_DIR})

LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src)
LINK_DIRECTORIES(/usr/xenomai/lib)

ADD_EXECUTABLE(test-vsa1controlclient vsa1control.cpp)
TARGET_LINK_LIBRARIES(test-vsa1controlclient vsa1control)
PKG_CONFIG_USE_DEPENDENCY (test-vsa1controlclient libxenomai_rtdm)
PKG_CONFIG_USE_DEPENDENCY (test-vsa1controlclient libxenomai_native)
