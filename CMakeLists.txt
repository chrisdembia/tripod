project(Tripod CXX)

cmake_minimum_required(VERSION 2.8)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(OpenSim REQUIRED)

set(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release"
      CACHE STRING "Semicolon separated list of supported configuration types.
      Only supports Debug, Release, MinSizeRel, and RelWithDebInfo; anything
      else will be ignored." FORCE )

include_directories(${OPENSIMSIMBODY_INCLUDE_DIRS})
link_directories(${OPENSIM_LIB_DIR})

add_executable(model model.cpp)
message("DEBUG" ${OPENSIMSIMBODY_LIBRARIES})
message("DEBUG ${OPENSIMSIMBODY_LIBRARIES}")
target_link_libraries(model ${OPENSIMSIMBODY_LIBRARIES})

#add_executable(synthesize synthesize.cpp)
#target_link_libraries(synthesize ${OpenSimSimbody_LIBS})


