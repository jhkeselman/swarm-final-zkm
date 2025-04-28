cmake_minimum_required(VERSION 3.8.0)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(ARGoS REQUIRED)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

add_library(footbot_foraging SHARED footbot_foraging.h footbot_foraging.cpp)
target_link_libraries(footbot_foraging
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)

set(foraging_loop_functions_SOURCES foraging_loop_functions.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(foraging_loop_functions_SOURCES
    ${foraging_loop_functions_SOURCES}
    foraging_qt_user_functions.cpp)
endif(ARGOS_QTOPENGL_FOUND)

add_library(foraging_loop_functions MODULE ${foraging_loop_functions_SOURCES})
target_link_libraries(foraging_loop_functions
  footbot_foraging
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(foraging_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
