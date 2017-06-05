project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
   src/ukf.cpp
   src/main.cpp
   src/tools.cpp
   src/ctrv_to_laser_transformer.cpp
   src/ctrv_to_radar_transformer.cpp
   src/entity.cpp
   src/sigma_point_manager.cpp)

add_executable(UnscentedKF ${sources})
