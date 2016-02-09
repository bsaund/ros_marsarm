cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ray_trace)


find_package(catkin REQUIRED
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  gazebo_ros
  tf)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})




add_service_files(
  FILES
  RayTrace.srv
  RayTraceEachParticle.srv
  RayTraceEntropy.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
 DEPENDS 
 roscpp
 gazebo_ros
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  hpp)


add_library(calc_entropy src/calcEntropy.cpp)
# add_executable(gazebo_rayTrace_service src/rayTraceService.cpp)
add_executable(gazebo_rayTrace_test src/rayTraceTestingClient.cpp)
add_executable(gazebo_rayTrace_plot src/rayTracePlotter.cpp)



# add_dependencies(gazebo_rayTrace_pluggin gazebo_ray_trace_gencpp)
add_dependencies(gazebo_rayTrace_test gazebo_ray_trace_gencpp)
add_dependencies(gazebo_rayTrace_plot gazebo_ray_trace_gencpp)
# add_dependencies(gazebo_rayTrace_service gazebo_ray_trace_gencpp)

target_link_libraries(gazebo_rayTrace_test
  ${catkin_LIBRARIES}
)

target_link_libraries(gazebo_rayTrace_plot
  ${catkin_LIBRARIES}
  calc_entropy
)

# target_link_libraries(gazebo_rayTrace_service
#   ${catkin_LIBRARIES}
# )


add_library(distance_ray_trace SHARED src/rayTracePlugin.cpp)
target_link_libraries(distance_ray_trace
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${SDF_LIBRARIES}
)




