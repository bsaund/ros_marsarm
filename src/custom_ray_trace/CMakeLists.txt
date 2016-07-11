cmake_minimum_required(VERSION 2.8.3)

# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
add_definitions(-std=c++11)

project(custom_ray_trace)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  tf)


add_message_files(
  FILES
  )

add_service_files(
  FILES
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )
catkin_package(
  DEPENDS roscpp
  INCLUDE_DIRS include
  LIBRARIES calc_entropy plot_ray_utils ray_tracer
  )



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  include/${PROJECT_NAME}
  )

add_library(stlParserCustom src/stlParser.cpp)
target_link_libraries(stlParserCustom
  ${catkin_LIBRARIES})

# add_library(ray_trace src/rayTrace.cpp)
# target_link_libraries(ray_trace ${catkin_LIBRARIES})


add_library(calc_entropy src/calcEntropy.cpp)
target_link_libraries(calc_entropy ${catkin_LIBRARIES})

add_library(ray_tracer src/rayTracer.cpp)
target_link_libraries(ray_tracer ${catkin_LIBRARIES} stlParserCustom calc_entropy)


add_library(plot_ray_utils src/plotRayUtils.cpp)
target_link_libraries(plot_ray_utils
  ${catkin_LIBRARIES}
  calc_entropy
  ray_tracer
  )

add_executable(rayPlot src/rayPlot.cpp)
target_link_libraries(rayPlot
  ${catkin_LIBRARIES} 
  ray_tracer
  plot_ray_utils)

add_executable(linePlot src/linePlot.cpp)
target_link_libraries(linePlot
  ${catkin_LIBRARIES} 
  ray_tracer
  plot_ray_utils)



# add_dependencies(plot_ray_utils custom_ray_trace_gencpp)

# add_library(ray_tracer src/rayTracePluginUtils.cpp)
# target_link_libraries(ray_tracer
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${SDF_LIBRARIES}
#   calc_entropy
# )
# add_dependencies(ray_tracer custom_trace_gencpp)
# add_dependencies(ray_tracer gazebo_ray_tracer_generate_messages_cpp)

# add_library(touch_optimizer src/touchOptimizationUtils.cpp)
# target_link_libraries(touch_optimizer
#   ${catkin_LIBRARIES}
#   ray_tracer
# )

# add_executable(custom_rayTrace_plot src/rayTracePlotter.cpp)
# target_link_libraries(custom_rayTrace_plot
#   ${catkin_LIBRARIES}
#   calc_entropy
#   plot_ray_utils
# )




install(TARGETS calc_entropy plot_ray_utils ray_tracer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")



