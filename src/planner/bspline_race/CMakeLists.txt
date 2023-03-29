cmake_minimum_required(VERSION 3.0.2)
project(bspline_race)

find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  message_generation
  message_filters
)

find_package(Eigen3 REQUIRED)

#add_dependencies(bspline_race ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_message_files(
  FILES
  BsplineTraj.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bspline_race
  CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs message_runtime
  #DEPENDS system_lib
)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${NLOPT_INCLUDE_DIR}
 
)
add_library( bspline_race 
   src/bspline_race.cpp 
   )
   
add_executable( flag_planner 
    src/test_planning.cpp
 )
target_link_libraries(flag_planner 
 bspline_race
 ${catkin_LIBRARIES}
 ${NLOPT_LIBRARIES}
)

