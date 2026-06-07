cmake_minimum_required(VERSION 2.8.3)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  #bspline_opt
  #path_searching
  fastnav_msgs
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  #cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  DataDisp.msg
  MultiBsplines.msg
  MincoTrajectory.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include include/traj_utils/minco
 LIBRARIES traj_utils 
 CATKIN_DEPENDS fastnav_msgs roscpp std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include/traj_utils/minco
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( traj_utils
    src/uniform_bspline.cpp
    src/planning_visualization.cpp 
    src/polynomial_traj.cpp
    src/corridor/safe_corridor_generator.cpp
    src/minco/minco_gcopter_optimizer.cpp
    )

add_dependencies(traj_utils
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries( traj_utils
    ${catkin_LIBRARIES} 
    )  

add_executable(minco_traj_server
    src/minco/minco_traj_server.cpp
)

add_dependencies(minco_traj_server
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(minco_traj_server
    traj_utils
    ${catkin_LIBRARIES}
)
