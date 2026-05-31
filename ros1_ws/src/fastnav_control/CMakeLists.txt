cmake_minimum_required(VERSION 3.0.2)
project(fastnav_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  mavros_msgs
  fastnav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## catkin specific configuration ##
################################################
catkin_package(
  INCLUDE_DIRS include    #  本包的include下的所有.h文件
  LIBRARIES fastnav_control_core    #  其他包在依赖这个包时，应该链接这些库
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs mavros_msgs fastnav_msgs   # 本包的依赖库
)
# SEEME: catkin_package()用于对外声明这个包的接口信息

###########
## Build ##
###########

## Specify additional locations of header files
## NOTE: 这个include_directories()函数是用来指定头文件的搜索路径的。它告诉编译器在编译过程中在哪里寻找头文件(SEEME: 自己定义的头文件库)。
include_directories(
  include    # 本身include目录下的头文件
  ${catkin_INCLUDE_DIRS} # 依赖包的头文件路径
)
# SEEME: catkin_INCLUDE_DIRS 的数值是由 find_package(catkin ...) 之后自动设置的，它包含了所有 catkin 包的头文件路径。通过将 ${catkin_INCLUDE_DIRS} 添加到 include_directories() 中，我们确保了编译器能够找到所有依赖包的头文件。

## Declare a C++ library (core logic, no main function)
add_library(fastnav_control_core
  src/px4_offboard_interface.cpp
  src/control_fsm.cpp
)
# SEEME: 添加了一个名为 fastnav_control_core 的库，这个库包含了 px4_offboard_interface.cpp 和 control_fsm.cpp 两个源文件。这个库可以被其他可执行文件链接使用。
# 这个与add_executable()不同，add_library()是用来创建一个库文件的(你的这些cpp中不能有main函数)，而add_executable()是用来创建一个可执行文件的。

## Link libraries to the core library (ROS dependencies)
target_link_libraries(fastnav_control_core
  ${catkin_LIBRARIES}  # 把这个编译连接到的文件链接到这个库
)

## Declare a C++ executable (offboard control node)
add_executable(offboard_control_node
  src/offboard_control_node.cpp
)
# SEEME: 添加了一个 offboard_control_node.cpp 的可执行文件offboard_control_node，并链接了这个 offboard_control_node可执行文件所需要的 fastnav_control_core 和 catkin 库（NOTE: 依赖的库 ）。

## Link libraries to the executable
## Include custom library (fastnav_control_core) and ROS dependencies
target_link_libraries(offboard_control_node # 将这个可执行文件链接到以下库
  fastnav_control_core # 自定义库
  ${catkin_LIBRARIES} # ROS等依赖库
)

#############
## Install ##
#############
# (Optional install rules, left commented for now)
