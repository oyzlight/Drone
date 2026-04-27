cmake_minimum_required(VERSION 3.0.2)
project(t1_offboard_takeoff)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
## is used, also find other catkin packages cv_bridge
set(cv_bridge_DIR "/usr/local/share/cv_bridge/cmake")
   
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  geometry_msgs
  message_generation
  message_runtime
  mavros_msgs
  tf2_eigen
  tf2_ros
  cv_bridge
  image_transport 
)
find_package(OpenCV 4.5.4 REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED COMPONENTS calib3d dnn)
#find_package(OpenCV REQUIRED calib3d dnn)
find_package(PkgConfig REQUIRED)
# set(InferenceEngine_DIR "~/openvino/cmake")
# set(InferenceEngine_DIR "/usr/local/share/InferenceEngine/cmake")
#find_package(InferenceEngine REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

set(inclue,/home/seeker/seeker_ws/src/t1_offboard_takeoff/include)
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")  # 4.5 版本的 cmake 配置目录
set(OpenCV_INCLUDE_DIRS "/usr/local/include/opencv4")  # 4.5 头文件目录
set(OpenCV_LIB_DIR "/usr/local/lib")  # 4.5 库文件目录

set(ONNXRUNTIME_LIB_PATH "/usr/local/lib")  # onnxruntime库所在目录
set(ONNXRUNTIME_INCLUDE_PATH "/usr/local/include/onnxruntime")  # 头文件目录

# 检查onnxruntime库是否存在
find_library(ONNXRUNTIME_LIB onnxruntime HINTS ${ONNXRUNTIME_LIB_PATH})
if(NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR "onnxruntime library not found! Check ONNXRUNTIME_LIB_PATH.")
endif()
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  # Message1.msg
  # Message2.msg
  Fly.msg
  Point.msg
  Tdata.msg
  Mydata.msg
  wild.msg
  Vedio.msg
  dxy.msg
  clean_wild.msg
  stop.msg
  Detect_animals.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
 
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES t1_offboard_takeoff
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
CATKIN_DEPENDS message_generation message_runtime geometry_msgs roscpp rospy std_msgs serial 
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_PATH}  
  ${InferenceEngine_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/t1_offboard_takeoff.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/t1_offboard_takeoff_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(offboard src/offboard.cpp)
add_executable(serial_node src/serial_node.cpp)
add_executable(livox_to_mavos src/livox_to_mavos.cpp)
add_executable(cv_image_node src/cv_image_node.cpp)
add_executable(cv_image1_node src/cv_image1_node.cpp)
add_executable(cv_num_node src/cv_num_node.cpp)
add_executable(serial_stm32_node src/serial_stm32_node.cpp)


target_link_libraries(offboard  ${catkin_LIBRARIES})
target_link_libraries(serial_node  ${catkin_LIBRARIES})
target_link_libraries(serial_stm32_node  ${catkin_LIBRARIES})
target_link_libraries(cv_image_node  ${catkin_LIBRARIES})
target_link_libraries(livox_to_mavos  ${catkin_LIBRARIES})
target_link_libraries(cv_image_node   ${catkin_LIBRARIES} ${OpenCV_LIBS}  ${ZBAR_LIBRARIES})
target_link_libraries(cv_image1_node   ${catkin_LIBRARIES} ${OpenCV_LIBS}  ${ZBAR_LIBRARIES}  ${InferenceEngine_LIBRARIES})
target_link_libraries(cv_num_node   ${catkin_LIBRARIES} ${OpenCV_LIBS}  ${ZBAR_LIBRARIES}  ${ONNXRUNTIME_LIB})


add_dependencies(cv_num_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cv_image_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cv_image1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_t1_offboard_takeoff.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
