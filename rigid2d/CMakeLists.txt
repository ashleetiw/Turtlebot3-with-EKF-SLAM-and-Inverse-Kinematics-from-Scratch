cmake_minimum_required(VERSION 3.9)
project(rigid2d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  catch_ros
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rostest
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# Generate services in the 'srv' folder
add_service_files(
   FILES
   setpose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES

)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# create the rigid2d library and add it's include directories
add_library(${PROJECT_NAME} src/diff_drive.cpp src/rigid2d.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include/)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17) # enable C++ 17
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra)

# create the executable  and link it with the rigid2d library
#add_executable(${PROJECT_NAME}_node src/main.cpp)
#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_odom src/odometer.cpp)
target_link_libraries(${PROJECT_NAME}_odom ${PROJECT_NAME} ${catkin_LIBRARIES} )

add_executable(${PROJECT_NAME}_turtle src/fake_turtle.cpp)
target_link_libraries(${PROJECT_NAME}_turtle ${PROJECT_NAME} ${catkin_LIBRARIES} )


install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install (TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_odom ${PROJECT_NAME}_turtle RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)


