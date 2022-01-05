cmake_minimum_required(VERSION 3.0.2)
project(nuslam)


find_package(catkin REQUIRED COMPONENTS
  catch_ros
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

find_package(rigid2d REQUIRED)
find_package(nuturtlebot REQUIRED)
find_package(Armadillo REQUIRED)

catkin_package(
  INCLUDE_DIRS include

  LIBRARIES
  ${rigid2d} 
  ${PROJECT_NAME}
  ${nuturtlebot_LIBRARIES}
  ${catkin_LIBRARIES}

  CATKIN_DEPENDS geometry_msgs nav_msgs nuturtlebot rigid2d roscpp sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)


include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${nuturtlebot_INCLUDE_DIRS}
  ${rigid2d_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} src/kalman.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include/)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17) # enable C++ 17
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra)

add_executable(${PROJECT_NAME}_node src/slam.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${rigid2d_LIBRARIES} ${nuturtlebot_LIBRARIES} ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_executable(${PROJECT_NAME}_dumb src/landmarks.cpp)
target_link_libraries(${PROJECT_NAME}_dumb ${PROJECT_NAME} ${rigid2d_LIBRARIES} ${nuturtlebot_LIBRARIES} ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})


install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)


install(TARGETS ${PROJECT_NAME}_dumb ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)


