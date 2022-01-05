cmake_minimum_required(VERSION 3.0.2)
project(nuturtle_robot)


find_package(catkin REQUIRED COMPONENTS
  catch_ros
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  roscpp
  rostest
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  visualization_msgs
)

find_package(rigid2d REQUIRED)
find_package(nuturtlebot REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Generate services in the 'srv' folder
add_service_files(
   FILES
   control.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES roscpp std_msgs
)

catkin_package(
 INCLUDE_DIRS 
  ${catkin_INCLUDE_DIRS}
  ${nuturtlebot_INCLUDE_DIRS}
  ${rigid2d_INCLUDE_DIRS}
  LIBRARIES
  ${rigid2d} 
  ${nuturtlebot_LIBRARIES}
  ${catkin_LIBRARIES}

  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs nuturtlebot rigid2d roscpp sensor_msgs std_msgs std_srvs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${nuturtlebot_INCLUDE_DIRS}
  ${rigid2d_INCLUDE_DIRS}
)

add_executable(turtle_interface src/turtle_interface.cpp)
target_link_libraries(turtle_interface  ${rigid2d_LIBRARIES} ${nuturtlebot_LIBRARIES} ${catkin_LIBRARIES} )


add_executable(follow_circle src/follow_circle.cpp)
target_link_libraries(follow_circle ${rigid2d_LIBRARIES} ${nuturtlebot_LIBRARIES} ${catkin_LIBRARIES} )

enable_testing()
add_executable(turtle_interface_test test/turtle_interface_test.cpp)
target_link_libraries(turtle_interface_test  ${rigid2d_LIBRARIES} ${nuturtlebot_LIBRARIES} ${catkin_LIBRARIES})
add_test(NAME Test_of_interface COMMAND turtle_interface_test)

catch_add_test(${PROJECT_NAME}_test test/turtle_interface_test.cpp)
target_link_libraries(${PROJECT_NAME}_test  ${rigid2d} ${nuturtlebot_LIBRARIES} ${catkin_LIBRARIES}
)


install(TARGETS follow_circle turtle_interface RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)






