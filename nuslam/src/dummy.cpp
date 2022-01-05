#include <ros/ros.h>
#include <cmath>
#include <armadillo>


int arr x[4]={1,2,3,4};
int arr y[4]={1,2,3,4};















add_executable(${PROJECT_NAME}_node src/tube_world.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node  ${rigid2d_LIBRARIES} ${nuturtlebot_LIBRARIES} ${catkin_LIBRARIES}
 )

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


install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)








