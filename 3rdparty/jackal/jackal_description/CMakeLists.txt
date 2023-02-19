cmake_minimum_required(VERSION 2.8.3)
project(jackal_description)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/description.launch)

install(DIRECTORY meshes launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(WIN32)
install(PROGRAMS scripts/env_run.bat
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
else()
install(PROGRAMS scripts/env_run
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()
