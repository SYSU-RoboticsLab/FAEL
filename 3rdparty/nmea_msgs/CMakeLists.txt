cmake_minimum_required(VERSION 2.8.3)
project(nmea_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(
  FILES
  Gpgga.msg
  Gpgsa.msg
  Gpgsv.msg
  GpgsvSatellite.msg
  Gprmc.msg
  Gpgst.msg
  Sentence.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

install(FILES LICENSE.txt
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
