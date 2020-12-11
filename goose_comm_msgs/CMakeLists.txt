cmake_minimum_required(VERSION 2.8.3)
project(goose_comm_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES sensor_data.msg node_link.msg Node.msg Way.msg OsmParcer.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS Boost
)
