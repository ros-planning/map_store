cmake_minimum_required(VERSION 2.8.3)
project(map_store)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  warehouse_ros
  nav_msgs
)

add_message_files(
  msg/MapListEntry.msg
)
add_service_files(
  srv/DeleteMap.srv
  srv/ListMaps.srv
  srv/PublishMap.srv
  srv/RenameMap.srv
  srv/SaveMap.srv
)
generate_messages()

catkin_package( CATKIN_DEPENDS message_runtime )

include_directories(${catkin_INCLUDE_DIRS})

rosbuild_genmsg()
rosbuild_gensrv()

add_executable(map_saver src/map_saver.cpp)
target_link_libraries(map_saver uuid)

add_executable(map_manager src/map_manager.cpp)
target_link_libraries(map_manager uuid)

rosbuild_add_rostest(test/map_store.test)
