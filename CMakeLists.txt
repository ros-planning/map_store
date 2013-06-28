cmake_minimum_required(VERSION 2.8.3)
project(map_store)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  rostest
  warehouse_ros
  nav_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  MapListEntry.msg
)
add_service_files(
  DIRECTORY srv
  FILES
  DeleteMap.srv
  ListMaps.srv
  PublishMap.srv
  RenameMap.srv
  SaveMap.srv
)
generate_messages()

catkin_package( CATKIN_DEPENDS message_runtime )

include_directories(${catkin_INCLUDE_DIRS})

add_executable(map_saver src/map_saver.cpp)
target_link_libraries(map_saver uuid ${catkin_LIBRARIES})

add_executable(map_manager src/map_manager.cpp)
target_link_libraries(map_manager uuid ${catkin_LIBRARIES})

add_rostest(test/map_store.test)
