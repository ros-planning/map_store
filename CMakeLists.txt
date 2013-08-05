cmake_minimum_required(VERSION 2.8.3)
project(map_store)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  rosconsole
  roscpp
  rostest
  warehouse_ros
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
add_dependencies(map_saver map_store_gencpp)
target_link_libraries(map_saver uuid ${catkin_LIBRARIES})

add_executable(map_manager src/map_manager.cpp)
add_dependencies(map_manager map_store_gencpp)
target_link_libraries(map_manager uuid ${catkin_LIBRARIES})

add_rostest(test/map_store.test)

install(TARGETS map_saver
	        map_manager
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/add_map.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
