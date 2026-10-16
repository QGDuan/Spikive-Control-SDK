cmake_minimum_required(VERSION 3.0.2)
project(spikive_control_sdk)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  controller_msgs
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  quadrotor_msgs
  pcl_conversions
  sensor_msgs
  roscpp
  std_msgs
)

find_package(PCL REQUIRED COMPONENTS io)
find_package(OpenSSL REQUIRED)

add_action_files(FILES takeoff.action land.action goto.action return_home.action relocalize.action)
add_service_files(FILES LoadMap.srv)
add_message_files(FILES LocalizationStatus.msg)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib actionlib_msgs controller_msgs mavros_msgs
                 message_runtime nav_msgs quadrotor_msgs roscpp std_msgs geometry_msgs sensor_msgs pcl_conversions
)

add_library(${PROJECT_NAME} src/config.cpp src/control_sdk.cpp src/localization_io.cpp src/localization_sdk.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} OpenSSL::Crypto)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(spikive_control_sdk_node src/flight_control_node.cpp)
target_link_libraries(spikive_control_sdk_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(spikive_control_sdk_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} spikive_control_sdk_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(localization_io_test test/localization_io_test.cpp)
  if(TARGET localization_io_test)
    target_link_libraries(localization_io_test ${PROJECT_NAME})
  endif()
endif()
