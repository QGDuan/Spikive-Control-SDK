cmake_minimum_required(VERSION 3.0.2)
project(spikive_control_sdk)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs controller_msgs geometry_msgs
  mavros_msgs message_generation nav_msgs quadrotor_msgs pcl_conversions sensor_msgs roscpp std_msgs)
find_package(PCL REQUIRED COMPONENTS io)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Threads REQUIRED)

add_action_files(FILES takeoff.action land.action goto.action return_home.action relocalize.action Record.action)
add_service_files(FILES LoadMap.srv EmergencyCmd.srv)
add_message_files(FILES LocalizationStatus.msg FlightState.msg)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)
catkin_package(INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} spikive_localization_sdk spikive_record_io
  CATKIN_DEPENDS actionlib actionlib_msgs controller_msgs mavros_msgs message_runtime nav_msgs
                 quadrotor_msgs roscpp std_msgs geometry_msgs sensor_msgs pcl_conversions)

function(sdk_target target)
  target_compile_features(${target} PUBLIC cxx_std_14)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(${target} PUBLIC include ${catkin_INCLUDE_DIRS})
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endfunction()

add_library(${PROJECT_NAME} src/config.cpp src/control_sdk.cpp)
sdk_target(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_library(spikive_localization_sdk src/localization_io.cpp src/localization_sdk.cpp)
sdk_target(spikive_localization_sdk)
target_include_directories(spikive_localization_sdk SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(spikive_localization_sdk ${PROJECT_NAME} ${PCL_LIBRARIES} OpenSSL::Crypto)
add_library(spikive_record_io src/record_io.cpp)
sdk_target(spikive_record_io)
target_link_libraries(spikive_record_io ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(spikive_flight_node src/flight_control_node.cpp)
sdk_target(spikive_flight_node)
target_link_libraries(spikive_flight_node ${PROJECT_NAME})
add_executable(spikive_localization_node src/localization_node.cpp)
sdk_target(spikive_localization_node)
target_link_libraries(spikive_localization_node spikive_localization_sdk)
add_executable(spikive_map_record_node src/map_record_node.cpp)
sdk_target(spikive_map_record_node)
target_link_libraries(spikive_map_record_node spikive_record_io Threads::Threads)
add_executable(spikive_data_mapping_node src/data_mapping_node.cpp src/visual_pointcloud.cpp)
sdk_target(spikive_data_mapping_node)
target_link_libraries(spikive_data_mapping_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} spikive_localization_sdk spikive_record_io
  spikive_flight_node spikive_localization_node spikive_map_record_node spikive_data_mapping_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  foreach(test_name localization_io_test coordinate_contract_test record_io_test)
    catkin_add_gtest(${test_name} test/${test_name}.cpp)
    if(TARGET ${test_name})
      target_link_libraries(${test_name} spikive_localization_sdk spikive_record_io)
    endif()
  endforeach()
  catkin_add_gtest(visual_pointcloud_test test/visual_pointcloud_test.cpp src/visual_pointcloud.cpp)
  if(TARGET visual_pointcloud_test)
    sdk_target(visual_pointcloud_test)
    target_include_directories(visual_pointcloud_test PRIVATE src)
    target_link_libraries(visual_pointcloud_test ${catkin_LIBRARIES})
  endif()
  catkin_install_python(PROGRAMS test/sdk_integration_test.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  catkin_install_python(PROGRAMS test/data_mapping_test.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/sdk_integration.test)
  add_rostest(test/data_mapping.test)
endif()
