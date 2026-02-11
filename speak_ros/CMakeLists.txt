cmake_minimum_required(VERSION 3.14)
project(speak_ros)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

# PortAudio dependency
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/audio_queue.cpp
  src/audio_player.cpp
  src/wav_utils.cpp
  src/${PROJECT_NAME}.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${PORTAUDIO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PORTAUDIO_LIBRARIES})
rclcpp_components_register_nodes(${PROJECT_NAME} "speak_ros::SpeakROS")

ament_auto_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

ament_auto_add_executable(list_plugins src/list_plugins.cpp)
target_link_libraries(list_plugins ${PROJECT_NAME})

ament_auto_add_executable(test_client src/speak_ros_test_client.cpp)
target_link_libraries(test_client ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble" OR $ENV{ROS_DISTRO} STREQUAL "jazzy")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package()
endif()
