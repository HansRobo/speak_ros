cmake_minimum_required(VERSION 3.14)
project(speak_ros)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

find_package(cpprestsdk REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

# PortAudio dependency
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/http_stream.cpp
  src/audio_queue.cpp
  src/audio_player.cpp
  src/topic_audio_sink.cpp
  src/wav_utils.cpp
  src/${PROJECT_NAME}.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${PORTAUDIO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PORTAUDIO_LIBRARIES}
  cpprestsdk::cpprest OpenSSL::SSL OpenSSL::Crypto)
ament_export_dependencies(cpprestsdk OpenSSL)
rclcpp_components_register_nodes(${PROJECT_NAME} "speak_ros::SpeakROS")

ament_auto_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

ament_auto_add_executable(list_plugins src/list_plugins.cpp)
target_link_libraries(list_plugins ${PROJECT_NAME})

ament_auto_add_executable(test_client src/speak_ros_test_client.cpp)
target_link_libraries(test_client ${PROJECT_NAME})

if(BUILD_TESTING)
  ament_auto_add_executable(pcm_stream_test test/pcm_stream_test.cpp)
  target_link_libraries(pcm_stream_test ${PROJECT_NAME})
  add_test(NAME pcm_stream_contract COMMAND $<TARGET_FILE:pcm_stream_test>)
  ament_auto_add_executable(topic_sink_client test/topic_sink_client.cpp)
  target_link_libraries(topic_sink_client ${PROJECT_NAME})
  add_test(NAME topic_sink_contract COMMAND $<TARGET_FILE:topic_sink_client>)
  set_tests_properties(topic_sink_contract PROPERTIES TIMEOUT 15 ENVIRONMENT "ROS_DOMAIN_ID=171")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble" OR $ENV{ROS_DISTRO} STREQUAL "jazzy")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR
    CONFIG_EXTRAS_POST "cmake/export_cpprest.cmake")
else()
  ament_auto_package(CONFIG_EXTRAS_POST "cmake/export_cpprest.cmake")
endif()
