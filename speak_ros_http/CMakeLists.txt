cmake_minimum_required(VERSION 3.14)
project(speak_ros_http)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()
find_package(ament_cmake_auto REQUIRED)
find_package(cpprestsdk REQUIRED)
find_package(OpenSSL REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED src/http_stream.cpp)
target_link_libraries(${PROJECT_NAME} cpprestsdk::cpprest OpenSSL::SSL OpenSSL::Crypto)
ament_export_dependencies(cpprestsdk OpenSSL)

if(BUILD_TESTING)
  ament_auto_add_executable(pcm_stream_test test/pcm_stream_test.cpp)
  add_test(NAME pcm_stream_contract COMMAND $<TARGET_FILE:pcm_stream_test>)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  CONFIG_EXTRAS_POST "cmake/export_cpprest.cmake"
)
