cmake_minimum_required(VERSION 3.14)
project(speak_ros_openai_tts_plugin)

# cpprestsdk still resolves Boost via find_package(Boost) on some systems.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(cpprestsdk REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME} cpprestsdk::cpprest)

pluginlib_export_plugin_description_file(speak_ros plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
