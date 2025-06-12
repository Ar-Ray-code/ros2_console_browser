cmake_minimum_required(VERSION 3.15)
project(ros2_console_browser_node)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find and include cpp-httplib
find_path(CPP_HTTPLIB_INCLUDE_DIRS httplib.h REQUIRED)
find_library(CPP_HTTPLIB_LIBRARIES NAMES cpp-httplib REQUIRED)

find_package(nlohmann_json REQUIRED)

set(TARGET rest_api_server_node)
ament_auto_add_executable(${TARGET}
  src/rest_api_server_node.cpp
)
target_link_libraries(${TARGET} 
  ${CPP_HTTPLIB_LIBRARIES}
  nlohmann_json::nlohmann_json
)
target_include_directories(${TARGET} PRIVATE ${CPP_HTTPLIB_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()