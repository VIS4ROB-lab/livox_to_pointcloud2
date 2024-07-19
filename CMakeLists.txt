cmake_minimum_required(VERSION 3.5)
project(livox_to_pointcloud2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare executables
ament_auto_add_executable(livox_to_pointcloud2_node src/livox_to_pointcloud2_node.cpp src/livox_to_pointcloud2.cpp)

# include file direcotry
target_include_directories(livox_to_pointcloud2_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

# link libraries
target_link_libraries(livox_to_pointcloud2_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(livox_to_pointcloud2_node pcl_conversions)

# Install rules
ament_auto_package(INSTALL_TO_SHARE
  launch
)