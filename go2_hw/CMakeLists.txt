cmake_minimum_required(VERSION 3.5)
project(go2_hw)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable good warnings if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# list(APPEND CMAKE_PREFIX_PATH "/opt/unitree")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/external/unitree")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(commutils REQUIRED)

# Get root path
get_filename_component(ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)

# Declare the shared library
add_library(${PROJECT_NAME}
  SHARED
    src/joint_controller.cpp
    src/robot_interface.cpp
)
# Specify the include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Link to dependencies
target_link_libraries(${PROJECT_NAME} 
  PUBLIC
    Eigen3::Eigen
    unitree_sdk2
    commutils::commutils
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install the target
install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Export targets for find_package
install(EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)




add_executable(go2_standing 
  demos/go2_standing.cpp
)
target_include_directories(go2_standing
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
set_target_properties(go2_standing
  PROPERTIES COMPILE_DEFINITIONS ROOT_PATH="${ROOT_PATH}"
)
target_link_libraries(go2_standing 
  PUBLIC
    ${PROJECT_NAME}
)

# Export for ament
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
