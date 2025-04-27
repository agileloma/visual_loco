cmake_minimum_required(VERSION 3.5)
project(go2_hw)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(commutils REQUIRED)


# build the main library
add_library(${PROJECT_NAME}
  SHARED
    src/hw_robot_setting.cpp
    src/joint_controller.cpp
    # src/${PROJECT_NAME}/hw_go2_interface.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:
      ${PROJECT_SOURCE_DIR}/include
      ${PROJECT_SOURCE_DIR}
    >
    $<INSTALL_INTERFACE: include>
)
target_link_libraries(${PROJECT_NAME} 
  PUBLIC
    Eigen3::Eigen
    unitree_sdk2
    commutils
)

# add_executable(go2_low_level 
#   example/go2_low_level.cpp
# )
# target_include_directories(go2_low_level
#   PUBLIC 
#     $<BUILD_INTERFACE:
#       ${CMAKE_CURRENT_SOURCE_DIR}/include
#     >
#     $<INSTALL_INTERFACE:
#       ${CMAKE_CURRENT_SOURCE_DIR}/include
#     >
# )
# target_link_libraries(go2_low_level 
#   PUBLIC
#     unitree_sdk2
# )


ament_package()
