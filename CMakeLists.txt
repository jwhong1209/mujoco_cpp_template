### Set minimum required version of cmake, project name and compile options
cmake_minimum_required(VERSION 3.16)
project(mujoco_cpp_template)

### Set C/C++ Standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

### Set repo's absolute directory
add_definitions(-DPROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

### Set MuJoCo PATH defined in ~/.bashrc
set(MUJOCO_DIR $ENV{MUJOCO_PATH})
link_directories(${MUJOCO_DIR}/lib)

### Find required packages
find_package(Eigen3 REQUIRED)

### Set compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic) # when you can't find build error, try this
  add_compile_options(-Wpedantic)
endif()

### Find source and header files to create executable simulation file
file(GLOB_RECURSE SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/common/src/*.cpp
)

### Create executable and link libraries
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME}
PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include/controller
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include/motion
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include/robot
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include/utilities
  ${MUJOCO_DIR}/include
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
PRIVATE
  mujoco
  glfw
  pthread
)
