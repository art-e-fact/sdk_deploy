cmake_minimum_required(VERSION 3.10)

project(m20_sdk_deploy)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# 警告忽略和编译选项
add_definitions(-w)
add_compile_options(-fPIC)

# 配置选项
set(BUILD_PLATFORM "x86" CACHE STRING "Target platform (x86 or arm)")

if(BUILD_PLATFORM STREQUAL "arm")
  set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drdds REQUIRED)
find_package(geometry_msgs REQUIRED)

set(THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

include_directories(
  include/types
  include/utils
  interface/robot
  interface/user_command
  interface/network
  state_machine
  run_policy
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/include
)

# On macOS we use a conda/pixi
# provided libonnxruntime.dylib instead of the bundled Linux ELF, and drop
# Linux-only link flags (-lrt). Set BUILD_M20_NATIVE=OFF to skip.
option(BUILD_M20_NATIVE "Build the native M20 rl_deploy executable" ON)

if(BUILD_M20_NATIVE)
# 子目录
add_subdirectory(interface)

# 运行时路径设置
if(APPLE)
  # macOS uses install_name / @rpath instead of --disable-new-dtags.
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  set(CMAKE_INSTALL_RPATH
    "@loader_path"
    "@loader_path/onnxruntime/"
  )
else()
  set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  # Keep vendored dir first: Linux links the bundled ORT 1.22 ELF (see below).
  set(CMAKE_INSTALL_RPATH
    ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/
    ../lib/onnxruntime/
  )
endif()

file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")

add_executable(rl_deploy main.cpp ${STATE_MACHINE_SRC})

# Locate libonnxruntime. On Linux we keep the bundled ELF (.so) shipped under
# third_party/onnxruntime/<plat>/lib. On macOS those ELFs cannot be loaded, so
# we look for a conda/pixi-provided libonnxruntime.dylib in the active
# environment (e.g. via `pixi add onnxruntime-cpp`).
add_library(onnxruntime SHARED IMPORTED)
if(APPLE)
  find_library(ONNXRUNTIME_DYLIB
    NAMES onnxruntime
    HINTS
      $ENV{CONDA_PREFIX}/lib
      $ENV{PIXI_PROJECT_ROOT}/.pixi/envs/default/lib
      ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib
  )
  if(NOT ONNXRUNTIME_DYLIB)
    message(FATAL_ERROR
      "libonnxruntime.dylib not found on macOS. "
      "Install via pixi (`pixi add onnxruntime-cpp`) or conda "
      "(`conda install -c conda-forge onnxruntime-cpp`). "
      "Then re-run colcon build.")
  endif()
  message(STATUS "Using macOS onnxruntime: ${ONNXRUNTIME_DYLIB}")
  set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${ONNXRUNTIME_DYLIB}")
  # The bundled headers under third_party/onnxruntime/x86/include are
  # ABI-stable for ORT >= 1.18 and work with newer dylibs, so we keep them on
  # the include path (set above).
else()
  set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${THIRD_PARTY}/onnxruntime//${BUILD_PLATFORM}/lib/libonnxruntime.so")
endif()

ament_target_dependencies(rl_deploy
  rclcpp
  drdds
  geometry_msgs
)
if(APPLE)
  # macOS: clock_gettime/dlopen are in libSystem, no -lrt; pthread is built-in;
  # libevdev is Linux-only (the evdev keyboard interface is not compiled).
  target_link_libraries(rl_deploy
    interface
    onnxruntime
    -lpthread
    -lm
    -ldl
  )
elseif(BUILD_PLATFORM STREQUAL "arm")
  target_link_libraries(rl_deploy
    interface
    onnxruntime
    -lpthread
    -lm
    -lrt
    -ldl
    -lstdc++
  )
else()
  target_link_libraries(rl_deploy
    interface
    onnxruntime
    -lpthread
    -lm
    -lrt
    -ldl
    -lstdc++
  )
endif()


install(TARGETS rl_deploy
  DESTINATION lib/${PROJECT_NAME}
)
else()
  message(STATUS
    "Skipping native rl_deploy build (BUILD_M20_NATIVE=OFF). "
    "Python simulation entrypoints will still be installed.")
endif()

set(M20_PYTHON_ENTRYPOINTS
  interface/robot/simulation/mujoco_simulation_ros2.py
)

foreach(entrypoint IN LISTS M20_PYTHON_ENTRYPOINTS)
  file(CHMOD ${CMAKE_CURRENT_SOURCE_DIR}/${entrypoint}
    PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
endforeach()

# Install Python simulation scripts (runnable via: ros2 run m20_sdk_deploy mujoco_simulation_ros2.py)
install(PROGRAMS
  interface/robot/simulation/mujoco_simulation_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  interface/robot/simulation/sensors
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.npy"
)

install(DIRECTORY
  M20_description
  DESTINATION share/${PROJECT_NAME}
)

ament_package()