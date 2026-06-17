cmake_minimum_required(VERSION 3.10)

project(lite3_sdk_deploy)

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

# 启用 backward-cpp 用于段错误排查
# 定义 USE_SIMULATION 宏以启用 main.cpp 中的 backward 代码
# 定义 BACKWARD_HAS_DW=1 以使用 libdw 进行符号解析

# add_definitions(-DUSE_SIMULATION -DBACKWARD_HAS_DW=1)

# 添加调试信息以便 backward 显示行号

# add_compile_options(-g -rdynamic)

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
  state_machine
  run_policy
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/include
)

# On macOS we use a conda/pixi
# provided libonnxruntime.dylib instead of the bundled Linux ELF, and drop
# Linux-only link flags (-lrt). Set BUILD_NATIVE_DEPLOY=OFF to skip.
option(BUILD_NATIVE_DEPLOY "Build the native rl_deploy executable" ON)

if(BUILD_NATIVE_DEPLOY)
# 子目录
add_subdirectory(interface)

# 运行时路径设置
if(APPLE)
  # macOS uses install_name / @rpath instead of --enable-new-dtags.
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  set(CMAKE_INSTALL_RPATH
    "@loader_path"
    "@loader_path/onnxruntime/"
  )
else()
  set(CMAKE_EXE_LINKER_FLAGS "-Wl,--enable-new-dtags")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  set(CMAKE_INSTALL_RPATH
    ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/
    ../lib/onnxruntime/
  )
endif()

file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")

add_executable(rl_deploy main.cpp ${STATE_MACHINE_SRC})

# Locate libonnxruntime. We look for a conda/pixi-provided libonnxruntime (.so/.dylib)
# in the active environment (e.g. via `pixi add onnxruntime-cpp`).
add_library(onnxruntime SHARED IMPORTED)

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  HINTS
    $ENV{CONDA_PREFIX}/lib
    $ENV{PIXI_PROJECT_ROOT}/.pixi/envs/default/lib
    ${CMAKE_PREFIX_PATH}
  PATH_SUFFIXES lib
)

if(NOT ONNXRUNTIME_LIB)
  if(APPLE)
    message(FATAL_ERROR
      "libonnxruntime.dylib not found on macOS. "
      "Install via pixi (`pixi add onnxruntime-cpp`) or conda "
      "(`conda install -c conda-forge onnxruntime-cpp`). "
      "Then re-run colcon build.")
  else()
    # Fallback to the bundled ELF if Pixi environment has no onnxruntime-cpp
    set(ONNXRUNTIME_LIB "${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/libonnxruntime.so")
  endif()
endif()

message(STATUS "Using onnxruntime library: ${ONNXRUNTIME_LIB}")
set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${ONNXRUNTIME_LIB}")

ament_target_dependencies(rl_deploy
  rclcpp
  drdds
  geometry_msgs
)
if(APPLE)
  # macOS: clock_gettime/dlopen are in libSystem, no -lrt; pthread is built-in.
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
    # -ldw  启用backward调试功能需要libdw库
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
    # -ldw  启用backward调试功能需要libdw库
  )
endif()


install(TARGETS rl_deploy
  DESTINATION lib/${PROJECT_NAME}
)
else()
  message(STATUS
    "Skipping native rl_deploy build (BUILD_NATIVE_DEPLOY=OFF). "
    "Python simulation entrypoints will still be installed.")
endif()

foreach(entrypoint IN LISTS LITE3_PYTHON_ENTRYPOINTS)
  file(CHMOD ${CMAKE_CURRENT_SOURCE_DIR}/${entrypoint}
    PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
endforeach()

install(DIRECTORY 
  Lite3_description
  DESTINATION share/${PROJECT_NAME}
)

ament_package()