cmake_minimum_required(VERSION 3.20)
project(RobotSimToolkit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Godot-cpp 路径
set(GODOT_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/godot-cpp)

# 检查 godot-cpp 是否存在
if(NOT EXISTS ${GODOT_CPP_DIR})
    message(FATAL_ERROR "godot-cpp not found. Please run: git submodule update --init --recursive")
endif()

# 平台检测
if(WIN32)
    set(TARGET_PLATFORM "windows")
    set(LIBRARY_PREFIX "")
    set(LIBRARY_EXTENSION ".dll")
elseif(UNIX AND NOT APPLE)
    set(TARGET_PLATFORM "linux")
    set(LIBRARY_PREFIX "lib")
    set(LIBRARY_EXTENSION ".so")
elseif(APPLE)
    set(TARGET_PLATFORM "macos")
    set(LIBRARY_PREFIX "lib")
    set(LIBRARY_EXTENSION ".dylib")
endif()

# 架构检测
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(TARGET_ARCH "x86_64")
else()
    set(TARGET_ARCH "x86_32")
endif()

# 构建 godot-cpp
add_subdirectory(${GODOT_CPP_DIR})

# 源文件
set(SOURCES
    src/register_types.cpp
    src/enhanced_motor_joint.cpp
    src/enhanced_physics_material.cpp
)

# 创建共享库
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# 包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${GODOT_CPP_DIR}/include
    ${GODOT_CPP_DIR}/gen/include
)

# 链接 godot-cpp
target_link_libraries(${PROJECT_NAME} PRIVATE godot::cpp)

# 输出目录（直接输出到 Godot 插件目录）
set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../godot_project/addons/robot_sim_toolkit/bin)

set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "robotparts"
    PREFIX "${LIBRARY_PREFIX}"
    SUFFIX ".${TARGET_PLATFORM}.${TARGET_ARCH}${LIBRARY_EXTENSION}"
    LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}
)

# 编译选项
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /EHsc)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
endif()

# 调试/发布配置
if(CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_ENABLED)
endif()

# 打印配置信息
message(STATUS "=== Robot Sim Toolkit Build Configuration ===")
message(STATUS "Platform: ${TARGET_PLATFORM}")
message(STATUS "Architecture: ${TARGET_ARCH}")
message(STATUS "Output: ${OUTPUT_DIR}/robotparts.${TARGET_PLATFORM}.${TARGET_ARCH}${LIBRARY_EXTENSION}")
message(STATUS "===========================================")
