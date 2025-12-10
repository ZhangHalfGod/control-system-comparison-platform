cmake_minimum_required(VERSION 3.10)

# 项目名称
project(cpp_control_lib VERSION 1.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含目录 - 只包含项目自己的include目录
include_directories(include)

# 源文件
set(SOURCES
    src/pid_controller.cpp
)

# 创建静态库
add_library(${PROJECT_NAME} STATIC ${SOURCES})

# 测试可执行文件
add_executable(test_pid tests/test_simple.cpp)

# 链接测试可执行文件与库
target_link_libraries(test_pid PRIVATE ${PROJECT_NAME})

# 设置安装规则
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES include/pid_controller.hpp DESTINATION include)
