cmake_minimum_required(VERSION 3.5)
project(controller)

# Default to C++14
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(ONNXRUNTIME_ROOTDIR "/usr/local/")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    "include"
    "${ONNXRUNTIME_ROOTDIR}/include" 
    "${ONNXRUNTIME_ROOTDIR}/include/onnxruntime/core/session"
    ${YAML_CPP_INCLUDE_DIRS}
    /usr/local/include)
link_directories("${ONNXRUNTIME_ROOTDIR}/lib")

# Core library
add_executable(controller src/controller.cpp)

target_link_libraries(controller
    onnxruntime 
    librt.so
    ${YAML_CPP_LIBRARIES}
)

target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS controller
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include 
    DESTINATION include/
)

ament_target_dependencies(controller rclcpp)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()