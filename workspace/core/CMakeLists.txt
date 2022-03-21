cmake_minimum_required(VERSION 3.5)
project(core)

# Default to C++14
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(JetsonGPIO REQUIRED)
find_package(Boost REQUIRED)

# Core library
add_library(core

)
target_link_libraries(core
    /usr/local/lib/libi2c.a
    JetsonGPIO::JetsonGPIO
    ${BOOST_LIBRARIES}
)

include_directories(
    include
    /usr/local/include
    ${BOOST_INCLUDE_DIRS}
)

install(TARGETS core
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include 
    DESTINATION include/
)

ament_target_dependencies(core rclcpp)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()