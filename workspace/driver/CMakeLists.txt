cmake_minimum_required(VERSION 3.5)
project(driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(JetsonGPIO REQUIRED)
find_package(Boost REQUIRED)
find_package(device REQUIRED)

add_library(driver
  src/imu_driver.cpp
  src/motor_driver.cpp
)
target_link_libraries(driver
  JetsonGPIO::JetsonGPIO
  imu
  /usr/lib/libi2c.a
  ${BOOST_LIBRARIES}
)

target_include_directories(driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS driver
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include
  DESTINATION include/
)

include_directories(
  include
  /usr/include
  ${BOOST_INCLUDE_DIRS}
)

ament_target_dependencies(driver rclcpp sensor_msgs device)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()