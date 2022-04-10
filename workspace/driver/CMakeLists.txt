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

include_directories(
  include
  /usr/include
  ${BOOST_INCLUDE_DIRS}
)

add_library(driver
  src/imu_driver.cpp
  src/motor_driver.cpp
)
target_link_libraries(driver
  JetsonGPIO::JetsonGPIO
  ${BOOST_LIBRARIES}
)

target_include_directories(driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(driver rclcpp sensor_msgs device)
ament_export_targets(driverTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(device JetsonGPIO)


install(TARGETS driver
  DESTINATION lib/${PROJECT_NAME}
)
  
install(DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS driver
  EXPORT driverTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()