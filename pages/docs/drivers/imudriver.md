---
layout: default
title: IMUDriver
parent: Drivers
grand_parent: Documentation
---

# IMUDriver
A driver for the IMU class. This class is built on top of a ROS2 node, and listens for interrupts being triggered by the IMU's interrupt pin. It publishes a
`sensor_msgs::msg::IMU` message on the /imu_data topic.

## API

### `IMUDriver(int interrupt_pin, int sda_pin, int scl_pin, int address)`
Constructor\\
`interrupt_pin` The GPIO pin connected to the interrupt pin of the IMU\\
`sda_pin` The GPIO pin connected to the SDA pin of the IMU\\
`scl_pin` The GPIO pin connected to the SCL pin of the IMU\\
`address` The address of the IMU