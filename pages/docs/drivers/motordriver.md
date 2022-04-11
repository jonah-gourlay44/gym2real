---
layout: default
title: MotorDriver
parent: Drivers
grand_parent: Documentation
---

# MotorDriver
A class to interface with the Lynxmotion motors. It will publish the motor state as a `sensor_msgs::msg::JointState` message to the /motor_state topic and read motor `sensor_msgs::msg::JointState` messages from the /motor_command topic

## API

### `MotorDriver(int pwm_motor_l, int pwm_motor_r, int encoder_l_a, int encoder_l_b, int encoder_r_a, int encoder_r_b)`
Constructor\\
`pwm_motor_l` The GPIO pin connected to the left motor driver PWM input\\
`pwm_motor_r` The GPIO pin connected to the right motor driver PWM input\\
`encoder_l_a` The GPIO pin connected to the left encoder input a\\
`encoder_l_b` The GPIO pin connected to the left encoder input b\\
`encoder_r_a` The GPIO pin connected to the right encoder input a\\
`encoder_r_b` The GPIO pin connected to the right encoder input b