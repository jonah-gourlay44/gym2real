---
layout: default
title: IMU
parent: Devices
grand_parent: Documentation
nav_order: 3
---

# IMU
A class for interfacing with an MPU-6050 IMU chip.

## API

### `IMU(int address, int scl_pin, int sda_pin)`
Constructor\\
`address` The address of the IMU \\
`scl_pin` The GPIO pin connected to the SCL bus \\
`sda_pin` The GPIO pin connected to the SDA bus

### `bool init()`
Initialize the IMU.\\
**returns** True if the initialization is successful and false otherwise

### `bool configure(int address, int value)`
Configure a register byte value\\
`address` The internal address to write to \\
`value` The byte value to write\\
**returns** True if configuring is successful and false otherwise

### `bool getData(DataIMU* data)`
Read all available data from the IMU\\
`data` A DataIMU struct pointer to read data into\\
**returns** True if reading is successful and false otherwise