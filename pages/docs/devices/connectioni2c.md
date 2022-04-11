---
layout: default
title: ConnectionI2C
parent: Devices
grand_parent: Documentation
nav_order: 2
---

# ConnectionI2C
A Device implementation that can read and write to/from an I2C channel.

## API

### `ConnectionI2C(int address, int scl_pin, int sda_pin)`
Constructor \\
`address` The I2C device's address \\
`scl_pin` The GPIO pin connected to the SCL bus \\
`sda_pin` The GPIO pin connected to the SDA bus

### `template <typename T> bool read(int address, T* data, int length)`
Read data from the I2C device \\
`address` The internal address to read from \\
`data` Data buffer to read into \\
`length` Length of data to read (number of elements)\\
**returns** True if the read is successful and false otherwise

### `template <typename T> bool write(int address, T* data, int length)`
Write data to the I2C device \\
`address` The internal address to write to \\
`data` Data buffer to read from \\
`length` Length of data to write (number of elements)\\
**returns** True if the write is successful and false otherwise