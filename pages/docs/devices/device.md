---
layout: default
title: Device
parent: Devices
grand_parent: Documentation
nav_order: 1
---

# Device
A basic class to connect and check connections to a hardware device. 

## API

### `virtual bool connect()`
Pure virtual function. Override with an implementation for connecting the device. Set the protected member variable initialized_ to true.\\
**returns** True if the connection is successful and false otherwise.

### `void checkConnection()`
Throw a runtime error if the device has not been properly initialized