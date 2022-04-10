---
layout: default
title: MemoryManager
parent: Memory
grand_parent: Documentation
---

The `MemoryManager` class can preallocate buffers and controllers from a YAML configuration file.

# YAML Configuration
## Buffers
Buffers can be added as key-value pairs under `buffers`. The key will be the name of the shared memory object using the POSIX API. The value will be the number of bytes the buffer should use (eg. for a `float32` buffer of length 4, the value should be 4*4=16).

## Controllers
Controllers can be added under `controllers`. Currently only the `onnx` type is implemented. An `onnx` controller can load a `.onnx` file whose path is under the key `model_path`.

# API
## `create_buffer`
Creates a buffer that can access the corresponding shared memory object.

## `get_controller`
Returns the controller with the given key.