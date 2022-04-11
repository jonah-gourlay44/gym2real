---
layout: default
title: OnnxController
parent: Controllers
grand_parent: Documentation
nav_order: 3
---

# OnnxController

This controller loads a ONNX model and performs inference on the GPU with CUDA.

## Options
### `observations`
Number of observations (inputs)
### `actions`
Number of actions (outputs)
### `model_path`
Path of `.onnx` file