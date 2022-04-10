---
layout: default
title: PidController
parent: Controllers
grand_parent: Documentation
nav_order: 2
---

This controller implements a PID controller. A separate PID controller is created for each observation-action pair. Each total action is the sum of controller outputs given each observation.

# Options
## `observations`
Number of observations (inputs)
## `actions`
Number of actions (outputs)
## `kp`
List of proportional gain (length should be actions*observations)
## `ki`
Integral gain (length should be actions*observations)
## `kd`
Derivative gain (length should be actions*observations)