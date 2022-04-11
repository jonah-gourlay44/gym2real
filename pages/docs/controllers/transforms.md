---
layout: default
title: TransformRule
parent: Controllers
grand_parent: Documentation
nav_order: 4
---

# TransformRule

When using encapsulated modules, data can often be in non-contiguous memory or in a heterogeneous formats. Controllers require inputs to be in a specific format and order and the outputs may not be directly usable. `TransformRule` objects allow for copying of data from an input buffer to an output buffer while applying arbitrary operations. 

## `RemapRule`
Takes in a lambda function to be applied on the buffers.

## `RangeRemapRule`
Scales from input range to output range (eg. can map values in interval [-1, 1] to [0, 100]).