---
layout: default
title: Memory
parent: Documentation
nav_order: 4
has_children: true
permalink: pages/docs/memory
---

# Memory

We have implemented a basic shared memory interface that allows paged memory to be allocated before program runtime. This makes for faster read/write operations, and prevents time-inefficient page faults from hanging up the main control loop. 