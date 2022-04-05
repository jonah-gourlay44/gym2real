---
layout: default
title: Hardware Setup
parent: Setup
nav_order: 1
---

# Hardware Setup

Your Jetson Nano will first need to be flashed with the Nvidia [developer kit image](https://developer.nvidia.com/embedded/downloads). Once setup, ensure you can connect to the Jetson via ssh.

Some changes to the base ubuntu kernel are required to run the system as desired for this project. A real-time kernel patch called PREEMPT_RT has to be installed, and a custom kernel patch to reduce I2C error delay time has to be added.

## PREEMPT_RT Kernel Patch

PREEMPT_RT is a kernel patch that allows for deterministic thread scheduling to be configured. This is crucial for real-time processes to ensure that high-priority processes are scheduled before lower priority ones. To install the kernel patch:

        sudo echo 'deb https://repo.download.nvidia.com/jetson/rt-kernel r32.6.1 main' >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
        sudo apt update
        sudo apt install nvidia-l4-rt-kernel nvidia-l4t-rt-kernel-headers

## Custom I2C Kernel Patch

**Kernel patch instructions here**

Now your Jetson Nano is setup, you are ready to follow the steps in the [Software Insallation](install) page.