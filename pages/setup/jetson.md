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
The current kernel version has an annoying 10 second timeout for I2C bus clears. This is detrimental to the performance of the robot which relies on consistent measurements from the sensor. We have patched and compiled the kernel with a fix for this specific sensor (MPU6050) that reduces the timeout to 10ms. You can find the file at our repo under [kernel_patch](https://github.com/jonah-gourlay44/gym2real). Steps to install the kernel are:

        sudo mkdir /usr/src/kernel/kernel-4.9
        cd /usr/src/kernel/kernel-4.9
        wget https://github.com/jonah-gourlay44/gym2real/blob/main/kernel_patch.tbz2
        tar -xvjf kernel_patch.tbz2

        sudo cp -R /usr/src/kernel/kernel-4.9/build/modules/lib/modules/4.9.253-rt168 /lib/modules
        sudo cp /usr/src/kernel/kernel-4.9/build/kernel/arch/arm64/boot/Image /boot/Image.i2c_patch

Once the image is copied into the boot folder, modify /boot/extlinux/extlinux.conf in order to boot into the new image.
Now your Jetson Nano is setup, you are ready to follow the steps in the [Software Insallation](install) page.