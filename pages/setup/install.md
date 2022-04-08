---
layout: default
title: Software Installation
parent: Setup
nav_order: 2
---

# Software Installation

## Quickstart

First, clone this repository onto your Jetson development board with

        git clone https://github.com/jonah-gourlay44/gym2real

Pull the docker environment with

        docker pull jonahg1/zeroshotrt

## Source Build

If you would rather build your environment from source, start in the gym2real repo and type

        git submodule init
        git submodule update
        ./build
        nvidia-docker build -t {image_name} .

This will take a significant amount of time. Best to grab yourself a coffee and maybe go for a jog at this point.

## Post Installation & Usage

Finally, complete the post installation steps with

        sudo scripts/post_inst

A container for the Docker image can be run with

        ./scripts/docker_run {container_name} {image_name}

The value for image_name will be jonahg1/zeroshotrt if you chose to pull from DockerHub, and whatever you named your image otherwise. The container can be started again after exiting with

        docker start -i {container_name}

To build the code in the Docker container type

        colcon build

From the 'workspace' directory