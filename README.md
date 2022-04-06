# gym2real
An advanced educational platform for analyzing reinforcement learning algorithms in realtime.

# Required Materials
- Jetson Nano development board running L4T version 32.6.1 or later

# Installing
First, clone this repository onto your Jetson development board with 

        git clone --recurse-submodules https://github.com/jonah-gourlay44/gym2real

From the gym2real directory, build the Docker image with

        nvidia-docker build -t {image_name} .
        
or pull with

        docker pull jonahg1/zeroshotrt

Perform the post installation steps with 

        sudo scripts/post_inst

Finally, reboot your system to complete the installation

# Usage
Run a new container from the zeroshotrt Docker image with

        ./scripts/docker_run {container_name}

The container can be restarted after exiting with

        docker start -i {container_name}

To build the code from inside the container type

        colcon build
        
From the 'workspace' directory.
