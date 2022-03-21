# Build third party dependencies
FROM nvcr.io/nvidia/l4t-base:r32.6.1

# Install build dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    	build-essential \
	    software-properties-common \
        pkg-config \
        ca-certificates \
        wget \
        git \
        curl \
        gnupg2 \
        lsb-release \
	    libopenblas-dev \
	    libnuma-dev \
        unattended-upgrades

# Install CMake
RUN cd /tmp && \
    wget https://github.com/Kitware/CMake/releases/download/v3.21.1/cmake-3.21.1-linux-aarch64.sh && \
    bash cmake-3.21.1-linux-aarch64.sh --prefix=/usr/local --exclude-subdir --skip-license
RUN rm -rf /tmp/*

# libi2c
ADD libi2c /root/libi2c/
WORKDIR /root/libi2c
RUN make libi2c.a

# JetsonGPIO
ADD JetsonGPIO /root/JetsonGPIO/
RUN mkdir -p /root/JetsonGPIO/build
WORKDIR /root/JetsonGPIO/build
RUN cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/ && make install

# onnxruntime
ADD onnxruntime /root/onnxruntime/
WORKDIR /root/onnxruntime
RUN ls /usr/local 
RUN ./build.sh \
        --use_cuda \
        --cuda_home /usr/local/cuda \
        --cudnn_home /usr/lib/aarch64-linux-gnu \
        --use_tensorrt \
        --tensorrt_home /usr/lib/aarch64-linux-gnu \
        --config RelWithDebInfo \
        --build_shared_lib \
        --build \
        --build_wheel \
        --update \
        --skip_submodule_sync \
        --skip_tests \
        --arm64 \
        --parallel 4 \
        --cmake_extra_defines ONNXRUNTIME_VERSION=1.9.1 CMAKE_INSTALL_PREFIX=/usr/local && \
    cd build/Linux/RelWithDebInfo && \
    make install

# Copy built binaries into final image
FROM timongentzsch/l4t-ubuntu20-ros2-desktop:latest
COPY --from=0 /usr/local/ /usr/
COPY --from=0 /root/libi2c/libi2c.a /usr/local/lib/
COPY --from=0 /root/libi2c/include/libi2c/ /usr/local/include/
COPY --from=0 /etc/udev/rules.d/99-gpio.rules /etc/udev/rules.d/