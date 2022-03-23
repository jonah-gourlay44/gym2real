# Copy built binaries into final image
FROM timongentzsch/l4t-ubuntu20-ros2-desktop:latest
ADD onnxruntime/install/lib /usr/local/lib/
ADD onnxruntime/install/include /usr/local/include/
ADD onnxruntime/install/bin /usr/local/bin/