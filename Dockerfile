# Copy built binaries into final image
FROM timongentzsch/l4t-ubuntu20-ros2-desktop:latest

RUN apt-get update && apt-get install -y build-essential libyamlcpp-dev

# Add onnxruntime install
ADD onnxruntime/install/lib /usr/local/lib/
ADD onnxruntime/install/include /usr/local/include/
ADD onnxruntime/install/bin /usr/local/bin/

# Add JetsonGPIO install
ADD JetsonGPIO/install/lib /usr/local/lib/
ADD JetsonGPIO/install/include /usr/local/include/

# Add libi2d install
ADD libi2c/install/lib /usr/local/lib/
ADD libi2c/install/include /usr/local/include/