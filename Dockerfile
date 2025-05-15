# base OS image
FROM ubuntu:20.04

# set non-interactive environment
ENV DEBIAN_FRONTEND=noninteractive

# install mandatory system packages
RUN apt-get update && apt-get install -y \
    build-essential cmake git wget unzip \
    libgl1-mesa-glx libglfw3 libglfw3-dev libglew-dev \
    libxrandr-dev libxinerama-dev libxcursor-dev \
    libosmesa6-dev libegl1-mesa-dev libglvnd-dev \
    libeigen3-dev pkg-config \
    vim

# install MuJoCo (ver 3.3.3)
ENV MUJOCO_VERSION=3.3.3
WORKDIR /root
RUN git clone --branch v${MUJOCO_VERSION} https://github.com/deepmind/mujoco.git && \
    cd mujoco && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/mujoco && \
    make -j$(nproc) && make install

# set environment variable path
ENV MUJOCO_PATH=/opt/mujoco
ENV LD_LIBRARY_PATH=${MUJOCO_PATH}/lib:$LD_LIBRARY_PATH
ENV CMAKE_PREFIX_PATH=${MUJOCO_PATH}:$CMAKE_PREFIX_PATH

# copy project
WORKDIR /app
COPY . .

# build project
RUN mkdir -p build && cd build && \
    cmake .. -DCMAKE_PREFIX_PATH="/opt/mujoco;/usr/include/eigen3" && \
    make -j$(nproc)

CMD ["./build/mujoco_cpp_template"]
