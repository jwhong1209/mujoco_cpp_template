This is a minimal MuJoCo (ver 3.3.3) C++ template with double pendulum model

# Installation
You have two options for install & build this repository.
1. Set up and install prerequisites & dependencies in you local PC.
2. Use Docker

## To install all the requirements locally by yourself...
### Prerequisities
- OS: **Ubuntu 20.04** (or higher distro should be fine)
  - You may try with **WSL2 in Windows**, but it is not recommended for rendering simulation.
  - this repo would **not be supported for macOS**
- build tools: basic tools for build and manage C++ project are required e.g. git, gcc, cmake, ...
  ```
  $ sudo apt update && sudo apt upgrade -y

  # install build-essential
  $ sudo apt install build-essential
  $ gcc --version # check if gcc is installed correctly

  # install git
  $ sudo apt-get install git
  $ git --version

  # install cmake 
  $ sudo apt install cmake
  $ cmake --version
  ```

- MuJoCo
  ```
  # build & install MuJoCo from source
  $ cd # change path to your home directory
  $ git clone https://github.com/deepmind/mujoco.git
  $ cd ~/mujoco
  $ mkdir install
  $ mkdir build && cd build
  $ cmake ..
  $ cmake --build .
  $ cmake .. -DCMAKE_INSTALL_PREFIX=~/mujoco/install
  $ cmake --install .

  # set environment library path at ~/.bashrc file
  gedit ~/.bashrc

  # Add following lines at the end of the ~/.bashrc file and save
  export MUJOCO_PATH=$HOME/mujoco/install
  export LD_LIBRARY_PATH=$MUJOCO_PATH/lib:$LD_LIBRARY_PATH

  # After that if you reopen a terminal the environment path is updated
  # Or you can type the following command at your current terminal
  $ source ~/.bashrc
  ```

- Dependencies
  - Eigen
  ```
  $ sudo apt install libeigen3-dev # Then, eigen headers are installed at /usr/include/eigen3
  ```
- Editor like **Visual Studio Code (VSCode)** or **Cursor AI** is recommended

## Using Docker
Whether you are programming in Linux (e.g. Ubuntu) or WSL2, you can install all the prerequisites
and dependencies easily using Docker. All you have to do is to install [Docker](https://www.docker.com/)
according to your system. Though, pretty sure that you can not try this repo using Docker in macOS
due to OpenGL rendering. (If you find a solution, please let me know. I would be appreciate for that.)

```
$ cd ~/<your-project-directory>
$ git clone https://github.com/jwhong1209/mujoco_cpp_template.git
$ docker build -t mujoco-cpp-template .
$ docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix mujoco-cpp-template
```

# Build & Run
Once you installed all the requirements whether in Linux or WSL2, now you can build and run this 
examplary repo with following commands in terminal.
```
$ cd ~/<<your-project-directory>>
$ mkdir build && cd build
$ cmake ..
$ make
$ ./mujoco_simulation # or you can change the name of executable in CMakeLists.txt
```

Now you are ready for writing your own program for MuJoCo simulation :)