FROM nvidia/cuda:11.7.1-runtime-ubuntu20.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive

# 安装基础工具和依赖
RUN apt update && apt install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt clean

ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# 添加 ROS2 的 apt 源
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# 更新并安装 ROS2 Foxy 和 Gazebo
RUN apt update && apt install -y \
    ros-foxy-desktop \
    gazebo11 \
    python3-pip \
    && apt clean

# 安装 Python 库
RUN pip3 install -U pip
RUN pip3 install torch torchvision numpy matplotlib

# 设置工作目录
WORKDIR /workspace

# 拷贝代码
COPY . /workspace

# 设置默认 shell
CMD ["/bin/bash"]
