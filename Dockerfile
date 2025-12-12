FROM nvidia/cuda:13.0.2-base-ubuntu22.04
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive

# # nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

SHELL ["/bin/bash", "-c"]

RUN apt update && \
    apt install -y --no-install-recommends \
        python3.11 python3-pip \
        curl wget zip unzip tar git cmake make build-essential \
        gnupg2 \
        lsb-release \
        ca-certificates \
        ffmpeg \
    && rm -rf /var/lib/apt/lists/*

####################################################################################################
######################################### ROS INSTALLATION #########################################
####################################################################################################

# Set locale
RUN apt update && apt install -y --no-install-recommends locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# Add the ROS 2 apt repository
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository universe && \
    rm -rf /var/lib/apt/lists/*

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# Install development tools and ROS tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-flake8-docstrings \
        python3-pip \
        python3-pytest-cov \
        ros-dev-tools \
        python3-flake8-blind-except \
        python3-flake8-builtins \
        python3-flake8-class-newline \
        python3-flake8-comprehensions \
        python3-flake8-deprecated \
        python3-flake8-import-order \
        python3-flake8-quotes \
        python3-pytest-repeat \
        python3-pytest-rerunfailures \
    && rm -rf /var/lib/apt/lists/*

# Get ROS 2 code
RUN mkdir -p ${ROS_ROOT}/src && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos ${ROS_ROOT}/src

# Install dependencies using rosdep
RUN apt-get update && \
    apt-get upgrade -y

RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths ${ROS_ROOT}/src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"


####################################################################################################
######################################### PIP PACKAGES #############################################
####################################################################################################
COPY requirements.txt .
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install -r requirements.txt --default-timeout=1000 --no-cache-dir

####################################################################################################
##################################### INSTALL UNITREE SDK ##########################################
####################################################################################################
# Unitree SDK2 C++ API
RUN apt-get update && \
	apt-get install -y --no-install-recommends \
	    cmake gcc build-essential libeigen3-dev \
	&& rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git && \
	cd unitree_sdk2/ && mkdir build && cd build && cmake .. && make install

# Unitree SDK2 Python API
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x  && \
    cd cyclonedds && mkdir build install && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_DDSPERF=OFF && \
    cmake --build . --target install
    
RUN git clone https://github.com/unitreerobotics/unitree_sdk2_python.git && \
    cd unitree_sdk2_python && \
    export CYCLONEDDS_HOME="/cyclonedds/install" && \
    python3 -m pip install -e .

####################################################################################################
##################################### INSTALL UNITREE ROS2 #########################################
####################################################################################################
# Dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        ros-${ROS_DISTRO}-ament-lint-auto \
        libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*
    
# Compile unitree_go and unitree_api packages
RUN git clone https://github.com/unitreerobotics/unitree_ros2 && \
    cd unitree_ros2/cyclonedds_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

####################################################################################################
########################################### FINALISATION ###########################################
####################################################################################################

ENTRYPOINT [ "" ]

WORKDIR /unitree_sdk2
CMD /bin/bash