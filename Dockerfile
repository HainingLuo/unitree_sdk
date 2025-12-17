ARG BASE_IMAGE
ARG ARCH
FROM ${BASE_IMAGE}
ENV ROS_DISTRO=jazzy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive

# # nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

SHELL ["/bin/bash", "-c"]

RUN apt update && \
    apt install -y --no-install-recommends \
        python3.12 python3-pip python3-venv \
        curl wget zip unzip tar git cmake make build-essential \
        gnupg2 \
        lsb-release \
        ca-certificates \
        ffmpeg \
    && rm -rf /var/lib/apt/lists/*
    
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN python3 -m pip install --upgrade pip

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
        ros-${ROS_DISTRO}-desktop \
        ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

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
    # cd unitree_ros2/cyclonedds_ws/src && \
    # git clone https://github.com/ros2/rmw_cyclonedds -b jazzy && \
    # git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x && \
    # cd .. && \
    # colcon build --packages-select cyclonedds && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select unitree_go

ARG ARCH
COPY ros_setup/setup_${ARCH}.sh /unitree_ros2/setup.sh

####################################################################################################
####################################### REALSENSE CAMERA ###########################################
####################################################################################################

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

####################################################################################################
######################################### PIP PACKAGES #############################################
####################################################################################################
COPY requirements.txt .
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install -r requirements.txt --default-timeout=1000 --no-cache-dir

####################################################################################################
########################################### FINALISATION ###########################################
####################################################################################################

ENTRYPOINT [ "" ]

WORKDIR /unitree_dev
CMD /bin/bash