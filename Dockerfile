FROM jenniferbuehler/ros-indigo-full-catkin 

MAINTAINER Jennifer Buehler

# Install system essentials
RUN apt-get update && apt-get install -y \
    cmake \
    sudo \
    vim \
    && rm -rf /var/lib/apt/lists/*

# need g++ for compiling with cmake even if gcc
# is already installed
RUN apt-get update && apt-get install -y g++ \
    && rm -rf /var/lib/apt/lists/*

# Install required dependencies
RUN apt-get update && apt-get install -y \
    libsoqt4-dev \
    libcoin80-dev \
    libqt4-dev \
    && rm -rf /var/lib/apt/lists/

# Install required ROS dependencies
#RUN apt-get update && apt-get install -y \
#    && rm -rf /var/lib/apt/lists/

COPY inventor_viewer /catkin_ws/src/inventor_viewer
COPY architecture_binding /catkin_ws/src/architecture_binding

# Build
RUN bin/bash -c "source /.bashrc \
    && cd /catkin_ws \
    && catkin_make \
    && catkin_make install"

RUN bin/bash -c "source .bashrc"

CMD ["bash","-l"]
