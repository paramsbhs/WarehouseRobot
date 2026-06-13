# Gazebo 11 (arm64) with the MathWorks co-simulation plugin.
# Copy GazeboPlugin.zip from your MATLAB installation next to this file
# before building. See docs/gazebo-setup.md.
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Stock Ubuntu 20.04 arm64 has no Gazebo 11; it comes from the OSRF repo
RUN apt-get update && apt-get install -y wget gnupg ca-certificates && \
    echo 'deb http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main' \
        > /etc/apt/sources.list.d/gazebo.list && \
    wget -qO - https://packages.osrfoundation.org/gazebo.key | apt-key add - && \
    apt-get update && apt-get install -y \
    gazebo11 libgazebo11-dev \
    build-essential cmake unzip \
    libprotobuf-dev protobuf-compiler libboost-all-dev \
    xvfb x11vnc libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Build single-threaded: parallel make (-j) exhausts RAM on a 16 GB machine
# compiling the heavy gazebomsghandler sources, and Docker kills a compiler
# process mid-build, surfacing as a misleading "make Error 2".
COPY GazeboPlugin.zip /tmp/
RUN unzip /tmp/GazeboPlugin.zip -d /opt && \
    cd /opt/GazeboPlugin && mkdir build && cd build && \
    cmake .. && make -j1

ENV GAZEBO_PLUGIN_PATH=/opt/GazeboPlugin/export

# MATLAB/Simulink connects to the co-sim plugin on this port
EXPOSE 14581

CMD ["gzserver", "--verbose", "/worlds/warehouse.world"]
