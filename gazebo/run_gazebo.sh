#!/usr/bin/env bash
# Build the image (first run) and start Gazebo with the warehouse world.
set -e
cd "$(dirname "$0")"

if [ ! -f GazeboPlugin.zip ]; then
    echo "GazeboPlugin.zip not found in gazebo/."
    echo "Copy it from your MATLAB installation. In MATLAB run:"
    echo "  fullfile(matlabroot,'toolbox','robotics','robotgazebo','gazeboplugin','GazeboPlugin.zip')"
    echo "If that file does not exist, search with:"
    echo "  dir(fullfile(matlabroot,'toolbox','**','GazeboPlugin.zip'))"
    exit 1
fi

if [ ! -f worlds/warehouse.world ]; then
    echo "worlds/warehouse.world not found. In MATLAB run: exportGazeboWorld"
    exit 1
fi

docker build -t warehouse-gazebo .
# 14581: MATLAB/Simulink co-sim.  5900: VNC view (see view_gazebo.sh).
docker run --rm -it -p 14581:14581 -p 5900:5900 \
    -v "$PWD/worlds:/worlds" \
    --name warehouse-gazebo warehouse-gazebo
